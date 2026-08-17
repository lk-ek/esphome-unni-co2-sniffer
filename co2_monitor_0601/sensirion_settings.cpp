// SPDX-FileCopyrightText: 2026 The esphome-unni-co2-sniffer contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Sensirion-compatible Device Settings service (0x8100).
// Selected UUID topology, settings semantics, and the authenticated/encrypted
// GATT security model are implemented with reference to the Sensirion SHT43
// DemoBoard BLE Firmware (Copyright (c) 2023, Sensirion AG, BSD-3-Clause).
// See THIRD_PARTY_NOTICES.md and LICENSES/.
// Sensirion documents 0x8120 for DIY gadgets; 0x81FE/0x8130 are SHT43
// DemoBoard settings. They remain here for direct GATT experiments, without
// assuming that MyAmbience renders them for a MyCO2 advertisement identity.
//
// Unlike ESPHome's BLECharacteristic wrapper, this service is created through
// the ESP-IDF GATTS API directly so the characteristics can use authenticated
// encrypted (MITM) GATT permissions, matching the security model used by the
// Sensirion SHT43 DemoBoard firmware.
#include "ble_options.h"
#if UNNI_BLE_ENABLED
#include "sensirion_settings.h"

#include "sensirion_ble.h"
#include "esphome/core/log.h"
#include "esphome/core/preferences.h"

#include <algorithm>
#include <array>
#include <cstring>

namespace esphome {
namespace co2_monitor_0601 {
namespace {
static const char *TAG = "sensirion_settings";

static constexpr uint16_t UUID_SERVICE = 0x8100;
static constexpr uint16_t UUID_VERSION = 0x81FF;
static constexpr uint16_t UUID_LOG_ENABLED = 0x81FE;
static constexpr uint16_t UUID_ADVERTISE_DATA_ENABLED = 0x8130;
static constexpr uint16_t UUID_ALTERNATIVE_NAME = 0x8120;
static constexpr uint16_t SERVICE_HANDLE_COUNT = 12;
static constexpr size_t MAX_NAME_LENGTH = 31;
#if UNNI_SHT43_IDENTITY_PROBE
static constexpr uint32_t SETTINGS_PREF_KEY = 0x81006843;
#else
static constexpr uint32_t SETTINGS_PREF_KEY = 0x8100683A;
#endif
static constexpr uint8_t SETTINGS_PREF_VERSION = 1;

// The 128-bit Sensirion UUID family is encoded in the byte order expected by
// the Bluedroid ESP-IDF API. Bytes 12/13 contain the short UUID LSB first.
esp_bt_uuid_t make_uuid(uint16_t short_uuid) {
  esp_bt_uuid_t uuid{};
  uuid.len = ESP_UUID_LEN_128;
  const uint8_t base[ESP_UUID_LEN_128] = {
      0x41, 0xEE, 0x68, 0x3A, 0x99, 0x0F, 0x0E, 0x72,
      0x85, 0x49, 0x8D, 0xB3, 0x00, 0x81, 0x00, 0x00,
  };
  std::memcpy(uuid.uuid.uuid128, base, sizeof(base));
  uuid.uuid.uuid128[12] = static_cast<uint8_t>(short_uuid & 0xFF);
  uuid.uuid.uuid128[13] = static_cast<uint8_t>(short_uuid >> 8);
  return uuid;
}

bool uuid_is(const esp_bt_uuid_t &uuid, uint16_t short_uuid) {
  if (uuid.len != ESP_UUID_LEN_128) return false;
  const esp_bt_uuid_t expected = make_uuid(short_uuid);
  return std::memcmp(uuid.uuid.uuid128, expected.uuid.uuid128, ESP_UUID_LEN_128) == 0;
}

struct PersistedSettings {
  uint8_t version;
  uint8_t log_enabled;
  uint8_t advertise_data_enabled;
  uint8_t reserved;
  char alternative_name[MAX_NAME_LENGTH + 1];
};

struct SettingsGatt {
  bool configured{false};
  bool creation_requested{false};
  bool service_started{false};
  esp32_ble_server::BLEServer *server{nullptr};
  esp_gatt_if_t gatts_if{ESP_GATT_IF_NONE};
  uint16_t service_handle{0};
  uint16_t version_handle{0};
  uint16_t log_enabled_handle{0};
  uint16_t advertise_data_enabled_handle{0};
  uint16_t alternative_name_handle{0};

  uint8_t version{1};
  uint8_t log_enabled{0};
  uint8_t advertise_data_enabled{1};
  std::array<uint8_t, MAX_NAME_LENGTH> alternative_name{};
  size_t alternative_name_len{0};

  ESPPreferenceObject preference{};
  bool preference_ready{false};
} gatt;

void load_settings(const std::string &configured_default_name) {
#if UNNI_SHT43_IDENTITY_PROBE
  const std::string default_name = "SHT43 DB";
#else
  const std::string &default_name = configured_default_name;
#endif
  gatt.version = 1;
  gatt.log_enabled = 0;
  gatt.advertise_data_enabled = 1;
  gatt.alternative_name.fill(0);
  gatt.alternative_name_len = std::min(default_name.size(), MAX_NAME_LENGTH);
  std::memcpy(gatt.alternative_name.data(), default_name.data(), gatt.alternative_name_len);

  if (global_preferences == nullptr) return;
  gatt.preference = global_preferences->make_preference<PersistedSettings>(SETTINGS_PREF_KEY, true);
  gatt.preference_ready = true;

  PersistedSettings saved{};
  if (!gatt.preference.load(&saved) || saved.version != SETTINGS_PREF_VERSION) return;
  gatt.log_enabled = saved.log_enabled ? 1 : 0;
#if UNNI_SHT43_IDENTITY_PROBE
  gatt.advertise_data_enabled = saved.advertise_data_enabled ? 1 : 0;
#else
  // MyAmbience does not expose 0x8130 for the MyCO2/DIY Gadget identity.
  // Never inherit a stale SHT43 privacy value into this identity: live values
  // are the primary data path and must remain advertised.
  gatt.advertise_data_enabled = 1;
#endif
  const size_t len = strnlen(saved.alternative_name, MAX_NAME_LENGTH);
#if !UNNI_SHT43_IDENTITY_PROBE
  // Migrate the historical built-in default to the YAML-configured default.
  // User-chosen AlternativeDeviceName values remain persistent.
  static constexpr const char *LEGACY_DEFAULT_NAME = "MyCO2 Gadget";
  const bool legacy_default = len == std::strlen(LEGACY_DEFAULT_NAME) &&
                              std::memcmp(saved.alternative_name, LEGACY_DEFAULT_NAME, len) == 0;
  if (!legacy_default) {
#endif
    gatt.alternative_name.fill(0);
    gatt.alternative_name_len = len;
    if (len != 0) std::memcpy(gatt.alternative_name.data(), saved.alternative_name, len);
#if !UNNI_SHT43_IDENTITY_PROBE
  } else {
    ESP_LOGI(TAG, "migrated legacy AlternativeDeviceName default to YAML name '%s'", default_name.c_str());
  }
#endif
  ESP_LOGI(TAG, "restored Device Settings: ha_disable=%u advertise_data=%u name_len=%u",
           static_cast<unsigned>(gatt.log_enabled),
           static_cast<unsigned>(gatt.advertise_data_enabled),
           static_cast<unsigned>(gatt.alternative_name_len));
}

void save_settings() {
  if (!gatt.preference_ready) return;
  PersistedSettings saved{};
  saved.version = SETTINGS_PREF_VERSION;
  saved.log_enabled = gatt.log_enabled;
  saved.advertise_data_enabled = gatt.advertise_data_enabled;
  const size_t len = std::min(gatt.alternative_name_len, MAX_NAME_LENGTH);
  if (len != 0) std::memcpy(saved.alternative_name, gatt.alternative_name.data(), len);
  saved.alternative_name[len] = '\0';
  if (!gatt.preference.save(&saved)) {
    ESP_LOGW(TAG, "failed to save Device Settings preference");
    return;
  }
  // These writes are initiated explicitly by the user from MyAmbience and are
  // rare, so commit them immediately rather than risking loss on power removal.
  global_preferences->sync();
}

void set_stack_value(uint16_t handle, const uint8_t *data, size_t len) {
  if (handle == 0) return;
  const esp_err_t err = esp_ble_gatts_set_attr_value(handle, static_cast<uint16_t>(len), data);
  if (err != ESP_OK)
    ESP_LOGW(TAG, "set attr 0x%04X failed: %s", static_cast<unsigned>(handle), esp_err_to_name(err));
}

esp_err_t add_characteristic(uint16_t short_uuid, esp_gatt_char_prop_t properties,
                             esp_gatt_perm_t permissions, uint8_t *value,
                             uint16_t value_len, uint16_t max_len) {
  esp_bt_uuid_t uuid = make_uuid(short_uuid);
  esp_attr_value_t initial{};
  initial.attr_max_len = max_len;
  initial.attr_len = value_len;
  initial.attr_value = value;
  esp_attr_control_t control{};
  control.auto_rsp = ESP_GATT_RSP_BY_APP;
  return esp_ble_gatts_add_char(gatt.service_handle, &uuid, permissions, properties, &initial, &control);
}

void add_next_characteristic(uint16_t just_added) {
  esp_err_t err = ESP_OK;
  if (just_added == 0) {
    err = add_characteristic(UUID_VERSION, ESP_GATT_CHAR_PROP_BIT_READ,
                             ESP_GATT_PERM_READ_ENC_MITM,
                             &gatt.version, 1, 1);
  } else if (just_added == UUID_VERSION) {
    err = add_characteristic(UUID_LOG_ENABLED,
                             static_cast<esp_gatt_char_prop_t>(ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE),
                             static_cast<esp_gatt_perm_t>(ESP_GATT_PERM_READ_ENC_MITM | ESP_GATT_PERM_WRITE_ENC_MITM),
                             &gatt.log_enabled, 1, 1);
  } else if (just_added == UUID_LOG_ENABLED) {
    err = add_characteristic(UUID_ADVERTISE_DATA_ENABLED,
                             static_cast<esp_gatt_char_prop_t>(ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE),
                             static_cast<esp_gatt_perm_t>(ESP_GATT_PERM_READ_ENC_MITM | ESP_GATT_PERM_WRITE_ENC_MITM),
                             &gatt.advertise_data_enabled, 1, 1);
  } else if (just_added == UUID_ADVERTISE_DATA_ENABLED) {
    err = add_characteristic(UUID_ALTERNATIVE_NAME,
                             static_cast<esp_gatt_char_prop_t>(ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE),
                             static_cast<esp_gatt_perm_t>(ESP_GATT_PERM_READ_ENC_MITM | ESP_GATT_PERM_WRITE_ENC_MITM),
                             gatt.alternative_name.data(), static_cast<uint16_t>(gatt.alternative_name_len),
                             MAX_NAME_LENGTH);
  } else if (just_added == UUID_ALTERNATIVE_NAME) {
    err = esp_ble_gatts_start_service(gatt.service_handle);
  }

  if (err != ESP_OK)
    ESP_LOGE(TAG, "Device Settings GATT construction failed after 0x%04X: %s",
             static_cast<unsigned>(just_added), esp_err_to_name(err));
}

uint16_t short_uuid_for_add_event(const esp_bt_uuid_t &uuid) {
  if (uuid_is(uuid, UUID_VERSION)) return UUID_VERSION;
  if (uuid_is(uuid, UUID_LOG_ENABLED)) return UUID_LOG_ENABLED;
  if (uuid_is(uuid, UUID_ADVERTISE_DATA_ENABLED)) return UUID_ADVERTISE_DATA_ENABLED;
  if (uuid_is(uuid, UUID_ALTERNATIVE_NAME)) return UUID_ALTERNATIVE_NAME;
  return 0;
}

bool handle_is_ours(uint16_t handle) {
  return handle != 0 && (handle == gatt.version_handle || handle == gatt.log_enabled_handle ||
                         handle == gatt.advertise_data_enabled_handle || handle == gatt.alternative_name_handle);
}

void send_read_response(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
  if (!param->read.need_rsp) return;
  const uint8_t *data = nullptr;
  size_t len = 0;
  if (param->read.handle == gatt.version_handle) {
    data = &gatt.version;
    len = 1;
  } else if (param->read.handle == gatt.log_enabled_handle) {
    data = &gatt.log_enabled;
    len = 1;
  } else if (param->read.handle == gatt.advertise_data_enabled_handle) {
    data = &gatt.advertise_data_enabled;
    len = 1;
  } else if (param->read.handle == gatt.alternative_name_handle) {
    data = gatt.alternative_name.data();
    len = gatt.alternative_name_len;
  } else {
    return;
  }

  esp_gatt_rsp_t response{};
  response.attr_value.handle = param->read.handle;
  const size_t offset = param->read.is_long ? param->read.offset : 0;
  if (offset > len) {
    esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id,
                                ESP_GATT_INVALID_OFFSET, &response);
    return;
  }
  const size_t remaining = len - offset;
  response.attr_value.offset = static_cast<uint16_t>(offset);
  response.attr_value.len = static_cast<uint16_t>(std::min(remaining, sizeof(response.attr_value.value)));
  if (response.attr_value.len != 0)
    std::memcpy(response.attr_value.value, data + offset, response.attr_value.len);
  response.attr_value.auth_req = ESP_GATT_AUTH_REQ_NONE;
  esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &response);
  ESP_LOGD(TAG, "secure READ handle=0x%04X len=%u conn=%u",
           static_cast<unsigned>(param->read.handle), static_cast<unsigned>(response.attr_value.len),
           static_cast<unsigned>(param->read.conn_id));
}

void send_write_response(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param, esp_gatt_status_t status) {
  if (!param->write.need_rsp) return;
  esp_gatt_rsp_t response{};
  response.attr_value.handle = param->write.handle;
  response.attr_value.offset = param->write.offset;
  response.attr_value.len = status == ESP_GATT_OK ? param->write.len : 0;
  if (response.attr_value.len > sizeof(response.attr_value.value))
    response.attr_value.len = sizeof(response.attr_value.value);
  if (response.attr_value.len != 0)
    std::memcpy(response.attr_value.value, param->write.value, response.attr_value.len);
  response.attr_value.auth_req = ESP_GATT_AUTH_REQ_NONE;
  esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, &response);
}

void handle_write(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
  if (!handle_is_ours(param->write.handle)) return;
  if (param->write.is_prep) {
    send_write_response(gatts_if, param, ESP_GATT_REQ_NOT_SUPPORTED);
    return;
  }

  esp_gatt_status_t status = ESP_GATT_OK;
  if (param->write.handle == gatt.log_enabled_handle) {
    if (param->write.len != 1) {
      status = ESP_GATT_INVALID_ATTR_LEN;
    } else {
      gatt.log_enabled = param->write.value[0] ? 1 : 0;
      set_stack_value(gatt.log_enabled_handle, &gatt.log_enabled, 1);
      save_settings();
      ESP_LOGW(TAG, "MyAmbience IsLogEnabled=%u -> HA/WiFi %s",
               static_cast<unsigned>(gatt.log_enabled),
               gatt.log_enabled ? "DISABLE requested" : "ENABLE requested");
    }
  } else if (param->write.handle == gatt.advertise_data_enabled_handle) {
    if (param->write.len != 1) {
      status = ESP_GATT_INVALID_ATTR_LEN;
    } else {
      sensirion_settings_set_advertise_data_enabled(param->write.value[0] != 0);
      ESP_LOGW(TAG, "MyAmbience IsAdvertiseDataEnabled=%u (applied to manufacturer sample advertising)",
               static_cast<unsigned>(gatt.advertise_data_enabled));
    }
  } else if (param->write.handle == gatt.alternative_name_handle) {
    if (param->write.len > MAX_NAME_LENGTH) {
      status = ESP_GATT_INVALID_ATTR_LEN;
    } else {
      gatt.alternative_name.fill(0);
      gatt.alternative_name_len = param->write.len;
      if (param->write.len != 0)
        std::memcpy(gatt.alternative_name.data(), param->write.value, param->write.len);
      set_stack_value(gatt.alternative_name_handle, gatt.alternative_name.data(), gatt.alternative_name_len);
      save_settings();
      ESP_LOGW(TAG, "MyAmbience AlternativeDeviceName stored (len=%u; GAP identity intentionally unchanged)",
               static_cast<unsigned>(gatt.alternative_name_len));
    }
  } else if (param->write.handle == gatt.version_handle) {
    status = ESP_GATT_WRITE_NOT_PERMIT;
  }

  send_write_response(gatts_if, param, status);
  ESP_LOGD(TAG, "secure WRITE handle=0x%04X len=%u conn=%u status=0x%02X",
           static_cast<unsigned>(param->write.handle), static_cast<unsigned>(param->write.len),
           static_cast<unsigned>(param->write.conn_id), static_cast<unsigned>(status));
}
}  // namespace

bool sensirion_settings_advertise_data_enabled() { return gatt.advertise_data_enabled != 0; }

bool sensirion_settings_ha_disabled() { return gatt.log_enabled != 0; }

void sensirion_settings_set_ha_disabled(bool disabled) {
  const uint8_t value = disabled ? 1 : 0;
  if (gatt.log_enabled == value) {
    set_stack_value(gatt.log_enabled_handle, &gatt.log_enabled, 1);
    return;
  }
  gatt.log_enabled = value;
  set_stack_value(gatt.log_enabled_handle, &gatt.log_enabled, 1);
  save_settings();
  ESP_LOGI(TAG, "IsLogEnabled=%u mapped to HA/WiFi %s",
           static_cast<unsigned>(value), disabled ? "disabled" : "enabled");
}

void sensirion_settings_set_advertise_data_enabled(bool enabled) {
#if !UNNI_SHT43_IDENTITY_PROBE
  if (!enabled) {
    ESP_LOGW(TAG, "ignoring privacy disable for MyCO2 identity; MyAmbience does not expose 0x8130 here");
  }
  enabled = true;
#endif
  const uint8_t value = enabled ? 1 : 0;
  if (gatt.advertise_data_enabled == value) {
    sensirion_ble_set_advertise_data_enabled(enabled);
    return;
  }
  gatt.advertise_data_enabled = value;
  set_stack_value(gatt.advertise_data_enabled_handle, &gatt.advertise_data_enabled, 1);
  save_settings();
  sensirion_ble_set_advertise_data_enabled(enabled);
  ESP_LOGI(TAG, "IsAdvertiseDataEnabled=%u (%s)", static_cast<unsigned>(value),
           enabled ? "live manufacturer data enabled" : "privacy mode");
}

void sensirion_settings_configure_gatt(esp32_ble_server::BLEServer *server, const std::string &default_name) {
  if (gatt.configured || server == nullptr) return;
  // Component setup runs before the Bluedroid application registration has
  // necessarily completed. Arm the service here and create it on the first
  // GATTS event that supplies a valid gatts_if.
  load_settings(default_name);
  sensirion_ble_set_advertise_data_enabled(gatt.advertise_data_enabled != 0);
  gatt.server = server;
  gatt.gatts_if = server->get_gatts_if();
  gatt.configured = true;
  ESP_LOGI(TAG, "secure Sensirion Device Settings armed (0x8100, MITM encrypted access)");
}

static void request_service_creation(esp_gatt_if_t gatts_if) {
  if (!gatt.configured || gatt.creation_requested || gatt.service_handle != 0 ||
      gatts_if == ESP_GATT_IF_NONE || gatts_if == 0)
    return;

  gatt.gatts_if = gatts_if;
  esp_gatt_srvc_id_t service_id{};
  service_id.is_primary = true;
  service_id.id.inst_id = 0;
  service_id.id.uuid = make_uuid(UUID_SERVICE);
  const esp_err_t err = esp_ble_gatts_create_service(gatts_if, &service_id, SERVICE_HANDLE_COUNT);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "failed to request secure Device Settings service: %s", esp_err_to_name(err));
    return;
  }
  gatt.creation_requested = true;
  ESP_LOGI(TAG, "secure Device Settings service creation requested on gatts_if=%u",
           static_cast<unsigned>(gatts_if));
}

void sensirion_settings_loop() {
  if (!gatt.configured || gatt.creation_requested || gatt.server == nullptr) return;
  // Wait until ESPHome has finished constructing and starting all of its own
  // services. This avoids interleaving our raw add-characteristic sequence with
  // BLEServer's asynchronous service-creation queue.
  if (!gatt.server->is_running()) return;
  request_service_creation(gatt.server->get_gatts_if());
}

void sensirion_settings_gatts_event_handler(esp_gatts_cb_event_t event,
                                            esp_gatt_if_t gatts_if,
                                            esp_ble_gatts_cb_param_t *param) {
  if (!gatt.configured || param == nullptr) return;

  switch (event) {
    case ESP_GATTS_CREATE_EVT:
      if (!uuid_is(param->create.service_id.id.uuid, UUID_SERVICE)) break;
      if (param->create.status != ESP_GATT_OK) {
        ESP_LOGE(TAG, "Device Settings service create failed: status=0x%02X",
                 static_cast<unsigned>(param->create.status));
        break;
      }
      gatt.service_handle = param->create.service_handle;
      ESP_LOGI(TAG, "Device Settings service created: handle=0x%04X",
               static_cast<unsigned>(gatt.service_handle));
      add_next_characteristic(0);
      break;

    case ESP_GATTS_ADD_CHAR_EVT: {
      if (param->add_char.service_handle != gatt.service_handle) break;
      const uint16_t short_uuid = short_uuid_for_add_event(param->add_char.char_uuid);
      if (short_uuid == 0) break;
      if (param->add_char.status != ESP_GATT_OK) {
        ESP_LOGE(TAG, "Device Settings char 0x%04X add failed: status=0x%02X",
                 static_cast<unsigned>(short_uuid), static_cast<unsigned>(param->add_char.status));
        break;
      }
      if (short_uuid == UUID_VERSION) gatt.version_handle = param->add_char.attr_handle;
      else if (short_uuid == UUID_LOG_ENABLED) gatt.log_enabled_handle = param->add_char.attr_handle;
      else if (short_uuid == UUID_ADVERTISE_DATA_ENABLED) gatt.advertise_data_enabled_handle = param->add_char.attr_handle;
      else if (short_uuid == UUID_ALTERNATIVE_NAME) gatt.alternative_name_handle = param->add_char.attr_handle;
      ESP_LOGD(TAG, "Device Settings char 0x%04X added: handle=0x%04X",
               static_cast<unsigned>(short_uuid), static_cast<unsigned>(param->add_char.attr_handle));
      add_next_characteristic(short_uuid);
      break;
    }

    case ESP_GATTS_START_EVT:
      if (param->start.service_handle != gatt.service_handle) break;
      if (param->start.status == ESP_GATT_OK) {
        gatt.service_started = true;
        ESP_LOGI(TAG, "secure Device Settings service started: 0x81FF/0x81FE/0x8130/0x8120, MITM required");
      } else {
        ESP_LOGE(TAG, "Device Settings service start failed: status=0x%02X",
                 static_cast<unsigned>(param->start.status));
      }
      break;

    case ESP_GATTS_READ_EVT:
      if (handle_is_ours(param->read.handle)) send_read_response(gatts_if, param);
      break;

    case ESP_GATTS_WRITE_EVT:
      handle_write(gatts_if, param);
      break;

    default:
      break;
  }
}

}  // namespace co2_monitor_0601
}  // namespace esphome
#endif
