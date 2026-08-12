// SPDX-License-Identifier: GPL-3.0-or-later
#include "ble_options.h"
#if UNNI_BLE_ENABLED
#include "sensirion_ble.h"

#include "esphome/components/esp32_ble/ble.h"
#include "esphome/core/log.h"

#include "esp_mac.h"

#include <array>

namespace esphome {
namespace bus_sniffer {

static const char *TAG = "sensirion_ble";

// Keep the identity used by the working MyAmbience test build.
static constexpr uint8_t TEST_BT_MAC[6] = {0x82, 0xF1, 0xB2, 0x61, 0x68, 0x3A};
static esp_err_t bt_mac_set_result = ESP_FAIL;

__attribute__((constructor)) static void set_bt_identity_early() {
  bt_mac_set_result = esp_iface_mac_addr_set(TEST_BT_MAC, ESP_MAC_BT);
}

static constexpr uint8_t COMPANY_ID_LO = 0xD5;
static constexpr uint8_t COMPANY_ID_HI = 0x06;
static constexpr uint8_t ADV_TYPE_SAMPLE = 0x00;
static constexpr uint8_t SAMPLE_TYPE_T_RH_CO2_ALT = 0x08;

static SensirionSample sample;
static uint16_t device_id = 0;
static bool device_id_ready = false;
static uint32_t advertising_interval_ms = 5000;

// Complete legacy advertising packet:
// flags (3) + manufacturer data field (20) + complete name "S" (3).
static std::array<uint8_t, 26> advertisement{};
static bool advertisement_ready = false;
static uint32_t payload_version = 0;
static uint32_t configured_version = 0;
static bool gatt_connected = false;

enum class AdvState : uint8_t { IDLE, CONFIGURING, STARTING, ADVERTISING, STOPPING };
static AdvState adv_state = AdvState::IDLE;
static esp_ble_adv_params_t adv_params{};

static void configure_advertisement();

static void request_refresh() {
  if (!advertisement_ready || esp32_ble::global_ble == nullptr ||
      !esp32_ble::global_ble->is_active())
    return;

  // Connectable advertising stops automatically on connect. Never race an
  // active GATT/history transfer with stop/configure/start operations.
  if (gatt_connected)
    return;

  if (adv_state == AdvState::ADVERTISING) {
    adv_state = AdvState::STOPPING;
    if (esp_ble_gap_stop_advertising() != ESP_OK) {
      adv_state = AdvState::IDLE;
      configure_advertisement();
    }
  } else if (adv_state == AdvState::IDLE) {
    configure_advertisement();
  }
}

static void configure_advertisement() {
  if (!advertisement_ready || gatt_connected)
    return;

  adv_state = AdvState::CONFIGURING;
  configured_version = payload_version;
  const esp_err_t err = esp_ble_gap_config_adv_data_raw(advertisement.data(), advertisement.size());
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "adv data setup failed: %s", esp_err_to_name(err));
    adv_state = AdvState::IDLE;
  }
}

void sensirion_ble_set_advertising_interval(uint32_t interval_ms) {
  if (interval_ms < 20) interval_ms = 20;
  if (interval_ms > 10240) interval_ms = 10240;

  advertising_interval_ms = interval_ms;
  const uint16_t units = static_cast<uint16_t>((interval_ms * 1000ULL + 624) / 625);
  adv_params.adv_int_min = units;
  adv_params.adv_int_max = units;
}

void sensirion_ble_gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
  switch (event) {
    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
      if (adv_state != AdvState::CONFIGURING)
        break;
      if (param->adv_data_raw_cmpl.status != ESP_BT_STATUS_SUCCESS) {
        ESP_LOGW(TAG, "adv data setup failed: %d", param->adv_data_raw_cmpl.status);
        adv_state = AdvState::IDLE;
        break;
      }
      if (gatt_connected) {
        adv_state = AdvState::IDLE;
        break;
      }
      adv_state = AdvState::STARTING;
      if (esp_ble_gap_start_advertising(&adv_params) != ESP_OK)
        adv_state = AdvState::IDLE;
      break;

    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
      if (adv_state == AdvState::STARTING) {
        if (param->adv_start_cmpl.status == ESP_BT_STATUS_SUCCESS) {
          adv_state = AdvState::ADVERTISING;
          if (configured_version != payload_version)
            request_refresh();
        } else {
          adv_state = AdvState::IDLE;
        }
      } else if (!gatt_connected && advertisement_ready &&
                 param->adv_start_cmpl.status == ESP_BT_STATUS_SUCCESS) {
        // ESPHome may restart its fast default advertiser after disconnect.
        // Replace it with our current packet and low-duty-cycle interval.
        adv_state = AdvState::STOPPING;
        if (esp_ble_gap_stop_advertising() != ESP_OK) {
          adv_state = AdvState::IDLE;
          configure_advertisement();
        }
      }
      break;

    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
      if (adv_state == AdvState::STOPPING) {
        adv_state = AdvState::IDLE;
        if (!gatt_connected)
          configure_advertisement();
      }
      break;

    default:
      break;
  }
}

void sensirion_ble_gatts_event_handler(esp_gatts_cb_event_t event,
                                       esp_ble_gatts_cb_param_t *param) {
  if (event == ESP_GATTS_CONNECT_EVT) {
    gatt_connected = true;
    adv_state = AdvState::IDLE;
    ESP_LOGI(TAG, "GATT connected (conn=%u)",
             param ? static_cast<unsigned>(param->connect.conn_id) : 0U);
  } else if (event == ESP_GATTS_DISCONNECT_EVT) {
    gatt_connected = false;
    adv_state = AdvState::IDLE;
    if (advertisement_ready)
      configure_advertisement();
  }
}

uint16_t sensirion_ble_get_device_id() {
  if (device_id_ready)
    return device_id;

  uint8_t mac[6]{};
  if (esp_read_mac(mac, ESP_MAC_BT) == ESP_OK)
    device_id = (static_cast<uint16_t>(mac[4]) << 8) | mac[5];
  if (device_id == 0)
    device_id = 0xC301;

  device_id_ready = true;
  return device_id;
}

static void build_advertisement() {
  if (!sample.complete())
    return;

  const auto encoded = sample.encoded();
  const uint16_t id = sensirion_ble_get_device_id();

  size_t p = 0;
  advertisement[p++] = 0x02;
  advertisement[p++] = ESP_BLE_AD_TYPE_FLAG;
  advertisement[p++] = ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT;

  advertisement[p++] = 19;  // type byte + 18 bytes manufacturer data
  advertisement[p++] = ESP_BLE_AD_MANUFACTURER_SPECIFIC_TYPE;
  advertisement[p++] = COMPANY_ID_LO;
  advertisement[p++] = COMPANY_ID_HI;
  advertisement[p++] = ADV_TYPE_SAMPLE;
  advertisement[p++] = SAMPLE_TYPE_T_RH_CO2_ALT;
  advertisement[p++] = static_cast<uint8_t>(id >> 8);
  advertisement[p++] = static_cast<uint8_t>(id & 0xFF);
  for (uint8_t byte : encoded)
    advertisement[p++] = byte;
  // The legacy Gadget library advertises a 12-byte sample backing store.
  for (int i = 0; i < 4; i++)
    advertisement[p++] = 0;

  advertisement[p++] = 0x02;
  advertisement[p++] = ESP_BLE_AD_TYPE_NAME_CMPL;
  advertisement[p++] = 'S';

  advertisement_ready = true;
  ++payload_version;
  request_refresh();

  ESP_LOGD(TAG, "T_RH_CO2_ALT: %.2f C / %.1f %% / %u ppm, device 0x%04X",
           sample.temperature_c, sample.humidity_percent,
           static_cast<unsigned>(sample.co2_ppm), static_cast<unsigned>(id));
}

void sensirion_ble_set_temperature_humidity(float temperature_c, float humidity_percent) {
  sample.temperature_c = temperature_c;
  sample.humidity_percent = humidity_percent;
  sample.have_temperature = true;
  sample.have_humidity = true;
}

void sensirion_ble_set_co2(uint16_t ppm) {
  sample.co2_ppm = ppm;
  sample.have_co2 = true;
}

void sensirion_ble_commit_live_advertisement() {
  build_advertisement();
}

const SensirionSample &sensirion_ble_sample() {
  return sample;
}

void sensirion_ble_setup() {
  adv_params = {
      .adv_int_min = 0,
      .adv_int_max = 0,
      .adv_type = ADV_TYPE_IND,
      .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
      .peer_addr = {0, 0, 0, 0, 0, 0},
      .peer_addr_type = BLE_ADDR_TYPE_PUBLIC,
      .channel_map = ADV_CHNL_ALL,
      .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
  };
  sensirion_ble_set_advertising_interval(advertising_interval_ms);

  uint8_t mac[6]{};
  const esp_err_t read_result = esp_read_mac(mac, ESP_MAC_BT);
  if (bt_mac_set_result == ESP_OK && read_result == ESP_OK) {
    ESP_LOGI(TAG, "BT MAC %02X:%02X:%02X:%02X:%02X:%02X, device 0x%04X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
             static_cast<unsigned>(sensirion_ble_get_device_id()));
  } else {
    ESP_LOGW(TAG, "BT identity setup/read failed: set=%d read=%d",
             static_cast<int>(bt_mac_set_result), static_cast<int>(read_result));
  }
  ESP_LOGI(TAG, "advertising interval: %u ms", static_cast<unsigned>(advertising_interval_ms));
}

}  // namespace bus_sniffer
}  // namespace esphome
#endif
