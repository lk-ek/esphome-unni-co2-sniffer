// SPDX-FileCopyrightText: 2026 The esphome-unni-co2-sniffer contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// The Gadget/MyAmbience-compatible GATT UUID topology and history-download
// wire format are implemented with reference to Sensirion Gadget BLE 1.5.0
// (BSD-3-Clause). The local RAM/flash persistence layer is project-specific.
// Upstream copyright notice: Copyright (c) 2020, Sensirion AG.
// See THIRD_PARTY_NOTICES.md and LICENSES/.
#include "ble_options.h"
#if UNNI_BLE_HISTORY_ENABLED
#include "sensirion_history.h"
#include "sensirion_ble.h"

#include "esphome/components/esp32_ble_server/ble_2902.h"
#include "esphome/components/esp32_ble_server/ble_characteristic.h"
#include "esphome/components/esp32_ble_server/ble_server.h"
#include "esphome/components/esp32_ble_server/ble_service.h"
#include "esphome/core/log.h"

#include "esp_partition.h"
#include "esp_timer.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>
#include <utility>
#include <vector>

namespace esphome::co2_monitor_0601 {
namespace {

static const char *const TAG = "sensirion_history";

constexpr size_t SAMPLE_SIZE = 8;
constexpr uint16_t RAM_CAPACITY = 4096;
constexpr uint32_t DEFAULT_INTERVAL_MS = 600000;
constexpr uint32_t FLASH_FLUSH_MS = 600000;
constexpr size_t FLASH_SECTOR_SIZE = 4096;
constexpr uint16_t FLASH_DATA_SECTORS = 14;
constexpr uint16_t FLASH_SAMPLES_PER_SECTOR = FLASH_SECTOR_SIZE / SAMPLE_SIZE;
constexpr uint16_t FLASH_CAPACITY = FLASH_DATA_SECTORS * FLASH_SAMPLES_PER_SECTOR;
constexpr uint32_t META_MAGIC = 0x53474832;  // "SGH2"
constexpr uint16_t META_VERSION = 2;
constexpr uint16_t DOWNLOAD_TYPE = 7;

using Sample = std::array<uint8_t, SAMPLE_SIZE>;

struct __attribute__((packed)) FlashMeta {
  uint32_t magic;
  uint16_t version;
  uint16_t size;
  uint32_t generation;
  uint32_t interval_ms;
  uint16_t count;
  uint16_t flash_write_slot;
  uint32_t reserved0;
  uint32_t crc;
  uint32_t reserved1;
};
static_assert(sizeof(FlashMeta) == 32);

struct HistoryState {
  std::array<Sample, RAM_CAPACITY> samples{};
  uint16_t head{0};
  uint16_t count{0};
  uint16_t pending{0};
  uint32_t interval_ms{DEFAULT_INTERVAL_MS};
  uint32_t latest_ms{0};
  uint32_t last_sample_ms{0};
  bool clock_started{false};
};

struct FlashState {
  const esp_partition_t *partition{nullptr};
  bool ready{false};
  uint16_t write_slot{0};
  uint16_t persisted{0};
  uint32_t generation{0};
  uint16_t next_meta_slot{0};
  uint32_t last_flush_ms{0};
};

class HistoryCCCD : public esp32_ble_server::BLE2902 {
 public:
  uint16_t handle() const { return handle_; }
};

struct GattState {
  esp32_ble_server::BLECharacteristic *count{nullptr};
  esp32_ble_server::BLECharacteristic *interval{nullptr};
  esp32_ble_server::BLECharacteristic *requested{nullptr};
  esp32_ble_server::BLECharacteristic *download{nullptr};
  HistoryCCCD *download_cccd{nullptr};
  bool bound{false};
};

enum class DownloadPhase : uint8_t { INACTIVE, HEADER, DATA };
struct DownloadState {
  DownloadPhase phase{DownloadPhase::INACTIVE};
  uint32_t requested{0};
  uint16_t count{0};
  uint16_t sent{0};
  uint16_t sequence{0};
  uint16_t start_logical{0};
  uint32_t age_ms{0};
  uint32_t last_packet_ms{0};
};

HistoryState history;
FlashState flash;
GattState gatt;
DownloadState download;

uint32_t now_ms() {
  return static_cast<uint32_t>(esp_timer_get_time() / 1000ULL);
}

uint32_t get_u32_le(std::span<const uint8_t> x) {
  return static_cast<uint32_t>(x[0]) |
         (static_cast<uint32_t>(x[1]) << 8) |
         (static_cast<uint32_t>(x[2]) << 16) |
         (static_cast<uint32_t>(x[3]) << 24);
}

void put_u16_le(uint8_t *p, uint16_t v) {
  p[0] = static_cast<uint8_t>(v);
  p[1] = static_cast<uint8_t>(v >> 8);
}

void put_u32_le(uint8_t *p, uint32_t v) {
  for (uint8_t i = 0; i < 4; ++i)
    p[i] = static_cast<uint8_t>(v >> (8 * i));
}

uint32_t meta_crc(const FlashMeta &meta) {
  const auto *p = reinterpret_cast<const uint8_t *>(&meta);
  uint32_t hash = 2166136261UL;
  for (size_t i = 0; i < offsetof(FlashMeta, crc); ++i) {
    hash ^= p[i];
    hash *= 16777619UL;
  }
  return hash;
}

bool valid_meta(const FlashMeta &meta) {
  return meta.magic == META_MAGIC && meta.version == META_VERSION &&
         meta.size == sizeof(FlashMeta) && meta.count <= RAM_CAPACITY &&
         meta.flash_write_slot < FLASH_CAPACITY && meta.interval_ms > 0 &&
         meta.crc == meta_crc(meta);
}

size_t sample_offset(uint16_t slot) {
  return FLASH_SECTOR_SIZE + static_cast<size_t>(slot) * SAMPLE_SIZE;
}

Sample logical_sample(uint16_t logical) {
  const uint16_t oldest = static_cast<uint16_t>(
      (history.head + RAM_CAPACITY - history.count) % RAM_CAPACITY);
  return history.samples[(oldest + logical) % RAM_CAPACITY];
}

void sync_gatt() {
  auto u32 = [](uint32_t v) {
    return std::vector<uint8_t>{static_cast<uint8_t>(v), static_cast<uint8_t>(v >> 8),
                                static_cast<uint8_t>(v >> 16), static_cast<uint8_t>(v >> 24)};
  };
  if (gatt.count != nullptr)
    gatt.count->set_value(u32(history.count));
  if (gatt.interval != nullptr)
    gatt.interval->set_value(u32(history.interval_ms));
}

bool write_meta() {
  if (!flash.ready)
    return false;

  constexpr uint16_t RECORDS_PER_SECTOR = FLASH_SECTOR_SIZE / sizeof(FlashMeta);
  if (flash.next_meta_slot >= RECORDS_PER_SECTOR) {
    const esp_err_t err = esp_partition_erase_range(flash.partition, 0, FLASH_SECTOR_SIZE);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "metadata sector erase failed: %d", err);
      return false;
    }
    flash.next_meta_slot = 0;
  }

  FlashMeta meta{};
  meta.magic = META_MAGIC;
  meta.version = META_VERSION;
  meta.size = sizeof(FlashMeta);
  meta.generation = ++flash.generation;
  meta.interval_ms = history.interval_ms;
  meta.count = flash.persisted;
  meta.flash_write_slot = flash.write_slot;
  meta.crc = meta_crc(meta);

  const size_t offset = static_cast<size_t>(flash.next_meta_slot) * sizeof(meta);
  const esp_err_t err = esp_partition_write(flash.partition, offset, &meta, sizeof(meta));
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "metadata write failed: %d", err);
    return false;
  }
  ++flash.next_meta_slot;
  return true;
}

void clear_history() {
  history.head = history.count = history.pending = 0;
  history.latest_ms = history.last_sample_ms = 0;
  history.clock_started = false;
  flash.write_slot = flash.persisted = 0;
  download = {};

  if (flash.ready) {
    const size_t bytes = (1 + FLASH_DATA_SECTORS) * FLASH_SECTOR_SIZE;
    const esp_err_t err = esp_partition_erase_range(flash.partition, 0, bytes);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "history erase failed: %d", err);
    } else {
      flash.generation = 0;
      flash.next_meta_slot = 0;
      write_meta();
    }
  }
  sync_gatt();
}

void set_interval(uint32_t ms) {
  if (ms == 0 || ms == history.interval_ms)
    return;
  ESP_LOGI(TAG, "logging interval changed %u -> %u ms; clearing history",
           static_cast<unsigned>(history.interval_ms), static_cast<unsigned>(ms));
  history.interval_ms = ms;
  clear_history();
}

bool flush_flash() {
  if (!flash.ready || history.pending == 0)
    return true;

  const uint16_t pending = history.pending;
  const uint16_t first_ram = static_cast<uint16_t>(
      (history.head + RAM_CAPACITY - pending) % RAM_CAPACITY);
  uint16_t write_slot = flash.write_slot;
  uint16_t persisted = flash.persisted;

  for (uint16_t i = 0; i < pending; ++i) {
    if ((write_slot % FLASH_SAMPLES_PER_SECTOR) == 0) {
      const size_t sector = 1 + write_slot / FLASH_SAMPLES_PER_SECTOR;
      const esp_err_t err = esp_partition_erase_range(
          flash.partition, sector * FLASH_SECTOR_SIZE, FLASH_SECTOR_SIZE);
      if (err != ESP_OK) {
        ESP_LOGE(TAG, "data sector erase failed: %d", err);
        return false;
      }
    }

    const uint16_t ram_slot = (first_ram + i) % RAM_CAPACITY;
    const esp_err_t err = esp_partition_write(
        flash.partition, sample_offset(write_slot), history.samples[ram_slot].data(), SAMPLE_SIZE);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "sample flash write failed: %d", err);
      return false;
    }
    write_slot = (write_slot + 1) % FLASH_CAPACITY;
    if (persisted < RAM_CAPACITY)
      ++persisted;
  }

  flash.write_slot = write_slot;
  flash.persisted = persisted;
  history.pending = 0;
  flash.last_flush_ms = now_ms();
  if (!write_meta())
    return false;

  ESP_LOGI(TAG, "flushed %u sample(s) to flash; persisted=%u, flash_slot=%u",
           static_cast<unsigned>(pending), static_cast<unsigned>(flash.persisted),
           static_cast<unsigned>(flash.write_slot));
  return true;
}

void init_flash() {
  flash.partition = esp_partition_find_first(
      ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "senshist");
  if (flash.partition == nullptr) {
    ESP_LOGW(TAG, "no 'senshist' partition; history works in RAM only");
    return;
  }

  const size_t required = (1 + FLASH_DATA_SECTORS) * FLASH_SECTOR_SIZE;
  if (flash.partition->size < required) {
    ESP_LOGE(TAG, "senshist partition too small: %u < %u",
             static_cast<unsigned>(flash.partition->size), static_cast<unsigned>(required));
    return;
  }
  flash.ready = true;

  constexpr uint16_t RECORDS_PER_SECTOR = FLASH_SECTOR_SIZE / sizeof(FlashMeta);
  FlashMeta best{};
  bool found = false;
  uint16_t best_slot = 0;
  for (uint16_t slot = 0; slot < RECORDS_PER_SECTOR; ++slot) {
    FlashMeta meta{};
    if (esp_partition_read(flash.partition, slot * sizeof(meta), &meta, sizeof(meta)) != ESP_OK)
      break;
    if (valid_meta(meta) && (!found || meta.generation > best.generation)) {
      best = meta;
      best_slot = slot;
      found = true;
    }
  }

  if (!found) {
    ESP_LOGI(TAG, "initializing empty flash history");
    flash.generation = 0;
    flash.next_meta_slot = 0;
    esp_partition_erase_range(flash.partition, 0, FLASH_SECTOR_SIZE);
    write_meta();
    return;
  }

  history.interval_ms = best.interval_ms;
  flash.write_slot = best.flash_write_slot;
  flash.persisted = best.count;
  flash.generation = best.generation;
  flash.next_meta_slot = best_slot + 1;

  const uint16_t first_flash = static_cast<uint16_t>(
      (best.flash_write_slot + FLASH_CAPACITY - best.count) % FLASH_CAPACITY);
  for (uint16_t i = 0; i < best.count; ++i) {
    const uint16_t slot = (first_flash + i) % FLASH_CAPACITY;
    Sample sample{};
    if (esp_partition_read(flash.partition, sample_offset(slot), sample.data(), SAMPLE_SIZE) != ESP_OK) {
      ESP_LOGW(TAG, "flash history read stopped at %u", i);
      break;
    }
    history.samples[history.head] = sample;
    history.head = (history.head + 1) % RAM_CAPACITY;
    if (history.count < RAM_CAPACITY)
      ++history.count;
  }
  flash.persisted = history.count;
  history.latest_ms = flash.last_flush_ms = now_ms();

  ESP_LOGI(TAG, "restored %u sample(s), interval=%u ms, flash_slot=%u",
           static_cast<unsigned>(history.count), static_cast<unsigned>(history.interval_ms),
           static_cast<unsigned>(flash.write_slot));
}

void commit_sample() {
  const auto &sample = sensirion_ble_sample();
  if (!sample.complete())
    return;

  history.samples[history.head] = sample.encoded();
  history.head = (history.head + 1) % RAM_CAPACITY;
  if (history.count < RAM_CAPACITY)
    ++history.count;
  if (history.pending < RAM_CAPACITY)
    ++history.pending;
  history.latest_ms = now_ms();
  sync_gatt();

  ESP_LOGI(TAG, "history sample %u/%u: %.2f C / %.1f %% / %u ppm",
           static_cast<unsigned>(history.count), static_cast<unsigned>(RAM_CAPACITY),
           sample.temperature_c, sample.humidity_percent, static_cast<unsigned>(sample.co2_ppm));
}

void sampling_tick() {
  const uint32_t now = now_ms();
  if (!history.clock_started) {
    if (sensirion_ble_sample().complete()) {
      history.clock_started = true;
      history.last_sample_ms = now;
      commit_sample();
    }
  } else if (static_cast<uint32_t>(now - history.last_sample_ms) >= history.interval_ms) {
    history.last_sample_ms += history.interval_ms;
    if (static_cast<uint32_t>(now - history.last_sample_ms) >= history.interval_ms)
      history.last_sample_ms = now;  // no stale backfill after a long pause
    commit_sample();
  }

  if (history.pending > 0 &&
      static_cast<uint32_t>(now - flash.last_flush_ms) >= FLASH_FLUSH_MS)
    flush_flash();
}

void start_download() {
  if (gatt.download == nullptr)
    return;

  uint16_t n = history.count;
  if (download.requested > 0 && download.requested < n)
    n = static_cast<uint16_t>(download.requested);

  download.count = n;
  download.sent = 0;
  download.sequence = 0;
  download.start_logical = history.count - n;
  download.age_ms = history.count ? now_ms() - history.latest_ms : 0;
  download.phase = DownloadPhase::HEADER;
  download.last_packet_ms = 0;

  ESP_LOGI(TAG, "history download subscribed: requested=%u available=%u sending=%u age=%u ms",
           static_cast<unsigned>(download.requested), static_cast<unsigned>(history.count),
           static_cast<unsigned>(n), static_cast<unsigned>(download.age_ms));
}

void download_tick() {
  if (download.phase == DownloadPhase::INACTIVE || gatt.download == nullptr)
    return;

  const uint32_t now = now_ms();
  if (download.last_packet_ms != 0 && now - download.last_packet_ms < 4)
    return;
  download.last_packet_ms = now;

  std::vector<uint8_t> packet(20, 0);
  if (download.phase == DownloadPhase::HEADER) {
    put_u16_le(&packet[4], DOWNLOAD_TYPE);
    put_u32_le(&packet[6], history.interval_ms);
    put_u32_le(&packet[10], download.age_ms);
    put_u16_le(&packet[14], download.count);
    gatt.download->set_value(std::move(packet));
    gatt.download->notify();
    download.sequence = 1;
    if (download.count == 0) {
      download.phase = DownloadPhase::INACTIVE;
      ESP_LOGI(TAG, "history download complete (0 samples)");
    } else {
      download.phase = DownloadPhase::DATA;
    }
    return;
  }

  put_u16_le(&packet[0], download.sequence);
  for (uint8_t in_packet = 0; in_packet < 2 && download.sent < download.count; ++in_packet) {
    const Sample sample = logical_sample(download.start_logical + download.sent++);
    memcpy(&packet[2 + in_packet * SAMPLE_SIZE], sample.data(), SAMPLE_SIZE);
  }
  gatt.download->set_value(std::move(packet));
  gatt.download->notify();
  ++download.sequence;

  if (download.sent >= download.count) {
    ESP_LOGI(TAG, "history download complete: %u sample(s), %u data packet(s)",
             static_cast<unsigned>(download.count), static_cast<unsigned>(download.sequence - 1));
    download.phase = DownloadPhase::INACTIVE;
    download.requested = 0;
  }
}

esp32_ble_server::BLECharacteristic *get_or_create_characteristic(
    esp32_ble_server::BLEService *service, const char *uuid, uint32_t properties) {
  const auto id = esp32_ble::ESPBTUUID::from_raw(uuid);
  auto *characteristic = service->get_characteristic(id);
  return characteristic != nullptr ? characteristic : service->create_characteristic(id, properties);
}

void configure_gatt_impl(esp32_ble_server::BLEServer *server) {
  if (gatt.bound || server == nullptr)
    return;

  using esp32_ble::ESPBTUUID;
  using esp32_ble_server::BLECharacteristic;
  const auto service_uuid = ESPBTUUID::from_raw("00008000-B38D-4985-720E-0F993A68EE41");
  auto *service = server->get_service(service_uuid);
  if (service == nullptr)
    service = server->create_service(service_uuid, false, 10);
  if (service == nullptr) {
    ESP_LOGE(TAG, "failed to create 0x8000 history service");
    return;
  }

  gatt.count = get_or_create_characteristic(
      service, "00008002-B38D-4985-720E-0F993A68EE41", BLECharacteristic::PROPERTY_READ);
  gatt.requested = get_or_create_characteristic(
      service, "00008003-B38D-4985-720E-0F993A68EE41", BLECharacteristic::PROPERTY_WRITE);
  gatt.interval = get_or_create_characteristic(
      service, "00008001-B38D-4985-720E-0F993A68EE41",
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  const auto download_uuid = ESPBTUUID::from_raw("00008004-B38D-4985-720E-0F993A68EE41");
  gatt.download = service->get_characteristic(download_uuid);
  if (gatt.download == nullptr) {
    gatt.download = service->create_characteristic(
        download_uuid, BLECharacteristic::PROPERTY_NOTIFY);
    if (gatt.download != nullptr) {
      gatt.download_cccd = new HistoryCCCD();
      gatt.download_cccd->set_value({0x00, 0x00});
      gatt.download->add_descriptor(gatt.download_cccd);
    }
  }

  if (gatt.count == nullptr || gatt.requested == nullptr || gatt.interval == nullptr ||
      gatt.download == nullptr || gatt.download_cccd == nullptr) {
    ESP_LOGE(TAG, "failed to create complete Sensirion GATT topology");
    return;
  }

  gatt.count->set_value({0, 0, 0, 0});
  gatt.requested->set_value({0, 0, 0, 0});
  gatt.download->set_value(std::vector<uint8_t>(20, 0));

  gatt.requested->on_write([](std::span<const uint8_t> x, uint16_t conn_id) {
    if (x.size() < 4)
      return;
    download.requested = get_u32_le(x);
    ESP_LOGI(TAG, "8003 requested samples = %u (conn=%u)",
             static_cast<unsigned>(download.requested), static_cast<unsigned>(conn_id));
  });
  gatt.interval->on_write([](std::span<const uint8_t> x, uint16_t conn_id) {
    if (x.size() < 4)
      return;
    const uint32_t ms = get_u32_le(x);
    ESP_LOGI(TAG, "8001 history interval WRITE = %u ms (conn=%u)",
             static_cast<unsigned>(ms), static_cast<unsigned>(conn_id));
    set_interval(ms);
  });

  sync_gatt();
  server->enqueue_start_service(service);

  const auto settings_uuid = ESPBTUUID::from_raw("00008100-B38D-4985-720E-0F993A68EE41");
  auto *settings = server->get_service(settings_uuid);
  if (settings == nullptr)
    settings = server->create_service(settings_uuid, false, 1);
  if (settings != nullptr)
    server->enqueue_start_service(settings);

  gatt.bound = true;
  ESP_LOGI(TAG, "Sensirion GATT configured in component: %u sample(s), interval=%u ms",
           static_cast<unsigned>(history.count), static_cast<unsigned>(history.interval_ms));
}

}  // namespace

void sensirion_history_setup() {
  init_flash();
  sync_gatt();
  flash.last_flush_ms = now_ms();
}

void sensirion_history_loop() {
  sampling_tick();
  download_tick();
}

void sensirion_history_configure_gatt(esp32_ble_server::BLEServer *server) {
  configure_gatt_impl(server);
}

void sensirion_history_gatts_event_handler(
    esp_gatts_cb_event_t event, esp_gatt_if_t, esp_ble_gatts_cb_param_t *param) {
  if (param == nullptr)
    return;
  if (event == ESP_GATTS_DISCONNECT_EVT) {
    download.phase = DownloadPhase::INACTIVE;
    download.requested = 0;
    return;
  }
  if (event != ESP_GATTS_WRITE_EVT || param->write.is_prep)
    return;

  ESP_LOGD(TAG, "GATT WRITE handle=0x%04X len=%u conn=%u",
           static_cast<unsigned>(param->write.handle), static_cast<unsigned>(param->write.len),
           static_cast<unsigned>(param->write.conn_id));

  if (gatt.download_cccd == nullptr || param->write.handle != gatt.download_cccd->handle() ||
      param->write.len != 2)
    return;

  const uint8_t lo = param->write.value[0];
  const uint8_t hi = param->write.value[1];
  ESP_LOGI(TAG, "8004 CCCD WRITE = %02X %02X (conn=%u, handle=0x%04X)", lo, hi,
           static_cast<unsigned>(param->write.conn_id), static_cast<unsigned>(param->write.handle));

  if (lo == 0x01 && hi == 0x00) {
    start_download();
  } else if (lo == 0x00 && hi == 0x00) {
    download.phase = DownloadPhase::INACTIVE;
    ESP_LOGI(TAG, "history download unsubscribed");
  }
}

}  // namespace esphome::co2_monitor_0601
#endif  // UNNI_BLE_HISTORY_ENABLED
