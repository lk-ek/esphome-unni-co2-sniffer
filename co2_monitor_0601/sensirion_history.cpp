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
#include "sensirion_history_format.h"
#include "sensirion_history_guard.h"

#include "esphome/components/esp32_ble_server/ble_2902.h"
#include "esphome/components/esp32_ble_server/ble_characteristic.h"
#include "esphome/components/esp32_ble_server/ble_server.h"
#include "esphome/components/esp32_ble_server/ble_service.h"
#include "esphome/core/log.h"
#include "esphome/components/time/real_time_clock.h"

#include "esp_partition.h"
#include "esp_timer.h"

#include <algorithm>
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
constexpr uint16_t HISTORY_CAPACITY = 4096;
constexpr uint16_t PENDING_CAPACITY = 64;
constexpr uint32_t DEFAULT_INTERVAL_MS = 600000;
constexpr uint32_t FLASH_FLUSH_MS = 600000;
constexpr uint32_t META_RETRY_MS = 1000;
constexpr size_t FLASH_SECTOR_SIZE = 4096;
constexpr uint16_t FLASH_DATA_SECTORS = 14;
constexpr uint16_t FLASH_PRIMARY_META_SECTOR = 0;
constexpr uint16_t FLASH_SECONDARY_META_SECTOR = FLASH_DATA_SECTORS + 1;
constexpr uint16_t FLASH_TOTAL_SECTORS = FLASH_DATA_SECTORS + 2;
constexpr uint16_t FLASH_SAMPLES_PER_SECTOR = FLASH_SECTOR_SIZE / SAMPLE_SIZE;
constexpr uint16_t FLASH_CAPACITY = FLASH_DATA_SECTORS * FLASH_SAMPLES_PER_SECTOR;
static_assert(HISTORY_CAPACITY <= FLASH_CAPACITY);
constexpr uint32_t META_MAGIC = 0x53474832;  // "SGH2"
constexpr uint16_t META_VERSION_V2 = 2;
constexpr uint16_t META_VERSION_V3 = 3;
constexpr uint16_t META_VERSION = 4;
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
  // Persisted history lives in the dedicated senshist flash partition. Keep
  // only the newest, not-yet-flushed samples in RAM; a full 4096-sample RAM
  // mirror costs 32 KiB on the ESP32-C3 and is unnecessary.
  std::array<Sample, PENDING_CAPACITY> pending_samples{};
  uint16_t pending_head{0};
  uint16_t count{0};
  uint16_t pending{0};
  uint32_t interval_ms{DEFAULT_INTERVAL_MS};
  uint32_t latest_ms{0};
  uint32_t last_sample_ms{0};
  uint32_t run_anchor_epoch_s{0};
  uint32_t last_sample_epoch_s{0};
  uint16_t run_count{0};
  bool clock_started{false};
  bool run_relative_age_valid{false};
  bool force_new_run_on_next_sample{false};
};

struct FlashState {
  const esp_partition_t *partition{nullptr};
  bool ready{false};
  uint16_t write_slot{0};
  uint16_t persisted{0};
  uint32_t generation{0};
  uint16_t meta_sector{FLASH_PRIMARY_META_SECTOR};
  uint16_t next_meta_slot{0};
  uint32_t last_flush_ms{0};
  uint32_t last_meta_attempt_ms{0};
  bool metadata_dirty{false};
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
  bool connected{false};
  bool download_notify_enabled{false};
};

enum class ClearPhase : uint8_t { INACTIVE, PREPARE_JOURNAL, ERASE_SECTORS };
struct ClearState {
  ClearPhase phase{ClearPhase::INACTIVE};
  uint16_t preserved_meta_sector{FLASH_PRIMARY_META_SECTOR};
  uint16_t next_sector{0};
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
  uint32_t started_ms{0};
  uint32_t last_progress_ms{0};
  uint32_t guard_pause_started_ms{0};
  uint32_t guard_paused_ms{0};
  uint16_t guard_pauses{0};
  bool guard_paused{false};
};

HistoryState history;
FlashState flash;
GattState gatt;
DownloadState download;
ClearState clearing;
sensirion_history_guard::Guard capture_guard;
uint32_t last_sampling_poll_ms = 0;
uint16_t last_synced_downloadable_count = UINT16_MAX;
time::RealTimeClock *wall_clock = nullptr;

uint64_t now_us() {
  return static_cast<uint64_t>(esp_timer_get_time());
}

uint32_t now_ms() {
  return static_cast<uint32_t>(now_us() / 1000ULL);
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

uint32_t fnv1a_update(uint32_t hash, const uint8_t *p, size_t size) {
  for (size_t i = 0; i < size; ++i) {
    hash ^= p[i];
    hash *= 16777619UL;
  }
  return hash;
}

uint32_t meta_crc_legacy(const FlashMeta &meta) {
  const auto *p = reinterpret_cast<const uint8_t *>(&meta);
  return fnv1a_update(2166136261UL, p, offsetof(FlashMeta, crc));
}

uint32_t meta_crc_v4(const FlashMeta &meta) {
  const auto *p = reinterpret_cast<const uint8_t *>(&meta);
  uint32_t hash = fnv1a_update(2166136261UL, p, offsetof(FlashMeta, crc));
  return fnv1a_update(hash, reinterpret_cast<const uint8_t *>(&meta.reserved1),
                      sizeof(meta.reserved1));
}

uint32_t meta_crc(const FlashMeta &meta) {
  return meta.version >= META_VERSION ? meta_crc_v4(meta) : meta_crc_legacy(meta);
}

bool valid_meta(const FlashMeta &meta) {
  if (meta.magic != META_MAGIC ||
      (meta.version != META_VERSION_V2 && meta.version != META_VERSION_V3 &&
       meta.version != META_VERSION) ||
      meta.size != sizeof(FlashMeta) || meta.count > HISTORY_CAPACITY ||
      meta.flash_write_slot >= FLASH_CAPACITY || meta.interval_ms == 0 ||
      meta.crc != meta_crc(meta))
    return false;
  if (meta.version == META_VERSION && meta.reserved1 > meta.count)
    return false;
  return true;
}

size_t meta_offset(uint16_t sector, uint16_t slot) {
  return static_cast<size_t>(sector) * FLASH_SECTOR_SIZE +
         static_cast<size_t>(slot) * sizeof(FlashMeta);
}

size_t sample_offset(uint16_t slot) {
  return FLASH_SECTOR_SIZE + static_cast<size_t>(slot) * SAMPLE_SIZE;
}

bool logical_sample(uint16_t logical, Sample &sample) {
  sample.fill(0);
  if (logical >= history.count)
    return false;

  // history.count is capped at HISTORY_CAPACITY. Pending samples are always the
  // newest entries. If the logical window is full, only the newest
  // (count-pending) persisted samples remain visible.
  const uint16_t visible_persisted = static_cast<uint16_t>(history.count - history.pending);
  if (logical < visible_persisted) {
    if (!flash.ready || flash.persisted == 0)
      return false;
    const uint16_t persisted_skip = static_cast<uint16_t>(flash.persisted - visible_persisted);
    const uint16_t oldest_flash = static_cast<uint16_t>(
        (flash.write_slot + FLASH_CAPACITY - flash.persisted) % FLASH_CAPACITY);
    const uint16_t slot = static_cast<uint16_t>(
        (oldest_flash + persisted_skip + logical) % FLASH_CAPACITY);
    if (esp_partition_read(flash.partition, sample_offset(slot), sample.data(), SAMPLE_SIZE) != ESP_OK) {
      ESP_LOGW(TAG, "history sample flash read failed at slot %u", static_cast<unsigned>(slot));
      return false;
    }
    return true;
  }

  const uint16_t pending_logical = static_cast<uint16_t>(logical - visible_persisted);
  const uint16_t oldest_pending = static_cast<uint16_t>(
      (history.pending_head + PENDING_CAPACITY - history.pending) % PENDING_CAPACITY);
  sample = history.pending_samples[(oldest_pending + pending_logical) % PENDING_CAPACITY];
  return true;
}

uint32_t wall_clock_epoch_s() {
  if (wall_clock == nullptr) return 0;
  const auto utc = wall_clock->utcnow();
  if (!utc.is_valid()) return 0;
  const auto epoch = static_cast<uint64_t>(utc.timestamp);
  if (epoch > UINT32_MAX) return 0;
  const uint32_t epoch_s = static_cast<uint32_t>(epoch);
  return sensirion_history_format::wall_clock_valid(epoch_s) ? epoch_s : 0;
}

bool restored_run_age_available() {
  if (history.run_count == 0 || history.run_anchor_epoch_s == 0) return false;
  const uint32_t now_epoch_s = wall_clock_epoch_s();
  if (now_epoch_s == 0) return false;
  const uint32_t latest_epoch_s = sensirion_history_format::run_latest_epoch_s(
      history.run_anchor_epoch_s, history.run_count, history.interval_ms);
  return latest_epoch_s != 0 && now_epoch_s >= latest_epoch_s;
}

uint16_t downloadable_count() {
  if (history.run_count == 0) return 0;
  if (history.run_relative_age_valid || restored_run_age_available()) return history.run_count;
  return 0;
}

void sync_gatt() {
  auto u32 = [](uint32_t v) {
    return std::vector<uint8_t>{static_cast<uint8_t>(v), static_cast<uint8_t>(v >> 8),
                                static_cast<uint8_t>(v >> 16), static_cast<uint8_t>(v >> 24)};
  };
  const uint16_t visible_count = downloadable_count();
  if (gatt.count != nullptr)
    gatt.count->set_value(u32(visible_count));
  last_synced_downloadable_count = visible_count;
  if (gatt.interval != nullptr)
    gatt.interval->set_value(u32(history.interval_ms));
}

bool write_meta() {
  if (!flash.ready)
    return false;

  constexpr uint16_t RECORDS_PER_SECTOR = FLASH_SECTOR_SIZE / sizeof(FlashMeta);
  if (flash.next_meta_slot >= RECORDS_PER_SECTOR) {
    const uint16_t next_sector = flash.meta_sector == FLASH_PRIMARY_META_SECTOR
                                     ? FLASH_SECONDARY_META_SECTOR
                                     : FLASH_PRIMARY_META_SECTOR;
    const esp_err_t err = esp_partition_erase_range(
        flash.partition, static_cast<size_t>(next_sector) * FLASH_SECTOR_SIZE,
        FLASH_SECTOR_SIZE);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "metadata sector erase failed: %d", err);
      return false;
    }
    flash.meta_sector = next_sector;
    flash.next_meta_slot = 0;
  }

  FlashMeta meta{};
  meta.magic = META_MAGIC;
  meta.version = META_VERSION;
  meta.size = sizeof(FlashMeta);
  meta.generation = flash.generation + 1;
  meta.interval_ms = history.interval_ms;
  meta.count = flash.persisted;
  meta.flash_write_slot = flash.write_slot;
  meta.reserved0 = history.run_anchor_epoch_s;
  meta.reserved1 = history.run_count;
  meta.crc = meta_crc(meta);

  const size_t offset = meta_offset(flash.meta_sector, flash.next_meta_slot);
  flash.last_meta_attempt_ms = now_ms();
  const esp_err_t err = esp_partition_write(flash.partition, offset, &meta, sizeof(meta));
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "metadata write failed: %d", err);
    return false;
  }
  flash.generation = meta.generation;
  ++flash.next_meta_slot;
  flash.metadata_dirty = false;
  return true;
}

void reset_history_in_memory() {
  history.pending_head = history.count = history.pending = 0;
  history.latest_ms = history.last_sample_ms = 0;
  history.run_anchor_epoch_s = history.last_sample_epoch_s = 0;
  history.run_count = 0;
  history.clock_started = false;
  history.run_relative_age_valid = false;
  history.force_new_run_on_next_sample = false;
  flash.write_slot = flash.persisted = 0;
  download = {};
}

void request_clear() {
  reset_history_in_memory();
  if (!flash.ready) {
    clearing = {};
    sync_gatt();
    return;
  }
  clearing.phase = ClearPhase::PREPARE_JOURNAL;
  clearing.preserved_meta_sector = flash.meta_sector == FLASH_PRIMARY_META_SECTOR
                                       ? FLASH_SECONDARY_META_SECTOR
                                       : FLASH_PRIMARY_META_SECTOR;
  clearing.next_sector = 0;
  sync_gatt();
}

void set_interval(uint32_t ms) {
  if (!sensirion_history_format::interval_valid(ms)) {
    ESP_LOGW(TAG, "rejected unsafe history interval %u ms (allowed %u..%u ms)",
             static_cast<unsigned>(ms),
             static_cast<unsigned>(sensirion_history_format::MIN_INTERVAL_MS),
             static_cast<unsigned>(sensirion_history_format::MAX_INTERVAL_MS));
    sync_gatt();
    return;
  }
  if (ms == history.interval_ms)
    return;
  ESP_LOGI(TAG, "logging interval changed %u -> %u ms; clearing history",
           static_cast<unsigned>(history.interval_ms), static_cast<unsigned>(ms));
  history.interval_ms = ms;
  request_clear();
}

void clear_tick() {
  if (clearing.phase == ClearPhase::INACTIVE || !flash.ready) return;

  if (clearing.phase == ClearPhase::PREPARE_JOURNAL) {
    const size_t offset = static_cast<size_t>(clearing.preserved_meta_sector) * FLASH_SECTOR_SIZE;
    const esp_err_t erase_err = esp_partition_erase_range(flash.partition, offset, FLASH_SECTOR_SIZE);
    if (erase_err != ESP_OK) {
      ESP_LOGE(TAG, "history clear journal erase failed: %d", erase_err);
      return;
    }
    flash.meta_sector = clearing.preserved_meta_sector;
    flash.next_meta_slot = 0;
    flash.metadata_dirty = true;
    if (!write_meta()) return;
    clearing.phase = ClearPhase::ERASE_SECTORS;
    clearing.next_sector = 0;
    return;
  }

  while (clearing.next_sector < FLASH_TOTAL_SECTORS &&
         clearing.next_sector == clearing.preserved_meta_sector)
    ++clearing.next_sector;
  if (clearing.next_sector >= FLASH_TOTAL_SECTORS) {
    clearing = {};
    flash.last_flush_ms = now_ms();
    ESP_LOGI(TAG, "history clear complete; interval=%u ms", static_cast<unsigned>(history.interval_ms));
    return;
  }

  const uint16_t sector = clearing.next_sector;
  const esp_err_t err = esp_partition_erase_range(
      flash.partition, static_cast<size_t>(sector) * FLASH_SECTOR_SIZE, FLASH_SECTOR_SIZE);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "history clear sector %u erase failed: %d", static_cast<unsigned>(sector), err);
    return;
  }
  ++clearing.next_sector;
}

bool flush_flash() {
  if (!flash.ready)
    return history.pending == 0 && !flash.metadata_dirty;
  if (clearing.phase != ClearPhase::INACTIVE) return false;
  if (history.pending == 0) {
    if (!flash.metadata_dirty) return true;
    return write_meta();
  }

  const uint16_t pending = history.pending;
  const uint16_t first_ram = static_cast<uint16_t>(
      (history.pending_head + PENDING_CAPACITY - pending) % PENDING_CAPACITY);
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

    const uint16_t ram_slot = (first_ram + i) % PENDING_CAPACITY;
    const esp_err_t err = esp_partition_write(
        flash.partition, sample_offset(write_slot), history.pending_samples[ram_slot].data(), SAMPLE_SIZE);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "sample flash write failed: %d", err);
      return false;
    }
    write_slot = (write_slot + 1) % FLASH_CAPACITY;
    if (persisted < HISTORY_CAPACITY)
      ++persisted;
  }

  flash.write_slot = write_slot;
  flash.persisted = persisted;
  flash.metadata_dirty = true;
  history.pending = 0;
  history.pending_head = 0;
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
    ESP_LOGE(TAG, "no 'senshist' partition; persistent 4096-sample history unavailable");
    return;
  }

  const size_t required = FLASH_TOTAL_SECTORS * FLASH_SECTOR_SIZE;
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
  uint16_t best_sector = FLASH_PRIMARY_META_SECTOR;
  uint16_t occupied_after_best_sector = 0;
  for (const uint16_t sector : {FLASH_PRIMARY_META_SECTOR, FLASH_SECONDARY_META_SECTOR}) {
    uint16_t occupied = 0;
    for (uint16_t slot = 0; slot < RECORDS_PER_SECTOR; ++slot) {
      FlashMeta meta{};
      if (esp_partition_read(flash.partition, meta_offset(sector, slot), &meta, sizeof(meta)) != ESP_OK)
        break;
      const auto *bytes = reinterpret_cast<const uint8_t *>(&meta);
      bool erased = true;
      for (size_t i = 0; i < sizeof(meta); ++i) erased &= bytes[i] == 0xFF;
      if (!erased) occupied = slot + 1;
      if (valid_meta(meta) &&
          (!found || sensirion_history_format::generation_newer(meta.generation, best.generation))) {
        best = meta;
        best_slot = slot;
        best_sector = sector;
        occupied_after_best_sector = occupied;
        found = true;
      }
    }
    if (found && best_sector == sector) occupied_after_best_sector = occupied;
  }

  if (!found) {
    ESP_LOGI(TAG, "initializing empty flash history");
    flash.generation = 0;
    flash.meta_sector = FLASH_PRIMARY_META_SECTOR;
    flash.next_meta_slot = 0;
    const esp_err_t erase_err = esp_partition_erase_range(flash.partition, 0, FLASH_SECTOR_SIZE);
    if (erase_err != ESP_OK) {
      ESP_LOGE(TAG, "initial metadata sector erase failed: %d", erase_err);
      flash.ready = false;
      return;
    }
    flash.metadata_dirty = true;
    write_meta();
    return;
  }

  history.interval_ms = best.interval_ms;
  flash.write_slot = best.flash_write_slot;
  flash.persisted = best.count;
  flash.generation = best.generation;
  flash.meta_sector = best_sector;
  flash.next_meta_slot = std::max<uint16_t>(best_slot + 1, occupied_after_best_sector);
  flash.metadata_dirty = best.version != META_VERSION;

  history.count = best.count;
  history.pending = 0;
  history.pending_head = 0;
  flash.persisted = best.count;
  history.latest_ms = flash.last_flush_ms = now_ms();
  if (best.version == META_VERSION) {
    history.run_anchor_epoch_s = best.reserved0;
    history.run_count = static_cast<uint16_t>(std::min<uint32_t>(best.reserved1, best.count));
  } else {
    // V2/V3 did not persist gap/run timing. Keep the samples, but do not expose
    // them as a falsely continuous MyAmbience run after migration.
    history.run_anchor_epoch_s = 0;
    history.run_count = 0;
  }
  history.run_relative_age_valid = false;
  history.force_new_run_on_next_sample = history.count != 0;

  ESP_LOGI(TAG, "restored %u sample(s), latest continuous run=%u, interval=%u ms, flash_slot=%u",
           static_cast<unsigned>(history.count), static_cast<unsigned>(history.run_count),
           static_cast<unsigned>(history.interval_ms), static_cast<unsigned>(flash.write_slot));
  if (flash.metadata_dirty)
    ESP_LOGI(TAG, "legacy history metadata v%u will migrate to sparse-time version 4 on next loop flush",
             static_cast<unsigned>(best.version));
}

void commit_sample(bool new_run) {
  const auto &sample = sensirion_ble_sample();
  if (!sample.complete())
    return;

  // Never overwrite an unflushed sample. At unusually short history
  // intervals, flush the small pending ring early instead of growing RAM use.
  if (history.pending >= PENDING_CAPACITY && !flush_flash()) {
    ESP_LOGW(TAG, "history pending ring full; dropping newest sample because flash flush failed");
    return;
  }
  const uint32_t sample_now_ms = now_ms();
  const uint32_t sample_epoch_s = wall_clock_epoch_s();

  if (!new_run && history.run_count > 0 && history.last_sample_epoch_s != 0 && sample_epoch_s != 0) {
    const uint32_t expected_epoch_s = history.last_sample_epoch_s + history.interval_ms / 1000U;
    const uint32_t delta_s = sample_epoch_s > expected_epoch_s
                                 ? sample_epoch_s - expected_epoch_s
                                 : expected_epoch_s - sample_epoch_s;
    if (delta_s > 2U) new_run = true;
  }

  history.pending_samples[history.pending_head] = sample.encoded();
  history.pending_head = (history.pending_head + 1) % PENDING_CAPACITY;
  if (history.count < HISTORY_CAPACITY) ++history.count;
  ++history.pending;

  if (new_run || history.run_count == 0) {
    history.run_count = 1;
    history.run_anchor_epoch_s = sample_epoch_s;
    flash.metadata_dirty = true;
    ESP_LOGI(TAG, "history continuous run started at sample %u%s",
             static_cast<unsigned>(history.count),
             sample_epoch_s != 0 ? " with wall-clock anchor" : " without wall-clock anchor");
  } else {
    if (history.run_count < HISTORY_CAPACITY) ++history.run_count;
    if (history.run_anchor_epoch_s == 0 && sample_epoch_s != 0) {
      // Time became valid after the run started. Cadence is still known from
      // uptime, so reconstruct the run's first-sample anchor once.
      const uint64_t prior_ms = static_cast<uint64_t>(history.run_count - 1U) * history.interval_ms;
      const uint32_t prior_s = static_cast<uint32_t>(prior_ms / 1000ULL);
      if (sample_epoch_s >= prior_s) {
        history.run_anchor_epoch_s = sample_epoch_s - prior_s;
        flash.metadata_dirty = true;
        ESP_LOGI(TAG, "history wall-clock acquired; anchored current %u-sample run",
                 static_cast<unsigned>(history.run_count));
      }
    }
  }
  history.last_sample_epoch_s = sample_epoch_s;
  history.force_new_run_on_next_sample = false;
  history.run_relative_age_valid = true;
  history.latest_ms = sample_now_ms;
  sync_gatt();

  ESP_LOGI(TAG, "history sample %u/%u: %.2f C / %.1f %% / %u ppm",
           static_cast<unsigned>(history.count), static_cast<unsigned>(HISTORY_CAPACITY),
           sample.temperature_c, sample.humidity_percent, static_cast<unsigned>(sample.co2_ppm));
}

void sampling_tick() {
  if (clearing.phase != ClearPhase::INACTIVE) return;
  const uint32_t now = now_ms();
  if (!history.clock_started) {
    if (sensirion_ble_sample().complete()) {
      history.clock_started = true;
      history.last_sample_ms = now;
      commit_sample(history.force_new_run_on_next_sample || history.run_count == 0);
    }
  } else {
    const uint32_t elapsed = static_cast<uint32_t>(now - history.last_sample_ms);
    if (elapsed >= history.interval_ms) {
      const bool cadence_gap = sensirion_history_format::cadence_gap(elapsed, history.interval_ms);
      history.last_sample_ms += history.interval_ms;
      if (static_cast<uint32_t>(now - history.last_sample_ms) >= history.interval_ms)
        history.last_sample_ms = now;  // no stale backfill after a long pause
      commit_sample(history.force_new_run_on_next_sample || cadence_gap);
    }
  }

  if (history.pending > 0 &&
      static_cast<uint32_t>(now - flash.last_flush_ms) >= FLASH_FLUSH_MS)
    flush_flash();
  else if (flash.metadata_dirty &&
           static_cast<uint32_t>(now - flash.last_meta_attempt_ms) >= META_RETRY_MS)
    flush_flash();
}

void start_download() {
  if (gatt.download == nullptr || !gatt.connected || !gatt.download_notify_enabled ||
      clearing.phase != ClearPhase::INACTIVE)
    return;

  const uint16_t available = downloadable_count();
  uint16_t n = available;
  if (download.requested > 0 && download.requested < n)
    n = static_cast<uint16_t>(download.requested);

  download.count = n;
  download.sent = 0;
  download.sequence = 0;
  download.start_logical = static_cast<uint16_t>(history.count - n);
  if (n == 0) {
    download.age_ms = 0;
  } else if (history.run_relative_age_valid) {
    download.age_ms = now_ms() - history.latest_ms;
  } else {
    const uint32_t now_epoch_s = wall_clock_epoch_s();
    const uint32_t latest_epoch_s = sensirion_history_format::run_latest_epoch_s(
        history.run_anchor_epoch_s, history.run_count, history.interval_ms);
    download.age_ms = (now_epoch_s >= latest_epoch_s)
                          ? (now_epoch_s - latest_epoch_s) * 1000U
                          : 0U;
  }
  download.phase = DownloadPhase::HEADER;
  download.last_packet_ms = 0;
  download.started_ms = now_ms();
  download.last_progress_ms = download.started_ms;

  ESP_LOGI(TAG,
           "history download subscribed: requested=%u stored=%u latest_run=%u sending=%u age=%u ms",
           static_cast<unsigned>(download.requested), static_cast<unsigned>(history.count),
           static_cast<unsigned>(available), static_cast<unsigned>(n),
           static_cast<unsigned>(download.age_ms));
}

uint32_t current_guard_paused_ms(uint32_t now) {
  return download.guard_paused
             ? download.guard_paused_ms +
                   static_cast<uint32_t>(now - download.guard_pause_started_ms)
             : download.guard_paused_ms;
}

void abort_download(const char *reason, uint32_t now) {
  ESP_LOGW(TAG,
           "history download aborted after %u ms (%s); guard=%u pause(s)/%u ms; history and CCCD retained",
           static_cast<unsigned>(now - download.started_ms), reason,
           static_cast<unsigned>(download.guard_pauses),
           static_cast<unsigned>(current_guard_paused_ms(now)));
  download = {};
}

bool capture_guard_blocked(SensirionHistoryCaptureProbe capture_probe) {
  const uint64_t current_us = now_us();
  capture_guard.set_capture_mask(capture_probe != nullptr ? capture_probe() : 0U,
                                 current_us);
  return capture_guard.blocked(current_us);
}

bool download_guard_blocked(SensirionHistoryCaptureProbe capture_probe, uint32_t now) {
  if (capture_guard_blocked(capture_probe)) {
    if (!download.guard_paused) {
      download.guard_paused = true;
      download.guard_pause_started_ms = now;
      if (download.guard_pauses != UINT16_MAX) ++download.guard_pauses;
    }
    return true;
  }
  if (download.guard_paused) {
    download.guard_paused_ms +=
        static_cast<uint32_t>(now - download.guard_pause_started_ms);
    download.guard_paused = false;
  }
  return false;
}

void download_tick(SensirionHistoryCaptureProbe capture_probe) {
  if (download.phase == DownloadPhase::INACTIVE || gatt.download == nullptr ||
      !gatt.connected || !gatt.download_notify_enabled ||
      clearing.phase != ClearPhase::INACTIVE)
    return;

  const uint32_t now = now_ms();
  if (sensirion_history_guard::download_total_timeout(now, download.started_ms)) {
    abort_download("120 s wall-clock timeout", now);
    return;
  }
  if (sensirion_history_guard::download_no_progress_timeout(now, download.last_progress_ms)) {
    abort_download("15 s without a queued notification", now);
    return;
  }
  if (download_guard_blocked(capture_probe, now))
    return;
  if (download.last_packet_ms != 0 && now - download.last_packet_ms < 4)
    return;

  auto &packet = gatt.download->get_value();
  if (packet.size() != 20) packet.resize(20);
  std::fill(packet.begin(), packet.end(), 0);
  if (download.phase == DownloadPhase::HEADER) {
    put_u16_le(&packet[4], DOWNLOAD_TYPE);
    put_u32_le(&packet[6], history.interval_ms);
    put_u32_le(&packet[10], download.age_ms);
    put_u16_le(&packet[14], download.count);
    if (download_guard_blocked(capture_probe, now)) return;
    download.last_packet_ms = now;
    gatt.download->notify();
    download.last_progress_ms = now;
    download.sequence = 1;
    if (download.count == 0) {
      ESP_LOGI(TAG, "history download complete (0 samples; guard=%u pause(s)/%u ms)",
               static_cast<unsigned>(download.guard_pauses),
               static_cast<unsigned>(download.guard_paused_ms));
      download = {};
    } else {
      download.phase = DownloadPhase::DATA;
    }
    return;
  }

  put_u16_le(&packet[0], download.sequence);
  uint16_t next_sent = download.sent;
  for (uint8_t in_packet = 0; in_packet < 2 && next_sent < download.count; ++in_packet) {
    Sample sample{};
    if (!logical_sample(download.start_logical + next_sent, sample)) {
      ESP_LOGE(TAG, "history download aborted: sample %u could not be read",
               static_cast<unsigned>(download.start_logical + next_sent));
      download = {};
      return;
    }
    ++next_sent;
    memcpy(&packet[2 + in_packet * SAMPLE_SIZE], sample.data(), SAMPLE_SIZE);
  }
  if (download_guard_blocked(capture_probe, now)) return;
  download.last_packet_ms = now;
  gatt.download->notify();
  download.sent = next_sent;
  download.last_progress_ms = now;
  ++download.sequence;

  if (download.sent >= download.count) {
    ESP_LOGI(TAG,
             "history download complete: %u sample(s), %u data packet(s), guard=%u pause(s)/%u ms",
             static_cast<unsigned>(download.count), static_cast<unsigned>(download.sequence - 1),
             static_cast<unsigned>(download.guard_pauses),
             static_cast<unsigned>(download.guard_paused_ms));
    download = {};
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
    if (gatt.download_notify_enabled && download.phase == DownloadPhase::INACTIVE)
      start_download();
  });
  gatt.interval->on_write([](std::span<const uint8_t> x, uint16_t conn_id) {
    if (x.size() != 4) {
      ESP_LOGW(TAG, "rejected history interval write with invalid length %u (conn=%u)",
               static_cast<unsigned>(x.size()), static_cast<unsigned>(conn_id));
      sync_gatt();
      return;
    }
    const uint32_t ms = get_u32_le(x);
    ESP_LOGI(TAG, "8001 history interval WRITE = %u ms (conn=%u)",
             static_cast<unsigned>(ms), static_cast<unsigned>(conn_id));
    set_interval(ms);
  });

  sync_gatt();
  server->enqueue_start_service(service);

  gatt.bound = true;
  ESP_LOGI(TAG, "Sensirion GATT configured in component: %u sample(s), interval=%u ms",
           static_cast<unsigned>(history.count), static_cast<unsigned>(history.interval_ms));
}

}  // namespace

void sensirion_history_setup(time::RealTimeClock *clock) {
  wall_clock = clock;
  init_flash();
  sync_gatt();
  flash.last_flush_ms = now_ms();
  ESP_LOGI(TAG,
           "history storage: flash-backed %u samples, RAM pending ring %u samples (%u bytes), sparse-time metadata v4",
           static_cast<unsigned>(HISTORY_CAPACITY), static_cast<unsigned>(PENDING_CAPACITY),
           static_cast<unsigned>(PENDING_CAPACITY * SAMPLE_SIZE));
}

bool sensirion_history_flush() {
  return flush_flash();
}

void sensirion_history_loop(SensirionHistoryCaptureProbe capture_probe) {
  clear_tick();
  if (clearing.phase == ClearPhase::INACTIVE) {
    const uint32_t now = now_ms();
    if (last_sampling_poll_ms == 0 ||
        static_cast<uint32_t>(now - last_sampling_poll_ms) >= 100U) {
      last_sampling_poll_ms = now;
      sampling_tick();
      const uint16_t visible_count = downloadable_count();
      if (visible_count != last_synced_downloadable_count) sync_gatt();
    }
  }
  download_tick(capture_probe);
}

void sensirion_history_note_valid_co2_frame() {
  capture_guard.note_co2_frame(now_us());
}

void sensirion_history_note_rtrh_cycle() { capture_guard.note_rtrh_cycle(now_us()); }

void sensirion_history_note_co2_capture(uint16_t raw_scl_edges, bool frame_error) {
  if (download.phase == DownloadPhase::INACTIVE) return;
  const bool damaged = raw_scl_edges < 130U || frame_error;
  const uint64_t previous_us = capture_guard.pre_guard_us();
  capture_guard.note_capture_quality(damaged);
  const uint64_t current_us = capture_guard.pre_guard_us();
  if (current_us != previous_us) {
    ESP_LOGI(TAG,
             "history adaptive pre-guard %u -> %u ms after CO2 capture SCL=%u frame_error=%s",
             static_cast<unsigned>(previous_us / 1000U),
             static_cast<unsigned>(current_us / 1000U),
             static_cast<unsigned>(raw_scl_edges), frame_error ? "yes" : "no");
  }
}

void sensirion_history_configure_gatt(esp32_ble_server::BLEServer *server) {
  configure_gatt_impl(server);
}

void sensirion_history_gatts_event_handler(
    esp_gatts_cb_event_t event, esp_gatt_if_t, esp_ble_gatts_cb_param_t *param) {
  if (param == nullptr)
    return;
  if (event == ESP_GATTS_CONNECT_EVT) {
    gatt.connected = true;
    return;
  }
  if (event == ESP_GATTS_DISCONNECT_EVT) {
    gatt.connected = false;
    gatt.download_notify_enabled = false;
    download = {};
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
    gatt.download_notify_enabled = true;
    start_download();
  } else if (lo == 0x00 && hi == 0x00) {
    gatt.download_notify_enabled = false;
    download = {};
    ESP_LOGI(TAG, "history download unsubscribed");
  }
}

}  // namespace esphome::co2_monitor_0601
#endif  // UNNI_BLE_HISTORY_ENABLED
