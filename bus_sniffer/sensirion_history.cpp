#include "ble_options.h"
#if UNNI_BLE_HISTORY_ENABLED
#include "sensirion_history.h"
#include "sensirion_ble.h"

#include "esphome/core/log.h"
#include "esphome/components/esp32_ble_server/ble_server.h"
#include "esphome/components/esp32_ble_server/ble_service.h"
#include "esphome/components/esp32_ble_server/ble_characteristic.h"
#include "esphome/components/esp32_ble_server/ble_2902.h"

#include "esp_partition.h"
#include "esp_timer.h"

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <span>
#include <utility>
#include <vector>

namespace esphome {
namespace bus_sniffer {

/*
 * ============================================================================
 * Sensirion-compatible history logger + download protocol
 * ============================================================================
 *
 * The official Gadget library uses:
 *   8001 READ|WRITE  history interval in milliseconds (uint32 LE)
 *   8002 READ        number of available samples (uint32 LE)
 *   8003 WRITE       requested number of samples (uint32 LE; 0 = all)
 *   8004 NOTIFY      20-byte download packets
 *
 * T_RH_CO2_ALT history samples are 8 bytes:
 *   T raw u16 LE, RH raw u16 LE, CO2 ppm u16 LE, 00 00.
 * Download type is 7 and two samples fit in each 20-byte packet.
 *
 * RAM keeps 4096 samples.  At 1 minute this is 68 h 16 min, i.e. about
 * 2.84 days.  The flash partition is an append-only physical log larger than
 * the logical RAM history.  This lets us erase a flash sector only after all
 * records in it are older than the 4096-sample logical window.
 */

static constexpr size_t SENS_HISTORY_SAMPLE_SIZE = 8;
static constexpr size_t SENS_HISTORY_CAPACITY = 4096;
static constexpr uint32_t SENS_HISTORY_DEFAULT_INTERVAL_MS = 600000;
static constexpr uint32_t SENS_HISTORY_FLASH_FLUSH_MS = 600000;
static constexpr size_t SENS_HISTORY_FLASH_SECTOR = 4096;
static constexpr size_t SENS_HISTORY_FLASH_META_SECTOR = 0;
static constexpr size_t SENS_HISTORY_FLASH_DATA_FIRST_SECTOR = 1;
static constexpr size_t SENS_HISTORY_FLASH_DATA_SECTORS = 14;
static constexpr size_t SENS_HISTORY_FLASH_SAMPLES_PER_SECTOR =
    SENS_HISTORY_FLASH_SECTOR / SENS_HISTORY_SAMPLE_SIZE;  // 512
static constexpr size_t SENS_HISTORY_FLASH_CAPACITY =
    SENS_HISTORY_FLASH_DATA_SECTORS * SENS_HISTORY_FLASH_SAMPLES_PER_SECTOR; // 7168
static constexpr uint32_t SENS_HISTORY_META_MAGIC = 0x53474832; // "SGH2"
static constexpr uint16_t SENS_HISTORY_META_VERSION = 2;
static constexpr uint16_t SENS_HISTORY_DOWNLOAD_TYPE = 7;

struct SensHistorySample {
  uint8_t data[SENS_HISTORY_SAMPLE_SIZE];
};

struct __attribute__((packed)) SensHistoryMeta {
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
static_assert(sizeof(SensHistoryMeta) == 32, "history meta record must be 32 bytes");

static SensHistorySample sens_history[SENS_HISTORY_CAPACITY];
static uint16_t sens_history_head = 0;  // next RAM write slot
static uint16_t sens_history_count = 0;
static uint32_t sens_history_interval_ms = SENS_HISTORY_DEFAULT_INTERVAL_MS;
static uint32_t sens_history_latest_ms = 0;
static uint32_t sens_history_last_sample_ms = 0;
static bool sens_history_sample_clock_started = false;

static const esp_partition_t *sens_history_partition = nullptr;
static bool sens_history_flash_ready = false;
static uint16_t sens_history_flash_write_slot = 0;
static uint16_t sens_history_persisted_count = 0;
static uint16_t sens_history_pending_count = 0;
static uint32_t sens_history_meta_generation = 0;
static uint16_t sens_history_meta_next_slot = 0;
static uint32_t sens_history_last_flush_ms = 0;

static esp32_ble_server::BLECharacteristic *sens_history_count_char = nullptr;
static esp32_ble_server::BLECharacteristic *sens_history_interval_char = nullptr;
static esp32_ble_server::BLECharacteristic *sens_history_requested_char = nullptr;
static esp32_ble_server::BLECharacteristic *sens_history_download_char = nullptr;

class SensirionHistoryCCCD : public esp32_ble_server::BLE2902 {
 public:
  uint16_t get_handle() const { return this->handle_; }
};

static SensirionHistoryCCCD *sens_history_download_cccd = nullptr;
static bool sens_history_gatt_bound = false;

static uint32_t sens_history_requested_samples = 0;

enum class SensHistoryDownloadState : uint8_t {
  INACTIVE = 0,
  START,
  DOWNLOADING,
};
static SensHistoryDownloadState sens_history_download_state =
    SensHistoryDownloadState::INACTIVE;
static uint16_t sens_history_download_sequence = 0;
static uint16_t sens_history_download_count = 0;
static uint16_t sens_history_download_sent = 0;
static uint16_t sens_history_download_start_logical = 0;
static uint32_t sens_history_download_age_ms = 0;
static uint32_t sens_history_last_packet_ms = 0;

static inline uint32_t sens_history_now_ms()
{
  return static_cast<uint32_t>(esp_timer_get_time() / 1000ULL);
}

static inline uint32_t sens_history_get_u32_le(std::span<const uint8_t> x)
{
  return static_cast<uint32_t>(x[0]) |
         (static_cast<uint32_t>(x[1]) << 8) |
         (static_cast<uint32_t>(x[2]) << 16) |
         (static_cast<uint32_t>(x[3]) << 24);
}

static inline void sens_history_put_u16_le(uint8_t *p, uint16_t v)
{
  p[0] = static_cast<uint8_t>(v & 0xFF);
  p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
}

static inline void sens_history_put_u32_le(uint8_t *p, uint32_t v)
{
  p[0] = static_cast<uint8_t>(v & 0xFF);
  p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
  p[2] = static_cast<uint8_t>((v >> 16) & 0xFF);
  p[3] = static_cast<uint8_t>((v >> 24) & 0xFF);
}

static uint32_t sens_history_meta_crc(const SensHistoryMeta &m)
{
  // FNV-1a over the record excluding crc/reserved1.
  const uint8_t *p = reinterpret_cast<const uint8_t *>(&m);
  constexpr size_t n = offsetof(SensHistoryMeta, crc);
  uint32_t h = 2166136261UL;
  for (size_t i = 0; i < n; i++) {
    h ^= p[i];
    h *= 16777619UL;
  }
  return h;
}

static bool sens_history_meta_valid(const SensHistoryMeta &m)
{
  return m.magic == SENS_HISTORY_META_MAGIC &&
         m.version == SENS_HISTORY_META_VERSION &&
         m.size == sizeof(SensHistoryMeta) &&
         m.count <= SENS_HISTORY_CAPACITY &&
         m.flash_write_slot < SENS_HISTORY_FLASH_CAPACITY &&
         m.interval_ms > 0 &&
         m.crc == sens_history_meta_crc(m);
}

static size_t sens_history_flash_sample_offset(uint16_t slot)
{
  return SENS_HISTORY_FLASH_DATA_FIRST_SECTOR * SENS_HISTORY_FLASH_SECTOR +
         static_cast<size_t>(slot) * SENS_HISTORY_SAMPLE_SIZE;
}

static bool sens_history_write_meta()
{
  if (!sens_history_flash_ready)
    return false;

  constexpr uint16_t records_per_sector =
      SENS_HISTORY_FLASH_SECTOR / sizeof(SensHistoryMeta);

  if (sens_history_meta_next_slot >= records_per_sector) {
    esp_err_t err = esp_partition_erase_range(
        sens_history_partition,
        SENS_HISTORY_FLASH_META_SECTOR * SENS_HISTORY_FLASH_SECTOR,
        SENS_HISTORY_FLASH_SECTOR);
    if (err != ESP_OK) {
      ESP_LOGE("sensirion_history", "metadata sector erase failed: %d", err);
      return false;
    }
    sens_history_meta_next_slot = 0;
  }

  SensHistoryMeta m{};
  m.magic = SENS_HISTORY_META_MAGIC;
  m.version = SENS_HISTORY_META_VERSION;
  m.size = sizeof(SensHistoryMeta);
  m.generation = ++sens_history_meta_generation;
  m.interval_ms = sens_history_interval_ms;
  m.count = sens_history_persisted_count;
  m.flash_write_slot = sens_history_flash_write_slot;
  m.crc = sens_history_meta_crc(m);

  const size_t offset = static_cast<size_t>(sens_history_meta_next_slot) *
                        sizeof(SensHistoryMeta);
  esp_err_t err = esp_partition_write(
      sens_history_partition, offset, &m, sizeof(m));
  if (err != ESP_OK) {
    ESP_LOGE("sensirion_history", "metadata write failed: %d", err);
    return false;
  }

  sens_history_meta_next_slot++;
  return true;
}

static void sens_history_sync_gatt_values()
{
  if (sens_history_count_char != nullptr) {
    const uint32_t n = sens_history_count;
    sens_history_count_char->set_value({
        static_cast<uint8_t>(n & 0xFF),
        static_cast<uint8_t>((n >> 8) & 0xFF),
        static_cast<uint8_t>((n >> 16) & 0xFF),
        static_cast<uint8_t>((n >> 24) & 0xFF)});
  }
  if (sens_history_interval_char != nullptr) {
    const uint32_t ms = sens_history_interval_ms;
    sens_history_interval_char->set_value({
        static_cast<uint8_t>(ms & 0xFF),
        static_cast<uint8_t>((ms >> 8) & 0xFF),
        static_cast<uint8_t>((ms >> 16) & 0xFF),
        static_cast<uint8_t>((ms >> 24) & 0xFF)});
  }
}

static void sens_history_flash_clear()
{
  sens_history_head = 0;
  sens_history_count = 0;
  sens_history_pending_count = 0;
  sens_history_persisted_count = 0;
  sens_history_flash_write_slot = 0;
  sens_history_sample_clock_started = false;
  sens_history_latest_ms = 0;
  sens_history_last_sample_ms = 0;
  sens_history_download_state = SensHistoryDownloadState::INACTIVE;

  if (sens_history_flash_ready) {
    const size_t erase_len =
        (1 + SENS_HISTORY_FLASH_DATA_SECTORS) * SENS_HISTORY_FLASH_SECTOR;
    esp_err_t err = esp_partition_erase_range(
        sens_history_partition, 0, erase_len);
    if (err != ESP_OK) {
      ESP_LOGE("sensirion_history", "history erase failed: %d", err);
    } else {
      sens_history_meta_generation = 0;
      sens_history_meta_next_slot = 0;
      sens_history_write_meta();
    }
  }

  sens_history_sync_gatt_values();
}

static void sens_history_set_interval(uint32_t ms)
{
  if (ms == 0)
    return;

  if (ms == sens_history_interval_ms)
    return;

  ESP_LOGI("sensirion_history",
           "logging interval changed %u -> %u ms; clearing history",
           static_cast<unsigned>(sens_history_interval_ms),
           static_cast<unsigned>(ms));

  sens_history_interval_ms = ms;
  sens_history_flash_clear();
}

static bool sens_history_flash_flush()
{
  if (!sens_history_flash_ready || sens_history_pending_count == 0)
    return true;

  // The pending samples are the newest pending_count entries in the RAM ring.
  uint16_t first_ram = static_cast<uint16_t>(
      (sens_history_head + SENS_HISTORY_CAPACITY - sens_history_pending_count) %
      SENS_HISTORY_CAPACITY);

  uint16_t new_flash_write_slot = sens_history_flash_write_slot;
  uint16_t new_persisted_count = sens_history_persisted_count;

  for (uint16_t i = 0; i < sens_history_pending_count; i++) {
    const uint16_t flash_slot = new_flash_write_slot;
    if ((flash_slot % SENS_HISTORY_FLASH_SAMPLES_PER_SECTOR) == 0) {
      const size_t sector = SENS_HISTORY_FLASH_DATA_FIRST_SECTOR +
          flash_slot / SENS_HISTORY_FLASH_SAMPLES_PER_SECTOR;
      esp_err_t err = esp_partition_erase_range(
          sens_history_partition,
          sector * SENS_HISTORY_FLASH_SECTOR,
          SENS_HISTORY_FLASH_SECTOR);
      if (err != ESP_OK) {
        ESP_LOGE("sensirion_history", "data sector erase failed: %d", err);
        return false;
      }
    }

    const uint16_t ram_slot = static_cast<uint16_t>(
        (first_ram + i) % SENS_HISTORY_CAPACITY);
    esp_err_t err = esp_partition_write(
        sens_history_partition,
        sens_history_flash_sample_offset(flash_slot),
        sens_history[ram_slot].data,
        SENS_HISTORY_SAMPLE_SIZE);
    if (err != ESP_OK) {
      ESP_LOGE("sensirion_history", "sample flash write failed: %d", err);
      return false;
    }

    new_flash_write_slot = static_cast<uint16_t>(
        (new_flash_write_slot + 1) % SENS_HISTORY_FLASH_CAPACITY);
    if (new_persisted_count < SENS_HISTORY_CAPACITY)
      new_persisted_count++;
  }

  // Commit the in-RAM flash journal pointers only after the complete batch has
  // been written successfully. If power fails before the metadata record, the
  // old journal entry remains authoritative and this batch is simply ignored.
  sens_history_flash_write_slot = new_flash_write_slot;
  sens_history_persisted_count = new_persisted_count;

  const uint16_t flushed = sens_history_pending_count;
  sens_history_pending_count = 0;
  sens_history_last_flush_ms = sens_history_now_ms();

  if (!sens_history_write_meta())
    return false;

  ESP_LOGI("sensirion_history",
           "flushed %u sample(s) to flash; persisted=%u, flash_slot=%u",
           static_cast<unsigned>(flushed),
           static_cast<unsigned>(sens_history_persisted_count),
           static_cast<unsigned>(sens_history_flash_write_slot));
  return true;
}

static void sens_history_flash_init()
{
  sens_history_partition = esp_partition_find_first(
      ESP_PARTITION_TYPE_DATA,
      ESP_PARTITION_SUBTYPE_ANY,
      "senshist");

  if (sens_history_partition == nullptr) {
    ESP_LOGW("sensirion_history",
             "no 'senshist' partition; history works in RAM only");
    return;
  }

  const size_t required =
      (1 + SENS_HISTORY_FLASH_DATA_SECTORS) * SENS_HISTORY_FLASH_SECTOR;
  if (sens_history_partition->size < required) {
    ESP_LOGE("sensirion_history",
             "senshist partition too small: %u < %u",
             static_cast<unsigned>(sens_history_partition->size),
             static_cast<unsigned>(required));
    return;
  }

  sens_history_flash_ready = true;

  constexpr uint16_t records_per_sector =
      SENS_HISTORY_FLASH_SECTOR / sizeof(SensHistoryMeta);
  SensHistoryMeta best{};
  bool found = false;
  uint16_t best_slot = 0;

  for (uint16_t slot = 0; slot < records_per_sector; slot++) {
    SensHistoryMeta m{};
    const size_t off = static_cast<size_t>(slot) * sizeof(m);
    if (esp_partition_read(sens_history_partition, off, &m, sizeof(m)) != ESP_OK)
      break;
    if (!sens_history_meta_valid(m))
      continue;
    if (!found || m.generation > best.generation) {
      best = m;
      best_slot = slot;
      found = true;
    }
  }

  if (!found) {
    ESP_LOGI("sensirion_history", "initializing empty flash history");
    sens_history_meta_generation = 0;
    sens_history_meta_next_slot = 0;
    // Erase only the metadata sector now. Data sectors are lazily erased before
    // their first write.
    esp_partition_erase_range(sens_history_partition, 0, SENS_HISTORY_FLASH_SECTOR);
    sens_history_write_meta();
    return;
  }

  sens_history_interval_ms = best.interval_ms;
  sens_history_flash_write_slot = best.flash_write_slot;
  sens_history_persisted_count = best.count;
  sens_history_meta_generation = best.generation;
  sens_history_meta_next_slot = static_cast<uint16_t>(best_slot + 1);

  // Rebuild RAM history in chronological order from the newest persisted
  // logical window in the larger circular flash log.
  const uint16_t n = best.count;
  const uint16_t first_flash = static_cast<uint16_t>(
      (best.flash_write_slot + SENS_HISTORY_FLASH_CAPACITY - n) %
      SENS_HISTORY_FLASH_CAPACITY);

  sens_history_head = 0;
  sens_history_count = 0;
  for (uint16_t i = 0; i < n; i++) {
    const uint16_t flash_slot = static_cast<uint16_t>(
        (first_flash + i) % SENS_HISTORY_FLASH_CAPACITY);
    SensHistorySample sample{};
    if (esp_partition_read(
            sens_history_partition,
            sens_history_flash_sample_offset(flash_slot),
            sample.data,
            SENS_HISTORY_SAMPLE_SIZE) != ESP_OK) {
      ESP_LOGW("sensirion_history", "flash history read stopped at %u", i);
      break;
    }
    sens_history[sens_history_head] = sample;
    sens_history_head = static_cast<uint16_t>(
        (sens_history_head + 1) % SENS_HISTORY_CAPACITY);
    if (sens_history_count < SENS_HISTORY_CAPACITY)
      sens_history_count++;
  }
  sens_history_persisted_count = sens_history_count;
  sens_history_pending_count = 0;
  sens_history_latest_ms = sens_history_now_ms();
  sens_history_last_flush_ms = sens_history_latest_ms;

  ESP_LOGI("sensirion_history",
           "restored %u sample(s), interval=%u ms, flash_slot=%u",
           static_cast<unsigned>(sens_history_count),
           static_cast<unsigned>(sens_history_interval_ms),
           static_cast<unsigned>(sens_history_flash_write_slot));
}

static SensHistorySample sens_history_current_sample()
{
  SensHistorySample s{};
  const uint16_t rt = sensirion_ble_encode_temperature(sensirion_ble_temperature());
  const uint16_t rh = sensirion_ble_encode_humidity(sensirion_ble_humidity());
  sens_history_put_u16_le(&s.data[0], rt);
  sens_history_put_u16_le(&s.data[2], rh);
  sens_history_put_u16_le(&s.data[4], sensirion_ble_co2());
  s.data[6] = 0;
  s.data[7] = 0;
  return s;
}

static void sens_history_commit_sample()
{
  if (!sensirion_ble_has_temperature() ||
      !sensirion_ble_has_humidity() ||
      !sensirion_ble_has_co2())
    return;

  sens_history[sens_history_head] = sens_history_current_sample();
  sens_history_head = static_cast<uint16_t>(
      (sens_history_head + 1) % SENS_HISTORY_CAPACITY);
  if (sens_history_count < SENS_HISTORY_CAPACITY)
    sens_history_count++;

  if (sens_history_pending_count < SENS_HISTORY_CAPACITY)
    sens_history_pending_count++;

  sens_history_latest_ms = sens_history_now_ms();
  sens_history_sync_gatt_values();

  ESP_LOGI("sensirion_history",
           "history sample %u/%u: %.2f C / %.1f %% / %u ppm",
           static_cast<unsigned>(sens_history_count),
           static_cast<unsigned>(SENS_HISTORY_CAPACITY),
           sensirion_ble_temperature(),
           sensirion_ble_humidity(),
           static_cast<unsigned>(sensirion_ble_co2()));
}

static void sens_history_sampling_tick()
{
  const uint32_t now = sens_history_now_ms();

  if (!sens_history_sample_clock_started) {
    if (sensirion_ble_has_temperature() && sensirion_ble_has_humidity() &&
        sensirion_ble_has_co2()) {
      sens_history_sample_clock_started = true;
      sens_history_last_sample_ms = now;
      sens_history_commit_sample();
    }
  } else if (static_cast<uint32_t>(now - sens_history_last_sample_ms) >=
             sens_history_interval_ms) {
    // Keep phase stable even if loop execution is delayed.
    sens_history_last_sample_ms += sens_history_interval_ms;
    if (static_cast<uint32_t>(now - sens_history_last_sample_ms) >=
        sens_history_interval_ms) {
      // Do not backfill stale duplicated measurements after a long pause.
      sens_history_last_sample_ms = now;
    }
    sens_history_commit_sample();
  }

  if (sens_history_pending_count > 0 &&
      static_cast<uint32_t>(now - sens_history_last_flush_ms) >=
          SENS_HISTORY_FLASH_FLUSH_MS) {
    sens_history_flash_flush();
  }
}

static SensHistorySample sens_history_get_logical(uint16_t logical_index)
{
  // logical_index 0 = oldest sample currently retained.
  const uint16_t oldest = static_cast<uint16_t>(
      (sens_history_head + SENS_HISTORY_CAPACITY - sens_history_count) %
      SENS_HISTORY_CAPACITY);
  const uint16_t slot = static_cast<uint16_t>(
      (oldest + logical_index) % SENS_HISTORY_CAPACITY);
  return sens_history[slot];
}

static void sens_history_start_download()
{
  if (sens_history_download_char == nullptr)
    return;

  uint32_t requested = sens_history_requested_samples;
  uint16_t n = sens_history_count;
  if (requested > 0 && requested < n)
    n = static_cast<uint16_t>(requested);

  sens_history_download_count = n;
  sens_history_download_sent = 0;
  sens_history_download_sequence = 0;
  sens_history_download_start_logical = static_cast<uint16_t>(
      sens_history_count - n);

  const uint32_t now = sens_history_now_ms();
  sens_history_download_age_ms =
      (sens_history_count > 0)
          ? static_cast<uint32_t>(now - sens_history_latest_ms)
          : 0;
  sens_history_download_state = SensHistoryDownloadState::START;
  sens_history_last_packet_ms = 0;

  ESP_LOGI("sensirion_history",
           "history download subscribed: requested=%u available=%u sending=%u age=%u ms",
           static_cast<unsigned>(requested),
           static_cast<unsigned>(sens_history_count),
           static_cast<unsigned>(n),
           static_cast<unsigned>(sens_history_download_age_ms));
}

static void sens_history_download_tick()
{
  if (sens_history_download_state == SensHistoryDownloadState::INACTIVE ||
      sens_history_download_char == nullptr)
    return;

  const uint32_t now = sens_history_now_ms();
  if (sens_history_last_packet_ms != 0 &&
      static_cast<uint32_t>(now - sens_history_last_packet_ms) < 4)
    return;
  sens_history_last_packet_ms = now;

  std::vector<uint8_t> packet(20, 0);

  if (sens_history_download_state == SensHistoryDownloadState::START) {
    // Header sequence bytes 0..3 stay zero, matching Sensirion DownloadHeader.
    sens_history_put_u16_le(&packet[4], SENS_HISTORY_DOWNLOAD_TYPE);
    sens_history_put_u32_le(&packet[6], sens_history_interval_ms);
    sens_history_put_u32_le(&packet[10], sens_history_download_age_ms);
    sens_history_put_u16_le(&packet[14], sens_history_download_count);

    sens_history_download_char->set_value(std::move(packet));
    sens_history_download_char->notify();
    sens_history_download_sequence = 1;

    if (sens_history_download_count == 0) {
      sens_history_download_state = SensHistoryDownloadState::INACTIVE;
      ESP_LOGI("sensirion_history", "history download complete (0 samples)");
    } else {
      sens_history_download_state = SensHistoryDownloadState::DOWNLOADING;
    }
    return;
  }

  sens_history_put_u16_le(&packet[0], sens_history_download_sequence);

  uint8_t packet_samples = 0;
  while (packet_samples < 2 &&
         sens_history_download_sent < sens_history_download_count) {
    const uint16_t logical = static_cast<uint16_t>(
        sens_history_download_start_logical + sens_history_download_sent);
    const SensHistorySample s = sens_history_get_logical(logical);
    memcpy(&packet[2 + packet_samples * SENS_HISTORY_SAMPLE_SIZE],
           s.data, SENS_HISTORY_SAMPLE_SIZE);
    packet_samples++;
    sens_history_download_sent++;
  }

  sens_history_download_char->set_value(std::move(packet));
  sens_history_download_char->notify();
  sens_history_download_sequence++;

  if (sens_history_download_sent >= sens_history_download_count) {
    ESP_LOGI("sensirion_history",
             "history download complete: %u sample(s), %u data packet(s)",
             static_cast<unsigned>(sens_history_download_count),
             static_cast<unsigned>(sens_history_download_sequence - 1));
    sens_history_download_state = SensHistoryDownloadState::INACTIVE;
    sens_history_requested_samples = 0;
  }
}

static void sens_history_configure_gatt(esp32_ble_server::BLEServer *server)
{
  if (sens_history_gatt_bound || server == nullptr)
    return;

  using esp32_ble::ESPBTUUID;
  using esp32_ble_server::BLECharacteristic;

  // Device Information (0x180A) stays owned by ESPHome.  It is generic
  // metadata, not part of the Sensirion history protocol.  Keeping it out of
  // this component also avoids coupling to ESPHome's DIS handle allocation.

  auto download_uuid = ESPBTUUID::from_raw(
      "00008000-B38D-4985-720E-0F993A68EE41");
  auto *service = server->get_service(download_uuid);
  if (service == nullptr) {
    // Service declaration + 4 characteristics + one CCCD = 10 handles.
    service = server->create_service(download_uuid, false, 10);
  }
  if (service == nullptr) {
    ESP_LOGE("sensirion_history", "failed to create 0x8000 history service");
    return;
  }

  sens_history_count_char = service->get_characteristic(
      ESPBTUUID::from_raw("00008002-B38D-4985-720E-0F993A68EE41"));
  if (sens_history_count_char == nullptr) {
    sens_history_count_char = service->create_characteristic(
        ESPBTUUID::from_raw("00008002-B38D-4985-720E-0F993A68EE41"),
        BLECharacteristic::PROPERTY_READ);
  }

  sens_history_requested_char = service->get_characteristic(
      ESPBTUUID::from_raw("00008003-B38D-4985-720E-0F993A68EE41"));
  if (sens_history_requested_char == nullptr) {
    sens_history_requested_char = service->create_characteristic(
        ESPBTUUID::from_raw("00008003-B38D-4985-720E-0F993A68EE41"),
        BLECharacteristic::PROPERTY_WRITE);
  }

  sens_history_interval_char = service->get_characteristic(
      ESPBTUUID::from_raw("00008001-B38D-4985-720E-0F993A68EE41"));
  if (sens_history_interval_char == nullptr) {
    sens_history_interval_char = service->create_characteristic(
        ESPBTUUID::from_raw("00008001-B38D-4985-720E-0F993A68EE41"),
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  }

  sens_history_download_char = service->get_characteristic(
      ESPBTUUID::from_raw("00008004-B38D-4985-720E-0F993A68EE41"));
  if (sens_history_download_char == nullptr) {
    sens_history_download_char = service->create_characteristic(
        ESPBTUUID::from_raw("00008004-B38D-4985-720E-0F993A68EE41"),
        BLECharacteristic::PROPERTY_NOTIFY);
    sens_history_download_cccd = new SensirionHistoryCCCD();
    sens_history_download_cccd->set_value({0x00, 0x00});
    sens_history_download_char->add_descriptor(sens_history_download_cccd);
  }

  if (sens_history_count_char == nullptr ||
      sens_history_requested_char == nullptr ||
      sens_history_interval_char == nullptr ||
      sens_history_download_char == nullptr ||
      sens_history_download_cccd == nullptr) {
    ESP_LOGE("sensirion_history", "failed to create complete Sensirion GATT topology");
    return;
  }

  sens_history_count_char->set_value({0, 0, 0, 0});
  sens_history_requested_char->set_value({0, 0, 0, 0});
  sens_history_download_char->set_value(std::vector<uint8_t>(20, 0));

  sens_history_requested_char->on_write(
      [](std::span<const uint8_t> x, uint16_t conn_id) {
        if (x.size() < 4)
          return;
        sens_history_requested_samples = sens_history_get_u32_le(x);
        ESP_LOGI("sensirion_history",
                 "8003 requested samples = %u (conn=%u)",
                 static_cast<unsigned>(sens_history_requested_samples),
                 static_cast<unsigned>(conn_id));
      });

  sens_history_interval_char->on_write(
      [](std::span<const uint8_t> x, uint16_t conn_id) {
        if (x.size() < 4)
          return;
        const uint32_t ms = sens_history_get_u32_le(x);
        ESP_LOGI("sensirion_history",
                 "8001 history interval WRITE = %u ms (conn=%u)",
                 static_cast<unsigned>(ms),
                 static_cast<unsigned>(conn_id));
        sens_history_set_interval(ms);
      });

  sens_history_sync_gatt_values();

  // Non-DIS services must be explicitly queued for start.
  server->enqueue_start_service(service);

  auto settings_uuid = ESPBTUUID::from_raw(
      "00008100-B38D-4985-720E-0F993A68EE41");
  auto *settings = server->get_service(settings_uuid);
  if (settings == nullptr)
    settings = server->create_service(settings_uuid, false, 1);
  if (settings != nullptr)
    server->enqueue_start_service(settings);

  sens_history_gatt_bound = true;
  ESP_LOGI("sensirion_history",
           "Sensirion GATT configured in component: %u sample(s), interval=%u ms",
           static_cast<unsigned>(sens_history_count),
           static_cast<unsigned>(sens_history_interval_ms));
}


void sensirion_history_setup()
{
  sens_history_flash_init();

  // GATT characteristics may already have been created with the compile-time
  // defaults before flash restoration runs. Publish the restored interval and
  // sample count immediately so MyAmbience sees the persistent state on its
  // first read after boot, rather than 600000 ms / 0 samples until the next
  // history sample is recorded.
  sens_history_sync_gatt_values();

  sens_history_last_flush_ms = sens_history_now_ms();
}

void sensirion_history_loop()
{
  sens_history_sampling_tick();
  sens_history_download_tick();
}

void sensirion_history_configure_gatt(esp32_ble_server::BLEServer *server)
{
  sens_history_configure_gatt(server);
}

void sensirion_history_gatts_event_handler(
    esp_gatts_cb_event_t event,
    esp_gatt_if_t,
    esp_ble_gatts_cb_param_t *param)
{
  if (param == nullptr)
    return;

  if (event == ESP_GATTS_DISCONNECT_EVT) {
    sens_history_download_state = SensHistoryDownloadState::INACTIVE;
    sens_history_requested_samples = 0;
    return;
  }

  if (event != ESP_GATTS_WRITE_EVT || param->write.is_prep)
    return;

  ESP_LOGD("sensirion_history",
           "GATT WRITE handle=0x%04X len=%u conn=%u",
           static_cast<unsigned>(param->write.handle),
           static_cast<unsigned>(param->write.len),
           static_cast<unsigned>(param->write.conn_id));

  if (sens_history_download_cccd == nullptr ||
      param->write.handle != sens_history_download_cccd->get_handle() ||
      param->write.len != 2)
    return;

  const uint8_t lo = param->write.value[0];
  const uint8_t hi = param->write.value[1];
  ESP_LOGI("sensirion_history",
           "8004 CCCD WRITE = %02X %02X (conn=%u, handle=0x%04X)",
           lo, hi, static_cast<unsigned>(param->write.conn_id),
           static_cast<unsigned>(param->write.handle));

  if (lo == 0x01 && hi == 0x00) {
    sens_history_start_download();
  } else if (lo == 0x00 && hi == 0x00) {
    sens_history_download_state = SensHistoryDownloadState::INACTIVE;
    ESP_LOGI("sensirion_history", "history download unsubscribed");
  }
}

}  // namespace bus_sniffer
}  // namespace esphome

#endif  // UNNI_BLE_HISTORY_ENABLED
