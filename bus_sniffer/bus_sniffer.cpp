#include "bus_sniffer.h"

#include "esphome/core/log.h"

#include "driver/gpio.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"

namespace esphome {
namespace bus_sniffer {

static const char *TAG = "BUS";

// Die drei zu untersuchenden Leitungen
static constexpr gpio_num_t PIN_A = GPIO_NUM_38;
static constexpr gpio_num_t PIN_B = GPIO_NUM_39;
static constexpr gpio_num_t PIN_C = GPIO_NUM_40;

static constexpr int MAX_SAMPLES = 4096;

// Ein I2C-Transfer ist normalerweise deutlich kürzer.
// Wir warten 5 ms nach der letzten Flanke, bevor ausgewertet wird.
static constexpr uint32_t FRAME_TIMEOUT_US = 5000;

struct Sample {
  uint32_t t;
  uint8_t value;
};

volatile Sample samples[MAX_SAMPLES];
volatile uint16_t sample_count = 0;

volatile uint32_t last_edge = 0;
volatile bool buffer_overflow = false;

BusSniffer *instance = nullptr;


// ---------------------------------------------------------------------------
// ISR
// ---------------------------------------------------------------------------

static void IRAM_ATTR gpio_isr(void *arg) {
  uint32_t now = (uint32_t) esp_timer_get_time();

  uint8_t v = 0;

  if (gpio_get_level(PIN_A))
    v |= 1;

  if (gpio_get_level(PIN_B))
    v |= 2;

  if (gpio_get_level(PIN_C))
    v |= 4;

  if (sample_count < MAX_SAMPLES) {
    samples[sample_count].t = now;
    samples[sample_count].value = v;
    sample_count++;
  } else {
    buffer_overflow = true;
  }

  last_edge = now;
}


// ---------------------------------------------------------------------------
// Hilfsfunktionen für Offline-I2C-Dekodierung
// ---------------------------------------------------------------------------

struct I2CResult {
  int sda;
  int scl;
  int third;

  uint32_t scl_rising_edges;
  uint32_t scl_falling_edges;
  uint32_t starts;
  uint32_t stops;

  uint32_t bytes;
  uint32_t ack;
  uint32_t nack;

  int score;
};


// Wert eines einzelnen Bits aus dem gespeicherten 3-Bit-Wert
static inline int get_bit(uint8_t value, int pin) {
  return (value >> pin) & 1;
}


// ---------------------------------------------------------------------------
// Einen bestimmten SDA/SCL-Kandidaten analysieren
// ---------------------------------------------------------------------------

static I2CResult analyze_i2c(
    const Sample *data,
    uint16_t count,
    int sda,
    int scl) {

  I2CResult r{};

  r.sda = sda;
  r.scl = scl;

  // Dritter Pin
  for (int p = 0; p < 3; p++) {
    if (p != sda && p != scl) {
      r.third = p;
      break;
    }
  }

  if (count < 2)
    return r;

  // -------------------------------------------------------------------------
  // Zunächst SCL-Flanken, START und STOP erkennen
  // -------------------------------------------------------------------------

  for (uint16_t i = 1; i < count; i++) {

    int old_scl = get_bit(data[i - 1].value, scl);
    int new_scl = get_bit(data[i].value, scl);

    int old_sda = get_bit(data[i - 1].value, sda);
    int new_sda = get_bit(data[i].value, sda);

    if (!old_scl && new_scl)
      r.scl_rising_edges++;

    if (old_scl && !new_scl)
      r.scl_falling_edges++;

    // START: SDA HIGH -> LOW während SCL HIGH
    if (old_sda && !new_sda && new_scl) {
      r.starts++;
    }

    // STOP: SDA LOW -> HIGH während SCL HIGH
    if (!old_sda && new_sda && new_scl) {
      r.stops++;
    }
  }

  // -------------------------------------------------------------------------
  // I2C-Bitstream dekodieren
  //
  // SDA wird auf steigender SCL-Flanke gelesen.
  //
  // 8 Datenbits + 1 ACK/NACK
  // -------------------------------------------------------------------------

  uint8_t byte_value = 0;
  int bit_count = 0;

  bool in_transaction = false;
  bool waiting_for_ack = false;

  for (uint16_t i = 1; i < count; i++) {

    int old_scl = get_bit(data[i - 1].value, scl);
    int new_scl = get_bit(data[i].value, scl);

    int old_sda = get_bit(data[i - 1].value, sda);
    int new_sda = get_bit(data[i].value, sda);

    // START
    if (old_sda && !new_sda && new_scl) {
      in_transaction = true;
      bit_count = 0;
      byte_value = 0;
      waiting_for_ack = false;
      continue;
    }

    // STOP
    if (in_transaction &&
        !old_sda && new_sda && new_scl) {

      in_transaction = false;
      bit_count = 0;
      byte_value = 0;
      waiting_for_ack = false;
      continue;
    }

    if (!in_transaction)
      continue;

    // Nur steigende SCL-Flanke
    if (!old_scl || !new_scl)
      continue;

    int bit = new_sda;

    // ACK/NACK-Bit
    if (waiting_for_ack) {

      if (bit == 0)
        r.ack++;
      else
        r.nack++;

      waiting_for_ack = false;
      bit_count = 0;
      byte_value = 0;

      continue;
    }

    // Datenbit
    byte_value <<= 1;

    if (bit)
      byte_value |= 1;

    bit_count++;

    if (bit_count == 8) {

      r.bytes++;

      // ---------------------------------------------------------------------
      // Byte ausgeben
      // ---------------------------------------------------------------------

      ESP_LOGI(
          TAG,
          "I2C candidate SDA=GPIO%d SCL=GPIO%d: byte 0x%02X",
          38 + sda,
          38 + scl,
          byte_value);

      waiting_for_ack = true;
    }
  }

  // -------------------------------------------------------------------------
  // Bewertung des Kandidaten
  // -------------------------------------------------------------------------

  //
  // Ein echter I2C-Bus sollte:
  //
  // - viele SCL-Flanken haben
  // - möglichst gleich viele rising/falling edges
  // - START/STOP erkennen
  // - Bytes ergeben
  // - ACKs enthalten
  //

  r.score = 0;

  if (r.scl_rising_edges >= 5)
    r.score += 2;

  if (r.scl_falling_edges >= 5)
    r.score += 2;

  if (r.starts > 0)
    r.score += 5;

  if (r.stops > 0)
    r.score += 5;

  if (r.bytes > 0)
    r.score += 10;

  if (r.ack > 0)
    r.score += 10;

  // ACK/NACK sollte typischerweise nicht ausschließlich NACK sein
  if (r.ack > r.nack)
    r.score += 5;

  return r;
}


// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------

void BusSniffer::setup() {

  instance = this;

  gpio_config_t io = {};

  io.mode = GPIO_MODE_INPUT;

  // Keine Pullups vom ESP.
  // Die Sensorplatine besitzt bereits ca. 10k Pullups.
  io.pull_up_en = GPIO_PULLUP_DISABLE;
  io.pull_down_en = GPIO_PULLDOWN_DISABLE;

  // Jede Flanke erfassen.
  io.intr_type = GPIO_INTR_ANYEDGE;

  io.pin_bit_mask =
      (1ULL << PIN_A) |
      (1ULL << PIN_B) |
      (1ULL << PIN_C);

  gpio_config(&io);

  gpio_install_isr_service(0);

  gpio_isr_handler_add(
      PIN_A,
      gpio_isr,
      nullptr);

  gpio_isr_handler_add(
      PIN_B,
      gpio_isr,
      nullptr);

  gpio_isr_handler_add(
      PIN_C,
      gpio_isr,
      nullptr);

  // Ausgangszustand protokollieren
  uint8_t initial =
      (gpio_get_level(PIN_A) ? 1 : 0) |
      (gpio_get_level(PIN_B) ? 2 : 0) |
      (gpio_get_level(PIN_C) ? 4 : 0);

  ESP_LOGI(
      TAG,
      "Bus sniffer started: GPIO38/39/40");

  ESP_LOGI(
      TAG,
      "Initial state: %u%u%u",
      (initial >> 0) & 1,
      (initial >> 1) & 1,
      (initial >> 2) & 1);
}


// ---------------------------------------------------------------------------
// Loop
// ---------------------------------------------------------------------------

void BusSniffer::loop() {

  if (sample_count == 0)
    return;

  uint32_t now =
      (uint32_t) esp_timer_get_time();

  // Noch Datenverkehr?
  if ((uint32_t)(now - last_edge) < FRAME_TIMEOUT_US)
    return;

  // -------------------------------------------------------------------------
  // Daten atomar übernehmen
  // -------------------------------------------------------------------------

  portDISABLE_INTERRUPTS();

  uint16_t count = sample_count;

  Sample local[MAX_SAMPLES];

  if (count > MAX_SAMPLES)
    count = MAX_SAMPLES;

  for (uint16_t i = 0; i < count; i++) {
    local[i].t = samples[i].t;
    local[i].value = samples[i].value;
  }

  bool overflow = buffer_overflow;

  sample_count = 0;
  buffer_overflow = false;

  portENABLE_INTERRUPTS();

  if (count == 0)
    return;

  // -------------------------------------------------------------------------
  // Header
  // -------------------------------------------------------------------------

  ESP_LOGI(TAG, "==============================");

  ESP_LOGI(
      TAG,
      "Captured %u samples",
      count);

  if (overflow) {
    ESP_LOGW(
        TAG,
        "Buffer overflow - frame truncated");
  }

  // -------------------------------------------------------------------------
  // Grundlegende Statistik
  // -------------------------------------------------------------------------

  uint32_t min_delta = UINT32_MAX;
  uint32_t max_delta = 0;

  uint32_t changes[3] = {0, 0, 0};

  for (uint16_t i = 1; i < count; i++) {

    uint32_t dt =
        local[i].t -
        local[i - 1].t;

    if (dt < min_delta)
      min_delta = dt;

    if (dt > max_delta)
      max_delta = dt;

    uint8_t diff =
        local[i].value ^
        local[i - 1].value;

    if (diff & 1)
      changes[0]++;

    if (diff & 2)
      changes[1]++;

    if (diff & 4)
      changes[2]++;
  }

  ESP_LOGI(
      TAG,
      "min edge spacing: %lu us",
      (unsigned long) min_delta);

  if (min_delta > 0 && min_delta != UINT32_MAX) {

    ESP_LOGI(
        TAG,
        "max edge rate: %.1f kHz",
        1000.0f / min_delta);
  }

  ESP_LOGI(
      TAG,
      "changes GPIO38=%lu GPIO39=%lu GPIO40=%lu",
      (unsigned long) changes[0],
      (unsigned long) changes[1],
      (unsigned long) changes[2]);

  // -------------------------------------------------------------------------
  // I2C-Kandidaten testen
  // -------------------------------------------------------------------------

  ESP_LOGI(TAG, "------------------------------");
  ESP_LOGI(TAG, "I2C ANALYSIS");

  I2CResult results[6];

  int index = 0;

  for (int sda = 0; sda < 3; sda++) {

    for (int scl = 0; scl < 3; scl++) {

      if (sda == scl)
        continue;

      results[index++] =
          analyze_i2c(
              local,
              count,
              sda,
              scl);
    }
  }

  // -------------------------------------------------------------------------
  // Ergebnisse zusammenfassen
  // -------------------------------------------------------------------------

  for (int i = 0; i < 6; i++) {

    I2CResult &r = results[i];

    ESP_LOGI(
        TAG,
        "SDA=GPIO%d SCL=GPIO%d third=GPIO%d:"
        " score=%d"
        " SCL↑=%lu SCL↓=%lu"
        " START=%lu STOP=%lu"
        " bytes=%lu ACK=%lu NACK=%lu",

        38 + r.sda,
        38 + r.scl,
        38 + r.third,

        r.score,

        (unsigned long) r.scl_rising_edges,
        (unsigned long) r.scl_falling_edges,

        (unsigned long) r.starts,
        (unsigned long) r.stops,

        (unsigned long) r.bytes,
        (unsigned long) r.ack,
        (unsigned long) r.nack);
  }

  // -------------------------------------------------------------------------
  // Besten Kandidaten bestimmen
  // -------------------------------------------------------------------------

  int best = 0;

  for (int i = 1; i < 6; i++) {
    if (results[i].score > results[best].score)
      best = i;
  }

  I2CResult &b = results[best];

  ESP_LOGI(
      TAG,
      "BEST I2C CANDIDATE: SDA=GPIO%d SCL=GPIO%d"
      " third=GPIO%d score=%d",

      38 + b.sda,
      38 + b.scl,
      38 + b.third,
      b.score);

  // -------------------------------------------------------------------------
  // Analyse des dritten Pins
  // -------------------------------------------------------------------------

  ESP_LOGI(
      TAG,
      "THIRD PIN ANALYSIS: GPIO%d",
      38 + b.third);

  uint32_t third_changes = 0;

  uint32_t third_high_time = 0;
  uint32_t third_low_time = 0;

  for (uint16_t i = 1; i < count; i++) {

    int old_level =
        get_bit(local[i - 1].value, b.third);

    int new_level =
        get_bit(local[i].value, b.third);

    uint32_t dt =
        local[i].t -
        local[i - 1].t;

    if (old_level != new_level)
      third_changes++;

    if (old_level)
      third_high_time += dt;
    else
      third_low_time += dt;
  }

  ESP_LOGI(
      TAG,
      "GPIO%d changes=%lu",
      38 + b.third,
      (unsigned long) third_changes);

  ESP_LOGI(
      TAG,
      "GPIO%d approx HIGH=%lu us LOW=%lu us",
      38 + b.third,
      (unsigned long) third_high_time,
      (unsigned long) third_low_time);

  // -------------------------------------------------------------------------
  // Rohdaten
  // -------------------------------------------------------------------------

  ESP_LOGI(TAG, "------------------------------");
  ESP_LOGI(TAG, "RAW DATA:");

  uint32_t base = local[0].t;

  for (uint16_t i = 0; i < count; i++) {

    uint32_t rel =
        local[i].t - base;

    // Nicht mehr tausende Zeilen ausgeben.
    // Bei kurzen Transfers alles, sonst maximal 300 Samples.
    if (i < 300 || count <= 300) {

      ESP_LOGI(
          TAG,
          "%lu us %u%u%u",

          (unsigned long) rel,

          (local[i].value >> 0) & 1,
          (local[i].value >> 1) & 1,
          (local[i].value >> 2) & 1);
    }
  }

  if (count > 300) {

    ESP_LOGI(
        TAG,
        "... %u samples omitted ...",
        count - 300);
  }

  ESP_LOGI(TAG, "==============================");
}

}  // namespace bus_sniffer
}  // namespace esphome

