#include "bus_sniffer.h"

#include "esphome/core/log.h"

#include "driver/gpio.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

#include <stdint.h>

namespace esphome {
namespace bus_sniffer {

static const char *TAG = "BUS";

// -----------------------------------------------------------------------------
// Pins
// -----------------------------------------------------------------------------

static constexpr gpio_num_t PIN_A = GPIO_NUM_38;
static constexpr gpio_num_t PIN_B = GPIO_NUM_39;
static constexpr gpio_num_t PIN_C = GPIO_NUM_40;

// -----------------------------------------------------------------------------
// Capture parameters
// -----------------------------------------------------------------------------

static constexpr uint16_t MAX_SAMPLES = 1024;

// Nach dieser Zeit ohne Flanke wird ein Frame als beendet betrachtet.
static constexpr uint32_t FRAME_TIMEOUT_US = 50000;

// Sehr kurze Störungen ignorieren.
static constexpr uint32_t GLITCH_FILTER_US = 3;

// -----------------------------------------------------------------------------
// Sample
// -----------------------------------------------------------------------------

struct Sample {
  uint32_t t;
  uint8_t value;
};

// -----------------------------------------------------------------------------
// Capture buffer
//
// Die Daten werden ausschließlich von der ISR geschrieben und später von
// loop() gelesen.
// -----------------------------------------------------------------------------

static volatile Sample samples[MAX_SAMPLES];

static volatile uint16_t sample_count = 0;
static volatile uint32_t last_edge = 0;
static volatile uint8_t last_value = 0xff;

static BusSniffer *instance = nullptr;

// -----------------------------------------------------------------------------
// Read all three pins
// -----------------------------------------------------------------------------

static inline uint8_t IRAM_ATTR read_bus()
{
  uint8_t value = 0;

  if (gpio_get_level(PIN_A))
    value |= 1;

  if (gpio_get_level(PIN_B))
    value |= 2;

  if (gpio_get_level(PIN_C))
    value |= 4;

  return value;
}

// -----------------------------------------------------------------------------
// GPIO ISR
// -----------------------------------------------------------------------------

static void IRAM_ATTR gpio_isr(void *arg)
{
  const uint32_t now =
      (uint32_t) esp_timer_get_time();

  const uint8_t value = read_bus();

  // Mehrere GPIO-Interrupts können praktisch gleichzeitig auftreten.
  // Wenn der kombinierte Zustand gleich geblieben ist, gibt es nichts
  // Interessantes zu speichern.
  if (value == last_value)
    return;

  // Sehr kurze Glitches unterdrücken.
  if ((uint32_t)(now - last_edge) < GLITCH_FILTER_US)
    return;

  last_edge = now;
  last_value = value;

  const uint16_t index = sample_count;

  if (index >= MAX_SAMPLES)
    return;

  samples[index].t = now;
  samples[index].value = value;

  sample_count = index + 1;
}

// -----------------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------------

static const char *pin_name(int pin)
{
  switch (pin) {
    case 0:
      return "GPIO38";
    case 1:
      return "GPIO39";
    case 2:
      return "GPIO40";
    default:
      return "?";
  }
}

// -----------------------------------------------------------------------------
// Analyse eines Frames
//
// Ziel:
//   - herausfinden, welche zwei Leitungen sich wie I2C verhalten
//   - dritte Leitung charakterisieren
// -----------------------------------------------------------------------------

static void analyze_frame(
    const Sample *data,
    uint16_t count)
{
  if (count < 2)
    return;

  uint32_t changes[3] = {0, 0, 0};

  uint32_t rising[3] = {0, 0, 0};
  uint32_t falling[3] = {0, 0, 0};

  uint32_t min_delta = UINT32_MAX;
  uint32_t max_delta = 0;

  // Zeitabstände und Flanken analysieren.
  for (uint16_t i = 1; i < count; i++) {

    const uint32_t dt =
        data[i].t - data[i - 1].t;

    if (dt < min_delta)
      min_delta = dt;

    if (dt > max_delta)
      max_delta = dt;

    const uint8_t old_value =
        data[i - 1].value;

    const uint8_t new_value =
        data[i].value;

    const uint8_t diff =
        old_value ^ new_value;

    for (int p = 0; p < 3; p++) {

      const uint8_t mask =
          (1 << p);

      if (!(diff & mask))
        continue;

      changes[p]++;

      if (new_value & mask)
        rising[p]++;
      else
        falling[p]++;
    }
  }

  // ---------------------------------------------------------------------------
  // Grunddaten
  // ---------------------------------------------------------------------------

  ESP_LOGI(TAG, "------------------------------");

  ESP_LOGI(
      TAG,
      "changes: GPIO38=%lu GPIO39=%lu GPIO40=%lu",
      (unsigned long) changes[0],
      (unsigned long) changes[1],
      (unsigned long) changes[2]);

  ESP_LOGI(
      TAG,
      "rising : GPIO38=%lu GPIO39=%lu GPIO40=%lu",
      (unsigned long) rising[0],
      (unsigned long) rising[1],
      (unsigned long) rising[2]);

  ESP_LOGI(
      TAG,
      "falling: GPIO38=%lu GPIO39=%lu GPIO40=%lu",
      (unsigned long) falling[0],
      (unsigned long) falling[1],
      (unsigned long) falling[2]);

  if (min_delta != UINT32_MAX) {

    ESP_LOGI(
        TAG,
        "min edge spacing: %lu us",
        (unsigned long) min_delta);

    if (min_delta > 0) {

      ESP_LOGI(
          TAG,
          "max edge rate: %.1f kHz",
          1000.0f / (float) min_delta);
    }
  }

  ESP_LOGI(
      TAG,
      "longest interval: %lu us",
      (unsigned long) max_delta);

  // ---------------------------------------------------------------------------
  // Versuch einer I2C-Erkennung
  //
  // Typisches I2C:
  //
  // SDA und SCL sind beide normalerweise HIGH.
  //
  // Während SCL HIGH ist:
  //   SDA LOW -> START
  //   SDA HIGH -> STOP
  //
  // Datenänderungen auf SDA sollten normalerweise bei SCL LOW stattfinden.
  // ---------------------------------------------------------------------------

  int best_sda = -1;
  int best_scl = -1;

  uint32_t best_score = 0;

  for (int sda = 0; sda < 3; sda++) {

    for (int scl = 0; scl < 3; scl++) {

      if (sda == scl)
        continue;

      uint32_t score = 0;

      for (uint16_t i = 1; i < count; i++) {

        const uint8_t old_v =
            data[i - 1].value;

        const uint8_t new_v =
            data[i].value;

        const uint8_t sda_mask =
            (1 << sda);

        const uint8_t scl_mask =
            (1 << scl);

        const bool sda_changed =
            ((old_v ^ new_v) & sda_mask) != 0;

        const bool scl_high =
            (new_v & scl_mask) != 0;

        // SDA darf bei I2C insbesondere dann nicht beliebig wechseln,
        // wenn SCL HIGH ist.
        if (sda_changed && scl_high)
          score += 4;

        // Eine SCL-Flanke ist ebenfalls interessant.
        if ((old_v ^ new_v) & scl_mask)
          score += 1;
      }

      if (score > best_score) {
        best_score = score;
        best_sda = sda;
        best_scl = scl;
      }
    }
  }

  if (best_sda >= 0 && best_scl >= 0) {

    ESP_LOGI(
        TAG,
        "possible I2C pair: SDA=%s SCL=%s score=%lu",
        pin_name(best_sda),
        pin_name(best_scl),
        (unsigned long) best_score);

    const int third =
        3 - best_sda - best_scl;

    ESP_LOGI(
        TAG,
        "third pin: %s",
        pin_name(third));
  }

  // ---------------------------------------------------------------------------
  // Zustandsstatistik
  // ---------------------------------------------------------------------------

  uint32_t state_count[8] = {
    0, 0, 0, 0,
    0, 0, 0, 0
  };

  for (uint16_t i = 0; i < count; i++)
    state_count[data[i].value & 7]++;

  ESP_LOGI(TAG, "state distribution:");

  for (int v = 0; v < 8; v++) {

    if (state_count[v] == 0)
      continue;

    ESP_LOGI(
        TAG,
        "  %u%u%u : %lu",
        (v >> 0) & 1,
        (v >> 1) & 1,
        (v >> 2) & 1,
        (unsigned long) state_count[v]);
  }

  // ---------------------------------------------------------------------------
  // Daten ausgeben
  // ---------------------------------------------------------------------------

  ESP_LOGI(TAG, "DATA:");

  const uint32_t base =
      data[0].t;

  for (uint16_t i = 0; i < count; i++) {

    const uint32_t rel =
        data[i].t - base;

    ESP_LOGI(
        TAG,
        "%lu us %u%u%u",
        (unsigned long) rel,
        (data[i].value >> 0) & 1,
        (data[i].value >> 1) & 1,
        (data[i].value >> 2) & 1);
  }
}

// -----------------------------------------------------------------------------
// Setup
// -----------------------------------------------------------------------------

void BusSniffer::setup()
{
  instance = this;

  gpio_config_t io = {};

  io.mode =
      GPIO_MODE_INPUT;

  io.pull_up_en =
      GPIO_PULLUP_DISABLE;

  io.pull_down_en =
      GPIO_PULLDOWN_DISABLE;

  io.intr_type =
      GPIO_INTR_ANYEDGE;

  io.pin_bit_mask =
      (1ULL << PIN_A) |
      (1ULL << PIN_B) |
      (1ULL << PIN_C);

  gpio_config(&io);

  // ISR service installieren.
  esp_err_t err =
      gpio_install_isr_service(0);

  if (err != ESP_OK &&
      err != ESP_ERR_INVALID_STATE) {

    ESP_LOGE(
        TAG,
        "gpio_install_isr_service failed: %d",
        (int) err);

    return;
  }

  err = gpio_isr_handler_add(
      PIN_A,
      gpio_isr,
      nullptr);

  if (err != ESP_OK) {
    ESP_LOGE(
        TAG,
        "ISR GPIO38 failed: %d",
        (int) err);
  }

  err = gpio_isr_handler_add(
      PIN_B,
      gpio_isr,
      nullptr);

  if (err != ESP_OK) {
    ESP_LOGE(
        TAG,
        "ISR GPIO39 failed: %d",
        (int) err);
  }

  err = gpio_isr_handler_add(
      PIN_C,
      gpio_isr,
      nullptr);

  if (err != ESP_OK) {
    ESP_LOGE(
        TAG,
        "ISR GPIO40 failed: %d",
        (int) err);
  }

  // Anfangszustand erfassen.
  last_value =
      read_bus();

  last_edge =
      (uint32_t) esp_timer_get_time();

  sample_count = 0;

  ESP_LOGI(
      TAG,
      "Bus sniffer started");

  ESP_LOGI(
      TAG,
      "Monitoring GPIO38, GPIO39, GPIO40");
}

// -----------------------------------------------------------------------------
// Main loop
// -----------------------------------------------------------------------------

void BusSniffer::loop()
{
  if (sample_count == 0)
    return;

  const uint64_t now =
      esp_timer_get_time();

  const uint32_t last =
      last_edge;

  // Noch Aktivität auf dem Bus.
  if ((uint32_t) now - last <
      FRAME_TIMEOUT_US)
    return;

  // ---------------------------------------------------------------------------
  // Capture atomar übernehmen.
  //
  // WICHTIG:
  // Keine Zuweisung
  //
  //   local[i] = samples[i]
  //
  // da samples[] volatile ist.
  //
  // Stattdessen werden die einzelnen Member explizit kopiert.
  // ---------------------------------------------------------------------------

  portDISABLE_INTERRUPTS();

  uint16_t count =
      sample_count;

  if (count > MAX_SAMPLES)
    count = MAX_SAMPLES;

  // Statische Variable statt 6 kB auf dem Stack.
  static Sample local[MAX_SAMPLES];

  for (uint16_t i = 0; i < count; i++) {

    local[i].t =
        samples[i].t;

    local[i].value =
        samples[i].value;
  }

  sample_count = 0;

  portENABLE_INTERRUPTS();

  // ---------------------------------------------------------------------------
  // Frame auswerten
  // ---------------------------------------------------------------------------

  ESP_LOGI(
      TAG,
      "==============================");

  ESP_LOGI(
      TAG,
      "Captured %u samples",
      count);

  if (count >= MAX_SAMPLES) {

    ESP_LOGW(
        TAG,
        "Buffer overflow - frame truncated");
  }

  if (count >= 2)
    analyze_frame(local, count);
  else if (count == 1) {

    ESP_LOGI(
        TAG,
        "Only one sample captured");
  }

  ESP_LOGI(
      TAG,
      "==============================");
}

}  // namespace bus_sniffer
}  // namespace esphome

