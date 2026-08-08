#include "bus_sniffer.h"

#include "esphome/core/log.h"

#include "driver/gpio.h"
#include "esp_cpu.h"
#include "esp_clk.h"

namespace esphome {
namespace bus_sniffer {

static const char *TAG = "BUS";

// ============================================================
// GPIOs
// ============================================================

static constexpr gpio_num_t PINS[3] = {
    GPIO_NUM_38,
    GPIO_NUM_39,
    GPIO_NUM_40
};

static constexpr int MAX_SAMPLES = 4096;

// Nach dieser Zeit ohne Flanke gilt eine Aufzeichnung als Frame.
// 50 ms ist für I2C mehr als ausreichend.
static constexpr uint32_t FRAME_TIMEOUT_US = 50000;

// ============================================================
// Sample
// ============================================================

struct Sample {
  uint32_t cycles;
  uint8_t value;
};

volatile Sample samples[MAX_SAMPLES];

volatile uint16_t sample_count = 0;
volatile bool overflow = false;

// Letzter Zustand der drei GPIOs.
// Wird ebenfalls ausschließlich in der ISR verwendet.
volatile uint8_t isr_last_value = 0xff;

// Zeitpunkt der letzten Flanke.
volatile uint32_t isr_last_cycles = 0;

// ============================================================
// Hilfsfunktionen
// ============================================================

static inline uint32_t cycles_to_us(uint32_t cycles) {
  const uint32_t cpu_hz = esp_clk_cpu_freq();

  if (cpu_hz == 0)
    return 0;

  return (uint32_t)(
      ((uint64_t)cycles * 1000000ULL) / cpu_hz
  );
}

static inline uint32_t cycles_delta_to_us(
    uint32_t a,
    uint32_t b) {

  return cycles_to_us(a - b);
}


// ============================================================
// GPIO ISR
// ============================================================
//
// Wichtig:
// Hier NICHT:
//   - ESP_LOG*
//   - esp_timer_get_time()
//   - komplexe Berechnungen
//   - Speicheroperationen
//
// Nur GPIO lesen und Sample ablegen.
//

static void IRAM_ATTR gpio_isr(void *arg) {

  const uint32_t cycles = esp_cpu_get_cycle_count();

  uint8_t value = 0;

  if (gpio_get_level(PINS[0]))
    value |= 1;

  if (gpio_get_level(PINS[1]))
    value |= 2;

  if (gpio_get_level(PINS[2]))
    value |= 4;

  // Nur echte Zustandsänderungen speichern.
  if (value == isr_last_value)
    return;

  isr_last_value = value;
  isr_last_cycles = cycles;

  const uint16_t index = sample_count;

  if (index < MAX_SAMPLES) {

    samples[index].cycles = cycles;
    samples[index].value = value;

    sample_count = index + 1;

  } else {

    overflow = true;
  }
}


// ============================================================
// Setup
// ============================================================

void BusSniffer::setup() {

  gpio_config_t io = {};

  io.mode = GPIO_MODE_INPUT;

  // Keine internen Pullups/Pulldowns.
  //
  // Die von dir gefundenen 10k Pullups sind vermutlich
  // bereits die I2C Pullups.
  //
  io.pull_up_en = GPIO_PULLUP_DISABLE;
  io.pull_down_en = GPIO_PULLDOWN_DISABLE;

  io.intr_type = GPIO_INTR_ANYEDGE;

  io.pin_bit_mask =
      (1ULL << PINS[0]) |
      (1ULL << PINS[1]) |
      (1ULL << PINS[2]);

  gpio_config(&io);

  // Aktuellen Zustand einmal bestimmen.
  uint8_t initial = 0;

  if (gpio_get_level(PINS[0]))
    initial |= 1;

  if (gpio_get_level(PINS[1]))
    initial |= 2;

  if (gpio_get_level(PINS[2]))
    initial |= 4;

  isr_last_value = initial;
  isr_last_cycles = esp_cpu_get_cycle_count();

  esp_err_t err = gpio_install_isr_service(
      ESP_INTR_FLAG_IRAM
  );

  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
    ESP_LOGE(TAG,
             "gpio_install_isr_service failed: %s",
             esp_err_to_name(err));
    return;
  }

  for (int i = 0; i < 3; i++) {

    err = gpio_isr_handler_add(
        PINS[i],
        gpio_isr,
        nullptr
    );

    if (err != ESP_OK) {

      ESP_LOGE(
          TAG,
          "gpio_isr_handler_add GPIO%d failed: %s",
          PINS[i],
          esp_err_to_name(err)
      );

      return;
    }
  }

  ESP_LOGI(TAG, "================================");
  ESP_LOGI(TAG, "3-wire bus sniffer started");
  ESP_LOGI(TAG, "GPIOs: %d / %d / %d",
           PINS[0],
           PINS[1],
           PINS[2]);

  ESP_LOGI(TAG, "Initial state: %u%u%u",
           (initial >> 0) & 1,
           (initial >> 1) & 1,
           (initial >> 2) & 1);

  ESP_LOGI(TAG, "================================");
}


// ============================================================
// I2C Analyse
// ============================================================

struct I2CAnalysis {

  int sda;
  int scl;
  int other;

  uint32_t scl_rising;
  uint32_t scl_falling;

  uint32_t sda_changes;
  uint32_t sda_changes_while_scl_high;

  uint32_t starts;
  uint32_t stops;

  uint32_t bytes;
  uint32_t ack;
  uint32_t nack;

  uint32_t scl_min_us;
  uint32_t scl_max_us;

  float scl_frequency;
};


// ============================================================
// Analyse einer möglichen SDA/SCL-Kombination
// ============================================================

static I2CAnalysis analyze_i2c(
    const Sample *data,
    uint16_t count,
    int sda_pin,
    int scl_pin) {

  I2CAnalysis result = {};

  result.sda = sda_pin;
  result.scl = scl_pin;
  result.other = 3 - sda_pin - scl_pin;

  result.scl_min_us = UINT32_MAX;

  bool prev_sda =
      (data[0].value >> sda_pin) & 1;

  bool prev_scl =
      (data[0].value >> scl_pin) & 1;

  // ----------------------------------------------------------
  // Rohflanken analysieren
  // ----------------------------------------------------------

  for (uint16_t i = 1; i < count; i++) {

    bool sda =
        (data[i].value >> sda_pin) & 1;

    bool scl =
        (data[i].value >> scl_pin) & 1;

    uint32_t dt =
        cycles_delta_to_us(
            data[i].cycles,
            data[i - 1].cycles
        );

    // SDA geändert
    if (sda != prev_sda) {

      result.sda_changes++;

      // SDA-Änderung während SCL HIGH:
      // bei I2C normalerweise START oder STOP.
      if (scl)
        result.sda_changes_while_scl_high++;

      // START:
      // SDA HIGH -> LOW bei SCL HIGH
      if (prev_sda && !sda && scl)
        result.starts++;

      // STOP:
      // SDA LOW -> HIGH bei SCL HIGH
      if (!prev_sda && sda && scl)
        result.stops++;
    }

    // SCL rising edge
    if (!prev_scl && scl) {

      result.scl_rising++;

      if (dt > 0 && dt < result.scl_min_us)
        result.scl_min_us = dt;

      if (dt > result.scl_max_us)
        result.scl_max_us = dt;
    }

    if (prev_scl && !scl)
      result.scl_falling++;

    prev_sda = sda;
    prev_scl = scl;
  }

  // ----------------------------------------------------------
  // Frequenz grob bestimmen
  //
  // Wir verwenden den mittleren Abstand zwischen SCL-Flanken.
  // ----------------------------------------------------------

  if (result.scl_rising >= 2) {

    uint64_t total_us = 0;
    uint32_t edges = 0;

    for (uint16_t i = 1; i < count; i++) {

      bool old_scl =
          (data[i - 1].value >> scl_pin) & 1;

      bool new_scl =
          (data[i].value >> scl_pin) & 1;

      if (!old_scl && new_scl) {

        if (edges > 0) {

          // vorherigen Zeitpunkt suchen
          // (für diese grobe Analyse reicht der
          // Abstand zwischen aufeinanderfolgenden
          // Samples mit SCL rising nicht direkt;
          // deshalb hier nur späterer Decoder).
        }

        edges++;
      }
    }
  }

  // ----------------------------------------------------------
  // Tatsächliches I2C Bit-/Byte-Decoding
  // ----------------------------------------------------------
  //
  // Datenbit wird bei SCL rising gelesen.
  //
  // 8 Bits:
  //
  //   bit7 ... bit0
  //
  // danach:
  //
  //   ACK/NACK
  //

  uint8_t current_byte = 0;
  int bit_count = 0;

  for (uint16_t i = 1; i < count; i++) {

    bool old_sda =
        (data[i - 1].value >> sda_pin) & 1;

    bool new_sda =
        (data[i].value >> sda_pin) & 1;

    bool old_scl =
        (data[i - 1].value >> scl_pin) & 1;

    bool new_scl =
        (data[i].value >> scl_pin) & 1;

    // START
    if (old_scl && new_scl &&
        old_sda && !new_sda) {

      current_byte = 0;
      bit_count = 0;
      continue;
    }

    // STOP
    if (old_scl && new_scl &&
        !old_sda && new_sda) {

      current_byte = 0;
      bit_count = 0;
      continue;
    }

    // Nur rising SCL auswerten
    if (!old_scl && new_scl) {

      if (bit_count < 8) {

        current_byte <<= 1;

        if (new_sda)
          current_byte |= 1;

        bit_count++;

      } else {

        // 9. Clock = ACK/NACK

        if (!new_sda)
          result.ack++;
        else
          result.nack++;

        result.bytes++;

        ESP_LOGD(
            TAG,
            "I2C candidate SDA=%d SCL=%d BYTE=0x%02X %s",
            PINS[sda_pin],
            PINS[scl_pin],
            current_byte,
            new_sda ? "NACK" : "ACK"
        );

        current_byte = 0;
        bit_count = 0;
      }
    }
  }

  return result;
}


// ============================================================
// Dritten Pin analysieren
// ============================================================

static void analyze_third_pin(
    const Sample *data,
    uint16_t count,
    int pin,
    int sda,
    int scl) {

  uint32_t rising = 0;
  uint32_t falling = 0;

  uint32_t min_us = UINT32_MAX;
  uint32_t max_us = 0;

  uint64_t high_time = 0;
  uint64_t low_time = 0;

  bool previous =
      (data[0].value >> pin) & 1;

  uint32_t last_transition =
      data[0].cycles;

  for (uint16_t i = 1; i < count; i++) {

    bool current =
        (data[i].value >> pin) & 1;

    if (current == previous)
      continue;

    uint32_t dt =
        cycles_delta_to_us(
            data[i].cycles,
            last_transition
        );

    if (dt < min_us)
      min_us = dt;

    if (dt > max_us)
      max_us = dt;

    if (current) {
      rising++;
      low_time += dt;
    } else {
      falling++;
      high_time += dt;
    }

    previous = current;
    last_transition = data[i].cycles;
  }

  ESP_LOGI(TAG, "--------------------------------");
  ESP_LOGI(TAG,
           "Third pin GPIO%d analysis",
           PINS[pin]);

  ESP_LOGI(TAG,
           "relative to candidate SDA=GPIO%d SCL=GPIO%d",
           PINS[sda],
           PINS[scl]);

  ESP_LOGI(TAG,
           "rising=%lu falling=%lu",
           (unsigned long)rising,
           (unsigned long)falling);

  if (rising || falling) {

    ESP_LOGI(TAG,
             "transition spacing min=%lu us max=%lu us",
             (unsigned long)min_us,
             (unsigned long)max_us);

    ESP_LOGI(TAG,
             "high-time=%llu us low-time=%llu us",
             (unsigned long long)high_time,
             (unsigned long long)low_time);
  }

  // ----------------------------------------------------------
  // Klassifikation
  // ----------------------------------------------------------

  if (rising == 0 && falling == 0) {

    ESP_LOGI(TAG,
             "classification: STATIC");

  } else if (
      rising <= 3 &&
      falling <= 3) {

    ESP_LOGI(TAG,
             "classification: SLOW / STATUS / INTERRUPT");

  } else {

    ESP_LOGI(TAG,
             "classification: ACTIVE SIGNAL");
  }
}


// ============================================================
// Frame auswerten
// ============================================================

void BusSniffer::loop() {

  if (sample_count == 0)
    return;

  // Snapshot der ISR-Daten.
  //
  // Wir verwenden einen kurzen kritischen Abschnitt.
  // Keine globalen Interrupts über die anschließende
  // Analyse hinweg deaktivieren.

  uint16_t count;

  bool was_overflow;

  portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

  portENTER_CRITICAL(&mux);

  count = sample_count;

  if (count > MAX_SAMPLES)
    count = MAX_SAMPLES;

  was_overflow = overflow;

  portEXIT_CRITICAL(&mux);

  if (count == 0)
    return;

  // ----------------------------------------------------------
  // Prüfen, ob der Frame fertig ist.
  // ----------------------------------------------------------

  uint32_t now_cycles =
      esp_cpu_get_cycle_count();

  uint32_t last_cycles =
      samples[count - 1].cycles;

  uint32_t elapsed_us =
      cycles_delta_to_us(
          now_cycles,
          last_cycles
      );

  if (elapsed_us < FRAME_TIMEOUT_US)
    return;

  // ----------------------------------------------------------
  // Jetzt Snapshot tatsächlich übernehmen.
  // ----------------------------------------------------------

  Sample local[MAX_SAMPLES];

  portENTER_CRITICAL(&mux);

  count = sample_count;

  if (count > MAX_SAMPLES)
    count = MAX_SAMPLES;

  for (uint16_t i = 0; i < count; i++) {
    local[i] = samples[i];
  }

  sample_count = 0;
  overflow = false;

  portEXIT_CRITICAL(&mux);

  if (count == 0)
    return;

  // ----------------------------------------------------------
  // Header
  // ----------------------------------------------------------

  ESP_LOGI(TAG, "");
  ESP_LOGI(TAG, "================================");
  ESP_LOGI(TAG, "Captured %u samples", count);

  if (was_overflow) {
    ESP_LOGW(TAG,
             "BUFFER OVERFLOW - frame truncated");
  }

  // ----------------------------------------------------------
  // Initialzustand
  // ----------------------------------------------------------

  ESP_LOGI(TAG,
           "Initial state GPIO%d/%d/%d = %u%u%u",
           PINS[0],
           PINS[1],
           PINS[2],
           (local[0].value >> 0) & 1,
           (local[0].value >> 1) & 1,
           (local[0].value >> 2) & 1);

  // ----------------------------------------------------------
  // Rohstatistik
  // ----------------------------------------------------------

  uint32_t changes[3] = {0, 0, 0};

  uint32_t min_delta = UINT32_MAX;
  uint32_t max_delta = 0;

  for (uint16_t i = 1; i < count; i++) {

    uint32_t dt =
        cycles_delta_to_us(
            local[i].cycles,
            local[i - 1].cycles
        );

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

  ESP_LOGI(TAG,
           "changes GPIO%d=%lu GPIO%d=%lu GPIO%d=%lu",
           PINS[0],
           (unsigned long)changes[0],
           PINS[1],
           (unsigned long)changes[1],
           PINS[2],
           (unsigned long)changes[2]);

  if (count > 1) {

    ESP_LOGI(TAG,
             "sample spacing min=%lu us max=%lu us",
             (unsigned long)min_delta,
             (unsigned long)max_delta);
  }

  // ----------------------------------------------------------
  // Alle sechs SDA/SCL Kombinationen testen.
  //
  // 38->39
  // 38->40
  // 39->38
  // 39->40
  // 40->38
  // 40->39
  // ----------------------------------------------------------

  ESP_LOGI(TAG, "");
  ESP_LOGI(TAG, "I2C candidate analysis:");

  I2CAnalysis best = {};
  int best_score = -100000;

  for (int sda = 0; sda < 3; sda++) {

    for (int scl = 0; scl < 3; scl++) {

      if (sda == scl)
        continue;

      I2CAnalysis a =
          analyze_i2c(
              local,
              count,
              sda,
              scl
          );

      // ------------------------------------------------------
      // Heuristik:
      //
      // I2C:
      // - SCL hat viele rising edges
      // - ungefähr gleich viele falling edges
      // - SDA ändert sich hauptsächlich bei SCL LOW
      // - START/STOP bei SCL HIGH
      // - Anzahl ACK/NACK sollte zu Bytes passen
      // ------------------------------------------------------

      int score = 0;

      score +=
          (int)a.scl_rising * 2;

      score +=
          (int)a.bytes * 20;

      score +=
          (int)a.ack * 10;

      score -=
          (int)a.nack * 2;

      score +=
          (int)a.starts * 10;

      score +=
          (int)a.stops * 10;

      // SDA sollte während SCL HIGH möglichst
      // stabil sein.
      if (a.sda_changes > 0) {

        uint32_t abnormal =
            a.sda_changes_while_scl_high;

        if (abnormal >
            a.starts + a.stops) {

          score -=
              (int)(
                  abnormal -
                  a.starts -
                  a.stops
              ) * 5;
        }
      }

      ESP_LOGI(
          TAG,
          "SDA=%d SCL=%d OTHER=%d "
          "SCL↑=%lu SCL↓=%lu "
          "SDAchg=%lu "
          "START=%lu STOP=%lu "
          "bytes=%lu ACK=%lu NACK=%lu "
          "score=%d",
          PINS[sda],
          PINS[scl],
          PINS[a.other],

          (unsigned long)a.scl_rising,
          (unsigned long)a.scl_falling,

          (unsigned long)a.sda_changes,

          (unsigned long)a.starts,
          (unsigned long)a.stops,

          (unsigned long)a.bytes,
          (unsigned long)a.ack,
          (unsigned long)a.nack,

          score
      );

      if (score > best_score) {

        best_score = score;
        best = a;
      }
    }
  }

  // ----------------------------------------------------------
  // Best candidate
  // ----------------------------------------------------------

  ESP_LOGI(TAG, "");
  ESP_LOGI(TAG, "BEST I2C CANDIDATE:");
  ESP_LOGI(TAG,
           "SDA = GPIO%d",
           PINS[best.sda]);

  ESP_LOGI(TAG,
           "SCL = GPIO%d",
           PINS[best.scl]);

  ESP_LOGI(TAG,
           "OTHER = GPIO%d",
           PINS[best.other]);

  ESP_LOGI(TAG,
           "START=%lu STOP=%lu bytes=%lu ACK=%lu NACK=%lu",
           (unsigned long)best.starts,
           (unsigned long)best.stops,
           (unsigned long)best.bytes,
           (unsigned long)best.ack,
           (unsigned long)best.nack);

  // ----------------------------------------------------------
  // Dritten Pin analysieren
  // ----------------------------------------------------------

  analyze_third_pin(
      local,
      count,
      best.other,
      best.sda,
      best.scl
  );

  // ----------------------------------------------------------
  // Rohdaten ausgeben
  //
  // Begrenzen, damit eine lange Messung nicht wieder
  // den ESPHome-Loop mit tausenden Logzeilen blockiert.
  // ----------------------------------------------------------

  ESP_LOGI(TAG, "");
  ESP_LOGI(TAG, "DATA:");

  uint32_t base =
      local[0].cycles;

  const uint16_t MAX_PRINT = 300;

  uint16_t print_count =
      count < MAX_PRINT ? count : MAX_PRINT;

  for (uint16_t i = 0;
       i < print_count;
       i++) {

    uint32_t rel =
        cycles_delta_to_us(
            local[i].cycles,
            base
        );

    ESP_LOGI(
        TAG,
        "%lu us %u%u%u",
        (unsigned long)rel,
        (local[i].value >> 0) & 1,
        (local[i].value >> 1) & 1,
        (local[i].value >> 2) & 1
    );
  }

  if (count > MAX_PRINT) {

    ESP_LOGI(
        TAG,
        "... %u samples not printed",
        count - MAX_PRINT
    );
  }

  ESP_LOGI(TAG, "================================");
}

}  // namespace bus_sniffer
}  // namespace esphome
