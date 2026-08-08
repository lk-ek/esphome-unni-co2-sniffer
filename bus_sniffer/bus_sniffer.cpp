#include "bus_sniffer.h"

#include "esphome/core/log.h"

#include "driver/gpio.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

namespace esphome {
namespace bus_sniffer {

static const char *TAG = "BUS";

/*
 * I2C:
 *
 * GPIO39 = SCL
 * GPIO40 = SDA
 *
 * GPIO38 = zusätzliches Signal, wird nur mitgesampelt.
 */
static constexpr gpio_num_t PIN_SCL = GPIO_NUM_39;
static constexpr gpio_num_t PIN_SDA = GPIO_NUM_40;
static constexpr gpio_num_t PIN_OTHER = GPIO_NUM_38;

static constexpr int MAX_SAMPLES = 4096;

/*
 * Ein Capture wird beendet, wenn für diese Zeit keine
 * SCL/SDA-Flanke mehr gekommen ist.
 */
static constexpr uint32_t FRAME_TIMEOUT_US = 5000;

/*
 * Sehr kurze Glitches ignorieren.
 * 1 us ist hier absichtlich relativ konservativ.
 */
static constexpr uint32_t GLITCH_FILTER_US = 1;

struct Sample {
  uint32_t t;
  uint8_t value;
};

/*
 * Bits:
 *
 * bit 0 = SCL
 * bit 1 = SDA
 * bit 2 = GPIO38
 */
volatile Sample samples[MAX_SAMPLES];
volatile uint16_t sample_count = 0;

volatile uint32_t last_edge = 0;
volatile uint8_t last_value = 0xff;

static uint8_t read_bus()
{
  uint8_t v = 0;

  if (gpio_get_level(PIN_SCL))
    v |= 1;

  if (gpio_get_level(PIN_SDA))
    v |= 2;

  if (gpio_get_level(PIN_OTHER))
    v |= 4;

  return v;
}


/*
 * ISR
 *
 * Nur SCL und SDA lösen Interrupts aus.
 * GPIO38 wird lediglich zusammen mit den beiden Busleitungen
 * abgetastet.
 */
static void IRAM_ATTR gpio_isr(void *arg)
{
  uint32_t now = (uint32_t) esp_timer_get_time();

  uint8_t v = read_bus();

  /*
   * Gleichen Zustand nicht zweimal speichern.
   */
  if (v == last_value)
    return;

  /*
   * Sehr kurze Glitches ignorieren.
   */
  if ((uint32_t)(now - last_edge) < GLITCH_FILTER_US)
    return;

  last_edge = now;
  last_value = v;

  if (sample_count < MAX_SAMPLES) {
    samples[sample_count].t = now;
    samples[sample_count].value = v;
    sample_count++;
  }
}


/*
 * Hilfsfunktionen
 */

static bool scl(const Sample &s)
{
  return (s.value & 1) != 0;
}

static bool sda(const Sample &s)
{
  return (s.value & 2) != 0;
}

static bool other(const Sample &s)
{
  return (s.value & 4) != 0;
}


/*
 * I2C Decoder
 */
struct I2CStats {
  uint32_t starts = 0;
  uint32_t stops = 0;

  uint32_t bytes = 0;
  uint32_t ack = 0;
  uint32_t nack = 0;

  uint32_t address_count = 0;

  uint8_t current_byte = 0;
  int bit_count = 0;

  bool active = false;
  bool have_previous = false;

  uint8_t previous_value = 0;
};


static void log_byte(uint8_t byte, bool address, bool read)
{
  if (address) {
    uint8_t addr = byte >> 1;

    ESP_LOGI(
        TAG,
        "  ADDRESS: 0x%02X (%u) %s",
        addr,
        addr,
        read ? "READ" : "WRITE");

  } else {

    char ascii = '.';

    if (byte >= 32 && byte <= 126)
      ascii = (char) byte;

    ESP_LOGI(
        TAG,
        "  DATA:    0x%02X (%3u) '%c'",
        byte,
        byte,
        ascii);
  }
}


/*
 * Decodiert einen einzelnen Capture.
 */
static void decode_i2c(
    const Sample *data,
    uint16_t count)
{
  if (count < 2)
    return;

  ESP_LOGI(TAG, "I2C DECODE");
  ESP_LOGI(TAG, "------------------------------");

  I2CStats st;

  uint32_t base = data[0].t;

  bool in_transaction = false;

  /*
   * Nach START ist das nächste Byte immer die Adresse.
   */
  bool first_byte = true;

  /*
   * Datenbitzähler:
   *
   * 0..7 = Datenbits
   * 8    = ACK/NACK
   */
  int bit_count = 0;
  uint8_t current_byte = 0;

  uint32_t last_scl_rise = 0;

  for (uint16_t i = 1; i < count; i++) {

    const Sample &prev = data[i - 1];
    const Sample &cur = data[i];

    bool prev_scl = scl(prev);
    bool prev_sda = sda(prev);

    bool cur_scl = scl(cur);
    bool cur_sda = sda(cur);

    uint32_t rel =
        (uint32_t)(cur.t - base);

    /*
     * ----------------------------------------
     * START
     * SDA: HIGH -> LOW
     * SCL bleibt HIGH
     * ----------------------------------------
     */
    if (prev_sda && !cur_sda &&
        prev_scl && cur_scl) {

      st.starts++;

      ESP_LOGI(
          TAG,
          "%lu us: START",
          (unsigned long) rel);

      in_transaction = true;
      first_byte = true;

      bit_count = 0;
      current_byte = 0;

      continue;
    }

    /*
     * ----------------------------------------
     * STOP
     * SDA: LOW -> HIGH
     * SCL bleibt HIGH
     * ----------------------------------------
     */
    if (!prev_sda && cur_sda &&
        prev_scl && cur_scl) {

      st.stops++;

      ESP_LOGI(
          TAG,
          "%lu us: STOP",
          (unsigned long) rel);

      in_transaction = false;

      bit_count = 0;
      current_byte = 0;

      continue;
    }

    /*
     * Nur innerhalb einer Transaktion Bytes dekodieren.
     */
    if (!in_transaction)
      continue;

    /*
     * ----------------------------------------
     * SCL rising edge
     *
     * SDA wird bei HIGH von SCL ausgewertet.
     * ----------------------------------------
     */
    if (!prev_scl && cur_scl) {

      /*
       * Erste steigende Flanke nach START:
       * Datenbit.
       */

      if (bit_count < 8) {

        current_byte <<= 1;

        if (cur_sda)
          current_byte |= 1;

        bit_count++;

        last_scl_rise = cur.t;

      } else {

        /*
         * 9. Clock:
         * ACK = SDA LOW
         * NACK = SDA HIGH
         */

        if (cur_sda) {
          st.nack++;

          ESP_LOGI(
              TAG,
              "  NACK");
        } else {
          st.ack++;

          ESP_LOGI(
              TAG,
              "  ACK");
        }

        st.bytes++;

        log_byte(
            current_byte,
            first_byte,
            first_byte ? ((current_byte & 1) != 0) : false);

        first_byte = false;

        bit_count = 0;
        current_byte = 0;
      }
    }
  }

  /*
   * Zusammenfassung
   */
  ESP_LOGI(TAG, "------------------------------");

  ESP_LOGI(
      TAG,
      "START=%lu STOP=%lu",
      (unsigned long) st.starts,
      (unsigned long) st.stops);

  ESP_LOGI(
      TAG,
      "BYTES=%lu ACK=%lu NACK=%lu",
      (unsigned long) st.bytes,
      (unsigned long) st.ack,
      (unsigned long) st.nack);

  ESP_LOGI(TAG, "------------------------------");
}


/*
 * Statistik über GPIO39/40
 */
static void analyze_bus(
    const Sample *data,
    uint16_t count)
{
  if (count < 2)
    return;

  uint32_t scl_rising = 0;
  uint32_t scl_falling = 0;

  uint32_t sda_rising = 0;
  uint32_t sda_falling = 0;

  uint32_t min_delta = UINT32_MAX;
  uint32_t max_delta = 0;

  uint32_t scl_min_delta = UINT32_MAX;
  uint32_t scl_max_delta = 0;

  uint32_t sda_min_delta = UINT32_MAX;
  uint32_t sda_max_delta = 0;

  uint32_t last_scl_edge = data[0].t;
  uint32_t last_sda_edge = data[0].t;

  uint32_t other_changes = 0;

  for (uint16_t i = 1; i < count; i++) {

    uint32_t dt =
        (uint32_t)(data[i].t - data[i - 1].t);

    if (dt < min_delta)
      min_delta = dt;

    if (dt > max_delta)
      max_delta = dt;

    bool pscl = scl(data[i - 1]);
    bool cscl = scl(data[i]);

    bool psda = sda(data[i - 1]);
    bool csda = sda(data[i]);

    bool pother = other(data[i - 1]);
    bool cother = other(data[i]);

    /*
     * SCL
     */
    if (!pscl && cscl) {

      scl_rising++;

      uint32_t d =
          (uint32_t)(data[i].t - last_scl_edge);

      if (d < scl_min_delta)
        scl_min_delta = d;

      if (d > scl_max_delta)
        scl_max_delta = d;

      last_scl_edge = data[i].t;
    }

    if (pscl && !cscl) {

      scl_falling++;

      uint32_t d =
          (uint32_t)(data[i].t - last_scl_edge);

      if (d < scl_min_delta)
        scl_min_delta = d;

      if (d > scl_max_delta)
        scl_max_delta = d;

      last_scl_edge = data[i].t;
    }

    /*
     * SDA
     */
    if (!psda && csda) {

      sda_rising++;

      uint32_t d =
          (uint32_t)(data[i].t - last_sda_edge);

      if (d < sda_min_delta)
        sda_min_delta = d;

      if (d > sda_max_delta)
        sda_max_delta = d;

      last_sda_edge = data[i].t;
    }

    if (psda && !csda) {

      sda_falling++;

      uint32_t d =
          (uint32_t)(data[i].t - last_sda_edge);

      if (d < sda_min_delta)
        sda_min_delta = d;

      if (d > sda_max_delta)
        sda_max_delta = d;

      last_sda_edge = data[i].t;
    }

    /*
     * GPIO38 separat zählen.
     */
    if (pother != cother)
      other_changes++;
  }

  ESP_LOGI(TAG, "BUS STATISTICS");
  ESP_LOGI(TAG, "------------------------------");

  ESP_LOGI(
      TAG,
      "SCL GPIO39: rising=%lu falling=%lu",
      (unsigned long) scl_rising,
      (unsigned long) scl_falling);

  ESP_LOGI(
      TAG,
      "SDA GPIO40: rising=%lu falling=%lu",
      (unsigned long) sda_rising,
      (unsigned long) sda_falling);

  if (scl_min_delta != UINT32_MAX) {

    ESP_LOGI(
        TAG,
        "SCL min interval: %lu us",
        (unsigned long) scl_min_delta);

    ESP_LOGI(
        TAG,
        "SCL max interval: %lu us",
        (unsigned long) scl_max_delta);

    if (scl_min_delta > 0) {

      ESP_LOGI(
          TAG,
          "SCL max edge rate: %.1f kHz",
          1000.0f / scl_min_delta);
    }
  }

  if (sda_min_delta != UINT32_MAX) {

    ESP_LOGI(
        TAG,
        "SDA min interval: %lu us",
        (unsigned long) sda_min_delta);

    ESP_LOGI(
        TAG,
        "SDA max interval: %lu us",
        (unsigned long) sda_max_delta);
  }

  ESP_LOGI(
      TAG,
      "GPIO38 changes: %lu",
      (unsigned long) other_changes);

  ESP_LOGI(TAG, "------------------------------");
}


/*
 * Rohdaten ausgeben.
 *
 * Bei großen Captures nur die ersten 200 Samples.
 */
static void dump_data(
    const Sample *data,
    uint16_t count)
{
  uint16_t limit = count;

  if (limit > 200)
    limit = 200;

  if (count > limit) {
    ESP_LOGI(
        TAG,
        "RAW DATA: first %u of %u samples",
        limit,
        count);
  } else {
    ESP_LOGI(
        TAG,
        "RAW DATA: %u samples",
        count);
  }

  uint32_t base = data[0].t;

  for (uint16_t i = 0; i < limit; i++) {

    uint32_t rel =
        (uint32_t)(data[i].t - base);

    ESP_LOGI(
        TAG,
        "%lu us SCL=%u SDA=%u GPIO38=%u",
        (unsigned long) rel,
        scl(data[i]) ? 1 : 0,
        sda(data[i]) ? 1 : 0,
        other(data[i]) ? 1 : 0);
  }
}


/*
 * Setup
 */
void BusSniffer::setup()
{
  /*
   * GPIOs als reine Eingänge.
   *
   * Keine internen Pullups aktivieren!
   * Die Platine hat bereits externe 10k Pullups.
   */
  gpio_config_t io = {};

  io.mode = GPIO_MODE_INPUT;

  io.pull_up_en = GPIO_PULLUP_DISABLE;
  io.pull_down_en = GPIO_PULLDOWN_DISABLE;

  /*
   * Wichtig:
   * Nur GPIO39 und GPIO40 lösen Interrupts aus.
   *
   * GPIO38 ist NICHT Bestandteil der Interrupt-Maske.
   */
  io.intr_type = GPIO_INTR_ANYEDGE;

  io.pin_bit_mask =
      (1ULL << PIN_SCL) |
      (1ULL << PIN_SDA);

  gpio_config(&io);

  /*
   * GPIO38 separat als Eingang konfigurieren.
   */
  gpio_config_t other_io = {};

  other_io.mode = GPIO_MODE_INPUT;
  other_io.pull_up_en = GPIO_PULLUP_DISABLE;
  other_io.pull_down_en = GPIO_PULLDOWN_DISABLE;
  other_io.intr_type = GPIO_INTR_DISABLE;
  other_io.pin_bit_mask = (1ULL << PIN_OTHER);

  gpio_config(&other_io);

  /*
   * ISR-Service.
   */
  esp_err_t err =
      gpio_install_isr_service(0);

  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {

    ESP_LOGE(
        TAG,
        "gpio_install_isr_service failed: %d",
        err);

    return;
  }

  err = gpio_isr_handler_add(
      PIN_SCL,
      gpio_isr,
      nullptr);

  if (err != ESP_OK) {

    ESP_LOGE(
        TAG,
        "ISR GPIO39 failed: %d",
        err);

    return;
  }

  err = gpio_isr_handler_add(
      PIN_SDA,
      gpio_isr,
      nullptr);

  if (err != ESP_OK) {

    ESP_LOGE(
        TAG,
        "ISR GPIO40 failed: %d",
        err);

    return;
  }

  /*
   * Anfangszustand synchronisieren.
   */
  last_value = read_bus();
  last_edge = (uint32_t) esp_timer_get_time();

  ESP_LOGI(TAG, "==============================");
  ESP_LOGI(TAG, "I2C sniffer started");
  ESP_LOGI(TAG, "SCL = GPIO39");
  ESP_LOGI(TAG, "SDA = GPIO40");
  ESP_LOGI(TAG, "OTHER = GPIO38");
  ESP_LOGI(TAG, "==============================");
}


/*
 * Loop
 */
void BusSniffer::loop()
{
  if (sample_count == 0)
    return;

  uint32_t now =
      (uint32_t) esp_timer_get_time();

  /*
   * Noch Aktivität?
   */
  if ((uint32_t)(now - last_edge) <
      FRAME_TIMEOUT_US)
    return;

  /*
   * Interrupts kurz sperren.
   */
  portDISABLE_INTERRUPTS();

  uint16_t count = sample_count;

  if (count > MAX_SAMPLES)
    count = MAX_SAMPLES;

  /*
   * Wichtig:
   *
   * volatile -> normal
   * deshalb NICHT:
   *
   * local[i] = samples[i];
   *
   * sondern jedes Feld einzeln kopieren.
   */
  Sample local[MAX_SAMPLES];

  for (uint16_t i = 0; i < count; i++) {

    local[i].t = samples[i].t;
    local[i].value = samples[i].value;
  }

  bool overflow =
      (sample_count >= MAX_SAMPLES);

  sample_count = 0;

  portENABLE_INTERRUPTS();

  if (count == 0)
    return;

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

  /*
   * Busstatistik
   */
  analyze_bus(
      local,
      count);

  /*
   * I2C dekodieren
   */
  decode_i2c(
      local,
      count);

  /*
   * Rohdaten nur bei überschaubaren Captures.
   *
   * Große Captures können den ESP-Logger massiv belasten.
   */
  if (count <= 300) {

    dump_data(
        local,
        count);

  } else {

    ESP_LOGI(
        TAG,
        "Raw data omitted (%u samples)",
        count);
  }

  ESP_LOGI(TAG, "==============================");
}

}  // namespace bus_sniffer
}  // namespace esphome
