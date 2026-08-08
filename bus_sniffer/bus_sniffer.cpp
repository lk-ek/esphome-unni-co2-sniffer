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
 * ============================================================
 * Pinbelegung
 * ============================================================
 *
 * Aufgrund der bisherigen Messungen:
 *
 * GPIO40 = CLOCK
 * GPIO39 = DATA
 * GPIO38 = FRAME / ENABLE / zusätzliches Signal
 *
 * GPIO39 und GPIO40 besitzen externe 10k Pullups.
 */
static constexpr gpio_num_t PIN_CLOCK = GPIO_NUM_40;
static constexpr gpio_num_t PIN_DATA  = GPIO_NUM_39;
static constexpr gpio_num_t PIN_FRAME = GPIO_NUM_38;

static constexpr int MAX_SAMPLES = 4096;

/*
 * Ein Capture wird beendet, wenn für diese Zeit keine
 * Flanke mehr auf CLOCK/DATA/FRAME gekommen ist.
 */
static constexpr uint32_t FRAME_TIMEOUT_US = 5000;

/*
 * Sehr kurze Glitches ignorieren.
 */
static constexpr uint32_t GLITCH_FILTER_US = 1;


/*
 * ============================================================
 * Sample
 * ============================================================
 *
 * bit 0 = CLOCK  (GPIO40)
 * bit 1 = DATA   (GPIO39)
 * bit 2 = FRAME  (GPIO38)
 */
struct Sample {
  uint32_t t;
  uint8_t value;
};

volatile Sample samples[MAX_SAMPLES];
volatile uint16_t sample_count = 0;

volatile uint32_t last_edge = 0;
volatile uint8_t last_value = 0xff;


/*
 * ============================================================
 * Bus lesen
 * ============================================================
 */
static uint8_t read_bus()
{
  uint8_t v = 0;

  if (gpio_get_level(PIN_CLOCK))
    v |= 1;

  if (gpio_get_level(PIN_DATA))
    v |= 2;

  if (gpio_get_level(PIN_FRAME))
    v |= 4;

  return v;
}


/*
 * ============================================================
 * ISR
 * ============================================================
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
   * Glitches ignorieren.
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
 * ============================================================
 * Hilfsfunktionen
 * ============================================================
 */

static bool clock_level(const Sample &s)
{
  return (s.value & 1) != 0;
}

static bool data_level(const Sample &s)
{
  return (s.value & 2) != 0;
}

static bool frame_level(const Sample &s)
{
  return (s.value & 4) != 0;
}


/*
 * ============================================================
 * Bus-Statistik
 * ============================================================
 */
static void analyze_bus(
    const Sample *data,
    uint16_t count)
{
  if (count < 2)
    return;

  uint32_t clock_rising = 0;
  uint32_t clock_falling = 0;

  uint32_t data_rising = 0;
  uint32_t data_falling = 0;

  uint32_t frame_rising = 0;
  uint32_t frame_falling = 0;

  uint32_t min_delta = UINT32_MAX;
  uint32_t max_delta = 0;

  uint32_t clock_min_delta = UINT32_MAX;
  uint32_t clock_max_delta = 0;

  uint32_t data_min_delta = UINT32_MAX;
  uint32_t data_max_delta = 0;

  uint32_t frame_min_delta = UINT32_MAX;
  uint32_t frame_max_delta = 0;

  uint32_t last_clock_edge = data[0].t;
  uint32_t last_data_edge = data[0].t;
  uint32_t last_frame_edge = data[0].t;

  for (uint16_t i = 1; i < count; i++) {

    uint32_t dt =
        (uint32_t)(data[i].t - data[i - 1].t);

    if (dt < min_delta)
      min_delta = dt;

    if (dt > max_delta)
      max_delta = dt;

    bool pclock = clock_level(data[i - 1]);
    bool cclock = clock_level(data[i]);

    bool pdata = data_level(data[i - 1]);
    bool cdata = data_level(data[i]);

    bool pframe = frame_level(data[i - 1]);
    bool cframe = frame_level(data[i]);

    /*
     * --------------------------------------------------------
     * CLOCK
     * --------------------------------------------------------
     */
    if (!pclock && cclock) {

      clock_rising++;

      uint32_t d =
          (uint32_t)(data[i].t - last_clock_edge);

      if (d < clock_min_delta)
        clock_min_delta = d;

      if (d > clock_max_delta)
        clock_max_delta = d;

      last_clock_edge = data[i].t;
    }

    if (pclock && !cclock) {

      clock_falling++;

      uint32_t d =
          (uint32_t)(data[i].t - last_clock_edge);

      if (d < clock_min_delta)
        clock_min_delta = d;

      if (d > clock_max_delta)
        clock_max_delta = d;

      last_clock_edge = data[i].t;
    }

    /*
     * --------------------------------------------------------
     * DATA
     * --------------------------------------------------------
     */
    if (!pdata && cdata) {

      data_rising++;

      uint32_t d =
          (uint32_t)(data[i].t - last_data_edge);

      if (d < data_min_delta)
        data_min_delta = d;

      if (d > data_max_delta)
        data_max_delta = d;

      last_data_edge = data[i].t;
    }

    if (pdata && !cdata) {

      data_falling++;

      uint32_t d =
          (uint32_t)(data[i].t - last_data_edge);

      if (d < data_min_delta)
        data_min_delta = d;

      if (d > data_max_delta)
        data_max_delta = d;

      last_data_edge = data[i].t;
    }

    /*
     * --------------------------------------------------------
     * FRAME / GPIO38
     * --------------------------------------------------------
     */
    if (!pframe && cframe) {

      frame_rising++;

      uint32_t d =
          (uint32_t)(data[i].t - last_frame_edge);

      if (d < frame_min_delta)
        frame_min_delta = d;

      if (d > frame_max_delta)
        frame_max_delta = d;

      last_frame_edge = data[i].t;
    }

    if (pframe && !cframe) {

      frame_falling++;

      uint32_t d =
          (uint32_t)(data[i].t - last_frame_edge);

      if (d < frame_min_delta)
        frame_min_delta = d;

      if (d > frame_max_delta)
        frame_max_delta = d;

      last_frame_edge = data[i].t;
    }
  }

  ESP_LOGI(TAG, "BUS STATISTICS");
  ESP_LOGI(TAG, "------------------------------");

  ESP_LOGI(
      TAG,
      "CLOCK GPIO40: rising=%lu falling=%lu",
      (unsigned long) clock_rising,
      (unsigned long) clock_falling);

  ESP_LOGI(
      TAG,
      "DATA  GPIO39: rising=%lu falling=%lu",
      (unsigned long) data_rising,
      (unsigned long) data_falling);

  ESP_LOGI(
      TAG,
      "FRAME GPIO38: rising=%lu falling=%lu",
      (unsigned long) frame_rising,
      (unsigned long) frame_falling);

  if (clock_min_delta != UINT32_MAX) {

    ESP_LOGI(
        TAG,
        "CLOCK min interval: %lu us",
        (unsigned long) clock_min_delta);

    ESP_LOGI(
        TAG,
        "CLOCK max interval: %lu us",
        (unsigned long) clock_max_delta);

    if (clock_min_delta > 0) {

      ESP_LOGI(
          TAG,
          "CLOCK max edge rate: %.1f kHz",
          1000.0f / clock_min_delta);
    }
  }

  if (data_min_delta != UINT32_MAX) {

    ESP_LOGI(
        TAG,
        "DATA min interval: %lu us",
        (unsigned long) data_min_delta);

    ESP_LOGI(
        TAG,
        "DATA max interval: %lu us",
        (unsigned long) data_max_delta);
  }

  if (frame_min_delta != UINT32_MAX) {

    ESP_LOGI(
        TAG,
        "FRAME min interval: %lu us",
        (unsigned long) frame_min_delta);

    ESP_LOGI(
        TAG,
        "FRAME max interval: %lu us",
        (unsigned long) frame_max_delta);
  }

  ESP_LOGI(TAG, "------------------------------");
}


/*
 * ============================================================
 * Serial Decoder
 * ============================================================
 *
 * Wir testen mehrere Interpretationen desselben Signals:
 *
 * 1. CLOCK rising, MSB first
 * 2. CLOCK rising, LSB first
 * 3. CLOCK falling, MSB first
 * 4. CLOCK falling, LSB first
 *
 * Dadurch müssen wir nicht vorher raten, welche Flanke und
 * Bitreihenfolge das unbekannte Protokoll verwendet.
 */


/*
 * Byte als Hex + Binär darstellen.
 */
static void log_serial_byte(
    const char *name,
    uint32_t index,
    uint8_t byte)
{
  ESP_LOGI(
      TAG,
      "  %s BYTE[%lu] = 0x%02X  %u%u%u%u%u%u%u%u",
      name,
      (unsigned long) index,
      byte,
      (byte >> 7) & 1,
      (byte >> 6) & 1,
      (byte >> 5) & 1,
      (byte >> 4) & 1,
      (byte >> 3) & 1,
      (byte >> 2) & 1,
      (byte >> 1) & 1,
      byte & 1);
}


/*
 * Allgemeiner Decoder.
 *
 * rising = true:
 *   Daten auf steigender CLOCK-Flanke sampeln.
 *
 * rising = false:
 *   Daten auf fallender CLOCK-Flanke sampeln.
 *
 * lsb_first = false:
 *   MSB first
 *
 * lsb_first = true:
 *   LSB first
 */
static void decode_serial_variant(
    const Sample *data,
    uint16_t count,
    bool rising,
    bool lsb_first,
    const char *name)
{
  uint8_t current_byte = 0;
  int bit_count = 0;

  uint32_t byte_count = 0;
  uint32_t bit_count_total = 0;

  uint32_t first_edge_time = 0;
  bool have_edge = false;

  /*
   * Nur CLOCK-Flanken verwenden.
   */
  for (uint16_t i = 1; i < count; i++) {

    bool prev_clock =
        clock_level(data[i - 1]);

    bool cur_clock =
        clock_level(data[i]);

    bool edge =
        rising
            ? (!prev_clock && cur_clock)
            : (prev_clock && !cur_clock);

    if (!edge)
      continue;

    /*
     * Optional: nur innerhalb eines aktiven FRAME-Signals
     * dekodieren.
     *
     * Für den ersten Durchlauf verwenden wir absichtlich
     * NICHT GPIO38 als Gate, damit wir nichts verpassen.
     */

    bool bit = data_level(data[i]);

    if (!have_edge) {
      first_edge_time = data[i].t;
      have_edge = true;
    }

    /*
     * --------------------------------------------------------
     * Bit in Byte einfügen
     * --------------------------------------------------------
     */
    if (lsb_first) {

      if (bit)
        current_byte |=
            (1 << bit_count);

    } else {

      current_byte <<= 1;

      if (bit)
        current_byte |= 1;
    }

    bit_count++;
    bit_count_total++;

    /*
     * --------------------------------------------------------
     * 8 Bits komplett
     * --------------------------------------------------------
     */
    if (bit_count == 8) {

      log_serial_byte(
          name,
          byte_count,
          current_byte);

      byte_count++;

      current_byte = 0;
      bit_count = 0;
    }
  }

  ESP_LOGI(
      TAG,
      "%s: %lu clock edges -> %lu complete bytes, %d remaining bits",
      name,
      (unsigned long) bit_count_total,
      (unsigned long) byte_count,
      bit_count);
}


/*
 * ============================================================
 * FRAME-basierter Decoder
 * ============================================================
 *
 * GPIO38 wird zusätzlich ausgewertet.
 *
 * Momentan testen wir:
 *
 *   FRAME HIGH = aktiver Datenbereich
 *
 * und dekodieren CLOCK rising / DATA.
 *
 * Damit können wir später sehr leicht feststellen, ob GPIO38
 * tatsächlich CS/ENABLE ist.
 */
static void decode_frame_serial(
    const Sample *data,
    uint16_t count)
{
  ESP_LOGI(TAG, "FRAME SERIAL DECODE");
  ESP_LOGI(TAG, "------------------------------");

  bool active = false;

  uint8_t current_byte = 0;
  int bit_count = 0;

  uint32_t byte_count = 0;
  uint32_t edge_count = 0;

  for (uint16_t i = 1; i < count; i++) {

    bool prev_frame =
        frame_level(data[i - 1]);

    bool cur_frame =
        frame_level(data[i]);

    /*
     * --------------------------------------------------------
     * FRAME rising
     * --------------------------------------------------------
     */
    if (!prev_frame && cur_frame) {

      active = true;

      current_byte = 0;
      bit_count = 0;
      byte_count = 0;
      edge_count = 0;

      ESP_LOGI(
          TAG,
          "%lu us: FRAME START",
          (unsigned long)
              (data[i].t - data[0].t));

      continue;
    }

    /*
     * --------------------------------------------------------
     * FRAME falling
     * --------------------------------------------------------
     */
    if (prev_frame && !cur_frame) {

      ESP_LOGI(
          TAG,
          "%lu us: FRAME END (%lu bits)",
          (unsigned long)
              (data[i].t - data[0].t),
          (unsigned long) edge_count);

      if (bit_count != 0) {

        ESP_LOGI(
            TAG,
            "  remaining bits: %d",
            bit_count);
      }

      active = false;

      continue;
    }

    if (!active)
      continue;

    /*
     * CLOCK rising = Datenbit.
     */
    bool prev_clock =
        clock_level(data[i - 1]);

    bool cur_clock =
        clock_level(data[i]);

    if (!prev_clock && cur_clock) {

      bool bit = data_level(data[i]);

      current_byte <<= 1;

      if (bit)
        current_byte |= 1;

      bit_count++;
      edge_count++;

      if (bit_count == 8) {

        ESP_LOGI(
            TAG,
            "  FRAME BYTE[%lu] = 0x%02X",
            (unsigned long) byte_count,
            current_byte);

        byte_count++;

        current_byte = 0;
        bit_count = 0;
      }
    }
  }

  ESP_LOGI(TAG, "------------------------------");
}


/*
 * ============================================================
 * Rohdaten
 * ============================================================
 */
static void dump_data(
    const Sample *data,
    uint16_t count)
{
  uint16_t limit = count;

  if (limit > 300)
    limit = 300;

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
        "%lu us CLK=%u DATA=%u FRAME=%u",
        (unsigned long) rel,
        clock_level(data[i]) ? 1 : 0,
        data_level(data[i]) ? 1 : 0,
        frame_level(data[i]) ? 1 : 0);
  }
}


/*
 * ============================================================
 * Setup
 * ============================================================
 */
void BusSniffer::setup()
{
  /*
   * GPIOs als reine Eingänge.
   *
   * Keine internen Pullups:
   * GPIO39 und GPIO40 besitzen externe 10k Pullups.
   */
  gpio_config_t io = {};

  io.mode = GPIO_MODE_INPUT;

  io.pull_up_en = GPIO_PULLUP_DISABLE;
  io.pull_down_en = GPIO_PULLDOWN_DISABLE;

  /*
   * Alle drei Leitungen lösen Interrupts aus.
   */
  io.intr_type = GPIO_INTR_ANYEDGE;

  io.pin_bit_mask =
      (1ULL << PIN_CLOCK) |
      (1ULL << PIN_DATA) |
      (1ULL << PIN_FRAME);

  gpio_config(&io);


  /*
   * ISR-Service.
   */
  esp_err_t err =
      gpio_install_isr_service(0);

  if (err != ESP_OK &&
      err != ESP_ERR_INVALID_STATE) {

    ESP_LOGE(
        TAG,
        "gpio_install_isr_service failed: %d",
        err);

    return;
  }


  /*
   * CLOCK
   */
  err =
      gpio_isr_handler_add(
          PIN_CLOCK,
          gpio_isr,
          nullptr);

  if (err != ESP_OK) {

    ESP_LOGE(
        TAG,
        "ISR CLOCK GPIO40 failed: %d",
        err);

    return;
  }


  /*
   * DATA
   */
  err =
      gpio_isr_handler_add(
          PIN_DATA,
          gpio_isr,
          nullptr);

  if (err != ESP_OK) {

    ESP_LOGE(
        TAG,
        "ISR DATA GPIO39 failed: %d",
        err);

    return;
  }


  /*
   * FRAME
   */
  err =
      gpio_isr_handler_add(
          PIN_FRAME,
          gpio_isr,
          nullptr);

  if (err != ESP_OK) {

    ESP_LOGE(
        TAG,
        "ISR FRAME GPIO38 failed: %d",
        err);

    return;
  }


  /*
   * Anfangszustand synchronisieren.
   */
  last_value = read_bus();

  last_edge =
      (uint32_t) esp_timer_get_time();


  ESP_LOGI(TAG, "==============================");
  ESP_LOGI(TAG, "SERIAL BUS SNIFFER STARTED");
  ESP_LOGI(TAG, "CLOCK = GPIO40");
  ESP_LOGI(TAG, "DATA  = GPIO39");
  ESP_LOGI(TAG, "FRAME = GPIO38");
  ESP_LOGI(TAG, "==============================");
}


/*
 * ============================================================
 * Loop
 * ============================================================
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


  uint16_t count =
      sample_count;

  if (count > MAX_SAMPLES)
    count = MAX_SAMPLES;


  /*
   * Volatile -> normal kopieren.
   */
  Sample local[MAX_SAMPLES];

  for (uint16_t i = 0; i < count; i++) {

    local[i].t =
        samples[i].t;

    local[i].value =
        samples[i].value;
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
   * ----------------------------------------------------------
   * Statistik
   * ----------------------------------------------------------
   */
  analyze_bus(
      local,
      count);


  /*
   * ----------------------------------------------------------
   * Variante 1:
   *
   * GPIO40 rising
   * GPIO39 DATA
   * MSB first
   * ----------------------------------------------------------
   */
  ESP_LOGI(TAG, "SERIAL DECODE");
  ESP_LOGI(TAG, "------------------------------");

  decode_serial_variant(
      local,
      count,
      true,
      false,
      "RISING / MSB");


  /*
   * ----------------------------------------------------------
   * Variante 2:
   *
   * GPIO40 rising
   * GPIO39 DATA
   * LSB first
   * ----------------------------------------------------------
   */
  decode_serial_variant(
      local,
      count,
      true,
      true,
      "RISING / LSB");


  /*
   * ----------------------------------------------------------
   * Variante 3:
   *
   * GPIO40 falling
   * GPIO39 DATA
   * MSB first
   * ----------------------------------------------------------
   */
  decode_serial_variant(
      local,
      count,
      false,
      false,
      "FALLING / MSB");


  /*
   * ----------------------------------------------------------
   * Variante 4:
   *
   * GPIO40 falling
   * GPIO39 DATA
   * LSB first
   * ----------------------------------------------------------
   */
  decode_serial_variant(
      local,
      count,
      false,
      true,
      "FALLING / LSB");


  /*
   * ----------------------------------------------------------
   * GPIO38 als Frame-Signal testen.
   * ----------------------------------------------------------
   */
  decode_frame_serial(
      local,
      count);


  /*
   * ----------------------------------------------------------
   * Rohdaten
   * ----------------------------------------------------------
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

