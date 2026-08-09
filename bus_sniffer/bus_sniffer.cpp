#include "bus_sniffer.h"

#include "esphome/core/log.h"
#include "esphome/components/web_server_base/web_server_base.h"

#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_timer.h"
#include "esp_http_server.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include <climits>
#include <cstdio>
#include <climits>
#include <cstring>
#include <string>
#include <utility>


namespace esphome {
namespace bus_sniffer {

static const char *TAG = "bus_sniffer";


/*
 * ============================================================================
 * Hardware
 * ============================================================================
 *
 * GPIO40 = SCL
 * GPIO39 = SDA
 * GPIO38 = zusätzlicher Logic-Analyzer-Kanal
 *
 * GPIO38 wird nur mitgesampelt und erzeugt selbst keine Interrupts.
 */

static constexpr gpio_num_t PIN_SCL =
    GPIO_NUM_40;

static constexpr gpio_num_t PIN_SDA =
    GPIO_NUM_39;

static constexpr gpio_num_t PIN_OTHER =
    GPIO_NUM_38;


/*
 * ============================================================================
 * Capture
 * ============================================================================
 */

static constexpr uint16_t MAX_SAMPLES =
    4096;


/*
 * Zwischen EC05 und der Antwort sehen wir etwa 2.13 ms.
 *
 * Mit 5 ms Timeout landen Request und Response im selben Capture.
 */
static constexpr uint32_t CAPTURE_TIMEOUT_US =
    5000;


struct Sample {
  uint32_t t;
  uint8_t value;
};


static volatile Sample samples[MAX_SAMPLES];

static volatile uint16_t sample_count = 0;

static volatile uint32_t last_edge = 0;

static volatile uint8_t last_value = 0xff;

static volatile uint8_t capture_initial_value =
    0xff;

static volatile bool capturing = true;

static volatile bool capture_finished = false;

static volatile bool capture_overflow = false;


/*
 * ============================================================================
 * Letzter Raw-Capture für HTTP
 * ============================================================================
 */

static std::string last_capture_data;

static SemaphoreHandle_t last_capture_mutex =
    nullptr;


/*
 * ============================================================================
 * GPIO
 * ============================================================================
 */

static inline uint8_t IRAM_ATTR read_gpio_state()
{
  uint8_t value = 0;

  if (gpio_get_level(PIN_SCL))
    value |= 0x01;

  if (gpio_get_level(PIN_SDA))
    value |= 0x02;

  if (gpio_get_level(PIN_OTHER))
    value |= 0x04;

  return value;
}


static inline bool scl_level(uint8_t value)
{
  return (value & 0x01) != 0;
}


static inline bool sda_level(uint8_t value)
{
  return (value & 0x02) != 0;
}


/*
 * ============================================================================
 * ISR
 * ============================================================================
 *
 * Nur SCL und SDA lösen Interrupts aus.
 */

static void IRAM_ATTR gpio_isr(void *arg)
{
  if (!capturing)
    return;


  const uint32_t now =
      static_cast<uint32_t>(
          esp_timer_get_time()
      );


  const uint8_t value =
      read_gpio_state();


  /*
   * Mehrere praktisch gleichzeitige GPIO-Interrupts können
   * denselben Gesamtzustand sehen.
   */
  if (value == last_value)
    return;


  /*
   * Zustand unmittelbar vor der ersten Flanke erhalten.
   * Wird für START/STOP-Erkennung benötigt.
   */
  if (sample_count == 0)
    capture_initial_value =
        last_value;


  last_value = value;
  last_edge = now;


  const uint16_t index =
      sample_count;


  if (index < MAX_SAMPLES) {

    samples[index].t =
        now;

    samples[index].value =
        value;

    sample_count =
        index + 1;

  } else {

    capture_overflow = true;
    capturing = false;
    capture_finished = true;
  }
}


/*
 * ============================================================================
 * CRC-8
 * ============================================================================
 *
 * Sensirion/SCD4x:
 *
 * Polynomial 0x31
 * Init       0xFF
 */

static uint8_t sensirion_crc(
    uint8_t byte0,
    uint8_t byte1)
{
  uint8_t crc =
      0xff;


  const uint8_t data[2] = {
      byte0,
      byte1
  };


  for (uint8_t n = 0;
       n < 2;
       n++) {

    crc ^=
        data[n];


    for (uint8_t bit = 0;
         bit < 8;
         bit++) {

      if (crc & 0x80) {

        crc =
            static_cast<uint8_t>(
                (crc << 1) ^ 0x31
            );

      } else {

        crc =
            static_cast<uint8_t>(
                crc << 1
            );
      }
    }
  }


  return crc;
}


/*
 * ============================================================================
 * I2C Decoder
 * ============================================================================
 */

struct I2CTransaction {
  uint8_t bytes[16];
  bool ack[16];

  uint8_t count;
};


struct DecodeResult {
  bool have_co2{false};

  uint16_t co2_ppm{0};

  uint32_t crc_errors{0};

  uint32_t frame_errors{0};
};


static void clear_transaction(
    I2CTransaction &txn)
{
  txn.count = 0;

  memset(
      txn.bytes,
      0,
      sizeof(txn.bytes)
  );

  memset(
      txn.ack,
      0,
      sizeof(txn.ack)
  );
}


/*
 * ============================================================================
 * Einzelne I2C-Transaktion
 * ============================================================================
 */

static void process_i2c_transaction(
    const I2CTransaction &txn,
    DecodeResult &result)
{
  if (txn.count == 0)
    return;


  /*
   * --------------------------------------------------------------------------
   * SCD4x-kompatibles read_measurement
   *
   * Adresse 0x62 WRITE:
   *
   * C4 EC 05
   * --------------------------------------------------------------------------
   */

  if (
      txn.count >= 3 &&
      txn.bytes[0] == 0xC4 &&
      txn.bytes[1] == 0xEC &&
      txn.bytes[2] == 0x05
  ) {

    ESP_LOGV(
        TAG,
        "read_measurement command"
    );

    return;
  }


  /*
   * --------------------------------------------------------------------------
   * Antwort:
   *
   * C5 CO2_MSB CO2_LSB CRC
   *
   * Danach beendet der originale Controller die Übertragung.
   * --------------------------------------------------------------------------
   */

  if (txn.bytes[0] != 0xC5)
    return;


  /*
   * Adresse + zwei Datenbytes + CRC erforderlich.
   */
  if (txn.count < 4) {

    result.frame_errors++;

    ESP_LOGV(
        TAG,
        "Incomplete CO2 transaction: %u bytes",
        txn.count
    );

    return;
  }


  /*
   * Erwartetes ACK-Muster:
   *
   * C5        ACK
   * CO2 MSB   ACK
   * CO2 LSB   ACK
   * CRC       NACK
   */

  if (
      !txn.ack[0] ||
      !txn.ack[1] ||
      !txn.ack[2] ||
      txn.ack[3]
  ) {

    result.frame_errors++;

    ESP_LOGV(
        TAG,
        "Invalid ACK sequence"
    );

    return;
  }


  const uint8_t msb =
      txn.bytes[1];

  const uint8_t lsb =
      txn.bytes[2];

  const uint8_t received_crc =
      txn.bytes[3];


  const uint8_t calculated_crc =
      sensirion_crc(
          msb,
          lsb
      );


  if (received_crc != calculated_crc) {

    result.crc_errors++;

    ESP_LOGV(
        TAG,
        "CRC mismatch: %02X %02X, "
        "received=%02X expected=%02X",
        msb,
        lsb,
        received_crc,
        calculated_crc
    );

    return;
  }


  result.co2_ppm =
      (
          static_cast<uint16_t>(msb)
          << 8
      ) |
      lsb;


  result.have_co2 =
      true;
}


/*
 * ============================================================================
 * Capture als I2C dekodieren
 * ============================================================================
 */

static DecodeResult decode_i2c_capture(
    const volatile Sample *data,
    uint16_t count,
    uint8_t initial_value)
{
  DecodeResult result;


  if (count == 0)
    return result;


  bool active =
      false;


  uint8_t current_byte =
      0;


  uint8_t bit_count =
      0;


  I2CTransaction txn;

  clear_transaction(
      txn
  );


  uint8_t previous =
      initial_value;


  for (uint16_t i = 0;
       i < count;
       i++) {

    const uint8_t current =
        data[i].value;


    const bool prev_scl =
        scl_level(previous);

    const bool cur_scl =
        scl_level(current);

    const bool prev_sda =
        sda_level(previous);

    const bool cur_sda =
        sda_level(current);


    /*
     * ------------------------------------------------------------------------
     * START / repeated START
     *
     * SDA fällt bei SCL HIGH.
     * ------------------------------------------------------------------------
     */

    if (
        prev_sda &&
        !cur_sda &&
        cur_scl
    ) {

      if (
          active &&
          txn.count != 0
      ) {

        process_i2c_transaction(
            txn,
            result
        );
      }


      clear_transaction(
          txn
      );


      current_byte = 0;
      bit_count = 0;
      active = true;

      previous =
          current;

      continue;
    }


    /*
     * ------------------------------------------------------------------------
     * STOP
     *
     * SDA steigt bei SCL HIGH.
     * ------------------------------------------------------------------------
     */

    if (
        active &&
        !prev_sda &&
        cur_sda &&
        cur_scl
    ) {

      if (txn.count != 0) {

        process_i2c_transaction(
            txn,
            result
        );
      }


      clear_transaction(
          txn
      );


      current_byte = 0;
      bit_count = 0;
      active = false;

      previous =
          current;

      continue;
    }


    /*
     * ------------------------------------------------------------------------
     * Daten auf steigender SCL-Flanke.
     * ------------------------------------------------------------------------
     */

    if (
        active &&
        !prev_scl &&
        cur_scl
    ) {

      const bool bit =
          cur_sda;


      /*
       * 8 Datenbits.
       */
      if (bit_count < 8) {

        current_byte =
            static_cast<uint8_t>(
                current_byte << 1
            );


        if (bit)
          current_byte |= 1;


        bit_count++;

      } else {

        /*
         * 9. Clock:
         *
         * LOW  = ACK
         * HIGH = NACK
         */

        if (
            txn.count <
            sizeof(txn.bytes)
        ) {

          txn.bytes[txn.count] =
              current_byte;


          txn.ack[txn.count] =
              !bit;


          txn.count++;
        }


        current_byte = 0;
        bit_count = 0;
      }
    }


    previous =
        current;
  }


  /*
   * Falls STOP selbst nicht mehr im Capture gelandet ist.
   */
  if (
      active &&
      txn.count != 0
  ) {

    process_i2c_transaction(
        txn,
        result
    );
  }


  return result;
}


/*
 * ============================================================================
 * Raw-Capture archivieren
 * ============================================================================
 *
 * Format LA01:
 *
 * 4 Byte  "LA01"
 * 4 Byte  sample count
 * 1 Byte  flags
 *
 * danach pro Event:
 *
 * 4 Byte  timestamp_us
 * 1 Byte  GPIO state
 *
 * flags:
 *
 * bit 0 = capture overflow
 */

static void store_raw_capture(
    const volatile Sample *data,
    uint16_t count,
    bool overflow)
{
  if (count == 0)
    return;


  std::string output;


  output.resize(
      9 +
      static_cast<size_t>(count) * 5
  );


  char *p =
      output.data();


  memcpy(
      p,
      "LA01",
      4
  );

  p += 4;


  const uint32_t count32 =
      count;


  memcpy(
      p,
      &count32,
      sizeof(count32)
  );

  p += 4;


  uint8_t flags =
      0;


  if (overflow)
    flags |= 0x01;


  *p++ =
      static_cast<char>(
          flags
      );


  const uint32_t base =
      data[0].t;


  for (uint16_t i = 0;
       i < count;
       i++) {

    const uint32_t timestamp =
        static_cast<uint32_t>(
            data[i].t - base
        );


    memcpy(
        p,
        &timestamp,
        sizeof(timestamp)
    );

    p += 4;


    *p++ =
        static_cast<char>(
            data[i].value
        );
  }


  if (
      last_capture_mutex != nullptr &&
      xSemaphoreTake(
          last_capture_mutex,
          pdMS_TO_TICKS(100)
      ) == pdTRUE
  ) {

    last_capture_data =
        std::move(output);


    xSemaphoreGive(
        last_capture_mutex
    );
  }
}


/*
 * ============================================================================
 * HTTP /capture
 * ============================================================================
 */

class CaptureHandler
    : public web_server_idf::AsyncWebHandler {
 public:

  bool canHandle(
      web_server_idf::AsyncWebServerRequest *request)
      const override
  {
    if (request->method() != HTTP_GET)
      return false;


    char url[
        web_server_idf::AsyncWebServerRequest::URL_BUF_SIZE
    ];


    return request->url_to(url) ==
        "/capture";
  }


  void handleRequest(
      web_server_idf::AsyncWebServerRequest *request)
      override
  {
    std::string output;


    if (
        last_capture_mutex != nullptr &&
        xSemaphoreTake(
            last_capture_mutex,
            pdMS_TO_TICKS(100)
        ) == pdTRUE
    ) {

      output =
          last_capture_data;


      xSemaphoreGive(
          last_capture_mutex
      );
    }


    if (output.empty()) {

      request->send(
          204,
          "text/plain",
          nullptr
      );

      return;
    }


    auto *response =
        request->beginResponse(
            200,
            "application/octet-stream",
            output
        );


    response->addHeader(
        "Content-Disposition",
        "attachment; filename=\"capture.la\""
    );


    request->send(
        response
    );
  }
};


static CaptureHandler capture_handler;


/*
 * ============================================================================
 * RT/RH digital edge capture
 * ============================================================================
 *
 * GPIO10..13 are connected to the four RT/RH test points.  They are treated
 * strictly as passive digital inputs.  The first edge starts a 500 ms capture;
 * every subsequent state change stores a microsecond timestamp plus the state
 * of all four lines.  When complete, the frozen capture is available as CSV at
 * /rt_rh_capture.csv.  Downloading it rearms the capture for the next cycle.
 */

static constexpr gpio_num_t PIN_RTRH0 = GPIO_NUM_10;
static constexpr gpio_num_t PIN_RTRH1 = GPIO_NUM_11;
static constexpr gpio_num_t PIN_RTRH2 = GPIO_NUM_12;
static constexpr gpio_num_t PIN_RTRH3 = GPIO_NUM_13;

static constexpr uint32_t RTRH_CAPTURE_US = 75000;
static constexpr uint16_t RTRH_MAX_SAMPLES = 3584;

struct __attribute__((packed)) RtRhSample {
  uint32_t t_us;
  uint8_t value;
};

static volatile RtRhSample rtrh_samples[RTRH_MAX_SAMPLES];
static volatile uint16_t rtrh_sample_count = 0;
static volatile uint8_t rtrh_last_value = 0xff;
static volatile uint32_t rtrh_start_us = 0;
static volatile bool rtrh_capturing = false;
static volatile bool rtrh_capture_ready = false;
static volatile bool rtrh_overflow = false;
static volatile uint32_t rtrh_sequence = 0;
static volatile bool rtrh_irqs_suspended = false;

/*
 * ============================================================================
 * Phase-synchronous RT/RH ADC probe
 * ============================================================================
 *
 * GPIO10 falling is our phase reference. The measured digital period is about
 * 76-77 us. RH testing showed that GPIO13 in SHORT mode changes most strongly
 * across the early/middle part of the cycle.  The 31..66 % RH sweep showed
 * that GPIO13 is already saturated by +24 us at high humidity, so move the
 * four phases earlier:
 *
 *   phase 0: +12 us
 *   phase 1: +16 us
 *   phase 2: +20 us
 *   phase 3: +24 us
 *
 * Eight repeats are retained.  Their individual SHORT/GPIO13 raw values are
 * also exported so cycle-to-cycle structure is not hidden by the mean.
 */

static constexpr uint8_t RTRH_ADC_PHASES = 4;
static constexpr uint8_t RTRH_ADC_CHANNELS = 4;
static constexpr uint8_t RTRH_ADC_MODES = 2;

// Legacy ADC diagnostics are disabled in this passive-only build, but the
// old helper functions remain compiled. Keep their GPIO lookup table so the
// dead code still parses.
static constexpr int RTRH_ADC_GPIOS[4] = {10, 11, 12, 13};

static constexpr uint8_t RTRH_ADC_REPEATS = 8;

// A valid SHORT/LONG reference edge normally arrives within << 1 ms.
// Never let one missed edge or an out-of-range sensor cycle block the ADC task
// forever.  100 ms is deliberately very generous compared with 76..175 us.
static constexpr uint32_t RTRH_ADC_EDGE_TIMEOUT_MS = 100;
static constexpr uint16_t RTRH_ADC_SAMPLES =
    RTRH_ADC_MODES * RTRH_ADC_PHASES * RTRH_ADC_CHANNELS * RTRH_ADC_REPEATS;

static constexpr uint8_t RTRH_MODE_SHORT = 0;
static constexpr uint8_t RTRH_MODE_LONG = 1;

// Observed GPIO10/GPIO11 falling-edge periods.
// 9-6: short median 77 us, long median 139 us.
// Keep generous margins for ISR jitter without allowing the classes to overlap.
static constexpr uint32_t RTRH_SHORT_MIN_US = 65;
static constexpr uint32_t RTRH_SHORT_MAX_US = 100;
static constexpr uint32_t RTRH_LONG_MIN_US = 115;
static constexpr uint32_t RTRH_LONG_MAX_US = 175;

// Empirical calibration from the combined 2026-08-09 heat/cool + RH series.
// The NTC/RC relation is measurably nonlinear over the observed range, so use
// the quadratic fit instead of the earlier linear approximation:
//   T[degC] = A*t^2 + B*t + C, t = LONG period in us.
static constexpr float RTRH_TEMP_A = 0.00595169f;
static constexpr float RTRH_TEMP_B = -2.025624f;
static constexpr float RTRH_TEMP_C = 194.833723f;
static constexpr uint16_t RTRH_ADC_STORED_SAMPLES = 32;
static constexpr uint8_t RTRH_ADC_SKIP_EDGES = 3;
static constexpr uint16_t RTRH_ADC_OFFSETS_US[RTRH_ADC_PHASES] = {
    12, 16, 20, 24
};

/*
 * Passive timing probe.
 *
 * Before any GPIO is switched to ADC mode, observe a handful of completely
 * untouched RT/RH cycles:
 *
 *   - GPIO10 falling-to-falling period -> SHORT/LONG classification
 *   - GPIO10 rising -> GPIO13 rising delay -> passive RH threshold timing
 *
 * After enough SHORT and LONG cycles have been observed, one ADC sequence is
 * allowed for the burst.  This prevents ADC pad reconfiguration from biasing
 * the timing values used for RT/RH decoding.
 */
static constexpr uint32_t RTRH_PASSIVE_QUIET_US = 100000;
static constexpr uint8_t RTRH_PASSIVE_LOCK_CYCLES = 8;
static constexpr uint8_t RTRH_PASSIVE_MAX_BLOCKS = 16;
static constexpr uint8_t RTRH_PASSIVE_DELAY_BINS = 8;  // d0..d6, d7plus

struct RtRhPassiveBlock {
  uint8_t mode{0xFF};
  uint16_t count{0};

  uint32_t period_sum{0};
  uint32_t low_sum{0};
  uint32_t g13_delay_sum{0};
  uint16_t g13_delay_count{0};

  uint16_t period_min{0xFFFF};
  uint16_t period_max{0};
  uint16_t low_min{0xFFFF};
  uint16_t low_max{0};
  uint16_t g13_delay_min{0xFFFF};
  uint16_t g13_delay_max{0};

  uint16_t g13_delay_hist[RTRH_PASSIVE_DELAY_BINS]{};
};

struct RtRhPassiveSnapshot {
  RtRhPassiveBlock blocks[RTRH_PASSIVE_MAX_BLOCKS];
  uint8_t block_count{0};
  bool overflow{false};
  uint32_t sequence{0};
};

static volatile uint32_t rtrh_passive_last_any_us = 0;
static volatile uint8_t rtrh_passive_last_state = 0;

static volatile uint32_t rtrh_passive_last_g10_fall_us = 0;
static volatile uint32_t rtrh_passive_g10_rise_us = 0;
static volatile uint32_t rtrh_passive_g13_rise_us = 0;
static volatile bool rtrh_passive_have_g10_rise = false;
static volatile bool rtrh_passive_have_g13_rise = false;

static volatile uint8_t rtrh_passive_current_mode = 0xFF;
static volatile uint8_t rtrh_passive_candidate_mode = 0xFF;
static volatile uint8_t rtrh_passive_candidate_run = 0;

static volatile bool rtrh_passive_collecting = false;
static volatile bool rtrh_passive_overflow = false;
static volatile uint8_t rtrh_passive_block_count = 0;

static RtRhPassiveBlock
    rtrh_passive_blocks[RTRH_PASSIVE_MAX_BLOCKS];

static RtRhPassiveSnapshot rtrh_passive_snapshot;
static volatile bool rtrh_passive_snapshot_ready = false;
static volatile bool rtrh_passive_snapshot_consumed = true;

struct RtRhAdcAggregate {
  uint16_t sum{0};
  uint16_t period_sum{0};
  uint16_t min{0xFFFF};
  uint16_t max{0};
  uint16_t count{0};
  uint16_t period_count{0};
  uint16_t max_lateness_us{0};
  uint16_t rejected_late{0};
  uint16_t read_errors{0};
};

struct RtRhAdcChannel {
  adc_unit_t unit{ADC_UNIT_1};
  adc_channel_t channel{ADC_CHANNEL_0};
  bool configured{false};
};

static RtRhAdcChannel rtrh_adc_channels[RTRH_ADC_CHANNELS];
static adc_oneshot_unit_handle_t rtrh_adc1_handle = nullptr;
static adc_oneshot_unit_handle_t rtrh_adc2_handle = nullptr;

static RtRhAdcAggregate
    rtrh_adc_agg[RTRH_ADC_MODES][RTRH_ADC_PHASES][RTRH_ADC_CHANNELS];

// Independent last-completed ADC snapshot for HTTP.
static RtRhAdcAggregate
    rtrh_adc_snapshot[RTRH_ADC_MODES][RTRH_ADC_PHASES][RTRH_ADC_CHANNELS];

// Individual accepted SHORT/GPIO13 samples. 0xFFFF means missing/rejected.
// Only 4 phases x 8 repeats = 64 bytes per live/snapshot buffer.
static uint16_t
    rtrh_short_g13_raw[RTRH_ADC_PHASES][RTRH_ADC_REPEATS];
static uint16_t
    rtrh_short_g13_raw_snapshot[RTRH_ADC_PHASES][RTRH_ADC_REPEATS];

static volatile bool rtrh_adc_snapshot_ready = false;
static volatile bool rtrh_adc_snapshot_in_use = false;
static volatile uint32_t rtrh_adc_snapshot_sequence = 0;
static portMUX_TYPE rtrh_adc_snapshot_mux = portMUX_INITIALIZER_UNLOCKED;

static volatile bool rtrh_adc_active = false;
static volatile bool rtrh_adc_ready = false;
static volatile bool rtrh_adc_pending = false;
static volatile uint16_t rtrh_adc_cycle = 0;
static volatile uint32_t rtrh_adc_edge_us = 0;
static volatile uint32_t rtrh_adc_sequence = 0;
static volatile uint32_t rtrh_adc_missed_edges = 0;
static volatile uint32_t rtrh_adc_reference_edge_us = 0;
static volatile uint8_t rtrh_adc_edges_to_skip = 0;
static volatile bool rtrh_adc_waiting_for_edge = false;
static volatile int8_t rtrh_adc_target_index = -1;
static volatile int8_t rtrh_adc_trigger_index = -1;
static volatile bool rtrh_adc_sequence_start = false;
static volatile uint8_t rtrh_pin_level[4] = {0, 0, 0, 0};
static volatile uint32_t rtrh_last_fall_us[4] = {0, 0, 0, 0};
static volatile uint16_t rtrh_adc_trigger_period_us = 0;
static volatile uint8_t rtrh_adc_desired_mode = RTRH_MODE_SHORT;

static TaskHandle_t rtrh_adc_task_handle = nullptr;

static inline uint8_t IRAM_ATTR read_rtrh_state()
{
  uint8_t value = 0;
  if (gpio_get_level(PIN_RTRH0)) value |= 0x01;
  if (gpio_get_level(PIN_RTRH1)) value |= 0x02;
  if (gpio_get_level(PIN_RTRH2)) value |= 0x04;
  if (gpio_get_level(PIN_RTRH3)) value |= 0x08;
  return value;
}


static adc_oneshot_unit_handle_t rtrh_adc_handle(adc_unit_t unit)
{
  return unit == ADC_UNIT_1 ? rtrh_adc1_handle : rtrh_adc2_handle;
}

static void reset_rtrh_adc_capture()
{
  /*
   * rtrh_adc_ready is also the ISR's start gate.
   *
   * Keep that gate CLOSED while resetting the shared state.  In the previous
   * version active=false/ready=false were written first, so a GPIO10 falling
   * edge could start a new sequence halfway through this function and the
   * remaining writes would then destroy parts of the freshly-started state.
   *
   * ESP32-S2 is single-core, so this ordering is sufficient: the ISR may
   * preempt us, but it will see ready=true until every field is clean.
   */
  rtrh_adc_ready = true;

  rtrh_adc_pending = false;
  rtrh_adc_waiting_for_edge = false;
  rtrh_adc_sequence_start = false;

  rtrh_adc_cycle = 0;
  rtrh_adc_missed_edges = 0;
  rtrh_adc_reference_edge_us = 0;
  rtrh_adc_edges_to_skip = 0;
  rtrh_adc_target_index = -1;
  rtrh_adc_trigger_index = -1;
  rtrh_adc_trigger_period_us = 0;

  rtrh_last_fall_us[0] = 0;
  rtrh_last_fall_us[1] = 0;
  rtrh_last_fall_us[2] = 0;
  rtrh_last_fall_us[3] = 0;

  // Open the start gate only after all shared state is known-good.
  rtrh_adc_active = false;
  rtrh_adc_ready = false;
}

static void clear_rtrh_adc_aggregates()
{
  for (uint8_t phase = 0; phase < RTRH_ADC_PHASES; phase++) {
    for (uint8_t repeat = 0; repeat < RTRH_ADC_REPEATS; repeat++) {
      rtrh_short_g13_raw[phase][repeat] = 0xFFFF;
    }
  }

  for (uint8_t mode = 0; mode < RTRH_ADC_MODES; mode++) {
    for (uint8_t phase = 0; phase < RTRH_ADC_PHASES; phase++) {
      for (uint8_t channel = 0; channel < RTRH_ADC_CHANNELS; channel++) {
        rtrh_adc_agg[mode][phase][channel] = RtRhAdcAggregate{};
      }
    }
  }
}


static void setup_rtrh_adc()
{
  adc_oneshot_unit_init_cfg_t unit1_cfg = {};
  unit1_cfg.unit_id = ADC_UNIT_1;

  esp_err_t err = adc_oneshot_new_unit(&unit1_cfg, &rtrh_adc1_handle);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "RT/RH ADC1 init failed: %s", esp_err_to_name(err));
    rtrh_adc1_handle = nullptr;
  }

  adc_oneshot_unit_init_cfg_t unit2_cfg = {};
  unit2_cfg.unit_id = ADC_UNIT_2;

  err = adc_oneshot_new_unit(&unit2_cfg, &rtrh_adc2_handle);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "RT/RH ADC2 init failed: %s", esp_err_to_name(err));
    rtrh_adc2_handle = nullptr;
  }

  /*
   * IMPORTANT:
   * Only map GPIO -> ADC channel here. adc_oneshot_config_channel() is
   * deliberately deferred until a measurement burst has actually started.
   * Configuring the channels here would put GPIO10..13 into analog mode and
   * kill the ANYEDGE interrupts that trigger the capture.
   */
  for (uint8_t i = 0; i < RTRH_ADC_CHANNELS; i++) {
    RtRhAdcChannel &ch = rtrh_adc_channels[i];

    err = adc_oneshot_io_to_channel(
        RTRH_ADC_GPIOS[i],
        &ch.unit,
        &ch.channel);

    if (err != ESP_OK) {
      ESP_LOGW(
          TAG,
          "RT/RH GPIO%d ADC mapping failed: %s",
          RTRH_ADC_GPIOS[i],
          esp_err_to_name(err));
      continue;
    }

    ch.configured = true;

    ESP_LOGI(
        TAG,
        "RT/RH GPIO%d mapped to ADC%d channel %d (deferred config)",
        RTRH_ADC_GPIOS[i],
        static_cast<int>(ch.unit) + 1,
        static_cast<int>(ch.channel));
  }
}

static bool configure_rtrh_pin_for_adc(uint8_t index)
{
  if (index >= RTRH_ADC_CHANNELS)
    return false;

  const RtRhAdcChannel &ch = rtrh_adc_channels[index];
  if (!ch.configured)
    return false;

  adc_oneshot_unit_handle_t handle = rtrh_adc_handle(ch.unit);
  if (handle == nullptr)
    return false;

  adc_oneshot_chan_cfg_t cfg = {};
  cfg.bitwidth = ADC_BITWIDTH_DEFAULT;
  cfg.atten = ADC_ATTEN_DB_12;

  const esp_err_t err =
      adc_oneshot_config_channel(handle, ch.channel, &cfg);

  if (err != ESP_OK) {
    ESP_LOGW(
        TAG,
        "RT/RH GPIO%d ADC config failed: %s",
        RTRH_ADC_GPIOS[index],
        esp_err_to_name(err));
    return false;
  }

  gpio_intr_disable(static_cast<gpio_num_t>(RTRH_ADC_GPIOS[index]));
  return true;
}

static void restore_rtrh_pin_digital(uint8_t index)
{
  if (index >= RTRH_ADC_CHANNELS)
    return;

  const gpio_num_t pin =
      static_cast<gpio_num_t>(RTRH_ADC_GPIOS[index]);

  gpio_config_t io = {};
  io.mode = GPIO_MODE_INPUT;
  io.pull_up_en = GPIO_PULLUP_DISABLE;
  io.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io.intr_type = GPIO_INTR_ANYEDGE;
  io.pin_bit_mask = 1ULL << pin;

  const esp_err_t err = gpio_config(&io);
  if (err != ESP_OK) {
    ESP_LOGE(
        TAG,
        "RT/RH GPIO%d digital restore failed: %s",
        RTRH_ADC_GPIOS[index],
        esp_err_to_name(err));
    return;
  }

  rtrh_pin_level[index] =
      static_cast<uint8_t>(gpio_get_level(pin));

  gpio_intr_enable(pin);
}

static void restore_rtrh_digital_inputs()
{
  gpio_config_t io = {};
  io.mode = GPIO_MODE_INPUT;
  io.pull_up_en = GPIO_PULLUP_DISABLE;
  io.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io.intr_type = GPIO_INTR_ANYEDGE;
  io.pin_bit_mask =
      (1ULL << PIN_RTRH0) |
      (1ULL << PIN_RTRH1) |
      (1ULL << PIN_RTRH2) |
      (1ULL << PIN_RTRH3);

  const esp_err_t err = gpio_config(&io);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "RT/RH digital restore failed: %s", esp_err_to_name(err));
    return;
  }

  rtrh_last_value = read_rtrh_state();
  rtrh_pin_level[0] = gpio_get_level(PIN_RTRH0);
  rtrh_pin_level[1] = gpio_get_level(PIN_RTRH1);
  rtrh_pin_level[2] = gpio_get_level(PIN_RTRH2);
  rtrh_pin_level[3] = gpio_get_level(PIN_RTRH3);
  rtrh_irqs_suspended = false;
}

static void rtrh_adc_task(void *arg)
{
  while (true) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    /*
     * First notification starts the sequence. All later notifications are
     * real falling edges from the currently selected digital trigger pin.
     */
    if (rtrh_adc_sequence_start) {
      rtrh_adc_sequence_start = false;
    } else if (!rtrh_adc_pending) {
      continue;
    }

    while (rtrh_adc_active && !rtrh_adc_ready) {
      const uint16_t sample_index = rtrh_adc_cycle;

      if (sample_index >= RTRH_ADC_SAMPLES) {
        rtrh_adc_active = false;
        rtrh_adc_ready = true;
        rtrh_adc_sequence++;
        rtrh_adc_waiting_for_edge = false;
        rtrh_adc_target_index = -1;
        rtrh_adc_trigger_index = -1;
        break;
      }

      const uint16_t samples_per_mode =
          RTRH_ADC_REPEATS * RTRH_ADC_PHASES * RTRH_ADC_CHANNELS;

      const uint8_t mode =
          static_cast<uint8_t>(sample_index / samples_per_mode);

      const uint16_t within_mode =
          static_cast<uint16_t>(sample_index % samples_per_mode);

      const uint8_t repeat =
          static_cast<uint8_t>(
              within_mode /
              (RTRH_ADC_PHASES * RTRH_ADC_CHANNELS));

      const uint8_t within_repeat =
          static_cast<uint8_t>(
              within_mode %
              (RTRH_ADC_PHASES * RTRH_ADC_CHANNELS));

      const uint8_t phase =
          static_cast<uint8_t>(
              within_repeat / RTRH_ADC_CHANNELS);

      const uint8_t channel =
          static_cast<uint8_t>(
              within_repeat % RTRH_ADC_CHANNELS);

      const uint16_t offset = RTRH_ADC_OFFSETS_US[phase];

      /*
       * Prepare ONLY the target pad for ADC before the timing reference.
       * A different, still-digital line supplies the falling-edge trigger.
       *
       * GPIO10 target -> trigger on GPIO11
       * GPIO11/12/13 target -> trigger on GPIO10
       */
      rtrh_adc_target_index = static_cast<int8_t>(channel);
      rtrh_adc_trigger_index =
          static_cast<int8_t>(channel == 0 ? 1 : 0);

      if (!configure_rtrh_pin_for_adc(channel)) {
        rtrh_adc_agg[mode][phase][channel].read_errors++;
        rtrh_adc_cycle =
            static_cast<uint16_t>(sample_index + 1);
        continue;
      }

      /*
       * Give the ADC/pad setup several complete sensor periods to settle.
       * The ISR counts REAL falling edges on the selected trigger line.
       */
      rtrh_adc_desired_mode = mode;
      rtrh_adc_edges_to_skip = RTRH_ADC_SKIP_EDGES;
      rtrh_adc_pending = false;
      rtrh_adc_waiting_for_edge = true;

      /*
       * Sleep until ISR captures the selected real falling edge.
       *
       * Do NOT wait forever: a missed edge, an unexpected period, or a pad
       * restore race must not leave rtrh_adc_active stuck true indefinitely.
       */
      const uint32_t notified =
          ulTaskNotifyTake(
              pdTRUE,
              pdMS_TO_TICKS(RTRH_ADC_EDGE_TIMEOUT_MS));

      if (notified == 0) {
        rtrh_adc_agg[mode][phase][channel].read_errors++;

        restore_rtrh_pin_digital(channel);

        ESP_LOGW(
            TAG,
            "RT/RH ADC edge timeout: mode=%s phase=%u GPIO%d; abort/rearm",
            mode == RTRH_MODE_SHORT ? "SHORT" : "LONG",
            static_cast<unsigned>(phase),
            RTRH_ADC_GPIOS[channel]);

        /*
         * Abort only this acquisition sequence.  The next real GPIO10 falling
         * edge may immediately start a fresh one.
         */
        reset_rtrh_adc_capture();
        break;
      }

      if (!rtrh_adc_pending || !rtrh_adc_active) {
        restore_rtrh_pin_digital(channel);

        /*
         * State changed while sleeping.  Rearm cleanly instead of looping with
         * a half-valid target/trigger setup.
         */
        reset_rtrh_adc_capture();
        break;
      }

      const uint32_t edge = rtrh_adc_reference_edge_us;
      const uint32_t target =
          static_cast<uint32_t>(edge + offset);

      while (static_cast<int32_t>(
                 target -
                 static_cast<uint32_t>(esp_timer_get_time())) > 0) {
        // intentional short spin
      }

      const uint32_t actual =
          static_cast<uint32_t>(esp_timer_get_time());

      int32_t late = static_cast<int32_t>(actual - target);
      if (late < 0)
        late = -late;

      RtRhAdcAggregate &agg =
          rtrh_adc_agg[mode][phase][channel];

      if (late > agg.max_lateness_us && late <= 65535)
        agg.max_lateness_us = static_cast<uint16_t>(late);

      const RtRhAdcChannel &ch = rtrh_adc_channels[channel];
      adc_oneshot_unit_handle_t handle = rtrh_adc_handle(ch.unit);

      int raw = 0;
      if (handle == nullptr ||
          adc_oneshot_read(handle, ch.channel, &raw) != ESP_OK) {
        agg.read_errors++;
      } else if (late > 10) {
        agg.rejected_late++;
      } else {
        if (raw < 0)
          raw = 0;
        if (raw > 65535)
          raw = 65535;

        const uint16_t value = static_cast<uint16_t>(raw);

        if (mode == RTRH_MODE_SHORT && channel == 3 &&
            phase < RTRH_ADC_PHASES && repeat < RTRH_ADC_REPEATS) {
          rtrh_short_g13_raw[phase][repeat] = value;
        }

        agg.sum += value;
        agg.count++;

        if (value < agg.min)
          agg.min = value;
        if (value > agg.max)
          agg.max = value;

        if (rtrh_adc_trigger_period_us != 0) {
          agg.period_sum += rtrh_adc_trigger_period_us;
          agg.period_count++;
        }
      }

      /*
       * Return the measured pin to ordinary digital input immediately.
       * The next iteration can then choose another target/trigger pair.
       */
      restore_rtrh_pin_digital(channel);

      rtrh_adc_pending = false;
      rtrh_adc_waiting_for_edge = false;
      rtrh_adc_cycle =
          static_cast<uint16_t>(sample_index + 1);
    }
  }
}

static void suspend_rtrh_irqs()
{
  if (rtrh_irqs_suspended)
    return;

  gpio_intr_disable(PIN_RTRH0);
  gpio_intr_disable(PIN_RTRH1);
  gpio_intr_disable(PIN_RTRH2);
  gpio_intr_disable(PIN_RTRH3);
  rtrh_irqs_suspended = true;
}

static void resume_rtrh_irqs()
{
  rtrh_last_value = read_rtrh_state();

  gpio_intr_enable(PIN_RTRH0);
  gpio_intr_enable(PIN_RTRH1);
  gpio_intr_enable(PIN_RTRH2);
  gpio_intr_enable(PIN_RTRH3);

  rtrh_irqs_suspended = false;
}


static inline void IRAM_ATTR clear_passive_block(
    RtRhPassiveBlock &b)
{
  b.mode = 0xFF;
  b.count = 0;
  b.period_sum = 0;
  b.low_sum = 0;
  b.g13_delay_sum = 0;
  b.g13_delay_count = 0;

  b.period_min = 0xFFFF;
  b.period_max = 0;
  b.low_min = 0xFFFF;
  b.low_max = 0;
  b.g13_delay_min = 0xFFFF;
  b.g13_delay_max = 0;

  for (uint8_t i = 0; i < RTRH_PASSIVE_DELAY_BINS; i++)
    b.g13_delay_hist[i] = 0;
}

static inline void IRAM_ATTR reset_rtrh_passive_working(
    uint32_t now,
    uint8_t state)
{
  rtrh_passive_collecting = true;
  rtrh_passive_overflow = false;
  rtrh_passive_block_count = 0;

  rtrh_passive_current_mode = 0xFF;
  rtrh_passive_candidate_mode = 0xFF;
  rtrh_passive_candidate_run = 0;

  rtrh_passive_last_g10_fall_us = 0;
  rtrh_passive_g10_rise_us = 0;
  rtrh_passive_g13_rise_us = 0;
  rtrh_passive_have_g10_rise = false;
  rtrh_passive_have_g13_rise = false;

  for (uint8_t i = 0; i < RTRH_PASSIVE_MAX_BLOCKS; i++)
    clear_passive_block(rtrh_passive_blocks[i]);

  rtrh_passive_last_any_us = now;
  rtrh_passive_last_state = state;
}

static inline void IRAM_ATTR start_passive_block(uint8_t mode)
{
  if (rtrh_passive_block_count >= RTRH_PASSIVE_MAX_BLOCKS) {
    rtrh_passive_overflow = true;
    // Do not point current_mode at a block that was not actually stored.
    rtrh_passive_current_mode = 0xFF;
    return;
  }

  RtRhPassiveBlock &b =
      rtrh_passive_blocks[rtrh_passive_block_count++];
  clear_passive_block(b);
  b.mode = mode;
  rtrh_passive_current_mode = mode;
}

static inline void IRAM_ATTR add_passive_cycle(
    RtRhPassiveBlock &b,
    uint32_t period,
    uint32_t low,
    bool have_delay,
    uint32_t delay)
{
  const uint16_t p =
      period > 65535U ? 65535U : static_cast<uint16_t>(period);
  const uint16_t l =
      low > 65535U ? 65535U : static_cast<uint16_t>(low);

  if (b.count != 0xFFFF)
    b.count++;

  b.period_sum += p;
  b.low_sum += l;

  if (p < b.period_min)
    b.period_min = p;
  if (p > b.period_max)
    b.period_max = p;

  if (l < b.low_min)
    b.low_min = l;
  if (l > b.low_max)
    b.low_max = l;

  if (have_delay && delay <= 65535U) {
    const uint16_t d = static_cast<uint16_t>(delay);
    b.g13_delay_sum += d;

    if (b.g13_delay_count != 0xFFFF)
      b.g13_delay_count++;

    if (d < b.g13_delay_min)
      b.g13_delay_min = d;
    if (d > b.g13_delay_max)
      b.g13_delay_max = d;

    const uint8_t bin =
        d < (RTRH_PASSIVE_DELAY_BINS - 1)
            ? static_cast<uint8_t>(d)
            : static_cast<uint8_t>(RTRH_PASSIVE_DELAY_BINS - 1);

    if (b.g13_delay_hist[bin] != 0xFFFF)
      b.g13_delay_hist[bin]++;
  }
}

/*
 * Called from loop() only after the four lines have been quiet for >100 ms.
 * At that point the physical measurement burst is over, so copying the small
 * summary is safe and does not perturb the live waveform.
 */
static void finalize_rtrh_passive_burst()
{
  if (!rtrh_passive_collecting || rtrh_passive_block_count == 0)
    return;

  const uint8_t n = rtrh_passive_block_count;

  rtrh_passive_snapshot.block_count = n;
  rtrh_passive_snapshot.overflow = rtrh_passive_overflow;

  for (uint8_t i = 0; i < RTRH_PASSIVE_MAX_BLOCKS; i++) {
    if (i < n)
      rtrh_passive_snapshot.blocks[i] = rtrh_passive_blocks[i];
    else
      clear_passive_block(rtrh_passive_snapshot.blocks[i]);
  }

  rtrh_passive_snapshot.sequence++;
  rtrh_passive_snapshot_ready = true;
  rtrh_passive_snapshot_consumed = false;
  rtrh_passive_collecting = false;
}

static void IRAM_ATTR rtrh_gpio_isr(void *arg)
{
  const intptr_t encoded = reinterpret_cast<intptr_t>(arg);
  if (encoded < 1 || encoded > 4)
    return;

  const uint8_t pin_index =
      static_cast<uint8_t>(encoded - 1);

  const gpio_num_t pin =
      static_cast<gpio_num_t>(RTRH_ADC_GPIOS[pin_index]);

  const uint8_t level =
      static_cast<uint8_t>(gpio_get_level(pin));

  const uint8_t previous =
      rtrh_pin_level[pin_index];

  if (level == previous)
    return;

  rtrh_pin_level[pin_index] = level;

  const uint32_t now =
      static_cast<uint32_t>(esp_timer_get_time());

  /*
   * Passive complete-burst decoder.
   *
   * A mode is accepted only after 8 consecutive cycles of the same class.
   * A one-off 140..160 us glitch inside a SHORT block therefore cannot create
   * a fake LONG block.
   */
  const uint8_t state = read_rtrh_state();

  const uint32_t previous_any = rtrh_passive_last_any_us;
  if (!rtrh_passive_collecting ||
      previous_any == 0 ||
      static_cast<uint32_t>(now - previous_any) > RTRH_PASSIVE_QUIET_US) {
    reset_rtrh_passive_working(now, state);
  } else {
    rtrh_passive_last_any_us = now;
  }

  const uint8_t old_state = rtrh_passive_last_state;
  if (state != old_state) {
    const bool old_g10 = (old_state & 0x01) != 0;
    const bool new_g10 = (state & 0x01) != 0;
    const bool old_g13 = (old_state & 0x08) != 0;
    const bool new_g13 = (state & 0x08) != 0;

    if (!old_g10 && new_g10) {
      rtrh_passive_g10_rise_us = now;
      rtrh_passive_have_g10_rise = true;

      if (new_g13) {
        rtrh_passive_g13_rise_us = now;
        rtrh_passive_have_g13_rise = true;
      } else {
        rtrh_passive_have_g13_rise = false;
      }
    }

    if (!old_g13 && new_g13 && rtrh_passive_have_g10_rise) {
      rtrh_passive_g13_rise_us = now;
      rtrh_passive_have_g13_rise = true;
    }

    if (old_g10 && !new_g10) {
      const uint32_t previous_fall = rtrh_passive_last_g10_fall_us;

      if (previous_fall != 0 &&
          rtrh_passive_have_g10_rise) {
        const uint32_t period =
            static_cast<uint32_t>(now - previous_fall);
        const uint32_t low =
            static_cast<uint32_t>(
                rtrh_passive_g10_rise_us - previous_fall);

        const bool have_delay =
            rtrh_passive_have_g13_rise &&
            static_cast<int32_t>(
                rtrh_passive_g13_rise_us -
                rtrh_passive_g10_rise_us) >= 0;

        const uint32_t delay =
            have_delay
                ? static_cast<uint32_t>(
                      rtrh_passive_g13_rise_us -
                      rtrh_passive_g10_rise_us)
                : 0;

        uint8_t mode = 0xFF;
        if (period >= RTRH_SHORT_MIN_US &&
            period <= RTRH_SHORT_MAX_US)
          mode = RTRH_MODE_SHORT;
        else if (period >= RTRH_LONG_MIN_US &&
                 period <= RTRH_LONG_MAX_US)
          mode = RTRH_MODE_LONG;

        if (mode == 0xFF) {
          // Invalid single cycles are ignored and do not terminate a block.
          rtrh_passive_candidate_mode = 0xFF;
          rtrh_passive_candidate_run = 0;
        } else if (mode == rtrh_passive_current_mode &&
                   rtrh_passive_block_count != 0) {
          // Returned to the current block before a different mode achieved
          // the 8-cycle lock: treat the excursion as a glitch.
          rtrh_passive_candidate_mode = 0xFF;
          rtrh_passive_candidate_run = 0;

          add_passive_cycle(
              rtrh_passive_blocks[rtrh_passive_block_count - 1],
              period,
              low,
              have_delay,
              delay);
        } else {
          if (mode == rtrh_passive_candidate_mode) {
            if (rtrh_passive_candidate_run < 255)
              rtrh_passive_candidate_run++;
          } else {
            rtrh_passive_candidate_mode = mode;
            rtrh_passive_candidate_run = 1;
          }

          if (rtrh_passive_candidate_run >= RTRH_PASSIVE_LOCK_CYCLES) {
            start_passive_block(mode);
            rtrh_passive_candidate_mode = 0xFF;
            rtrh_passive_candidate_run = 0;

            if (rtrh_passive_block_count != 0 &&
                rtrh_passive_block_count <= RTRH_PASSIVE_MAX_BLOCKS) {
              add_passive_cycle(
                  rtrh_passive_blocks[rtrh_passive_block_count - 1],
                  period,
                  low,
                  have_delay,
                  delay);
            }
          }
        }
      }

      rtrh_passive_last_g10_fall_us = now;
      rtrh_passive_have_g10_rise = false;
      rtrh_passive_have_g13_rise = false;
    }

    rtrh_passive_last_state = state;
  }

  /*
   * Once a target ADC pad has been prepared, only the selected other pin is
   * used as timing reference. Falling edges are counted in hardware time.
   */
  if (false && previous && !level) {
    const uint32_t previous_fall = rtrh_last_fall_us[pin_index];
    rtrh_last_fall_us[pin_index] = now;

    if (rtrh_adc_active &&
        rtrh_adc_waiting_for_edge &&
        !rtrh_adc_pending &&
        pin_index == static_cast<uint8_t>(rtrh_adc_trigger_index) &&
        previous_fall != 0) {

      const uint32_t period =
          static_cast<uint32_t>(now - previous_fall);

      bool wanted = false;

      if (rtrh_adc_desired_mode == RTRH_MODE_SHORT) {
        wanted =
            period >= RTRH_SHORT_MIN_US &&
            period <= RTRH_SHORT_MAX_US;
      } else {
        wanted =
            period >= RTRH_LONG_MIN_US &&
            period <= RTRH_LONG_MAX_US;
      }

      if (wanted) {
        if (rtrh_adc_edges_to_skip > 0) {
          rtrh_adc_edges_to_skip--;
        } else {
          rtrh_adc_reference_edge_us = now;
          rtrh_adc_trigger_period_us =
              period > 65535U ? 65535U : static_cast<uint16_t>(period);
          rtrh_adc_pending = true;
          rtrh_adc_waiting_for_edge = false;

          if (rtrh_adc_task_handle != nullptr) {
            BaseType_t higher_priority_woken = pdFALSE;
            vTaskNotifyGiveFromISR(
                rtrh_adc_task_handle,
                &higher_priority_woken);
            if (higher_priority_woken)
              portYIELD_FROM_ISR();
          }
        }
      }
    }
  }

  /*
   * Don't try to build a four-line digital state trace while one of those
   * four pads is intentionally in analog mode. The earlier digital captures
   * have already characterized the ~13 kHz waveform; ADC timing is the goal
   * during this acquisition.
   */
  if (rtrh_adc_active)
    return;

  if (rtrh_capture_ready)
    return;

  const uint8_t value = read_rtrh_state();

  if (value == rtrh_last_value)
    return;

  rtrh_last_value = value;

  if (!rtrh_capturing) {
    rtrh_capturing = true;
    rtrh_start_us = now;
    rtrh_sample_count = 0;
    rtrh_overflow = false;
  }

  const uint16_t index = rtrh_sample_count;
  if (index >= RTRH_MAX_SAMPLES) {
    rtrh_overflow = true;
    rtrh_capturing = false;
    rtrh_capture_ready = true;
    rtrh_sequence++;
    return;
  }

  rtrh_samples[index].t_us =
      static_cast<uint32_t>(now - rtrh_start_us);
  rtrh_samples[index].value = value;
  rtrh_sample_count = index + 1;
}

class RtRhCaptureHandler : public web_server_idf::AsyncWebHandler {
 public:
  bool canHandle(web_server_idf::AsyncWebServerRequest *request) const override
  {
    if (request->method() != HTTP_GET)
      return false;
    char url[web_server_idf::AsyncWebServerRequest::URL_BUF_SIZE];
    return request->url_to(url) == "/rt_rh_capture.csv";
  }

  void handleRequest(web_server_idf::AsyncWebServerRequest *request) override
  {
    if (!rtrh_capture_ready || rtrh_sample_count == 0) {
      request->send(204, "text/plain", nullptr);
      return;
    }

    httpd_req_t *req = *request;
    httpd_resp_set_status(req, "200 OK");
    httpd_resp_set_type(req, "text/csv");
    httpd_resp_set_hdr(
        req,
        "Content-Disposition",
        "attachment; filename=\"rt_rh_capture.csv\"");

    static constexpr char HEADER[] =
        "sequence,t_us,gpio10,gpio11,gpio12,gpio13,state,overflow\n";
    esp_err_t err = httpd_resp_send_chunk(req, HEADER, sizeof(HEADER) - 1);

    const uint16_t count = rtrh_sample_count;
    const uint32_t sequence = rtrh_sequence;
    const bool overflow = rtrh_overflow;

    // Batch many CSV rows into each TCP chunk. Sending one HTTP chunk per
    // edge (~3000-4000 chunks) needlessly stresses lwIP on the ESP32-S2.
    // Keep these buffers out of the small ESP-IDF HTTP server task stack.
    // The previous 2 KiB automatic buffer caused vApplicationStackOverflowHook.
    static char chunk[128];
    static char line[80];
    size_t used = 0;

    for (uint16_t i = 0; i < count && err == ESP_OK; i++) {
      const uint32_t t = rtrh_samples[i].t_us;
      const uint8_t v = rtrh_samples[i].value;

      const int n = snprintf(
          line, sizeof(line),
          "%lu,%lu,%u,%u,%u,%u,0x%02X,%u\n",
          static_cast<unsigned long>(sequence),
          static_cast<unsigned long>(t),
          (v & 0x01) ? 1U : 0U,
          (v & 0x02) ? 1U : 0U,
          (v & 0x04) ? 1U : 0U,
          (v & 0x08) ? 1U : 0U,
          v,
          overflow ? 1U : 0U);

      if (n <= 0)
        continue;

      const size_t line_len = static_cast<size_t>(n);

      if (used + line_len > sizeof(chunk)) {
        err = httpd_resp_send_chunk(req, chunk, used);
        used = 0;
      }

      if (err == ESP_OK && line_len <= sizeof(chunk)) {
        memcpy(chunk + used, line, line_len);
        used += line_len;
      }
    }

    if (err == ESP_OK && used != 0)
      err = httpd_resp_send_chunk(req, chunk, used);

    if (err == ESP_OK)
      httpd_resp_send_chunk(req, nullptr, 0);

    // Rearm only after the frozen capture has been downloaded.
    rtrh_sample_count = 0;
    rtrh_overflow = false;
    rtrh_capture_ready = false;
    rtrh_capturing = false;
    resume_rtrh_irqs();

    if (err != ESP_OK)
      ESP_LOGW(TAG, "rt_rh_capture.csv client disconnected (%d)", err);
  }
};

static RtRhCaptureHandler rtrh_capture_handler;

static uint16_t median_u16_ignore_missing(
    const uint16_t *values,
    uint8_t count)
{
  uint16_t tmp[RTRH_ADC_REPEATS];
  uint8_t n = 0;

  for (uint8_t i = 0; i < count && i < RTRH_ADC_REPEATS; i++) {
    if (values[i] != 0xFFFF)
      tmp[n++] = values[i];
  }

  if (n == 0)
    return 0;

  // Tiny insertion sort; n <= 8.
  for (uint8_t i = 1; i < n; i++) {
    const uint16_t v = tmp[i];
    int j = static_cast<int>(i) - 1;

    while (j >= 0 && tmp[j] > v) {
      tmp[j + 1] = tmp[j];
      j--;
    }

    tmp[j + 1] = v;
  }

  if (n & 1)
    return tmp[n / 2];

  return static_cast<uint16_t>(
      (static_cast<uint32_t>(tmp[n / 2 - 1]) +
       static_cast<uint32_t>(tmp[n / 2])) / 2U);
}


class RtRhTimingHandler : public web_server_idf::AsyncWebHandler {
 public:
  bool canHandle(web_server_idf::AsyncWebServerRequest *request) const override
  {
    if (request->method() != HTTP_GET)
      return false;
    char url[web_server_idf::AsyncWebServerRequest::URL_BUF_SIZE];
    return request->url_to(url) == "/rt_rh_timing.csv";
  }

  void handleRequest(web_server_idf::AsyncWebServerRequest *request) override
  {
    if (!rtrh_passive_snapshot_ready || rtrh_passive_snapshot_consumed) {
      request->send(204, "text/plain", nullptr);
      return;
    }

    const RtRhPassiveSnapshot s = rtrh_passive_snapshot;

    httpd_req_t *req = *request;
    httpd_resp_set_status(req, "200 OK");
    httpd_resp_set_type(req, "text/csv");
    httpd_resp_set_hdr(
        req,
        "Content-Disposition",
        "attachment; filename=\"rt_rh_timing.csv\"");

    static constexpr char HEADER[] =
        "sequence,block,mode,mode_name,count,"
        "period_mean_us,period_min_us,period_max_us,"
        "low_mean_us,low_min_us,low_max_us,"
        "g13_delay_count,g13_delay_mean_us,g13_delay_min_us,g13_delay_max_us,"
        "d0,d1,d2,d3,d4,d5,d6,d7plus,overflow\n";

    esp_err_t err =
        httpd_resp_send_chunk(req, HEADER, sizeof(HEADER) - 1);

    static char line[256];

    for (uint8_t i = 0; i < s.block_count && err == ESP_OK; i++) {
      const RtRhPassiveBlock &b = s.blocks[i];

      const float period_mean =
          b.count
              ? static_cast<float>(b.period_sum) /
                    static_cast<float>(b.count)
              : 0.0f;

      const float low_mean =
          b.count
              ? static_cast<float>(b.low_sum) /
                    static_cast<float>(b.count)
              : 0.0f;

      const float delay_mean =
          b.g13_delay_count
              ? static_cast<float>(b.g13_delay_sum) /
                    static_cast<float>(b.g13_delay_count)
              : 0.0f;

      const int n = snprintf(
          line,
          sizeof(line),
          "%lu,%u,%u,%s,%u,%.3f,%u,%u,%.3f,%u,%u,"
          "%u,%.3f,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\n",
          static_cast<unsigned long>(s.sequence),
          static_cast<unsigned>(i),
          static_cast<unsigned>(b.mode),
          b.mode == RTRH_MODE_SHORT ? "short" :
              (b.mode == RTRH_MODE_LONG ? "long" : "unknown"),
          static_cast<unsigned>(b.count),
          period_mean,
          b.count ? static_cast<unsigned>(b.period_min) : 0U,
          static_cast<unsigned>(b.period_max),
          low_mean,
          b.count ? static_cast<unsigned>(b.low_min) : 0U,
          static_cast<unsigned>(b.low_max),
          static_cast<unsigned>(b.g13_delay_count),
          delay_mean,
          b.g13_delay_count
              ? static_cast<unsigned>(b.g13_delay_min)
              : 0U,
          static_cast<unsigned>(b.g13_delay_max),
          static_cast<unsigned>(b.g13_delay_hist[0]),
          static_cast<unsigned>(b.g13_delay_hist[1]),
          static_cast<unsigned>(b.g13_delay_hist[2]),
          static_cast<unsigned>(b.g13_delay_hist[3]),
          static_cast<unsigned>(b.g13_delay_hist[4]),
          static_cast<unsigned>(b.g13_delay_hist[5]),
          static_cast<unsigned>(b.g13_delay_hist[6]),
          static_cast<unsigned>(b.g13_delay_hist[7]),
          s.overflow ? 1U : 0U);

      if (n <= 0 || static_cast<size_t>(n) >= sizeof(line)) {
        err = ESP_FAIL;
        break;
      }

      err = httpd_resp_send_chunk(req, line, n);
    }

    if (err == ESP_OK)
      err = httpd_resp_send_chunk(req, nullptr, 0);

    if (err == ESP_OK)
      rtrh_passive_snapshot_consumed = true;
  }
};

static RtRhTimingHandler rtrh_timing_handler;

class RtRhAdcHandler : public web_server_idf::AsyncWebHandler {
 public:
  bool canHandle(web_server_idf::AsyncWebServerRequest *request) const override
  {
    if (request->method() != HTTP_GET)
      return false;

    char url[web_server_idf::AsyncWebServerRequest::URL_BUF_SIZE];
    return request->url_to(url) == "/rt_rh_adc.csv";
  }

  void handleRequest(web_server_idf::AsyncWebServerRequest *request) override
  {
    bool have_snapshot = false;

    portENTER_CRITICAL(&rtrh_adc_snapshot_mux);
    if (rtrh_adc_snapshot_ready && !rtrh_adc_snapshot_in_use) {
      rtrh_adc_snapshot_in_use = true;
      have_snapshot = true;
    }
    portEXIT_CRITICAL(&rtrh_adc_snapshot_mux);

    if (!have_snapshot) {
      request->send(204, "text/plain", nullptr);
      return;
    }

    httpd_req_t *req = *request;
    httpd_resp_set_status(req, "200 OK");
    httpd_resp_set_type(req, "text/csv");
    httpd_resp_set_hdr(
        req,
        "Content-Disposition",
        "attachment; filename=\"rt_rh_adc.csv\"");

    static constexpr char HEADER[] =
        "sequence,mode,mode_name,phase,gpio,offset_us,"
        "count,mean,min,max,period_mean_us,max_lateness_us,"
        "rejected_late,read_errors,median,"
        "raw0,raw1,raw2,raw3,raw4,raw5,raw6,raw7\n";

    esp_err_t err =
        httpd_resp_send_chunk(req, HEADER, sizeof(HEADER) - 1);

    static char chunk[256];
    static char line[224];
    size_t used = 0;

    const uint32_t sequence = rtrh_adc_snapshot_sequence;

    for (uint8_t mode = 0; mode < RTRH_ADC_MODES && err == ESP_OK; mode++) {
      for (uint8_t phase = 0; phase < RTRH_ADC_PHASES && err == ESP_OK; phase++) {
        for (uint8_t channel = 0;
             channel < RTRH_ADC_CHANNELS && err == ESP_OK;
             channel++) {

          const RtRhAdcAggregate &agg =
              rtrh_adc_snapshot[mode][phase][channel];

          const uint32_t mean =
              agg.count ? agg.sum / agg.count : 0;

          const uint32_t period_mean =
              agg.period_count
                  ? agg.period_sum / agg.period_count
                  : 0;

          const uint16_t min_value =
              agg.count ? agg.min : 0;

          uint16_t median_value = 0;

          if (mode == RTRH_MODE_SHORT && channel == 3) {
            median_value =
                median_u16_ignore_missing(
                    rtrh_short_g13_raw_snapshot[phase],
                    RTRH_ADC_REPEATS);
          }

          char raw_fields[64] = ",,,,,,,,";
          if (mode == RTRH_MODE_SHORT && channel == 3) {
            char *rp = raw_fields;
            size_t remain = sizeof(raw_fields);

            for (uint8_t rr = 0; rr < RTRH_ADC_REPEATS; rr++) {
              const uint16_t rv =
                  rtrh_short_g13_raw_snapshot[phase][rr];

              int rn;
              if (rv == 0xFFFF)
                rn = snprintf(rp, remain, ",");
              else
                rn = snprintf(rp, remain, ",%u", static_cast<unsigned>(rv));

              if (rn <= 0 || static_cast<size_t>(rn) >= remain)
                break;

              rp += rn;
              remain -= static_cast<size_t>(rn);
            }
          }

          const int n = snprintf(
              line,
              sizeof(line),
              "%lu,%u,%s,%u,%d,%u,%u,%lu,%u,%u,%lu,%u,%u,%u,%u%s\n",
              static_cast<unsigned long>(sequence),
              static_cast<unsigned>(mode),
              mode == RTRH_MODE_SHORT ? "short" : "long",
              static_cast<unsigned>(phase),
              RTRH_ADC_GPIOS[channel],
              static_cast<unsigned>(RTRH_ADC_OFFSETS_US[phase]),
              static_cast<unsigned>(agg.count),
              static_cast<unsigned long>(mean),
              static_cast<unsigned>(min_value),
              static_cast<unsigned>(agg.max),
              static_cast<unsigned long>(period_mean),
              static_cast<unsigned>(agg.max_lateness_us),
              static_cast<unsigned>(agg.rejected_late),
              static_cast<unsigned>(agg.read_errors),
              static_cast<unsigned>(median_value),
              raw_fields);

          if (n <= 0)
            continue;

          const size_t len = static_cast<size_t>(n);

          if (used + len > sizeof(chunk)) {
            err = httpd_resp_send_chunk(req, chunk, used);
            used = 0;
          }

          if (err == ESP_OK && len <= sizeof(chunk)) {
            memcpy(chunk + used, line, len);
            used += len;
          }
        }
      }
    }

    if (err == ESP_OK && used != 0)
      err = httpd_resp_send_chunk(req, chunk, used);

    if (err == ESP_OK)
      httpd_resp_send_chunk(req, nullptr, 0);

    portENTER_CRITICAL(&rtrh_adc_snapshot_mux);
    rtrh_adc_snapshot_in_use = false;

    // A successfully downloaded snapshot is consumed.  Until the next
    // completed ADC sequence arrives, repeated GETs return 204 instead of
    // silently serving stale data under a new display label.
    if (err == ESP_OK)
      rtrh_adc_snapshot_ready = false;

    portEXIT_CRITICAL(&rtrh_adc_snapshot_mux);

    if (err != ESP_OK)
      ESP_LOGW(TAG, "rt_rh_adc.csv client disconnected (%d)", err);
  }
};

static RtRhAdcHandler rtrh_adc_handler;


/*
 * ============================================================================
 * Setup
 * ============================================================================
 */

void BusSniffer::setup()
{
  last_capture_mutex =
      xSemaphoreCreateMutex();


  gpio_config_t io = {};


  io.mode =
      GPIO_MODE_INPUT;

  io.pull_up_en =
      GPIO_PULLUP_DISABLE;

  io.pull_down_en =
      GPIO_PULLDOWN_DISABLE;

  io.intr_type =
      GPIO_INTR_DISABLE;


  io.pin_bit_mask =
      (1ULL << PIN_SCL) |
      (1ULL << PIN_SDA) |
      (1ULL << PIN_OTHER) |
      (1ULL << PIN_RTRH0) |
      (1ULL << PIN_RTRH1) |
      (1ULL << PIN_RTRH2) |
      (1ULL << PIN_RTRH3);


  esp_err_t err =
      gpio_config(
          &io
      );


  if (err != ESP_OK) {

    ESP_LOGE(
        TAG,
        "gpio_config failed: %d",
        err
    );

    return;
  }


  /*
   * Nur SCL und SDA erzeugen Interrupts.
   */

  gpio_set_intr_type(
      PIN_SCL,
      GPIO_INTR_ANYEDGE
  );


  gpio_set_intr_type(
      PIN_SDA,
      GPIO_INTR_ANYEDGE
  );


  gpio_set_intr_type(PIN_RTRH0, GPIO_INTR_ANYEDGE);
  gpio_set_intr_type(PIN_RTRH1, GPIO_INTR_ANYEDGE);
  gpio_set_intr_type(PIN_RTRH2, GPIO_INTR_ANYEDGE);
  gpio_set_intr_type(PIN_RTRH3, GPIO_INTR_ANYEDGE);


  last_value =
      read_gpio_state();


  capture_initial_value =
      last_value;

  rtrh_last_value = read_rtrh_state();
  rtrh_pin_level[0] = gpio_get_level(PIN_RTRH0);
  rtrh_pin_level[1] = gpio_get_level(PIN_RTRH1);
  rtrh_pin_level[2] = gpio_get_level(PIN_RTRH2);
  rtrh_pin_level[3] = gpio_get_level(PIN_RTRH3);

  // Passive-block build: do not initialize ADC units and do not start the
  // ADC worker. GPIO10..13 remain ordinary digital inputs for the entire run.
  reset_rtrh_adc_capture();


  last_edge =
      static_cast<uint32_t>(
          esp_timer_get_time()
      );


  err =
      gpio_install_isr_service(0);


  if (
      err != ESP_OK &&
      err != ESP_ERR_INVALID_STATE
  ) {

    ESP_LOGE(
        TAG,
        "gpio_install_isr_service failed: %d",
        err
    );

    return;
  }


  err =
      gpio_isr_handler_add(
          PIN_SCL,
          gpio_isr,
          nullptr
      );


  if (err != ESP_OK) {

    ESP_LOGE(
        TAG,
        "SCL ISR failed: %d",
        err
    );

    return;
  }


  err =
      gpio_isr_handler_add(
          PIN_SDA,
          gpio_isr,
          nullptr
      );


  if (err != ESP_OK) {

    ESP_LOGE(
        TAG,
        "SDA ISR failed: %d",
        err
    );

    return;
  }


  static constexpr gpio_num_t RTRH_PINS[] = {
      PIN_RTRH0, PIN_RTRH1, PIN_RTRH2, PIN_RTRH3};

  for (uint8_t i = 0; i < 4; i++) {
    const gpio_num_t pin = RTRH_PINS[i];
    err = gpio_isr_handler_add(
        pin,
        rtrh_gpio_isr,
        reinterpret_cast<void *>(static_cast<intptr_t>(i + 1)));
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "RT/RH ISR GPIO%d failed: %d", static_cast<int>(pin), err);
      return;
    }
  }


  if (
      web_server_base::global_web_server_base !=
      nullptr
  ) {

    web_server_base::
        global_web_server_base->
        add_handler(
            &capture_handler
        );

    web_server_base::global_web_server_base->add_handler(
        &rtrh_capture_handler);

    web_server_base::global_web_server_base->add_handler(
        &rtrh_timing_handler);

  } else {

    ESP_LOGW(
        TAG,
        "web_server_base unavailable"
    );
  }


  /*
   * Diagnosezähler mit 0 initialisieren.
   */

  if (this->crc_errors_sensor_ != nullptr)
    this->crc_errors_sensor_->publish_state(0);


  if (this->frame_errors_sensor_ != nullptr)
    this->frame_errors_sensor_->publish_state(0);


  ESP_LOGI(
      TAG,
      "Passive CO2 sniffer ready "
      "(I2C 0x62, SCL GPIO40, SDA GPIO39)"
  );


  ESP_LOGD(
      TAG,
      "Raw captures at /capture; RT/RH edges /rt_rh_capture.csv; phase ADC /rt_rh_adc.csv"
  );
}


/*
 * ============================================================================
 * Loop
 * ============================================================================
 */

void BusSniffer::loop()
{
  /*
   * End-of-burst detector.  The ISR only accumulates tiny block summaries.
   * Once the lines have been quiet for 100 ms, freeze the complete burst.
   */
  if (rtrh_passive_collecting &&
      rtrh_passive_block_count != 0) {
    const uint32_t now =
        static_cast<uint32_t>(esp_timer_get_time());
    const uint32_t last = rtrh_passive_last_any_us;

    if (last != 0 &&
        static_cast<uint32_t>(now - last) > RTRH_PASSIVE_QUIET_US) {
      finalize_rtrh_passive_burst();
    }
  }

  static uint32_t last_passive_sequence = 0;

  if (rtrh_passive_snapshot_ready &&
      rtrh_passive_snapshot.sequence != last_passive_sequence) {
    const RtRhPassiveSnapshot s = rtrh_passive_snapshot;
    last_passive_sequence = s.sequence;

    ESP_LOGI(
        TAG,
        "RT/RH passive burst %lu: %u stable blocks%s",
        static_cast<unsigned long>(s.sequence),
        static_cast<unsigned>(s.block_count),
        s.overflow ? " OVERFLOW" : "");

    for (uint8_t i = 0; i < s.block_count; i++) {
      const RtRhPassiveBlock &b = s.blocks[i];

      const float period_mean =
          b.count
              ? static_cast<float>(b.period_sum) /
                    static_cast<float>(b.count)
              : 0.0f;

      const float low_mean =
          b.count
              ? static_cast<float>(b.low_sum) /
                    static_cast<float>(b.count)
              : 0.0f;

      const float delay_mean =
          b.g13_delay_count
              ? static_cast<float>(b.g13_delay_sum) /
                    static_cast<float>(b.g13_delay_count)
              : 0.0f;

      ESP_LOGI(
          TAG,
          "  block %u %s: n=%u period=%.3f us low=%.3f us G13delay=%.3f us (%u)",
          static_cast<unsigned>(i),
          b.mode == RTRH_MODE_SHORT ? "SHORT" :
              (b.mode == RTRH_MODE_LONG ? "LONG" : "?"),
          static_cast<unsigned>(b.count),
          period_mean,
          low_mean,
          delay_mean,
          static_cast<unsigned>(b.g13_delay_count));
    }
  }

  /*
   * Diagnostic build: do not publish RT Temperature yet.  We first need to
   * identify which LONG block is the actual RT conversion.
   */
  static uint32_t last_logged_adc_sequence = 0;

  if (false && rtrh_adc_ready &&
      rtrh_adc_sequence != last_logged_adc_sequence) {
    last_logged_adc_sequence = rtrh_adc_sequence;

    ESP_LOGI(
        TAG,
        "RT/RH mode ADC ready: %u samples, sequence %lu",
        RTRH_ADC_SAMPLES,
        static_cast<unsigned long>(rtrh_adc_sequence));

    // The LONG cycle duration tracks the NTC temperature almost perfectly.
    // Average all valid LONG-period observations from this sequence.
    uint32_t long_period_sum = 0;
    uint32_t long_period_count = 0;

    for (uint8_t phase = 0; phase < RTRH_ADC_PHASES; phase++) {
      for (uint8_t channel = 0; channel < RTRH_ADC_CHANNELS; channel++) {
        const RtRhAdcAggregate &agg =
            rtrh_adc_agg[RTRH_MODE_LONG][phase][channel];

        long_period_sum += agg.period_sum;
        long_period_count += agg.period_count;
      }
    }

    if (long_period_count != 0) {
      const float long_period_us =
          static_cast<float>(long_period_sum) /
          static_cast<float>(long_period_count);

      const float temperature_c =
          RTRH_TEMP_A * long_period_us * long_period_us +
          RTRH_TEMP_B * long_period_us +
          RTRH_TEMP_C;

      ESP_LOGD(
          TAG,
          "ADC LONG period diagnostic: %.2f us -> %.2f C (%lu observations)",
          long_period_us,
          temperature_c,
          static_cast<unsigned long>(long_period_count));
    } else {
      ESP_LOGW(TAG, "RT temperature unavailable: no valid LONG period");
    }

    for (uint8_t mode = 0; mode < RTRH_ADC_MODES; mode++) {
      for (uint8_t phase = 0; phase < RTRH_ADC_PHASES; phase++) {
        const RtRhAdcAggregate &a0 = rtrh_adc_agg[mode][phase][0];
        const RtRhAdcAggregate &a1 = rtrh_adc_agg[mode][phase][1];
        const RtRhAdcAggregate &a2 = rtrh_adc_agg[mode][phase][2];
        const RtRhAdcAggregate &a3 = rtrh_adc_agg[mode][phase][3];

        const uint32_t period_sum =
            a0.period_sum + a1.period_sum + a2.period_sum + a3.period_sum;
        const uint32_t period_count =
            a0.period_count + a1.period_count +
            a2.period_count + a3.period_count;

        uint16_t max_late = a0.max_lateness_us;
        if (a1.max_lateness_us > max_late) max_late = a1.max_lateness_us;
        if (a2.max_lateness_us > max_late) max_late = a2.max_lateness_us;
        if (a3.max_lateness_us > max_late) max_late = a3.max_lateness_us;

        ESP_LOGI(
            TAG,
            "RT/RH %s phase %u (+%u us, period~%lu): "
            "G10=%lu(%u) G11=%lu(%u) G12=%lu(%u) G13=%lu(%u), max_late=%u us",
            mode == RTRH_MODE_SHORT ? "SHORT" : "LONG",
            phase,
            RTRH_ADC_OFFSETS_US[phase],
            period_count
                ? static_cast<unsigned long>(period_sum / period_count)
                : 0UL,
            a0.count ? static_cast<unsigned long>(a0.sum / a0.count) : 0UL,
            a0.count,
            a1.count ? static_cast<unsigned long>(a1.sum / a1.count) : 0UL,
            a1.count,
            a2.count ? static_cast<unsigned long>(a2.sum / a2.count) : 0UL,
            a2.count,
            a3.count ? static_cast<unsigned long>(a3.sum / a3.count) : 0UL,
            a3.count,
            max_late);
      }
    }
    // Preserve newest completed result for HTTP without blocking acquisition.
    portENTER_CRITICAL(&rtrh_adc_snapshot_mux);
    if (!rtrh_adc_snapshot_in_use) {
      memcpy(
          rtrh_adc_snapshot,
          rtrh_adc_agg,
          sizeof(rtrh_adc_snapshot));
      memcpy(
          rtrh_short_g13_raw_snapshot,
          rtrh_short_g13_raw,
          sizeof(rtrh_short_g13_raw_snapshot));
      rtrh_adc_snapshot_sequence = rtrh_adc_sequence;
      rtrh_adc_snapshot_ready = true;
    }
    portEXIT_CRITICAL(&rtrh_adc_snapshot_mux);

    // Measurement state is independent of HTTP now.
    reset_rtrh_adc_capture();

  }

  // A completed digital trace stays frozen for HTTP, but the GPIO IRQs stay
  // enabled because the autonomous RT/RH timing/ADC engine uses the same edges.
  if (rtrh_capture_ready && !rtrh_adc_active) {
    static uint32_t last_reported_rtrh_sequence = UINT32_MAX;

    if (last_reported_rtrh_sequence != rtrh_sequence) {
      last_reported_rtrh_sequence = rtrh_sequence;

      ESP_LOGI(
          TAG,
          "RT/RH edge capture ready: %u events, sequence %lu%s",
          rtrh_sample_count,
          static_cast<unsigned long>(rtrh_sequence),
          rtrh_overflow ? " OVERFLOW" : "");
    }
  }

  if (rtrh_capturing && !rtrh_capture_ready) {
    const uint32_t now = static_cast<uint32_t>(esp_timer_get_time());

    if (static_cast<uint32_t>(now - rtrh_start_us) >= RTRH_CAPTURE_US) {
      rtrh_capturing = false;
      rtrh_capture_ready = true;
      rtrh_sequence++;

      ESP_LOGI(
          TAG,
          "RT/RH edge capture ready: %u events, sequence %lu%s",
          rtrh_sample_count,
          static_cast<unsigned long>(rtrh_sequence),
          rtrh_overflow ? " OVERFLOW" : "");
    }
  }

  /*
   * Capture noch aktiv?
   */

  if (!capture_finished) {

    if (sample_count == 0)
      return;


    const uint32_t now =
        static_cast<uint32_t>(
            esp_timer_get_time()
        );


    if (
        static_cast<uint32_t>(
            now - last_edge
        ) <
        CAPTURE_TIMEOUT_US
    ) {

      return;
    }


    /*
     * Bus war 5 ms ruhig.
     */

    capturing = false;
    capture_finished = true;
  }


  /*
   * ISR schreibt jetzt nicht mehr.
   */

  uint16_t count =
      sample_count;


  if (count > MAX_SAMPLES)
    count = MAX_SAMPLES;


  const bool overflow =
      capture_overflow;


  const uint8_t initial_value =
      capture_initial_value;


  if (count != 0) {

    const uint32_t duration =
        static_cast<uint32_t>(
            samples[count - 1].t -
            samples[0].t
        );


    /*
     * Nur noch VERBOSE:
     *
     * Bei normalem DEBUG-Logging erscheint damit nicht
     * mehr alle ~6 Sekunden eine Capture-Zeile.
     */

    ESP_LOGV(
        TAG,
        "Capture: %u events, %lu us%s",
        count,
        static_cast<unsigned long>(
            duration
        ),
        overflow ? " OVERFLOW" : ""
    );


    /*
     * Overflow = garantiert unvollständiger Capture.
     */

    if (overflow) {

      this->frame_errors_++;


      if (
          this->frame_errors_sensor_ !=
          nullptr
      ) {

        this->frame_errors_sensor_->
            publish_state(
                this->frame_errors_
            );
      }

    } else {

      const DecodeResult result =
          decode_i2c_capture(
              samples,
              count,
              initial_value
          );


      /*
       * CRC-Fehler übernehmen.
       */

      if (result.crc_errors != 0) {

        this->crc_errors_ +=
            result.crc_errors;


        if (
            this->crc_errors_sensor_ !=
            nullptr
        ) {

          this->crc_errors_sensor_->
              publish_state(
                  this->crc_errors_
              );
        }
      }


      /*
       * Framefehler übernehmen.
       */

      if (result.frame_errors != 0) {

        this->frame_errors_ +=
            result.frame_errors;


        if (
            this->frame_errors_sensor_ !=
            nullptr
        ) {

          this->frame_errors_sensor_->
              publish_state(
                  this->frame_errors_
              );
        }
      }


      /*
       * CO2 nur publizieren, wenn sich der Wert
       * tatsächlich geändert hat.
       */

      if (result.have_co2) {

        const uint16_t ppm =
            result.co2_ppm;


        if (
            !this->have_last_ppm_ ||
            ppm != this->last_ppm_
        ) {

          this->have_last_ppm_ =
              true;


          this->last_ppm_ =
              ppm;


          ESP_LOGI(
              TAG,
              "CO2: %u ppm",
              ppm
          );


          if (
              this->co2_sensor_ !=
              nullptr
          ) {

            this->co2_sensor_->
                publish_state(
                    static_cast<float>(
                        ppm
                    )
                );
          }

        } else {

          ESP_LOGV(
              TAG,
              "CO2 unchanged: %u ppm",
              ppm
          );
        }
      }
    }


    /*
     * Den letzten Capture unabhängig vom Decoderergebnis
     * weiterhin für /capture aufbewahren.
     */

    store_raw_capture(
        samples,
        count,
        overflow
    );
  }


  /*
   * ==========================================================================
   * Nächsten Capture sofort starten
   * ==========================================================================
   */

  sample_count = 0;

  capture_overflow = false;

  capture_finished = false;


  last_value =
      read_gpio_state();


  capture_initial_value =
      last_value;


  last_edge =
      static_cast<uint32_t>(
          esp_timer_get_time()
      );


  capturing = true;
}


}  // namespace bus_sniffer
}  // namespace esphome
