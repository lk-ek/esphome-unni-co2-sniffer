#include "bus_sniffer.h"

#include "esphome/core/log.h"
#include "esphome/components/web_server_base/web_server_base.h"

#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_http_server.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include <climits>
#include <cmath>
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
 * GPIO10..13 are connected to the four RT/RH test points. They are treated
 * strictly as passive digital inputs.
 *
 * Reverse-engineering capture:
 *   - capture 450 ms, long enough for REF -> RT -> RH (~380 ms)
 *   - store the first edge
 *   - store every 16th subsequent edge as a time anchor
 *   - additionally store every 8th edge whose 4-bit state is neither 0x00 nor 0x0F
 *
 * edge_no is the 16-bit number of the original (non-decimated)
 * state-changing edge, so skipped edges remain visible in the CSV.
 *
 * When complete, the frozen capture is available as CSV at
 * /rt_rh_capture.csv. Downloading it rearms the capture for the next cycle.
 */

static constexpr gpio_num_t PIN_RTRH0 = GPIO_NUM_10;
static constexpr gpio_num_t PIN_RTRH1 = GPIO_NUM_11;
static constexpr gpio_num_t PIN_RTRH2 = GPIO_NUM_12;
static constexpr gpio_num_t PIN_RTRH3 = GPIO_NUM_13;

static constexpr uint32_t RTRH_CAPTURE_US = 450000;
static constexpr uint16_t RTRH_MAX_SAMPLES = 1536;
static constexpr uint32_t RTRH_CAPTURE_DECIMATION = 16;
static constexpr uint32_t RTRH_CAPTURE_UNUSUAL_DECIMATION = 8;

struct __attribute__((packed)) RtRhSample {
  uint32_t t_us;
  uint16_t edge_no;
  uint8_t value;
};

static volatile RtRhSample rtrh_samples[RTRH_MAX_SAMPLES];
static volatile uint16_t rtrh_sample_count = 0;
static volatile uint16_t rtrh_capture_edge_no = 0;
static volatile uint16_t rtrh_capture_unusual_no = 0;
static volatile uint8_t rtrh_last_value = 0xff;
static volatile uint32_t rtrh_start_us = 0;
static volatile bool rtrh_capturing = false;
static volatile bool rtrh_capture_ready = false;
static volatile bool rtrh_overflow = false;
static volatile uint32_t rtrh_sequence = 0;

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

static constexpr int RTRH_GPIOS[4] = {10, 11, 12, 13};

// REF-normalized passive calibration.
//
// The REF phase is treated as the internal timing reference.  Using ratios
// suppresses common drift from C12, GPIO threshold and supply.
//
// Historical calibration was obtained around REF ~= 76.72 us.  The old
// absolute fits have therefore been algebraically transformed to ratios;
// at REF=76.72 us they are numerically identical to v5.
//
// Temperature:
//   ratio = RT_period / REF_period
//   T[degC] = M * ratio + C
static constexpr float RTRH_TEMP_RATIO_M = -31.940170136f;
static constexpr float RTRH_TEMP_RATIO_C = 84.38101f;

// Humidity:
//   ratio = RH_period / REF_period
//   x = ln(ratio)
//   RH[%] = A*x^2 + B*x + C
// Anchors remain approximately 775 us -> 34 %, 457 us -> 39 %, 94 us -> 61 %
// when REF is 76.72 us.
static constexpr float RTRH_RH_RATIO_A = 2.1072311f;
static constexpr float RTRH_RH_RATIO_B = -18.10026849f;
static constexpr float RTRH_RH_RATIO_C = 64.58980155f;

// Measurement quality limits.  Invalid snapshots are logged but not published.
static constexpr float RTRH_REF_VALID_MIN_US = 72.0f;
static constexpr float RTRH_REF_VALID_MAX_US = 82.0f;
static constexpr float RTRH_REF_DURATION_MIN_MS = 115.0f;
static constexpr float RTRH_REF_DURATION_MAX_MS = 135.0f;
static constexpr float RTRH_RT_DURATION_MIN_MS = 120.0f;
static constexpr float RTRH_RT_DURATION_MAX_MS = 135.0f;
static constexpr float RTRH_RH_DURATION_MIN_MS = 120.0f;
static constexpr float RTRH_RH_DURATION_MAX_MS = 135.0f;
static constexpr uint16_t RTRH_REF_COUNT_MIN = 1450;
static constexpr uint16_t RTRH_REF_COUNT_MAX = 1750;
static constexpr uint16_t RTRH_RT_PHASE_COUNT_MIN = 850;
static constexpr uint16_t RTRH_RT_PHASE_COUNT_MAX = 930;

// Phase-based passive RT/RH decoder.
// Sequence: REF (~76.7 us) -> RT (~138 us) -> RH (variable).
static constexpr uint32_t RTRH_MEASUREMENT_QUIET_US = 15000000;
static constexpr uint32_t RTRH_CYCLE_GAP_US = 2000;
static constexpr uint8_t RTRH_PASSIVE_MAX_TRAINS = 16;
static constexpr uint8_t RTRH_PASSIVE_DELAY_BINS = 8;
static constexpr uint32_t RTRH_REF_MIN_US = 60;
static constexpr uint32_t RTRH_REF_MAX_US = 105;
static constexpr uint32_t RTRH_RT_MIN_US = 105;
static constexpr uint32_t RTRH_RT_MAX_US = 190;
static constexpr uint16_t RTRH_RT_TEMP_CYCLES = 880;
static constexpr uint16_t RTRH_RT_MIN_BEFORE_RH = 800;
static constexpr uint16_t RTRH_RT_FORCE_END_CYCLES = 920;
static constexpr uint8_t RTRH_PHASE_LOCK_CYCLES = 8;

enum RtRhPhase : uint8_t {
  RTRH_PHASE_WAIT_REF = 0,
  RTRH_PHASE_REF = 1,
  RTRH_PHASE_RT = 2,
  RTRH_PHASE_RH = 3,
};

enum RtRhTrainRole : uint8_t {
  RTRH_ROLE_UNKNOWN = 0,
  RTRH_ROLE_REF = 1,
  RTRH_ROLE_RT = 2,
  RTRH_ROLE_RH = 3,
  RTRH_ROLE_MIXED = 4,
};

struct RtRhPassiveTrain {
  uint8_t role{RTRH_ROLE_UNKNOWN};
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

  uint32_t start_us{0};
  uint32_t end_us{0};
  uint32_t gap_before_us{0};
};

struct RtRhPassiveSnapshot {
  RtRhPassiveTrain trains[RTRH_PASSIVE_MAX_TRAINS];
  uint8_t train_count{0};
  bool overflow{false};
  uint32_t sequence{0};

  bool rt_valid{false};
  bool rt_mixed{false};
  uint32_t rt_period_sum{0};
  uint16_t rt_count{0};
};

static volatile bool rtrh_passive_collecting = false;
static volatile bool rtrh_passive_overflow = false;
static volatile uint8_t rtrh_passive_train_count = 0;

static volatile uint32_t rtrh_passive_last_any_us = 0;
static volatile uint8_t rtrh_passive_last_state = 0;

static volatile uint32_t rtrh_passive_last_g10_fall_us = 0;
static volatile uint32_t rtrh_passive_g10_rise_us = 0;
static volatile uint32_t rtrh_passive_g13_rise_us = 0;
static volatile bool rtrh_passive_have_g10_rise = false;
static volatile bool rtrh_passive_have_g13_rise = false;

static volatile bool rtrh_passive_train_active = false;
static volatile uint32_t rtrh_passive_last_train_end_us = 0;
static volatile uint8_t rtrh_passive_phase = RTRH_PHASE_WAIT_REF;
static volatile uint8_t rtrh_passive_phase_candidate_run = 0;
static volatile uint32_t rtrh_passive_rt_temp_period_sum = 0;
static volatile uint16_t rtrh_passive_rt_temp_count = 0;

static RtRhPassiveTrain rtrh_passive_current_train;
static RtRhPassiveTrain
    rtrh_passive_trains[RTRH_PASSIVE_MAX_TRAINS];

static RtRhPassiveSnapshot rtrh_passive_snapshot;
static volatile bool rtrh_passive_snapshot_ready = false;
static volatile bool rtrh_passive_snapshot_consumed = true;

static volatile uint8_t rtrh_pin_level[4] = {0, 0, 0, 0};

// Experimental state-recurrence decoder.
//
// Periods are measured between repeated arrivals at the characteristic complete
// 4-bit states, not from individual GPIO10 edges:
//   REF: 0x0F -> 0x0F
//   RT : 0x07 -> 0x07
//   RH : 0x08 -> 0x08
//
// We keep a small rolling interval reservoir and use its median outside the
// ISR.  This rejects occasional missed characteristic states (2*T, 3*T, ...)
// as well as short ordering glitches without making the ISR choose a period.
static constexpr uint8_t RTRH_STATE_PERIOD_SAMPLES = 96;

struct RtRhStatePeriodStats {
  uint32_t last_us{0};
  uint16_t samples[RTRH_STATE_PERIOD_SAMPLES]{};
  uint8_t write_pos{0};
  uint8_t sample_count{0};
  uint32_t seen{0};
};

struct RtRhStatePeriodSnapshot {
  RtRhStatePeriodStats ref;
  RtRhStatePeriodStats rt;
  RtRhStatePeriodStats rh;
  uint32_t sequence{0};
};

static RtRhStatePeriodStats rtrh_state_ref;
static RtRhStatePeriodStats rtrh_state_rt;
static RtRhStatePeriodStats rtrh_state_rh;
static uint8_t rtrh_state_phase = RTRH_PHASE_WAIT_REF;
static RtRhStatePeriodSnapshot rtrh_state_snapshot;

static inline void IRAM_ATTR clear_rtrh_state_period(
    RtRhStatePeriodStats &s)
{
  s.last_us = 0;
  s.write_pos = 0;
  s.sample_count = 0;
  s.seen = 0;
  for (uint8_t i = 0; i < RTRH_STATE_PERIOD_SAMPLES; i++)
    s.samples[i] = 0;
}

static inline void IRAM_ATTR reset_rtrh_state_periods()
{
  clear_rtrh_state_period(rtrh_state_ref);
  clear_rtrh_state_period(rtrh_state_rt);
  clear_rtrh_state_period(rtrh_state_rh);
  rtrh_state_phase = RTRH_PHASE_WAIT_REF;
}

static inline void IRAM_ATTR add_rtrh_state_interval(
    RtRhStatePeriodStats &s,
    uint32_t now)
{
  if (s.last_us != 0) {
    const uint32_t dt = static_cast<uint32_t>(now - s.last_us);

    // All known physical periods fit comfortably below 2 ms.  The generous
    // upper bound still lets the median expose missed states as multiples,
    // while discarding phase gaps and unrelated long pauses.
    if (dt >= 40 && dt <= 2000) {
      s.samples[s.write_pos] = static_cast<uint16_t>(dt);
      s.write_pos =
          static_cast<uint8_t>((s.write_pos + 1) %
                               RTRH_STATE_PERIOD_SAMPLES);
      if (s.sample_count < RTRH_STATE_PERIOD_SAMPLES)
        s.sample_count++;
    }
  }

  s.last_us = now;
  s.seen++;
}

static inline void IRAM_ATTR observe_rtrh_characteristic_state(
    uint32_t now,
    uint8_t state)
{
  const uint8_t phase = rtrh_passive_phase;

  if (phase != rtrh_state_phase) {
    rtrh_state_phase = phase;

    // Do not bridge a phase boundary with a recurrence interval.
    if (phase == RTRH_PHASE_REF)
      rtrh_state_ref.last_us = 0;
    else if (phase == RTRH_PHASE_RT)
      rtrh_state_rt.last_us = 0;
    else if (phase == RTRH_PHASE_RH)
      rtrh_state_rh.last_us = 0;
  }

  if (phase == RTRH_PHASE_REF && state == 0x0F)
    add_rtrh_state_interval(rtrh_state_ref, now);
  else if (phase == RTRH_PHASE_RT && state == 0x07)
    add_rtrh_state_interval(rtrh_state_rt, now);
  else if (phase == RTRH_PHASE_RH && state == 0x08)
    add_rtrh_state_interval(rtrh_state_rh, now);
}

static float rtrh_state_period_median(
    const RtRhStatePeriodStats &s)
{
  const uint8_t n = s.sample_count;
  if (n == 0)
    return 0.0f;

  uint16_t tmp[RTRH_STATE_PERIOD_SAMPLES];
  for (uint8_t i = 0; i < n; i++)
    tmp[i] = s.samples[i];

  // n <= 96, once per 30 s: simple insertion sort is smaller than dragging in
  // a generic sort helper and deterministic enough here.
  for (uint8_t i = 1; i < n; i++) {
    const uint16_t v = tmp[i];
    uint8_t j = i;
    while (j > 0 && tmp[j - 1] > v) {
      tmp[j] = tmp[j - 1];
      j--;
    }
    tmp[j] = v;
  }

  if (n & 1)
    return static_cast<float>(tmp[n / 2]);

  return 0.5f *
      (static_cast<float>(tmp[n / 2 - 1]) +
       static_cast<float>(tmp[n / 2]));
}


static inline uint8_t IRAM_ATTR read_rtrh_state()
{
  uint8_t value = 0;
  if (gpio_get_level(PIN_RTRH0)) value |= 0x01;
  if (gpio_get_level(PIN_RTRH1)) value |= 0x02;
  if (gpio_get_level(PIN_RTRH2)) value |= 0x04;
  if (gpio_get_level(PIN_RTRH3)) value |= 0x08;
  return value;
}


static inline void IRAM_ATTR clear_passive_train(
    RtRhPassiveTrain &t)
{
  t.role = RTRH_ROLE_UNKNOWN;
  t.count = 0;

  t.period_sum = 0;
  t.low_sum = 0;
  t.g13_delay_sum = 0;
  t.g13_delay_count = 0;

  t.period_min = 0xFFFF;
  t.period_max = 0;
  t.low_min = 0xFFFF;
  t.low_max = 0;
  t.g13_delay_min = 0xFFFF;
  t.g13_delay_max = 0;

  for (uint8_t i = 0; i < RTRH_PASSIVE_DELAY_BINS; i++)
    t.g13_delay_hist[i] = 0;

  t.start_us = 0;
  t.end_us = 0;
  t.gap_before_us = 0;
}


static inline void IRAM_ATTR reset_rtrh_passive_measurement(
    uint32_t now,
    uint8_t state)
{
  rtrh_passive_collecting = true;
  rtrh_passive_overflow = false;
  rtrh_passive_train_count = 0;

  rtrh_passive_last_any_us = now;
  rtrh_passive_last_state = state;

  rtrh_passive_last_g10_fall_us = 0;
  rtrh_passive_g10_rise_us = 0;
  rtrh_passive_g13_rise_us = 0;
  rtrh_passive_have_g10_rise = false;
  rtrh_passive_have_g13_rise = false;

  rtrh_passive_train_active = false;
  rtrh_passive_last_train_end_us = 0;
  rtrh_passive_phase = RTRH_PHASE_WAIT_REF;
  rtrh_passive_phase_candidate_run = 0;
  rtrh_passive_rt_temp_period_sum = 0;
  rtrh_passive_rt_temp_count = 0;
  reset_rtrh_state_periods();
  clear_passive_train(rtrh_passive_current_train);

  for (uint8_t i = 0; i < RTRH_PASSIVE_MAX_TRAINS; i++)
    clear_passive_train(rtrh_passive_trains[i]);
}


static inline void IRAM_ATTR start_rtrh_passive_train(
    uint32_t now, uint8_t role)
{
  clear_passive_train(rtrh_passive_current_train);
  rtrh_passive_current_train.role = role;
  rtrh_passive_current_train.start_us = now;
  if (rtrh_passive_last_train_end_us != 0)
    rtrh_passive_current_train.gap_before_us =
        static_cast<uint32_t>(now - rtrh_passive_last_train_end_us);
  rtrh_passive_train_active = true;
  rtrh_passive_last_g10_fall_us = now;
  rtrh_passive_have_g10_rise = false;
  rtrh_passive_have_g13_rise = false;
}


static inline void IRAM_ATTR append_current_rtrh_passive_train()
{
  if (!rtrh_passive_train_active)
    return;

  RtRhPassiveTrain &cur = rtrh_passive_current_train;

  if (cur.count != 0) {
    cur.end_us = rtrh_passive_last_g10_fall_us;

    if (rtrh_passive_train_count < RTRH_PASSIVE_MAX_TRAINS) {
      rtrh_passive_trains[rtrh_passive_train_count++] = cur;
    } else {
      rtrh_passive_overflow = true;
    }

    rtrh_passive_last_train_end_us = cur.end_us;
  }

  clear_passive_train(rtrh_passive_current_train);
  rtrh_passive_train_active = false;
  rtrh_passive_last_g10_fall_us = 0;
  rtrh_passive_have_g10_rise = false;
  rtrh_passive_have_g13_rise = false;
}


static inline void IRAM_ATTR add_rtrh_passive_cycle(
    uint32_t period,
    uint32_t low,
    bool have_delay,
    uint32_t delay)
{
  if (!rtrh_passive_train_active)
    return;

  RtRhPassiveTrain &t = rtrh_passive_current_train;

  const uint16_t p =
      period > 65535U ? 65535U : static_cast<uint16_t>(period);
  const uint16_t l =
      low > 65535U ? 65535U : static_cast<uint16_t>(low);

  if (t.count != 0xFFFF)
    t.count++;

  t.period_sum += p;
  t.low_sum += l;

  if (p < t.period_min)
    t.period_min = p;
  if (p > t.period_max)
    t.period_max = p;

  if (l < t.low_min)
    t.low_min = l;
  if (l > t.low_max)
    t.low_max = l;

  if (have_delay && delay <= 65535U) {
    const uint16_t d = static_cast<uint16_t>(delay);

    t.g13_delay_sum += d;
    if (t.g13_delay_count != 0xFFFF)
      t.g13_delay_count++;

    if (d < t.g13_delay_min)
      t.g13_delay_min = d;
    if (d > t.g13_delay_max)
      t.g13_delay_max = d;

    const uint8_t bin =
        d < (RTRH_PASSIVE_DELAY_BINS - 1)
            ? static_cast<uint8_t>(d)
            : static_cast<uint8_t>(RTRH_PASSIVE_DELAY_BINS - 1);

    if (t.g13_delay_hist[bin] != 0xFFFF)
      t.g13_delay_hist[bin]++;
  }
}


static inline float rtrh_train_period_mean(
    const RtRhPassiveTrain &t)
{
  return t.count
      ? static_cast<float>(t.period_sum) /
            static_cast<float>(t.count)
      : 0.0f;
}


static inline bool rtrh_train_is_reference(
    const RtRhPassiveTrain &t)
{
  if (t.count < 8)
    return false;

  const float p = rtrh_train_period_mean(t);
  return p >= RTRH_REF_MIN_US && p <= RTRH_REF_MAX_US;
}


static void classify_rtrh_passive_snapshot(
    RtRhPassiveSnapshot &s)
{
  s.rt_valid = s.rt_count >= 800 && s.rt_count <= RTRH_RT_TEMP_CYCLES;
  s.rt_mixed = false;
}


static void finalize_rtrh_passive_measurement()
{
  if (!rtrh_passive_collecting)
    return;

  append_current_rtrh_passive_train();

  if (rtrh_passive_train_count == 0) {
    rtrh_passive_collecting = false;
    return;
  }

  RtRhPassiveSnapshot next{};

  next.train_count = rtrh_passive_train_count;
  next.overflow = rtrh_passive_overflow;
  next.sequence = rtrh_passive_snapshot.sequence + 1;

  for (uint8_t i = 0; i < RTRH_PASSIVE_MAX_TRAINS; i++) {
    if (i < next.train_count)
      next.trains[i] = rtrh_passive_trains[i];
    else
      clear_passive_train(next.trains[i]);
  }

  next.rt_period_sum = rtrh_passive_rt_temp_period_sum;
  next.rt_count = rtrh_passive_rt_temp_count;
  classify_rtrh_passive_snapshot(next);

  rtrh_passive_snapshot = next;

  rtrh_state_snapshot.ref = rtrh_state_ref;
  rtrh_state_snapshot.rt = rtrh_state_rt;
  rtrh_state_snapshot.rh = rtrh_state_rh;
  rtrh_state_snapshot.sequence = next.sequence;

  rtrh_passive_snapshot_ready = true;
  rtrh_passive_snapshot_consumed = false;
  rtrh_passive_collecting = false;
}


static const char *rtrh_train_role_name(uint8_t role)
{
  switch (role) {
    case RTRH_ROLE_REF:
      return "ref";
    case RTRH_ROLE_RT:
      return "rt";
    case RTRH_ROLE_RH:
      return "rh";
    case RTRH_ROLE_MIXED:
      return "mixed";
    default:
      return "unknown";
  }
}


static void IRAM_ATTR rtrh_gpio_isr(void *arg)
{
  const intptr_t encoded = reinterpret_cast<intptr_t>(arg);
  if (encoded < 1 || encoded > 4) return;
  const uint8_t pin_index = static_cast<uint8_t>(encoded - 1);
  const gpio_num_t pin = static_cast<gpio_num_t>(RTRH_GPIOS[pin_index]);
  const uint8_t level = static_cast<uint8_t>(gpio_get_level(pin));
  const uint8_t previous = rtrh_pin_level[pin_index];
  if (level == previous) return;
  rtrh_pin_level[pin_index] = level;

  const uint32_t now = static_cast<uint32_t>(esp_timer_get_time());
  const uint8_t state = read_rtrh_state();

  if (!rtrh_passive_collecting) reset_rtrh_passive_measurement(now, state);
  else rtrh_passive_last_any_us = now;

  const uint8_t old_state = rtrh_passive_last_state;
  if (state != old_state) {
    const bool old_g10 = (old_state & 0x01) != 0;
    const bool new_g10 = (state & 0x01) != 0;
    const bool old_g13 = (old_state & 0x08) != 0;
    const bool new_g13 = (state & 0x08) != 0;

    if (!old_g10 && new_g10) {
      rtrh_passive_g10_rise_us = now;
      rtrh_passive_have_g10_rise = true;
      rtrh_passive_have_g13_rise = new_g13;
      if (new_g13) rtrh_passive_g13_rise_us = now;
    }
    if (!old_g13 && new_g13 && rtrh_passive_have_g10_rise) {
      rtrh_passive_g13_rise_us = now;
      rtrh_passive_have_g13_rise = true;
    }

    if (old_g10 && !new_g10) {
      const uint32_t pf = rtrh_passive_last_g10_fall_us;
      if (pf == 0) {
        rtrh_passive_last_g10_fall_us = now;
      } else {
        const uint32_t period = static_cast<uint32_t>(now - pf);
        rtrh_passive_last_g10_fall_us = now;

        if (period < RTRH_CYCLE_GAP_US && rtrh_passive_have_g10_rise) {
          const uint32_t low =
              static_cast<uint32_t>(rtrh_passive_g10_rise_us - pf);
          const bool have_delay =
              rtrh_passive_have_g13_rise &&
              static_cast<int32_t>(rtrh_passive_g13_rise_us -
                                   rtrh_passive_g10_rise_us) >= 0;
          const uint32_t delay = have_delay
              ? static_cast<uint32_t>(rtrh_passive_g13_rise_us -
                                      rtrh_passive_g10_rise_us)
              : 0;
          const bool is_ref = period >= RTRH_REF_MIN_US && period <= RTRH_REF_MAX_US;
          const bool is_rt  = period >= RTRH_RT_MIN_US  && period <= RTRH_RT_MAX_US;

          switch (rtrh_passive_phase) {
            case RTRH_PHASE_WAIT_REF:
              if (is_ref) {
                start_rtrh_passive_train(now, RTRH_ROLE_REF);
                rtrh_passive_phase = RTRH_PHASE_REF;
                add_rtrh_passive_cycle(period, low, have_delay, delay);
              }
              break;

            case RTRH_PHASE_REF:
              if (is_ref) {
                // A genuine REF cycle cancels any tentative REF->RT transition.
                rtrh_passive_phase_candidate_run = 0;
                add_rtrh_passive_cycle(period, low, have_delay, delay);
              } else if (is_rt) {
                // REF contains occasional isolated 140..160 us glitches.
                // Do not leave REF until RT-like timing is stable for 8 cycles.
                if (rtrh_passive_phase_candidate_run < 255)
                  rtrh_passive_phase_candidate_run++;

                if (rtrh_passive_phase_candidate_run >=
                    RTRH_PHASE_LOCK_CYCLES) {
                  append_current_rtrh_passive_train();
                  start_rtrh_passive_train(now, RTRH_ROLE_RT);
                  rtrh_passive_phase = RTRH_PHASE_RT;
                  rtrh_passive_phase_candidate_run = 0;

                  // The first 7 candidate cycles are intentionally discarded;
                  // one cycle is enough to seed the confirmed RT phase.
                  add_rtrh_passive_cycle(period, low, have_delay, delay);

                  if (rtrh_passive_rt_temp_count < RTRH_RT_TEMP_CYCLES) {
                    rtrh_passive_rt_temp_period_sum += period;
                    rtrh_passive_rt_temp_count++;
                  }
                }
              } else {
                // Neither REF nor RT: transition/glitch, do not accumulate lock.
                rtrh_passive_phase_candidate_run = 0;
              }
              break;

            case RTRH_PHASE_RT:
              if (rtrh_passive_current_train.count >=
                  RTRH_RT_FORCE_END_CYCLES) {
                // Ambiguous RT/RH overlap: physical RT length is known.
                append_current_rtrh_passive_train();
                start_rtrh_passive_train(now, RTRH_ROLE_RH);
                rtrh_passive_phase = RTRH_PHASE_RH;
                rtrh_passive_phase_candidate_run = 0;
                add_rtrh_passive_cycle(period, low, have_delay, delay);
              } else if (is_rt) {
                rtrh_passive_phase_candidate_run = 0;
                add_rtrh_passive_cycle(period, low, have_delay, delay);

                if (rtrh_passive_rt_temp_count < RTRH_RT_TEMP_CYCLES) {
                  rtrh_passive_rt_temp_period_sum += period;
                  rtrh_passive_rt_temp_count++;
                }
              } else {
                // Do not permit an RT->RH transition until a physically
                // plausible RT section has actually been collected.  This
                // prevents transition glitches immediately after REF from
                // turning almost the whole measurement into RH.
                if (rtrh_passive_current_train.count <
                    RTRH_RT_MIN_BEFORE_RH) {
                  rtrh_passive_phase_candidate_run = 0;
                } else {
                  if (rtrh_passive_phase_candidate_run < 255)
                    rtrh_passive_phase_candidate_run++;

                  if (rtrh_passive_phase_candidate_run >=
                      RTRH_PHASE_LOCK_CYCLES) {
                    append_current_rtrh_passive_train();
                    start_rtrh_passive_train(now, RTRH_ROLE_RH);
                    rtrh_passive_phase = RTRH_PHASE_RH;
                    rtrh_passive_phase_candidate_run = 0;
                    add_rtrh_passive_cycle(period, low, have_delay, delay);
                  }
                }
              }
              break;

            case RTRH_PHASE_RH:
              add_rtrh_passive_cycle(period, low, have_delay, delay);
              break;
          }
        }
      }
      rtrh_passive_have_g10_rise = false;
      rtrh_passive_have_g13_rise = false;
    }

    // State-recurrence measurement.  This intentionally runs after the
    // original v7c phase FSM so the current REF/RT/RH phase is already known.
    observe_rtrh_characteristic_state(now, state);

    rtrh_passive_last_state = state;
  }

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
    rtrh_capture_edge_no = 0;
    rtrh_capture_unusual_no = 0;
    rtrh_overflow = false;
  }

  const uint16_t edge_no = rtrh_capture_edge_no++;

  // Keep regular global anchors, but also sample recurring "unusual" states.
  // RT contains ~900 repetitions of 0x07, so storing every unusual edge would
  // still overflow the buffer.  Every 8th unusual state is sufficient to show
  // the phase pattern while preserving plenty of transition detail.
  const bool unusual_state = value != 0x00 && value != 0x0F;
  const bool time_anchor =
      edge_no == 0 || (edge_no % RTRH_CAPTURE_DECIMATION) == 0;

  bool unusual_anchor = false;
  if (unusual_state) {
    const uint16_t unusual_no = rtrh_capture_unusual_no++;
    unusual_anchor =
        unusual_no == 0 ||
        (unusual_no % RTRH_CAPTURE_UNUSUAL_DECIMATION) == 0;
  }

  if (!time_anchor && !unusual_anchor)
    return;

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
  rtrh_samples[index].edge_no = edge_no;
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
        "sequence,t_us,edge_no,gpio10,gpio11,gpio12,gpio13,state,overflow\n";
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
      const uint16_t edge_no = rtrh_samples[i].edge_no;
      const uint8_t v = rtrh_samples[i].value;

      const int n = snprintf(
          line, sizeof(line),
          "%lu,%lu,%u,%u,%u,%u,%u,0x%02X,%u\n",
          static_cast<unsigned long>(sequence),
          static_cast<unsigned long>(t),
          static_cast<unsigned>(edge_no),
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
    rtrh_capture_edge_no = 0;
    rtrh_capture_unusual_no = 0;
    rtrh_overflow = false;
    rtrh_capture_ready = false;
    rtrh_capturing = false;

    if (err != ESP_OK)
      ESP_LOGW(TAG, "rt_rh_capture.csv client disconnected (%d)", err);
  }
};

static RtRhCaptureHandler rtrh_capture_handler;

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
    if (!rtrh_passive_snapshot_ready ||
        rtrh_passive_snapshot_consumed) {
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
        "measurement,train,role,count,gap_before_us,"
        "period_mean_us,period_min_us,period_max_us,"
        "duration_ms,frequency_hz,"
        "low_mean_us,low_min_us,low_max_us,"
        "g13_delay_count,g13_delay_mean_us,g13_delay_min_us,g13_delay_max_us,"
        "d0,d1,d2,d3,d4,d5,d6,d7plus,overflow\n";

    esp_err_t err =
        httpd_resp_send_chunk(req, HEADER, sizeof(HEADER) - 1);

    static char line[320];

    for (uint8_t i = 0;
         i < s.train_count && err == ESP_OK;
         i++) {
      const RtRhPassiveTrain &t = s.trains[i];

      const float period_mean =
          t.count
              ? static_cast<float>(t.period_sum) /
                    static_cast<float>(t.count)
              : 0.0f;

      const float low_mean =
          t.count
              ? static_cast<float>(t.low_sum) /
                    static_cast<float>(t.count)
              : 0.0f;

      const float delay_mean =
          t.g13_delay_count
              ? static_cast<float>(t.g13_delay_sum) /
                    static_cast<float>(t.g13_delay_count)
              : 0.0f;

      const float duration_ms =
          static_cast<float>(t.period_sum) / 1000.0f;

      const float frequency_hz =
          period_mean > 0.0f
              ? 1000000.0f / period_mean
              : 0.0f;

      const int n = snprintf(
          line,
          sizeof(line),
          "%lu,%u,%s,%u,%lu,%.3f,%u,%u,%.3f,%.2f,%.3f,%u,%u,"
          "%u,%.3f,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\n",
          static_cast<unsigned long>(s.sequence),
          static_cast<unsigned>(i),
          rtrh_train_role_name(t.role),
          static_cast<unsigned>(t.count),
          static_cast<unsigned long>(t.gap_before_us),
          period_mean,
          t.count ? static_cast<unsigned>(t.period_min) : 0U,
          static_cast<unsigned>(t.period_max),
          duration_ms,
          frequency_hz,
          low_mean,
          t.count ? static_cast<unsigned>(t.low_min) : 0U,
          static_cast<unsigned>(t.low_max),
          static_cast<unsigned>(t.g13_delay_count),
          delay_mean,
          t.g13_delay_count
              ? static_cast<unsigned>(t.g13_delay_min)
              : 0U,
          static_cast<unsigned>(t.g13_delay_max),
          static_cast<unsigned>(t.g13_delay_hist[0]),
          static_cast<unsigned>(t.g13_delay_hist[1]),
          static_cast<unsigned>(t.g13_delay_hist[2]),
          static_cast<unsigned>(t.g13_delay_hist[3]),
          static_cast<unsigned>(t.g13_delay_hist[4]),
          static_cast<unsigned>(t.g13_delay_hist[5]),
          static_cast<unsigned>(t.g13_delay_hist[6]),
          static_cast<unsigned>(t.g13_delay_hist[7]),
          s.overflow ? 1U : 0U);

      if (n <= 0 ||
          static_cast<size_t>(n) >= sizeof(line)) {
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

  // GPIO10..13 stay ordinary high-impedance digital inputs for the entire run.


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
      "Raw captures at /capture; RT/RH edges /rt_rh_capture.csv; trains /rt_rh_timing.csv"
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
   * A whole sensor measurement may contain long internal pauses.  Only 15 s
   * without ANY RT/RH edge freezes and classifies the measurement.
   */
  if (rtrh_passive_collecting) {
    const uint32_t now =
        static_cast<uint32_t>(esp_timer_get_time());
    const uint32_t last_any =
        rtrh_passive_last_any_us;

    if (last_any != 0 &&
        static_cast<uint32_t>(now - last_any) >
            RTRH_MEASUREMENT_QUIET_US) {
      gpio_intr_disable(PIN_RTRH0);
      gpio_intr_disable(PIN_RTRH1);
      gpio_intr_disable(PIN_RTRH2);
      gpio_intr_disable(PIN_RTRH3);

      const uint32_t now2 =
          static_cast<uint32_t>(esp_timer_get_time());
      const uint32_t last_any2 =
          rtrh_passive_last_any_us;

      if (rtrh_passive_collecting &&
          last_any2 != 0 &&
          static_cast<uint32_t>(now2 - last_any2) >
              RTRH_MEASUREMENT_QUIET_US) {
        finalize_rtrh_passive_measurement();
      }

      rtrh_passive_last_state = read_rtrh_state();
      rtrh_pin_level[0] = gpio_get_level(PIN_RTRH0);
      rtrh_pin_level[1] = gpio_get_level(PIN_RTRH1);
      rtrh_pin_level[2] = gpio_get_level(PIN_RTRH2);
      rtrh_pin_level[3] = gpio_get_level(PIN_RTRH3);

      gpio_intr_enable(PIN_RTRH0);
      gpio_intr_enable(PIN_RTRH1);
      gpio_intr_enable(PIN_RTRH2);
      gpio_intr_enable(PIN_RTRH3);
    }
  }

  static uint32_t last_passive_sequence = 0;

  if (rtrh_passive_snapshot_ready &&
      rtrh_passive_snapshot.sequence !=
          last_passive_sequence) {
    const RtRhPassiveSnapshot s =
        rtrh_passive_snapshot;

    last_passive_sequence = s.sequence;

    ESP_LOGI(
        TAG,
        "RT/RH passive measurement %lu: %u phases%s%s",
        static_cast<unsigned long>(s.sequence),
        static_cast<unsigned>(s.train_count),
        s.overflow ? " OVERFLOW" : "",
        s.rt_mixed ? " RT/RH-MIXED" : "");

    const RtRhPassiveTrain *ref_train = nullptr;
    const RtRhPassiveTrain *rt_train = nullptr;
    const RtRhPassiveTrain *rh_train = nullptr;

    for (uint8_t i = 0; i < s.train_count; i++) {
      const RtRhPassiveTrain &train = s.trains[i];

      const float period_mean =
          train.count
              ? static_cast<float>(train.period_sum) /
                    static_cast<float>(train.count)
              : 0.0f;

      const float low_mean =
          train.count
              ? static_cast<float>(train.low_sum) /
                    static_cast<float>(train.count)
              : 0.0f;

      const float delay_mean =
          train.g13_delay_count
              ? static_cast<float>(train.g13_delay_sum) /
                    static_cast<float>(train.g13_delay_count)
              : 0.0f;

      ESP_LOGI(
          TAG,
          "  train %u %-7s: n=%u gap=%lu us period=%.3f us "
          "low=%.3f us G13delay=%.3f us (%u)",
          static_cast<unsigned>(i),
          rtrh_train_role_name(train.role),
          static_cast<unsigned>(train.count),
          static_cast<unsigned long>(train.gap_before_us),
          period_mean,
          low_mean,
          delay_mean,
          static_cast<unsigned>(train.g13_delay_count));

      if (train.role == RTRH_ROLE_REF && ref_train == nullptr)
        ref_train = &s.trains[i];
      else if (train.role == RTRH_ROLE_RT && rt_train == nullptr)
        rt_train = &s.trains[i];
      else if (train.role == RTRH_ROLE_RH && rh_train == nullptr)
        rh_train = &s.trains[i];
    }

    bool measurement_valid =
        !s.overflow &&
        !s.rt_mixed &&
        ref_train != nullptr &&
        rt_train != nullptr &&
        rh_train != nullptr &&
        ref_train->count != 0 &&
        rt_train->count != 0 &&
        rh_train->count != 0;

    float ref_period_us = 0.0f;
    float rt_phase_period_us = 0.0f;
    float rh_period_us = 0.0f;
    float ref_duration_ms = 0.0f;
    float rt_duration_ms = 0.0f;
    float rh_duration_ms = 0.0f;

    if (measurement_valid) {
      ref_period_us =
          static_cast<float>(ref_train->period_sum) /
          static_cast<float>(ref_train->count);

      rt_phase_period_us =
          static_cast<float>(rt_train->period_sum) /
          static_cast<float>(rt_train->count);

      rh_period_us =
          static_cast<float>(rh_train->period_sum) /
          static_cast<float>(rh_train->count);

      ref_duration_ms =
          static_cast<float>(ref_train->period_sum) / 1000.0f;
      rt_duration_ms =
          static_cast<float>(rt_train->period_sum) / 1000.0f;
      rh_duration_ms =
          static_cast<float>(rh_train->period_sum) / 1000.0f;

      const bool ref_ok =
          ref_period_us >= RTRH_REF_VALID_MIN_US &&
          ref_period_us <= RTRH_REF_VALID_MAX_US &&
          ref_duration_ms >= RTRH_REF_DURATION_MIN_MS &&
          ref_duration_ms <= RTRH_REF_DURATION_MAX_MS &&
          ref_train->count >= RTRH_REF_COUNT_MIN &&
          ref_train->count <= RTRH_REF_COUNT_MAX;

      const bool rt_ok =
          rt_duration_ms >= RTRH_RT_DURATION_MIN_MS &&
          rt_duration_ms <= RTRH_RT_DURATION_MAX_MS &&
          rt_train->count >= RTRH_RT_PHASE_COUNT_MIN &&
          rt_train->count <= RTRH_RT_PHASE_COUNT_MAX &&
          s.rt_valid &&
          s.rt_count >= 800;

      const bool rh_ok =
          rh_duration_ms >= RTRH_RH_DURATION_MIN_MS &&
          rh_duration_ms <= RTRH_RH_DURATION_MAX_MS;

      measurement_valid = ref_ok && rt_ok && rh_ok;

      ESP_LOGI(
          TAG,
          "RT/RH quality: REF %.3f us / %.3f ms / %u, "
          "RT %.3f ms / %u, RH %.3f ms / %u -> %s",
          ref_period_us,
          ref_duration_ms,
          static_cast<unsigned>(ref_train->count),
          rt_duration_ms,
          static_cast<unsigned>(rt_train->count),
          rh_duration_ms,
          static_cast<unsigned>(rh_train->count),
          measurement_valid ? "VALID" : "REJECT");
    }

    if (!measurement_valid) {
      ESP_LOGW(
          TAG,
          "RT/RH values not published: measurement failed "
          "phase/quality checks");
    } else {
      // Temperature uses the protected first 880 RT cycles, normalized by REF.
      const float rt_period_us =
          static_cast<float>(s.rt_period_sum) /
          static_cast<float>(s.rt_count);

      const float rt_ratio =
          rt_period_us / ref_period_us;

      const float temperature_c =
          RTRH_TEMP_RATIO_M * rt_ratio +
          RTRH_TEMP_RATIO_C;

      // RH is normalized to the same REF period.  The RH phase itself is a
      // fixed ~127 ms counting window, so count and period carry the same
      // information; period/count consistency is inherently checked by the
      // duration gate above.
      const float rh_ratio =
          rh_period_us / ref_period_us;

      const float x = logf(rh_ratio);

      float rh_percent =
          RTRH_RH_RATIO_A * x * x +
          RTRH_RH_RATIO_B * x +
          RTRH_RH_RATIO_C;

      if (rh_percent < 0.0f)
        rh_percent = 0.0f;
      else if (rh_percent > 100.0f)
        rh_percent = 100.0f;

      const float rh_frequency_hz =
          1000000.0f / rh_period_us;

      ESP_LOGI(
          TAG,
          "RT normalized: ratio=%.6f (RT %.3f / REF %.3f us)",
          rt_ratio,
          rt_period_us,
          ref_period_us);

      ESP_LOGI(
          TAG,
          "RH normalized: ratio=%.6f (RH %.3f / REF %.3f us), "
          "cycles=%u, duration=%.3f ms, freq=%.2f Hz",
          rh_ratio,
          rh_period_us,
          ref_period_us,
          static_cast<unsigned>(rh_train->count),
          rh_duration_ms,
          rh_frequency_hz);

      ESP_LOGI(
          TAG,
          "RH humidity PASSIVE: %.1f %% from normalized RH ratio %.6f",
          rh_percent,
          rh_ratio);

      ESP_LOGI(
          TAG,
          "RT temperature PASSIVE: %.2f C from normalized RT ratio %.6f "
          "(%.3f us, %u cycles)",
          temperature_c,
          rt_ratio,
          rt_period_us,
          static_cast<unsigned>(s.rt_count));

      // Experimental v9 result: period from complete-state recurrence.
      const RtRhStatePeriodSnapshot ss = rtrh_state_snapshot;
      const float state_ref_us = rtrh_state_period_median(ss.ref);
      const float state_rt_us = rtrh_state_period_median(ss.rt);
      const float state_rh_us = rtrh_state_period_median(ss.rh);

      const bool state_periods_valid =
          ss.sequence == s.sequence &&
          ss.ref.sample_count >= 16 &&
          ss.rt.sample_count >= 16 &&
          ss.rh.sample_count >= 8 &&
          state_ref_us >= RTRH_REF_VALID_MIN_US &&
          state_ref_us <= RTRH_REF_VALID_MAX_US &&
          state_rt_us >= 105.0f &&
          state_rt_us <= 190.0f &&
          state_rh_us >= 70.0f &&
          state_rh_us <= 1200.0f;

      if (state_periods_valid) {
        const float state_rt_ratio = state_rt_us / state_ref_us;
        const float state_temperature_c =
            RTRH_TEMP_RATIO_M * state_rt_ratio + RTRH_TEMP_RATIO_C;

        const float state_rh_ratio = state_rh_us / state_ref_us;
        const float state_x = logf(state_rh_ratio);
        float state_rh_percent =
            RTRH_RH_RATIO_A * state_x * state_x +
            RTRH_RH_RATIO_B * state_x +
            RTRH_RH_RATIO_C;
        if (state_rh_percent < 0.0f) state_rh_percent = 0.0f;
        if (state_rh_percent > 100.0f) state_rh_percent = 100.0f;

        ESP_LOGI(
            TAG,
            "STATE periods: REF %.3f us (%u/%lu), RT %.3f us (%u/%lu), "
            "RH %.3f us (%u/%lu)",
            state_ref_us,
            static_cast<unsigned>(ss.ref.sample_count),
            static_cast<unsigned long>(ss.ref.seen),
            state_rt_us,
            static_cast<unsigned>(ss.rt.sample_count),
            static_cast<unsigned long>(ss.rt.seen),
            state_rh_us,
            static_cast<unsigned>(ss.rh.sample_count),
            static_cast<unsigned long>(ss.rh.seen));

        ESP_LOGI(
            TAG,
            "STATE normalized: RT ratio=%.6f -> %.2f C ; "
            "RH ratio=%.6f -> %.1f %%",
            state_rt_ratio,
            state_temperature_c,
            state_rh_ratio,
            state_rh_percent);

        // v9 deliberately publishes the state-recurrence result.  The complete
        // v7c calculation above remains in the log as the golden comparison.
        if (this->rh_humidity_sensor_ != nullptr)
          this->rh_humidity_sensor_->publish_state(state_rh_percent);

        if (this->rt_temperature_sensor_ != nullptr)
          this->rt_temperature_sensor_->publish_state(state_temperature_c);
      } else {
        ESP_LOGW(
            TAG,
            "STATE periods invalid: REF %.3f us (%u), RT %.3f us (%u), "
            "RH %.3f us (%u); keeping v7c result",
            state_ref_us, static_cast<unsigned>(ss.ref.sample_count),
            state_rt_us, static_cast<unsigned>(ss.rt.sample_count),
            state_rh_us, static_cast<unsigned>(ss.rh.sample_count));

        if (this->rh_humidity_sensor_ != nullptr)
          this->rh_humidity_sensor_->publish_state(rh_percent);

        if (this->rt_temperature_sensor_ != nullptr)
          this->rt_temperature_sensor_->publish_state(temperature_c);
      }
    }
  }

  // A completed digital trace stays frozen for HTTP; train decoding continues.
  if (rtrh_capture_ready) {
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
