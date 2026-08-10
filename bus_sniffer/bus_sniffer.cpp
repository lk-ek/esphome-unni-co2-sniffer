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
 * RT/RH decoder
 * ============================================================================
 *
 * Production path:
 *   GPIO10 falling edges -> REF -> RT -> RH FSM -> REF-normalized values.
 *
 * Debug path:
 *   GPIO11..13 interrupts are added only for raw 4-bit state capture and
 *   G13 timing diagnostics.  Set RTRH_DEBUG_CAPTURE to 0 for the lean
 *   one-interrupt production decoder.
 */

#define RTRH_DEBUG_CAPTURE 1

static constexpr gpio_num_t PIN_RTRH0 = GPIO_NUM_10;
static constexpr gpio_num_t PIN_RTRH1 = GPIO_NUM_11;
static constexpr gpio_num_t PIN_RTRH2 = GPIO_NUM_12;
static constexpr gpio_num_t PIN_RTRH3 = GPIO_NUM_13;

static constexpr gpio_num_t RTRH_PINS[4] = {
    PIN_RTRH0, PIN_RTRH1, PIN_RTRH2, PIN_RTRH3};

// REF-normalized calibration.
static constexpr float RTRH_TEMP_RATIO_M = -31.940170136f;
static constexpr float RTRH_TEMP_RATIO_C = 84.38101f;

static constexpr float RTRH_RH_RATIO_A = 2.1072311f;
static constexpr float RTRH_RH_RATIO_B = -18.10026849f;
static constexpr float RTRH_RH_RATIO_C = 64.58980155f;

// Measurement/FSM limits.
static constexpr uint32_t RTRH_MEASUREMENT_QUIET_US = 15000000;
static constexpr uint32_t RTRH_CYCLE_GAP_US = 2000;
static constexpr uint32_t RTRH_REF_MIN_US = 60;
static constexpr uint32_t RTRH_REF_MAX_US = 105;
static constexpr uint32_t RTRH_RT_MIN_US = 105;
static constexpr uint32_t RTRH_RT_MAX_US = 190;
static constexpr uint16_t RTRH_RT_TEMP_CYCLES = 880;
static constexpr uint16_t RTRH_RT_MIN_BEFORE_RH = 800;
static constexpr uint16_t RTRH_RT_FORCE_END_CYCLES = 920;
static constexpr uint8_t RTRH_PHASE_LOCK_CYCLES = 8;

// Publication quality gates.
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

enum class RtRhPhase : uint8_t {
  WAIT_REF = 0,
  REF,
  RT,
  RH,
};

struct RtRhPhaseStats {
  uint32_t period_sum{0};
  uint16_t count{0};

#if RTRH_DEBUG_CAPTURE
  uint32_t low_sum{0};
  uint32_t g13_delay_sum{0};
  uint16_t g13_delay_count{0};
  uint16_t period_min{0xFFFF};
  uint16_t period_max{0};
  uint16_t low_min{0xFFFF};
  uint16_t low_max{0};
  uint16_t g13_delay_min{0xFFFF};
  uint16_t g13_delay_max{0};
  uint16_t g13_delay_hist[8]{};
#endif
};

struct RtRhSnapshot {
  RtRhPhaseStats ref;
  RtRhPhaseStats rt;
  RtRhPhaseStats rh;
  uint32_t rt_temp_period_sum{0};
  uint16_t rt_temp_count{0};
  uint32_t sequence{0};
};

static volatile bool rtrh_collecting = false;
static volatile uint32_t rtrh_last_any_us = 0;
static volatile uint32_t rtrh_last_g10_fall_us = 0;
static volatile uint32_t rtrh_g10_rise_us = 0;
static volatile bool rtrh_have_g10_rise = false;
static volatile RtRhPhase rtrh_phase = RtRhPhase::WAIT_REF;
static volatile uint8_t rtrh_phase_candidate_run = 0;
static volatile uint32_t rtrh_rt_temp_period_sum = 0;
static volatile uint16_t rtrh_rt_temp_count = 0;
static volatile uint8_t rtrh_last_g10 = 0;

static RtRhPhaseStats rtrh_ref;
static RtRhPhaseStats rtrh_rt;
static RtRhPhaseStats rtrh_rh;
static RtRhSnapshot rtrh_snapshot;
static volatile bool rtrh_snapshot_ready = false;

#if RTRH_DEBUG_CAPTURE
static volatile uint8_t rtrh_pin_level[4] = {0, 0, 0, 0};
static volatile uint8_t rtrh_last_state = 0;
static volatile uint32_t rtrh_g13_rise_us = 0;
static volatile bool rtrh_have_g13_rise = false;

static constexpr uint32_t RTRH_CAPTURE_US = 450000;
static constexpr uint16_t RTRH_MAX_SAMPLES = 1536;
static constexpr uint16_t RTRH_CAPTURE_DECIMATION = 16;
static constexpr uint16_t RTRH_CAPTURE_UNUSUAL_DECIMATION = 8;

struct __attribute__((packed)) RtRhSample {
  uint32_t t_us;
  uint16_t edge_no;
  uint8_t value;
};

static volatile RtRhSample rtrh_samples[RTRH_MAX_SAMPLES];
static volatile uint16_t rtrh_sample_count = 0;
static volatile uint16_t rtrh_capture_edge_no = 0;
static volatile uint16_t rtrh_capture_unusual_no = 0;
static volatile uint8_t rtrh_capture_last_value = 0xff;
static volatile uint32_t rtrh_capture_start_us = 0;
static volatile bool rtrh_capturing = false;
static volatile bool rtrh_capture_ready = false;
static volatile bool rtrh_capture_overflow = false;
static volatile uint32_t rtrh_capture_sequence = 0;
#endif

static inline void IRAM_ATTR clear_rtrh_stats(RtRhPhaseStats &s)
{
  s.period_sum = 0;
  s.count = 0;
#if RTRH_DEBUG_CAPTURE
  s.low_sum = 0;
  s.g13_delay_sum = 0;
  s.g13_delay_count = 0;
  s.period_min = 0xFFFF;
  s.period_max = 0;
  s.low_min = 0xFFFF;
  s.low_max = 0;
  s.g13_delay_min = 0xFFFF;
  s.g13_delay_max = 0;
  for (uint8_t i = 0; i < 8; i++)
    s.g13_delay_hist[i] = 0;
#endif
}

#if RTRH_DEBUG_CAPTURE
static inline uint8_t IRAM_ATTR read_rtrh_state()
{
  uint8_t value = 0;
  if (gpio_get_level(PIN_RTRH0)) value |= 0x01;
  if (gpio_get_level(PIN_RTRH1)) value |= 0x02;
  if (gpio_get_level(PIN_RTRH2)) value |= 0x04;
  if (gpio_get_level(PIN_RTRH3)) value |= 0x08;
  return value;
}
#endif

static inline void IRAM_ATTR reset_rtrh_measurement(uint32_t now)
{
  rtrh_collecting = true;
  rtrh_last_any_us = now;
  rtrh_last_g10_fall_us = 0;
  rtrh_g10_rise_us = 0;
  rtrh_have_g10_rise = false;
  rtrh_phase = RtRhPhase::WAIT_REF;
  rtrh_phase_candidate_run = 0;
  rtrh_rt_temp_period_sum = 0;
  rtrh_rt_temp_count = 0;
  clear_rtrh_stats(rtrh_ref);
  clear_rtrh_stats(rtrh_rt);
  clear_rtrh_stats(rtrh_rh);
#if RTRH_DEBUG_CAPTURE
  rtrh_g13_rise_us = 0;
  rtrh_have_g13_rise = false;
#endif
}

static inline void IRAM_ATTR add_rtrh_cycle(
    RtRhPhaseStats &s,
    uint32_t period,
    uint32_t low
#if RTRH_DEBUG_CAPTURE
    , bool have_delay,
    uint32_t delay
#endif
)
{
  if (s.count != 0xFFFF)
    s.count++;
  s.period_sum += period;

#if RTRH_DEBUG_CAPTURE
  const uint16_t p =
      period > 65535U ? 65535U : static_cast<uint16_t>(period);
  const uint16_t l =
      low > 65535U ? 65535U : static_cast<uint16_t>(low);

  s.low_sum += l;
  if (p < s.period_min) s.period_min = p;
  if (p > s.period_max) s.period_max = p;
  if (l < s.low_min) s.low_min = l;
  if (l > s.low_max) s.low_max = l;

  if (have_delay && delay <= 65535U) {
    const uint16_t d = static_cast<uint16_t>(delay);
    s.g13_delay_sum += d;
    if (s.g13_delay_count != 0xFFFF)
      s.g13_delay_count++;
    if (d < s.g13_delay_min) s.g13_delay_min = d;
    if (d > s.g13_delay_max) s.g13_delay_max = d;
    const uint8_t bin = d < 7 ? static_cast<uint8_t>(d) : 7;
    if (s.g13_delay_hist[bin] != 0xFFFF)
      s.g13_delay_hist[bin]++;
  }
#endif
}

static void finalize_rtrh_measurement()
{
  if (!rtrh_collecting)
    return;

  if (rtrh_ref.count == 0 || rtrh_rt.count == 0 || rtrh_rh.count == 0) {
    rtrh_collecting = false;
    return;
  }

  RtRhSnapshot next{};
  next.ref = rtrh_ref;
  next.rt = rtrh_rt;
  next.rh = rtrh_rh;
  next.rt_temp_period_sum = rtrh_rt_temp_period_sum;
  next.rt_temp_count = rtrh_rt_temp_count;
  next.sequence = rtrh_snapshot.sequence + 1;

  rtrh_snapshot = next;
  rtrh_snapshot_ready = true;
  rtrh_collecting = false;
}

static inline void IRAM_ATTR process_rtrh_g10_edge(
    uint32_t now,
    uint8_t new_g10
#if RTRH_DEBUG_CAPTURE
    , uint8_t old_state,
    uint8_t new_state
#endif
)
{
  if (!rtrh_collecting)
    reset_rtrh_measurement(now);
  else
    rtrh_last_any_us = now;

  const uint8_t old_g10 = rtrh_last_g10;
  rtrh_last_g10 = new_g10;

  if (!old_g10 && new_g10) {
    rtrh_g10_rise_us = now;
    rtrh_have_g10_rise = true;
#if RTRH_DEBUG_CAPTURE
    const bool new_g13 = (new_state & 0x08) != 0;
    rtrh_have_g13_rise = new_g13;
    if (new_g13)
      rtrh_g13_rise_us = now;
#endif
    return;
  }

  if (!(old_g10 && !new_g10))
    return;

  const uint32_t previous_fall = rtrh_last_g10_fall_us;
  rtrh_last_g10_fall_us = now;

  if (previous_fall == 0 || !rtrh_have_g10_rise)
    goto done;

  {
    const uint32_t period = static_cast<uint32_t>(now - previous_fall);
    if (period >= RTRH_CYCLE_GAP_US)
      goto done;

    const uint32_t low =
        static_cast<uint32_t>(rtrh_g10_rise_us - previous_fall);
    const bool is_ref =
        period >= RTRH_REF_MIN_US && period <= RTRH_REF_MAX_US;
    const bool is_rt =
        period >= RTRH_RT_MIN_US && period <= RTRH_RT_MAX_US;

#if RTRH_DEBUG_CAPTURE
    const bool have_delay =
        rtrh_have_g13_rise &&
        static_cast<int32_t>(rtrh_g13_rise_us - rtrh_g10_rise_us) >= 0;
    const uint32_t delay =
        have_delay
            ? static_cast<uint32_t>(rtrh_g13_rise_us - rtrh_g10_rise_us)
            : 0;
#define RTRH_ADD(stats) add_rtrh_cycle((stats), period, low, have_delay, delay)
#else
#define RTRH_ADD(stats) add_rtrh_cycle((stats), period, low)
#endif

    switch (rtrh_phase) {
      case RtRhPhase::WAIT_REF:
        if (is_ref) {
          rtrh_phase = RtRhPhase::REF;
          RTRH_ADD(rtrh_ref);
        }
        break;

      case RtRhPhase::REF:
        if (is_ref) {
          rtrh_phase_candidate_run = 0;
          RTRH_ADD(rtrh_ref);
        } else if (is_rt) {
          if (rtrh_phase_candidate_run < 255)
            rtrh_phase_candidate_run++;

          if (rtrh_phase_candidate_run >= RTRH_PHASE_LOCK_CYCLES) {
            rtrh_phase = RtRhPhase::RT;
            rtrh_phase_candidate_run = 0;
            RTRH_ADD(rtrh_rt);

            if (rtrh_rt_temp_count < RTRH_RT_TEMP_CYCLES) {
              rtrh_rt_temp_period_sum += period;
              rtrh_rt_temp_count++;
            }
          }
        } else {
          rtrh_phase_candidate_run = 0;
        }
        break;

      case RtRhPhase::RT:
        if (rtrh_rt.count >= RTRH_RT_FORCE_END_CYCLES) {
          rtrh_phase = RtRhPhase::RH;
          rtrh_phase_candidate_run = 0;
          RTRH_ADD(rtrh_rh);
        } else if (is_rt) {
          rtrh_phase_candidate_run = 0;
          RTRH_ADD(rtrh_rt);

          if (rtrh_rt_temp_count < RTRH_RT_TEMP_CYCLES) {
            rtrh_rt_temp_period_sum += period;
            rtrh_rt_temp_count++;
          }
        } else if (rtrh_rt.count < RTRH_RT_MIN_BEFORE_RH) {
          rtrh_phase_candidate_run = 0;
        } else {
          if (rtrh_phase_candidate_run < 255)
            rtrh_phase_candidate_run++;

          if (rtrh_phase_candidate_run >= RTRH_PHASE_LOCK_CYCLES) {
            rtrh_phase = RtRhPhase::RH;
            rtrh_phase_candidate_run = 0;
            RTRH_ADD(rtrh_rh);
          }
        }
        break;

      case RtRhPhase::RH:
        RTRH_ADD(rtrh_rh);
        break;
    }

#undef RTRH_ADD
  }

done:
  rtrh_have_g10_rise = false;
#if RTRH_DEBUG_CAPTURE
  rtrh_have_g13_rise = false;
#endif
}

#if RTRH_DEBUG_CAPTURE
static inline void IRAM_ATTR capture_rtrh_state(uint32_t now, uint8_t value)
{
  if (rtrh_capture_ready || value == rtrh_capture_last_value)
    return;

  rtrh_capture_last_value = value;

  if (!rtrh_capturing) {
    rtrh_capturing = true;
    rtrh_capture_start_us = now;
    rtrh_sample_count = 0;
    rtrh_capture_edge_no = 0;
    rtrh_capture_unusual_no = 0;
    rtrh_capture_overflow = false;
  }

  const uint16_t edge_no = rtrh_capture_edge_no++;
  const bool unusual = value != 0x00 && value != 0x0F;
  const bool regular_anchor =
      edge_no == 0 || (edge_no % RTRH_CAPTURE_DECIMATION) == 0;

  bool unusual_anchor = false;
  if (unusual) {
    const uint16_t unusual_no = rtrh_capture_unusual_no++;
    unusual_anchor =
        unusual_no == 0 ||
        (unusual_no % RTRH_CAPTURE_UNUSUAL_DECIMATION) == 0;
  }

  if (!regular_anchor && !unusual_anchor)
    return;

  const uint16_t index = rtrh_sample_count;
  if (index >= RTRH_MAX_SAMPLES) {
    rtrh_capture_overflow = true;
    rtrh_capturing = false;
    rtrh_capture_ready = true;
    rtrh_capture_sequence++;
    return;
  }

  rtrh_samples[index].t_us =
      static_cast<uint32_t>(now - rtrh_capture_start_us);
  rtrh_samples[index].edge_no = edge_no;
  rtrh_samples[index].value = value;
  rtrh_sample_count = index + 1;
}
#endif

static void IRAM_ATTR rtrh_gpio_isr(void *arg)
{
  const intptr_t encoded = reinterpret_cast<intptr_t>(arg);
  const uint8_t pin_index =
      encoded >= 1 && encoded <= 4
          ? static_cast<uint8_t>(encoded - 1)
          : 0;

#if RTRH_DEBUG_CAPTURE
  const gpio_num_t pin = RTRH_PINS[pin_index];
  const uint8_t level = static_cast<uint8_t>(gpio_get_level(pin));
  if (level == rtrh_pin_level[pin_index])
    return;
  rtrh_pin_level[pin_index] = level;

  const uint32_t now = static_cast<uint32_t>(esp_timer_get_time());
  const uint8_t old_state = rtrh_last_state;
  const uint8_t state = read_rtrh_state();
  if (state == old_state)
    return;
  rtrh_last_state = state;

  const bool old_g13 = (old_state & 0x08) != 0;
  const bool new_g13 = (state & 0x08) != 0;
  if (!old_g13 && new_g13 && rtrh_have_g10_rise) {
    rtrh_g13_rise_us = now;
    rtrh_have_g13_rise = true;
  }

  const uint8_t old_g10 = (old_state & 0x01) ? 1 : 0;
  const uint8_t new_g10 = (state & 0x01) ? 1 : 0;
  if (old_g10 != new_g10)
    process_rtrh_g10_edge(now, new_g10, old_state, state);

  capture_rtrh_state(now, state);
#else
  (void) pin_index;
  const uint32_t now = static_cast<uint32_t>(esp_timer_get_time());
  const uint8_t level = static_cast<uint8_t>(gpio_get_level(PIN_RTRH0));
  if (level == rtrh_last_g10)
    return;
  process_rtrh_g10_edge(now, level);
#endif
}

static inline void rtrh_disable_irqs()
{
  gpio_intr_disable(PIN_RTRH0);
#if RTRH_DEBUG_CAPTURE
  gpio_intr_disable(PIN_RTRH1);
  gpio_intr_disable(PIN_RTRH2);
  gpio_intr_disable(PIN_RTRH3);
#endif
}

static inline void rtrh_enable_irqs()
{
  gpio_intr_enable(PIN_RTRH0);
#if RTRH_DEBUG_CAPTURE
  gpio_intr_enable(PIN_RTRH1);
  gpio_intr_enable(PIN_RTRH2);
  gpio_intr_enable(PIN_RTRH3);
#endif
}

#if RTRH_DEBUG_CAPTURE
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
        req, "Content-Disposition",
        "attachment; filename=\"rt_rh_capture.csv\"");

    static constexpr char HEADER[] =
        "sequence,t_us,edge_no,gpio10,gpio11,gpio12,gpio13,state,overflow\n";
    esp_err_t err = httpd_resp_send_chunk(req, HEADER, sizeof(HEADER) - 1);

    const uint16_t count = rtrh_sample_count;
    const uint32_t sequence = rtrh_capture_sequence;
    const bool overflow = rtrh_capture_overflow;
    static char chunk[128];
    static char line[80];
    size_t used = 0;

    for (uint16_t i = 0; i < count && err == ESP_OK; i++) {
      const uint32_t stamp = rtrh_samples[i].t_us;
      const uint16_t edge_no = rtrh_samples[i].edge_no;
      const uint8_t v = rtrh_samples[i].value;

      const int n = snprintf(
          line, sizeof(line),
          "%lu,%lu,%u,%u,%u,%u,%u,0x%02X,%u\n",
          static_cast<unsigned long>(sequence),
          static_cast<unsigned long>(stamp),
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

    rtrh_sample_count = 0;
    rtrh_capture_edge_no = 0;
    rtrh_capture_unusual_no = 0;
    rtrh_capture_overflow = false;
    rtrh_capture_ready = false;
    rtrh_capturing = false;
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
    if (!rtrh_snapshot_ready) {
      request->send(204, "text/plain", nullptr);
      return;
    }

    const RtRhSnapshot s = rtrh_snapshot;

    httpd_req_t *req = *request;
    httpd_resp_set_status(req, "200 OK");
    httpd_resp_set_type(req, "text/csv");
    httpd_resp_set_hdr(
        req, "Content-Disposition",
        "attachment; filename=\"rt_rh_timing.csv\"");

    static constexpr char HEADER[] =
        "measurement,train,role,count,gap_before_us,period_mean_us,"
        "period_min_us,period_max_us,duration_ms,frequency_hz,"
        "low_mean_us,low_min_us,low_max_us,g13_delay_count,"
        "g13_delay_mean_us,g13_delay_min_us,g13_delay_max_us,"
        "d0,d1,d2,d3,d4,d5,d6,d7plus,overflow\n";

    esp_err_t err = httpd_resp_send_chunk(req, HEADER, sizeof(HEADER) - 1);

    const RtRhPhaseStats *phases[3] = {&s.ref, &s.rt, &s.rh};
    static constexpr const char *names[3] = {"ref", "rt", "rh"};
    static char line[320];

    for (uint8_t i = 0; i < 3 && err == ESP_OK; i++) {
      const RtRhPhaseStats &p = *phases[i];
      const float mean = p.count
          ? static_cast<float>(p.period_sum) / static_cast<float>(p.count)
          : 0.0f;
      const float low_mean = p.count
          ? static_cast<float>(p.low_sum) / static_cast<float>(p.count)
          : 0.0f;
      const float delay_mean = p.g13_delay_count
          ? static_cast<float>(p.g13_delay_sum) /
                static_cast<float>(p.g13_delay_count)
          : 0.0f;
      const float duration_ms = static_cast<float>(p.period_sum) / 1000.0f;
      const float frequency_hz = mean > 0.0f ? 1000000.0f / mean : 0.0f;

      const int n = snprintf(
          line, sizeof(line),
          "%lu,%u,%s,%u,0,%.3f,%u,%u,%.3f,%.2f,%.3f,%u,%u,"
          "%u,%.3f,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,0\n",
          static_cast<unsigned long>(s.sequence),
          static_cast<unsigned>(i),
          names[i],
          static_cast<unsigned>(p.count),
          mean,
          p.period_min == 0xFFFF ? 0 : p.period_min,
          p.period_max,
          duration_ms,
          frequency_hz,
          low_mean,
          p.low_min == 0xFFFF ? 0 : p.low_min,
          p.low_max,
          static_cast<unsigned>(p.g13_delay_count),
          delay_mean,
          p.g13_delay_min == 0xFFFF ? 0 : p.g13_delay_min,
          p.g13_delay_max,
          p.g13_delay_hist[0], p.g13_delay_hist[1],
          p.g13_delay_hist[2], p.g13_delay_hist[3],
          p.g13_delay_hist[4], p.g13_delay_hist[5],
          p.g13_delay_hist[6], p.g13_delay_hist[7]);

      if (n > 0)
        err = httpd_resp_send_chunk(req, line, static_cast<size_t>(n));
    }

    if (err == ESP_OK)
      httpd_resp_send_chunk(req, nullptr, 0);
  }
};

static RtRhTimingHandler rtrh_timing_handler;
#endif

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
#if RTRH_DEBUG_CAPTURE
  gpio_set_intr_type(PIN_RTRH1, GPIO_INTR_ANYEDGE);
  gpio_set_intr_type(PIN_RTRH2, GPIO_INTR_ANYEDGE);
  gpio_set_intr_type(PIN_RTRH3, GPIO_INTR_ANYEDGE);
#endif


  last_value =
      read_gpio_state();


  capture_initial_value =
      last_value;

  rtrh_last_g10 = static_cast<uint8_t>(gpio_get_level(PIN_RTRH0));
#if RTRH_DEBUG_CAPTURE
  rtrh_last_state = read_rtrh_state();
  rtrh_capture_last_value = rtrh_last_state;
  for (uint8_t i = 0; i < 4; i++)
    rtrh_pin_level[i] = static_cast<uint8_t>(gpio_get_level(RTRH_PINS[i]));
#endif

  // GPIO10..13 remain high-impedance digital inputs for the entire run.


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


#if RTRH_DEBUG_CAPTURE
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
#else
  err = gpio_isr_handler_add(
      PIN_RTRH0,
      rtrh_gpio_isr,
      reinterpret_cast<void *>(static_cast<intptr_t>(1)));
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "RT/RH ISR GPIO10 failed: %d", err);
    return;
  }
#endif


  if (
      web_server_base::global_web_server_base !=
      nullptr
  ) {

    web_server_base::
        global_web_server_base->
        add_handler(
            &capture_handler
        );

#if RTRH_DEBUG_CAPTURE
    web_server_base::global_web_server_base->add_handler(
        &rtrh_capture_handler);

    web_server_base::global_web_server_base->add_handler(
        &rtrh_timing_handler);
#endif

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


#if RTRH_DEBUG_CAPTURE
  ESP_LOGD(
      TAG,
      "RT/RH decoder ready (debug: 4 GPIO IRQs, /rt_rh_capture.csv, /rt_rh_timing.csv)"
  );
#else
  ESP_LOGD(
      TAG,
      "RT/RH decoder ready (production: GPIO10 IRQ only)"
  );
#endif
}


/*
 * ============================================================================
 * Loop
 * ============================================================================
 */

void BusSniffer::loop()
{
  // A whole RT/RH measurement is finalized after 15 s without any GPIO10 edge.
  if (rtrh_collecting) {
    const uint32_t now = static_cast<uint32_t>(esp_timer_get_time());
    const uint32_t last_any = rtrh_last_any_us;

    if (last_any != 0 &&
        static_cast<uint32_t>(now - last_any) > RTRH_MEASUREMENT_QUIET_US) {
      rtrh_disable_irqs();

      const uint32_t now2 = static_cast<uint32_t>(esp_timer_get_time());
      const uint32_t last_any2 = rtrh_last_any_us;
      if (rtrh_collecting &&
          last_any2 != 0 &&
          static_cast<uint32_t>(now2 - last_any2) >
              RTRH_MEASUREMENT_QUIET_US) {
        finalize_rtrh_measurement();
      }

      rtrh_last_g10 = static_cast<uint8_t>(gpio_get_level(PIN_RTRH0));
#if RTRH_DEBUG_CAPTURE
      rtrh_last_state = read_rtrh_state();
      for (uint8_t i = 0; i < 4; i++)
        rtrh_pin_level[i] =
            static_cast<uint8_t>(gpio_get_level(RTRH_PINS[i]));
#endif
      rtrh_enable_irqs();
    }
  }

  static uint32_t last_rtrh_sequence = 0;
  if (rtrh_snapshot_ready &&
      rtrh_snapshot.sequence != last_rtrh_sequence) {
    const RtRhSnapshot s = rtrh_snapshot;
    last_rtrh_sequence = s.sequence;

    const float ref_period_us =
        s.ref.count
            ? static_cast<float>(s.ref.period_sum) /
                  static_cast<float>(s.ref.count)
            : 0.0f;
    const float rt_phase_period_us =
        s.rt.count
            ? static_cast<float>(s.rt.period_sum) /
                  static_cast<float>(s.rt.count)
            : 0.0f;
    const float rh_period_us =
        s.rh.count
            ? static_cast<float>(s.rh.period_sum) /
                  static_cast<float>(s.rh.count)
            : 0.0f;

    const float ref_duration_ms =
        static_cast<float>(s.ref.period_sum) / 1000.0f;
    const float rt_duration_ms =
        static_cast<float>(s.rt.period_sum) / 1000.0f;
    const float rh_duration_ms =
        static_cast<float>(s.rh.period_sum) / 1000.0f;

    const bool ref_ok =
        ref_period_us >= RTRH_REF_VALID_MIN_US &&
        ref_period_us <= RTRH_REF_VALID_MAX_US &&
        ref_duration_ms >= RTRH_REF_DURATION_MIN_MS &&
        ref_duration_ms <= RTRH_REF_DURATION_MAX_MS &&
        s.ref.count >= RTRH_REF_COUNT_MIN &&
        s.ref.count <= RTRH_REF_COUNT_MAX;

    const bool rt_ok =
        rt_duration_ms >= RTRH_RT_DURATION_MIN_MS &&
        rt_duration_ms <= RTRH_RT_DURATION_MAX_MS &&
        s.rt.count >= RTRH_RT_PHASE_COUNT_MIN &&
        s.rt.count <= RTRH_RT_PHASE_COUNT_MAX &&
        s.rt_temp_count >= 800;

    const bool rh_ok =
        rh_duration_ms >= RTRH_RH_DURATION_MIN_MS &&
        rh_duration_ms <= RTRH_RH_DURATION_MAX_MS &&
        s.rh.count != 0;

    const bool valid = ref_ok && rt_ok && rh_ok;

#if RTRH_DEBUG_CAPTURE
    const RtRhPhaseStats *phases[3] = {&s.ref, &s.rt, &s.rh};
    static constexpr const char *names[3] = {"ref", "rt", "rh"};

    ESP_LOGI(
        TAG, "RT/RH measurement %lu: 3 phases",
        static_cast<unsigned long>(s.sequence));

    for (uint8_t i = 0; i < 3; i++) {
      const RtRhPhaseStats &p = *phases[i];
      const float mean = p.count
          ? static_cast<float>(p.period_sum) / static_cast<float>(p.count)
          : 0.0f;
      const float low_mean = p.count
          ? static_cast<float>(p.low_sum) / static_cast<float>(p.count)
          : 0.0f;
      const float delay_mean = p.g13_delay_count
          ? static_cast<float>(p.g13_delay_sum) /
                static_cast<float>(p.g13_delay_count)
          : 0.0f;

      ESP_LOGI(
          TAG,
          "  %-3s: n=%u period=%.3f us low=%.3f us G13delay=%.3f us (%u)",
          names[i],
          static_cast<unsigned>(p.count),
          mean,
          low_mean,
          delay_mean,
          static_cast<unsigned>(p.g13_delay_count));
    }
#endif

    ESP_LOGI(
        TAG,
        "RT/RH quality: REF %.3f us / %.3f ms / %u, "
        "RT %.3f ms / %u, RH %.3f ms / %u -> %s",
        ref_period_us,
        ref_duration_ms,
        static_cast<unsigned>(s.ref.count),
        rt_duration_ms,
        static_cast<unsigned>(s.rt.count),
        rh_duration_ms,
        static_cast<unsigned>(s.rh.count),
        valid ? "VALID" : "REJECT");

    if (!valid) {
      ESP_LOGW(
          TAG,
          "RT/RH values not published: measurement failed quality checks");
    } else {
      const float rt_period_us =
          static_cast<float>(s.rt_temp_period_sum) /
          static_cast<float>(s.rt_temp_count);
      const float rt_ratio = rt_period_us / ref_period_us;
      const float temperature_c =
          RTRH_TEMP_RATIO_M * rt_ratio + RTRH_TEMP_RATIO_C;

      const float rh_ratio = rh_period_us / ref_period_us;
      const float x = logf(rh_ratio);
      float rh_percent =
          RTRH_RH_RATIO_A * x * x +
          RTRH_RH_RATIO_B * x +
          RTRH_RH_RATIO_C;
      if (rh_percent < 0.0f) rh_percent = 0.0f;
      if (rh_percent > 100.0f) rh_percent = 100.0f;

      ESP_LOGI(
          TAG,
          "RT normalized: ratio=%.6f (RT %.3f / REF %.3f us)",
          rt_ratio, rt_period_us, ref_period_us);
      ESP_LOGI(
          TAG,
          "RH normalized: ratio=%.6f (RH %.3f / REF %.3f us)",
          rh_ratio, rh_period_us, ref_period_us);
      ESP_LOGI(
          TAG,
          "RH humidity: %.1f %% ; RT temperature: %.2f C",
          rh_percent, temperature_c);

      if (this->rh_humidity_sensor_ != nullptr)
        this->rh_humidity_sensor_->publish_state(rh_percent);
      if (this->rt_temperature_sensor_ != nullptr)
        this->rt_temperature_sensor_->publish_state(temperature_c);
    }
  }

#if RTRH_DEBUG_CAPTURE
  if (rtrh_capture_ready) {
    static uint32_t last_reported_capture_sequence = UINT32_MAX;
    if (last_reported_capture_sequence != rtrh_capture_sequence) {
      last_reported_capture_sequence = rtrh_capture_sequence;
      ESP_LOGI(
          TAG,
          "RT/RH edge capture ready: %u events, sequence %lu%s",
          rtrh_sample_count,
          static_cast<unsigned long>(rtrh_capture_sequence),
          rtrh_capture_overflow ? " OVERFLOW" : "");
    }
  }

  if (rtrh_capturing && !rtrh_capture_ready) {
    const uint32_t now = static_cast<uint32_t>(esp_timer_get_time());
    if (static_cast<uint32_t>(now - rtrh_capture_start_us) >=
        RTRH_CAPTURE_US) {
      rtrh_capturing = false;
      rtrh_capture_ready = true;
      rtrh_capture_sequence++;
    }
  }
#endif

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
