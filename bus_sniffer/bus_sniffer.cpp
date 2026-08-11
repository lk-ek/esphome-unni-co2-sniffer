#include "bus_sniffer.h"
#include "calibration.h"
#include "measurement_quality.h"
#include "ble_options.h"
#if UNNI_BLE_ENABLED
#include "sensirion_ble.h"
#endif
#if UNNI_BLE_HISTORY_ENABLED
#include "sensirion_history.h"
#endif

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
#include <cstring>
#include <string>
#include <utility>
#include <vector>

namespace esphome {
namespace bus_sniffer {

static const char *TAG = "bus_sniffer";

/*
 * ============================================================================
 * Hardware
 * ============================================================================
 *
 * XIAO ESP32-C3 pin mapping:
 *
 * D5 / GPIO7 = CO2 SCL  (yellow; old ESP32-S2 GPIO40)
 * D4 / GPIO6 = CO2 SDA  (blue;   old ESP32-S2 GPIO39)
 * D1 / GPIO3 = RT/RH G10 (green;  old ESP32-S2 GPIO10)
 * D3 / GPIO5 = RT/RH G11 (blue;   old ESP32-S2 GPIO11)
 * D2 / GPIO4 = RT/RH G13 (yellow; old ESP32-S2 GPIO13)
 *
 * The former RT/RH G12 net is omitted because it duplicates G10.
 * The old extra CO2 logic-analyzer channel (ESP32-S2 GPIO38) is omitted.
 */

static constexpr gpio_num_t PIN_SCL =
    GPIO_NUM_7;

static constexpr gpio_num_t PIN_SDA =
    GPIO_NUM_6;


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
 * RT/RH minimal hybrid decoder
 * ============================================================================
 *
 * Three RT/RH GPIO interrupts remain active on the XIAO:
 * GPIO3 / D1 observes the former ESP32-S2 GPIO10 sensor net.
 * GPIO5 / D3 observes the former ESP32-S2 GPIO11 sensor net.
 * GPIO4 / D2 observes the former ESP32-S2 GPIO13 sensor net.
 * Former GPIO12 duplicates G10 and remains omitted.
 * Every interrupt re-reads the complete observed three-line state.
 * REF/RT edge timing is accepted only from the actual GPIO3/D1 IRQ.
 *
 * Phase identity is based solely on elapsed time from the first edge after the
 * long inter-measurement quiet interval:
 *   REF:   0 .. 125 ms
 *   RT : 125 .. 252 ms
 *   RH : 252 ms .. end of activity
 *
 * The measured period and the number of cycles do NOT select the phase.  This
 * keeps extreme temperatures decodable even when RT becomes shorter than REF
 * or far longer than the old 105..190 us range.
 *
 * Measurement:
 *   REF: physical G10-net (XIAO GPIO3/D1) falling-edge IRQ period sum/count
 *   RT : physical G10-net (XIAO GPIO3/D1) falling-edge IRQ period sum/count;
 *        first up to 880 cycles for temperature
 *   RH : humidity from repeated arrivals at G10=0, G11=0, G13=1
 *
 * The G10-net-derived RH period is retained ONLY as a phase-duration /
 * quality accumulator.  It is never converted to humidity.
 *
 * Set RTRH_DEBUG_CAPTURE=0 for the lean production build.  This removes the
 * raw RT/RH capture buffer and both RT/RH CSV handlers without changing the
 * three-GPIO decoder.
 */

#ifndef RTRH_DEBUG_CAPTURE
#define RTRH_DEBUG_CAPTURE 1
#endif

static constexpr gpio_num_t PIN_RTRH0 = GPIO_NUM_3;
static constexpr gpio_num_t PIN_RTRH1 = GPIO_NUM_5;
static constexpr gpio_num_t PIN_RTRH3 = GPIO_NUM_4;

static constexpr int RTRH_GPIOS[3] = {3, 5, 4};

// Calibration lives in calibration.h.  The decoder only produces normalized
// RT/REF and RH-state/REF ratios and passes them to that module.

// The breath test exposed an alias/failure mode where only 12 RH recurrence
// samples were seen and a 6.7 ms pseudo-period was nevertheless accepted.
// Normal captures fill all 96 slots.  Require enough independent recurrences
// and reject implausibly large state/REF ratios.

// Measurement quality limits.
static constexpr float RTRH_REF_VALID_MIN_US = 72.0f;
static constexpr float RTRH_REF_VALID_MAX_US = 82.0f;
static constexpr float RTRH_REF_DURATION_MIN_MS = 115.0f;
static constexpr float RTRH_REF_DURATION_MAX_MS = 135.0f;
static constexpr float RTRH_RT_DURATION_MIN_MS = 110.0f;
static constexpr float RTRH_RT_DURATION_MAX_MS = 135.0f;
static constexpr float RTRH_RH_DURATION_MIN_MS = 110.0f;
static constexpr float RTRH_RH_DURATION_MAX_MS = 145.0f;
static constexpr uint16_t RTRH_REF_COUNT_MIN = 1450;
static constexpr uint16_t RTRH_REF_COUNT_MAX = 1750;
static constexpr uint16_t RTRH_RT_TEMP_COUNT_MIN = 64;

// Time-based phase decoder.
//
// The original controller spends about 125 ms in REF, then about 127 ms in RT.
// Those phase lengths stay nearly constant while the actual RC period changes
// strongly with temperature/humidity.  Therefore period/count must never decide
// which phase we are in.
static constexpr uint32_t RTRH_MEASUREMENT_QUIET_US = 15000000;
static constexpr uint32_t RTRH_REF_PHASE_END_US = 125000;
static constexpr uint32_t RTRH_RT_PHASE_END_US = 252000;

// Reject only implausibly long "cycles" inside a phase.  This is deliberately
// much wider than the old 2 ms / 60..190 us classifier so hot/cold extremes
// remain decodable.
static constexpr uint32_t RTRH_CYCLE_MAX_US = 20000;

// Preserve the historical "first 880 RT cycles" averaging when available.
// At low cycle counts all available cycles are used; quality no longer depends
// on reaching a temperature-specific number of cycles.
static constexpr uint16_t RTRH_RT_TEMP_CYCLES = 880;

enum RtRhPhase : uint8_t {
  RTRH_PHASE_WAIT_REF = 0,
  RTRH_PHASE_REF,
  RTRH_PHASE_RT,
  RTRH_PHASE_RH,
};

struct RtRhAccum {
  uint32_t period_sum{0};
  uint16_t count{0};
};

static constexpr uint8_t RTRH_RH_STATE_PERIOD_SAMPLES = 96;

struct RtRhRhStateStats {
  uint32_t last_us{0};
  uint16_t samples[RTRH_RH_STATE_PERIOD_SAMPLES]{};
  uint8_t write_pos{0};
  uint8_t sample_count{0};
  uint32_t seen{0};
};

struct RtRhSnapshot {
  RtRhAccum ref;
  RtRhAccum rt;
  RtRhAccum rh_timing;

  uint32_t rt_temp_period_sum{0};
  uint16_t rt_temp_count{0};

  RtRhRhStateStats rh_state;

  uint32_t sequence{0};
};

static volatile bool rtrh_collecting = false;
static volatile uint32_t rtrh_measurement_start_us = 0;
static volatile uint32_t rtrh_last_any_us = 0;
static volatile uint8_t rtrh_last_state = 0;

static volatile uint32_t rtrh_last_g10_fall_us = 0;
static volatile bool rtrh_have_g10_rise = false;

static volatile uint8_t rtrh_phase = RTRH_PHASE_WAIT_REF;

static RtRhAccum rtrh_ref;
static RtRhAccum rtrh_rt;
static RtRhAccum rtrh_rh_timing;

static volatile uint32_t rtrh_rt_temp_period_sum = 0;
static volatile uint16_t rtrh_rt_temp_count = 0;

static RtRhRhStateStats rtrh_rh_state;

static RtRhSnapshot rtrh_snapshot;
static volatile bool rtrh_snapshot_ready = false;

struct RtRhDerived {
  uint32_t sequence{0};
  bool valid{false};
  float rt_ratio{NAN};
  float rh_ratio{NAN};
  float temperature_c{NAN};
  float humidity_percent{NAN};
  float quality_percent{0.0f};
  measurement_quality::RejectReason reject_reason{
      measurement_quality::RejectReason::NONE};
  bool thermal_transient{false};
  bool temperature_extrapolation{false};
  bool humidity_extrapolation{false};
  bool calibration_extrapolation{false};
};

static RtRhDerived rtrh_derived;

static volatile uint8_t rtrh_pin_level[3] = {0, 0, 0};

static inline uint8_t IRAM_ATTR read_rtrh_state()
{
  uint8_t value = 0;
  if (gpio_get_level(PIN_RTRH0)) value |= 0x01;
  if (gpio_get_level(PIN_RTRH1)) value |= 0x02;
  if (gpio_get_level(PIN_RTRH3)) value |= 0x08;
  return value;
}

static inline void IRAM_ATTR clear_rtrh_accum(RtRhAccum &a)
{
  a.period_sum = 0;
  a.count = 0;
}

static inline void IRAM_ATTR clear_rtrh_rh_state()
{
  rtrh_rh_state.last_us = 0;
  rtrh_rh_state.write_pos = 0;
  rtrh_rh_state.sample_count = 0;
  rtrh_rh_state.seen = 0;

  for (uint8_t i = 0; i < RTRH_RH_STATE_PERIOD_SAMPLES; i++)
    rtrh_rh_state.samples[i] = 0;
}

static inline void IRAM_ATTR reset_rtrh_measurement(
    uint32_t now,
    uint8_t state)
{
  rtrh_collecting = true;
  rtrh_measurement_start_us = now;
  rtrh_last_any_us = now;
  rtrh_last_state = state;

  rtrh_last_g10_fall_us = 0;
  rtrh_have_g10_rise = false;

  // The first edge after the long quiet interval is phase zero: REF.
  rtrh_phase = RTRH_PHASE_REF;

  clear_rtrh_accum(rtrh_ref);
  clear_rtrh_accum(rtrh_rt);
  clear_rtrh_accum(rtrh_rh_timing);

  rtrh_rt_temp_period_sum = 0;
  rtrh_rt_temp_count = 0;

  clear_rtrh_rh_state();
}

static inline void IRAM_ATTR add_rtrh_period(
    RtRhAccum &a,
    uint32_t period)
{
  if (a.count != 0xFFFF)
    a.count++;

  a.period_sum += period;
}

static inline void IRAM_ATTR update_rtrh_phase_by_time(
    uint32_t now)
{
  const uint32_t elapsed =
      static_cast<uint32_t>(
          now - rtrh_measurement_start_us);

  uint8_t next_phase;

  if (elapsed < RTRH_REF_PHASE_END_US)
    next_phase = RTRH_PHASE_REF;
  else if (elapsed < RTRH_RT_PHASE_END_US)
    next_phase = RTRH_PHASE_RT;
  else
    next_phase = RTRH_PHASE_RH;

  if (next_phase == rtrh_phase)
    return;

  rtrh_phase = next_phase;

  // Never let a period straddle a fixed phase boundary.
  rtrh_last_g10_fall_us = 0;
  rtrh_have_g10_rise = false;

  // RH recurrence timing starts fresh at the RH boundary.
  if (next_phase == RTRH_PHASE_RH)
    rtrh_rh_state.last_us = 0;
}


static inline void IRAM_ATTR observe_rtrh_rh_state(
    uint32_t now,
    uint8_t state)
{
  // Characteristic RH state on the three observed lines:
  // G10-net (GPIO3)=0, G11-net (GPIO5)=0, G13-net (GPIO4)=1.
  const bool rh_state =
      (state & 0x0B) == 0x08;

  if (rtrh_phase != RTRH_PHASE_RH || !rh_state)
    return;

  if (rtrh_rh_state.last_us != 0) {
    const uint32_t dt =
        static_cast<uint32_t>(now - rtrh_rh_state.last_us);

    if (dt >= 40 && dt <= 60000) {
      rtrh_rh_state.samples[rtrh_rh_state.write_pos] =
          static_cast<uint16_t>(dt);

      rtrh_rh_state.write_pos =
          static_cast<uint8_t>(
              (rtrh_rh_state.write_pos + 1) %
              RTRH_RH_STATE_PERIOD_SAMPLES);

      if (rtrh_rh_state.sample_count < RTRH_RH_STATE_PERIOD_SAMPLES)
        rtrh_rh_state.sample_count++;
    }
  }

  rtrh_rh_state.last_us = now;
  rtrh_rh_state.seen++;
}

static float rtrh_rh_state_period_median(
    const RtRhRhStateStats &s)
{
  const uint8_t n = s.sample_count;
  if (n == 0)
    return 0.0f;

  uint16_t tmp[RTRH_RH_STATE_PERIOD_SAMPLES];

  for (uint8_t i = 0; i < n; i++)
    tmp[i] = s.samples[i];

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

static void finalize_rtrh_measurement()
{
  if (!rtrh_collecting)
    return;

  if (rtrh_ref.count == 0 ||
      rtrh_rt.count == 0 ||
      rtrh_rh_timing.count == 0) {
    rtrh_collecting = false;
    return;
  }

  RtRhSnapshot next{};

  next.ref = rtrh_ref;
  next.rt = rtrh_rt;
  next.rh_timing = rtrh_rh_timing;

  next.rt_temp_period_sum = rtrh_rt_temp_period_sum;
  next.rt_temp_count = rtrh_rt_temp_count;

  next.rh_state = rtrh_rh_state;

  next.sequence = rtrh_snapshot.sequence + 1;

  rtrh_snapshot = next;
  rtrh_snapshot_ready = true;
  rtrh_collecting = false;
}

#if RTRH_DEBUG_CAPTURE
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
#endif

static void IRAM_ATTR rtrh_gpio_isr(void *arg)
{
  const intptr_t encoded = reinterpret_cast<intptr_t>(arg);
  if (encoded < 1 || encoded > 3)
    return;

  const uint8_t pin_index =
      static_cast<uint8_t>(encoded - 1);
  const gpio_num_t pin =
      static_cast<gpio_num_t>(RTRH_GPIOS[pin_index]);

  const uint8_t level =
      static_cast<uint8_t>(gpio_get_level(pin));
  const uint8_t previous =
      rtrh_pin_level[pin_index];

  if (level == previous)
    return;

  rtrh_pin_level[pin_index] = level;

  const uint32_t now =
      static_cast<uint32_t>(esp_timer_get_time());
  const uint8_t state = read_rtrh_state();

  if (!rtrh_collecting)
    reset_rtrh_measurement(now, state);
  else
    rtrh_last_any_us = now;

  // Phase identity comes only from elapsed time in the measurement cycle.
  update_rtrh_phase_by_time(now);

  /*
   * REF/RT timing must be tied to the interrupt from the physical G10 net
   * itself (XIAO GPIO3 / D1).  Reconstructing a G10 edge from a later
   * three-line state read is ordering-dependent when several sensor nets
   * switch almost simultaneously, which differs between ESP32-S2 and C3.
   */
  const bool is_g10_irq = pin_index == 0;

  if (is_g10_irq && previous == 0 && level != 0)
    rtrh_have_g10_rise = true;

  if (is_g10_irq && previous != 0 && level == 0) {
    const uint32_t previous_fall =
        rtrh_last_g10_fall_us;

    rtrh_last_g10_fall_us = now;

    if (previous_fall != 0 &&
        rtrh_have_g10_rise) {
      const uint32_t period =
          static_cast<uint32_t>(
              now - previous_fall);

      if (period <= RTRH_CYCLE_MAX_US) {
        switch (rtrh_phase) {
          case RTRH_PHASE_REF:
            add_rtrh_period(
                rtrh_ref,
                period);
            break;

          case RTRH_PHASE_RT:
            add_rtrh_period(
                rtrh_rt,
                period);

            if (rtrh_rt_temp_count <
                RTRH_RT_TEMP_CYCLES) {
              rtrh_rt_temp_period_sum +=
                  period;
              rtrh_rt_temp_count++;
            }
            break;

          case RTRH_PHASE_RH:
            // G10 timing is only a phase-duration/quality accumulator here.
            add_rtrh_period(
                rtrh_rh_timing,
                period);
            break;

          case RTRH_PHASE_WAIT_REF:
          default:
            break;
        }
      }
    }

    rtrh_have_g10_rise = false;
  }

  /*
   * RH still intentionally uses the complete post-edge three-line state.
   * This is independent of which of the three GPIOs caused this ISR.
   */
  const uint8_t old_state = rtrh_last_state;

  if (state != old_state) {
    observe_rtrh_rh_state(now, state);
    rtrh_last_state = state;
  }

#if RTRH_DEBUG_CAPTURE
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

  const bool unusual_state =
      value != 0x00 && value != 0x0F;
  const bool time_anchor =
      edge_no == 0 ||
      (edge_no % RTRH_CAPTURE_DECIMATION) == 0;

  bool unusual_anchor = false;

  if (unusual_state) {
    const uint16_t unusual_no =
        rtrh_capture_unusual_no++;

    unusual_anchor =
        unusual_no == 0 ||
        (unusual_no %
         RTRH_CAPTURE_UNUSUAL_DECIMATION) == 0;
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
#endif
}

#if RTRH_DEBUG_CAPTURE
class RtRhCaptureHandler
    : public web_server_idf::AsyncWebHandler {
 public:
  bool canHandle(
      web_server_idf::AsyncWebServerRequest *request)
      const override
  {
    if (request->method() != HTTP_GET)
      return false;

    char url[
        web_server_idf::AsyncWebServerRequest::URL_BUF_SIZE];

    return request->url_to(url) ==
        "/rt_rh_capture.csv";
  }

  void handleRequest(
      web_server_idf::AsyncWebServerRequest *request)
      override
  {
    if (!rtrh_capture_ready ||
        rtrh_sample_count == 0) {
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
        "sequence,t_us,edge_no,gpio10,gpio11,"
        "gpio13,state,overflow\n";

    esp_err_t err =
        httpd_resp_send_chunk(
            req, HEADER, sizeof(HEADER) - 1);

    const uint16_t count = rtrh_sample_count;
    const uint32_t sequence = rtrh_sequence;
    const bool overflow = rtrh_overflow;

    static char chunk[128];
    static char line[80];
    size_t used = 0;

    for (uint16_t i = 0;
         i < count && err == ESP_OK;
         i++) {
      const uint32_t stamp =
          rtrh_samples[i].t_us;
      const uint16_t edge_no =
          rtrh_samples[i].edge_no;
      const uint8_t v =
          rtrh_samples[i].value;

      const int n = snprintf(
          line,
          sizeof(line),
          "%lu,%lu,%u,%u,%u,%u,0x%02X,%u\n",
          static_cast<unsigned long>(sequence),
          static_cast<unsigned long>(stamp),
          static_cast<unsigned>(edge_no),
          (v & 0x01) ? 1U : 0U,
          (v & 0x02) ? 1U : 0U,
          (v & 0x08) ? 1U : 0U,
          v,
          overflow ? 1U : 0U);

      if (n <= 0)
        continue;

      const size_t line_len =
          static_cast<size_t>(n);

      if (used + line_len > sizeof(chunk)) {
        err =
            httpd_resp_send_chunk(
                req, chunk, used);
        used = 0;
      }

      if (err == ESP_OK &&
          line_len <= sizeof(chunk)) {
        memcpy(
            chunk + used,
            line,
            line_len);
        used += line_len;
      }
    }

    if (err == ESP_OK && used != 0)
      err =
          httpd_resp_send_chunk(
              req, chunk, used);

    if (err == ESP_OK)
      httpd_resp_send_chunk(req, nullptr, 0);

    rtrh_sample_count = 0;
    rtrh_capture_edge_no = 0;
    rtrh_capture_unusual_no = 0;
    rtrh_overflow = false;
    rtrh_capture_ready = false;
    rtrh_capturing = false;

    if (err != ESP_OK)
      ESP_LOGW(
          TAG,
          "rt_rh_capture.csv client disconnected (%d)",
          err);
  }
};

static RtRhCaptureHandler rtrh_capture_handler;

class RtRhTimingHandler
    : public web_server_idf::AsyncWebHandler {
 public:
  bool canHandle(
      web_server_idf::AsyncWebServerRequest *request)
      const override
  {
    if (request->method() != HTTP_GET)
      return false;

    char url[
        web_server_idf::AsyncWebServerRequest::URL_BUF_SIZE];

    return request->url_to(url) ==
        "/rt_rh_timing.csv";
  }

  void handleRequest(
      web_server_idf::AsyncWebServerRequest *request)
      override
  {
    if (!rtrh_snapshot_ready) {
      request->send(204, "text/plain", nullptr);
      return;
    }

    const RtRhSnapshot s = rtrh_snapshot;

    const float ref_us =
        s.ref.count
            ? static_cast<float>(s.ref.period_sum) /
                  static_cast<float>(s.ref.count)
            : 0.0f;

    const float rt_us =
        s.rt.count
            ? static_cast<float>(s.rt.period_sum) /
                  static_cast<float>(s.rt.count)
            : 0.0f;

    const float rh_timing_us =
        s.rh_timing.count
            ? static_cast<float>(
                  s.rh_timing.period_sum) /
                  static_cast<float>(
                  s.rh_timing.count)
            : 0.0f;

    const float rh_state_us =
        rtrh_rh_state_period_median(s.rh_state);

    const RtRhDerived d = rtrh_derived;
    const bool have_derived = d.sequence == s.sequence;

    char body[1024];

    const int n = snprintf(
        body,
        sizeof(body),
        "measurement,phase,count,period_mean_us,"
        "duration_ms,state_rh_median_us,state_rh_samples,"
        "state_rh_seen,valid,rt_ratio,rh_ratio,temperature_c,"
        "humidity_percent,quality_percent,reject_reason,"
        "thermal_transient,temperature_extrapolation,"
        "humidity_extrapolation,calibration_extrapolation\n"
        "%lu,ref,%u,%.3f,%.3f,,,,,,,,,,,,,,,,\n"
        "%lu,rt,%u,%.3f,%.3f,,,,,,,,,,,,,,,,\n"
        "%lu,rh,%u,%.3f,%.3f,%.3f,%u,%lu,%u,%.6f,%.6f,"
        "%.3f,%.3f,%.1f,%s,%u,%u,%u,%u\n",
        static_cast<unsigned long>(s.sequence),
        static_cast<unsigned>(s.ref.count),
        ref_us,
        static_cast<float>(s.ref.period_sum) / 1000.0f,
        static_cast<unsigned long>(s.sequence),
        static_cast<unsigned>(s.rt.count),
        rt_us,
        static_cast<float>(s.rt.period_sum) / 1000.0f,
        static_cast<unsigned long>(s.sequence),
        static_cast<unsigned>(s.rh_timing.count),
        rh_timing_us,
        static_cast<float>(
            s.rh_timing.period_sum) / 1000.0f,
        rh_state_us,
        static_cast<unsigned>(
            s.rh_state.sample_count),
        static_cast<unsigned long>(
            s.rh_state.seen),
        have_derived && d.valid ? 1U : 0U,
        have_derived ? d.rt_ratio : NAN,
        have_derived ? d.rh_ratio : NAN,
        have_derived ? d.temperature_c : NAN,
        have_derived ? d.humidity_percent : NAN,
        have_derived ? d.quality_percent : 0.0f,
        have_derived
            ? measurement_quality::reject_reason_to_string(d.reject_reason)
            : "UNKNOWN",
        have_derived && d.thermal_transient ? 1U : 0U,
        have_derived && d.temperature_extrapolation ? 1U : 0U,
        have_derived && d.humidity_extrapolation ? 1U : 0U,
        have_derived && d.calibration_extrapolation ? 1U : 0U);

    if (n <= 0) {
      request->send(500, "text/plain", nullptr);
      return;
    }

    httpd_req_t *req = *request;
    httpd_resp_set_status(req, "200 OK");
    httpd_resp_set_type(req, "text/csv");
    httpd_resp_set_hdr(
        req,
        "Content-Disposition",
        "attachment; filename=\"rt_rh_timing.csv\"");

    httpd_resp_send(
        req,
        body,
        static_cast<ssize_t>(n));
  }
};

static RtRhTimingHandler rtrh_timing_handler;
#endif

#if UNNI_BLE_ENABLED
void BusSniffer::configure_gatt_server(esp32_ble_server::BLEServer *server)
{
#if UNNI_BLE_HISTORY_ENABLED
  sensirion_history_configure_gatt(server);
#else
  (void) server;
#endif
}


void BusSniffer::set_ble_advertising_interval(uint32_t interval_ms)
{
  sensirion_ble_set_advertising_interval(interval_ms);
}

void BusSniffer::gap_event_handler(
    esp_gap_ble_cb_event_t event,
    esp_ble_gap_cb_param_t *param)
{
  sensirion_ble_gap_event_handler(event, param);
}


void BusSniffer::gatts_event_handler(
    esp_gatts_cb_event_t event,
    esp_gatt_if_t gatts_if,
    esp_ble_gatts_cb_param_t *param)
{
#if UNNI_BLE_HISTORY_ENABLED
  sensirion_history_gatts_event_handler(event, gatts_if, param);
#else
  (void) event;
  (void) gatts_if;
  (void) param;
#endif
}
#endif



/*
 * ============================================================================
 * Setup
 * ============================================================================
 */

void BusSniffer::setup()
{
#if UNNI_BLE_ENABLED
  sensirion_ble_setup();
#endif
#if UNNI_BLE_HISTORY_ENABLED
  sensirion_history_setup();
#endif

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
      (1ULL << PIN_RTRH0) |
      (1ULL << PIN_RTRH1) |
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
  gpio_set_intr_type(PIN_RTRH3, GPIO_INTR_ANYEDGE);


  last_value =
      read_gpio_state();


  capture_initial_value =
      last_value;

#if RTRH_DEBUG_CAPTURE
  rtrh_last_value = read_rtrh_state();
#endif
  rtrh_last_state = read_rtrh_state();
  rtrh_pin_level[0] = gpio_get_level(PIN_RTRH0);
  rtrh_pin_level[1] = gpio_get_level(PIN_RTRH1);
  rtrh_pin_level[2] = gpio_get_level(PIN_RTRH3);

  // All five XIAO signal pins stay ordinary high-impedance digital inputs.


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
      PIN_RTRH0, PIN_RTRH1, PIN_RTRH3};

  for (uint8_t i = 0; i < 3; i++) {
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
      "(I2C 0x62, SCL GPIO7/D5, SDA GPIO6/D4)"
  );


#if RTRH_DEBUG_CAPTURE
  ESP_LOGD(
      TAG,
      "Raw capture /capture; RT/RH debug /rt_rh_capture.csv + /rt_rh_timing.csv"
  );
#else
  ESP_LOGD(
      TAG,
      "RT/RH time-phase decoder (GPIO3/D1 + GPIO5/D3 + GPIO4/D2); debug capture disabled"
  );
#endif
}


void BusSniffer::maybe_publish_ha_()
{
  const uint32_t now = millis();
  if (this->last_ha_publish_ms_ != 0 &&
      static_cast<uint32_t>(now - this->last_ha_publish_ms_) < this->ha_publish_interval_ms_)
    return;

  bool published = false;
  if (this->ha_have_co2_ && this->co2_sensor_ != nullptr &&
      this->ha_initial_co2_published_) {
    this->co2_sensor_->publish_state(this->ha_co2_);
    published = true;
  }
  if (this->ha_have_temperature_ && this->rt_temperature_sensor_ != nullptr &&
      this->ha_initial_temperature_published_) {
    this->rt_temperature_sensor_->publish_state(this->ha_temperature_);
    published = true;
  }
  if (this->ha_have_humidity_ && this->rh_humidity_sensor_ != nullptr &&
      this->ha_initial_humidity_published_) {
    this->rh_humidity_sensor_->publish_state(this->ha_humidity_);
    published = true;
  }

  if (published)
    this->last_ha_publish_ms_ = now;
}


/*
 * ============================================================================
 * Loop
 * ============================================================================
 */

void BusSniffer::loop()
{
#if UNNI_BLE_HISTORY_ENABLED
  sensirion_history_loop();
#endif
  this->maybe_publish_ha_();

  // Freeze a complete RT/RH measurement after 15 s without any RT/RH edge.
  if (rtrh_collecting) {
    const uint32_t now =
        static_cast<uint32_t>(esp_timer_get_time());
    const uint32_t last_any =
        rtrh_last_any_us;

    if (last_any != 0 &&
        static_cast<uint32_t>(now - last_any) >
            RTRH_MEASUREMENT_QUIET_US) {
      gpio_intr_disable(PIN_RTRH0);
      gpio_intr_disable(PIN_RTRH1);
      gpio_intr_disable(PIN_RTRH3);

      const uint32_t now2 =
          static_cast<uint32_t>(esp_timer_get_time());
      const uint32_t last_any2 =
          rtrh_last_any_us;

      if (rtrh_collecting &&
          last_any2 != 0 &&
          static_cast<uint32_t>(now2 - last_any2) >
              RTRH_MEASUREMENT_QUIET_US) {
        finalize_rtrh_measurement();
      }

      rtrh_last_state = read_rtrh_state();

      rtrh_pin_level[0] =
          gpio_get_level(PIN_RTRH0);
      rtrh_pin_level[1] =
          gpio_get_level(PIN_RTRH1);
      rtrh_pin_level[2] =
          gpio_get_level(PIN_RTRH3);

      gpio_intr_enable(PIN_RTRH0);
      gpio_intr_enable(PIN_RTRH1);
      gpio_intr_enable(PIN_RTRH3);
    }
  }

  static uint32_t last_rtrh_sequence = 0;

  if (rtrh_snapshot_ready &&
      rtrh_snapshot.sequence !=
          last_rtrh_sequence) {
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

    const float ref_duration_ms =
        static_cast<float>(s.ref.period_sum) /
        1000.0f;

    const float rt_duration_ms =
        static_cast<float>(s.rt.period_sum) /
        1000.0f;

    const float rh_duration_ms =
        static_cast<float>(
            s.rh_timing.period_sum) /
        1000.0f;

    const float rh_state_us =
        rtrh_rh_state_period_median(
            s.rh_state);

    const float rt_period_us =
        s.rt_temp_count
            ? static_cast<float>(s.rt_temp_period_sum) /
                  static_cast<float>(s.rt_temp_count)
            : 0.0f;

    const float rt_ratio =
        ref_period_us > 0.0f
            ? rt_period_us / ref_period_us
            : NAN;

    const float rh_ratio =
        ref_period_us > 0.0f && rh_state_us > 0.0f
            ? rh_state_us / ref_period_us
            : NAN;

    measurement_quality::Inputs quality_inputs;
    quality_inputs.ref_period_us = ref_period_us;
    quality_inputs.ref_duration_ms = ref_duration_ms;
    quality_inputs.rt_period_us = rt_period_us;
    quality_inputs.rt_duration_ms = rt_duration_ms;
    quality_inputs.rt_count = s.rt_temp_count;
    quality_inputs.rh_duration_ms = rh_duration_ms;
    quality_inputs.rh_state_us = rh_state_us;
    quality_inputs.rh_state_samples = s.rh_state.sample_count;
    quality_inputs.rh_state_seen = s.rh_state.seen;
    quality_inputs.rh_ratio = rh_ratio;

    const auto quality =
        measurement_quality::evaluate(quality_inputs);
    const bool measurement_valid = quality.valid;
    const float quality_percent = quality.score_percent;

    ESP_LOGI(
        TAG,
        "RT/RH measurement %lu quality: "
        "REF %.3f us / %.3f ms / %u, "
        "RT %.3f us / %.3f ms / %u, "
        "RH %.3f ms / state %.3f us (%u/%lu) -> %s",
        static_cast<unsigned long>(s.sequence),
        ref_period_us,
        ref_duration_ms,
        static_cast<unsigned>(s.ref.count),
        rt_phase_period_us,
        rt_duration_ms,
        static_cast<unsigned>(s.rt.count),
        rh_duration_ms,
        rh_state_us,
        static_cast<unsigned>(
            s.rh_state.sample_count),
        static_cast<unsigned long>(
            s.rh_state.seen),
        measurement_valid ? "VALID" : "REJECT");

    if (!measurement_valid) {
      rtrh_derived.sequence = s.sequence;
      rtrh_derived.valid = false;
      rtrh_derived.rt_ratio = rt_ratio;
      rtrh_derived.rh_ratio = rh_ratio;
      rtrh_derived.temperature_c =
          std::isfinite(rt_ratio)
              ? calibration::temperature_from_ratio(rt_ratio)
              : NAN;
      rtrh_derived.humidity_percent = NAN;
      rtrh_derived.quality_percent = quality_percent;
      rtrh_derived.reject_reason = quality.reason;
      rtrh_derived.thermal_transient = false;
      rtrh_derived.temperature_extrapolation = true;
      rtrh_derived.humidity_extrapolation = true;
      rtrh_derived.calibration_extrapolation = true;

      if (this->measurement_quality_sensor_ != nullptr)
        this->measurement_quality_sensor_->publish_state(quality_percent);
      if (this->ref_period_sensor_ != nullptr)
        this->ref_period_sensor_->publish_state(ref_period_us);
      if (this->rt_period_sensor_ != nullptr && rt_period_us > 0.0f)
        this->rt_period_sensor_->publish_state(rt_period_us);
      if (this->rh_state_period_sensor_ != nullptr && rh_state_us > 0.0f)
        this->rh_state_period_sensor_->publish_state(rh_state_us);
      if (this->rt_ratio_sensor_ != nullptr && std::isfinite(rt_ratio))
        this->rt_ratio_sensor_->publish_state(rt_ratio);
      if (this->rh_ratio_sensor_ != nullptr && std::isfinite(rh_ratio))
        this->rh_ratio_sensor_->publish_state(rh_ratio);
      if (this->temperature_extrapolation_sensor_ != nullptr)
        this->temperature_extrapolation_sensor_->publish_state(true);
      if (this->humidity_extrapolation_sensor_ != nullptr)
        this->humidity_extrapolation_sensor_->publish_state(true);
      if (this->calibration_extrapolation_sensor_ != nullptr)
        this->calibration_extrapolation_sensor_->publish_state(true);

      ESP_LOGW(
          TAG,
          "RT/RH measurement %lu values not published: "
          "REJECT=%s (quality %.0f%%)",
          static_cast<unsigned long>(s.sequence),
          measurement_quality::reject_reason_to_string(quality.reason),
          quality_percent);
    } else {
      // Temperature and humidity calibration are deliberately isolated from
      // the edge decoder so future fits can be changed without touching capture.
      const float temperature_c =
          calibration::temperature_from_ratio(rt_ratio);
      const float rh_log = calibration::log_rh_ratio(rh_ratio);
      const float rh_percent =
          calibration::humidity_from_ratio_temperature(
              rh_ratio,
              temperature_c);

      const uint32_t now_ms = millis();
      float temperature_rate_c_per_min = 0.0f;
      if (this->have_last_valid_temperature_) {
        const uint32_t dt_ms = now_ms - this->last_valid_temperature_ms_;
        if (dt_ms > 0) {
          temperature_rate_c_per_min =
              fabsf(temperature_c - this->last_valid_temperature_c_) *
              60000.0f / static_cast<float>(dt_ms);
        }
      }

      if (!this->thermal_transient_active_) {
        if (this->have_last_valid_temperature_ &&
            temperature_rate_c_per_min >=
                this->thermal_transient_on_rate_c_per_min_) {
          this->thermal_transient_active_ = true;
        }
      } else if (temperature_rate_c_per_min <=
                 this->thermal_transient_off_rate_c_per_min_) {
        this->thermal_transient_active_ = false;
      }
      const bool thermal_transient = this->thermal_transient_active_;

      const bool temperature_extrapolation =
          temperature_c < calibration::CAL_TEMP_MIN_C ||
          temperature_c > calibration::CAL_TEMP_MAX_C;
      const bool humidity_extrapolation =
          rh_ratio < calibration::CAL_RH_RATIO_MIN ||
          rh_ratio > calibration::CAL_RH_RATIO_MAX;
      const bool calibration_extrapolation =
          temperature_extrapolation || humidity_extrapolation;

      this->last_valid_temperature_c_ = temperature_c;
      this->last_valid_temperature_ms_ = now_ms;
      this->have_last_valid_temperature_ = true;

      rtrh_derived.sequence = s.sequence;
      rtrh_derived.valid = true;
      rtrh_derived.rt_ratio = rt_ratio;
      rtrh_derived.rh_ratio = rh_ratio;
      rtrh_derived.temperature_c = temperature_c;
      rtrh_derived.humidity_percent = rh_percent;
      rtrh_derived.quality_percent = quality_percent;
      rtrh_derived.reject_reason =
          measurement_quality::RejectReason::NONE;
      rtrh_derived.thermal_transient = thermal_transient;
      rtrh_derived.temperature_extrapolation =
          temperature_extrapolation;
      rtrh_derived.humidity_extrapolation =
          humidity_extrapolation;
      rtrh_derived.calibration_extrapolation =
          calibration_extrapolation;

      ESP_LOGI(
          TAG,
          "RT/RH measurement %lu RT: "
          "%.3f / REF %.3f us = %.6f -> %.2f C",
          static_cast<unsigned long>(s.sequence),
          rt_period_us,
          ref_period_us,
          rt_ratio,
          temperature_c);

      ESP_LOGI(
          TAG,
          "RT/RH measurement %lu RH: "
          "state %.3f / REF %.3f us = %.6f -> %.1f %%",
          static_cast<unsigned long>(s.sequence),
          rh_state_us,
          ref_period_us,
          rh_ratio,
          rh_percent);

      if (this->debug_metrics_) {
        ESP_LOGI(
            TAG,
            "RT/RH diagnostics %lu: quality %.0f%%, ln(RH/REF)=%.6f, "
            "|dT/dt|=%.2f C/min, thermal_transient=%s, "
            "T_extrap=%s, RH_extrap=%s",
            static_cast<unsigned long>(s.sequence),
            quality_percent,
            rh_log,
            temperature_rate_c_per_min,
            thermal_transient ? "YES" : "no",
            temperature_extrapolation ? "YES" : "no",
            humidity_extrapolation ? "YES" : "no");
      }

      if (this->ref_period_sensor_ != nullptr)
        this->ref_period_sensor_->publish_state(ref_period_us);
      if (this->rt_period_sensor_ != nullptr)
        this->rt_period_sensor_->publish_state(rt_period_us);
      if (this->rh_state_period_sensor_ != nullptr)
        this->rh_state_period_sensor_->publish_state(rh_state_us);
      if (this->rt_ratio_sensor_ != nullptr)
        this->rt_ratio_sensor_->publish_state(rt_ratio);
      if (this->rh_ratio_sensor_ != nullptr)
        this->rh_ratio_sensor_->publish_state(rh_ratio);
      if (this->rh_log_sensor_ != nullptr)
        this->rh_log_sensor_->publish_state(rh_log);
      if (this->measurement_quality_sensor_ != nullptr)
        this->measurement_quality_sensor_->publish_state(quality_percent);
      if (this->thermal_transient_sensor_ != nullptr)
        this->thermal_transient_sensor_->publish_state(thermal_transient);
      if (this->temperature_extrapolation_sensor_ != nullptr)
        this->temperature_extrapolation_sensor_->publish_state(
            temperature_extrapolation);
      if (this->humidity_extrapolation_sensor_ != nullptr)
        this->humidity_extrapolation_sensor_->publish_state(
            humidity_extrapolation);
      if (this->calibration_extrapolation_sensor_ != nullptr)
        this->calibration_extrapolation_sensor_->publish_state(
            calibration_extrapolation);

#if UNNI_BLE_LIVE_ENABLED
      sensirion_ble_set_temperature_humidity(
          temperature_c,
          rh_percent);
#endif

      this->ha_temperature_ = temperature_c;
      this->ha_humidity_ = rh_percent;
      this->ha_have_temperature_ = true;
      this->ha_have_humidity_ = true;

      // First valid values are published immediately; subsequent API traffic
      // is throttled by ha_publish_interval (30 s default).
      if (!this->ha_initial_temperature_published_ &&
          this->rt_temperature_sensor_ != nullptr) {
        this->rt_temperature_sensor_->publish_state(temperature_c);
        this->ha_initial_temperature_published_ = true;
        this->last_ha_publish_ms_ = millis();
      }
      if (!this->ha_initial_humidity_published_ &&
          this->rh_humidity_sensor_ != nullptr) {
        this->rh_humidity_sensor_->publish_state(rh_percent);
        this->ha_initial_humidity_published_ = true;
        this->last_ha_publish_ms_ = millis();
      }
    }
  }

#if RTRH_DEBUG_CAPTURE
  if (rtrh_capture_ready) {
    static uint32_t last_reported_sequence =
        UINT32_MAX;

    if (last_reported_sequence !=
        rtrh_sequence) {
      last_reported_sequence =
          rtrh_sequence;

      ESP_LOGI(
          TAG,
          "RT/RH edge capture ready: %u events, sequence %lu%s",
          rtrh_sample_count,
          static_cast<unsigned long>(
              rtrh_sequence),
          rtrh_overflow ?
              " OVERFLOW" : "");
    }
  }

  if (rtrh_capturing &&
      !rtrh_capture_ready) {
    const uint32_t now =
        static_cast<uint32_t>(
            esp_timer_get_time());

    if (static_cast<uint32_t>(
            now - rtrh_start_us) >=
        RTRH_CAPTURE_US) {
      rtrh_capturing = false;
      rtrh_capture_ready = true;
      rtrh_sequence++;

      ESP_LOGI(
          TAG,
          "RT/RH edge capture ready: %u events, sequence %lu%s",
          rtrh_sample_count,
          static_cast<unsigned long>(
              rtrh_sequence),
          rtrh_overflow ?
              " OVERFLOW" : "");
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

        // Keep BLE live advertisement fresh even when the ppm value did not
        // change, while HA publication below remains change-only.
#if UNNI_BLE_LIVE_ENABLED
        sensirion_ble_set_co2(ppm);
#endif
        this->ha_co2_ = static_cast<float>(ppm);
        this->ha_have_co2_ = true;


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


          this->ha_co2_ = static_cast<float>(ppm);
          this->ha_have_co2_ = true;

          if (!this->ha_initial_co2_published_ &&
              this->co2_sensor_ != nullptr) {
            this->co2_sensor_->publish_state(this->ha_co2_);
            this->ha_initial_co2_published_ = true;
            this->last_ha_publish_ms_ = millis();
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
