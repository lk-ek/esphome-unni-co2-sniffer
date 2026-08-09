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
 * 76-77 us. To keep CPU/ADC2 load very small, one ADC phase is sampled per
 * period, rotating through four offsets:
 *
 *   phase 0: +8 us
 *   phase 1: +25 us
 *   phase 2: +45 us
 *   phase 3: +62 us
 *
 * 32 periods -> 8 observations of each phase, then acquisition stops.
 */

static constexpr uint8_t RTRH_ADC_PHASES = 4;
static constexpr uint8_t RTRH_ADC_CHANNELS = 4;
static constexpr uint8_t RTRH_ADC_MODES = 2;
static constexpr uint8_t RTRH_ADC_REPEATS = 4;
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

// Empirical calibration from the 2026-08-09 heat/cool series:
// 11 display-reference points, 27.4..38.8 degC.
// Linear fit: T[degC] = intercept + slope * LONG_period_us
static constexpr float RTRH_TEMP_SLOPE = -0.49395309f;
static constexpr float RTRH_TEMP_INTERCEPT = 96.61457803f;
static constexpr uint16_t RTRH_ADC_STORED_SAMPLES = 32;
static constexpr uint8_t RTRH_ADC_SKIP_EDGES = 3;
static constexpr uint16_t RTRH_ADC_OFFSETS_US[RTRH_ADC_PHASES] = {
    8, 25, 45, 62
};
static constexpr int RTRH_ADC_GPIOS[RTRH_ADC_CHANNELS] = {
    10, 11, 12, 13
};

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
  rtrh_adc_active = false;
  rtrh_adc_ready = false;
  rtrh_adc_pending = false;
  rtrh_adc_cycle = 0;
  rtrh_adc_missed_edges = 0;
  rtrh_adc_reference_edge_us = 0;
  rtrh_adc_edges_to_skip = 0;
  rtrh_adc_waiting_for_edge = false;
  rtrh_adc_target_index = -1;
  rtrh_adc_trigger_index = -1;
  rtrh_adc_sequence_start = false;
  rtrh_adc_trigger_period_us = 0;
  rtrh_last_fall_us[0] = 0;
  rtrh_last_fall_us[1] = 0;
  rtrh_last_fall_us[2] = 0;
  rtrh_last_fall_us[3] = 0;
}

static void clear_rtrh_adc_aggregates()
{
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
       */
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

      if (!rtrh_adc_pending || !rtrh_adc_active) {
        restore_rtrh_pin_digital(channel);
        continue;
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
   * Start one ADC sequence from the first observed GPIO10 falling edge.
   * The task then pre-arms each target channel BEFORE waiting for its own
   * timing reference edge.
   */
  if (!rtrh_adc_ready && !rtrh_adc_active &&
      pin_index == 0 && previous && !level) {
    clear_rtrh_adc_aggregates();
    rtrh_adc_active = true;
    rtrh_adc_cycle = 0;
    rtrh_adc_missed_edges = 0;
    rtrh_adc_sequence_start = true;

    if (rtrh_adc_task_handle != nullptr) {
      BaseType_t higher_priority_woken = pdFALSE;
      vTaskNotifyGiveFromISR(
          rtrh_adc_task_handle,
          &higher_priority_woken);
      if (higher_priority_woken)
        portYIELD_FROM_ISR();
    }
  }

  /*
   * Once a target ADC pad has been prepared, only the selected other pin is
   * used as timing reference. Falling edges are counted in hardware time.
   */
  if (previous && !level) {
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
        "rejected_late,read_errors\n";

    esp_err_t err =
        httpd_resp_send_chunk(req, HEADER, sizeof(HEADER) - 1);

    static char chunk[128];
    static char line[128];
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

          const int n = snprintf(
              line,
              sizeof(line),
              "%lu,%u,%s,%u,%d,%u,%u,%lu,%u,%u,%lu,%u,%u,%u\n",
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
              static_cast<unsigned>(agg.read_errors));

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

  setup_rtrh_adc();
  reset_rtrh_adc_capture();

  if (xTaskCreate(
          rtrh_adc_task,
          "rtrh_adc",
          3072,
          nullptr,
          18,
          &rtrh_adc_task_handle) != pdPASS) {
    ESP_LOGE(TAG, "Failed to create RT/RH ADC task");
    rtrh_adc_task_handle = nullptr;
  }


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
        &rtrh_adc_handler);

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
  static uint32_t last_logged_adc_sequence = 0;

  if (rtrh_adc_ready &&
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
          RTRH_TEMP_INTERCEPT +
          RTRH_TEMP_SLOPE * long_period_us;

      ESP_LOGI(
          TAG,
          "RT temperature: %.2f C from LONG period %.2f us (%lu observations)",
          temperature_c,
          long_period_us,
          static_cast<unsigned long>(long_period_count));

      if (this->rt_temperature_sensor_ != nullptr)
        this->rt_temperature_sensor_->publish_state(temperature_c);
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
