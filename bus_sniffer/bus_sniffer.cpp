#include "bus_sniffer.h"

#include "esphome/core/log.h"
#include "esphome/components/web_server_base/web_server_base.h"

#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include <climits>
#include <cstdio>
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
 * RT/RH ADC probe diagnostics
 * ============================================================================
 *
 * GPIO10 = ADC1
 * GPIO11..13 = ADC2 on ESP32-S2.
 *
 * We deliberately use the ESP-IDF oneshot driver directly here instead of the
 * ESPHome ADC component so that ADC2 can at least be attempted while Wi-Fi is
 * active. Failed/contended reads are counted and exposed as a diagnostic.
 *
 * This is discovery instrumentation, not the final temperature/RH conversion.
 */

static constexpr uint8_t PROBE_COUNT = 4;
static constexpr int PROBE_GPIOS[PROBE_COUNT] = {10, 11, 12, 13};

// Target: about 2 kSamples/s per channel. Loop jitter is intentional/useful
// while we are trying to discover AC excitation on the RH network.
static constexpr uint32_t PROBE_SAMPLE_INTERVAL_US = 500;
static constexpr uint32_t PROBE_PUBLISH_INTERVAL_US = 2000000;

struct ProbeState {
  adc_unit_t unit{ADC_UNIT_1};
  adc_channel_t channel{ADC_CHANNEL_0};
  bool configured{false};

  uint32_t samples{0};
  uint64_t sum{0};
  int min_raw{INT_MAX};
  int max_raw{INT_MIN};
};

static ProbeState probe_states[PROBE_COUNT];
static adc_oneshot_unit_handle_t adc1_handle = nullptr;
static adc_oneshot_unit_handle_t adc2_handle = nullptr;
static uint32_t next_probe_sample_us = 0;
static uint32_t next_probe_publish_us = 0;


/*
 * ============================================================================
 * High-speed analog trigger capture
 * ============================================================================
 *
 * A dedicated FreeRTOS task samples GPIO10..13 at roughly 1 kHz per channel.
 * When any valid ADC reading rises above ANALOG_TRIGGER_RAW, the ring buffer is
 * frozen after a post-trigger interval.  The finished waveform remains
 * available as CSV at /analog_capture.csv while acquisition immediately
 * rearms for the next measurement burst.
 */

static constexpr uint32_t ANALOG_SAMPLE_PERIOD_US = 1000;
static constexpr uint16_t ANALOG_RING_SAMPLES = 4096;
static constexpr uint16_t ANALOG_PRETRIGGER_SAMPLES = 500;
static constexpr uint16_t ANALOG_POSTTRIGGER_SAMPLES = 3000;
static constexpr int ANALOG_TRIGGER_RAW = 100;
static constexpr uint16_t ANALOG_REARM_QUIET_SAMPLES = 250;

struct AnalogSample {
  uint32_t t;
  uint16_t raw[PROBE_COUNT];
  uint8_t valid_mask;
};

static AnalogSample analog_ring[ANALOG_RING_SAMPLES];
static AnalogSample analog_capture[ANALOG_PRETRIGGER_SAMPLES + 1 + ANALOG_POSTTRIGGER_SAMPLES];

static volatile uint16_t analog_write_index = 0;
static volatile uint32_t analog_total_samples = 0;
static volatile bool analog_triggered = false;
static volatile uint16_t analog_trigger_index = 0;
static volatile uint16_t analog_post_remaining = 0;
static volatile bool analog_armed = true;
static volatile uint16_t analog_quiet_samples = 0;

static uint16_t analog_capture_count = 0;
static uint32_t analog_capture_sequence = 0;
static SemaphoreHandle_t analog_capture_mutex = nullptr;
static TaskHandle_t analog_capture_task_handle = nullptr;
static esp_timer_handle_t analog_sample_timer = nullptr;

static adc_oneshot_unit_handle_t probe_handle_for_unit(adc_unit_t unit);


static void analog_snapshot_capture()
{
  const uint16_t wanted =
      ANALOG_PRETRIGGER_SAMPLES + 1 + ANALOG_POSTTRIGGER_SAMPLES;

  uint16_t available = ANALOG_RING_SAMPLES;
  if (analog_total_samples < ANALOG_RING_SAMPLES)
    available = static_cast<uint16_t>(analog_total_samples);

  uint16_t count = wanted;
  if (count > available)
    count = available;

  // The newest sample is the one immediately before analog_write_index.
  uint16_t start = static_cast<uint16_t>(
      (analog_write_index + ANALOG_RING_SAMPLES - count) % ANALOG_RING_SAMPLES);

  if (analog_capture_mutex != nullptr &&
      xSemaphoreTake(analog_capture_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    for (uint16_t i = 0; i < count; i++) {
      analog_capture[i] = analog_ring[(start + i) % ANALOG_RING_SAMPLES];
    }
    analog_capture_count = count;
    analog_capture_sequence++;
    xSemaphoreGive(analog_capture_mutex);
  }
}


static void analog_capture_task(void *arg)
{
  while (true) {
    // Woken by an esp_timer at 1 kHz, independent of the FreeRTOS tick rate.
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    AnalogSample sample{};
    sample.t = static_cast<uint32_t>(esp_timer_get_time());

    bool above_trigger = false;

    for (uint8_t i = 0; i < PROBE_COUNT; i++) {
      ProbeState &state = probe_states[i];
      if (!state.configured)
        continue;

      adc_oneshot_unit_handle_t handle = probe_handle_for_unit(state.unit);
      if (handle == nullptr)
        continue;

      int raw = 0;
      if (adc_oneshot_read(handle, state.channel, &raw) != ESP_OK)
        continue;

      if (raw < 0)
        raw = 0;
      if (raw > 65535)
        raw = 65535;

      sample.raw[i] = static_cast<uint16_t>(raw);
      sample.valid_mask |= static_cast<uint8_t>(1U << i);

      if (raw > ANALOG_TRIGGER_RAW)
        above_trigger = true;
    }

    const uint16_t this_index = analog_write_index;
    analog_ring[this_index] = sample;
    analog_write_index = static_cast<uint16_t>((this_index + 1) % ANALOG_RING_SAMPLES);
    analog_total_samples++;

    if (analog_armed && !analog_triggered && above_trigger) {
      analog_triggered = true;
      analog_trigger_index = this_index;
      analog_post_remaining = ANALOG_POSTTRIGGER_SAMPLES;
      analog_armed = false;
      analog_quiet_samples = 0;
      ESP_LOGI(TAG, "Analog trigger detected");
    } else if (analog_triggered) {
      if (analog_post_remaining > 0)
        analog_post_remaining--;

      if (analog_post_remaining == 0) {
        analog_triggered = false;
        analog_snapshot_capture();
        ESP_LOGI(
            TAG,
            "Analog capture ready: %u samples, sequence %lu",
            analog_capture_count,
            static_cast<unsigned long>(analog_capture_sequence));
      }
    } else if (!analog_armed) {
      if (above_trigger) {
        analog_quiet_samples = 0;
      } else if (++analog_quiet_samples >= ANALOG_REARM_QUIET_SAMPLES) {
        analog_armed = true;
        analog_quiet_samples = 0;
        ESP_LOGD(TAG, "Analog trigger rearmed");
      }
    }

  }
}


static void analog_sample_timer_callback(void *arg)
{
  if (analog_capture_task_handle != nullptr)
    xTaskNotifyGive(analog_capture_task_handle);
}


static adc_oneshot_unit_handle_t probe_handle_for_unit(adc_unit_t unit)
{
  return unit == ADC_UNIT_1 ? adc1_handle : adc2_handle;
}


static void reset_probe_window(ProbeState &state)
{
  state.samples = 0;
  state.sum = 0;
  state.min_raw = INT_MAX;
  state.max_raw = INT_MIN;
}


static void setup_adc_probes()
{
  adc_oneshot_unit_init_cfg_t unit1_cfg = {};
  unit1_cfg.unit_id = ADC_UNIT_1;

  esp_err_t err = adc_oneshot_new_unit(&unit1_cfg, &adc1_handle);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "ADC1 init failed: %s", esp_err_to_name(err));
    adc1_handle = nullptr;
  }

  adc_oneshot_unit_init_cfg_t unit2_cfg = {};
  unit2_cfg.unit_id = ADC_UNIT_2;

  err = adc_oneshot_new_unit(&unit2_cfg, &adc2_handle);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "ADC2 init failed: %s", esp_err_to_name(err));
    adc2_handle = nullptr;
  }

  adc_oneshot_chan_cfg_t chan_cfg = {};
  chan_cfg.bitwidth = ADC_BITWIDTH_DEFAULT;
  chan_cfg.atten = ADC_ATTEN_DB_12;

  for (uint8_t i = 0; i < PROBE_COUNT; i++) {
    ProbeState &state = probe_states[i];
    reset_probe_window(state);

    err = adc_oneshot_io_to_channel(
        PROBE_GPIOS[i],
        &state.unit,
        &state.channel);

    if (err != ESP_OK) {
      ESP_LOGW(
          TAG,
          "GPIO%d is not a usable ADC input: %s",
          PROBE_GPIOS[i],
          esp_err_to_name(err));
      continue;
    }

    adc_oneshot_unit_handle_t handle =
        probe_handle_for_unit(state.unit);

    if (handle == nullptr) {
      ESP_LOGW(
          TAG,
          "GPIO%d ADC unit unavailable",
          PROBE_GPIOS[i]);
      continue;
    }

    err = adc_oneshot_config_channel(
        handle,
        state.channel,
        &chan_cfg);

    if (err != ESP_OK) {
      ESP_LOGW(
          TAG,
          "GPIO%d ADC channel config failed: %s",
          PROBE_GPIOS[i],
          esp_err_to_name(err));
      continue;
    }

    state.configured = true;

    ESP_LOGI(
        TAG,
        "Probe GPIO%d -> ADC%d channel %d",
        PROBE_GPIOS[i],
        static_cast<int>(state.unit) + 1,
        static_cast<int>(state.channel));
  }

  const uint32_t now = static_cast<uint32_t>(esp_timer_get_time());
  next_probe_sample_us = now;
  next_probe_publish_us = now + PROBE_PUBLISH_INTERVAL_US;
}


void BusSniffer::sample_adc_probes_()
{
  const uint32_t now = static_cast<uint32_t>(esp_timer_get_time());

  if (static_cast<int32_t>(now - next_probe_sample_us) < 0)
    return;

  next_probe_sample_us = now + PROBE_SAMPLE_INTERVAL_US;

  for (uint8_t i = 0; i < PROBE_COUNT; i++) {
    ProbeState &state = probe_states[i];

    if (!state.configured)
      continue;

    adc_oneshot_unit_handle_t handle =
        probe_handle_for_unit(state.unit);

    if (handle == nullptr)
      continue;

    int raw = 0;
    const esp_err_t err = adc_oneshot_read(
        handle,
        state.channel,
        &raw);

    if (err != ESP_OK) {
      this->adc_read_errors_++;
      continue;
    }

    state.samples++;
    state.sum += static_cast<uint32_t>(raw);

    if (raw < state.min_raw)
      state.min_raw = raw;

    if (raw > state.max_raw)
      state.max_raw = raw;
  }
}


void BusSniffer::publish_adc_probes_()
{
  const uint32_t now = static_cast<uint32_t>(esp_timer_get_time());

  if (static_cast<int32_t>(now - next_probe_publish_us) < 0)
    return;

  next_probe_publish_us = now + PROBE_PUBLISH_INTERVAL_US;

  char line[512];
  int pos = snprintf(line, sizeof(line), "ADC probes raw:");

  for (uint8_t i = 0; i < PROBE_COUNT; i++) {
    ProbeState &state = probe_states[i];

    if (!state.configured) {
      pos += snprintf(
          line + pos,
          sizeof(line) - pos,
          " G%d=unavailable",
          PROBE_GPIOS[i]);
      continue;
    }

    if (state.samples == 0) {
      pos += snprintf(
          line + pos,
          sizeof(line) - pos,
          " G%d=no-data",
          PROBE_GPIOS[i]);
      continue;
    }

    const float avg =
        static_cast<float>(state.sum) /
        static_cast<float>(state.samples);

    pos += snprintf(
        line + pos,
        sizeof(line) - pos,
        " G%d avg=%.0f min=%d max=%d n=%lu",
        PROBE_GPIOS[i],
        avg,
        state.min_raw,
        state.max_raw,
        static_cast<unsigned long>(state.samples));

    if (this->probe_sensors_[i] != nullptr)
      this->probe_sensors_[i]->publish_state(avg);
  }

  ESP_LOGI(TAG, "%s", line);

  if (this->adc_read_errors_sensor_ != nullptr)
    this->adc_read_errors_sensor_->publish_state(this->adc_read_errors_);

  if (this->adc_read_errors_ != 0) {
    ESP_LOGD(
        TAG,
        "ADC read errors/timeouts so far: %lu",
        static_cast<unsigned long>(this->adc_read_errors_));
  }

  for (uint8_t i = 0; i < PROBE_COUNT; i++)
    reset_probe_window(probe_states[i]);
}



/*
 * ============================================================================
 * HTTP /analog_capture.csv
 * ============================================================================
 */

class AnalogCaptureHandler : public web_server_idf::AsyncWebHandler {
 public:
  bool canHandle(web_server_idf::AsyncWebServerRequest *request) const override
  {
    if (request->method() != HTTP_GET)
      return false;

    char url[web_server_idf::AsyncWebServerRequest::URL_BUF_SIZE];
    return request->url_to(url) == "/analog_capture.csv";
  }

  void handleRequest(web_server_idf::AsyncWebServerRequest *request) override
  {
    if (analog_capture_mutex == nullptr ||
        xSemaphoreTake(analog_capture_mutex, pdMS_TO_TICKS(200)) != pdTRUE) {
      request->send(503, "text/plain", "capture busy");
      return;
    }

    const uint16_t count = analog_capture_count;
    const uint32_t sequence = analog_capture_sequence;

    if (count == 0) {
      xSemaphoreGive(analog_capture_mutex);
      request->send(204, "text/plain", nullptr);
      return;
    }

    std::string output;
    output.reserve(static_cast<size_t>(count) * 42 + 128);
    output += "sequence,t_us,gpio10,gpio11,gpio12,gpio13,valid_mask\n";

    const uint32_t base = analog_capture[0].t;
    char line[128];

    for (uint16_t i = 0; i < count; i++) {
      const AnalogSample &sample = analog_capture[i];
      const uint32_t rel = static_cast<uint32_t>(sample.t - base);

      const int n = snprintf(
          line,
          sizeof(line),
          "%lu,%lu,%u,%u,%u,%u,0x%02X\n",
          static_cast<unsigned long>(sequence),
          static_cast<unsigned long>(rel),
          sample.raw[0],
          sample.raw[1],
          sample.raw[2],
          sample.raw[3],
          sample.valid_mask);

      if (n > 0)
        output.append(line, static_cast<size_t>(n));
    }

    xSemaphoreGive(analog_capture_mutex);

    auto *response = request->beginResponse(200, "text/csv", output);
    response->addHeader(
        "Content-Disposition",
        "attachment; filename=\"analog_capture.csv\"");
    request->send(response);
  }
};

static AnalogCaptureHandler analog_capture_handler;


/*
 * ============================================================================
 * Setup
 * ============================================================================
 */

void BusSniffer::setup()
{
  last_capture_mutex =
      xSemaphoreCreateMutex();


  setup_adc_probes();

  analog_capture_mutex = xSemaphoreCreateMutex();

  if (analog_capture_mutex == nullptr) {
    ESP_LOGE(TAG, "Failed to create analog capture mutex");
  } else {
    const BaseType_t task_result = xTaskCreate(
        analog_capture_task,
        "analog_capture",
        4096,
        nullptr,
        1,
        &analog_capture_task_handle);

    if (task_result != pdPASS) {
      analog_capture_task_handle = nullptr;
      ESP_LOGE(TAG, "Failed to start analog capture task");
    } else {
      esp_timer_create_args_t timer_args = {};
      timer_args.callback = &analog_sample_timer_callback;
      timer_args.arg = nullptr;
      timer_args.dispatch_method = ESP_TIMER_TASK;
      timer_args.name = "analog_sample";

      esp_err_t timer_err = esp_timer_create(&timer_args, &analog_sample_timer);
      if (timer_err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create analog sample timer: %s", esp_err_to_name(timer_err));
      } else {
        timer_err = esp_timer_start_periodic(analog_sample_timer, ANALOG_SAMPLE_PERIOD_US);
        if (timer_err != ESP_OK) {
          ESP_LOGE(TAG, "Failed to start analog sample timer: %s", esp_err_to_name(timer_err));
        } else {
          ESP_LOGI(
              TAG,
              "Analog trigger capture: GPIO10..13 @ 1 kHz, trigger raw>%d",
              ANALOG_TRIGGER_RAW);
        }
      }
    }
  }


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
      (1ULL << PIN_OTHER);


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


  last_value =
      read_gpio_state();


  capture_initial_value =
      last_value;


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
        &analog_capture_handler);

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


  if (this->adc_read_errors_sensor_ != nullptr)
    this->adc_read_errors_sensor_->publish_state(0);


  ESP_LOGI(
      TAG,
      "Passive CO2 sniffer ready "
      "(I2C 0x62, SCL GPIO40, SDA GPIO39)"
  );


  ESP_LOGD(
      TAG,
      "Raw captures available at /capture; analog at /analog_capture.csv"
  );
}


/*
 * ============================================================================
 * Loop
 * ============================================================================
 */

void BusSniffer::loop()
{
  // Independent RT/RH discovery sampling must keep running even while there
  // is no CO2 bus activity.
  this->sample_adc_probes_();
  this->publish_adc_probes_();

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
