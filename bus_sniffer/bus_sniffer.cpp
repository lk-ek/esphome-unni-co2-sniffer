#include "bus_sniffer.h"

#include "esphome/core/log.h"
#include "esphome/components/web_server_base/web_server_base.h"

#include "driver/gpio.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include <cstring>
#include <string>

namespace esphome {
namespace bus_sniffer {

static const char *TAG = "bus_sniffer";


/*
 * ============================================================================
 * Pins
 * ============================================================================
 *
 * Aus den Captures eindeutig:
 *
 * GPIO40 = SCL
 * GPIO39 = SDA
 *
 * GPIO38 bleibt als dritter Logic-Analyzer-Kanal erhalten,
 * erzeugt aber KEINE Interrupts mehr.
 */

static constexpr gpio_num_t PIN_SCL   = GPIO_NUM_40;
static constexpr gpio_num_t PIN_SDA   = GPIO_NUM_39;
static constexpr gpio_num_t PIN_OTHER = GPIO_NUM_38;


/*
 * ============================================================================
 * Capture
 * ============================================================================
 */

static constexpr uint16_t MAX_SAMPLES = 4096;

// EC05 write -> Antwortpause ~2.13 ms.
// 5 ms reicht also bequem, um beide I2C-Transaktionen in
// einem Capture zu halten.
static constexpr uint32_t CAPTURE_TIMEOUT_US = 5000;


struct Sample {
  uint32_t t;
  uint8_t value;
};


static volatile Sample samples[MAX_SAMPLES];

static volatile uint16_t sample_count = 0;

static volatile uint32_t last_edge = 0;

static volatile uint8_t last_value = 0xff;

// Zustand unmittelbar VOR dem ersten Event.
// Wichtig für START-Erkennung.
static volatile uint8_t capture_initial_value = 0xff;

static volatile bool capturing = true;
static volatile bool capture_finished = false;
static volatile bool capture_overflow = false;


/*
 * ============================================================================
 * Letzter Raw-Capture für HTTP
 * ============================================================================
 *
 * Anders als vorher wird die Aufnahme NICHT angehalten, bis jemand
 * /capture herunterlädt.
 *
 * Nach jedem Frame:
 *
 *   1. dekodieren
 *   2. letzten Capture hier speichern
 *   3. sofort neuen Capture starten
 */

static std::string last_capture_data;

static SemaphoreHandle_t last_capture_mutex = nullptr;


/*
 * ============================================================================
 * GPIO helpers
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
 * Nur SCL und SDA erzeugen Interrupts.
 *
 * GPIO38 wird jeweils mitgesampelt, kann den ESP32 aber nicht mehr
 * mit Interrupts bombardieren.
 */

static void IRAM_ATTR gpio_isr(void *arg)
{
  if (!capturing)
    return;


  uint32_t now =
      (uint32_t) esp_timer_get_time();


  uint8_t value =
      read_gpio_state();


  if (value == last_value)
    return;


  /*
   * Beim ersten Event den Zustand VOR der Flanke merken.
   *
   * Damit können wir später z.B.
   *
   *     SDA 1 -> 0 bei SCL=1
   *
   * zuverlässig als I2C START erkennen.
   */
  if (sample_count == 0)
    capture_initial_value = last_value;


  last_value = value;
  last_edge = now;


  uint16_t index = sample_count;


  if (index < MAX_SAMPLES) {

    samples[index].t = now;
    samples[index].value = value;

    sample_count = index + 1;

  } else {

    capture_overflow = true;
    capturing = false;
    capture_finished = true;
  }
}


/*
 * ============================================================================
 * Sensirion CRC-8
 * ============================================================================
 *
 * Polynomial: 0x31
 * Init:       0xFF
 *
 * Kein final XOR.
 */

static uint8_t sensirion_crc(
    uint8_t byte0,
    uint8_t byte1)
{
  uint8_t crc = 0xff;

  uint8_t data[2] = {
      byte0,
      byte1
  };


  for (uint8_t n = 0; n < 2; n++) {

    crc ^= data[n];

    for (uint8_t bit = 0; bit < 8; bit++) {

      if (crc & 0x80)
        crc = (crc << 1) ^ 0x31;
      else
        crc <<= 1;
    }
  }

  return crc;
}


/*
 * ============================================================================
 * I2C decoder
 * ============================================================================
 */

struct I2CTransaction {
  uint8_t bytes[16];
  bool ack[16];

  uint8_t count;
};


static void clear_transaction(
    I2CTransaction &txn)
{
  txn.count = 0;

  memset(
      txn.bytes,
      0,
      sizeof(txn.bytes));

  memset(
      txn.ack,
      0,
      sizeof(txn.ack));
}


/*
 * ============================================================================
 * Einzelne I2C-Transaktion auswerten
 * ============================================================================
 */

void process_i2c_transaction(
    const I2CTransaction &txn,
    sensor::Sensor *co2_sensor)
{
  if (txn.count == 0)
    return;


  /*
   * --------------------------------------------------------------------------
   * WRITE an 0x62
   *
   * Capture zeigt:
   *
   *   C4 EC 05
   *
   * C4:
   *
   *   0x62 << 1 | WRITE
   */
  if (
      txn.count >= 3 &&
      txn.bytes[0] == 0xC4 &&
      txn.bytes[1] == 0xEC &&
      txn.bytes[2] == 0x05) {

    ESP_LOGV(
        TAG,
        "I2C: SCD4x read_measurement command");

    return;
  }


  /*
   * --------------------------------------------------------------------------
   * READ von 0x62
   *
   * Capture zeigt:
   *
   *   C5 CO2_MSB CO2_LSB CRC
   *
   * Danach beendet der originale Controller den Read.
   */
  if (
      txn.count >= 4 &&
      txn.bytes[0] == 0xC5) {

    uint8_t msb =
        txn.bytes[1];

    uint8_t lsb =
        txn.bytes[2];

    uint8_t received_crc =
        txn.bytes[3];


    uint8_t calculated_crc =
        sensirion_crc(
            msb,
            lsb);


    /*
     * CRC muss stimmen.
     *
     * Damit publizieren wir niemals einen durch verlorene
     * Flanken beschädigten ppm-Wert.
     */
    if (received_crc != calculated_crc) {

      ESP_LOGW(
          TAG,
          "CO2 CRC mismatch: "
          "data=%02X %02X "
          "received=%02X expected=%02X",
          msb,
          lsb,
          received_crc,
          calculated_crc);

      return;
    }


    uint16_t ppm =
        (static_cast<uint16_t>(msb) << 8) |
        lsb;


    ESP_LOGI(
        TAG,
        "CO2: %u ppm",
        ppm);


    if (co2_sensor != nullptr) {

      co2_sensor->publish_state(
          static_cast<float>(ppm));
    }

    return;
  }


  /*
   * Andere I2C-Transaktionen ignorieren.
   *
   * Bei VERBOSE kann man sie trotzdem sehen.
   */
  ESP_LOGV(
      TAG,
      "I2C transaction: %u bytes, first=0x%02X",
      txn.count,
      txn.bytes[0]);
}


/*
 * ============================================================================
 * Gesamten Capture als I2C dekodieren
 * ============================================================================
 */

static void decode_i2c_capture(
    const volatile Sample *data,
    uint16_t count,
    uint8_t initial_value,
    sensor::Sensor *co2_sensor)
{
  if (count == 0)
    return;


  bool active = false;

  uint8_t current_byte = 0;
  uint8_t bit_count = 0;

  I2CTransaction txn;
  clear_transaction(txn);


  uint8_t previous =
      initial_value;


  for (uint16_t i = 0;
       i < count;
       i++) {

    uint8_t current =
        data[i].value;


    bool prev_scl =
        scl_level(previous);

    bool cur_scl =
        scl_level(current);


    bool prev_sda =
        sda_level(previous);

    bool cur_sda =
        sda_level(current);


    /*
     * ------------------------------------------------------------------------
     * START
     *
     * SDA HIGH -> LOW während SCL HIGH
     * ------------------------------------------------------------------------
     */
    if (
        prev_sda &&
        !cur_sda &&
        cur_scl) {

      /*
       * Repeated START:
       *
       * Falls bereits eine Transaktion lief, deren vorhandene Bytes
       * zuerst abschließen.
       */
      if (active && txn.count != 0) {

        process_i2c_transaction(
            txn,
            co2_sensor);
      }


      clear_transaction(txn);

      current_byte = 0;
      bit_count = 0;

      active = true;

      previous = current;

      continue;
    }


    /*
     * ------------------------------------------------------------------------
     * STOP
     *
     * SDA LOW -> HIGH während SCL HIGH
     * ------------------------------------------------------------------------
     */
    if (
        active &&
        !prev_sda &&
        cur_sda &&
        cur_scl) {

      if (txn.count != 0) {

        process_i2c_transaction(
            txn,
            co2_sensor);
      }


      clear_transaction(txn);

      active = false;
      current_byte = 0;
      bit_count = 0;

      previous = current;

      continue;
    }


    /*
     * ------------------------------------------------------------------------
     * Daten werden auf steigender SCL-Flanke gesampelt.
     * ------------------------------------------------------------------------
     */
    if (
        active &&
        !prev_scl &&
        cur_scl) {

      bool bit =
          cur_sda;


      /*
       * 8 Datenbits
       */
      if (bit_count < 8) {

        current_byte <<= 1;

        if (bit)
          current_byte |= 1;

        bit_count++;

      } else {

        /*
         * 9. Clock = ACK/NACK.
         *
         * SDA LOW  = ACK
         * SDA HIGH = NACK
         */

        if (
            txn.count <
            sizeof(txn.bytes)) {

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


    previous = current;
  }


  /*
   * Falls Capture ohne sichtbaren STOP endet.
   */
  if (
      active &&
      txn.count != 0) {

    process_i2c_transaction(
        txn,
        co2_sensor);
  }
}


/*
 * ============================================================================
 * Raw Capture speichern
 * ============================================================================
 *
 * LA01:
 *
 *   4 Byte  "LA01"
 *   4 Byte  sample count
 *   1 Byte  flags
 *
 * pro Event:
 *
 *   4 Byte timestamp relativ zum ersten Event
 *   1 Byte GPIO state
 *
 * flags bit0 = overflow
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
      static_cast<size_t>(count) * 5);


  char *p =
      output.data();


  memcpy(
      p,
      "LA01",
      4);

  p += 4;


  uint32_t count32 =
      count;

  memcpy(
      p,
      &count32,
      sizeof(count32));

  p += 4;


  uint8_t flags = 0;

  if (overflow)
    flags |= 0x01;

  *p++ =
      static_cast<char>(flags);


  uint32_t base =
      data[0].t;


  for (uint16_t i = 0;
       i < count;
       i++) {

    uint32_t timestamp =
        (uint32_t)
        (data[i].t - base);


    memcpy(
        p,
        &timestamp,
        sizeof(timestamp));

    p += 4;


    *p++ =
        static_cast<char>(
            data[i].value);
  }


  /*
   * HTTP-Task und ESPHome-loop können auf unterschiedlichen
   * FreeRTOS-Tasks laufen.
   */
  if (last_capture_mutex != nullptr) {

    if (
        xSemaphoreTake(
            last_capture_mutex,
            pdMS_TO_TICKS(100))
        == pdTRUE) {

      last_capture_data =
          std::move(output);

      xSemaphoreGive(
          last_capture_mutex);
    }
  }
}


/*
 * ============================================================================
 * HTTP Handler
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


    return request->url_to(url) == "/capture";
  }


  void handleRequest(
      web_server_idf::AsyncWebServerRequest *request)
      override
  {
    std::string output;


    if (last_capture_mutex != nullptr) {

      if (
          xSemaphoreTake(
              last_capture_mutex,
              pdMS_TO_TICKS(100))
          == pdTRUE) {

        output =
            last_capture_data;

        xSemaphoreGive(
            last_capture_mutex);
      }
    }


    if (output.empty()) {

      request->send(
          204,
          "text/plain",
          nullptr);

      return;
    }


    auto *response =
        request->beginResponse(
            200,
            "application/octet-stream",
            output);


    response->addHeader(
        "Content-Disposition",
        "attachment; filename=\"capture.la\"");


    request->send(
        response);
  }
};


static CaptureHandler capture_handler;


/*
 * ============================================================================
 * Setup
 * ============================================================================
 */

void BusSniffer::setup()
{
  /*
   * Mutex für letzten HTTP-Capture.
   */
  last_capture_mutex =
      xSemaphoreCreateMutex();


  /*
   * Alle drei Pins Eingänge.
   *
   * Nur SCL/SDA bekommen anschließend ISR-Handler.
   */
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
      gpio_config(&io);


  if (err != ESP_OK) {

    ESP_LOGE(
        TAG,
        "gpio_config failed: %d",
        err);

    return;
  }


  /*
   * Interrupts nur für SCL + SDA.
   */
  gpio_set_intr_type(
      PIN_SCL,
      GPIO_INTR_ANYEDGE);

  gpio_set_intr_type(
      PIN_SDA,
      GPIO_INTR_ANYEDGE);


  /*
   * Anfangszustand.
   */
  last_value =
      read_gpio_state();

  capture_initial_value =
      last_value;

  last_edge =
      (uint32_t)
      esp_timer_get_time();


  /*
   * ISR Service.
   */
  err =
      gpio_install_isr_service(0);


  if (
      err != ESP_OK &&
      err != ESP_ERR_INVALID_STATE) {

    ESP_LOGE(
        TAG,
        "gpio_install_isr_service failed: %d",
        err);

    return;
  }


  err =
      gpio_isr_handler_add(
          PIN_SCL,
          gpio_isr,
          nullptr);


  if (err != ESP_OK) {

    ESP_LOGE(
        TAG,
        "SCL ISR failed: %d",
        err);

    return;
  }


  err =
      gpio_isr_handler_add(
          PIN_SDA,
          gpio_isr,
          nullptr);


  if (err != ESP_OK) {

    ESP_LOGE(
        TAG,
        "SDA ISR failed: %d",
        err);

    return;
  }


  /*
   * HTTP.
   */
  if (
      web_server_base::global_web_server_base
      != nullptr) {

    web_server_base::
        global_web_server_base->
        add_handler(
            &capture_handler);


    ESP_LOGI(
        TAG,
        "Raw capture: GET /capture");

  } else {

    ESP_LOGW(
        TAG,
        "web_server_base unavailable");
  }


  ESP_LOGI(
      TAG,
      "================================");


  ESP_LOGI(
      TAG,
      "Passive CD40/SCD4x CO2 sniffer");


  ESP_LOGI(
      TAG,
      "SCL = GPIO40");


  ESP_LOGI(
      TAG,
      "SDA = GPIO39");


  ESP_LOGI(
      TAG,
      "CH2 = GPIO38 (sample only)");


  ESP_LOGI(
      TAG,
      "I2C target = 0x62");


  ESP_LOGI(
      TAG,
      "command = 0xEC05");


  ESP_LOGI(
      TAG,
      "================================");
}


/*
 * ============================================================================
 * Loop
 * ============================================================================
 */

void BusSniffer::loop()
{
  /*
   * ------------------------------------------------------------
   * Capture durch Buffer-Overflow abgeschlossen?
   * ------------------------------------------------------------
   */

  if (!capture_finished) {

    if (sample_count == 0)
      return;


    uint32_t now =
        (uint32_t)
        esp_timer_get_time();


    /*
     * Bus noch aktiv.
     */
    if (
        (uint32_t)
        (now - last_edge)
        <
        CAPTURE_TIMEOUT_US) {

      return;
    }


    /*
     * Capture einfrieren.
     */
    capturing = false;
    capture_finished = true;
  }


  /*
   * ISR verändert samples[] jetzt nicht mehr.
   */

  uint16_t count =
      sample_count;


  if (count > MAX_SAMPLES)
    count = MAX_SAMPLES;


  bool overflow =
      capture_overflow;


  uint8_t initial_value =
      capture_initial_value;


  if (count != 0) {

    uint32_t duration =
        (uint32_t)
        (samples[count - 1].t -
         samples[0].t);


    ESP_LOGD(
        TAG,
        "Capture: %u events, %lu us%s",
        count,
        (unsigned long) duration,
        overflow ? " OVERFLOW" : "");


    /*
     * I2C auswerten.
     */
    decode_i2c_capture(
        samples,
        count,
        initial_value,
        this->co2_sensor_);


    /*
     * Rohdaten für /capture archivieren.
     */
    store_raw_capture(
        samples,
        count,
        overflow);
  }


  /*
   * ------------------------------------------------------------
   * Sofort nächsten Capture starten.
   * ------------------------------------------------------------
   */

  sample_count = 0;

  capture_overflow = false;
  capture_finished = false;


  last_value =
      read_gpio_state();

  capture_initial_value =
      last_value;


  last_edge =
      (uint32_t)
      esp_timer_get_time();


  capturing = true;
}


}  // namespace bus_sniffer
}  // namespace esphome
