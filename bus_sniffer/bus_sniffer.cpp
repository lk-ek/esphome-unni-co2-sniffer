#include "bus_sniffer.h"

#include "esphome/core/log.h"
#include "esphome/components/web_server_base/web_server_base.h"

#include "driver/gpio.h"
#include "esp_timer.h"

#include <cstring>
#include <string>

namespace esphome {
namespace bus_sniffer {

static const char *TAG = "bus_sniffer";

/*
 * ============================================================================
 * 3-channel Logic Analyzer
 * ============================================================================
 *
 * CH0 = GPIO40
 * CH1 = GPIO39
 * CH2 = GPIO38
 *
 * Jede Flanke auf einem der drei GPIOs erzeugt ein Event:
 *
 *   timestamp + Zustand aller drei GPIOs
 *
 * Es findet keinerlei Protokolldekodierung auf dem ESP32 statt.
 *
 * Nach CAPTURE_TIMEOUT_US ohne weitere Flanke wird der Capture eingefroren
 * und kann über
 *
 *   GET /capture
 *
 * heruntergeladen werden.
 *
 * Nach dem Download wird automatisch der nächste Capture gestartet.
 */


/*
 * ============================================================================
 * Pins
 * ============================================================================
 */

static constexpr gpio_num_t PIN_CH0 = GPIO_NUM_40;
static constexpr gpio_num_t PIN_CH1 = GPIO_NUM_39;
static constexpr gpio_num_t PIN_CH2 = GPIO_NUM_38;


/*
 * ============================================================================
 * Capture-Konfiguration
 * ============================================================================
 */

// Anzahl der maximal gespeicherten Flanken.
//
// Ein Sample benötigt logisch 5 Byte:
//   uint32_t timestamp
//   uint8_t  value
//
// Im RAM kann der Compiler das Struct allerdings auf 8 Byte ausrichten.
static constexpr uint16_t MAX_SAMPLES = 4096;


// Capture wird beendet, wenn so lange keine Flanke mehr auftrat.
static constexpr uint32_t CAPTURE_TIMEOUT_US = 5000;


/*
 * ============================================================================
 * Sample
 * ============================================================================
 *
 * value:
 *
 *   bit 0 = GPIO40 / CH0
 *   bit 1 = GPIO39 / CH1
 *   bit 2 = GPIO38 / CH2
 */

struct Sample {
  uint32_t t;
  uint8_t value;
};


/*
 * ============================================================================
 * Capture State
 * ============================================================================
 */

static volatile Sample samples[MAX_SAMPLES];

static volatile uint16_t sample_count = 0;

static volatile uint32_t last_edge = 0;

static volatile uint8_t last_value = 0;


// true:
// ISR darf Samples schreiben.
static volatile bool capturing = true;


// true:
// abgeschlossener Capture liegt zum Download bereit.
static volatile bool capture_ready = false;


// true:
// Buffer ist vollgelaufen.
static volatile bool capture_overflow = false;


/*
 * ============================================================================
 * GPIOs lesen
 * ============================================================================
 */

static inline uint8_t IRAM_ATTR read_gpio_state()
{
  uint8_t value = 0;

  if (gpio_get_level(PIN_CH0))
    value |= 0x01;

  if (gpio_get_level(PIN_CH1))
    value |= 0x02;

  if (gpio_get_level(PIN_CH2))
    value |= 0x04;

  return value;
}


/*
 * ============================================================================
 * GPIO ISR
 * ============================================================================
 */

static void IRAM_ATTR gpio_isr(void *arg)
{
  if (!capturing)
    return;


  uint32_t now =
      (uint32_t) esp_timer_get_time();


  uint8_t value =
      read_gpio_state();


  /*
   * Falls mehrere GPIO-Interrupts praktisch gleichzeitig ausgelöst wurden,
   * kann der zweite ISR-Aufruf denselben Gesamtzustand sehen.
   *
   * Diesen brauchen wir nicht zweimal zu speichern.
   */
  if (value == last_value)
    return;


  last_value = value;
  last_edge = now;


  /*
   * Event speichern.
   */
  uint16_t index = sample_count;

  if (index < MAX_SAMPLES) {

    samples[index].t = now;
    samples[index].value = value;

    sample_count = index + 1;

  } else {

    /*
     * Buffer voll.
     *
     * Capture sofort einfrieren.
     */
    capture_overflow = true;
    capturing = false;
    capture_ready = true;
  }
}


/*
 * ============================================================================
 * HTTP Capture Handler
 * ============================================================================
 *
 * GET /capture
 *
 *
 * Binärformat LA01
 * ----------------
 *
 * Header:
 *
 *   Offset  Size   Inhalt
 *
 *      0      4    "LA01"
 *      4      4    Anzahl Samples, uint32 little endian
 *      8      1    Flags
 *
 *
 * Flags:
 *
 *   bit 0 = Capture Buffer Overflow
 *
 *
 * Danach pro Sample:
 *
 *   4 Byte uint32 timestamp_us
 *   1 Byte GPIO state
 *
 *
 * timestamp_us ist relativ zum ersten gespeicherten Event.
 *
 *
 * GPIO state:
 *
 *   bit 0 = GPIO40 / CH0
 *   bit 1 = GPIO39 / CH1
 *   bit 2 = GPIO38 / CH2
 *
 *
 * Gesamtgröße:
 *
 *   9 + sample_count * 5 Byte
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
    /*
     * Noch kein abgeschlossener Capture.
     */
    if (!capture_ready) {

      request->send(
          204,
          "text/plain",
          nullptr);

      return;
    }


    /*
     * capturing == false.
     *
     * Die ISR verändert samples[] jetzt nicht mehr.
     */
    uint16_t count =
        sample_count;


    bool overflow =
        capture_overflow;


    /*
     * HTTP-Datei erzeugen.
     *
     * 9 Byte Header
     * 5 Byte pro Sample
     */
    std::string output;

    output.resize(
        9 +
        static_cast<size_t>(count) * 5
    );


    char *p =
        output.data();


    /*
     * Magic
     */
    memcpy(
        p,
        "LA01",
        4);

    p += 4;


    /*
     * Anzahl Samples
     */
    uint32_t count32 =
        count;

    memcpy(
        p,
        &count32,
        sizeof(count32));

    p += sizeof(count32);


    /*
     * Flags
     */
    uint8_t flags = 0;

    if (overflow)
      flags |= 0x01;

    *p++ =
        static_cast<char>(flags);


    /*
     * Timestamp des ersten Events.
     */
    uint32_t base_time = 0;

    if (count > 0)
      base_time = samples[0].t;


    /*
     * Samples serialisieren.
     */
    for (uint16_t i = 0;
         i < count;
         i++) {

      uint32_t timestamp =
          (uint32_t)
          (samples[i].t - base_time);


      memcpy(
          p,
          &timestamp,
          sizeof(timestamp));

      p += sizeof(timestamp);


      *p++ =
          static_cast<char>(
              samples[i].value);
    }


    /*
     * HTTP Response.
     *
     * Der std::string-Overload übernimmt die Daten in das
     * Response-Objekt.
     */
    auto *response =
        request->beginResponse(
            200,
            "application/octet-stream",
            output);


    response->addHeader(
        "Content-Disposition",
        "attachment; filename=\"capture.la\"");


    /*
     * Jetzt darf der nächste Capture beginnen.
     */
    sample_count = 0;

    capture_overflow = false;
    capture_ready = false;


    last_value =
        read_gpio_state();


    last_edge =
        (uint32_t)
        esp_timer_get_time();


    capturing = true;


    /*
     * Response abschicken.
     */
    request->send(response);
  }
};


/*
 * Handler muss dauerhaft existieren.
 */
static CaptureHandler capture_handler;


/*
 * ============================================================================
 * Setup
 * ============================================================================
 */

void BusSniffer::setup()
{
  /*
   * GPIO-Konfiguration.
   *
   * Keine internen Pullups.
   *
   * GPIO39 und GPIO40 besitzen bei dir externe 10k Pullups.
   */
  gpio_config_t io = {};

  io.mode =
      GPIO_MODE_INPUT;

  io.pull_up_en =
      GPIO_PULLUP_DISABLE;

  io.pull_down_en =
      GPIO_PULLDOWN_DISABLE;

  io.intr_type =
      GPIO_INTR_ANYEDGE;


/*  io.pin_bit_mask =
      (1ULL << PIN_CH0) |
      (1ULL << PIN_CH1) |
      (1ULL << PIN_CH2);
*/

  io.pin_bit_mask =
      (1ULL << PIN_CH0) |
      (1ULL << PIN_CH1);


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
   * Anfangszustand übernehmen.
   */
  last_value =
      read_gpio_state();


  last_edge =
      (uint32_t)
      esp_timer_get_time();


  /*
   * GPIO ISR Service.
   */
  err =
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
   * CH0 / GPIO40
   */
  err =
      gpio_isr_handler_add(
          PIN_CH0,
          gpio_isr,
          nullptr);


  if (err != ESP_OK) {

    ESP_LOGE(
        TAG,
        "GPIO40 ISR failed: %d",
        err);

    return;
  }


  /*
   * CH1 / GPIO39
   */
  err =
      gpio_isr_handler_add(
          PIN_CH1,
          gpio_isr,
          nullptr);


  if (err != ESP_OK) {

    ESP_LOGE(
        TAG,
        "GPIO39 ISR failed: %d",
        err);

    return;
  }


  /*
   * CH2 / GPIO38
  err =
      gpio_isr_handler_add(
          PIN_CH2,
          gpio_isr,
          nullptr);
   */


  if (err != ESP_OK) {

    ESP_LOGE(
        TAG,
        "GPIO38 ISR failed: %d",
        err);

    return;
  }


  /*
   * HTTP Handler registrieren.
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
        "HTTP endpoint: /capture");

  } else {

    ESP_LOGE(
        TAG,
        "web_server_base unavailable");

    return;
  }


  ESP_LOGI(
      TAG,
      "================================");


  ESP_LOGI(
      TAG,
      "3-channel logic analyzer ready");


  ESP_LOGI(
      TAG,
      "CH0 = GPIO40");


  ESP_LOGI(
      TAG,
      "CH1 = GPIO39");


  ESP_LOGI(
      TAG,
      "CH2 = GPIO38");


  ESP_LOGI(
      TAG,
      "MAX_SAMPLES = %u",
      MAX_SAMPLES);


  ESP_LOGI(
      TAG,
      "timeout = %lu us",
      (unsigned long)
      CAPTURE_TIMEOUT_US);


  ESP_LOGI(
      TAG,
      "GET /capture");


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
   * Capture bereits abgeschlossen.
   */
  if (!capturing)
    return;


  /*
   * Noch keine Flanke gesehen.
   */
  if (sample_count == 0)
    return;


  uint32_t now =
      (uint32_t)
      esp_timer_get_time();


  /*
   * Noch Aktivität auf dem Bus.
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
   *
   * Ab jetzt schreibt die ISR nicht mehr in samples[].
   */
  capturing = false;

  capture_ready = true;


  ESP_LOGI(
      TAG,
      "Capture ready: %u events, %lu us",
      sample_count,
      (unsigned long)
      (samples[sample_count - 1].t -
       samples[0].t));
}


}  // namespace bus_sniffer
}  // namespace esphome

