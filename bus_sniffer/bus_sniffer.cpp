#include "bus_sniffer.h"

#include "esphome/core/log.h"
#include "esphome/components/web_server_base/web_server_base.h"

#include "driver/gpio.h"
#include "esp_timer.h"

#include <string>
#include <cstring>

namespace esphome {
namespace bus_sniffer {

static const char *TAG = "BUS";

// -----------------------------------------------------------------------------
// 3 Logic-Analyzer-Kanäle
//
// Kanal 0 = GPIO40
// Kanal 1 = GPIO39
// Kanal 2 = GPIO38
// -----------------------------------------------------------------------------

static constexpr gpio_num_t PIN0 = GPIO_NUM_40;
static constexpr gpio_num_t PIN1 = GPIO_NUM_39;
static constexpr gpio_num_t PIN2 = GPIO_NUM_38;

// Maximale Anzahl Events pro Capture.
// Ein Event = Zeitpunkt + Zustand aller 3 GPIOs.
static constexpr uint16_t MAX_SAMPLES = 4096;

// Capture endet nach dieser Zeit ohne weitere Flanke.
static constexpr uint32_t CAPTURE_TIMEOUT_US = 5000;


// -----------------------------------------------------------------------------
// Rohdaten
// -----------------------------------------------------------------------------

struct Sample {
  uint32_t t;
  uint8_t value;
};

// Wird ausschließlich von ISR beschrieben, solange capture_ready == false.
static volatile Sample samples[MAX_SAMPLES];
static volatile uint16_t sample_count = 0;

static volatile uint32_t last_edge = 0;
static volatile uint8_t last_value = 0xff;

static volatile bool capturing = true;
static volatile bool capture_ready = false;


// -----------------------------------------------------------------------------
// GPIO lesen
// -----------------------------------------------------------------------------

static inline uint8_t read_gpio_state()
{
  uint8_t value = 0;

  if (gpio_get_level(PIN0))
    value |= 1;

  if (gpio_get_level(PIN1))
    value |= 2;

  if (gpio_get_level(PIN2))
    value |= 4;

  return value;
}


// -----------------------------------------------------------------------------
// GPIO ISR
// -----------------------------------------------------------------------------

static void IRAM_ATTR gpio_isr(void *arg)
{
  // Nach abgeschlossenem Capture nichts mehr aufzeichnen.
  if (!capturing)
    return;

  uint32_t now = (uint32_t) esp_timer_get_time();
  uint8_t value = read_gpio_state();

  // Nur tatsächliche Zustandsänderungen speichern.
  if (value == last_value)
    return;

  last_value = value;
  last_edge = now;

  if (sample_count < MAX_SAMPLES) {
    samples[sample_count].t = now;
    samples[sample_count].value = value;
    sample_count++;
  } else {
    // Buffer voll -> Capture beenden.
    capturing = false;
    capture_ready = true;
  }
}


// -----------------------------------------------------------------------------
// HTTP Handler
//
// GET /capture
//
// Format:
//
// Header:
//   4 Byte  "LA01"
//   4 Byte  Anzahl Samples (uint32)
//
// Sample:
//   4 Byte  Zeit relativ zum ersten Sample, µs
//   1 Byte  GPIO-Zustand
//
// GPIO-Zustand:
//   bit 0 = GPIO40
//   bit 1 = GPIO39
//   bit 2 = GPIO38
// -----------------------------------------------------------------------------

class CaptureHandler : public web_server_idf::AsyncWebHandler {
 public:
  bool canHandle(web_server_idf::AsyncWebServerRequest *request) const override
  {
    if (request->method() != HTTP_GET)
      return false;

    char url[web_server_idf::AsyncWebServerRequest::URL_BUF_SIZE];
    return request->url_to(url) == "/capture";
  }

  void handleRequest(web_server_idf::AsyncWebServerRequest *request) override
  {
    if (!capture_ready) {
      request->send(
          204,
          "text/plain",
          nullptr);
      return;
    }

    // Anzahl Samples einmal atomar übernehmen.
    uint16_t count = sample_count;

    // Binärdaten erzeugen.
    //
    // 8 Byte Header + 5 Byte pro Sample
    std::string output;
    output.resize(8 + (static_cast<size_t>(count) * 5));

    char *p = output.data();

    // Magic
    memcpy(p, "LA01", 4);
    p += 4;

    // Sample count
    uint32_t count32 = count;
    memcpy(p, &count32, 4);
    p += 4;

    uint32_t base_time = 0;

    if (count > 0)
      base_time = samples[0].t;

    for (uint16_t i = 0; i < count; i++) {

      uint32_t relative_time =
          (uint32_t)(samples[i].t - base_time);

      memcpy(p, &relative_time, 4);
      p += 4;

      *reinterpret_cast<uint8_t *>(p) =
          samples[i].value;

      p++;
    }

    // Danach kann sofort wieder ein neuer Capture beginnen.
    sample_count = 0;
    last_value = read_gpio_state();
    last_edge = (uint32_t) esp_timer_get_time();

    capture_ready = false;
    capturing = true;

    auto *response =
        request->beginResponse(
            200,
            "application/octet-stream",
            reinterpret_cast<const uint8_t *>(output.data()),
            output.size());

    response->addHeader(
        "Content-Disposition",
        "attachment; filename=\"capture.la\"");

    request->send(response);
  }
};


// Handler statisch anlegen.
// ESPHome übernimmt den Handler beim Webserver.
static CaptureHandler capture_handler;


// -----------------------------------------------------------------------------
// Setup
// -----------------------------------------------------------------------------

void BusSniffer::setup()
{
  gpio_config_t io = {};

  io.mode = GPIO_MODE_INPUT;
  io.pull_up_en = GPIO_PULLUP_DISABLE;
  io.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io.intr_type = GPIO_INTR_ANYEDGE;

  io.pin_bit_mask =
      (1ULL << PIN0) |
      (1ULL << PIN1) |
      (1ULL << PIN2);

  gpio_config(&io);

  // Anfangszustand übernehmen.
  last_value = read_gpio_state();
  last_edge = (uint32_t) esp_timer_get_time();

  // ISR-Service installieren.
  esp_err_t err = gpio_install_isr_service(0);

  if (err != ESP_OK &&
      err != ESP_ERR_INVALID_STATE) {

    ESP_LOGE(
        TAG,
        "gpio_install_isr_service failed: %d",
        err);

    return;
  }

  // Drei GPIOs -> dieselbe ISR.
  err = gpio_isr_handler_add(
      PIN0,
      gpio_isr,
      nullptr);

  if (err != ESP_OK) {
    ESP_LOGE(TAG, "GPIO40 ISR failed: %d", err);
    return;
  }

  err = gpio_isr_handler_add(
      PIN1,
      gpio_isr,
      nullptr);

  if (err != ESP_OK) {
    ESP_LOGE(TAG, "GPIO39 ISR failed: %d", err);
    return;
  }

  err = gpio_isr_handler_add(
      PIN2,
      gpio_isr,
      nullptr);

  if (err != ESP_OK) {
    ESP_LOGE(TAG, "GPIO38 ISR failed: %d", err);
    return;
  }

  // Eigenen HTTP-Endpunkt registrieren.
  if (web_server_base::global_web_server_base != nullptr) {

    web_server_base::global_web_server_base->add_handler(
        &capture_handler);

    ESP_LOGI(
        TAG,
        "HTTP capture endpoint: /capture");

  } else {

    ESP_LOGW(
        TAG,
        "web_server is not enabled - /capture unavailable");
  }

  ESP_LOGI(TAG, "==============================");
  ESP_LOGI(TAG, "3-CHANNEL LOGIC ANALYZER");
  ESP_LOGI(TAG, "CH0 = GPIO40");
  ESP_LOGI(TAG, "CH1 = GPIO39");
  ESP_LOGI(TAG, "CH2 = GPIO38");
  ESP_LOGI(TAG, "==============================");
}


// -----------------------------------------------------------------------------
// Loop
// -----------------------------------------------------------------------------

void BusSniffer::loop()
{
  // Nichts aufzuzeichnen.
  if (!capturing)
    return;

  if (sample_count == 0)
    return;

  uint32_t now =
      (uint32_t) esp_timer_get_time();

  // Nach 5 ms ohne Flanke Capture einfrieren.
  if ((uint32_t)(now - last_edge) >= CAPTURE_TIMEOUT_US) {

    capturing = false;
    capture_ready = true;

    ESP_LOGI(
        TAG,
        "Capture complete: %u samples",
        sample_count);
  }
}

}  // namespace bus_sniffer
}  // namespace esphome
