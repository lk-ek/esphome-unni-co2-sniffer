// SPDX-FileCopyrightText: 2026 The esphome-unni-co2-sniffer contributors
// SPDX-License-Identifier: GPL-3.0-or-later
#include "co2_decoder.h"


#include "esphome/core/log.h"
#include "driver/gpio.h"
#include "esp_timer.h"

#include <cstring>

#if RTRH_DEBUG_CAPTURE
#include "esphome/components/web_server_base/web_server_base.h"
#include "esp_http_server.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <string>
#include <utility>
#endif

namespace esphome {
namespace bus_sniffer {
namespace co2_decoder {

static const char *TAG = "co2_decoder";
static gpio_num_t pin_scl = GPIO_NUM_7;
static gpio_num_t pin_sda = GPIO_NUM_6;
static constexpr uint16_t MAX_SAMPLES = 4096;
static constexpr uint32_t CAPTURE_TIMEOUT_US = 5000;

struct Sample {
  uint32_t t;
  uint8_t value;
};

static volatile Sample samples[MAX_SAMPLES];
static volatile uint16_t sample_count = 0;
static volatile uint32_t last_edge = 0;
static volatile uint8_t last_value = 0xff;
static volatile uint8_t capture_initial_value = 0xff;
static volatile bool capturing = true;
static bool capture_enabled = true;
static volatile bool capture_finished = false;
static volatile bool capture_overflow = false;

static inline uint8_t IRAM_ATTR read_gpio_state() {
  uint8_t value = 0;
  if (gpio_get_level(pin_scl)) value |= 0x01;
  if (gpio_get_level(pin_sda)) value |= 0x02;
  return value;
}
static inline bool scl_level(uint8_t value) { return (value & 0x01) != 0; }
static inline bool sda_level(uint8_t value) { return (value & 0x02) != 0; }

static void IRAM_ATTR gpio_isr(void *) {
  if (!capturing) return;
  const uint32_t now = static_cast<uint32_t>(esp_timer_get_time());
  const uint8_t value = read_gpio_state();
  if (value == last_value) return;
  if (sample_count == 0) capture_initial_value = last_value;
  last_value = value;
  last_edge = now;

  const uint16_t index = sample_count;
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

// Protocol provenance: this CRC uses the Sensirion/SCD4x-compatible wire
// parameters observed by the passive decoder (initial value 0xFF, polynomial
// 0x31). It is local protocol code, not a vendored Sensirion driver.
static uint8_t sensirion_crc(uint8_t byte0, uint8_t byte1) {
  uint8_t crc = 0xff;
  const uint8_t data[2] = {byte0, byte1};
  for (uint8_t value : data) {
    crc ^= value;
    for (uint8_t bit = 0; bit < 8; bit++)
      crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0x31)
                         : static_cast<uint8_t>(crc << 1);
  }
  return crc;
}

struct I2CTransaction {
  uint8_t bytes[16]{};
  bool ack[16]{};
  uint8_t count{0};
};

static void clear_transaction(I2CTransaction &txn) {
  txn.count = 0;
  memset(txn.bytes, 0, sizeof(txn.bytes));
  memset(txn.ack, 0, sizeof(txn.ack));
}

static void process_transaction(const I2CTransaction &txn, Result &result) {
  if (txn.count == 0) return;

  // SCD4x-compatible read_measurement command: C4 EC 05.
  if (txn.count >= 3 && txn.bytes[0] == 0xC4 &&
      txn.bytes[1] == 0xEC && txn.bytes[2] == 0x05)
    return;

  // Response: C5 CO2_MSB CO2_LSB CRC.
  if (txn.bytes[0] != 0xC5) return;
  if (txn.count < 4) {
    result.frame_errors++;
    return;
  }
  if (!txn.ack[0] || !txn.ack[1] || !txn.ack[2] || txn.ack[3]) {
    result.frame_errors++;
    return;
  }

  const uint8_t msb = txn.bytes[1];
  const uint8_t lsb = txn.bytes[2];
  if (txn.bytes[3] != sensirion_crc(msb, lsb)) {
    result.crc_errors++;
    return;
  }

  result.co2_ppm = (static_cast<uint16_t>(msb) << 8) | lsb;
  result.have_co2 = true;
}

static Result decode_capture(const volatile Sample *data, uint16_t count,
                             uint8_t initial_value) {
  Result result;
  if (count == 0) return result;

  bool active = false;
  uint8_t current_byte = 0;
  uint8_t bit_count = 0;
  I2CTransaction txn;
  clear_transaction(txn);
  uint8_t previous = initial_value;

  for (uint16_t i = 0; i < count; i++) {
    const uint8_t current = data[i].value;
    const bool prev_scl = scl_level(previous);
    const bool cur_scl = scl_level(current);
    const bool prev_sda = sda_level(previous);
    const bool cur_sda = sda_level(current);

    if (prev_sda && !cur_sda && cur_scl) {  // START / repeated START
      if (active && txn.count) process_transaction(txn, result);
      clear_transaction(txn);
      current_byte = 0;
      bit_count = 0;
      active = true;
      previous = current;
      continue;
    }

    if (active && !prev_sda && cur_sda && cur_scl) {  // STOP
      if (txn.count) process_transaction(txn, result);
      clear_transaction(txn);
      current_byte = 0;
      bit_count = 0;
      active = false;
      previous = current;
      continue;
    }

    if (active && !prev_scl && cur_scl) {
      const bool bit = cur_sda;
      if (bit_count < 8) {
        current_byte = static_cast<uint8_t>(current_byte << 1);
        if (bit) current_byte |= 1;
        bit_count++;
      } else {
        if (txn.count < sizeof(txn.bytes)) {
          txn.bytes[txn.count] = current_byte;
          txn.ack[txn.count] = !bit;
          txn.count++;
        }
        current_byte = 0;
        bit_count = 0;
      }
    }
    previous = current;
  }

  if (active && txn.count) process_transaction(txn, result);
  return result;
}

#if RTRH_DEBUG_CAPTURE
static std::string last_capture_data;
static SemaphoreHandle_t last_capture_mutex = nullptr;

static void store_raw_capture(const volatile Sample *data, uint16_t count,
                              bool overflow) {
  if (!count) return;
  std::string output;
  output.resize(9 + static_cast<size_t>(count) * 5);
  char *p = output.data();
  memcpy(p, "LA01", 4); p += 4;
  const uint32_t count32 = count;
  memcpy(p, &count32, sizeof(count32)); p += 4;
  *p++ = overflow ? 0x01 : 0x00;
  const uint32_t base = data[0].t;
  for (uint16_t i = 0; i < count; i++) {
    const uint32_t timestamp = static_cast<uint32_t>(data[i].t - base);
    memcpy(p, &timestamp, sizeof(timestamp)); p += 4;
    *p++ = static_cast<char>(data[i].value);
  }
  if (last_capture_mutex &&
      xSemaphoreTake(last_capture_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    last_capture_data = std::move(output);
    xSemaphoreGive(last_capture_mutex);
  }
}

class CaptureHandler : public web_server_idf::AsyncWebHandler {
 public:
  bool canHandle(web_server_idf::AsyncWebServerRequest *request) const override {
    if (request->method() != HTTP_GET) return false;
    char url[web_server_idf::AsyncWebServerRequest::URL_BUF_SIZE];
    return request->url_to(url) == "/capture";
  }

  void handleRequest(web_server_idf::AsyncWebServerRequest *request) override {
    std::string output;
    if (last_capture_mutex &&
        xSemaphoreTake(last_capture_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      output = last_capture_data;
      xSemaphoreGive(last_capture_mutex);
    }
    if (output.empty()) {
      request->send(204, "text/plain", nullptr);
      return;
    }
    auto *response = request->beginResponse(200, "application/octet-stream", output);
    response->addHeader("Content-Disposition", "attachment; filename=\"capture.la\"");
    request->send(response);
  }
};
static CaptureHandler capture_handler;

void register_debug_handler() {
  if (!last_capture_mutex) last_capture_mutex = xSemaphoreCreateMutex();
  if (web_server_base::global_web_server_base)
    web_server_base::global_web_server_base->add_handler(&capture_handler);
  else
    ESP_LOGW(TAG, "web_server_base unavailable");
}
#endif

bool setup(uint8_t sda_pin, uint8_t scl_pin) {
  pin_sda = static_cast<gpio_num_t>(sda_pin);
  pin_scl = static_cast<gpio_num_t>(scl_pin);
  gpio_config_t io{};
  io.mode = GPIO_MODE_INPUT;
  io.pull_up_en = GPIO_PULLUP_DISABLE;
  io.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io.intr_type = GPIO_INTR_DISABLE;
  io.pin_bit_mask = (1ULL << pin_scl) | (1ULL << pin_sda);
  esp_err_t err = gpio_config(&io);
  if (err != ESP_OK) return false;

  gpio_set_intr_type(pin_scl, GPIO_INTR_ANYEDGE);
  gpio_set_intr_type(pin_sda, GPIO_INTR_ANYEDGE);
  last_value = read_gpio_state();
  capture_initial_value = last_value;
  last_edge = static_cast<uint32_t>(esp_timer_get_time());

  err = gpio_isr_handler_add(pin_scl, gpio_isr, nullptr);
  if (err != ESP_OK) return false;
  err = gpio_isr_handler_add(pin_sda, gpio_isr, nullptr);
  return err == ESP_OK;
}


void set_capture_enabled(bool enabled) {
  if (capture_enabled == enabled) return;

  // Stop both edge sources while resetting shared ISR state so no half-frame
  // can leak across a sleep/awake boundary.
  gpio_intr_disable(pin_scl);
  gpio_intr_disable(pin_sda);
  capture_enabled = enabled;
  capturing = false;
  sample_count = 0;
  capture_finished = false;
  capture_overflow = false;
  last_value = read_gpio_state();
  capture_initial_value = last_value;
  last_edge = static_cast<uint32_t>(esp_timer_get_time());
  capturing = enabled;
  if (enabled) {
    gpio_intr_enable(pin_scl);
    gpio_intr_enable(pin_sda);
  }
}

bool poll(Result &result) {
  if (!capture_enabled) return false;
  if (!capture_finished) {
    if (sample_count == 0) return false;
    const uint32_t now = static_cast<uint32_t>(esp_timer_get_time());
    if (static_cast<uint32_t>(now - last_edge) < CAPTURE_TIMEOUT_US) return false;
    capturing = false;
    capture_finished = true;
  }

  uint16_t count = sample_count;
  if (count > MAX_SAMPLES) count = MAX_SAMPLES;
  const bool overflow = capture_overflow;
  const uint8_t initial_value = capture_initial_value;
  result = {};

  if (count) {
    if (overflow)
      result.frame_errors++;
    else
      result = decode_capture(samples, count, initial_value);
#if RTRH_DEBUG_CAPTURE
    store_raw_capture(samples, count, overflow);
#endif
  }

  sample_count = 0;
  capture_overflow = false;
  capture_finished = false;
  last_value = read_gpio_state();
  capture_initial_value = last_value;
  last_edge = static_cast<uint32_t>(esp_timer_get_time());
  capturing = true;
  return true;
}

}  // namespace co2_decoder
}  // namespace bus_sniffer
}  // namespace esphome
