// SPDX-FileCopyrightText: 2026 The esphome-unni-co2-sniffer contributors
// SPDX-License-Identifier: GPL-3.0-or-later
#include "i2c_sniffer.h"

#include "esphome/core/log.h"
#include "driver/gpio.h"
#include "esp_timer.h"

#include <cstdio>
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
namespace i2c_sniffer {

#if RTRH_DEBUG_CAPTURE
static const char *TAG = "i2c_sniffer";
#endif
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

struct RawFrame {
  uint8_t bytes[MAX_DATA_BYTES + 1]{};
  bool ack[MAX_DATA_BYTES + 1]{};
  uint8_t count{0};
  bool truncated{false};
};

static void clear_raw_frame(RawFrame &raw) {
  raw.count = 0;
  raw.truncated = false;
}

static void append_frame(const RawFrame &raw, EndCondition end_condition,
                         Capture &capture) {
  if (raw.count == 0) return;
  if (capture.frame_count >= MAX_FRAMES) {
    capture.frame_errors++;
    return;
  }

  Frame &frame = capture.frames[capture.frame_count++];
  const uint8_t address_byte = raw.bytes[0];
  frame.address = static_cast<uint8_t>(address_byte >> 1);
  frame.direction = (address_byte & 0x01) ? Direction::Read : Direction::Write;
  frame.address_ack = raw.ack[0];
  frame.end_condition = end_condition;
  frame.truncated = raw.truncated;
  frame.length = static_cast<uint8_t>(raw.count - 1);
  if (frame.length > MAX_DATA_BYTES) frame.length = MAX_DATA_BYTES;
  for (uint8_t i = 0; i < frame.length; i++) {
    frame.data[i] = raw.bytes[i + 1];
    frame.ack[i] = raw.ack[i + 1];
  }
}

static void decode_capture(const volatile Sample *data, uint16_t count,
                           uint8_t initial_value, Capture &capture) {
  if (count == 0) return;

  bool active = false;
  uint8_t current_byte = 0;
  uint8_t bit_count = 0;
  RawFrame raw;
  clear_raw_frame(raw);
  uint8_t previous = initial_value;

  for (uint16_t i = 0; i < count; i++) {
    const uint8_t current = data[i].value;
    const bool prev_scl = scl_level(previous);
    const bool cur_scl = scl_level(current);
    const bool prev_sda = sda_level(previous);
    const bool cur_sda = sda_level(current);

    if (prev_sda && !cur_sda && cur_scl) {  // START / repeated START
      if (active && raw.count)
        append_frame(raw, EndCondition::RepeatedStart, capture);
      clear_raw_frame(raw);
      current_byte = 0;
      bit_count = 0;
      active = true;
      previous = current;
      continue;
    }

    if (active && !prev_sda && cur_sda && cur_scl) {  // STOP
      if (raw.count) append_frame(raw, EndCondition::Stop, capture);
      clear_raw_frame(raw);
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
        if (raw.count < sizeof(raw.bytes)) {
          raw.bytes[raw.count] = current_byte;
          raw.ack[raw.count] = !bit;
          raw.count++;
        } else {
          raw.truncated = true;
        }
        current_byte = 0;
        bit_count = 0;
      }
    }
    previous = current;
  }

  if (active && raw.count)
    append_frame(raw, EndCondition::CaptureEnd, capture);
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

static const char *end_condition_name(EndCondition condition) {
  switch (condition) {
    case EndCondition::Stop: return "STOP";
    case EndCondition::RepeatedStart: return "RESTART";
    case EndCondition::CaptureEnd: return "CAPTURE_END";
  }
  return "?";
}

void log_frame(const Frame &frame, const char *label) {
  char line[256];
  int used = snprintf(line, sizeof(line), "%s: %c 0x%02X addr=%s",
                      label ? label : "I2C frame",
                      frame.direction == Direction::Read ? 'R' : 'W',
                      frame.address, frame.address_ack ? "ACK" : "NACK");
  if (used < 0) return;

  for (uint8_t i = 0; i < frame.length && used < static_cast<int>(sizeof(line)); i++) {
    const int written = snprintf(line + used, sizeof(line) - static_cast<size_t>(used),
                                 " %02X(%c)", frame.data[i], frame.ack[i] ? 'A' : 'N');
    if (written < 0) break;
    used += written;
  }
  if (used < static_cast<int>(sizeof(line)))
    snprintf(line + used, sizeof(line) - static_cast<size_t>(used), " [%s%s]",
             end_condition_name(frame.end_condition), frame.truncated ? ",TRUNC" : "");

  ESP_LOGD(TAG, "%s", line);
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

bool poll(Capture &capture) {
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
  capture.frame_count = 0;
  capture.frame_errors = 0;

  if (count) {
    if (overflow) {
      capture.frame_errors++;
    } else {
      decode_capture(samples, count, initial_value, capture);
    }
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

}  // namespace i2c_sniffer
}  // namespace bus_sniffer
}  // namespace esphome
