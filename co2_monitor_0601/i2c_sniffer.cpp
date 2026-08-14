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
namespace co2_monitor_0601 {
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

static FrameStatus classify_frame(const RawFrame &raw,
                                  EndCondition end_condition,
                                  uint8_t partial_bits) {
  if (raw.truncated) return FrameStatus::Truncated;
  if (partial_bits != 0) return FrameStatus::IncompleteByte;
  if (end_condition == EndCondition::CaptureEnd)
    return FrameStatus::CaptureEndedInFrame;
  return FrameStatus::Valid;
}

static void append_frame(const RawFrame &raw, EndCondition end_condition,
                         uint8_t partial_bits, Capture &capture) {
  if (raw.count == 0) return;
  if (capture.frame_count >= MAX_FRAMES) {
    capture.frame_errors++;
    return;
  }

  Frame &frame = capture.frames[capture.frame_count++];
  frame = Frame{};
  const uint8_t address_byte = raw.bytes[0];
  frame.address = static_cast<uint8_t>(address_byte >> 1);
  frame.direction = (address_byte & 0x01) ? Direction::Read : Direction::Write;
  frame.address_ack = raw.ack[0];
  frame.end_condition = end_condition;
  frame.partial_bits = partial_bits;
  frame.status = classify_frame(raw, end_condition, partial_bits);
  frame.length = static_cast<uint8_t>(raw.count - 1);
  if (frame.length > MAX_DATA_BYTES) frame.length = MAX_DATA_BYTES;
  for (uint8_t i = 0; i < frame.length; i++) {
    frame.data[i] = raw.bytes[i + 1];
    frame.ack[i] = raw.ack[i + 1];
  }

  if (!frame_valid(frame)) capture.frame_errors++;
}

static void finish_active_frame(const RawFrame &raw, EndCondition end_condition,
                                uint8_t partial_bits, Capture &capture) {
  if (raw.count != 0) {
    append_frame(raw, end_condition, partial_bits, capture);
    return;
  }

  // START followed by STOP/repeated START/capture end before even one complete
  // address byte is a malformed capture segment. There is no trustworthy
  // address to expose as a Frame, so account for it at Capture level only.
  capture.frame_errors++;
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
      if (active) {
        // The SCL rise used to set up a repeated START is indistinguishable
        // from the first bit of a following byte until SDA falls while SCL is
        // high. Roll that one speculative bit back at the boundary.
        const uint8_t partial_bits = bit_count == 1 ? 0 : bit_count;
        finish_active_frame(raw, EndCondition::RepeatedStart, partial_bits, capture);
      }
      clear_raw_frame(raw);
      current_byte = 0;
      bit_count = 0;
      active = true;
      previous = current;
      continue;
    }

    if (active && !prev_sda && cur_sda && cur_scl) {  // STOP
      // As with repeated START, the SCL rise immediately preceding STOP is
      // tentatively seen as one data bit. It is part of the STOP setup, not an
      // incomplete byte.
      const uint8_t partial_bits = bit_count == 1 ? 0 : bit_count;
      finish_active_frame(raw, EndCondition::Stop, partial_bits, capture);
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

  if (active)
    finish_active_frame(raw, EndCondition::CaptureEnd, bit_count, capture);
}

#if RTRH_DEBUG_CAPTURE
static std::string last_capture_data;
static SemaphoreHandle_t last_capture_mutex = nullptr;
static bool last_capture_frozen = false;
static uint32_t last_capture_sequence = 0;
static uint32_t next_capture_sequence = 0;

static uint32_t store_raw_capture(const volatile Sample *data, uint16_t count,
                                  uint8_t initial_value, bool overflow) {
  if (!count || !last_capture_mutex) return 0;

  // Do not serialize and allocate ~20 KiB every cycle while an earlier
  // suspicious capture is intentionally being preserved.
  if (xSemaphoreTake(last_capture_mutex, pdMS_TO_TICKS(100)) != pdTRUE) return 0;
  const bool frozen = last_capture_frozen;
  xSemaphoreGive(last_capture_mutex);
  if (frozen) return 0;

  std::string output;
  // LA02 adds flags + the bus state before the first captured edge. Preserving
  // that initial state is important for correctly reconstructing the first
  // START/STOP transition in an exported waveform.
  output.resize(10 + static_cast<size_t>(count) * 5);
  char *p = output.data();
  memcpy(p, "LA02", 4); p += 4;
  const uint32_t count32 = count;
  memcpy(p, &count32, sizeof(count32)); p += 4;
  *p++ = overflow ? 0x01 : 0x00;
  *p++ = static_cast<char>(initial_value);
  const uint32_t base = data[0].t;
  for (uint16_t i = 0; i < count; i++) {
    const uint32_t timestamp = static_cast<uint32_t>(data[i].t - base);
    memcpy(p, &timestamp, sizeof(timestamp)); p += 4;
    *p++ = static_cast<char>(data[i].value);
  }

  if (xSemaphoreTake(last_capture_mutex, pdMS_TO_TICKS(100)) != pdTRUE) return 0;
  uint32_t sequence = 0;
  if (!last_capture_frozen) {
    last_capture_data = std::move(output);
    last_capture_sequence = ++next_capture_sequence;
    sequence = last_capture_sequence;
  }
  xSemaphoreGive(last_capture_mutex);
  return sequence;
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
    bool released_freeze = false;
    uint32_t sequence = 0;
    if (last_capture_mutex &&
        xSemaphoreTake(last_capture_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      output = last_capture_data;
      sequence = last_capture_sequence;
      if (last_capture_frozen && !output.empty()) {
        last_capture_frozen = false;
        released_freeze = true;
      }
      xSemaphoreGive(last_capture_mutex);
    }
    if (output.empty()) {
      request->send(204, "text/plain", nullptr);
      return;
    }
    auto *response = request->beginResponse(200, "application/octet-stream", output);
    response->addHeader("Content-Disposition", "attachment; filename=\"capture.la\"");
    request->send(response);
    if (released_freeze) {
      ESP_LOGD(TAG, "Released frozen raw I2C capture #%lu after /capture download",
               static_cast<unsigned long>(sequence));
      (void) sequence;
    }
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

bool freeze_last_capture(uint32_t sequence, const char *reason) {
  (void) reason;
  if (!last_capture_mutex || sequence == 0) return false;

  bool frozen = false;
  if (xSemaphoreTake(last_capture_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    if (!last_capture_frozen && !last_capture_data.empty() &&
        last_capture_sequence == sequence) {
      last_capture_frozen = true;
      frozen = true;
    }
    xSemaphoreGive(last_capture_mutex);
  }

  if (frozen) {
    ESP_LOGD(TAG, "Frozen suspicious raw I2C capture #%lu (%s); GET /capture to release",
             static_cast<unsigned long>(sequence), reason ? reason : "unknown reason");
    (void) sequence;
  }
  return frozen;
}

static const char *end_condition_name(EndCondition condition) {
  switch (condition) {
    case EndCondition::Stop: return "STOP";
    case EndCondition::RepeatedStart: return "RESTART";
    case EndCondition::CaptureEnd: return "CAPTURE_END";
  }
  return "?";
}

static const char *frame_status_name(FrameStatus status) {
  switch (status) {
    case FrameStatus::Valid: return "VALID";
    case FrameStatus::IncompleteByte: return "INCOMPLETE_BYTE";
    case FrameStatus::CaptureEndedInFrame: return "CAPTURE_END_IN_FRAME";
    case FrameStatus::Truncated: return "TRUNCATED";
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
  if (used >= static_cast<int>(sizeof(line))) return;

  if (frame.status == FrameStatus::Valid) {
    snprintf(line + used, sizeof(line) - static_cast<size_t>(used), " [%s]",
             end_condition_name(frame.end_condition));
  } else if (frame.partial_bits != 0) {
    snprintf(line + used, sizeof(line) - static_cast<size_t>(used), " [%s,%s,bits=%u]",
             end_condition_name(frame.end_condition), frame_status_name(frame.status),
             frame.partial_bits);
  } else {
    snprintf(line + used, sizeof(line) - static_cast<size_t>(used), " [%s,%s]",
             end_condition_name(frame.end_condition), frame_status_name(frame.status));
  }

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
#if RTRH_DEBUG_CAPTURE
  capture.debug_raw_sequence = 0;
#endif

  if (count) {
    if (overflow) {
      capture.frame_errors++;
    } else {
      decode_capture(samples, count, initial_value, capture);
    }
#if RTRH_DEBUG_CAPTURE
    capture.debug_raw_sequence = store_raw_capture(samples, count, initial_value, overflow);
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
}  // namespace co2_monitor_0601
}  // namespace esphome
