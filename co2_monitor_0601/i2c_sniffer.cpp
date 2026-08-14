// SPDX-FileCopyrightText: 2026 The esphome-unni-co2-sniffer contributors
// SPDX-License-Identifier: GPL-3.0-or-later
#include "i2c_sniffer.h"

#include "esphome/core/log.h"
#include "driver/gpio.h"
#if CO2_MONITOR_0601_RMT_SCL_ASSIST
#include "driver/rmt_rx.h"
#endif
#include "esp_timer.h"

#include <cstddef>
#include <cstdio>
#include <cstring>

#if RTRH_DEBUG_CAPTURE
#include "esphome/components/web_server_base/web_server_base.h"
#include "esp_http_server.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <new>
#include <string>
#include <utility>
#endif

namespace esphome {
namespace co2_monitor_0601 {
namespace i2c_sniffer {

static const char *TAG = "i2c_sniffer";
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

#if CO2_MONITOR_0601_RMT_SCL_ASSIST
// ESP32-C3 RMT hardware independently records SCL pulse durations. GPIO ISR
// capture is still used for SDA and for the raw debug trace, but RMT gives us
// an authoritative SCL edge count even when CPU interrupt latency collapses a
// whole high/low pulse into no GPIO state change at all.
static constexpr uint32_t RMT_RESOLUTION_HZ = 1000000;  // 1 us per tick
static constexpr uint16_t RMT_BUFFER_SYMBOLS = 128;
static constexpr uint16_t MAX_RMT_EDGES = RMT_BUFFER_SYMBOLS * 2;
static constexpr uint32_t RMT_EDGE_MATCH_TOLERANCE_US = 15;
static constexpr uint32_t RMT_WAIT_TIMEOUT_US = 20000;
static constexpr uint16_t MAX_RMT_REPAIRS = 32;

static rmt_channel_handle_t rmt_scl_channel = nullptr;
static rmt_symbol_word_t rmt_scl_symbols[RMT_BUFFER_SYMBOLS];
static volatile bool rmt_available = false;
static volatile bool rmt_enabled = false;
static volatile bool rmt_receive_started = false;
static volatile bool rmt_receive_done = false;
static volatile size_t rmt_received_symbols = 0;
static volatile uint32_t rmt_receive_start_us = 0;
static volatile esp_err_t rmt_arm_error = ESP_OK;
// Must live in internal RAM because rmt_receive() is started from the GPIO ISR.
static rmt_receive_config_t rmt_receive_config{};

static bool IRAM_ATTR rmt_rx_done_callback(rmt_channel_handle_t,
                                           const rmt_rx_done_event_data_t *edata,
                                           void *) {
  rmt_received_symbols = edata ? edata->num_symbols : 0;
  rmt_receive_started = false;
  rmt_receive_done = true;
  return false;
}

static esp_err_t IRAM_ATTR arm_rmt_receive_from_isr(uint32_t start_us) {
  if (!rmt_available || !rmt_enabled || rmt_scl_channel == nullptr ||
      rmt_receive_started || rmt_receive_done)
    return ESP_OK;

  rmt_received_symbols = 0;
  rmt_receive_start_us = start_us;
  const esp_err_t err = rmt_receive(rmt_scl_channel, rmt_scl_symbols,
                                    sizeof(rmt_scl_symbols), &rmt_receive_config);
  if (err == ESP_OK) {
    rmt_receive_started = true;
  } else {
    rmt_arm_error = err;
  }
  return err;
}

static bool setup_rmt_scl() {
  rmt_rx_channel_config_t config{};
  config.gpio_num = pin_scl;
  config.clk_src = RMT_CLK_SRC_DEFAULT;
  config.resolution_hz = RMT_RESOLUTION_HZ;
  // One ESP32-C3 hardware block is 48 symbols. The driver ping-pongs those
  // symbols into the larger user buffer, so a normal CO2 exchange fits without
  // reserving a second RMT memory block.
  config.mem_block_symbols = 48;
  config.intr_priority = 0;
  config.flags.invert_in = false;
  config.flags.with_dma = false;
  config.flags.allow_pd = false;

  esp_err_t err = rmt_new_rx_channel(&config, &rmt_scl_channel);
  if (err != ESP_OK) return false;

  // ESP-IDF currently enables an internal pull-up when allocating an RMT RX
  // channel. This sniffer must remain electrically passive; the external I2C
  // bus already owns its pull-ups. Undo that driver side effect immediately.
  gpio_pullup_dis(pin_scl);
  gpio_pulldown_dis(pin_scl);

  rmt_rx_event_callbacks_t callbacks{};
  callbacks.on_recv_done = rmt_rx_done_callback;
  err = rmt_rx_register_event_callbacks(rmt_scl_channel, &callbacks, nullptr);
  if (err != ESP_OK) {
    rmt_del_channel(rmt_scl_channel);
    rmt_scl_channel = nullptr;
    return false;
  }

  err = rmt_enable(rmt_scl_channel);
  if (err != ESP_OK) {
    rmt_del_channel(rmt_scl_channel);
    rmt_scl_channel = nullptr;
    return false;
  }
  rmt_receive_config.signal_range_min_ns = 1000;
  rmt_receive_config.signal_range_max_ns = CAPTURE_TIMEOUT_US * 1000U;
  rmt_receive_config.flags.en_partial_rx = false;
  rmt_enabled = true;
  rmt_available = true;
  rmt_receive_started = false;
  rmt_receive_done = false;
  rmt_received_symbols = 0;
  rmt_receive_start_us = 0;
  rmt_arm_error = ESP_OK;
  // Do not start RX while the bus is idle: the hardware idle threshold would
  // finish that job long before the next ~6 s CO2 transaction. The first GPIO
  // edge of each capture arms RMT instead.
  return true;
}

#endif

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
#if CO2_MONITOR_0601_RMT_SCL_ASSIST
  if (index == 0 && rmt_available && rmt_enabled && !rmt_receive_started &&
      !rmt_receive_done) {
    // Start hardware SCL timing at the beginning of the same GPIO burst.
    // rmt_receive() is explicitly ISR-safe when CONFIG_RMT_RECV_FUNC_IN_IRAM
    // is enabled. Starting here avoids an RMT job expiring during the long
    // idle interval between CO2 transactions.
    (void) arm_rmt_receive_from_isr(now);
  }
#endif
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

#if CO2_MONITOR_0601_RMT_SCL_ASSIST
struct SclEdge {
  uint32_t t;
  bool level;
};

// Scratch storage is static because this component is single-threaded outside
// its ISRs and ESPHome loop-task stack space is more valuable than a few KiB
// of fixed DRAM here.
static SclEdge gpio_scl_edges_work[MAX_RMT_EDGES];
static SclEdge rmt_scl_edges_work[MAX_RMT_EDGES];
static SclEdge rmt_missing_edges_work[MAX_RMT_REPAIRS];

static uint16_t collect_gpio_scl_edges(const volatile Sample *data, uint16_t count,
                                       uint8_t initial_value, SclEdge *edges,
                                       uint16_t max_edges) {
  uint16_t edge_count = 0;
  bool previous = scl_level(initial_value);
  for (uint16_t i = 0; i < count; i++) {
    const bool current = scl_level(data[i].value);
    if (current != previous) {
      if (edge_count < max_edges) {
        edges[edge_count].t = data[i].t;
        edges[edge_count].level = current;
      }
      edge_count++;
      previous = current;
    }
  }
  return edge_count;
}

static uint16_t collect_rmt_scl_edges(SclEdge *edges, uint16_t max_edges) {
  // A completely full user buffer may have had excess symbols discarded by
  // the driver. Do not use a potentially truncated RMT trace for repair.
  if (!rmt_receive_done || rmt_received_symbols == 0 ||
      rmt_received_symbols >= RMT_BUFFER_SYMBOLS)
    return 0;
  const size_t symbols = rmt_received_symbols;

  uint16_t edge_count = 0;
  uint32_t t = rmt_receive_start_us;
  bool have_level = false;
  bool level = false;

  auto append_part = [&](uint16_t duration, bool part_level) {
    if (duration == 0) return;
    // The first RMT part describes the level that already existed when RX was
    // armed; it is not itself an edge. Record transitions only at boundaries
    // between pulse parts. The previous implementation emitted a synthetic
    // edge at t=0, which made real GPIO/RMT timelines fail alignment.
    if (!have_level) {
      have_level = true;
      level = part_level;
      t += duration;
      return;
    }
    if (part_level != level) {
      if (edge_count < max_edges) {
        edges[edge_count].t = t;
        edges[edge_count].level = part_level;
      }
      edge_count++;
      level = part_level;
    }
    t += duration;
  };

  for (size_t i = 0; i < symbols; i++) {
    const rmt_symbol_word_t symbol = rmt_scl_symbols[i];
    append_part(symbol.duration0, symbol.level0 != 0);
    append_part(symbol.duration1, symbol.level1 != 0);
  }
  return edge_count;
}

static bool time_within(uint32_t a, uint32_t b, uint32_t tolerance) {
  const int32_t delta = static_cast<int32_t>(a - b);
  return delta >= -static_cast<int32_t>(tolerance) &&
         delta <= static_cast<int32_t>(tolerance);
}

static uint16_t repair_missing_scl_edges(volatile Sample *data, uint16_t &count,
                                         uint8_t initial_value, Capture &capture) {
  if (!rmt_available || !rmt_enabled || !rmt_receive_done || count == 0) return 0;

  SclEdge *gpio_edges = gpio_scl_edges_work;
  SclEdge *rmt_edges = rmt_scl_edges_work;
  const uint16_t gpio_total = collect_gpio_scl_edges(
      data, count, initial_value, gpio_edges, MAX_RMT_EDGES);
  const uint16_t rmt_total = collect_rmt_scl_edges(rmt_edges, MAX_RMT_EDGES);
  capture.gpio_scl_edges = gpio_total;
  capture.rmt_scl_edges = rmt_total;

  if (gpio_total == 0 || rmt_total == 0 ||
      gpio_total > MAX_RMT_EDGES || rmt_total > MAX_RMT_EDGES)
    return 0;

  // RMT starts from the first GPIO state change. If that first change itself
  // was an SCL edge, the edge can precede the point at which the RMT engine is
  // fully armed. Find an early matching edge pair and align from there rather
  // than requiring edge zero to match edge zero.
  uint16_t gpio_index = 0;
  uint16_t rmt_index = 0;
  bool aligned = false;
  const uint16_t gpio_search = gpio_total < 4 ? gpio_total : 4;
  const uint16_t rmt_search = rmt_total < 4 ? rmt_total : 4;
  for (uint16_t g = 0; g < gpio_search && !aligned; g++) {
    for (uint16_t r = 0; r < rmt_search; r++) {
      if (gpio_edges[g].level == rmt_edges[r].level &&
          time_within(gpio_edges[g].t, rmt_edges[r].t, RMT_EDGE_MATCH_TOLERANCE_US)) {
        gpio_index = g;
        rmt_index = r;
        aligned = true;
        break;
      }
    }
  }
  if (!aligned) return 0;

  capture.rmt_used = true;
  SclEdge *missing = rmt_missing_edges_work;
  uint16_t missing_count = 0;

  while (rmt_index < rmt_total) {
    const uint32_t expected = rmt_edges[rmt_index].t;
    const bool expected_level = rmt_edges[rmt_index].level;

    if (gpio_index < gpio_total &&
        gpio_edges[gpio_index].level == expected_level &&
        time_within(gpio_edges[gpio_index].t, expected, RMT_EDGE_MATCH_TOLERANCE_US)) {
      gpio_index++;
      rmt_index++;
      continue;
    }

    if (gpio_index < gpio_total) {
      const int32_t delta = static_cast<int32_t>(gpio_edges[gpio_index].t - expected);
      if (delta < -static_cast<int32_t>(RMT_EDGE_MATCH_TOLERANCE_US)) {
        // After alignment, GPIO seeing an edge which RMT did not means the two
        // timelines are no longer trustworthy enough for synthesis.
        capture.rmt_used = false;
        return 0;
      }
      if (delta <= static_cast<int32_t>(RMT_EDGE_MATCH_TOLERANCE_US) &&
          gpio_edges[gpio_index].level != expected_level) {
        capture.rmt_used = false;
        return 0;
      }
    }

    // RMT saw an edge before the next GPIO edge: preserve the SDA state from
    // GPIO and synthesize only the missing SCL transition.
    const uint32_t last_sample_t = data[count - 1].t;
    if (static_cast<int32_t>(expected - last_sample_t) >
        static_cast<int32_t>(RMT_EDGE_MATCH_TOLERANCE_US))
      break;
    if (missing_count >= MAX_RMT_REPAIRS || count + missing_count >= MAX_SAMPLES) {
      capture.rmt_used = false;
      return 0;
    }
    missing[missing_count].t = expected;
    missing[missing_count].level = expected_level;
    missing_count++;
    rmt_index++;
  }

  // Unmatched trailing GPIO edges after an established alignment mean RMT did
  // not provide a strict reference timeline. Do not modify the raw capture.
  if (gpio_index != gpio_total) {
    capture.rmt_used = false;
    return 0;
  }

  if (missing_count == 0) return 0;

  uint16_t repaired = 0;
  for (uint16_t m = 0; m < missing_count; m++) {
    uint16_t pos = 0;
    while (pos < count && static_cast<int32_t>(data[pos].t - missing[m].t) < 0) pos++;

    const uint8_t before = pos == 0 ? initial_value : data[pos - 1].value;
    if (scl_level(before) == missing[m].level) continue;

    for (uint16_t j = count; j > pos; j--) {
      data[j].t = data[j - 1].t;
      data[j].value = data[j - 1].value;
    }
    data[pos].t = missing[m].t;
    data[pos].value = static_cast<uint8_t>((before & 0x02U) |
                                           (missing[m].level ? 0x01U : 0x00U));
    count++;
    repaired++;
  }

  capture.rmt_repaired_edges = repaired;
  return repaired;
}

#endif

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
    const bool scl_rise = !prev_scl && cur_scl;
    const bool sda_changed = prev_sda != cur_sda;

    // A delayed shared GPIO ISR can observe an SDA transition and SCL rising
    // edge in one state sample even though SDA actually changed first while
    // SCL was still low. START/STOP are only legal at byte boundaries. In the
    // middle of a byte (including its ACK bit), therefore interpret a combined
    // SCL-rise + SDA-change as data setup followed by the clock edge.
    const bool coalesced_data_edge =
        active && scl_rise && sda_changed && bit_count != 0;
    if (coalesced_data_edge) capture.coalesced_edges_resolved++;

    if (!coalesced_data_edge && prev_sda && !cur_sda && cur_scl) {  // START / repeated START
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

    if (!coalesced_data_edge && active && !prev_sda && cur_sda && cur_scl) {  // STOP
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

    if (active && scl_rise) {
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

static void release_frozen_capture_after_success(uint32_t sequence, bool was_frozen) {
  if (!was_frozen || !last_capture_mutex) return;
  bool released = false;
  if (xSemaphoreTake(last_capture_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    if (last_capture_frozen && last_capture_sequence == sequence) {
      last_capture_frozen = false;
      released = true;
    }
    xSemaphoreGive(last_capture_mutex);
  }
  if (released) {
    ESP_LOGD(TAG, "Released frozen raw I2C capture #%lu after successful /capture download",
             static_cast<unsigned long>(sequence));
  }
}

struct CaptureSendContext {
  httpd_req_t *request{nullptr};
  std::string output;
  uint32_t sequence{0};
  bool was_frozen{false};
};

static void capture_send_task(void *arg) {
  auto *ctx = static_cast<CaptureSendContext *>(arg);
  httpd_req_t *req = ctx->request;

  httpd_resp_set_status(req, "200 OK");
  httpd_resp_set_type(req, "application/octet-stream");
  httpd_resp_set_hdr(req, "Content-Disposition", "attachment; filename=\"capture.la\"");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  httpd_resp_set_hdr(req, "Connection", "close");

  // httpd_resp_send() already loops over partial socket writes internally. Run
  // it from an asynchronous request task so a slow or stalled client can never
  // occupy ESP-IDF's main HTTP server task.
  const esp_err_t err = httpd_resp_send(req, ctx->output.data(), ctx->output.size());
  if (err == ESP_OK) {
    release_frozen_capture_after_success(ctx->sequence, ctx->was_frozen);
  } else {
    ESP_LOGW(TAG, "/capture send failed (%d/0x%04X); raw I2C capture #%lu preserved for retry",
             static_cast<int>(err), static_cast<unsigned>(err),
             static_cast<unsigned long>(ctx->sequence));
  }

  httpd_req_async_handler_complete(req);
  delete ctx;
  vTaskDelete(nullptr);
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
    bool was_frozen = false;
    uint32_t sequence = 0;
    if (last_capture_mutex &&
        xSemaphoreTake(last_capture_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      output = last_capture_data;
      sequence = last_capture_sequence;
      was_frozen = last_capture_frozen && !output.empty();
      xSemaphoreGive(last_capture_mutex);
    }
    if (output.empty()) {
      request->send(204, "text/plain", nullptr);
      return;
    }

    auto *ctx = new (std::nothrow) CaptureSendContext;
    if (ctx == nullptr) {
      request->send(503, "text/plain", "capture sender allocation failed");
      return;
    }
    ctx->output = std::move(output);
    ctx->sequence = sequence;
    ctx->was_frozen = was_frozen;

    httpd_req_t *req = *request;
    httpd_req_t *async_req = nullptr;
    const esp_err_t async_err = httpd_req_async_handler_begin(req, &async_req);
    if (async_err != ESP_OK) {
      ESP_LOGW(TAG, "/capture async begin failed (%d/0x%04X)",
               static_cast<int>(async_err), static_cast<unsigned>(async_err));
      delete ctx;
      request->send(503, "text/plain", "capture sender busy");
      return;
    }
    ctx->request = async_req;

    ESP_LOGD(TAG, "Starting async /capture download #%lu (%u bytes)",
             static_cast<unsigned long>(sequence),
             static_cast<unsigned>(ctx->output.size()));
    if (xTaskCreate(capture_send_task, "i2c_capture_http", 4096, ctx, 2, nullptr) != pdPASS) {
      ESP_LOGW(TAG, "/capture sender task creation failed; capture #%lu preserved",
               static_cast<unsigned long>(sequence));
      httpd_resp_send_err(async_req, HTTPD_500_INTERNAL_SERVER_ERROR,
                          "capture sender task creation failed");
      httpd_req_async_handler_complete(async_req);
      delete ctx;
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
  if (err != ESP_OK) return false;

#if CO2_MONITOR_0601_RMT_SCL_ASSIST
  if (setup_rmt_scl()) {
    ESP_LOGW(TAG, "Experimental RMT SCL assist enabled: 1 us resolution, GPIO fallback retained");
  } else {
    ESP_LOGW(TAG, "RMT SCL assist unavailable; continuing with GPIO-only I2C capture");
  }
#else
  ESP_LOGI(TAG, "GPIO-only I2C capture active (RMT SCL assist disabled)");
#endif
  return true;
}

#if CO2_MONITOR_0601_RMT_SCL_ASSIST
static void disable_rmt_capture() {
  if (!rmt_available || !rmt_enabled || rmt_scl_channel == nullptr) return;
  // Prevent any ISR-side arm while the channel is being disabled.
  rmt_enabled = false;
  const esp_err_t err = rmt_disable(rmt_scl_channel);
  if (err != ESP_OK)
    ESP_LOGW(TAG, "rmt_disable(SCL) failed: %d", static_cast<int>(err));
  rmt_receive_started = false;
  rmt_receive_done = false;
  rmt_received_symbols = 0;
  rmt_receive_start_us = 0;
  rmt_arm_error = ESP_OK;
}

static void enable_rmt_capture() {
  if (!rmt_available || rmt_enabled || rmt_scl_channel == nullptr) return;
  const esp_err_t err = rmt_enable(rmt_scl_channel);
  if (err == ESP_OK) {
    rmt_enabled = true;
    rmt_receive_started = false;
    rmt_receive_done = false;
    rmt_received_symbols = 0;
    rmt_receive_start_us = 0;
    rmt_arm_error = ESP_OK;
    return;
  }
  ESP_LOGW(TAG, "RMT SCL capture restart failed (%d); falling back to GPIO",
           static_cast<int>(err));
  rmt_enabled = false;
  rmt_available = false;
}

#endif

void set_capture_enabled(bool enabled) {
  if (capture_enabled == enabled) return;

  // Stop GPIO edge capture while resetting shared ISR state so no half-frame
  // can leak across a sleep/awake boundary. The optional experimental RMT
  // helper follows the same policy when compiled in.
  gpio_intr_disable(pin_scl);
  gpio_intr_disable(pin_sda);
#if CO2_MONITOR_0601_RMT_SCL_ASSIST
  if (!enabled) disable_rmt_capture();
#endif
  capture_enabled = enabled;
  capturing = false;
  sample_count = 0;
  capture_finished = false;
  capture_overflow = false;
  last_value = read_gpio_state();
  capture_initial_value = last_value;
  last_edge = static_cast<uint32_t>(esp_timer_get_time());
#if CO2_MONITOR_0601_RMT_SCL_ASSIST
  if (enabled) enable_rmt_capture();
#endif
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

#if CO2_MONITOR_0601_RMT_SCL_ASSIST
  // When RMT was armed by the first GPIO edge, wait briefly for its own 5 ms
  // idle detector to close the same transaction. Never wait indefinitely: a
  // failed hardware arm must degrade to the original GPIO-only decoder.
  if (rmt_available && rmt_enabled && rmt_receive_started && !rmt_receive_done) {
    const uint32_t now = static_cast<uint32_t>(esp_timer_get_time());
    if (static_cast<uint32_t>(now - last_edge) < RMT_WAIT_TIMEOUT_US) return false;
    ESP_LOGW(TAG, "RMT SCL receive did not finish after bus idle; resetting assist");
    rmt_enabled = false;
    (void) rmt_disable(rmt_scl_channel);
    const esp_err_t reenable_err = rmt_enable(rmt_scl_channel);
    if (reenable_err == ESP_OK) {
      rmt_enabled = true;
    } else {
      rmt_available = false;
      ESP_LOGW(TAG, "RMT SCL re-enable failed (%d); falling back to GPIO",
               static_cast<int>(reenable_err));
    }
    rmt_receive_started = false;
    rmt_receive_done = false;
    rmt_received_symbols = 0;
  }

#endif

  uint16_t count = sample_count;
  if (count > MAX_SAMPLES) count = MAX_SAMPLES;
  const bool overflow = capture_overflow;
  const uint8_t initial_value = capture_initial_value;
  capture = Capture{};

  if (count) {
#if RTRH_DEBUG_CAPTURE
    // Preserve the unmodified GPIO waveform. If RMT later repairs SCL edges, a
    // frozen /capture still shows the original failure rather than hiding it.
    capture.debug_raw_sequence = store_raw_capture(samples, count, initial_value, overflow);
#endif
    if (overflow) {
      capture.frame_errors++;
    } else {
#if CO2_MONITOR_0601_RMT_SCL_ASSIST
      repair_missing_scl_edges(samples, count, initial_value, capture);
#endif
      decode_capture(samples, count, initial_value, capture);
    }
  }

  sample_count = 0;
  capture_overflow = false;
  capture_finished = false;
  last_value = read_gpio_state();
  capture_initial_value = last_value;
  last_edge = static_cast<uint32_t>(esp_timer_get_time());

#if CO2_MONITOR_0601_RMT_SCL_ASSIST
  if (rmt_arm_error != ESP_OK) {
    ESP_LOGW(TAG, "RMT SCL arm failed (%d); GPIO-only capture used",
             static_cast<int>(rmt_arm_error));
  }
  // Leave the enabled RMT channel idle. The next GPIO capture's first edge
  // starts the next RX job, keeping RMT synchronized with the actual bus burst.
  rmt_receive_started = false;
  rmt_receive_done = false;
  rmt_received_symbols = 0;
  rmt_receive_start_us = 0;
  rmt_arm_error = ESP_OK;
#endif
  capturing = true;
  return true;
}

}  // namespace i2c_sniffer
}  // namespace co2_monitor_0601
}  // namespace esphome
