// SPDX-FileCopyrightText: 2026 The esphome-unni-co2-sniffer contributors
// SPDX-License-Identifier: GPL-3.0-or-later
#include "i2c_sniffer.h"
#include "debug_udp.h"
#include "power_save.h"

#include "esphome/core/log.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "soc/gpio_reg.h"
#include "soc/soc.h"
#if defined(CONFIG_IDF_TARGET_ESP32C3) || defined(CONFIG_IDF_TARGET_ESP32C6)
#include "driver/rmt_rx.h"
#include "soc/soc_caps.h"
#endif

#include <algorithm>
#include <cstddef>
#include <cstdlib>
#include <functional>
#include <cstdio>
#include <cstring>

#if RTRH_DEBUG_CAPTURE
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <string>
#include <utility>
#if defined(USE_WEB_SERVER_BASE)
#include "esphome/components/web_server_base/web_server_base.h"
#include "esp_http_server.h"
#endif
#endif

namespace esphome {
namespace co2_monitor_0601 {
namespace i2c_sniffer {

static const char *TAG = "i2c_sniffer";
static gpio_num_t pin_scl = GPIO_NUM_7;
static gpio_num_t pin_sda = GPIO_NUM_6;
static uint32_t pin_scl_mask = 1U << 7;
static uint32_t pin_sda_mask = 1U << 6;
static constexpr uint16_t MAX_SAMPLES = 4096;
static constexpr uint32_t CAPTURE_TIMEOUT_US = 5000;

struct EdgeBuffer {
  // Structure-of-arrays keeps timestamps naturally aligned while avoiding the
  // 3 bytes of padding that a {uint32_t,uint8_t} Sample needs on ESP32.
  // 4096 entries therefore use 20 KiB instead of 32 KiB without reducing the
  // capture depth or changing the LA02 debug format.
  volatile uint32_t t[MAX_SAMPLES];
  volatile uint8_t value[MAX_SAMPLES];
};

static EdgeBuffer samples;
static_assert(sizeof(EdgeBuffer) == MAX_SAMPLES * (sizeof(uint32_t) + sizeof(uint8_t)),
              "EdgeBuffer unexpectedly contains padding");
static void decode_capture(const EdgeBuffer &data, uint16_t count,
                           uint8_t initial_value, Capture &capture);
static volatile uint16_t sample_count = 0;
static volatile uint32_t last_edge = 0;
static volatile uint8_t last_value = 0xff;
static volatile uint8_t capture_initial_value = 0xff;
static volatile bool capturing = true;
static bool capture_enabled = true;
static volatile bool co2_warmup_activity_watch = false;
static volatile bool capture_finished = false;
static volatile bool capture_overflow = false;

#if defined(CONFIG_IDF_TARGET_ESP32C3) || defined(CONFIG_IDF_TARGET_ESP32C6)
// RMT-SCL assist does not replace the GPIO stream. It records the clock line in
// hardware and is consulted only when strict protocol validation rejects a GPIO
// capture. This avoids the unsynchronised-timebase problem of dual RX-RMT while
// still protecting the line on which we empirically lose most edges under BLE
// load. C3/C6 have 48 RMT words/channel and RX ping-pong support.
static constexpr uint32_t RMT_SCL_RESOLUTION_HZ = 5000000U;  // 0.2 us/tick
static constexpr size_t RMT_RX_USER_SYMBOLS = SOC_RMT_MEM_WORDS_PER_CHANNEL;
static constexpr size_t RMT_SCL_ACCUM_SYMBOLS = 128;
static constexpr size_t RMT_SCL_MAX_EDGES = RMT_SCL_ACCUM_SYMBOLS * 2U + 1U;
static constexpr uint32_t RMT_ALIGN_TOLERANCE_US = 12U;
static constexpr uint8_t RMT_MAX_REPAIR_EDGES = 16U;

static bool rmt_scl_assist_enabled = false;
static rmt_channel_handle_t rmt_scl_channel = nullptr;
static bool rmt_scl_channel_enabled = false;
static rmt_symbol_word_t rmt_scl_user_buffer[RMT_RX_USER_SYMBOLS];
static rmt_symbol_word_t rmt_scl_accum[RMT_SCL_ACCUM_SYMBOLS];
static volatile size_t rmt_scl_accum_count = 0;
static volatile bool rmt_scl_done = false;
static volatile bool rmt_scl_overflow = false;
static uint32_t diag_rmt_captures = 0;
static uint32_t diag_rmt_repairs = 0;
static uint32_t diag_rmt_edges_restored = 0;
static uint32_t diag_rmt_fallbacks = 0;
static uint32_t diag_prev_rmt_captures = 0;
static uint32_t diag_prev_rmt_repairs = 0;
static uint32_t diag_prev_rmt_edges_restored = 0;
static uint32_t diag_prev_rmt_fallbacks = 0;

static bool IRAM_ATTR rmt_scl_done_callback(rmt_channel_handle_t,
                                             const rmt_rx_done_event_data_t *edata,
                                             void *) {
  size_t out = rmt_scl_accum_count;
  for (size_t i = 0; i < edata->num_symbols; ++i) {
    if (out < RMT_SCL_ACCUM_SYMBOLS) {
      rmt_scl_accum[out++] = edata->received_symbols[i];
    } else {
      rmt_scl_overflow = true;
    }
  }
  rmt_scl_accum_count = out;
  if (edata->flags.is_last) rmt_scl_done = true;
  return false;
}

static bool rmt_scl_arm_() {
  if (!rmt_scl_assist_enabled || !rmt_scl_channel || !capture_enabled) return true;
  rmt_scl_accum_count = 0;
  rmt_scl_done = false;
  rmt_scl_overflow = false;
  rmt_receive_config_t rx_cfg{};
  rx_cfg.signal_range_min_ns = 400;  // reject sub-0.4-us glitches only
  rx_cfg.signal_range_max_ns = CAPTURE_TIMEOUT_US * 1000U;
#if SOC_RMT_SUPPORT_RX_PINGPONG
  rx_cfg.flags.en_partial_rx = true;
#endif
  const esp_err_t err =
      rmt_receive(rmt_scl_channel, rmt_scl_user_buffer, sizeof(rmt_scl_user_buffer), &rx_cfg);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "RMT SCL receive arm failed: %s", esp_err_to_name(err));
    return false;
  }
  return true;
}

static void rmt_scl_suspend_() {
  if (!rmt_scl_assist_enabled || !rmt_scl_channel) return;
  if (rmt_scl_channel_enabled) {
    const esp_err_t err = rmt_disable(rmt_scl_channel);
    if (err == ESP_OK || err == ESP_ERR_INVALID_STATE) {
      rmt_scl_channel_enabled = false;
    } else {
      ESP_LOGW(TAG, "RMT SCL disable failed: %s", esp_err_to_name(err));
    }
  }
  rmt_scl_done = false;
  rmt_scl_accum_count = 0;
  rmt_scl_overflow = false;
}

static bool rmt_scl_resume_() {
  if (!rmt_scl_assist_enabled || !rmt_scl_channel) return true;
  if (!rmt_scl_channel_enabled) {
    const esp_err_t err = rmt_enable(rmt_scl_channel);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
      ESP_LOGW(TAG, "RMT SCL enable failed: %s", esp_err_to_name(err));
      return false;
    }
    rmt_scl_channel_enabled = true;
  }
  return rmt_scl_arm_();
}
#endif

// Low-overhead edge diagnostics used to distinguish decoder failures from an
// electrically quiet or misconfigured passive I2C tap. Counters are updated
// only inside the GPIO ISR; reporting happens from the main loop.
static volatile uint32_t diag_isr_calls = 0;
// State/SCL/SDA counts are reconstructed in the main loop from the captured
// edge stream. Keeping these out of the ISR removes three volatile read/modify/
// write operations from every real bus edge without losing diagnostic data.
static uint32_t diag_state_changes = 0;
static uint32_t diag_scl_changes = 0;
static uint32_t diag_sda_changes = 0;
static volatile uint32_t diag_overflows = 0;
static uint32_t diag_completed_captures = 0;
static uint32_t diag_last_report_ms = 0;
static uint32_t diag_prev_isr_calls = 0;
static uint32_t diag_prev_state_changes = 0;
static uint32_t diag_prev_scl_changes = 0;
static uint32_t diag_prev_sda_changes = 0;
static uint32_t diag_prev_completed_captures = 0;


static inline uint8_t IRAM_ATTR read_gpio_state() {
#if defined(CONFIG_IDF_TARGET_ESP32C3) || defined(CONFIG_IDF_TARGET_ESP32C6)
  // Both Unni tap GPIOs live in the low 32-bit GPIO bank on C3/C6. Read that
  // bank once so SDA/SCL are captured atomically and with substantially less
  // ISR work than two gpio_get_level() calls. Besides reducing handler time,
  // this avoids synthesizing an impossible mixed bus state if one line changes
  // between two independent GPIO reads.
  const uint32_t levels = REG_READ(GPIO_IN_REG);
  return static_cast<uint8_t>(((levels & pin_scl_mask) ? 0x01U : 0U) |
                              ((levels & pin_sda_mask) ? 0x02U : 0U));
#else
  // Portable fallback for other ESP32 families.
  uint8_t value = 0;
  if (gpio_get_level(pin_scl)) value |= 0x01;
  if (gpio_get_level(pin_sda)) value |= 0x02;
  return value;
#endif
}
static inline bool scl_level(uint8_t value) { return (value & 0x01) != 0; }
static inline bool sda_level(uint8_t value) { return (value & 0x02) != 0; }

struct EdgeDiagCounts {
  uint32_t state{0};
  uint32_t scl{0};
  uint32_t sda{0};
};

static EdgeDiagCounts calculate_edge_diagnostics(const EdgeBuffer &data, uint16_t count,
                                                  uint8_t initial_value) {
  EdgeDiagCounts result;
  uint8_t previous = initial_value;
  for (uint16_t i = 0; i < count; ++i) {
    const uint8_t current = data.value[i];
    const uint8_t changed = static_cast<uint8_t>(current ^ previous);
    if (changed == 0) continue;
    result.state++;
    if (changed & 0x01U) result.scl++;
    if (changed & 0x02U) result.sda++;
    previous = current;
  }
  return result;
}

static void account_edge_diagnostics(const EdgeDiagCounts &delta) {
  diag_state_changes += delta.state;
  diag_scl_changes += delta.scl;
  diag_sda_changes += delta.sda;
}

static void account_edge_diagnostics(const EdgeBuffer &data, uint16_t count,
                                     uint8_t initial_value) {
  account_edge_diagnostics(calculate_edge_diagnostics(data, count, initial_value));
}


#if defined(CONFIG_IDF_TARGET_ESP32C3) || defined(CONFIG_IDF_TARGET_ESP32C6)
struct RmtSclEdge {
  uint32_t rel_ticks{0};
  uint8_t level{0};
};

static size_t build_rmt_scl_edges_(RmtSclEdge *out, size_t capacity) {
  if (!out || capacity == 0 || rmt_scl_accum_count == 0 || rmt_scl_overflow) return 0;
  uint32_t t = 0;
  bool have_level = false;
  uint8_t last_level = 0;
  size_t count = 0;
  const size_t symbols = std::min(static_cast<size_t>(rmt_scl_accum_count), RMT_SCL_ACCUM_SYMBOLS);
  for (size_t i = 0; i < symbols; ++i) {
    const rmt_symbol_word_t symbol = rmt_scl_accum[i];
    const uint16_t durations[2] = {static_cast<uint16_t>(symbol.duration0),
                                   static_cast<uint16_t>(symbol.duration1)};
    const uint8_t levels[2] = {static_cast<uint8_t>(symbol.level0), static_cast<uint8_t>(symbol.level1)};
    for (uint8_t half = 0; half < 2; ++half) {
      const uint16_t duration = durations[half];
      if (duration == 0) continue;
      if (!have_level || levels[half] != last_level) {
        if (count >= capacity) return 0;
        out[count++] = RmtSclEdge{t, levels[half]};
        last_level = levels[half];
        have_level = true;
      }
      t += duration;
    }
  }
  return count;
}

static inline uint32_t rmt_ticks_to_us_(uint32_t ticks) {
  // Rounded conversion; at 5 MHz each tick is 0.2 us.
  return static_cast<uint32_t>((static_cast<uint64_t>(ticks) * 1000000ULL +
                                RMT_SCL_RESOLUTION_HZ / 2U) /
                               RMT_SCL_RESOLUTION_HZ);
}

static bool insert_sample_(EdgeBuffer &data, uint16_t &count, uint32_t t, uint8_t value,
                           uint16_t *inserted_index) {
  if (count >= MAX_SAMPLES) return false;
  uint16_t pos = 0;
  while (pos < count && static_cast<int32_t>(data.t[pos] - t) <= 0) ++pos;
  for (uint16_t i = count; i > pos; --i) {
    data.t[i] = data.t[i - 1];
    data.value[i] = data.value[i - 1];
  }
  data.t[pos] = t;
  data.value[pos] = value;
  count++;
  if (inserted_index) *inserted_index = pos;
  return true;
}

static void remove_sample_(EdgeBuffer &data, uint16_t &count, uint16_t pos) {
  if (pos >= count) return;
  for (uint16_t i = pos; i + 1U < count; ++i) {
    data.t[i] = data.t[i + 1U];
    data.value[i] = data.value[i + 1U];
  }
  count--;
}

static bool try_rmt_scl_repair_(EdgeBuffer &data, uint16_t &count, uint8_t initial_value,
                                CaptureValidator validator, Capture &recovered) {
  if (!rmt_scl_assist_enabled || !validator || !rmt_scl_done || rmt_scl_overflow) return false;

  static RmtSclEdge rmt_edges[RMT_SCL_MAX_EDGES];
  const size_t rmt_count = build_rmt_scl_edges_(rmt_edges, RMT_SCL_MAX_EDGES);
  if (rmt_count < 16) return false;

  struct GpioSclEdge {
    uint32_t t;
    uint8_t level;
  };
  static GpioSclEdge gpio_edges[512];
  size_t gpio_count = 0;
  uint8_t previous = initial_value;
  for (uint16_t i = 0; i < count && gpio_count < 256; ++i) {
    const uint8_t current = data.value[i];
    if ((current ^ previous) & 0x01U)
      gpio_edges[gpio_count++] = GpioSclEdge{data.t[i], static_cast<uint8_t>(scl_level(current))};
    previous = current;
  }
  if (gpio_count < 12) return false;

  // RMT RX has no shared absolute epoch with the GPIO timer. Find the epoch by
  // fitting the first few hardware edges to the first few GPIO-observed SCL
  // edges. Scoring all observed clocks prevents a single delayed ISR from
  // deciding the alignment. Restricting the search to the beginning avoids the
  // whole-cycle ambiguity of an almost periodic clock waveform.
  int64_t best_offset = 0;
  size_t best_score = 0;
  const size_t gpio_seed = std::min<size_t>(gpio_count, 6);
  const size_t rmt_seed = std::min<size_t>(rmt_count, 6);
  for (size_t gi = 0; gi < gpio_seed; ++gi) {
    for (size_t ri = 0; ri < rmt_seed; ++ri) {
      if (gpio_edges[gi].level != rmt_edges[ri].level) continue;
      const int64_t candidate = static_cast<int64_t>(gpio_edges[gi].t) -
                                static_cast<int64_t>(rmt_ticks_to_us_(rmt_edges[ri].rel_ticks));
      size_t score = 0;
      size_t rp = 0;
      for (size_t g = 0; g < gpio_count; ++g) {
        while (rp + 1 < rmt_count) {
          const int64_t here = candidate + rmt_ticks_to_us_(rmt_edges[rp].rel_ticks);
          const int64_t next = candidate + rmt_ticks_to_us_(rmt_edges[rp + 1].rel_ticks);
          if (llabs(next - static_cast<int64_t>(gpio_edges[g].t)) <
              llabs(here - static_cast<int64_t>(gpio_edges[g].t)))
            ++rp;
          else
            break;
        }
        const int64_t dt = candidate + rmt_ticks_to_us_(rmt_edges[rp].rel_ticks) -
                           static_cast<int64_t>(gpio_edges[g].t);
        if (rmt_edges[rp].level == gpio_edges[g].level &&
            llabs(dt) <= static_cast<int64_t>(RMT_ALIGN_TOLERANCE_US))
          ++score;
      }
      if (score > best_score || (score == best_score && ri < 2)) {
        best_score = score;
        best_offset = candidate;
      }
    }
  }
  if (best_score * 100U < gpio_count * 80U) return false;

  uint16_t inserted_positions[RMT_MAX_REPAIR_EDGES]{};
  uint8_t inserted = 0;
  for (size_t ri = 0; ri < rmt_count; ++ri) {
    const int64_t abs64 = best_offset + static_cast<int64_t>(rmt_ticks_to_us_(rmt_edges[ri].rel_ticks));
    if (abs64 < 0 || abs64 > 0xFFFFFFFFLL) continue;
    const uint32_t abs_t = static_cast<uint32_t>(abs64);
    bool present = false;
    for (size_t gi = 0; gi < gpio_count; ++gi) {
      if (gpio_edges[gi].level != rmt_edges[ri].level) continue;
      const int64_t dt = static_cast<int64_t>(gpio_edges[gi].t) - static_cast<int64_t>(abs_t);
      if (llabs(dt) <= static_cast<int64_t>(RMT_ALIGN_TOLERANCE_US)) {
        present = true;
        break;
      }
    }
    if (present) continue;
    if (inserted >= RMT_MAX_REPAIR_EDGES) {
      for (int i = inserted - 1; i >= 0; --i) remove_sample_(data, count, inserted_positions[i]);
      return false;
    }

    // Preserve SDA from the latest known GPIO state at this instant and replace
    // only the hardware-known SCL level.
    uint8_t bus = initial_value;
    uint16_t pos = 0;
    while (pos < count && static_cast<int32_t>(data.t[pos] - abs_t) <= 0) {
      bus = data.value[pos];
      ++pos;
    }
    bus = static_cast<uint8_t>((bus & ~0x01U) | (rmt_edges[ri].level ? 0x01U : 0U));
    if (scl_level(bus) == scl_level(pos ? data.value[pos - 1] : initial_value)) continue;

    uint16_t inserted_pos = 0;
    if (!insert_sample_(data, count, abs_t, bus, &inserted_pos)) {
      for (int i = inserted - 1; i >= 0; --i) remove_sample_(data, count, inserted_positions[i]);
      return false;
    }
    // Earlier insertions shift later positions. Keep positions sorted and update
    // prior bookkeeping so rollback remains exact.
    for (uint8_t i = 0; i < inserted; ++i)
      if (inserted_positions[i] >= inserted_pos) inserted_positions[i]++;
    inserted_positions[inserted++] = inserted_pos;
  }

  if (inserted == 0) return false;
  Capture candidate{};
  decode_capture(data, count, initial_value, candidate);
  if (!validator(candidate)) {
    // Roll back in descending index order.
    std::sort(inserted_positions, inserted_positions + inserted, std::greater<uint16_t>());
    for (uint8_t i = 0; i < inserted; ++i) remove_sample_(data, count, inserted_positions[i]);
    return false;
  }

  candidate.rmt_scl_edges_recovered = inserted;
  recovered = candidate;
  // The RMT repair is decoder-side only. Restore the original GPIO waveform so
  // diagnostics and /capture/UDP always expose what the ISR actually observed.
  std::sort(inserted_positions, inserted_positions + inserted, std::greater<uint16_t>());
  for (uint8_t i = 0; i < inserted; ++i) remove_sample_(data, count, inserted_positions[i]);
  diag_rmt_repairs++;
  diag_rmt_edges_restored += inserted;
  return true;
}
#endif

static void IRAM_ATTR gpio_isr(void *) {
  if (!capturing) {
    diag_isr_calls++;
    return;
  }

  // Sample the bus before doing timestamp or diagnostic work. The precise
  // timestamp may move by a few CPU cycles, but reading SDA/SCL as early as
  // possible reduces the window in which a second physical edge can collapse
  // into the same observed GPIO state.
  const uint8_t value = read_gpio_state();
  diag_isr_calls++;
  const uint8_t previous_value = last_value;
  // GPIO ANYEDGE can occasionally deliver an ISR after another edge has
  // already returned the pins to the state we last stored. Do not pay for the
  // 64-bit esp_timer read in that case; there is no sample to timestamp.
  if (value == previous_value) return;

  // The extra cross-component ISR call is armed only during the CO2 warm-up;
  // normal high-rate I2C capture pays only this predictable branch.
  if (co2_warmup_activity_watch) power_save::on_co2_edge_from_isr();

  const uint32_t now = static_cast<uint32_t>(esp_timer_get_time());
  if (sample_count == 0) capture_initial_value = previous_value;
  last_value = value;
  last_edge = now;

  const uint16_t index = sample_count;
  if (index < MAX_SAMPLES) {
    samples.t[index] = now;
    samples.value[index] = value;
    sample_count = index + 1;
  } else {
    capture_overflow = true;
    diag_overflows++;
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

static void decode_capture(const EdgeBuffer &data, uint16_t count,
                           uint8_t initial_value, Capture &capture) {
  if (count == 0) return;

  bool active = false;
  uint8_t current_byte = 0;
  uint8_t bit_count = 0;
  RawFrame raw;
  clear_raw_frame(raw);
  uint8_t previous = initial_value;

  for (uint16_t i = 0; i < count; i++) {
    const uint8_t current = data.value[i];
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

// Missing-clock recovery is based on unusually long intervals during which
// the captured SCL level never changes. SDA-only samples inside such an
// interval must not split the timing gap: the shared GPIO ISR can miss a full
// SCL pulse while still observing one or more SDA transitions.
//
// Timing only proposes candidates. A reconstructed capture is accepted solely
// when the caller-supplied protocol validator accepts it. Up to two complete
// SCL pulses are considered because real field captures have shown both one-
// and two-pulse losses on the single-core ESP32-C3.
static constexpr uint8_t MAX_RECOVERY_PULSE_CANDIDATES = 32;
static constexpr uint8_t MAX_RECOVERY_SDA_EVENTS = 4;
static constexpr uint32_t RECOVERY_MIN_LEVEL_US = 55;
static constexpr uint32_t RECOVERY_MAX_LEVEL_US = 300;

struct LevelTiming {
  uint32_t low_us{0};
  uint32_t high_us{0};
};

struct PulseCandidate {
  uint32_t t1{0};
  uint32_t t2{0};
  uint32_t interval_us{0};
  uint8_t interval_id{0};
};

static uint32_t median_duration(uint32_t *values, uint8_t count) {
  if (count == 0) return 0;
  for (uint8_t i = 1; i < count; i++) {
    const uint32_t value = values[i];
    uint8_t j = i;
    while (j > 0 && values[j - 1] > value) {
      values[j] = values[j - 1];
      j--;
    }
    values[j] = value;
  }
  return values[count / 2];
}

// Estimate normal HIGH and LOW pulse widths independently. Durations above
// 60 us are deliberately excluded from the baseline because those are the
// intervals we may later investigate for a completely missed opposite pulse.
static bool estimate_scl_level_timing(const EdgeBuffer &data, uint16_t count,
                                      uint8_t initial_value, LevelTiming &timing) {
  uint32_t low[96]{};
  uint32_t high[96]{};
  uint8_t low_count = 0;
  uint8_t high_count = 0;
  bool level = scl_level(initial_value);
  uint32_t interval_start = count ? data.t[0] : 0;

  for (uint16_t i = 0; i < count; i++) {
    const bool next_level = scl_level(data.value[i]);
    if (next_level == level) continue;

    const uint32_t duration = static_cast<uint32_t>(data.t[i] - interval_start);
    if (duration >= 10 && duration <= 60) {
      if (level) {
        if (high_count < 96) high[high_count++] = duration;
      } else {
        if (low_count < 96) low[low_count++] = duration;
      }
    }
    interval_start = data.t[i];
    level = next_level;
  }

  if (low_count < 8 || high_count < 8) return false;
  timing.low_us = median_duration(low, low_count);
  timing.high_us = median_duration(high, high_count);
  return timing.low_us != 0 && timing.high_us != 0;
}

static bool add_pulse_candidate(PulseCandidate *out, uint8_t &count,
                                uint8_t interval_id, uint32_t interval_us,
                                uint32_t t1, uint32_t t2,
                                uint32_t opposite_level_us) {
  if (t2 <= t1) return true;
  const uint32_t pulse_us = static_cast<uint32_t>(t2 - t1);
  const uint32_t min_pulse = opposite_level_us / 2U > 4U
                                 ? opposite_level_us / 2U : 4U;
  const uint32_t max_pulse = opposite_level_us * 2U + 5U;
  if (pulse_us < min_pulse || pulse_us > max_pulse) return true;
  if (count >= MAX_RECOVERY_PULSE_CANDIDATES) return false;

  out[count].t1 = t1;
  out[count].t2 = t2;
  out[count].interval_us = interval_us;
  out[count].interval_id = interval_id;
  count++;
  return true;
}

// Generate representative pulse placements based on ordering relative to SDA
// changes inside one constant-SCL interval. Exact microsecond placement is not
// used as evidence; placements that produce the same decoded protocol result
// are considered equivalent later.
static bool add_interval_candidates(PulseCandidate *out, uint8_t &count,
                                    uint8_t interval_id,
                                    uint32_t start_t, uint32_t end_t,
                                    const uint32_t *sda_times, uint8_t sda_count,
                                    uint32_t opposite_level_us) {
  uint32_t bounds[MAX_RECOVERY_SDA_EVENTS + 2]{};
  bounds[0] = start_t;
  for (uint8_t i = 0; i < sda_count; i++) bounds[i + 1] = sda_times[i];
  bounds[sda_count + 1] = end_t;
  const uint8_t gap_count = static_cast<uint8_t>(sda_count + 1);
  const uint32_t interval_us = static_cast<uint32_t>(end_t - start_t);

  for (uint8_t first = 0; first < gap_count; first++) {
    const uint32_t a1 = bounds[first];
    const uint32_t b1 = bounds[first + 1];
    const uint32_t width1 = static_cast<uint32_t>(b1 - a1);
    if (width1 < 9) continue;

    // Both synthetic transitions in the same SDA-stable slot.
    const uint32_t same_t1 = a1 + width1 / 3U;
    const uint32_t same_t2 = a1 + (width1 * 2U) / 3U;
    if (!add_pulse_candidate(out, count, interval_id, interval_us,
                             same_t1, same_t2, opposite_level_us))
      return false;

    // Or let one or more observed SDA changes happen while the missing pulse
    // is at the opposite SCL level. One midpoint per ordering is sufficient.
    for (uint8_t second = static_cast<uint8_t>(first + 1);
         second < gap_count; second++) {
      const uint32_t a2 = bounds[second];
      const uint32_t b2 = bounds[second + 1];
      if (b2 - a2 < 5) continue;
      const uint32_t t1 = a1 + width1 / 2U;
      const uint32_t t2 = a2 + (b2 - a2) / 2U;
      if (!add_pulse_candidate(out, count, interval_id, interval_us,
                               t1, t2, opposite_level_us))
        return false;
    }
  }
  return true;
}

static bool collect_pulse_candidates(const EdgeBuffer &data, uint16_t count,
                                     uint8_t initial_value,
                                     const LevelTiming &timing,
                                     PulseCandidate *out, uint8_t &out_count) {
  out_count = 0;
  if (count < 2) return true;

  bool level = scl_level(initial_value);
  uint32_t interval_start = data.t[0];
  uint32_t sda_times[MAX_RECOVERY_SDA_EVENTS]{};
  uint8_t sda_count = 0;
  uint8_t interval_id = 0;
  uint8_t previous = initial_value;

  for (uint16_t i = 0; i < count; i++) {
    const uint8_t current = data.value[i];
    const bool next_level = scl_level(current);
    const bool scl_changed = next_level != level;
    const bool sda_changed = sda_level(current) != sda_level(previous);

    if (scl_changed) {
      const uint32_t end_t = data.t[i];
      const uint32_t duration = static_cast<uint32_t>(end_t - interval_start);
      const uint32_t same_level_us = level ? timing.high_us : timing.low_us;
      const uint32_t opposite_level_us = level ? timing.low_us : timing.high_us;
      const uint32_t timing_threshold = same_level_us * 2U > 5U
                                            ? same_level_us * 2U - 5U : 0U;
      const uint32_t min_interval = timing_threshold > RECOVERY_MIN_LEVEL_US
                                        ? timing_threshold : RECOVERY_MIN_LEVEL_US;

      if (duration >= min_interval && duration <= RECOVERY_MAX_LEVEL_US) {
        if (!add_interval_candidates(out, out_count, interval_id, interval_start,
                                     end_t, sda_times, sda_count,
                                     opposite_level_us))
          return false;  // Candidate truncation would make ambiguity unknowable.
        interval_id++;
      }

      interval_start = end_t;
      level = next_level;
      sda_count = 0;
    } else if (sda_changed && sda_count < MAX_RECOVERY_SDA_EVENTS) {
      sda_times[sda_count++] = data.t[i];
    } else if (sda_changed) {
      // More SDA transitions than the bounded candidate model can represent.
      // Skip recovery rather than silently omit possible orderings.
      return false;
    }
    previous = current;
  }
  return true;
}

static bool remove_sample_at(EdgeBuffer &data, uint16_t &count,
                             uint16_t index) {
  if (index >= count) return false;
  for (uint16_t i = index; i + 1 < count; i++) {
    data.t[i] = data.t[i + 1];
    data.value[i] = data.value[i + 1];
  }
  count--;
  return true;
}

static uint16_t find_insert_position(const EdgeBuffer &data, uint16_t count,
                                     uint32_t t) {
  uint16_t pos = 0;
  while (pos < count && static_cast<int32_t>(data.t[pos] - t) < 0) pos++;
  return pos;
}

// Apply one synthetic opposite-level SCL pulse in-place. Existing SDA-only
// samples between t1/t2 keep their SDA value but inherit the synthetic SCL
// level. poll() has paused ISR writes, so this scratch transformation is safe.
static bool apply_pulse(EdgeBuffer &data, uint16_t &count,
                        uint8_t initial_value, const PulseCandidate &pulse) {
  if (count > MAX_SAMPLES - 2 || pulse.t2 <= pulse.t1) return false;
  uint16_t pos1 = find_insert_position(data, count, pulse.t1);
  uint16_t pos2 = find_insert_position(data, count, pulse.t2);
  if ((pos1 < count && data.t[pos1] == pulse.t1) ||
      (pos2 < count && data.t[pos2] == pulse.t2))
    return false;

  const uint8_t before1 = pos1 == 0 ? initial_value : data.value[pos1 - 1];

  // Flip SCL on all already-captured SDA transitions that occurred while the
  // missing opposite-level pulse should have been active.
  for (uint16_t i = pos1; i < count && data.t[i] < pulse.t2; i++)
    data.value[i] ^= 0x01U;

  // Insert first edge.
  for (uint16_t i = count; i > pos1; i--) {
    data.t[i] = data.t[i - 1];
    data.value[i] = data.value[i - 1];
  }
  data.t[pos1] = pulse.t1;
  data.value[pos1] = static_cast<uint8_t>(before1 ^ 0x01U);
  count++;

  // Insert return edge using the current SDA state immediately before t2.
  pos2 = find_insert_position(data, count, pulse.t2);
  const uint8_t before2 = pos2 == 0 ? initial_value : data.value[pos2 - 1];
  for (uint16_t i = count; i > pos2; i--) {
    data.t[i] = data.t[i - 1];
    data.value[i] = data.value[i - 1];
  }
  data.t[pos2] = pulse.t2;
  data.value[pos2] = static_cast<uint8_t>(before2 ^ 0x01U);
  count++;
  return true;
}

static bool undo_pulse(EdgeBuffer &data, uint16_t &count,
                       const PulseCandidate &pulse) {
  uint16_t p1 = count;
  uint16_t p2 = count;
  for (uint16_t i = 0; i < count; i++) {
    if (data.t[i] == pulse.t1) p1 = i;
    if (data.t[i] == pulse.t2) p2 = i;
  }
  if (p1 == count || p2 == count) return false;
  if (p2 < p1) {
    const uint16_t tmp = p1;
    p1 = p2;
    p2 = tmp;
  }
  if (!remove_sample_at(data, count, p2)) return false;
  if (!remove_sample_at(data, count, p1)) return false;

  // Restore the original SCL bit on SDA-only samples inside the interval.
  for (uint16_t i = 0; i < count; i++) {
    if (data.t[i] > pulse.t1 && data.t[i] < pulse.t2)
      data.value[i] ^= 0x01U;
  }
  return true;
}

static bool same_frame(const Frame &a, const Frame &b) {
  if (a.address != b.address || a.direction != b.direction ||
      a.address_ack != b.address_ack || a.length != b.length ||
      a.end_condition != b.end_condition || a.status != b.status ||
      a.partial_bits != b.partial_bits)
    return false;
  for (uint8_t i = 0; i < a.length; i++) {
    if (a.data[i] != b.data[i] || a.ack[i] != b.ack[i]) return false;
  }
  return true;
}

static bool same_decoded_capture(const Capture &a, const Capture &b) {
  if (a.frame_errors != b.frame_errors || a.frame_count != b.frame_count)
    return false;
  for (uint8_t i = 0; i < a.frame_count; i++)
    if (!same_frame(a.frames[i], b.frames[i])) return false;
  return true;
}

static bool remember_valid_recovery(const Capture &candidate,
                                    const PulseCandidate *pulses,
                                    uint8_t pulse_count,
                                    bool &have_accepted, bool &ambiguous,
                                    Capture &accepted) {
  if (!have_accepted) {
    accepted = candidate;
    accepted.recovered_missing_clocks = pulse_count;
    for (uint8_t i = 0; i < pulse_count && i < 2; i++)
      accepted.recovered_gap_us[i] = pulses[i].interval_us;
    have_accepted = true;
    return true;
  }

  // Different timing placements are harmless when they decode to the exact
  // same bus transaction. Only competing protocol results make recovery
  // ambiguous and therefore unacceptable.
  if (!same_decoded_capture(accepted, candidate)) {
    ambiguous = true;
    return false;
  }
  return true;
}

static bool try_missing_clock_recovery(EdgeBuffer &data, uint16_t count,
                                       uint8_t initial_value,
                                       CaptureValidator validator,
                                       Capture &accepted) {
  if (!validator || count < 4 || count > MAX_SAMPLES - 4) return false;

  LevelTiming timing{};
  if (!estimate_scl_level_timing(data, count, initial_value, timing)) return false;

  PulseCandidate candidates[MAX_RECOVERY_PULSE_CANDIDATES]{};
  uint8_t candidate_count = 0;
  if (!collect_pulse_candidates(data, count, initial_value, timing,
                                candidates, candidate_count) ||
      candidate_count == 0)
    return false;

  bool have_accepted = false;
  bool ambiguous = false;

  // Prefer the minimum edit distance. If one missing pulse can reconstruct a
  // unique valid transaction, do not consider any two-pulse hypotheses.
  for (uint8_t i = 0; i < candidate_count; i++) {
    uint16_t work_count = count;
    if (!apply_pulse(data, work_count, initial_value, candidates[i])) continue;
    Capture candidate{};
    decode_capture(data, work_count, initial_value, candidate);
    const bool valid = validator(candidate);
    const bool restored = undo_pulse(data, work_count, candidates[i]);
    if (!restored || work_count != count) return false;
    if (!valid) continue;
    if (!remember_valid_recovery(candidate, &candidates[i], 1,
                                 have_accepted, ambiguous, accepted))
      break;
  }
  if (ambiguous) return false;
  if (have_accepted) return true;

  // Real captures have also shown two independently lost SCL pulses. Search
  // pairs from different constant-SCL intervals only; this bounds work and
  // avoids inventing multiple pulses inside one already-ambiguous interval.
  for (uint8_t i = 0; i < candidate_count; i++) {
    for (uint8_t j = static_cast<uint8_t>(i + 1); j < candidate_count; j++) {
      if (candidates[i].interval_id == candidates[j].interval_id) continue;

      const PulseCandidate *first = &candidates[i];
      const PulseCandidate *second = &candidates[j];
      if (second->t1 < first->t1) {
        const PulseCandidate *tmp = first;
        first = second;
        second = tmp;
      }
      if (first->t2 >= second->t1) continue;

      uint16_t work_count = count;
      if (!apply_pulse(data, work_count, initial_value, *first)) continue;
      if (!apply_pulse(data, work_count, initial_value, *second)) {
        (void) undo_pulse(data, work_count, *first);
        continue;
      }

      Capture candidate{};
      decode_capture(data, work_count, initial_value, candidate);
      const bool valid = validator(candidate);
      const bool restored_second = undo_pulse(data, work_count, *second);
      const bool restored_first = undo_pulse(data, work_count, *first);
      if (!restored_second || !restored_first || work_count != count) return false;
      if (!valid) continue;

      PulseCandidate used[2] = {*first, *second};
      if (!remember_valid_recovery(candidate, used, 2,
                                   have_accepted, ambiguous, accepted))
        break;
    }
    if (ambiguous) break;
  }

  return have_accepted && !ambiguous;
}

#if RTRH_DEBUG_CAPTURE
static std::string last_capture_data;
static SemaphoreHandle_t last_capture_mutex = nullptr;
static bool last_capture_frozen = false;
static uint32_t last_capture_sequence = 0;
static uint32_t next_capture_sequence = 0;

static uint8_t la02_stream_byte(const EdgeBuffer &data, uint16_t count,
                                uint8_t initial_value, bool overflow, uint32_t base,
                                size_t offset) {
  if (offset < 4U) return static_cast<uint8_t>("LA02"[offset]);
  if (offset < 8U) return static_cast<uint8_t>((static_cast<uint32_t>(count) >> ((offset - 4U) * 8U)) & 0xFFU);
  if (offset == 8U) return overflow ? 1U : 0U;
  if (offset == 9U) return initial_value;
  offset -= 10U;
  const size_t sample_index = offset / 5U;
  const size_t field_byte = offset % 5U;
  if (sample_index >= count) return 0;
  if (field_byte < 4U) {
    const uint32_t timestamp = static_cast<uint32_t>(data.t[sample_index] - base);
    return static_cast<uint8_t>((timestamp >> (field_byte * 8U)) & 0xFFU);
  }
  return data.value[sample_index];
}

struct UdpPendingCapture {
  bool active{false};
  uint16_t count{0};
  uint8_t initial_value{0xff};
  bool overflow{false};
  uint32_t sequence{0};
  uint16_t packet_index{0};
  uint16_t packet_count{0};
};
static UdpPendingCapture udp_pending;

static bool udp_export_pending_step() {
  if (!debug_udp::enabled() || !udp_pending.active || !udp_pending.count) return true;

  const size_t total_bytes = 10U + static_cast<size_t>(udp_pending.count) * 5U;
  if (udp_pending.packet_count == 0) {
    udp_pending.packet_count = static_cast<uint16_t>(
        (total_bytes + debug_udp::MAX_PAYLOAD - 1U) / debug_udp::MAX_PAYLOAD);
  }
  if (udp_pending.packet_index >= udp_pending.packet_count) return true;

  const uint32_t base = samples.t[0];
  const size_t offset = static_cast<size_t>(udp_pending.packet_index) * debug_udp::MAX_PAYLOAD;
  const uint16_t len = static_cast<uint16_t>(
      std::min<size_t>(debug_udp::MAX_PAYLOAD, total_bytes - offset));
  static uint8_t payload[debug_udp::MAX_PAYLOAD];
  for (uint16_t i = 0; i < len; i++)
    payload[i] = la02_stream_byte(samples, udp_pending.count, udp_pending.initial_value,
                                  udp_pending.overflow, base, offset + i);

  if (!debug_udp::send_packet(debug_udp::PacketType::I2C_LA02, udp_pending.sequence,
                              udp_pending.packet_index, udp_pending.packet_count,
                              payload, len, udp_pending.overflow ? 1U : 0U)) {
    if (debug_udp::sustained_resource_pressure()) {
      ESP_LOGW(TAG, "Dropping raw I2C capture #%lu after sustained UDP ENOMEM pressure",
               static_cast<unsigned long>(udp_pending.sequence));
      debug_udp::reset_after_resource_pressure();
      udp_pending.active = false;
      udp_pending.packet_index = 0;
      udp_pending.packet_count = 0;
      return true;
    }
    return false;
  }

  udp_pending.packet_index++;
  if (udp_pending.packet_index >= udp_pending.packet_count) {
    ESP_LOGD(TAG, "UDP exported raw I2C capture #%lu (%u samples, %u packets)",
             static_cast<unsigned long>(udp_pending.sequence),
             static_cast<unsigned>(udp_pending.count),
             static_cast<unsigned>(udp_pending.packet_count));
    udp_pending.active = false;
    return true;
  }
  return false;
}

static bool udp_begin_raw_capture(uint16_t count, uint8_t initial_value,
                                  bool overflow, uint32_t sequence) {
  if (!debug_udp::ready_for_export() || !count || udp_pending.active) return false;
  udp_pending.active = true;
  udp_pending.count = count;
  udp_pending.initial_value = initial_value;
  udp_pending.overflow = overflow;
  udp_pending.sequence = sequence;
  udp_pending.packet_index = 0;
  udp_pending.packet_count = 0;
  return true;
}

static uint32_t store_raw_capture(const EdgeBuffer &data, uint16_t count,
                                  uint8_t initial_value, bool overflow) {
  if (!count || !last_capture_mutex) return 0;

  if (debug_udp::enabled()) {
    if (debug_udp::ready_for_export()) {
      uint32_t sequence = 0;
      if (xSemaphoreTake(last_capture_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        sequence = ++next_capture_sequence;
        last_capture_sequence = sequence;
        xSemaphoreGive(last_capture_mutex);
      }
      if (sequence != 0) udp_begin_raw_capture(count, initial_value, overflow, sequence);
      return sequence;
    }
#if !defined(USE_WEB_SERVER_BASE)
    // UDP is configured but its circuit breaker is open. The normal debug
    // firmware has no HTTP capture endpoint, so retaining a heap-backed copy
    // here would only waste memory while the collector is unavailable.
    return 0;
#endif
  }

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
  const uint32_t base = data.t[0];
  for (uint16_t i = 0; i < count; i++) {
    const uint32_t timestamp = static_cast<uint32_t>(data.t[i] - base);
    memcpy(p, &timestamp, sizeof(timestamp)); p += 4;
    *p++ = static_cast<char>(data.value[i]);
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

#if defined(USE_WEB_SERVER_BASE)
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

    // Captures observed in practice are typically well below 1 KiB. Sending
    // them directly from ESP-IDF's HTTP server task avoids a second FreeRTOS
    // task and cross-task request ownership, both of which added unnecessary
    // scheduling pressure on the single-core ESP32-C3.
    httpd_req_t *req = *request;
    httpd_resp_set_status(req, "200 OK");
    httpd_resp_set_type(req, "application/octet-stream");
    httpd_resp_set_hdr(req, "Content-Disposition", "attachment; filename=\"capture.la\"");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");

    ESP_LOGD(TAG, "Serving /capture #%lu synchronously (%u bytes)",
             static_cast<unsigned long>(sequence),
             static_cast<unsigned>(output.size()));
    const esp_err_t err = httpd_resp_send(req, output.data(), output.size());
    if (err == ESP_OK) {
      release_frozen_capture_after_success(sequence, was_frozen);
    } else {
      ESP_LOGW(TAG, "/capture send failed (%d/0x%04X); raw I2C capture #%lu preserved for retry",
               static_cast<int>(err), static_cast<unsigned>(err),
               static_cast<unsigned long>(sequence));
    }
  }
};
static CaptureHandler capture_handler;
#endif


void register_debug_handler() {
  if (!last_capture_mutex) last_capture_mutex = xSemaphoreCreateMutex();
#if defined(USE_WEB_SERVER_BASE)
  if (web_server_base::global_web_server_base) {
    web_server_base::global_web_server_base->add_handler(&capture_handler);
  } else {
    ESP_LOGW(TAG, "web_server_base unavailable");
  }
#else
  ESP_LOGD(TAG, debug_udp::enabled() ? "Raw I2C capture instrumentation enabled with UDP export"
                                  : "Raw I2C capture instrumentation enabled without network export");
#endif
}

bool freeze_last_capture(uint32_t sequence, const char *reason) {
  if (sequence == 0) return false;
  if (debug_udp::ready_for_export()) {
    ESP_LOGD(TAG, "Suspicious raw I2C capture #%lu already exported via UDP (%s)",
             static_cast<unsigned long>(sequence), reason ? reason : "unknown reason");
    return true;
  }
  if (!last_capture_mutex) return false;

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

bool debug_export_pending() {
  return debug_udp::enabled() && udp_pending.active;
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



static void restore_passive_gpio_() {
#if defined(CONFIG_IDF_TARGET_ESP32C3) || defined(CONFIG_IDF_TARGET_ESP32C6)
  if (rmt_scl_assist_enabled) rmt_scl_suspend_();
#endif
  gpio_intr_disable(pin_scl);
  gpio_intr_disable(pin_sda);
  gpio_set_intr_type(pin_scl, GPIO_INTR_DISABLE);
  gpio_set_intr_type(pin_sda, GPIO_INTR_DISABLE);

  gpio_config_t io{};
  io.mode = GPIO_MODE_INPUT;
  io.pull_up_en = GPIO_PULLUP_DISABLE;
  io.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io.intr_type = GPIO_INTR_DISABLE;
  io.pin_bit_mask = (1ULL << pin_scl) | (1ULL << pin_sda);
  gpio_config(&io);

  capturing = false;
  sample_count = 0;
  capture_finished = false;
  capture_overflow = false;
  last_value = read_gpio_state();
  capture_initial_value = last_value;
  last_edge = static_cast<uint32_t>(esp_timer_get_time());

  gpio_set_intr_type(pin_scl, GPIO_INTR_ANYEDGE);
  gpio_set_intr_type(pin_sda, GPIO_INTR_ANYEDGE);
  capturing = capture_enabled;
  if (capture_enabled) {
    gpio_intr_enable(pin_scl);
    gpio_intr_enable(pin_sda);
#if defined(CONFIG_IDF_TARGET_ESP32C3) || defined(CONFIG_IDF_TARGET_ESP32C6)
    if (rmt_scl_assist_enabled) rmt_scl_resume_();
#endif
  }
}

static void hard_sda_output_() {
  gpio_set_direction(pin_sda, GPIO_MODE_OUTPUT);
}

static void hard_sda_input_() {
  gpio_set_direction(pin_sda, GPIO_MODE_INPUT);
  gpio_pullup_en(pin_sda);
  gpio_pulldown_dis(pin_sda);
}

static bool begin_active_master_() {
#if defined(CONFIG_IDF_TARGET_ESP32C3) || defined(CONFIG_IDF_TARGET_ESP32C6)
  if (rmt_scl_assist_enabled) rmt_scl_suspend_();
#endif
  gpio_intr_disable(pin_scl);
  gpio_intr_disable(pin_sda);
  capturing = false;
  sample_count = 0;
  capture_finished = false;
  capture_overflow = false;

  // Aggressive diagnostic mode: the user's external 10 kOhm series resistors
  // bound worst-case DC current to about 0.33 mA per line at 3.3 V. Drive the
  // ESP side HIGH push-pull so a powered-down / weakly clamped Unni bus has a
  // chance to rise. SDA is released back to input for ACK/data sampling.
  gpio_config_t io{};
  io.mode = GPIO_MODE_OUTPUT;
  io.pull_up_en = GPIO_PULLUP_DISABLE;
  io.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io.intr_type = GPIO_INTR_DISABLE;
  io.pin_bit_mask = (1ULL << pin_scl) | (1ULL << pin_sda);
  if (gpio_config(&io) != ESP_OK) {
    restore_passive_gpio_();
    return false;
  }
  gpio_set_level(pin_scl, 1);
  gpio_set_level(pin_sda, 1);
  esp_rom_delay_us(2000);
  return true;
}

static inline void i2c_delay_() { esp_rom_delay_us(50); }  // ~10 kHz, intentionally slow
static inline void drive_scl_(bool high) { gpio_set_level(pin_scl, high ? 1 : 0); }
static inline void drive_sda_(bool high) {
  hard_sda_output_();
  gpio_set_level(pin_sda, high ? 1 : 0);
}

static bool wait_scl_high_() {
  // SCL is intentionally driven push-pull HIGH through the external 10 kOhm
  // tap resistor in this diagnostic mode. Do not use the ESP-side level as a
  // claim that the Unni-side node actually reached VIH.
  drive_scl_(true);
  esp_rom_delay_us(50);
  return true;
}

static bool master_start_() {
  drive_sda_(true);
  if (!wait_scl_high_()) return false;
  i2c_delay_();
  drive_sda_(false);
  i2c_delay_();
  drive_scl_(false);
  i2c_delay_();
  return true;
}

static void master_stop_() {
  drive_sda_(false);
  i2c_delay_();
  if (!wait_scl_high_()) return;
  i2c_delay_();
  drive_sda_(true);
  i2c_delay_();
}

static bool master_write_byte_(uint8_t value) {
  for (int bit = 7; bit >= 0; --bit) {
    drive_sda_((value & (1U << bit)) != 0);
    i2c_delay_();
    if (!wait_scl_high_()) return false;
    i2c_delay_();
    drive_scl_(false);
    i2c_delay_();
  }

  // For ACK the master must stop driving SDA. Precharge it HIGH first, then
  // release it to an input with the weak internal pull-up; a live slave can
  // pull the Unni side LOW through the 10 kOhm tap and be observed here.
  drive_sda_(true);
  esp_rom_delay_us(20);
  hard_sda_input_();
  i2c_delay_();
  if (!wait_scl_high_()) return false;
  i2c_delay_();
  const bool ack = !gpio_get_level(pin_sda);
  drive_scl_(false);
  i2c_delay_();
  hard_sda_output_();
  gpio_set_level(pin_sda, 1);
  return ack;
}

static bool master_read_byte_(uint8_t &value, bool ack) {
  value = 0;
  for (uint8_t bit = 0; bit < 8; ++bit) {
    // Precharge the ESP side HIGH through 10 kOhm, then release SDA before the
    // clock edge so the slave can pull it LOW for a zero bit.
    drive_sda_(true);
    esp_rom_delay_us(20);
    hard_sda_input_();
    i2c_delay_();
    if (!wait_scl_high_()) return false;
    i2c_delay_();
    value = static_cast<uint8_t>((value << 1) | (gpio_get_level(pin_sda) ? 1U : 0U));
    drive_scl_(false);
    i2c_delay_();
  }

  drive_sda_(!ack);
  i2c_delay_();
  if (!wait_scl_high_()) return false;
  i2c_delay_();
  drive_scl_(false);
  drive_sda_(true);
  i2c_delay_();
  return true;
}

bool setup(uint8_t sda_pin, uint8_t scl_pin, bool use_rmt_scl_assist) {
  pin_sda = static_cast<gpio_num_t>(sda_pin);
  pin_scl = static_cast<gpio_num_t>(scl_pin);
#if defined(CONFIG_IDF_TARGET_ESP32C3) || defined(CONFIG_IDF_TARGET_ESP32C6)
  // C3/C6 expose all GPIOs through the low 32-bit input register. Keep the
  // optimized path explicit so other ESP32 families can still use pins >= 32
  // through the gpio_get_level() fallback without an invalid 32-bit shift.
  if (static_cast<uint32_t>(pin_scl) >= 32U || static_cast<uint32_t>(pin_sda) >= 32U) return false;
  pin_scl_mask = 1U << static_cast<uint32_t>(pin_scl);
  pin_sda_mask = 1U << static_cast<uint32_t>(pin_sda);
#endif
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
  if (err != ESP_OK) {
    gpio_isr_handler_remove(pin_scl);
    return false;
  }

#if defined(CONFIG_IDF_TARGET_ESP32C3) || defined(CONFIG_IDF_TARGET_ESP32C6)
  rmt_scl_assist_enabled = use_rmt_scl_assist;
  if (rmt_scl_assist_enabled) {
    rmt_rx_channel_config_t rmt_cfg{};
    rmt_cfg.gpio_num = pin_scl;
    rmt_cfg.clk_src = RMT_CLK_SRC_DEFAULT;
    rmt_cfg.resolution_hz = RMT_SCL_RESOLUTION_HZ;
    rmt_cfg.mem_block_symbols = SOC_RMT_MEM_WORDS_PER_CHANNEL;
    rmt_cfg.intr_priority = 3;
    rmt_cfg.flags.invert_in = false;
    rmt_cfg.flags.with_dma = false;
    rmt_cfg.flags.allow_pd = false;
    err = rmt_new_rx_channel(&rmt_cfg, &rmt_scl_channel);
    if (err == ESP_OK) {
      rmt_rx_event_callbacks_t cbs{};
      cbs.on_recv_done = rmt_scl_done_callback;
      err = rmt_rx_register_event_callbacks(rmt_scl_channel, &cbs, nullptr);
    }
    if (err == ESP_OK) {
      err = rmt_enable(rmt_scl_channel);
      if (err == ESP_OK) rmt_scl_channel_enabled = true;
    }
    if (err == ESP_OK && !rmt_scl_arm_()) err = ESP_FAIL;
    if (err != ESP_OK) {
      ESP_LOGW(TAG, "RMT SCL assist unavailable (%s); continuing with GPIO backend",
               esp_err_to_name(err));
      if (rmt_scl_channel) {
        if (rmt_scl_channel_enabled) rmt_disable(rmt_scl_channel);
        rmt_scl_channel_enabled = false;
        rmt_del_channel(rmt_scl_channel);
        rmt_scl_channel = nullptr;
      }
      rmt_scl_assist_enabled = false;
    } else {
      ESP_LOGI(TAG,
               "I2C capture backend: GPIO + RMT-SCL assist (%lu Hz, %u-symbol HW block, partial RX)",
               static_cast<unsigned long>(RMT_SCL_RESOLUTION_HZ),
               static_cast<unsigned>(SOC_RMT_MEM_WORDS_PER_CHANNEL));
    }
  } else {
    ESP_LOGI(TAG, "I2C capture backend: GPIO");
  }
#else
  if (use_rmt_scl_assist)
    ESP_LOGW(TAG, "RMT SCL assist requested but only implemented for ESP32-C3/C6; using GPIO");
#endif

  return true;
}

void shutdown() {
  capture_enabled = false;
  capturing = false;
  gpio_intr_disable(pin_scl);
  gpio_intr_disable(pin_sda);
  gpio_isr_handler_remove(pin_scl);
  gpio_isr_handler_remove(pin_sda);
#if defined(CONFIG_IDF_TARGET_ESP32C3) || defined(CONFIG_IDF_TARGET_ESP32C6)
  if (rmt_scl_channel != nullptr) {
    if (rmt_scl_channel_enabled) rmt_disable(rmt_scl_channel);
    rmt_scl_channel_enabled = false;
    rmt_del_channel(rmt_scl_channel);
    rmt_scl_channel = nullptr;
  }
  rmt_scl_assist_enabled = false;
#endif
  gpio_set_intr_type(pin_scl, GPIO_INTR_DISABLE);
  gpio_set_intr_type(pin_sda, GPIO_INTR_DISABLE);
  gpio_set_direction(pin_scl, GPIO_MODE_INPUT);
  gpio_set_direction(pin_sda, GPIO_MODE_INPUT);
  gpio_pullup_dis(pin_scl);
  gpio_pullup_dis(pin_sda);
  gpio_pulldown_dis(pin_scl);
  gpio_pulldown_dis(pin_sda);
}


void set_capture_enabled(bool enabled) {
  if (capture_enabled == enabled) return;

  // Stop GPIO edge capture while resetting shared ISR state so no half-frame
  // can leak across a sleep/awake boundary.
  gpio_intr_disable(pin_scl);
  gpio_intr_disable(pin_sda);
#if defined(CONFIG_IDF_TARGET_ESP32C3) || defined(CONFIG_IDF_TARGET_ESP32C6)
  if (rmt_scl_assist_enabled) rmt_scl_suspend_();
#endif
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
#if defined(CONFIG_IDF_TARGET_ESP32C3) || defined(CONFIG_IDF_TARGET_ESP32C6)
    if (rmt_scl_assist_enabled) rmt_scl_resume_();
#endif
  }
}

void set_co2_warmup_activity_watch(bool enabled) { co2_warmup_activity_watch = enabled; }

bool capture_in_progress() {
  return capture_enabled && (sample_count != 0 || capture_finished);
}

void rearm_after_light_sleep() {
  // Automatic Light-sleep should retain GPIO configuration, but the observed
  // battery-only failure mode leaves the passive tap reading both CO2 lines
  // LOW until Light-sleep is disabled. Re-assert only the input-side GPIO
  // state here; never enable pulls and never drive either bus line.
  gpio_intr_disable(pin_scl);
  gpio_intr_disable(pin_sda);
#if defined(CONFIG_IDF_TARGET_ESP32C3) || defined(CONFIG_IDF_TARGET_ESP32C6)
  if (rmt_scl_assist_enabled) rmt_scl_suspend_();
#endif
  gpio_set_intr_type(pin_scl, GPIO_INTR_DISABLE);
  gpio_set_intr_type(pin_sda, GPIO_INTR_DISABLE);

  gpio_set_direction(pin_scl, GPIO_MODE_INPUT);
  gpio_set_direction(pin_sda, GPIO_MODE_INPUT);
  gpio_pullup_dis(pin_scl);
  gpio_pullup_dis(pin_sda);
  gpio_pulldown_dis(pin_scl);
  gpio_pulldown_dis(pin_sda);

  // Drop any waveform fragment spanning sleep/wake and establish a fresh
  // baseline before ANYEDGE capture is armed again.
  capturing = false;
  sample_count = 0;
  capture_finished = false;
  capture_overflow = false;
  last_value = read_gpio_state();
  capture_initial_value = last_value;
  last_edge = static_cast<uint32_t>(esp_timer_get_time());

  gpio_set_intr_type(pin_scl, GPIO_INTR_ANYEDGE);
  gpio_set_intr_type(pin_sda, GPIO_INTR_ANYEDGE);
  capturing = capture_enabled;
  if (capture_enabled) {
    gpio_intr_enable(pin_scl);
    gpio_intr_enable(pin_sda);
#if defined(CONFIG_IDF_TARGET_ESP32C3) || defined(CONFIG_IDF_TARGET_ESP32C6)
    if (rmt_scl_assist_enabled) rmt_scl_resume_();
#endif
  }

  ESP_LOGI(TAG, "I2C GPIOs re-armed after Light-sleep wake: SCL=%d SDA=%d",
           gpio_get_level(pin_scl), gpio_get_level(pin_sda));
}

bool poll(Capture &capture, CaptureValidator recovery_validator) {
  if (!capture_enabled) return false;
#if RTRH_DEBUG_CAPTURE
  // Keep the shared ISR sample buffer frozen while a multi-packet UDP export is
  // pending. One packet is attempted per poll/loop; only after the final packet
  // succeeds do we re-arm GPIO capture.
  if (debug_udp::enabled() && udp_pending.active && capture_finished) {
    if (!udp_export_pending_step()) return false;
    account_edge_diagnostics(samples, sample_count, capture_initial_value);
    sample_count = 0;
    capture_overflow = false;
    capture_finished = false;
    last_value = read_gpio_state();
    capture_initial_value = last_value;
    last_edge = static_cast<uint32_t>(esp_timer_get_time());
    capturing = true;
#if defined(CONFIG_IDF_TARGET_ESP32C3) || defined(CONFIG_IDF_TARGET_ESP32C6)
    if (rmt_scl_assist_enabled) { rmt_scl_suspend_(); rmt_scl_resume_(); }
#endif
    return false;
  }
#endif
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
  const EdgeDiagCounts edge_diag =
      calculate_edge_diagnostics(samples, count, initial_value);
#if defined(CONFIG_IDF_TARGET_ESP32C3) || defined(CONFIG_IDF_TARGET_ESP32C6)
  if (rmt_scl_assist_enabled && rmt_scl_done) diag_rmt_captures++;
#endif
  capture = Capture{};

  if (count) {
#if RTRH_DEBUG_CAPTURE
    // Preserve the unmodified GPIO waveform. Decoder-side recovery operates on
    // a temporary in-place transformation and never rewrites the frozen trace.
    capture.debug_raw_sequence = store_raw_capture(samples, count, initial_value, overflow);
    if (debug_udp::enabled() && udp_pending.active) udp_export_pending_step();
#endif
    if (overflow) {
      capture.frame_errors++;
    } else {
      decode_capture(samples, count, initial_value, capture);

      // The shared GPIO ISR may occasionally miss one or two complete SCL
      // pulses on the single-core ESP32-C3. Only attempt reconstruction when
      // the caller rejects the original capture, and accept it only when all
      // protocol-valid candidates decode to the exact same transaction.
      if (recovery_validator && !recovery_validator(capture)) {
        Capture recovered{};
        bool repaired = false;
#if defined(CONFIG_IDF_TARGET_ESP32C3) || defined(CONFIG_IDF_TARGET_ESP32C6)
        if (rmt_scl_assist_enabled) {
          if (rmt_scl_done && !rmt_scl_overflow) {
            repaired = try_rmt_scl_repair_(samples, count, initial_value,
                                           recovery_validator, recovered);
          } else {
            diag_rmt_fallbacks++;
          }
        }
#endif
        if (!repaired) {
          repaired = try_missing_clock_recovery(samples, count, initial_value,
                                                recovery_validator, recovered);
        }
        if (repaired) {
#if RTRH_DEBUG_CAPTURE
          recovered.debug_raw_sequence = capture.debug_raw_sequence;
#endif
          capture = recovered;
        }
      }
    }
  }
  capture.raw_scl_edges = static_cast<uint16_t>(
      std::min<uint32_t>(edge_diag.scl, UINT16_MAX));

#if RTRH_DEBUG_CAPTURE
  if (debug_udp::enabled() && udp_pending.active) {
    // The decoded Capture is already valid for the caller, but the raw sample
    // array must stay untouched until all UDP fragments have left successfully.
    return true;
  }
#endif

  account_edge_diagnostics(edge_diag);
  sample_count = 0;
  capture_overflow = false;
  capture_finished = false;
  last_value = read_gpio_state();
  capture_initial_value = last_value;
  last_edge = static_cast<uint32_t>(esp_timer_get_time());

  capturing = true;
#if defined(CONFIG_IDF_TARGET_ESP32C3) || defined(CONFIG_IDF_TARGET_ESP32C6)
  if (rmt_scl_assist_enabled) { rmt_scl_suspend_(); rmt_scl_resume_(); }
#endif
  diag_completed_captures++;
  return true;
}



bool bus_is_low_low() {
  return gpio_get_level(pin_scl) == 0 && gpio_get_level(pin_sda) == 0;
}

uint32_t last_edge_age_us() {
  return static_cast<uint32_t>(static_cast<uint32_t>(esp_timer_get_time()) - last_edge);
}

bool active_write_command(uint16_t command) {
  if (!begin_active_master_()) {
    ESP_LOGW(TAG, "Active I2C probe: hard-drive master setup failed");
    return false;
  }

  bool ok = master_start_();
  if (ok) ok = master_write_byte_(static_cast<uint8_t>((0x62U << 1) | 0U));
  if (ok) ok = master_write_byte_(static_cast<uint8_t>(command >> 8));
  if (ok) ok = master_write_byte_(static_cast<uint8_t>(command & 0xFF));
  master_stop_();
  restore_passive_gpio_();

  ESP_LOGI(TAG, "Active I2C probe: W 0x62 %02X %02X -> %s",
           static_cast<unsigned>(command >> 8), static_cast<unsigned>(command & 0xFF),
           ok ? "ACK" : "NACK/failed");
  return ok;
}

bool active_read_command(uint16_t command, uint8_t *data, uint8_t length) {
  if (!data || length == 0) return false;
  if (!begin_active_master_()) {
    ESP_LOGW(TAG, "Active I2C probe read: hard-drive master setup failed");
    return false;
  }

  ESP_LOGI(TAG, "Active I2C probe: passive IRQs disabled; taking temporary bus ownership");

  // Keep ownership for the complete command + read sequence. In particular,
  // do not restore/re-arm the passive sniffer between EC05 and the direct read:
  // our own GPIO transitions must never look like native Unni bus activity.
  bool ok = master_start_();
  if (ok) ok = master_write_byte_(static_cast<uint8_t>((0x62U << 1) | 0U));
  if (ok) ok = master_write_byte_(static_cast<uint8_t>(command >> 8));
  if (ok) ok = master_write_byte_(static_cast<uint8_t>(command & 0xFF));
  master_stop_();

  ESP_LOGI(TAG, "Active I2C probe: W 0x62 %02X %02X -> %s",
           static_cast<unsigned>(command >> 8), static_cast<unsigned>(command & 0xFF),
           ok ? "ACK" : "NACK/failed");

  if (ok) {
    // SCD4x command-to-read delay is tiny for read_measurement; retain the
    // diagnostic build's conservative 3 ms delay while keeping IRQs disabled.
    esp_rom_delay_us(3000);
    ok = master_start_();
    if (ok) ok = master_write_byte_(static_cast<uint8_t>((0x62U << 1) | 1U));
    for (uint8_t i = 0; ok && i < length; ++i)
      ok = master_read_byte_(data[i], i + 1U < length);
    master_stop_();
  }

  if (ok) {
    char bytes[3 * 9 + 1]{};
    size_t used = 0;
    for (uint8_t i = 0; i < length && used + 4 < sizeof(bytes); ++i)
      used += static_cast<size_t>(snprintf(bytes + used, sizeof(bytes) - used, "%02X%s",
                                          data[i], i + 1U < length ? " " : ""));
    ESP_LOGI(TAG, "Active I2C probe: direct R 0x62 -> %s", bytes);
  } else {
    ESP_LOGI(TAG, "Active I2C probe: direct R 0x62 -> NACK/failed");
  }

  restore_passive_gpio_();
  ESP_LOGI(TAG, "Active I2C probe: passive sniffer restored: SCL=%d SDA=%d",
           gpio_get_level(pin_scl), gpio_get_level(pin_sda));
  return ok;
}

void log_edge_diagnostics(uint32_t now_ms) {
  if (diag_last_report_ms == 0) {
    diag_last_report_ms = now_ms;
    diag_prev_isr_calls = diag_isr_calls;
    diag_prev_state_changes = diag_state_changes;
    diag_prev_scl_changes = diag_scl_changes;
    diag_prev_sda_changes = diag_sda_changes;
    diag_prev_completed_captures = diag_completed_captures;
    return;
  }
  if (static_cast<uint32_t>(now_ms - diag_last_report_ms) < 5000U) return;

  const uint32_t isr = diag_isr_calls;
  // Completed captures are accumulated outside the ISR. Add the currently
  // active/frozen buffer here so the 5-second diagnostic retains its original
  // near-real-time semantics without paying three volatile increments per edge.
  uint16_t pending_count = sample_count;
  if (pending_count > MAX_SAMPLES) pending_count = MAX_SAMPLES;
  const EdgeDiagCounts pending =
      calculate_edge_diagnostics(samples, pending_count, capture_initial_value);
  const uint32_t changes = diag_state_changes + pending.state;
  const uint32_t scl = diag_scl_changes + pending.scl;
  const uint32_t sda = diag_sda_changes + pending.sda;
  const uint32_t completed = diag_completed_captures;
  const uint32_t edge_age_us = static_cast<uint32_t>(
      static_cast<uint32_t>(esp_timer_get_time()) - last_edge);
  const uint8_t levels = read_gpio_state();

#if defined(CONFIG_IDF_TARGET_ESP32C3) || defined(CONFIG_IDF_TARGET_ESP32C6)
  if (rmt_scl_assist_enabled) {
    ESP_LOGI(TAG,
             "I2C edge diag: +ISR=%lu +state=%lu +SCL=%lu +SDA=%lu /5s; levels SCL=%u SDA=%u; samples=%u capturing=%s finished=%s enabled=%s; +captures=%lu overflows=%lu; RMT +captures=%lu +repairs=%lu +edges=%lu +fallbacks=%lu symbols=%u done=%s overflow=%s; last_edge=%lu us ago",
             static_cast<unsigned long>(isr - diag_prev_isr_calls),
             static_cast<unsigned long>(changes - diag_prev_state_changes),
             static_cast<unsigned long>(scl - diag_prev_scl_changes),
             static_cast<unsigned long>(sda - diag_prev_sda_changes),
             scl_level(levels) ? 1U : 0U, sda_level(levels) ? 1U : 0U,
             static_cast<unsigned>(sample_count), capturing ? "yes" : "no",
             capture_finished ? "yes" : "no", capture_enabled ? "yes" : "no",
             static_cast<unsigned long>(completed - diag_prev_completed_captures),
             static_cast<unsigned long>(diag_overflows),
             static_cast<unsigned long>(diag_rmt_captures - diag_prev_rmt_captures),
             static_cast<unsigned long>(diag_rmt_repairs - diag_prev_rmt_repairs),
             static_cast<unsigned long>(diag_rmt_edges_restored - diag_prev_rmt_edges_restored),
             static_cast<unsigned long>(diag_rmt_fallbacks - diag_prev_rmt_fallbacks),
             static_cast<unsigned>(rmt_scl_accum_count), rmt_scl_done ? "yes" : "no",
             rmt_scl_overflow ? "yes" : "no", static_cast<unsigned long>(edge_age_us));
  } else
#endif
  {
    ESP_LOGI(TAG,
             "I2C edge diag: +ISR=%lu +state=%lu +SCL=%lu +SDA=%lu /5s; levels SCL=%u SDA=%u; samples=%u capturing=%s finished=%s enabled=%s; +captures=%lu overflows=%lu; last_edge=%lu us ago",
             static_cast<unsigned long>(isr - diag_prev_isr_calls),
             static_cast<unsigned long>(changes - diag_prev_state_changes),
             static_cast<unsigned long>(scl - diag_prev_scl_changes),
             static_cast<unsigned long>(sda - diag_prev_sda_changes),
             scl_level(levels) ? 1U : 0U, sda_level(levels) ? 1U : 0U,
             static_cast<unsigned>(sample_count), capturing ? "yes" : "no",
             capture_finished ? "yes" : "no", capture_enabled ? "yes" : "no",
             static_cast<unsigned long>(completed - diag_prev_completed_captures),
             static_cast<unsigned long>(diag_overflows),
             static_cast<unsigned long>(edge_age_us));
  }

  diag_last_report_ms = now_ms;
  diag_prev_isr_calls = isr;
  diag_prev_state_changes = changes;
  diag_prev_scl_changes = scl;
  diag_prev_sda_changes = sda;
  diag_prev_completed_captures = completed;
#if defined(CONFIG_IDF_TARGET_ESP32C3) || defined(CONFIG_IDF_TARGET_ESP32C6)
  diag_prev_rmt_captures = diag_rmt_captures;
  diag_prev_rmt_repairs = diag_rmt_repairs;
  diag_prev_rmt_edges_restored = diag_rmt_edges_restored;
  diag_prev_rmt_fallbacks = diag_rmt_fallbacks;
#endif
}

}  // namespace i2c_sniffer
}  // namespace co2_monitor_0601
}  // namespace esphome
