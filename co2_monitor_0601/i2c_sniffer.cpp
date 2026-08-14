// SPDX-FileCopyrightText: 2026 The esphome-unni-co2-sniffer contributors
// SPDX-License-Identifier: GPL-3.0-or-later
#include "i2c_sniffer.h"

#include "esphome/core/log.h"
#include "driver/gpio.h"
#include "esp_timer.h"

#include <cstddef>
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
static bool estimate_scl_level_timing(const volatile Sample *data, uint16_t count,
                                      uint8_t initial_value, LevelTiming &timing) {
  uint32_t low[96]{};
  uint32_t high[96]{};
  uint8_t low_count = 0;
  uint8_t high_count = 0;
  bool level = scl_level(initial_value);
  uint32_t interval_start = count ? data[0].t : 0;

  for (uint16_t i = 0; i < count; i++) {
    const bool next_level = scl_level(data[i].value);
    if (next_level == level) continue;

    const uint32_t duration = static_cast<uint32_t>(data[i].t - interval_start);
    if (duration >= 10 && duration <= 60) {
      if (level) {
        if (high_count < 96) high[high_count++] = duration;
      } else {
        if (low_count < 96) low[low_count++] = duration;
      }
    }
    interval_start = data[i].t;
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

static bool collect_pulse_candidates(const volatile Sample *data, uint16_t count,
                                     uint8_t initial_value,
                                     const LevelTiming &timing,
                                     PulseCandidate *out, uint8_t &out_count) {
  out_count = 0;
  if (count < 2) return true;

  bool level = scl_level(initial_value);
  uint32_t interval_start = data[0].t;
  uint32_t sda_times[MAX_RECOVERY_SDA_EVENTS]{};
  uint8_t sda_count = 0;
  uint8_t interval_id = 0;
  uint8_t previous = initial_value;

  for (uint16_t i = 0; i < count; i++) {
    const uint8_t current = data[i].value;
    const bool next_level = scl_level(current);
    const bool scl_changed = next_level != level;
    const bool sda_changed = sda_level(current) != sda_level(previous);

    if (scl_changed) {
      const uint32_t end_t = data[i].t;
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
      sda_times[sda_count++] = data[i].t;
    } else if (sda_changed) {
      // More SDA transitions than the bounded candidate model can represent.
      // Skip recovery rather than silently omit possible orderings.
      return false;
    }
    previous = current;
  }
  return true;
}

static bool remove_sample_at(volatile Sample *data, uint16_t &count,
                             uint16_t index) {
  if (index >= count) return false;
  for (uint16_t i = index; i + 1 < count; i++) {
    data[i].t = data[i + 1].t;
    data[i].value = data[i + 1].value;
  }
  count--;
  return true;
}

static uint16_t find_insert_position(const volatile Sample *data, uint16_t count,
                                     uint32_t t) {
  uint16_t pos = 0;
  while (pos < count && static_cast<int32_t>(data[pos].t - t) < 0) pos++;
  return pos;
}

// Apply one synthetic opposite-level SCL pulse in-place. Existing SDA-only
// samples between t1/t2 keep their SDA value but inherit the synthetic SCL
// level. poll() has paused ISR writes, so this scratch transformation is safe.
static bool apply_pulse(volatile Sample *data, uint16_t &count,
                        uint8_t initial_value, const PulseCandidate &pulse) {
  if (count > MAX_SAMPLES - 2 || pulse.t2 <= pulse.t1) return false;
  uint16_t pos1 = find_insert_position(data, count, pulse.t1);
  uint16_t pos2 = find_insert_position(data, count, pulse.t2);
  if ((pos1 < count && data[pos1].t == pulse.t1) ||
      (pos2 < count && data[pos2].t == pulse.t2))
    return false;

  const uint8_t before1 = pos1 == 0 ? initial_value : data[pos1 - 1].value;

  // Flip SCL on all already-captured SDA transitions that occurred while the
  // missing opposite-level pulse should have been active.
  for (uint16_t i = pos1; i < count && data[i].t < pulse.t2; i++)
    data[i].value ^= 0x01U;

  // Insert first edge.
  for (uint16_t i = count; i > pos1; i--) {
    data[i].t = data[i - 1].t;
    data[i].value = data[i - 1].value;
  }
  data[pos1].t = pulse.t1;
  data[pos1].value = static_cast<uint8_t>(before1 ^ 0x01U);
  count++;

  // Insert return edge using the current SDA state immediately before t2.
  pos2 = find_insert_position(data, count, pulse.t2);
  const uint8_t before2 = pos2 == 0 ? initial_value : data[pos2 - 1].value;
  for (uint16_t i = count; i > pos2; i--) {
    data[i].t = data[i - 1].t;
    data[i].value = data[i - 1].value;
  }
  data[pos2].t = pulse.t2;
  data[pos2].value = static_cast<uint8_t>(before2 ^ 0x01U);
  count++;
  return true;
}

static bool undo_pulse(volatile Sample *data, uint16_t &count,
                       const PulseCandidate &pulse) {
  uint16_t p1 = count;
  uint16_t p2 = count;
  for (uint16_t i = 0; i < count; i++) {
    if (data[i].t == pulse.t1) p1 = i;
    if (data[i].t == pulse.t2) p2 = i;
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
    if (data[i].t > pulse.t1 && data[i].t < pulse.t2)
      data[i].value ^= 0x01U;
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

static bool try_missing_clock_recovery(volatile Sample *data, uint16_t count,
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

void register_debug_handler() {
  if (!last_capture_mutex) last_capture_mutex = xSemaphoreCreateMutex();
  if (web_server_base::global_web_server_base) {
    web_server_base::global_web_server_base->add_handler(&capture_handler);
  } else {
    ESP_LOGW(TAG, "web_server_base unavailable");
  }
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

  return true;
}


void set_capture_enabled(bool enabled) {
  if (capture_enabled == enabled) return;

  // Stop GPIO edge capture while resetting shared ISR state so no half-frame
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

bool poll(Capture &capture, CaptureValidator recovery_validator) {
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
  capture = Capture{};

  if (count) {
#if RTRH_DEBUG_CAPTURE
    // Preserve the unmodified GPIO waveform. Decoder-side recovery operates on
    // a temporary in-place transformation and never rewrites the frozen trace.
    capture.debug_raw_sequence = store_raw_capture(samples, count, initial_value, overflow);
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
        if (try_missing_clock_recovery(samples, count, initial_value,
                                       recovery_validator, recovered)) {
#if RTRH_DEBUG_CAPTURE
          recovered.debug_raw_sequence = capture.debug_raw_sequence;
#endif
          capture = recovered;
        }
      }
    }
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
