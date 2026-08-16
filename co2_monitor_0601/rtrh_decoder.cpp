// SPDX-FileCopyrightText: 2026 The esphome-unni-co2-sniffer contributors
// SPDX-License-Identifier: GPL-3.0-or-later
#include "rtrh_decoder.h"
#include "debug_udp.h"

#include "calibration.h"
#include "power_save.h"
#include "esphome/core/log.h"
#include "driver/gpio.h"
#include "esp_timer.h"

#include <algorithm>
#include <cstdio>
#include <cstring>

#if RTRH_DEBUG_CAPTURE
#if defined(USE_WEB_SERVER_BASE)
#include "esphome/components/web_server_base/web_server_base.h"
#include "esp_http_server.h"
#include <string>
#endif
#endif

namespace esphome {
namespace co2_monitor_0601 {
namespace rtrh_decoder {

static const char *TAG = "rtrh_decoder";
static gpio_num_t pin_rt = GPIO_NUM_3;
static gpio_num_t pin_rh = GPIO_NUM_4;
static gpio_num_t pins[] = {GPIO_NUM_3, GPIO_NUM_4};

// The controller spends ~125 ms in REF and ~127 ms in RT. Phase identity is
// deliberately based on elapsed time, never on RC period or cycle count.
static constexpr uint32_t MEASUREMENT_QUIET_US = 100000;
static constexpr uint32_t REF_PHASE_END_US = 125000;
static constexpr uint32_t RT_PHASE_END_US = 252000;
static constexpr uint32_t CYCLE_MAX_US = 20000;
static constexpr uint16_t RT_TEMP_CYCLES = 880;
static constexpr uint8_t RH_STATE_PERIOD_SAMPLES = 96;

struct Accum {
  uint32_t period_sum{0};
  uint16_t count{0};
};

struct RhStateStats {
  uint32_t last_us{0};
  uint16_t samples[RH_STATE_PERIOD_SAMPLES]{};
  uint8_t write_pos{0};
  uint8_t sample_count{0};
  uint32_t seen{0};
};

struct Snapshot {
  Accum ref;
  Accum rt;
  Accum rh_timing;
  uint32_t rt_temp_period_sum{0};
  uint16_t rt_temp_count{0};
  RhStateStats rh_state;
  uint32_t sequence{0};
};

enum class Phase : uint8_t { WAIT_REF = 0, REF, RT, RH };

// Capture acceptance limits. These are deliberately kept next to the decoder
// because they describe whether one decoded RT/RH cycle is trustworthy.
static constexpr float REF_PERIOD_MIN_US = 72.0f;
static constexpr float REF_PERIOD_MAX_US = 82.0f;
static constexpr float REF_DURATION_MIN_MS = 122.0f;
static constexpr float REF_DURATION_MAX_MS = 128.0f;
static constexpr float RT_PERIOD_MIN_US = 100.0f;
static constexpr float RT_PERIOD_MAX_US = 220.0f;
static constexpr float RT_DURATION_MIN_MS = 123.0f;
static constexpr float RT_DURATION_MAX_MS = 130.0f;
static constexpr uint16_t RT_COUNT_MIN = 600;
static constexpr float RH_DURATION_MIN_MS = 127.0f;
static constexpr float RH_DURATION_MAX_MS = 134.0f;
static constexpr uint8_t RH_STATE_SAMPLES_MIN = 32;
static constexpr float RH_STATE_MIN_US = 40.0f;
static constexpr float RH_STATE_MAX_US = 60000.0f;
static constexpr float RH_RATIO_VALID_MAX = 20.0f;

const char *reject_reason_to_string(RejectReason reason) {
  switch (reason) {
    case RejectReason::NONE: return "NONE";
    case RejectReason::REF_PERIOD: return "REF_PERIOD";
    case RejectReason::REF_DURATION: return "REF_DURATION";
    case RejectReason::RT_PERIOD: return "RT_PERIOD";
    case RejectReason::RT_DURATION: return "RT_DURATION";
    case RejectReason::RT_COUNT: return "RT_COUNT";
    case RejectReason::RH_DURATION: return "RH_DURATION";
    case RejectReason::RH_TOO_FEW_SAMPLES: return "RH_TOO_FEW_SAMPLES";
    case RejectReason::RH_STATE_PERIOD: return "RH_STATE_PERIOD";
    case RejectReason::RH_RATIO_IMPLAUSIBLE: return "RH_RATIO_IMPLAUSIBLE";
  }
  return "UNKNOWN";
}

struct DecoderState {
  volatile bool collecting{false};
  volatile uint32_t measurement_start_us{0};
  volatile uint32_t last_edge_us{0};
  volatile uint8_t gpio_state{0};
  volatile uint32_t last_rt_fall_us{0};
  volatile bool have_rt_rise{false};
  volatile Phase phase{Phase::WAIT_REF};

  Accum ref;
  Accum rt;
  Accum rh;
  volatile uint32_t rt_temperature_period_sum{0};
  volatile uint16_t rt_temperature_count{0};
  RhStateStats rh_state;

  Snapshot snapshot;
  volatile bool snapshot_ready{false};
  volatile uint8_t pin_level[2]{0, 0};
  uint32_t last_polled_sequence{0};
  Measurement latest_measurement;
};

static DecoderState decoder;

static inline uint8_t IRAM_ATTR read_state() {
  uint8_t value = 0;
  if (gpio_get_level(pin_rt)) value |= 0x01;
  if (gpio_get_level(pin_rh)) value |= 0x08;
  return value;
}

static inline void IRAM_ATTR clear_accum(Accum &a) {
  a.period_sum = 0;
  a.count = 0;
}

static inline void IRAM_ATTR clear_rh_state_stats(RhStateStats &s) {
  s.last_us = 0;
  s.write_pos = 0;
  s.sample_count = 0;
  s.seen = 0;
  for (uint8_t i = 0; i < RH_STATE_PERIOD_SAMPLES; i++) s.samples[i] = 0;
}

static inline void IRAM_ATTR clear_rh_state() {
  clear_rh_state_stats(decoder.rh_state);
}

static inline void IRAM_ATTR reset_measurement(uint32_t now, uint8_t state) {
  decoder.collecting = true;
  decoder.measurement_start_us = now;
  decoder.last_edge_us = now;
  decoder.gpio_state = state;
  decoder.last_rt_fall_us = 0;
  decoder.have_rt_rise = false;
  decoder.phase = Phase::REF;
  clear_accum(decoder.ref);
  clear_accum(decoder.rt);
  clear_accum(decoder.rh);
  decoder.rt_temperature_period_sum = 0;
  decoder.rt_temperature_count = 0;
  clear_rh_state();
}

static inline void IRAM_ATTR add_period(Accum &a, uint32_t period) {
  if (a.count != 0xFFFF) a.count++;
  a.period_sum += period;
}

static inline void IRAM_ATTR update_phase(uint32_t now) {
  const uint32_t elapsed = static_cast<uint32_t>(now - decoder.measurement_start_us);
  const Phase next = elapsed < REF_PHASE_END_US ? Phase::REF :
                     elapsed < RT_PHASE_END_US ? Phase::RT : Phase::RH;
  if (next == decoder.phase) return;
  decoder.phase = next;
  decoder.last_rt_fall_us = 0;  // Never let a period cross a fixed phase boundary.
  decoder.have_rt_rise = false;
  if (next == Phase::RH) {
    decoder.rh_state.last_us = 0;
  }
}

static inline void IRAM_ATTR observe_rh_stats(uint32_t now, RhStateStats &stats) {
  if (stats.last_us != 0) {
    const uint32_t dt = static_cast<uint32_t>(now - stats.last_us);
    if (dt >= 40 && dt <= 60000) {
      stats.samples[stats.write_pos] = static_cast<uint16_t>(dt);
      stats.write_pos = static_cast<uint8_t>((stats.write_pos + 1) % RH_STATE_PERIOD_SAMPLES);
      if (stats.sample_count < RH_STATE_PERIOD_SAMPLES) stats.sample_count++;
    }
  }
  stats.last_us = now;
  stats.seen++;
}

static inline void IRAM_ATTR observe_rh_state(uint32_t now, uint8_t state) {
  // Characteristic RH state using only the two required RT/RH lines:
  // Characteristic RH state: RT=0, RH=1.
  if (decoder.phase != Phase::RH || (state & 0x09) != 0x08) return;
  observe_rh_stats(now, decoder.rh_state);
}

static uint8_t rh_state_sorted_samples(const RhStateStats &s, uint16_t *tmp) {
  const uint8_t n = s.sample_count;
  for (uint8_t i = 0; i < n; i++) tmp[i] = s.samples[i];
  for (uint8_t i = 1; i < n; i++) {
    const uint16_t value = tmp[i];
    uint8_t j = i;
    while (j > 0 && tmp[j - 1] > value) {
      tmp[j] = tmp[j - 1];
      j--;
    }
    tmp[j] = value;
  }
  return n;
}

static float rh_state_period_median(const RhStateStats &s) {
  uint16_t tmp[RH_STATE_PERIOD_SAMPLES];
  const uint8_t n = rh_state_sorted_samples(s, tmp);
  if (!n) return 0.0f;
  if (n & 1) return static_cast<float>(tmp[n / 2]);
  return 0.5f * (static_cast<float>(tmp[n / 2 - 1]) + static_cast<float>(tmp[n / 2]));
}

static void derive_rh_distribution(const RhStateStats &s, Measurement &m) {
  uint16_t tmp[RH_STATE_PERIOD_SAMPLES];
  const uint8_t n = rh_state_sorted_samples(s, tmp);
  if (!n) return;

  m.rh_state_min_us = tmp[0];
  m.rh_state_p25_us = tmp[(static_cast<uint16_t>(n - 1) * 25U) / 100U];
  m.rh_state_p75_us = tmp[(static_cast<uint16_t>(n - 1) * 75U) / 100U];
  m.rh_state_max_us = tmp[n - 1];

  // Wide diagnostic windows around the two populations observed in the field:
  // roughly 220 us and its near-exact 2x alias around 440 us. Keep an explicit
  // 'other' bucket so a future third population cannot hide in the summary.
  for (uint8_t i = 0; i < n; i++) {
    const uint16_t value = tmp[i];
    if (value >= 160U && value <= 280U) {
      m.rh_state_near_220++;
    } else if (value >= 360U && value <= 520U) {
      m.rh_state_near_440++;
    } else {
      m.rh_state_other++;
    }
  }
}

static void finalize_measurement() {
  if (!decoder.collecting) return;
  if (!decoder.ref.count || !decoder.rt.count || !decoder.rh.count) {
    decoder.collecting = false;
    return;
  }
  Snapshot next{};
  next.ref = decoder.ref;
  next.rt = decoder.rt;
  next.rh_timing = decoder.rh;
  next.rt_temp_period_sum = decoder.rt_temperature_period_sum;
  next.rt_temp_count = decoder.rt_temperature_count;
  next.rh_state = decoder.rh_state;
  next.sequence = decoder.snapshot.sequence + 1;
  decoder.snapshot = next;
  decoder.snapshot_ready = true;
  decoder.collecting = false;
}

#if RTRH_DEBUG_CAPTURE
static constexpr uint32_t CAPTURE_US = 450000;
static constexpr uint16_t MAX_SAMPLES = 1536;
static constexpr uint32_t CAPTURE_DECIMATION = 16;
static constexpr uint32_t CAPTURE_UNUSUAL_DECIMATION = 8;
struct __attribute__((packed)) DebugSample { uint32_t t_us; uint16_t edge_no; uint8_t value; };
struct DebugCaptureState {
  volatile DebugSample samples[MAX_SAMPLES];
  volatile uint16_t sample_count{0};
  volatile uint16_t edge_count{0};
  volatile uint16_t unusual_count{0};
  volatile uint8_t last_value{0xff};
  volatile uint32_t start_us{0};
  volatile bool capturing{false};
  volatile bool ready{false};
  volatile bool overflow{false};
  volatile uint32_t sequence{0};
};
static DebugCaptureState debug;
static uint16_t debug_udp_packet_index = 0;
static bool debug_udp_timing_pending = false;

static uint8_t rtrh_stream_byte(size_t offset, uint16_t count, bool overflow) {
  // Stream prefix: sample_count LE16, overflow U8, reserved U8.
  if (offset == 0) return static_cast<uint8_t>(count & 0xFFU);
  if (offset == 1) return static_cast<uint8_t>((count >> 8) & 0xFFU);
  if (offset == 2) return overflow ? 1U : 0U;
  if (offset == 3) return 0U;
  offset -= 4;
  const size_t sample_index = offset / 7U;
  const size_t field_byte = offset % 7U;
  if (sample_index >= count) return 0;
  const uint32_t t = debug.samples[sample_index].t_us;
  const uint16_t edge = debug.samples[sample_index].edge_no;
  const uint8_t value = debug.samples[sample_index].value;
  if (field_byte < 4U) return static_cast<uint8_t>((t >> (field_byte * 8U)) & 0xFFU);
  if (field_byte < 6U) return static_cast<uint8_t>((edge >> ((field_byte - 4U) * 8U)) & 0xFFU);
  return value;
}

static void debug_udp_timing_loop();

static void debug_udp_loop() {
  if (!debug_udp::enabled()) return;
  if (!debug.ready || !debug.sample_count) {
    debug_udp_timing_loop();
    return;
  }
  const uint16_t count = debug.sample_count;
  const bool overflow = debug.overflow;
  const size_t total_bytes = 4U + static_cast<size_t>(count) * 7U;
  const uint16_t packet_count = static_cast<uint16_t>(
      (total_bytes + debug_udp::MAX_PAYLOAD - 1U) / debug_udp::MAX_PAYLOAD);
  if (debug_udp_packet_index >= packet_count) debug_udp_packet_index = 0;

  const size_t offset = static_cast<size_t>(debug_udp_packet_index) * debug_udp::MAX_PAYLOAD;
  const uint16_t payload_len = static_cast<uint16_t>(
      std::min<size_t>(debug_udp::MAX_PAYLOAD, total_bytes - offset));
  static uint8_t payload[debug_udp::MAX_PAYLOAD];
  for (uint16_t i = 0; i < payload_len; i++)
    payload[i] = rtrh_stream_byte(offset + i, count, overflow);

  if (!debug_udp::send_packet(debug_udp::PacketType::RTRH_RAW, debug.sequence,
                              debug_udp_packet_index, packet_count, payload, payload_len,
                              overflow ? 1U : 0U))
    return;

  debug_udp_packet_index++;
  if (debug_udp_packet_index >= packet_count) {
    ESP_LOGD(TAG, "UDP exported RT/RH raw capture #%lu (%u samples, %u packets)",
             static_cast<unsigned long>(debug.sequence), static_cast<unsigned>(count),
             static_cast<unsigned>(packet_count));
    debug.sample_count = debug.edge_count = debug.unusual_count = 0;
    debug.overflow = debug.ready = debug.capturing = false;
    debug_udp_packet_index = 0;
  }
}
#endif

static void IRAM_ATTR gpio_isr(void *arg) {
  power_save::on_rtrh_edge_from_isr();
  const intptr_t encoded = reinterpret_cast<intptr_t>(arg);
  if (encoded < 1 || encoded > 2) return;
  const uint8_t pin_index = static_cast<uint8_t>(encoded - 1);
  const gpio_num_t pin = pins[pin_index];
  const uint8_t level = static_cast<uint8_t>(gpio_get_level(pin));
  const uint8_t previous = decoder.pin_level[pin_index];
  if (level == previous) return;
  decoder.pin_level[pin_index] = level;

  const uint32_t now = static_cast<uint32_t>(esp_timer_get_time());
  const uint8_t state = read_state();
  if (!decoder.collecting) reset_measurement(now, state);
  else decoder.last_edge_us = now;
  update_phase(now);

  // REF/RT timing comes only from the physical RT IRQ. This avoids ordering
  // errors when several sensor lines change almost simultaneously.
  const bool is_rt_irq = pin_index == 0;
  if (is_rt_irq && previous == 0 && level != 0) decoder.have_rt_rise = true;
  if (is_rt_irq && previous != 0 && level == 0) {
    const uint32_t previous_fall = decoder.last_rt_fall_us;
    decoder.last_rt_fall_us = now;
    if (previous_fall && decoder.have_rt_rise) {
      const uint32_t period = static_cast<uint32_t>(now - previous_fall);
      if (period <= CYCLE_MAX_US) {
        if (decoder.phase == Phase::REF) add_period(decoder.ref, period);
        else if (decoder.phase == Phase::RT) {
          add_period(decoder.rt, period);
          if (decoder.rt_temperature_count < RT_TEMP_CYCLES) {
            decoder.rt_temperature_period_sum += period;
            decoder.rt_temperature_count++;
          }
        } else if (decoder.phase == Phase::RH) {
          add_period(decoder.rh, period);
        }
      }
    }
    decoder.have_rt_rise = false;
  }

  if (state != decoder.gpio_state) {
    observe_rh_state(now, state);
    decoder.gpio_state = state;
  }


#if RTRH_DEBUG_CAPTURE
  if (debug.ready) return;
  const uint8_t value = read_state();
  if (value == debug.last_value) return;
  debug.last_value = value;
  if (!debug.capturing) {
    debug.capturing = true;
    debug.start_us = now;
    debug.sample_count = debug.edge_count = debug.unusual_count = 0;
    debug.overflow = false;
  }
  const uint16_t edge_no = debug.edge_count++;
  const bool unusual = value != 0x00 && value != 0x09;
  const bool time_anchor = edge_no == 0 || (edge_no % CAPTURE_DECIMATION) == 0;
  bool unusual_anchor = false;
  if (unusual) {
    const uint16_t unusual_no = debug.unusual_count++;
    unusual_anchor = unusual_no == 0 || (unusual_no % CAPTURE_UNUSUAL_DECIMATION) == 0;
  }
  if (!time_anchor && !unusual_anchor) return;
  const uint16_t index = debug.sample_count;
  if (index >= MAX_SAMPLES) {
    debug.overflow = true;
    debug.capturing = false;
    debug.ready = true;
    debug.sequence++;
    return;
  }
  debug.samples[index].t_us = static_cast<uint32_t>(now - debug.start_us);
  debug.samples[index].edge_no = edge_no;
  debug.samples[index].value = value;
  debug.sample_count = index + 1;
#endif
}

bool setup(uint8_t rt_pin, uint8_t rh_pin) {
  pin_rt = static_cast<gpio_num_t>(rt_pin);
  pin_rh = static_cast<gpio_num_t>(rh_pin);
  pins[0] = pin_rt;
  pins[1] = pin_rh;
  gpio_config_t io{};
  io.mode = GPIO_MODE_INPUT;
  io.pull_up_en = GPIO_PULLUP_DISABLE;
  io.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io.intr_type = GPIO_INTR_DISABLE;
  io.pin_bit_mask = (1ULL << pin_rt) | (1ULL << pin_rh);
  esp_err_t err = gpio_config(&io);
  if (err != ESP_OK) return false;

  for (gpio_num_t pin : pins) gpio_set_intr_type(pin, GPIO_INTR_ANYEDGE);
  decoder.gpio_state = read_state();
  for (uint8_t i = 0; i < 2; i++) decoder.pin_level[i] = gpio_get_level(pins[i]);
#if RTRH_DEBUG_CAPTURE
  debug.last_value = read_state();
#endif

  for (uint8_t i = 0; i < 2; i++) {
    err = gpio_isr_handler_add(pins[i], gpio_isr,
        reinterpret_cast<void *>(static_cast<intptr_t>(i + 1)));
    if (err != ESP_OK) return false;
  }
  return true;
}

void loop() {
  // RH can legitimately contain periods up to 60 ms. 100 ms of silence therefore
  // terminates the ~383 ms RT/RH transaction safely without the former 15 s delay.
  if (decoder.collecting) {
    const uint32_t now = static_cast<uint32_t>(esp_timer_get_time());
    const uint32_t last = decoder.last_edge_us;
    if (last && static_cast<uint32_t>(now - last) > MEASUREMENT_QUIET_US) {
      for (gpio_num_t pin : pins) gpio_intr_disable(pin);
      const uint32_t now2 = static_cast<uint32_t>(esp_timer_get_time());
      const uint32_t last2 = decoder.last_edge_us;
      if (decoder.collecting && last2 && static_cast<uint32_t>(now2 - last2) > MEASUREMENT_QUIET_US)
        finalize_measurement();
      decoder.gpio_state = read_state();
      for (uint8_t i = 0; i < 2; i++) decoder.pin_level[i] = gpio_get_level(pins[i]);
      for (gpio_num_t pin : pins) gpio_intr_enable(pin);
    }
  }

#if RTRH_DEBUG_CAPTURE
  if (debug.capturing && !debug.ready) {
    const uint32_t now = static_cast<uint32_t>(esp_timer_get_time());
    if (static_cast<uint32_t>(now - debug.start_us) >= CAPTURE_US) {
      debug.capturing = false;
      debug.ready = true;
      debug.sequence++;
      ESP_LOGI(TAG, "RT/RH edge capture ready: %u events, sequence %lu%s",
               debug.sample_count, static_cast<unsigned long>(debug.sequence),
               debug.overflow ? " OVERFLOW" : "");
      debug_udp_packet_index = 0;
    }
  }
  debug_udp_loop();
#endif
}

static RejectReason reject_reason(const Measurement &m) {
  if (!std::isfinite(m.ref_period_us) || m.ref_period_us < REF_PERIOD_MIN_US ||
      m.ref_period_us > REF_PERIOD_MAX_US)
    return RejectReason::REF_PERIOD;
  if (!std::isfinite(m.ref_duration_ms) || m.ref_duration_ms < REF_DURATION_MIN_MS ||
      m.ref_duration_ms > REF_DURATION_MAX_MS)
    return RejectReason::REF_DURATION;
  if (!std::isfinite(m.rt_period_us) || m.rt_period_us < RT_PERIOD_MIN_US ||
      m.rt_period_us > RT_PERIOD_MAX_US)
    return RejectReason::RT_PERIOD;
  if (!std::isfinite(m.rt_duration_ms) || m.rt_duration_ms < RT_DURATION_MIN_MS ||
      m.rt_duration_ms > RT_DURATION_MAX_MS)
    return RejectReason::RT_DURATION;
  if (m.rt_count < RT_COUNT_MIN) return RejectReason::RT_COUNT;
  if (!std::isfinite(m.rh_duration_ms) || m.rh_duration_ms < RH_DURATION_MIN_MS ||
      m.rh_duration_ms > RH_DURATION_MAX_MS)
    return RejectReason::RH_DURATION;
  if (m.rh_state_samples < RH_STATE_SAMPLES_MIN) return RejectReason::RH_TOO_FEW_SAMPLES;
  if (!std::isfinite(m.rh_state_us) || m.rh_state_us < RH_STATE_MIN_US ||
      m.rh_state_us > RH_STATE_MAX_US)
    return RejectReason::RH_STATE_PERIOD;
  if (!std::isfinite(m.rh_ratio) || m.rh_ratio <= 0.0f || m.rh_ratio > RH_RATIO_VALID_MAX)
    return RejectReason::RH_RATIO_IMPLAUSIBLE;
  return RejectReason::NONE;
}

static float quality_score(const Measurement &m) {
  const float ref_score = std::fmax(0.0f, 1.0f - std::fabs(m.ref_period_us - 76.75f) / 2.0f);
  const float rt_score = std::fmin(1.0f, static_cast<float>(m.rt_count) / 880.0f);
  const float rh_fill_score = std::fmin(1.0f, static_cast<float>(m.rh_state_samples) / 96.0f);

  float rh_seen_score = 1.0f;
  if (m.rh_state_seen > 0) {
    const float ratio = static_cast<float>(m.rh_state_seen) / static_cast<float>(m.rh_state_samples);
    rh_seen_score = std::fmin(1.0f, ratio / 2.0f);
  }

  return 100.0f * (0.25f * ref_score + 0.30f * rt_score +
                   0.30f * rh_fill_score + 0.15f * rh_seen_score);
}

static Measurement derive(const Snapshot &s) {
  Measurement m;
  m.sequence = s.sequence;
  m.ref_count = s.ref.count;
  m.rt_phase_count = s.rt.count;
  m.rt_count = s.rt_temp_count;
  m.rh_state_samples = s.rh_state.sample_count;
  m.rh_state_seen = s.rh_state.seen;

  m.ref_period_us = s.ref.count ? float(s.ref.period_sum) / s.ref.count : 0.0f;
  m.ref_duration_ms = float(s.ref.period_sum) / 1000.0f;
  m.rt_phase_period_us = s.rt.count ? float(s.rt.period_sum) / s.rt.count : 0.0f;
  m.rt_duration_ms = float(s.rt.period_sum) / 1000.0f;
  m.rt_period_us = s.rt_temp_count ? float(s.rt_temp_period_sum) / s.rt_temp_count : 0.0f;
  m.rh_duration_ms = float(s.rh_timing.period_sum) / 1000.0f;
  m.rh_state_us = rh_state_period_median(s.rh_state);
  derive_rh_distribution(s.rh_state, m);
  m.rt_ratio = m.ref_period_us > 0.0f ? m.rt_period_us / m.ref_period_us : NAN;
  m.rh_ratio = m.ref_period_us > 0.0f && m.rh_state_us > 0.0f
                   ? m.rh_state_us / m.ref_period_us : NAN;

  m.reject_reason = reject_reason(m);
  m.valid = m.reject_reason == RejectReason::NONE;
  if (!m.valid) {
    // Preserve the old behaviour: rejected captures report 0% quality and
    // expose a temperature estimate only when RT/REF itself is finite.
    m.temperature_c = std::isfinite(m.rt_ratio) ? calibration::temperature_from_ratio(m.rt_ratio) : NAN;
    m.temperature_extrapolation = true;
    m.humidity_extrapolation = true;
    m.calibration_extrapolation = true;
    return m;
  }

  m.quality_percent = quality_score(m);
  m.temperature_c = calibration::temperature_from_ratio(m.rt_ratio);
  m.rh_log = calibration::log_rh_ratio(m.rh_ratio);
  m.humidity_percent = calibration::humidity_from_ratio_temperature(m.rh_ratio, m.temperature_c);
  m.temperature_extrapolation = m.temperature_c < calibration::CAL_TEMP_MIN_C ||
                                m.temperature_c > calibration::CAL_TEMP_MAX_C;
  m.humidity_extrapolation = m.rh_ratio < calibration::CAL_RH_RATIO_MIN ||
                             m.rh_ratio > calibration::CAL_RH_RATIO_MAX;
  m.calibration_extrapolation = m.temperature_extrapolation || m.humidity_extrapolation;
  return m;
}

#if RTRH_DEBUG_CAPTURE
struct __attribute__((packed)) TimingPayload {
  uint32_t sequence;
  uint8_t valid;
  uint8_t reject_reason;
  uint8_t rh_state_samples;
  uint8_t flags;
  uint32_t rh_state_seen;
  float quality_percent;
  float ref_period_us;
  float rt_period_us;
  float rh_state_us;
  float temperature_c;
  float humidity_percent;
  uint16_t rh_min_us;
  uint16_t rh_p25_us;
  uint16_t rh_p75_us;
  uint16_t rh_max_us;
  uint8_t near_220;
  uint8_t near_440;
  uint8_t other;
  uint8_t reserved;
};

static TimingPayload debug_udp_timing_payload{};

static void debug_udp_timing_loop() {
  if (!debug_udp::enabled() || !debug_udp_timing_pending) return;
  const auto &payload = debug_udp_timing_payload;
  if (debug_udp::send_packet(debug_udp::PacketType::RTRH_TIMING, payload.sequence, 0, 1,
                             reinterpret_cast<const uint8_t *>(&payload), sizeof(payload)))
    debug_udp_timing_pending = false;
}

static void send_timing_udp(const Measurement &m) {
  if (!debug_udp::enabled()) return;
  TimingPayload payload{};
  payload.sequence = m.sequence;
  payload.valid = m.valid ? 1U : 0U;
  payload.reject_reason = static_cast<uint8_t>(m.reject_reason);
  payload.rh_state_samples = m.rh_state_samples;
  payload.flags = (m.thermal_transient ? 0x01U : 0U) |
                  (m.temperature_extrapolation ? 0x02U : 0U) |
                  (m.humidity_extrapolation ? 0x04U : 0U) |
                  (m.calibration_extrapolation ? 0x08U : 0U);
  payload.rh_state_seen = m.rh_state_seen;
  payload.quality_percent = m.quality_percent;
  payload.ref_period_us = m.ref_period_us;
  payload.rt_period_us = m.rt_period_us;
  payload.rh_state_us = m.rh_state_us;
  payload.temperature_c = m.temperature_c;
  payload.humidity_percent = m.humidity_percent;
  payload.rh_min_us = m.rh_state_min_us;
  payload.rh_p25_us = m.rh_state_p25_us;
  payload.rh_p75_us = m.rh_state_p75_us;
  payload.rh_max_us = m.rh_state_max_us;
  payload.near_220 = m.rh_state_near_220;
  payload.near_440 = m.rh_state_near_440;
  payload.other = m.rh_state_other;
  debug_udp_timing_payload = payload;
  debug_udp_timing_pending = true;
}

#endif

bool poll(Measurement &measurement) {
  if (!decoder.snapshot_ready || decoder.snapshot.sequence == decoder.last_polled_sequence) return false;
  measurement = derive(decoder.snapshot);
  decoder.latest_measurement = measurement;
  decoder.last_polled_sequence = measurement.sequence;
#if RTRH_DEBUG_CAPTURE
  send_timing_udp(measurement);
#endif
  return true;
}

void update_latest(const Measurement &measurement) { decoder.latest_measurement = measurement; }

#if RTRH_DEBUG_CAPTURE
#if defined(USE_WEB_SERVER_BASE)
class CaptureHandler : public web_server_idf::AsyncWebHandler {
 public:
  bool canHandle(web_server_idf::AsyncWebServerRequest *request) const override {
    if (request->method() != HTTP_GET) return false;
    char url[web_server_idf::AsyncWebServerRequest::URL_BUF_SIZE];
    return request->url_to(url) == "/rt_rh_capture.csv";
  }
  void handleRequest(web_server_idf::AsyncWebServerRequest *request) override {
    if (!debug.ready || !debug.sample_count) { request->send(204, "text/plain", nullptr); return; }
    httpd_req_t *req = *request;
    httpd_resp_set_status(req, "200 OK");
    httpd_resp_set_type(req, "text/csv");
    httpd_resp_set_hdr(req, "Content-Disposition", "attachment; filename=\"rt_rh_capture.csv\"");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    static constexpr char HEADER[] = "sequence,t_us,edge_no,gpio10,gpio13,state,overflow\n";
    esp_err_t err = httpd_resp_send_chunk(req, HEADER, sizeof(HEADER) - 1);
    const uint16_t count = debug.sample_count;
    const uint32_t sequence = debug.sequence;
    const bool did_overflow = debug.overflow;

    // The old 128-byte buffer caused hundreds of HTTP chunks for one capture.
    // Build ~1 KiB chunks on the heap instead; debug.ready keeps the ISR from
    // modifying the captured samples while this handler is streaming them.
    static constexpr size_t HTTP_CHUNK_BYTES = 1024;
    std::string chunk;
    chunk.reserve(HTTP_CHUNK_BYTES + 96);
    char line[96];
    for (uint16_t i = 0; i < count && err == ESP_OK; i++) {
      const uint8_t v = debug.samples[i].value;
      const int n = snprintf(line, sizeof(line), "%lu,%lu,%u,%u,%u,0x%02X,%u\n",
          static_cast<unsigned long>(sequence), static_cast<unsigned long>(debug.samples[i].t_us),
          static_cast<unsigned>(debug.samples[i].edge_no), (v & 0x01) ? 1U : 0U,
          (v & 0x08) ? 1U : 0U, v, did_overflow ? 1U : 0U);
      if (n <= 0) continue;
      const size_t len = static_cast<size_t>(n);
      if (!chunk.empty() && chunk.size() + len > HTTP_CHUNK_BYTES) {
        err = httpd_resp_send_chunk(req, chunk.data(), chunk.size());
        chunk.clear();
      }
      if (err == ESP_OK) chunk.append(line, len);
    }
    if (err == ESP_OK && !chunk.empty()) err = httpd_resp_send_chunk(req, chunk.data(), chunk.size());
    if (err == ESP_OK) err = httpd_resp_send_chunk(req, nullptr, 0);

    if (err == ESP_OK) {
      // Only consume the snapshot after a complete HTTP transfer. A disconnected
      // curl/client can retry without waiting for another RT/RH measurement.
      debug.sample_count = debug.edge_count = debug.unusual_count = 0;
      debug.overflow = debug.ready = debug.capturing = false;
      ESP_LOGD(TAG, "Downloaded RT/RH raw capture #%lu (%u samples)",
               static_cast<unsigned long>(sequence), static_cast<unsigned>(count));
    } else {
      ESP_LOGW(TAG, "rt_rh_capture.csv send failed (%d/0x%04X); capture #%lu preserved for retry",
               static_cast<int>(err), static_cast<unsigned>(err),
               static_cast<unsigned long>(sequence));
    }
  }
};
static CaptureHandler capture_handler;

static void format_fixed(char *out, size_t out_size, float value, unsigned decimals) {
  if (!std::isfinite(value)) {
    snprintf(out, out_size, "nan");
    return;
  }
  int32_t scale = 1;
  for (unsigned i = 0; i < decimals; i++) scale *= 10;
  const int32_t scaled = static_cast<int32_t>(lroundf(value * static_cast<float>(scale)));
  const bool negative = scaled < 0;
  const uint32_t magnitude = negative ? static_cast<uint32_t>(-static_cast<int64_t>(scaled))
                                      : static_cast<uint32_t>(scaled);
  const uint32_t whole = magnitude / static_cast<uint32_t>(scale);
  const uint32_t fraction = magnitude % static_cast<uint32_t>(scale);
  if (decimals == 0) {
    snprintf(out, out_size, "%s%lu", negative ? "-" : "", static_cast<unsigned long>(whole));
  } else {
    snprintf(out, out_size, "%s%lu.%0*lu", negative ? "-" : "",
             static_cast<unsigned long>(whole), static_cast<int>(decimals),
             static_cast<unsigned long>(fraction));
  }
}

class TimingHandler : public web_server_idf::AsyncWebHandler {
 public:
  bool canHandle(web_server_idf::AsyncWebServerRequest *request) const override {
    if (request->method() != HTTP_GET) return false;
    char url[web_server_idf::AsyncWebServerRequest::URL_BUF_SIZE];
    return request->url_to(url) == "/rt_rh_timing.csv";
  }
  void handleRequest(web_server_idf::AsyncWebServerRequest *request) override {
    if (!decoder.snapshot_ready) { request->send(204, "text/plain", nullptr); return; }
    const Snapshot s = decoder.snapshot;
    const float ref_us = s.ref.count ? float(s.ref.period_sum) / s.ref.count : 0.0f;
    const float rt_us = s.rt.count ? float(s.rt.period_sum) / s.rt.count : 0.0f;
    const float rh_us = s.rh_timing.count ? float(s.rh_timing.period_sum) / s.rh_timing.count : 0.0f;
    const float rh_state_us = rh_state_period_median(s.rh_state);
    const Measurement d = decoder.latest_measurement;
    const bool have = d.sequence == s.sequence;

    // Avoid newlib's floating-point printf path (_dtoa_r) in the ESP-IDF
    // HTTP server task.  Convert floats to short fixed-point strings first,
    // then serialize the CSV using integer/string printf only.
    char ref_period[20], ref_duration[20], rt_period[20], rt_duration[20];
    char rh_period[20], rh_duration[20], rh_state[20];
    char rt_ratio[24], rh_ratio[24], temperature[20], humidity[20], quality[16];
    format_fixed(ref_period, sizeof(ref_period), ref_us, 3);
    format_fixed(ref_duration, sizeof(ref_duration), float(s.ref.period_sum) / 1000.0f, 3);
    format_fixed(rt_period, sizeof(rt_period), rt_us, 3);
    format_fixed(rt_duration, sizeof(rt_duration), float(s.rt.period_sum) / 1000.0f, 3);
    format_fixed(rh_period, sizeof(rh_period), rh_us, 3);
    format_fixed(rh_duration, sizeof(rh_duration), float(s.rh_timing.period_sum) / 1000.0f, 3);
    format_fixed(rh_state, sizeof(rh_state), rh_state_us, 3);
    format_fixed(rt_ratio, sizeof(rt_ratio), have ? d.rt_ratio : NAN, 6);
    format_fixed(rh_ratio, sizeof(rh_ratio), have ? d.rh_ratio : NAN, 6);
    format_fixed(temperature, sizeof(temperature), have ? d.temperature_c : NAN, 3);
    format_fixed(humidity, sizeof(humidity), have ? d.humidity_percent : NAN, 3);
    format_fixed(quality, sizeof(quality), have ? d.quality_percent : 0.0f, 1);

    httpd_req_t *req = *request;
    httpd_resp_set_status(req, "200 OK");
    httpd_resp_set_type(req, "text/csv");
    httpd_resp_set_hdr(req, "Content-Disposition", "attachment; filename=\"rt_rh_timing.csv\"");

    static constexpr char HEADER[] =
        "measurement,phase,count,period_mean_us,duration_ms,state_rh_median_us,state_rh_samples,state_rh_seen,valid,rt_ratio,rh_ratio,temperature_c,humidity_percent,quality_percent,reject_reason,thermal_transient,temperature_extrapolation,humidity_extrapolation,calibration_extrapolation\n";
    esp_err_t err = httpd_resp_send_chunk(req, HEADER, sizeof(HEADER) - 1);
    char line[384];

    if (err == ESP_OK) {
      const int n = snprintf(line, sizeof(line), "%lu,ref,%u,%s,%s,,,,,,,,,,,,,,,\n",
          static_cast<unsigned long>(s.sequence), static_cast<unsigned>(s.ref.count),
          ref_period, ref_duration);
      if (n <= 0 || static_cast<size_t>(n) >= sizeof(line)) err = ESP_FAIL;
      else err = httpd_resp_send_chunk(req, line, n);
    }
    if (err == ESP_OK) {
      const int n = snprintf(line, sizeof(line), "%lu,rt,%u,%s,%s,,,,,,,,,,,,,,,\n",
          static_cast<unsigned long>(s.sequence), static_cast<unsigned>(s.rt.count),
          rt_period, rt_duration);
      if (n <= 0 || static_cast<size_t>(n) >= sizeof(line)) err = ESP_FAIL;
      else err = httpd_resp_send_chunk(req, line, n);
    }
    if (err == ESP_OK) {
      const int n = snprintf(line, sizeof(line),
          "%lu,rh,%u,%s,%s,%s,%u,%lu,%u,%s,%s,%s,%s,%s,%s,%u,%u,%u,%u\n",
          static_cast<unsigned long>(s.sequence), static_cast<unsigned>(s.rh_timing.count),
          rh_period, rh_duration, rh_state, static_cast<unsigned>(s.rh_state.sample_count),
          static_cast<unsigned long>(s.rh_state.seen), have && d.valid ? 1U : 0U, rt_ratio, rh_ratio, temperature, humidity, quality,
          have ? reject_reason_to_string(d.reject_reason) : "UNKNOWN",
          have && d.thermal_transient ? 1U : 0U, have && d.temperature_extrapolation ? 1U : 0U,
          have && d.humidity_extrapolation ? 1U : 0U, have && d.calibration_extrapolation ? 1U : 0U);
      if (n <= 0 || static_cast<size_t>(n) >= sizeof(line)) err = ESP_FAIL;
      else err = httpd_resp_send_chunk(req, line, n);
    }
    if (err == ESP_OK) err = httpd_resp_send_chunk(req, nullptr, 0);
    if (err != ESP_OK) ESP_LOGW(TAG, "rt_rh_timing.csv send failed/client disconnected (%d)", err);
  }
};
static TimingHandler timing_handler;
#endif

void register_debug_handlers() {
#if defined(USE_WEB_SERVER_BASE)
  if (!web_server_base::global_web_server_base) {
    ESP_LOGW(TAG, "web_server_base unavailable");
    return;
  }
  web_server_base::global_web_server_base->add_handler(&capture_handler);
  web_server_base::global_web_server_base->add_handler(&timing_handler);
#else
  ESP_LOGD(TAG, "RT/RH debug capture instrumentation enabled without HTTP endpoints");
#endif
}
#endif

}  // namespace rtrh_decoder
}  // namespace co2_monitor_0601
}  // namespace esphome
