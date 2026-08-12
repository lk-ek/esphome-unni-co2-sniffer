// SPDX-License-Identifier: GPL-3.0-or-later
#include "rtrh_decoder.h"

#include "calibration.h"
#include "esphome/core/log.h"
#include "driver/gpio.h"
#include "esp_timer.h"

#include <cstdio>
#include <cstring>

#if RTRH_DEBUG_CAPTURE
#include "esphome/components/web_server_base/web_server_base.h"
#include "esp_http_server.h"
#endif

namespace esphome {
namespace bus_sniffer {
namespace rtrh_decoder {

static const char *TAG = "rtrh_decoder";
static constexpr gpio_num_t PIN_G10 = GPIO_NUM_3;
static constexpr gpio_num_t PIN_G11 = GPIO_NUM_5;
static constexpr gpio_num_t PIN_G13 = GPIO_NUM_4;
static constexpr gpio_num_t PINS[] = {PIN_G10, PIN_G11, PIN_G13};

// The controller spends ~125 ms in REF and ~127 ms in RT. Phase identity is
// deliberately based on elapsed time, never on RC period or cycle count.
static constexpr uint32_t MEASUREMENT_QUIET_US = 15000000;
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

enum Phase : uint8_t { WAIT_REF = 0, REF, RT, RH };

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

static volatile bool collecting = false;
static volatile uint32_t measurement_start_us = 0;
static volatile uint32_t last_any_us = 0;
static volatile uint8_t last_state = 0;
static volatile uint32_t last_g10_fall_us = 0;
static volatile bool have_g10_rise = false;
static volatile uint8_t phase = WAIT_REF;
static Accum ref;
static Accum rt;
static Accum rh_timing;
static volatile uint32_t rt_temp_period_sum = 0;
static volatile uint16_t rt_temp_count = 0;
static RhStateStats rh_state;
static Snapshot latest_snapshot;
static volatile bool snapshot_ready = false;
static volatile uint8_t pin_level[3] = {0, 0, 0};
static Measurement latest_measurement;

static inline uint8_t IRAM_ATTR read_state() {
  uint8_t value = 0;
  if (gpio_get_level(PIN_G10)) value |= 0x01;
  if (gpio_get_level(PIN_G11)) value |= 0x02;
  if (gpio_get_level(PIN_G13)) value |= 0x08;
  return value;
}

static inline void IRAM_ATTR clear_accum(Accum &a) {
  a.period_sum = 0;
  a.count = 0;
}

static inline void IRAM_ATTR clear_rh_state() {
  rh_state.last_us = 0;
  rh_state.write_pos = 0;
  rh_state.sample_count = 0;
  rh_state.seen = 0;
  for (uint8_t i = 0; i < RH_STATE_PERIOD_SAMPLES; i++) rh_state.samples[i] = 0;
}

static inline void IRAM_ATTR reset_measurement(uint32_t now, uint8_t state) {
  collecting = true;
  measurement_start_us = now;
  last_any_us = now;
  last_state = state;
  last_g10_fall_us = 0;
  have_g10_rise = false;
  phase = REF;
  clear_accum(ref);
  clear_accum(rt);
  clear_accum(rh_timing);
  rt_temp_period_sum = 0;
  rt_temp_count = 0;
  clear_rh_state();
}

static inline void IRAM_ATTR add_period(Accum &a, uint32_t period) {
  if (a.count != 0xFFFF) a.count++;
  a.period_sum += period;
}

static inline void IRAM_ATTR update_phase(uint32_t now) {
  const uint32_t elapsed = static_cast<uint32_t>(now - measurement_start_us);
  uint8_t next = elapsed < REF_PHASE_END_US ? REF :
                 elapsed < RT_PHASE_END_US ? RT : RH;
  if (next == phase) return;
  phase = next;
  last_g10_fall_us = 0;  // Never let a period cross a fixed phase boundary.
  have_g10_rise = false;
  if (next == RH) rh_state.last_us = 0;
}

static inline void IRAM_ATTR observe_rh_state(uint32_t now, uint8_t state) {
  // Characteristic RH state: G10=0, G11=0, G13=1.
  if (phase != RH || (state & 0x0B) != 0x08) return;
  if (rh_state.last_us != 0) {
    const uint32_t dt = static_cast<uint32_t>(now - rh_state.last_us);
    if (dt >= 40 && dt <= 60000) {
      rh_state.samples[rh_state.write_pos] = static_cast<uint16_t>(dt);
      rh_state.write_pos = static_cast<uint8_t>((rh_state.write_pos + 1) % RH_STATE_PERIOD_SAMPLES);
      if (rh_state.sample_count < RH_STATE_PERIOD_SAMPLES) rh_state.sample_count++;
    }
  }
  rh_state.last_us = now;
  rh_state.seen++;
}

static float rh_state_period_median(const RhStateStats &s) {
  const uint8_t n = s.sample_count;
  if (!n) return 0.0f;
  uint16_t tmp[RH_STATE_PERIOD_SAMPLES];
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
  if (n & 1) return static_cast<float>(tmp[n / 2]);
  return 0.5f * (static_cast<float>(tmp[n / 2 - 1]) + static_cast<float>(tmp[n / 2]));
}

static void finalize_measurement() {
  if (!collecting) return;
  if (!ref.count || !rt.count || !rh_timing.count) {
    collecting = false;
    return;
  }
  Snapshot next{};
  next.ref = ref;
  next.rt = rt;
  next.rh_timing = rh_timing;
  next.rt_temp_period_sum = rt_temp_period_sum;
  next.rt_temp_count = rt_temp_count;
  next.rh_state = rh_state;
  next.sequence = latest_snapshot.sequence + 1;
  latest_snapshot = next;
  snapshot_ready = true;
  collecting = false;
}

#if RTRH_DEBUG_CAPTURE
static constexpr uint32_t CAPTURE_US = 450000;
static constexpr uint16_t MAX_SAMPLES = 1536;
static constexpr uint32_t CAPTURE_DECIMATION = 16;
static constexpr uint32_t CAPTURE_UNUSUAL_DECIMATION = 8;
struct __attribute__((packed)) DebugSample { uint32_t t_us; uint16_t edge_no; uint8_t value; };
static volatile DebugSample samples[MAX_SAMPLES];
static volatile uint16_t sample_count = 0;
static volatile uint16_t capture_edge_no = 0;
static volatile uint16_t capture_unusual_no = 0;
static volatile uint8_t debug_last_value = 0xff;
static volatile uint32_t capture_start_us = 0;
static volatile bool capturing = false;
static volatile bool capture_ready = false;
static volatile bool overflow = false;
static volatile uint32_t capture_sequence = 0;
#endif

static void IRAM_ATTR gpio_isr(void *arg) {
  const intptr_t encoded = reinterpret_cast<intptr_t>(arg);
  if (encoded < 1 || encoded > 3) return;
  const uint8_t pin_index = static_cast<uint8_t>(encoded - 1);
  const gpio_num_t pin = PINS[pin_index];
  const uint8_t level = static_cast<uint8_t>(gpio_get_level(pin));
  const uint8_t previous = pin_level[pin_index];
  if (level == previous) return;
  pin_level[pin_index] = level;

  const uint32_t now = static_cast<uint32_t>(esp_timer_get_time());
  const uint8_t state = read_state();
  if (!collecting) reset_measurement(now, state);
  else last_any_us = now;
  update_phase(now);

  // REF/RT timing comes only from the physical G10 IRQ. This avoids ordering
  // errors when several sensor lines change almost simultaneously.
  const bool is_g10_irq = pin_index == 0;
  if (is_g10_irq && previous == 0 && level != 0) have_g10_rise = true;
  if (is_g10_irq && previous != 0 && level == 0) {
    const uint32_t previous_fall = last_g10_fall_us;
    last_g10_fall_us = now;
    if (previous_fall && have_g10_rise) {
      const uint32_t period = static_cast<uint32_t>(now - previous_fall);
      if (period <= CYCLE_MAX_US) {
        if (phase == REF) add_period(ref, period);
        else if (phase == RT) {
          add_period(rt, period);
          if (rt_temp_count < RT_TEMP_CYCLES) {
            rt_temp_period_sum += period;
            rt_temp_count++;
          }
        } else if (phase == RH) {
          add_period(rh_timing, period);
        }
      }
    }
    have_g10_rise = false;
  }

  if (state != last_state) {
    observe_rh_state(now, state);
    last_state = state;
  }

#if RTRH_DEBUG_CAPTURE
  if (capture_ready) return;
  const uint8_t value = read_state();
  if (value == debug_last_value) return;
  debug_last_value = value;
  if (!capturing) {
    capturing = true;
    capture_start_us = now;
    sample_count = capture_edge_no = capture_unusual_no = 0;
    overflow = false;
  }
  const uint16_t edge_no = capture_edge_no++;
  const bool unusual = value != 0x00 && value != 0x0F;
  const bool time_anchor = edge_no == 0 || (edge_no % CAPTURE_DECIMATION) == 0;
  bool unusual_anchor = false;
  if (unusual) {
    const uint16_t unusual_no = capture_unusual_no++;
    unusual_anchor = unusual_no == 0 || (unusual_no % CAPTURE_UNUSUAL_DECIMATION) == 0;
  }
  if (!time_anchor && !unusual_anchor) return;
  const uint16_t index = sample_count;
  if (index >= MAX_SAMPLES) {
    overflow = true;
    capturing = false;
    capture_ready = true;
    capture_sequence++;
    return;
  }
  samples[index].t_us = static_cast<uint32_t>(now - capture_start_us);
  samples[index].edge_no = edge_no;
  samples[index].value = value;
  sample_count = index + 1;
#endif
}

bool setup() {
  gpio_config_t io{};
  io.mode = GPIO_MODE_INPUT;
  io.pull_up_en = GPIO_PULLUP_DISABLE;
  io.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io.intr_type = GPIO_INTR_DISABLE;
  io.pin_bit_mask = (1ULL << PIN_G10) | (1ULL << PIN_G11) | (1ULL << PIN_G13);
  esp_err_t err = gpio_config(&io);
  if (err != ESP_OK) return false;

  for (gpio_num_t pin : PINS) gpio_set_intr_type(pin, GPIO_INTR_ANYEDGE);
  last_state = read_state();
  for (uint8_t i = 0; i < 3; i++) pin_level[i] = gpio_get_level(PINS[i]);
#if RTRH_DEBUG_CAPTURE
  debug_last_value = read_state();
#endif

  for (uint8_t i = 0; i < 3; i++) {
    err = gpio_isr_handler_add(PINS[i], gpio_isr,
        reinterpret_cast<void *>(static_cast<intptr_t>(i + 1)));
    if (err != ESP_OK) return false;
  }
  return true;
}

void loop() {
  // A measurement is complete after the sensor has been silent for 15 s.
  if (collecting) {
    const uint32_t now = static_cast<uint32_t>(esp_timer_get_time());
    const uint32_t last = last_any_us;
    if (last && static_cast<uint32_t>(now - last) > MEASUREMENT_QUIET_US) {
      for (gpio_num_t pin : PINS) gpio_intr_disable(pin);
      const uint32_t now2 = static_cast<uint32_t>(esp_timer_get_time());
      const uint32_t last2 = last_any_us;
      if (collecting && last2 && static_cast<uint32_t>(now2 - last2) > MEASUREMENT_QUIET_US)
        finalize_measurement();
      last_state = read_state();
      for (uint8_t i = 0; i < 3; i++) pin_level[i] = gpio_get_level(PINS[i]);
      for (gpio_num_t pin : PINS) gpio_intr_enable(pin);
    }
  }

#if RTRH_DEBUG_CAPTURE
  if (capturing && !capture_ready) {
    const uint32_t now = static_cast<uint32_t>(esp_timer_get_time());
    if (static_cast<uint32_t>(now - capture_start_us) >= CAPTURE_US) {
      capturing = false;
      capture_ready = true;
      capture_sequence++;
      ESP_LOGI(TAG, "RT/RH edge capture ready: %u events, sequence %lu%s",
               sample_count, static_cast<unsigned long>(capture_sequence),
               overflow ? " OVERFLOW" : "");
    }
  }
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

bool poll(Measurement &measurement) {
  static uint32_t last_sequence = 0;
  if (!snapshot_ready || latest_snapshot.sequence == last_sequence) return false;
  measurement = derive(latest_snapshot);
  latest_measurement = measurement;
  last_sequence = measurement.sequence;
  return true;
}

void update_latest(const Measurement &measurement) { latest_measurement = measurement; }

#if RTRH_DEBUG_CAPTURE
class CaptureHandler : public web_server_idf::AsyncWebHandler {
 public:
  bool canHandle(web_server_idf::AsyncWebServerRequest *request) const override {
    if (request->method() != HTTP_GET) return false;
    char url[web_server_idf::AsyncWebServerRequest::URL_BUF_SIZE];
    return request->url_to(url) == "/rt_rh_capture.csv";
  }
  void handleRequest(web_server_idf::AsyncWebServerRequest *request) override {
    if (!capture_ready || !sample_count) { request->send(204, "text/plain", nullptr); return; }
    httpd_req_t *req = *request;
    httpd_resp_set_status(req, "200 OK");
    httpd_resp_set_type(req, "text/csv");
    httpd_resp_set_hdr(req, "Content-Disposition", "attachment; filename=\"rt_rh_capture.csv\"");
    static constexpr char HEADER[] = "sequence,t_us,edge_no,gpio10,gpio11,gpio13,state,overflow\n";
    esp_err_t err = httpd_resp_send_chunk(req, HEADER, sizeof(HEADER) - 1);
    const uint16_t count = sample_count;
    const uint32_t sequence = capture_sequence;
    const bool did_overflow = overflow;
    char chunk[128], line[80];
    size_t used = 0;
    for (uint16_t i = 0; i < count && err == ESP_OK; i++) {
      const uint8_t v = samples[i].value;
      const int n = snprintf(line, sizeof(line), "%lu,%lu,%u,%u,%u,%u,0x%02X,%u\n",
          static_cast<unsigned long>(sequence), static_cast<unsigned long>(samples[i].t_us),
          static_cast<unsigned>(samples[i].edge_no), (v & 0x01) ? 1U : 0U,
          (v & 0x02) ? 1U : 0U, (v & 0x08) ? 1U : 0U, v, did_overflow ? 1U : 0U);
      if (n <= 0) continue;
      const size_t len = static_cast<size_t>(n);
      if (used + len > sizeof(chunk)) { err = httpd_resp_send_chunk(req, chunk, used); used = 0; }
      if (err == ESP_OK && len <= sizeof(chunk)) { memcpy(chunk + used, line, len); used += len; }
    }
    if (err == ESP_OK && used) err = httpd_resp_send_chunk(req, chunk, used);
    if (err == ESP_OK) httpd_resp_send_chunk(req, nullptr, 0);
    sample_count = capture_edge_no = capture_unusual_no = 0;
    overflow = capture_ready = capturing = false;
    if (err != ESP_OK) ESP_LOGW(TAG, "rt_rh_capture.csv client disconnected (%d)", err);
  }
};
static CaptureHandler capture_handler;

class TimingHandler : public web_server_idf::AsyncWebHandler {
 public:
  bool canHandle(web_server_idf::AsyncWebServerRequest *request) const override {
    if (request->method() != HTTP_GET) return false;
    char url[web_server_idf::AsyncWebServerRequest::URL_BUF_SIZE];
    return request->url_to(url) == "/rt_rh_timing.csv";
  }
  void handleRequest(web_server_idf::AsyncWebServerRequest *request) override {
    if (!snapshot_ready) { request->send(204, "text/plain", nullptr); return; }
    const Snapshot s = latest_snapshot;
    const float ref_us = s.ref.count ? float(s.ref.period_sum) / s.ref.count : 0.0f;
    const float rt_us = s.rt.count ? float(s.rt.period_sum) / s.rt.count : 0.0f;
    const float rh_us = s.rh_timing.count ? float(s.rh_timing.period_sum) / s.rh_timing.count : 0.0f;
    const float rh_state_us = rh_state_period_median(s.rh_state);
    const Measurement d = latest_measurement;
    const bool have = d.sequence == s.sequence;
    char body[1024];
    const int n = snprintf(body, sizeof(body),
        "measurement,phase,count,period_mean_us,duration_ms,state_rh_median_us,state_rh_samples,state_rh_seen,valid,rt_ratio,rh_ratio,temperature_c,humidity_percent,quality_percent,reject_reason,thermal_transient,temperature_extrapolation,humidity_extrapolation,calibration_extrapolation\n"
        "%lu,ref,%u,%.3f,%.3f,,,,,,,,,,,,,,,\n"
        "%lu,rt,%u,%.3f,%.3f,,,,,,,,,,,,,,,\n"
        "%lu,rh,%u,%.3f,%.3f,%.3f,%u,%lu,%u,%.6f,%.6f,%.3f,%.3f,%.1f,%s,%u,%u,%u,%u\n",
        static_cast<unsigned long>(s.sequence), static_cast<unsigned>(s.ref.count), ref_us, float(s.ref.period_sum)/1000.0f,
        static_cast<unsigned long>(s.sequence), static_cast<unsigned>(s.rt.count), rt_us, float(s.rt.period_sum)/1000.0f,
        static_cast<unsigned long>(s.sequence), static_cast<unsigned>(s.rh_timing.count), rh_us, float(s.rh_timing.period_sum)/1000.0f,
        rh_state_us, static_cast<unsigned>(s.rh_state.sample_count), static_cast<unsigned long>(s.rh_state.seen),
        have && d.valid ? 1U : 0U, have ? d.rt_ratio : NAN, have ? d.rh_ratio : NAN,
        have ? d.temperature_c : NAN, have ? d.humidity_percent : NAN, have ? d.quality_percent : 0.0f,
        have ? reject_reason_to_string(d.reject_reason) : "UNKNOWN",
        have && d.thermal_transient ? 1U : 0U, have && d.temperature_extrapolation ? 1U : 0U,
        have && d.humidity_extrapolation ? 1U : 0U, have && d.calibration_extrapolation ? 1U : 0U);
    if (n <= 0) { request->send(500, "text/plain", nullptr); return; }
    httpd_req_t *req = *request;
    httpd_resp_set_status(req, "200 OK");
    httpd_resp_set_type(req, "text/csv");
    httpd_resp_set_hdr(req, "Content-Disposition", "attachment; filename=\"rt_rh_timing.csv\"");
    httpd_resp_send(req, body, static_cast<ssize_t>(n));
  }
};
static TimingHandler timing_handler;

void register_debug_handlers() {
  if (!web_server_base::global_web_server_base) {
    ESP_LOGW(TAG, "web_server_base unavailable");
    return;
  }
  web_server_base::global_web_server_base->add_handler(&capture_handler);
  web_server_base::global_web_server_base->add_handler(&timing_handler);
}
#endif

}  // namespace rtrh_decoder
}  // namespace bus_sniffer
}  // namespace esphome
