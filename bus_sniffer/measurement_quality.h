#pragma once

#include <cmath>
#include <cstdint>

namespace esphome {
namespace bus_sniffer {
namespace measurement_quality {

enum class RejectReason : uint8_t {
  NONE = 0,
  REF_PERIOD,
  REF_DURATION,
  RT_PERIOD,
  RT_DURATION,
  RT_COUNT,
  RH_DURATION,
  RH_TOO_FEW_SAMPLES,
  RH_STATE_PERIOD,
  RH_RATIO_IMPLAUSIBLE,
};

inline const char *reject_reason_to_string(RejectReason reason) {
  switch (reason) {
    case RejectReason::NONE:
      return "NONE";
    case RejectReason::REF_PERIOD:
      return "REF_PERIOD";
    case RejectReason::REF_DURATION:
      return "REF_DURATION";
    case RejectReason::RT_PERIOD:
      return "RT_PERIOD";
    case RejectReason::RT_DURATION:
      return "RT_DURATION";
    case RejectReason::RT_COUNT:
      return "RT_COUNT";
    case RejectReason::RH_DURATION:
      return "RH_DURATION";
    case RejectReason::RH_TOO_FEW_SAMPLES:
      return "RH_TOO_FEW_SAMPLES";
    case RejectReason::RH_STATE_PERIOD:
      return "RH_STATE_PERIOD";
    case RejectReason::RH_RATIO_IMPLAUSIBLE:
      return "RH_RATIO_IMPLAUSIBLE";
  }
  return "UNKNOWN";
}

struct Inputs {
  float ref_period_us{NAN};
  float ref_duration_ms{NAN};
  float rt_period_us{NAN};
  float rt_duration_ms{NAN};
  uint16_t rt_count{0};

  float rh_duration_ms{NAN};
  float rh_state_us{NAN};
  uint8_t rh_state_samples{0};
  uint32_t rh_state_seen{0};

  float rh_ratio{NAN};
};

struct Result {
  bool valid{false};
  float score_percent{0.0f};
  RejectReason reason{RejectReason::NONE};
};

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

inline Result evaluate(const Inputs &in) {
  Result out;

  if (!std::isfinite(in.ref_period_us) ||
      in.ref_period_us < REF_PERIOD_MIN_US ||
      in.ref_period_us > REF_PERIOD_MAX_US) {
    out.reason = RejectReason::REF_PERIOD;
    return out;
  }
  if (!std::isfinite(in.ref_duration_ms) ||
      in.ref_duration_ms < REF_DURATION_MIN_MS ||
      in.ref_duration_ms > REF_DURATION_MAX_MS) {
    out.reason = RejectReason::REF_DURATION;
    return out;
  }
  if (!std::isfinite(in.rt_period_us) ||
      in.rt_period_us < RT_PERIOD_MIN_US ||
      in.rt_period_us > RT_PERIOD_MAX_US) {
    out.reason = RejectReason::RT_PERIOD;
    return out;
  }
  if (!std::isfinite(in.rt_duration_ms) ||
      in.rt_duration_ms < RT_DURATION_MIN_MS ||
      in.rt_duration_ms > RT_DURATION_MAX_MS) {
    out.reason = RejectReason::RT_DURATION;
    return out;
  }
  if (in.rt_count < RT_COUNT_MIN) {
    out.reason = RejectReason::RT_COUNT;
    return out;
  }
  if (!std::isfinite(in.rh_duration_ms) ||
      in.rh_duration_ms < RH_DURATION_MIN_MS ||
      in.rh_duration_ms > RH_DURATION_MAX_MS) {
    out.reason = RejectReason::RH_DURATION;
    return out;
  }
  if (in.rh_state_samples < RH_STATE_SAMPLES_MIN) {
    out.reason = RejectReason::RH_TOO_FEW_SAMPLES;
    return out;
  }
  if (!std::isfinite(in.rh_state_us) ||
      in.rh_state_us < RH_STATE_MIN_US ||
      in.rh_state_us > RH_STATE_MAX_US) {
    out.reason = RejectReason::RH_STATE_PERIOD;
    return out;
  }
  if (!std::isfinite(in.rh_ratio) ||
      in.rh_ratio <= 0.0f ||
      in.rh_ratio > RH_RATIO_VALID_MAX) {
    out.reason = RejectReason::RH_RATIO_IMPLAUSIBLE;
    return out;
  }

  // Score independent from acceptance thresholds: enough to compare "good"
  // valid captures while retaining a deterministic reject reason.
  const float ref_center = 76.75f;
  const float ref_score =
      std::fmax(0.0f, 1.0f - std::fabs(in.ref_period_us - ref_center) / 2.0f);
  const float rt_score =
      std::fmin(1.0f, static_cast<float>(in.rt_count) / 880.0f);
  const float rh_fill_score =
      std::fmin(1.0f, static_cast<float>(in.rh_state_samples) / 96.0f);

  float rh_seen_score = 1.0f;
  if (in.rh_state_seen > 0) {
    // Normal stable captures have many observations relative to retained
    // median samples. This only gently penalizes sparse/aliased captures.
    const float ratio =
        static_cast<float>(in.rh_state_seen) /
        static_cast<float>(in.rh_state_samples);
    rh_seen_score = std::fmin(1.0f, ratio / 2.0f);
  }

  out.valid = true;
  out.reason = RejectReason::NONE;
  out.score_percent =
      100.0f *
      (0.25f * ref_score +
       0.30f * rt_score +
       0.30f * rh_fill_score +
       0.15f * rh_seen_score);
  return out;
}

}  // namespace measurement_quality
}  // namespace bus_sniffer
}  // namespace esphome
