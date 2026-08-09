#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"

namespace esphome {
namespace bus_sniffer {

class BusSniffer : public Component {
 public:
  void setup() override;
  void loop() override;

  void set_co2_sensor(sensor::Sensor *sensor) {
    this->co2_sensor_ = sensor;
  }

  void set_crc_errors_sensor(sensor::Sensor *sensor) {
    this->crc_errors_sensor_ = sensor;
  }

  void set_frame_errors_sensor(sensor::Sensor *sensor) {
    this->frame_errors_sensor_ = sensor;
  }

 protected:
  // ESPHome entities
  sensor::Sensor *co2_sensor_{nullptr};
  sensor::Sensor *crc_errors_sensor_{nullptr};
  sensor::Sensor *frame_errors_sensor_{nullptr};

  // Last successfully published CO2 value
  bool have_last_ppm_{false};
  uint16_t last_ppm_{0};

  // Diagnostics
  uint32_t crc_errors_{0};
  uint32_t frame_errors_{0};
};

}  // namespace bus_sniffer
}  // namespace esphome
