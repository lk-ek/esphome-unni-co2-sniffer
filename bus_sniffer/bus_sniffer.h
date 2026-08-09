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

  // GPIO10..13: temporary raw ADC diagnostics for RT/RH reverse engineering.
  void set_probe_sensor(uint8_t index, sensor::Sensor *sensor) {
    if (index < 4)
      this->probe_sensors_[index] = sensor;
  }

  void set_adc_read_errors_sensor(sensor::Sensor *sensor) {
    this->adc_read_errors_sensor_ = sensor;
  }

 protected:
  sensor::Sensor *co2_sensor_{nullptr};
  sensor::Sensor *crc_errors_sensor_{nullptr};
  sensor::Sensor *frame_errors_sensor_{nullptr};

  sensor::Sensor *probe_sensors_[4]{nullptr, nullptr, nullptr, nullptr};
  sensor::Sensor *adc_read_errors_sensor_{nullptr};

  bool have_last_ppm_{false};
  uint16_t last_ppm_{0};

  uint32_t crc_errors_{0};
  uint32_t frame_errors_{0};
  uint32_t adc_read_errors_{0};

  void sample_adc_probes_();
  void publish_adc_probes_();
};

}  // namespace bus_sniffer
}  // namespace esphome
