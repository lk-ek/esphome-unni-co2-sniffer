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

 protected:
  sensor::Sensor *co2_sensor_{nullptr};
};

}  // namespace bus_sniffer
}  // namespace esphome
