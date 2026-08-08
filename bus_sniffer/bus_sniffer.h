#pragma once

#include "esphome/core/component.h"

namespace esphome {
namespace bus_sniffer {

class BusSniffer : public Component {
 public:
  void setup() override;
  void loop() override;
};

}  // namespace bus_sniffer
}  // namespace esphome

