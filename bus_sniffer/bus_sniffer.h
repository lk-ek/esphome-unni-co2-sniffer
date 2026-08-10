#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include <esp_gatts_api.h>
#include "esphome/components/esp32_ble_server/ble_server.h"

namespace esphome {
namespace bus_sniffer {

class BusSniffer : public Component {
 public:
  void setup() override;
  void loop() override;
  void configure_gatt_server(esp32_ble_server::BLEServer *server);

  void gatts_event_handler(esp_gatts_cb_event_t event,
                           esp_gatt_if_t gatts_if,
                           esp_ble_gatts_cb_param_t *param);

  void set_co2_sensor(sensor::Sensor *sensor) {
    this->co2_sensor_ = sensor;
  }

  void set_crc_errors_sensor(sensor::Sensor *sensor) {
    this->crc_errors_sensor_ = sensor;
  }

  void set_frame_errors_sensor(sensor::Sensor *sensor) {
    this->frame_errors_sensor_ = sensor;
  }

  void set_rt_temperature_sensor(sensor::Sensor *sensor) {
    this->rt_temperature_sensor_ = sensor;
  }

  void set_rh_humidity_sensor(sensor::Sensor *sensor) {
    this->rh_humidity_sensor_ = sensor;
  }

 protected:
  sensor::Sensor *co2_sensor_{nullptr};
  sensor::Sensor *crc_errors_sensor_{nullptr};
  sensor::Sensor *frame_errors_sensor_{nullptr};
  sensor::Sensor *rt_temperature_sensor_{nullptr};
  sensor::Sensor *rh_humidity_sensor_{nullptr};

  bool have_last_ppm_{false};
  uint16_t last_ppm_{0};

  uint32_t crc_errors_{0};
  uint32_t frame_errors_{0};
};

}  // namespace bus_sniffer
}  // namespace esphome
