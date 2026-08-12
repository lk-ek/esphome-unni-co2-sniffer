// SPDX-License-Identifier: GPL-3.0-or-later
#include "power_save.h"

#include "driver/gpio.h"
#include "esp_pm.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "esphome/core/log.h"

namespace esphome {
namespace bus_sniffer {
namespace power_save {

static const char *TAG = "unni_power";
static constexpr gpio_num_t PIN_G10 = GPIO_NUM_3;
static constexpr gpio_num_t PIN_G13 = GPIO_NUM_4;

static bool configured = false;
static esp_pm_lock_handle_t awake_lock = nullptr;
static esp_pm_lock_handle_t cpu_lock = nullptr;
static volatile bool lock_held = false;
static volatile uint64_t wake_started_us = 0;
static volatile bool rtrh_complete = false;
static volatile bool co2_after_rtrh = false;
static uint32_t max_awake_ms = 10000;
static uint32_t completed_cycles = 0;
static uint32_t timeout_cycles = 0;

static void configure_wakeup_pin(gpio_num_t pin) {
  // Wake on the opposite of the current idle level. RT/RH lines spend almost
  // all of their time idle, so the first transition of the next cycle wakes us.
  const int idle = gpio_get_level(pin);
  const gpio_int_type_t wake_level = idle ? GPIO_INTR_LOW_LEVEL : GPIO_INTR_HIGH_LEVEL;
  const esp_err_t err = gpio_wakeup_enable(pin, wake_level);
  if (err != ESP_OK)
    ESP_LOGW(TAG, "gpio_wakeup_enable(GPIO%d) failed: %d", static_cast<int>(pin), err);
  else
    ESP_LOGI(TAG, "GPIO%d Light-sleep wake on %s (idle=%d)", static_cast<int>(pin),
             idle ? "LOW" : "HIGH", idle);
}

bool setup(bool enabled_value, uint32_t max_awake_value_ms) {
  if (!enabled_value) return true;
  max_awake_ms = max_awake_value_ms;

  esp_pm_config_t config{};
  config.max_freq_mhz = 80;
  config.min_freq_mhz = 40;
  config.light_sleep_enable = true;
  esp_err_t err = esp_pm_configure(&config);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_pm_configure(auto Light-sleep) failed: %d", err);
    return false;
  }

  err = esp_pm_lock_create(ESP_PM_NO_LIGHT_SLEEP, 0, "unni_rtrh", &awake_lock);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_pm_lock_create(NO_LIGHT_SLEEP) failed: %d", err);
    return false;
  }
  err = esp_pm_lock_create(ESP_PM_CPU_FREQ_MAX, 0, "unni_capture_cpu", &cpu_lock);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_pm_lock_create(CPU_FREQ_MAX) failed: %d", err);
    return false;
  }

  configure_wakeup_pin(PIN_G10);
  configure_wakeup_pin(PIN_G13);
  err = esp_sleep_enable_gpio_wakeup();
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_sleep_enable_gpio_wakeup failed: %d", err);
    return false;
  }

  configured = true;
  ESP_LOGI(TAG,
           "Auto Light-sleep enabled: CPU 40..80 MHz, forced 80 MHz while capturing; "
           "wake GPIO3/GPIO4; CO2 GPIO6/GPIO7 excluded; awake timeout %lu ms",
           static_cast<unsigned long>(max_awake_ms));
  return true;
}

void IRAM_ATTR on_rtrh_edge_from_isr() {
  if (!configured || awake_lock == nullptr || cpu_lock == nullptr || lock_held) return;
  if (esp_pm_lock_acquire(awake_lock) != ESP_OK) return;
  if (esp_pm_lock_acquire(cpu_lock) != ESP_OK) {
    esp_pm_lock_release(awake_lock);
    return;
  }
  lock_held = true;
  wake_started_us = static_cast<uint64_t>(esp_timer_get_time());
  rtrh_complete = false;
  co2_after_rtrh = false;
}

void on_rtrh_complete(bool valid) {
  if (!configured || !lock_held) return;
  // Even a rejected complete cycle is enough to avoid being stuck awake; the
  // validity is logged by BusSniffer and the next cycle gets another chance.
  (void) valid;
  rtrh_complete = true;
}

void on_valid_co2() {
  if (!configured || !lock_held || !rtrh_complete) return;
  co2_after_rtrh = true;
}

void loop() {
  if (!configured || !lock_held || awake_lock == nullptr || cpu_lock == nullptr) return;

  const uint64_t now_us = static_cast<uint64_t>(esp_timer_get_time());
  const uint64_t elapsed_us = now_us - wake_started_us;
  const bool complete = rtrh_complete && co2_after_rtrh;
  const bool timeout = elapsed_us >= static_cast<uint64_t>(max_awake_ms) * 1000ULL;
  if (!complete && !timeout) return;

  // Update state before releasing: a new GPIO ISR after this point may acquire
  // the lock for the next cycle without recursively incrementing this handle.
  lock_held = false;
  rtrh_complete = false;
  co2_after_rtrh = false;
  const esp_err_t cpu_err = esp_pm_lock_release(cpu_lock);
  const esp_err_t sleep_err = esp_pm_lock_release(awake_lock);
  if (cpu_err != ESP_OK || sleep_err != ESP_OK) {
    ESP_LOGE(TAG, "esp_pm_lock_release failed: CPU=%d sleep=%d", cpu_err, sleep_err);
    return;
  }

  if (complete) {
    completed_cycles++;
    ESP_LOGI(TAG, "Light-sleep window complete after %llu ms (cycle %lu)",
             static_cast<unsigned long long>(elapsed_us / 1000ULL),
             static_cast<unsigned long>(completed_cycles));
  } else {
    timeout_cycles++;
    ESP_LOGW(TAG, "Light-sleep awake-window timeout after %llu ms (timeout %lu)",
             static_cast<unsigned long long>(elapsed_us / 1000ULL),
             static_cast<unsigned long>(timeout_cycles));
  }
}

bool enabled() { return configured; }
bool awake_window_active() { return lock_held; }

}  // namespace power_save
}  // namespace bus_sniffer
}  // namespace esphome
