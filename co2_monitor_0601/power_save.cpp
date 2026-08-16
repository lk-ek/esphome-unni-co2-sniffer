// SPDX-FileCopyrightText: 2026 The esphome-unni-co2-sniffer contributors
// SPDX-License-Identifier: GPL-3.0-or-later
#include "power_save.h"

#include "driver/gpio.h"
#include "esp_pm.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esphome/core/log.h"

namespace esphome {
namespace co2_monitor_0601 {
namespace power_save {

static const char *TAG = "unni_power";
static gpio_num_t pin_rt = GPIO_NUM_3;
static gpio_num_t pin_rh = GPIO_NUM_4;
static gpio_num_t pin_co2_sda = GPIO_NUM_6;
static gpio_num_t pin_co2_scl = GPIO_NUM_7;

static bool configured = false;
static esp_pm_lock_handle_t awake_lock = nullptr;
static esp_pm_lock_handle_t cpu_lock = nullptr;
static esp_pm_lock_handle_t external_awake_lock = nullptr;
static esp_pm_lock_handle_t external_cpu_lock = nullptr;
static volatile bool external_power = false;
static bool external_locks_held = false;
static volatile bool lock_held = false;
static volatile uint64_t wake_started_us = 0;
static volatile bool rtrh_complete = false;
static volatile bool co2_after_rtrh = false;
static volatile bool transport_busy = false;
static uint32_t max_awake_ms = 10000;
static uint32_t completed_cycles = 0;
static uint32_t timeout_cycles = 0;
static constexpr uint32_t TRANSPORT_DRAIN_GRACE_MS = 1000;

static void configure_wakeup_pin(gpio_num_t pin) {
  // Wake on the opposite of the current idle level. RT/RH lines spend almost
  // all of their time idle, so the first transition of the next cycle wakes us.
  const int idle = gpio_get_level(pin);
  const gpio_int_type_t wake_level = idle ? GPIO_INTR_LOW_LEVEL : GPIO_INTR_HIGH_LEVEL;
  const esp_err_t err = gpio_wakeup_enable(pin, wake_level);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "gpio_wakeup_enable(GPIO%d) failed: %d", static_cast<int>(pin), err);
  } else {
    ESP_LOGI(TAG, "GPIO%d Light-sleep wake on %s (idle=%d)", static_cast<int>(pin),
             idle ? "LOW" : "HIGH", idle);
  }
}

bool setup(bool enabled_value, uint32_t max_awake_value_ms, uint8_t rt_pin, uint8_t rh_pin,
           uint8_t co2_sda_pin, uint8_t co2_scl_pin) {
  pin_rt = static_cast<gpio_num_t>(rt_pin);
  pin_rh = static_cast<gpio_num_t>(rh_pin);
  pin_co2_sda = static_cast<gpio_num_t>(co2_sda_pin);
  pin_co2_scl = static_cast<gpio_num_t>(co2_scl_pin);
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
  err = esp_pm_lock_create(ESP_PM_NO_LIGHT_SLEEP, 0, "unni_usb_awake", &external_awake_lock);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_pm_lock_create(USB NO_LIGHT_SLEEP) failed: %d", err);
    return false;
  }
  err = esp_pm_lock_create(ESP_PM_CPU_FREQ_MAX, 0, "unni_usb_cpu", &external_cpu_lock);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_pm_lock_create(USB CPU_FREQ_MAX) failed: %d", err);
    return false;
  }

  configure_wakeup_pin(pin_rt);
  configure_wakeup_pin(pin_rh);
  err = esp_sleep_enable_gpio_wakeup();
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_sleep_enable_gpio_wakeup failed: %d", err);
    return false;
  }

  configured = true;
  ESP_LOGI(TAG,
           "Auto Light-sleep enabled: CPU 40..80 MHz, forced 80 MHz while capturing; "
           "wake GPIO%d/GPIO%d; CO2 GPIO%d/GPIO%d excluded; awake timeout %lu ms; "
           "debug transport drain grace %lu ms",
           static_cast<int>(pin_rt), static_cast<int>(pin_rh),
           static_cast<int>(pin_co2_sda), static_cast<int>(pin_co2_scl),
           static_cast<unsigned long>(max_awake_ms),
           static_cast<unsigned long>(TRANSPORT_DRAIN_GRACE_MS));
  return true;
}

void set_external_power(bool present) {
  external_power = present;

  // USB power is unconstrained: favor Wi-Fi robustness/latency. Battery and
  // Energy Save use MIN_MODEM so the AP can buffer traffic between DTIM wakeups.
  // In Wi-Fi/BLE coexistence ESP-IDF may still sleep outside the Wi-Fi time slice
  // even with WIFI_PS_NONE; this call nevertheless selects the least aggressive
  // Wi-Fi power-saving policy available to the station.
  const wifi_ps_type_t wifi_ps = present ? WIFI_PS_NONE : WIFI_PS_MIN_MODEM;
  const esp_err_t wifi_err = esp_wifi_set_ps(wifi_ps);
  if (wifi_err == ESP_OK) {
    ESP_LOGI(TAG, "WiFi power save: %s (%s policy)",
             present ? "NONE" : "MIN_MODEM", present ? "USB" : "battery");
  } else if (wifi_err != ESP_ERR_WIFI_NOT_INIT) {
    ESP_LOGW(TAG, "esp_wifi_set_ps(%s) failed: %s",
             present ? "NONE" : "MIN_MODEM", esp_err_to_name(wifi_err));
  }

  if (!configured || external_awake_lock == nullptr || external_cpu_lock == nullptr) return;
  if (present == external_locks_held) return;

  if (present) {
    const esp_err_t sleep_err = esp_pm_lock_acquire(external_awake_lock);
    if (sleep_err != ESP_OK) {
      ESP_LOGE(TAG, "USB NO_LIGHT_SLEEP lock acquire failed: %d", sleep_err);
      return;
    }
    const esp_err_t cpu_err = esp_pm_lock_acquire(external_cpu_lock);
    if (cpu_err != ESP_OK) {
      esp_pm_lock_release(external_awake_lock);
      ESP_LOGE(TAG, "USB CPU_FREQ_MAX lock acquire failed: %d", cpu_err);
      return;
    }
    external_locks_held = true;
    ESP_LOGI(TAG, "USB power mode: Light-sleep disabled, CPU locked at 80 MHz");
  } else {
    const esp_err_t cpu_err = esp_pm_lock_release(external_cpu_lock);
    const esp_err_t sleep_err = esp_pm_lock_release(external_awake_lock);
    if (cpu_err != ESP_OK || sleep_err != ESP_OK) {
      ESP_LOGE(TAG, "USB power lock release failed: CPU=%d sleep=%d", cpu_err, sleep_err);
      return;
    }
    external_locks_held = false;
    ESP_LOGI(TAG, "Battery mode: automatic Light-sleep restored");
  }
}

bool external_power_present() { return external_power; }

void IRAM_ATTR on_rtrh_edge_from_isr() {
  if (!configured || external_power || awake_lock == nullptr || cpu_lock == nullptr || lock_held) return;
  // ESP-IDF allows PM-lock acquire/release from ISR context, but operations on
  // the same lock handle are not thread-safe against concurrent calls. loop()
  // therefore masks both RT/RH GPIO interrupts while it releases these handles.
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
  // validity is logged by CO2Monitor0601 and the next cycle gets another chance.
  (void) valid;
  rtrh_complete = true;
}

void on_valid_co2() {
  if (!configured || !lock_held || !rtrh_complete) return;
  co2_after_rtrh = true;
}

void set_transport_busy(bool busy) { transport_busy = busy; }

void loop() {
  if (!configured || !lock_held || awake_lock == nullptr || cpu_lock == nullptr) return;

  const uint64_t now_us = static_cast<uint64_t>(esp_timer_get_time());
  const uint64_t elapsed_us = now_us - wake_started_us;
  const bool complete = rtrh_complete && co2_after_rtrh;
  const bool timeout = elapsed_us >= static_cast<uint64_t>(max_awake_ms) * 1000ULL;
  if (!complete && !timeout) return;

  // Let queued debug UDP packets drain while the NO_LIGHT_SLEEP lock is still
  // held. Never let a missing collector keep the ESP awake indefinitely.
  const uint64_t drain_deadline_us =
      (static_cast<uint64_t>(max_awake_ms) + TRANSPORT_DRAIN_GRACE_MS) * 1000ULL;
  if (transport_busy && elapsed_us < drain_deadline_us) {
    return;
  }

  // esp_pm_lock_* is not thread-safe when the same handle is manipulated from
  // ISR and task context concurrently. Mask both RT/RH IRQs across the release
  // so on_rtrh_edge_from_isr() cannot recursively acquire either handle in the
  // small state transition window.
  gpio_intr_disable(pin_rt);
  gpio_intr_disable(pin_rh);

  const esp_err_t cpu_err = esp_pm_lock_release(cpu_lock);
  const esp_err_t sleep_err = esp_pm_lock_release(awake_lock);

  // Only expose an unlocked state after both handles have been released. The
  // previous ordering cleared lock_held first, allowing an ISR to re-acquire a
  // handle while loop() was concurrently releasing it.
  lock_held = false;
  rtrh_complete = false;
  co2_after_rtrh = false;

  gpio_intr_enable(pin_rt);
  gpio_intr_enable(pin_rh);

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
    ESP_LOGW(TAG, "Light-sleep awake-window timeout after %llu ms (timeout %lu)%s",
             static_cast<unsigned long long>(elapsed_us / 1000ULL),
             static_cast<unsigned long>(timeout_cycles),
             transport_busy ? " after debug-transport drain grace" : "");
  }
}

bool enabled() { return configured; }
bool awake_window_active() { return lock_held; }

}  // namespace power_save
}  // namespace co2_monitor_0601
}  // namespace esphome
