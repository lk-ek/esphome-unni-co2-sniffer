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
static esp_pm_lock_handle_t co2_active_lock = nullptr;
static esp_pm_lock_handle_t co2_cpu_lock = nullptr;
static volatile bool external_power = false;
struct PmLockPair {
  esp_pm_lock_handle_t sleep{nullptr};
  esp_pm_lock_handle_t cpu{nullptr};
  bool sleep_held{false};
  bool cpu_held{false};
  uint32_t errors{0};

  bool acquire(const char *name) {
    bool acquired_sleep_now = false;
    if (!sleep_held) {
      const esp_err_t err = esp_pm_lock_acquire(sleep);
      if (err != ESP_OK) {
        ++errors;
        ESP_LOGE(TAG, "%s NO_LIGHT_SLEEP acquire failed: %d (errors=%lu)", name, err,
                 static_cast<unsigned long>(errors));
        return false;
      }
      sleep_held = true;
      acquired_sleep_now = true;
    }
    if (!cpu_held) {
      const esp_err_t err = esp_pm_lock_acquire(cpu);
      if (err != ESP_OK) {
        ++errors;
        ESP_LOGE(TAG, "%s CPU_FREQ_MAX acquire failed: %d (errors=%lu)", name, err,
                 static_cast<unsigned long>(errors));
        if (acquired_sleep_now) {
          const esp_err_t rollback = esp_pm_lock_release(sleep);
          if (rollback == ESP_OK) {
            sleep_held = false;
          } else {
            ++errors;
            ESP_LOGE(TAG, "%s NO_LIGHT_SLEEP acquire rollback failed: %d (errors=%lu)",
                     name, rollback, static_cast<unsigned long>(errors));
          }
        }
        return false;
      }
      cpu_held = true;
    }
    return sleep_held && cpu_held;
  }

  bool release(const char *name) {
    if (cpu_held) {
      const esp_err_t err = esp_pm_lock_release(cpu);
      if (err == ESP_OK) {
        cpu_held = false;
      } else {
        ++errors;
        ESP_LOGE(TAG, "%s CPU_FREQ_MAX release failed: %d (errors=%lu)", name, err,
                 static_cast<unsigned long>(errors));
      }
    }
    if (sleep_held) {
      const esp_err_t err = esp_pm_lock_release(sleep);
      if (err == ESP_OK) {
        sleep_held = false;
      } else {
        ++errors;
        ESP_LOGE(TAG, "%s NO_LIGHT_SLEEP release failed: %d (errors=%lu)", name, err,
                 static_cast<unsigned long>(errors));
      }
    }
    return !sleep_held && !cpu_held;
  }

  bool held() const { return sleep_held || cpu_held; }
  bool fully_held() const { return sleep_held && cpu_held; }
};
static PmLockPair external_locks;
static PmLockPair co2_locks;
static bool co2_state_initialized = false;
static volatile bool co2_powered_down = false;
static bool co2_scl_wakeup_armed = false;
static volatile bool lock_held = false;
static volatile bool awake_lock_held = false;
static volatile bool cpu_lock_held = false;
static uint32_t rtrh_lock_errors = 0;
static volatile uint64_t wake_started_us = 0;
static volatile bool rtrh_complete = false;
static volatile bool co2_after_rtrh = false;
static volatile bool transport_busy = false;
static volatile uint32_t wake_gen = 0;
static uint32_t max_awake_ms = 10000;
static uint32_t completed_cycles = 0;
static uint32_t timeout_cycles = 0;
static uint64_t last_pair_retry_us = 0;
static constexpr uint32_t TRANSPORT_DRAIN_GRACE_MS = 1000;

static void delete_lock(esp_pm_lock_handle_t &lock) {
  if (lock != nullptr) {
    esp_pm_lock_delete(lock);
    lock = nullptr;
  }
}

static void cleanup_locks() {
  delete_lock(co2_cpu_lock);
  delete_lock(co2_active_lock);
  delete_lock(external_cpu_lock);
  delete_lock(external_awake_lock);
  delete_lock(cpu_lock);
  delete_lock(awake_lock);
  external_locks = {};
  co2_locks = {};
}

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
  if (!enabled_value) {
    ESP_LOGW(TAG, "A/B diagnostic: automatic Light-sleep disabled; battery WiFi/publish policy remains active");
    return true;
  }
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
    cleanup_locks();
    return false;
  }
  err = esp_pm_lock_create(ESP_PM_NO_LIGHT_SLEEP, 0, "unni_usb_awake", &external_awake_lock);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_pm_lock_create(USB NO_LIGHT_SLEEP) failed: %d", err);
    cleanup_locks();
    return false;
  }
  err = esp_pm_lock_create(ESP_PM_CPU_FREQ_MAX, 0, "unni_usb_cpu", &external_cpu_lock);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_pm_lock_create(USB CPU_FREQ_MAX) failed: %d", err);
    cleanup_locks();
    return false;
  }
  err = esp_pm_lock_create(ESP_PM_NO_LIGHT_SLEEP, 0, "unni_co2_window", &co2_active_lock);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_pm_lock_create(CO2 NO_LIGHT_SLEEP) failed: %d", err);
    cleanup_locks();
    return false;
  }
  err = esp_pm_lock_create(ESP_PM_CPU_FREQ_MAX, 0, "unni_co2_cpu", &co2_cpu_lock);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_pm_lock_create(CO2 CPU_FREQ_MAX) failed: %d", err);
    cleanup_locks();
    return false;
  }
  external_locks.sleep = external_awake_lock;
  external_locks.cpu = external_cpu_lock;
  co2_locks.sleep = co2_active_lock;
  co2_locks.cpu = co2_cpu_lock;

  configure_wakeup_pin(pin_rt);
  configure_wakeup_pin(pin_rh);
  err = esp_sleep_enable_gpio_wakeup();
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_sleep_enable_gpio_wakeup failed: %d", err);
    gpio_wakeup_disable(pin_rt);
    gpio_wakeup_disable(pin_rh);
    cleanup_locks();
    return false;
  }

  configured = true;
  ESP_LOGI(TAG,
           "Auto Light-sleep enabled: CPU 40..80 MHz, forced 80 MHz while capturing; "
           "wake GPIO%d/GPIO%d; CO2 SCL GPIO%d armed HIGH only during powered-down windows "
           "(SDA GPIO%d remains passive); awake timeout %lu ms; "
           "debug transport drain grace %lu ms",
           static_cast<int>(pin_rt), static_cast<int>(pin_rh),
           static_cast<int>(pin_co2_scl), static_cast<int>(pin_co2_sda),
           static_cast<unsigned long>(max_awake_ms),
           static_cast<unsigned long>(TRANSPORT_DRAIN_GRACE_MS));
  return true;
}

void set_co2_bus_powered_down(bool powered_down) {
  if (!configured || co2_active_lock == nullptr || co2_cpu_lock == nullptr) return;

  // USB already holds the ESP awake continuously. Keep the CO2-specific wake
  // source and lock out of the way until battery policy is active again.
  if (external_power) {
    if (co2_scl_wakeup_armed) {
      gpio_wakeup_disable(pin_co2_scl);
      co2_scl_wakeup_armed = false;
    }
    co2_locks.release("CO2 window");
    co2_powered_down = false;
    co2_state_initialized = false;
    return;
  }

  const bool locks_settled = powered_down ? !co2_locks.held() : co2_locks.fully_held();
  if (co2_state_initialized && powered_down == co2_powered_down && locks_settled) return;
  co2_state_initialized = true;
  co2_powered_down = powered_down;

  if (powered_down) {
    co2_locks.release("CO2 window");

    // The dead Unni bus sits LOW/LOW. Its next power-up raises SCL, so HIGH is
    // a stable level wake condition and gives us ample lead time before 21B1.
    const esp_err_t wake_err = gpio_wakeup_enable(pin_co2_scl, GPIO_INTR_HIGH_LEVEL);
    if (wake_err == ESP_OK) {
      co2_scl_wakeup_armed = true;
      ESP_LOGI(TAG, "CO2 subsystem powered down: SCL GPIO%d HIGH wake armed",
               static_cast<int>(pin_co2_scl));
    } else {
      ESP_LOGE(TAG, "CO2 SCL wake arm failed on GPIO%d: %d",
               static_cast<int>(pin_co2_scl), wake_err);
    }

    // If an RT/RH wake window is currently waiting for CO2, don't burn the
    // full timeout: CO2 is known unavailable until SCL rises again.
    if (lock_held && rtrh_complete) co2_after_rtrh = true;
  } else {
    if (co2_scl_wakeup_armed) {
      const esp_err_t wake_err = gpio_wakeup_disable(pin_co2_scl);
      if (wake_err != ESP_OK) {
        ESP_LOGW(TAG, "CO2 SCL wake disable failed on GPIO%d: %d",
                 static_cast<int>(pin_co2_scl), wake_err);
      }
      co2_scl_wakeup_armed = false;
    }
    co2_locks.acquire("CO2 window");
    ESP_LOGI(TAG, "CO2 subsystem active: SCL wake disarmed; keeping ESP awake at 80 MHz for native window");
  }
}

bool co2_bus_powered_down() { return co2_powered_down; }

void set_external_power(bool present) {
  external_power = present;

  if (present) {
    if (co2_scl_wakeup_armed) {
      gpio_wakeup_disable(pin_co2_scl);
      co2_scl_wakeup_armed = false;
    }
    if (co2_active_lock != nullptr && co2_cpu_lock != nullptr)
      co2_locks.release("CO2 window");
    co2_powered_down = false;
    co2_state_initialized = false;
  }

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
  if (present) {
    if (external_locks.acquire("USB"))
      ESP_LOGI(TAG, "USB power mode: Light-sleep disabled, CPU locked at 80 MHz");
  } else {
    if (external_locks.release("USB"))
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
  awake_lock_held = true;
  if (esp_pm_lock_acquire(cpu_lock) != ESP_OK) {
    if (esp_pm_lock_release(awake_lock) == ESP_OK) {
      awake_lock_held = false;
    } else {
      // Preserve the partial ownership state so task context can retry the
      // release. Pretending to be unlocked here would recursively acquire the
      // same NO_LIGHT_SLEEP handle on the next edge.
      lock_held = true;
      wake_started_us = static_cast<uint64_t>(esp_timer_get_time());
      rtrh_complete = true;
      co2_after_rtrh = true;
    }
    return;
  }
  cpu_lock_held = true;
  lock_held = true;
  wake_started_us = static_cast<uint64_t>(esp_timer_get_time());
  rtrh_complete = false;
  co2_after_rtrh = false;
  wake_gen++;
}

uint32_t wake_generation() { return wake_gen; }

void on_rtrh_complete() {
  if (!configured || !lock_held) return;
  rtrh_complete = true;

  // The short RT/RH wake window does not need to wait for a CO2 result once
  // the independent CO2 window owns both of its PM locks. The CO2 lock pair
  // keeps Light Sleep disabled and the CPU at 80 MHz until the native CO2
  // window closes, so retaining the RT/RH lock here only creates a misleading
  // max-awake timeout and duplicate lock ownership.
  if (co2_powered_down || co2_locks.fully_held()) co2_after_rtrh = true;
}

void on_valid_co2() {
  if (!configured || !lock_held || !rtrh_complete) return;
  co2_after_rtrh = true;
}

void set_transport_busy(bool busy) { transport_busy = busy; }

void loop() {
  if (!configured) return;

  // A policy transition may have observed a transient PM-lock error. Reconcile
  // partial per-handle ownership at a bounded rate even when the orchestrator's
  // high-level power state itself has not changed again.
  const bool pair_retry_needed =
      external_power ? (!external_locks.fully_held() || co2_locks.held())
                     : (external_locks.held() ||
                        (co2_state_initialized &&
                         (co2_powered_down ? co2_locks.held() : !co2_locks.fully_held())));
  uint64_t now_us = 0;
  if (pair_retry_needed) now_us = static_cast<uint64_t>(esp_timer_get_time());
  if (pair_retry_needed &&
      (last_pair_retry_us == 0 || now_us - last_pair_retry_us >= 1000000ULL)) {
    last_pair_retry_us = now_us;
    if (external_power) {
      if (!external_locks.fully_held()) external_locks.acquire("USB retry");
      if (co2_locks.held()) co2_locks.release("CO2 window retry");
    } else {
      if (external_locks.held()) external_locks.release("USB retry");
      if (co2_state_initialized) {
        if (co2_powered_down) {
          if (co2_locks.held()) co2_locks.release("CO2 window retry");
        } else if (!co2_locks.fully_held()) {
          co2_locks.acquire("CO2 window retry");
        }
      }
    }
  }

  if (!lock_held || awake_lock == nullptr || cpu_lock == nullptr) return;

  if (now_us == 0) now_us = static_cast<uint64_t>(esp_timer_get_time());
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

  esp_err_t cpu_err = ESP_OK;
  esp_err_t sleep_err = ESP_OK;
  if (cpu_lock_held) {
    cpu_err = esp_pm_lock_release(cpu_lock);
    if (cpu_err == ESP_OK) cpu_lock_held = false;
  }
  if (awake_lock_held) {
    sleep_err = esp_pm_lock_release(awake_lock);
    if (sleep_err == ESP_OK) awake_lock_held = false;
  }

  // Only expose an unlocked state after both handles have been released. The
  // previous ordering cleared lock_held first, allowing an ISR to re-acquire a
  // handle while loop() was concurrently releasing it.
  if (!cpu_lock_held && !awake_lock_held) {
    lock_held = false;
    rtrh_complete = false;
    co2_after_rtrh = false;
  }

  gpio_intr_enable(pin_rt);
  gpio_intr_enable(pin_rh);

  if (cpu_err != ESP_OK || sleep_err != ESP_OK) {
    ++rtrh_lock_errors;
    ESP_LOGE(TAG, "esp_pm_lock_release failed: CPU=%d sleep=%d (errors=%lu; retained CPU=%u sleep=%u)",
             cpu_err, sleep_err, static_cast<unsigned long>(rtrh_lock_errors),
             static_cast<unsigned>(cpu_lock_held), static_cast<unsigned>(awake_lock_held));
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
