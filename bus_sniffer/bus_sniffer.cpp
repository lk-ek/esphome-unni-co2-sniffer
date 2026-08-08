#include "bus_sniffer.h"

#include "esphome/core/log.h"

#include "driver/gpio.h"
#include "esp_timer.h"

namespace esphome {
namespace bus_sniffer {

static const char *TAG = "BUS";

static constexpr gpio_num_t PIN_A = GPIO_NUM_38;
static constexpr gpio_num_t PIN_B = GPIO_NUM_39;
static constexpr gpio_num_t PIN_C = GPIO_NUM_40;

static constexpr int MAX_SAMPLES = 1024;

static constexpr uint32_t FRAME_TIMEOUT_US = 50000;
static constexpr uint32_t GLITCH_FILTER_US = 5;


struct Sample {
  uint32_t t;
  uint8_t value;
};


volatile Sample samples[MAX_SAMPLES];
volatile uint16_t sample_count = 0;

volatile uint64_t last_edge = 0;
volatile uint8_t last_value = 0xff;

BusSniffer *instance = nullptr;



static void IRAM_ATTR gpio_isr(void *arg) {

  uint32_t now = (uint32_t) esp_timer_get_time();

  uint8_t v =
      (gpio_get_level(PIN_A) ? 1 : 0) |
      (gpio_get_level(PIN_B) ? 2 : 0) |
      (gpio_get_level(PIN_C) ? 4 : 0);


  // gleiche Zustände nicht speichern
  if (v == last_value)
    return;


  // Glitches unter 5us ignorieren
  if ((now - last_edge) < GLITCH_FILTER_US)
    return;


  last_edge = now;


  if (sample_count < MAX_SAMPLES) {

    samples[sample_count].t = now;
    samples[sample_count].value = v;

    sample_count++;

    last_value = v;
  }
}

void BusSniffer::setup() {

  instance = this;

  gpio_config_t io = {};

  io.mode = GPIO_MODE_INPUT;
  io.pull_up_en = GPIO_PULLUP_DISABLE;
  io.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io.intr_type = GPIO_INTR_ANYEDGE;

  io.pin_bit_mask =
      (1ULL << PIN_A) |
      (1ULL << PIN_B) |
      (1ULL << PIN_C);

  gpio_config(&io);


  gpio_install_isr_service(0);

  gpio_isr_handler_add(
      PIN_A,
      gpio_isr,
      nullptr);

  gpio_isr_handler_add(
      PIN_B,
      gpio_isr,
      nullptr);

  gpio_isr_handler_add(
      PIN_C,
      gpio_isr,
      nullptr);


  ESP_LOGI(TAG, "Bus sniffer started");
}


void BusSniffer::loop() {

  if (sample_count == 0)
    return;


  uint64_t now = esp_timer_get_time();


  // noch Datenverkehr?
  if (now - last_edge < FRAME_TIMEOUT_US)
    return;


  // Interrupt kurz sperren
  portDISABLE_INTERRUPTS();

  uint16_t count = sample_count;

  Sample local[MAX_SAMPLES];

  if (count > MAX_SAMPLES)
    count = MAX_SAMPLES;

  for (uint16_t i = 0; i < count; i++) {
    local[i].t = samples[i].t;
    local[i].value = samples[i].value;
  }

  sample_count = 0;

  portENABLE_INTERRUPTS();


  ESP_LOGI(TAG, "==============================");
  ESP_LOGI(TAG, "Captured %u samples", count);
  if (count >= MAX_SAMPLES) {
    ESP_LOGW(TAG, "Buffer overflow, frame truncated");
  }


  if (count == 0)
    return;


  uint32_t min_delta = UINT32_MAX;
  uint32_t max_delta = 0;

  uint32_t changes[3] = {0,0,0};


  for (uint16_t i=1;i<count;i++) {

    uint32_t dt =
        local[i].t -
        local[i-1].t;


    if (dt < min_delta)
      min_delta = dt;

    if (dt > max_delta)
      max_delta = dt;


    uint8_t diff =
        local[i].value ^
        local[i-1].value;


    if (diff & 1)
      changes[0]++;

    if (diff & 2)
      changes[1]++;

    if (diff & 4)
      changes[2]++;
  }


  ESP_LOGI(TAG,
      "min edge spacing: %lu us",
      min_delta);


  if (min_delta > 0)
    ESP_LOGI(TAG,
      "max edge rate: %.1f kHz",
      1000.0f / min_delta);


  ESP_LOGI(TAG,
      "changes GPIO38=%lu GPIO39=%lu GPIO40=%lu",
      changes[0],
      changes[1],
      changes[2]);


  ESP_LOGI(TAG,"DATA:");

  uint32_t base =
      local[0].t;

  uint32_t rel;

  for(uint16_t i = 0; i < count; i++) {

    rel = (uint32_t)(local[i].t - base);
 
    if (i < 200 || count < 250) {
    ESP_LOGI(TAG,
        "%lu us %u%u%u",
        (unsigned long)rel,
        (local[i].value >> 0) & 1,
        (local[i].value >> 1) & 1,
        (local[i].value >> 2) & 1
    );
  }

  }


  ESP_LOGI(TAG,"==============================");
}


} // namespace bus_sniffer
} // namespace esphome
