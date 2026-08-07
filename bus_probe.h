#pragma once

#include "esphome.h"
#include "driver/gpio.h"
#include "esp_timer.h"

#define SAMPLE_BUFFER_SIZE 512


struct Sample {
  uint32_t dt;
  uint8_t value;
};


static volatile Sample samples[SAMPLE_BUFFER_SIZE];

static volatile uint16_t sample_count = 0;
static volatile bool capturing = false;
static volatile bool capture_ready = false;

static uint32_t capture_start_time = 0;


static uint8_t read_bus()
{
  uint8_t v = 0;

  if (gpio_get_level(GPIO_NUM_38))
    v |= 1;

  if (gpio_get_level(GPIO_NUM_39))
    v |= 2;

  if (gpio_get_level(GPIO_NUM_40))
    v |= 4;

  return v;
}


static void IRAM_ATTR bus_isr(void *arg)
{
  if (capture_ready)
    return;

  uint32_t now = esp_timer_get_time();

  if (!capturing)
  {
    capturing = true;
    sample_count = 0;
    capture_start_time = now;
  }


  if (sample_count < SAMPLE_BUFFER_SIZE)
  {
    samples[sample_count].dt = now - capture_start_time;
    samples[sample_count].value = read_bus();
    sample_count++;
  }
  else
  {
    capturing = false;
    capture_ready = true;
  }
}



void bus_probe_init()
{
  gpio_install_isr_service(0);


  gpio_num_t pins[] = {
    GPIO_NUM_38,
    GPIO_NUM_39,
    GPIO_NUM_40
  };


  for (auto pin : pins)
  {
    gpio_config_t io = {};

    io.pin_bit_mask = (1ULL << pin);
    io.mode = GPIO_MODE_INPUT;

    io.pull_up_en = GPIO_PULLUP_DISABLE;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;

    io.intr_type = GPIO_INTR_ANYEDGE;

    gpio_config(&io);

    gpio_isr_handler_add(
      pin,
      bus_isr,
      nullptr);
  }


  ESP_LOGI("BUS", "edge triggered capture active");
}



void bus_probe_report()
{
  if (!capture_ready)
    return;


  ESP_LOGI(
    "BUS",
    "Captured %d samples",
    sample_count
  );


  for (int i = 0; i < sample_count; i++)
  {
    uint8_t v = samples[i].value;

    ESP_LOGI(
      "BUS",
      "%6lu us  GPIO38=%d GPIO39=%d GPIO40=%d",
      samples[i].dt,
      !!(v & 1),
      !!(v & 2),
      !!(v & 4)
    );
  }


  sample_count = 0;
  capture_ready = false;
}
