#pragma once

#include "esphome.h"

#include "driver/gpio.h"
#include "esp_timer.h"


struct BusPin {
  gpio_num_t pin;
  volatile uint32_t edges = 0;
  volatile uint32_t rising = 0;
  volatile uint32_t falling = 0;
  volatile uint32_t last_time = 0;
  volatile uint32_t min_delta = 0xffffffff;
};


static BusPin pins[] = {
  {GPIO_NUM_38},
  {GPIO_NUM_39},
  {GPIO_NUM_40}
};


static void IRAM_ATTR gpio_isr(void *arg)
{
  BusPin *p = (BusPin *)arg;

  uint32_t now = esp_timer_get_time();

  p->edges++;

  if (gpio_get_level(p->pin))
    p->rising++;
  else
    p->falling++;

  if (p->last_time != 0)
  {
    uint32_t delta = now - p->last_time;
    if (delta < p->min_delta)
      p->min_delta = delta;
  }

  p->last_time = now;
}


void bus_probe_init()
{
  gpio_install_isr_service(0);

  for (auto &p : pins)
  {
    gpio_config_t io = {};

    io.pin_bit_mask = (1ULL << p.pin);
    io.mode = GPIO_MODE_INPUT;
    io.pull_up_en = GPIO_PULLUP_DISABLE;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.intr_type = GPIO_INTR_ANYEDGE;

    gpio_config(&io);

    gpio_isr_handler_add(
        p.pin,
        gpio_isr,
        &p);
  }

  ESP_LOGI("BUS", "GPIO monitor started");
}


void bus_probe_report()
{
  for (auto &p : pins)
  {
    ESP_LOGI(
      "BUS",
      "GPIO%d level=%d edges=%lu rise=%lu fall=%lu min=%lu us",
      (int)p.pin,
      gpio_get_level(p.pin),
      p.edges,
      p.rising,
      p.falling,
      p.min_delta
    );

    p.edges = 0;
    p.rising = 0;
    p.falling = 0;
    p.min_delta = 0xffffffff;
  }
}
