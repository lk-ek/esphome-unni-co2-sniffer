#pragma once

#include "esphome.h"
#include "driver/gpio.h"
#include "esp_timer.h"

#define SAMPLE_BUFFER_SIZE 512


struct Sample {
  uint32_t time;
  uint8_t value;
};


volatile Sample sample_buffer[SAMPLE_BUFFER_SIZE];
volatile uint16_t sample_index = 0;
volatile bool capture_active = false;


static const gpio_num_t monitored_pins[] = {
  GPIO_NUM_38,
  GPIO_NUM_39,
  GPIO_NUM_40
};


static void IRAM_ATTR gpio_capture_isr(void *arg)
{
  uint32_t now = esp_timer_get_time();

  if (!capture_active)
    return;

  if (sample_index >= SAMPLE_BUFFER_SIZE)
  {
    capture_active = false;
    return;
  }

  uint8_t value = 0;

  if (gpio_get_level(GPIO_NUM_38))
    value |= 1;

  if (gpio_get_level(GPIO_NUM_39))
    value |= 2;

  if (gpio_get_level(GPIO_NUM_40))
    value |= 4;


  sample_buffer[sample_index].time = now;
  sample_buffer[sample_index].value = value;

  sample_index++;
}



void bus_probe_init()
{
  gpio_install_isr_service(0);


  for (auto pin : monitored_pins)
  {
    gpio_config_t io = {};

    io.pin_bit_mask = (1ULL << pin);
    io.mode = GPIO_MODE_INPUT;

    // keine Pullups!
    io.pull_up_en = GPIO_PULLUP_DISABLE;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;

    io.intr_type = GPIO_INTR_ANYEDGE;

    gpio_config(&io);

    gpio_isr_handler_add(
      pin,
      gpio_capture_isr,
      nullptr);
  }


  ESP_LOGI("BUS", "capture ready");
}



void bus_probe_report()
{
  ESP_LOGI(
    "BUS",
    "GPIO levels: %d %d %d",
    gpio_get_level(GPIO_NUM_38),
    gpio_get_level(GPIO_NUM_39),
    gpio_get_level(GPIO_NUM_40)
  );


  // wenn gerade nichts aufgenommen wird:
  if (!capture_active)
  {
    sample_index = 0;
    capture_active = true;
    ESP_LOGI("BUS", "capture started");
    return;
  }


  // Ausgabe, wenn Buffer voll:
  ESP_LOGI("BUS", "capture dump %d samples", sample_index);


  for (int i = 0; i < sample_index; i++)
  {
    ESP_LOGI(
      "BUS",
      "%lu us  %c%c%c",
      sample_buffer[i].time,
      (sample_buffer[i].value & 1) ? '1':'0',
      (sample_buffer[i].value & 2) ? '1':'0',
      (sample_buffer[i].value & 4) ? '1':'0'
    );
  }


  capture_active = false;
}
