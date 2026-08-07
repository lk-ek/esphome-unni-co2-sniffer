#pragma once

#include "esphome.h"
#include "driver/gpio.h"
#include "esp_timer.h"


#define SAMPLE_BUFFER_SIZE 2048

// Burst-Ende nach 5ms ohne Flanke
#define IDLE_TIMEOUT_US 5000


struct Sample {
  uint32_t dt;
  uint8_t value;
};


static volatile Sample samples[SAMPLE_BUFFER_SIZE];

static volatile uint16_t sample_count = 0;
static volatile bool capturing = false;
static volatile bool capture_ready = false;

static volatile uint32_t last_edge_time = 0;
static volatile uint32_t start_time = 0;


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
  uint32_t now = esp_timer_get_time();


  if (capture_ready)
    return;


  if (!capturing)
  {
    capturing = true;
    sample_count = 0;
    start_time = now;
  }


  last_edge_time = now;


  if (sample_count < SAMPLE_BUFFER_SIZE)
  {
    samples[sample_count].dt = now - start_time;
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
      nullptr
    );
  }


  ESP_LOGI("BUS", "edge capture enabled");
}




void bus_probe_report()
{

  uint32_t now = esp_timer_get_time();


  // Aufnahme läuft noch
  if (capturing)
  {
    if ((now - last_edge_time) > IDLE_TIMEOUT_US)
    {
      capturing = false;
      capture_ready = true;
    }
  }



  if (!capture_ready)
    return;



  ESP_LOGI(
    "BUS",
    "=============================="
  );


  ESP_LOGI(
    "BUS",
    "Captured %u samples",
    sample_count
  );


  if (sample_count > 1)
  {

    uint32_t min_dt = 0xffffffff;
    uint32_t max_dt = 0;

    uint32_t changes[3] = {0,0,0};


    for (int i=1;i<sample_count;i++)
    {

      uint32_t d =
        samples[i].dt -
        samples[i-1].dt;


      if (d < min_dt)
        min_dt=d;

      if (d > max_dt)
        max_dt=d;



      uint8_t diff =
        samples[i].value ^
        samples[i-1].value;


      if (diff & 1)
        changes[0]++;

      if (diff & 2)
        changes[1]++;

      if (diff & 4)
        changes[2]++;
    }


    ESP_LOGI(
      "BUS",
      "min edge spacing: %lu us",
      min_dt
    );


    if (min_dt)
    {
      ESP_LOGI(
        "BUS",
        "max edge rate: %.1f kHz",
        1000.0f / min_dt
      );
    }


    ESP_LOGI(
      "BUS",
      "changes GPIO38=%lu GPIO39=%lu GPIO40=%lu",
      changes[0],
      changes[1],
      changes[2]
    );
  }



  ESP_LOGI("BUS","DATA:");

  for (int i=0;i<sample_count;i++)
  {

    ESP_LOGI(
      "BUS",
      "%8lu us  %d%d%d",
      samples[i].dt,
      !!(samples[i].value & 4),
      !!(samples[i].value & 2),
      !!(samples[i].value & 1)
    );

  }


  ESP_LOGI(
    "BUS",
    "=============================="
  );


  sample_count=0;
  capture_ready=false;
}
