#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "esphome.h"

namespace esphome {
namespace bus_sniffer {



#define BUS_TAG "BUS"

#define PIN_A 38
#define PIN_B 39
#define PIN_C 40

#define BUFFER_SIZE 4096

// Zeit ohne Flanke -> Frame beendet
#define FRAME_TIMEOUT_US 20000

// maximale Ausgabe pro Frame
#define MAX_PRINT 300


volatile uint32_t edge_time[BUFFER_SIZE];
volatile uint8_t edge_state[BUFFER_SIZE];

volatile uint16_t write_pos = 0;
volatile bool overflow = false;

volatile uint32_t last_edge_time = 0;


// ISR
void IRAM_ATTR bus_edge_isr()
{
  uint16_t pos = write_pos;

  if (pos >= BUFFER_SIZE) {
    overflow = true;
    return;
  }


  uint8_t state = 0;

  if (digitalRead(PIN_A))
    state |= 0x01;

  if (digitalRead(PIN_B))
    state |= 0x02;

  if (digitalRead(PIN_C))
    state |= 0x04;


  edge_time[pos] = micros();
  edge_state[pos] = state;

  write_pos = pos + 1;

  last_edge_time = edge_time[pos];
}



class BusSniffer : public Component {

 public:


  void setup() override
  {
    ESP_LOGI(BUS_TAG, "Starting bus sniffer");


    pinMode(PIN_A, INPUT);
    pinMode(PIN_B, INPUT);
    pinMode(PIN_C, INPUT);


    attachInterrupt(
      PIN_A,
      bus_edge_isr,
      CHANGE
    );

    attachInterrupt(
      PIN_B,
      bus_edge_isr,
      CHANGE
    );

    attachInterrupt(
      PIN_C,
      bus_edge_isr,
      CHANGE
    );


    ESP_LOGI(
      BUS_TAG,
      "Watching GPIO %d %d %d",
      PIN_A,
      PIN_B,
      PIN_C
    );
  }



  void loop() override
  {

    uint16_t count;
    uint32_t idle;


    noInterrupts();

    count = write_pos;

    idle = micros() - last_edge_time;

    interrupts();



    if (count < 5)
      return;



    if (idle < FRAME_TIMEOUT_US)
      return;



    //
    // Capture übernehmen
    //

    uint32_t times[BUFFER_SIZE];
    uint8_t states[BUFFER_SIZE];

    noInterrupts();

    uint16_t n = write_pos;

    if (n > BUFFER_SIZE)
      n = BUFFER_SIZE;


    memcpy(
      times,
      (const void*)edge_time,
      n * sizeof(uint32_t)
    );


    memcpy(
      states,
      (const void*)edge_state,
      n * sizeof(uint8_t)
    );


    write_pos = 0;
    overflow = false;

    interrupts();



    ESP_LOGI(BUS_TAG, "==============================");

    ESP_LOGI(
      BUS_TAG,
      "Captured %u samples",
      n
    );


    if (overflow)
    {
      ESP_LOGW(
        BUS_TAG,
        "BUFFER OVERFLOW"
      );
    }



    //
    // Statistik
    //

    uint32_t min_delta = UINT32_MAX;
    uint32_t max_delta = 0;


    uint8_t changes[3] = {0,0,0};


    for(uint16_t i=1;i<n;i++)
    {
      uint32_t delta =
        times[i]-times[i-1];


      if(delta < min_delta)
        min_delta = delta;


      if(delta > max_delta)
        max_delta = delta;


      uint8_t diff =
        states[i-1] ^ states[i];


      if(diff & 1)
        changes[0]++;

      if(diff & 2)
        changes[1]++;

      if(diff & 4)
        changes[2]++;
    }


    ESP_LOGI(
      BUS_TAG,
      "min edge spacing: %lu us",
      min_delta
    );


    if(min_delta)
    {
      ESP_LOGI(
        BUS_TAG,
        "max edge rate: %.1f kHz",
        1000.0f / min_delta
      );
    }


    ESP_LOGI(
      BUS_TAG,
      "changes GPIO%d=%d GPIO%d=%d GPIO%d=%d",
      PIN_A, changes[0],
      PIN_B, changes[1],
      PIN_C, changes[2]
    );



    //
    // Daten ausgeben
    //

    ESP_LOGI(BUS_TAG,"DATA:");

    uint32_t start = times[0];


    uint16_t print_count = n;

    if(print_count > MAX_PRINT)
      print_count = MAX_PRINT;



    for(uint16_t i=0;i<print_count;i++)
    {

      ESP_LOGI(
        BUS_TAG,
        "%8lu us  %03d",
        times[i]-start,
        states[i]
      );

    }


    if(n > MAX_PRINT)
    {
      ESP_LOGI(
        BUS_TAG,
        "... %u samples omitted",
        n-MAX_PRINT
      );
    }


    ESP_LOGI(
      BUS_TAG,
      "=============================="
    );

  }

};

} // namespace bus_sniffer
} // namespace esphome
