#include "bus_sniffer.h"

#include "esphome/core/log.h"

#include "driver/gpio.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

#include <stdint.h>

namespace esphome {
namespace bus_sniffer {

static const char *TAG = "BUS";

/*
 * Vermutete Belegung:
 *
 * GPIO38 = SCL
 * GPIO40 = SDA
 * GPIO39 = unbekanntes drittes Signal
 */
static constexpr gpio_num_t PIN_SCL = GPIO_NUM_38;
static constexpr gpio_num_t PIN_AUX = GPIO_NUM_39;
static constexpr gpio_num_t PIN_SDA = GPIO_NUM_40;


/*
 * GPIO39 erzeugt offenbar sehr viele Flanken.
 *
 * 2048 Samples reichen für die bisher beobachteten Transaktionen
 * normalerweise aus. Der Speicherbedarf beträgt nur ca. 8 kB.
 */
static constexpr uint16_t MAX_SAMPLES = 2048;


/*
 * Eine Transaktion gilt als beendet, wenn 20 ms lang keine
 * Flanke mehr aufgetreten ist.
 *
 * Das ist absichtlich deutlich kürzer als vorher 50 ms, damit
 * kontinuierliche Signale nicht alles zu einem riesigen Frame
 * zusammenfassen.
 */
static constexpr uint32_t FRAME_TIMEOUT_US = 20000;


/*
 * Sehr kurze Glitches ignorieren.
 *
 * 2-3 us wurden bereits beobachtet. Deshalb zunächst nur
 * 2 us als Grenze verwenden.
 */
static constexpr uint32_t GLITCH_FILTER_US = 2;


/*
 * Sample:
 *
 * bit 0 = GPIO38 / SCL
 * bit 1 = GPIO39 / AUX
 * bit 2 = GPIO40 / SDA
 */
struct Sample {
  uint32_t t;
  uint8_t value;
};


/*
 * ISR-Ring/Buffer.
 *
 * Die ISR schreibt ausschließlich hier hinein.
 */
static volatile Sample samples[MAX_SAMPLES];
static volatile uint16_t sample_count = 0;

static volatile uint32_t last_edge = 0;
static volatile uint8_t last_value = 0xff;

static portMUX_TYPE sample_mux = portMUX_INITIALIZER_UNLOCKED;


/*
 * Hilfsfunktionen
 */

static inline uint8_t get_scl(uint8_t v) {
  return (v & 0x01) ? 1 : 0;
}

static inline uint8_t get_aux(uint8_t v) {
  return (v & 0x02) ? 1 : 0;
}

static inline uint8_t get_sda(uint8_t v) {
  return (v & 0x04) ? 1 : 0;
}


/*
 * I2C-Dekoder
 *
 * Wir analysieren ausschließlich:
 *
 *   GPIO38 = SCL
 *   GPIO40 = SDA
 *
 * GPIO39 wird separat ausgewertet.
 */
struct I2CDecoder {
  bool active = false;

  uint8_t byte_value = 0;
  uint8_t bit_count = 0;

  uint32_t start_time = 0;

  uint32_t bytes = 0;
  uint32_t starts = 0;
  uint32_t stops = 0;
  uint32_t acks = 0;
  uint32_t nacks = 0;

  bool have_last = false;
  uint8_t last_scl = 1;
  uint8_t last_sda = 1;
};


/*
 * Ausgabe eines fertig dekodierten Bytes.
 */
static void log_i2c_byte(uint8_t value, bool ack, uint32_t t) {

  uint8_t address = value >> 1;
  bool read = value & 1;

  /*
   * Das erste Byte nach START ist normalerweise die Adresse.
   */
  ESP_LOGI(
      TAG,
      "I2C %lu us: BYTE 0x%02X%s",
      (unsigned long)t,
      value,
      ack ? " ACK" : " NACK"
  );

  /*
   * 0x62 ist die erwartete Adresse der SCD4x-Familie.
   */
  if (address == 0x62) {
    ESP_LOGI(
        TAG,
        "       -> SCD4x address 0x62, %s",
        read ? "READ" : "WRITE"
    );
  }
}


/*
 * I2C-Decoder über die aufgezeichneten Samples laufen lassen.
 */
static void decode_i2c(
    const Sample *data,
    uint16_t count
) {

  I2CDecoder d;

  if (count < 2) {
    return;
  }

  d.last_scl = get_scl(data[0].value);
  d.last_sda = get_sda(data[0].value);
  d.have_last = true;

  for (uint16_t i = 1; i < count; i++) {

    const uint8_t prev = data[i - 1].value;
    const uint8_t cur = data[i].value;

    const uint8_t prev_scl = get_scl(prev);
    const uint8_t prev_sda = get_sda(prev);

    const uint8_t scl = get_scl(cur);
    const uint8_t sda = get_sda(cur);

    const uint32_t t = data[i].t;


    /*
     * START:
     *
     * SDA: HIGH -> LOW
     * SCL: HIGH
     */
    if (prev_sda == 1 && sda == 0 && scl == 1) {

      d.active = true;
      d.byte_value = 0;
      d.bit_count = 0;
      d.start_time = t;
      d.starts++;

      ESP_LOGI(
          TAG,
          "I2C %lu us: START",
          (unsigned long)t
      );

      continue;
    }


    /*
     * STOP:
     *
     * SDA: LOW -> HIGH
     * SCL: HIGH
     */
    if (prev_sda == 0 && sda == 1 && scl == 1) {

      if (d.active) {

        ESP_LOGI(
            TAG,
            "I2C %lu us: STOP",
            (unsigned long)t
        );
      }

      d.active = false;
      d.byte_value = 0;
      d.bit_count = 0;
      d.stops++;

      continue;
    }


    if (!d.active) {
      continue;
    }


    /*
     * Daten werden auf der steigenden SCL-Flanke
     * übernommen.
     */
    if (prev_scl == 0 && scl == 1) {

      /*
       * Die ersten 8 Clock-Flanken enthalten das Byte.
       */
      if (d.bit_count < 8) {

        d.byte_value <<= 1;

        if (sda) {
          d.byte_value |= 1;
        }

        d.bit_count++;

      } else {

        /*
         * 9. Clock = ACK/NACK
         *
         * SDA LOW  = ACK
         * SDA HIGH = NACK
         */
        const bool ack = (sda == 0);

        if (ack) {
          d.acks++;
        } else {
          d.nacks++;
        }

        d.bytes++;

        log_i2c_byte(
            d.byte_value,
            ack,
            t
        );

        d.byte_value = 0;
        d.bit_count = 0;
      }
    }
  }


  ESP_LOGI(TAG, "I2C summary:");
  ESP_LOGI(
      TAG,
      "  START=%lu STOP=%lu BYTES=%lu ACK=%lu NACK=%lu",
      (unsigned long)d.starts,
      (unsigned long)d.stops,
      (unsigned long)d.bytes,
      (unsigned long)d.acks,
      (unsigned long)d.nacks
  );
}


/*
 * Analyse des dritten Pins GPIO39.
 */
static void analyze_aux(
    const Sample *data,
    uint16_t count
) {

  uint32_t rising = 0;
  uint32_t falling = 0;

  uint32_t min_delta = UINT32_MAX;
  uint32_t max_delta = 0;

  uint32_t last_change = 0;

  uint8_t previous = get_aux(data[0].value);

  /*
   * Wir zählen nur tatsächliche GPIO39-Änderungen.
   */
  for (uint16_t i = 1; i < count; i++) {

    const uint8_t current = get_aux(data[i].value);

    if (current == previous) {
      continue;
    }

    const uint32_t dt = data[i].t - last_change;

    /*
     * last_change ist zunächst 0.
     * Deshalb erst ab dem zweiten echten Wechsel
     * die Differenz auswerten.
     */
    if (last_change != 0) {

      if (dt < min_delta) {
        min_delta = dt;
      }

      if (dt > max_delta) {
        max_delta = dt;
      }
    }

    last_change = data[i].t;

    if (current) {
      rising++;
    } else {
      falling++;
    }

    previous = current;
  }


  ESP_LOGI(TAG, "GPIO39 analysis:");
  ESP_LOGI(
      TAG,
      "  rising=%lu falling=%lu total=%lu",
      (unsigned long)rising,
      (unsigned long)falling,
      (unsigned long)(rising + falling)
  );

  if (min_delta != UINT32_MAX) {

    ESP_LOGI(
        TAG,
        "  min GPIO39 interval=%lu us",
        (unsigned long)min_delta
    );

    ESP_LOGI(
        TAG,
        "  max GPIO39 rate=%.1f kHz",
        1000.0f / (float)min_delta
    );
  }

  if (max_delta != 0) {

    ESP_LOGI(
        TAG,
        "  max GPIO39 interval=%lu us",
        (unsigned long)max_delta
    );
  }
}


/*
 * Allgemeine Statistik.
 */
static void analyze_statistics(
    const Sample *data,
    uint16_t count
) {

  uint32_t changes[3] = {0, 0, 0};

  uint32_t rising[3] = {0, 0, 0};
  uint32_t falling[3] = {0, 0, 0};

  uint32_t min_delta = UINT32_MAX;
  uint32_t max_delta = 0;

  for (uint16_t i = 1; i < count; i++) {

    const uint32_t dt =
        data[i].t - data[i - 1].t;

    if (dt < min_delta) {
      min_delta = dt;
    }

    if (dt > max_delta) {
      max_delta = dt;
    }

    const uint8_t old_value =
        data[i - 1].value;

    const uint8_t new_value =
        data[i].value;

    const uint8_t diff =
        old_value ^ new_value;


    for (int bit = 0; bit < 3; bit++) {

      const uint8_t mask =
          1 << bit;

      if (!(diff & mask)) {
        continue;
      }

      changes[bit]++;

      if (new_value & mask) {
        rising[bit]++;
      } else {
        falling[bit]++;
      }
    }
  }


  ESP_LOGI(TAG, "------------------------------");

  ESP_LOGI(
      TAG,
      "changes: GPIO38=%lu GPIO39=%lu GPIO40=%lu",
      (unsigned long)changes[0],
      (unsigned long)changes[1],
      (unsigned long)changes[2]
  );

  ESP_LOGI(
      TAG,
      "rising : GPIO38=%lu GPIO39=%lu GPIO40=%lu",
      (unsigned long)rising[0],
      (unsigned long)rising[1],
      (unsigned long)rising[2]
  );

  ESP_LOGI(
      TAG,
      "falling: GPIO38=%lu GPIO39=%lu GPIO40=%lu",
      (unsigned long)falling[0],
      (unsigned long)falling[1],
      (unsigned long)falling[2]
  );

  if (min_delta != UINT32_MAX) {

    ESP_LOGI(
        TAG,
        "min edge spacing: %lu us",
        (unsigned long)min_delta
    );

    ESP_LOGI(
        TAG,
        "max edge rate: %.1f kHz",
        1000.0f / (float)min_delta
    );
  }

  ESP_LOGI(
      TAG,
      "longest interval: %lu us",
      (unsigned long)max_delta
  );
}


/*
 * Rohdaten ausgeben.
 *
 * Maximal 300 Samples, damit der ESPHome-Logger nicht
 * mit tausenden Zeilen geflutet wird.
 */
static void dump_samples(
    const Sample *data,
    uint16_t count
) {

  ESP_LOGI(TAG, "DATA:");

  const uint16_t limit =
      (count < 300) ? count : 300;

  const uint32_t base =
      data[0].t;

  for (uint16_t i = 0; i < limit; i++) {

    const uint32_t rel =
        data[i].t - base;

    ESP_LOGI(
        TAG,
        "%lu us %u%u%u",
        (unsigned long)rel,
        get_scl(data[i].value),
        get_aux(data[i].value),
        get_sda(data[i].value)
    );
  }

  if (count > limit) {

    ESP_LOGI(
        TAG,
        "... %u more samples omitted",
        count - limit
    );
  }
}


/*
 * GPIO ISR
 */
static void IRAM_ATTR gpio_isr(void *arg) {

  const uint32_t now =
      (uint32_t)esp_timer_get_time();


  const uint8_t value =
      (gpio_get_level(PIN_SCL) ? 0x01 : 0) |
      (gpio_get_level(PIN_AUX) ? 0x02 : 0) |
      (gpio_get_level(PIN_SDA) ? 0x04 : 0);


  /*
   * Identische Zustände nicht erneut speichern.
   */
  if (value == last_value) {
    return;
  }


  /*
   * Sehr kurze Glitches unterdrücken.
   */
  if ((uint32_t)(now - last_edge) < GLITCH_FILTER_US) {
    return;
  }


  last_edge = now;
  last_value = value;


  /*
   * Sample speichern.
   */
  if (sample_count < MAX_SAMPLES) {

    samples[sample_count].t = now;
    samples[sample_count].value = value;

    sample_count++;
  }
}


/*
 * Setup
 */
void BusSniffer::setup() {

  /*
   * Alle drei Pins als reine Eingänge.
   *
   * WICHTIG:
   * Keine internen Pull-Ups.
   *
   * Die beiden 10-kOhm-Pullups auf der Sensorplatine
   * sind für I2C genau das, was wir erwarten.
   */
  gpio_config_t io = {};

  io.mode = GPIO_MODE_INPUT;

  io.pull_up_en =
      GPIO_PULLUP_DISABLE;

  io.pull_down_en =
      GPIO_PULLDOWN_DISABLE;

  io.intr_type =
      GPIO_INTR_ANYEDGE;

  io.pin_bit_mask =
      (1ULL << PIN_SCL) |
      (1ULL << PIN_AUX) |
      (1ULL << PIN_SDA);


  ESP_ERROR_CHECK(
      gpio_config(&io)
  );


  /*
   * GPIO-ISR-Service.
   */
  esp_err_t err =
      gpio_install_isr_service(0);

  if (err != ESP_OK &&
      err != ESP_ERR_INVALID_STATE) {

    ESP_LOGE(
        TAG,
        "gpio_install_isr_service failed: %s",
        esp_err_to_name(err)
    );

    return;
  }


  ESP_ERROR_CHECK(
      gpio_isr_handler_add(
          PIN_SCL,
          gpio_isr,
          nullptr
      )
  );

  ESP_ERROR_CHECK(
      gpio_isr_handler_add(
          PIN_AUX,
          gpio_isr,
          nullptr
      )
  );

  ESP_ERROR_CHECK(
      gpio_isr_handler_add(
          PIN_SDA,
          gpio_isr,
          nullptr
      )
  );


  /*
   * Anfangszustand aufnehmen.
   */
  last_value =
      (gpio_get_level(PIN_SCL) ? 0x01 : 0) |
      (gpio_get_level(PIN_AUX) ? 0x02 : 0) |
      (gpio_get_level(PIN_SDA) ? 0x04 : 0);

  last_edge =
      (uint32_t)esp_timer_get_time();


  ESP_LOGI(
      TAG,
      "Bus sniffer started"
  );

  ESP_LOGI(
      TAG,
      "I2C candidate: SCL=GPIO38 SDA=GPIO40"
  );

  ESP_LOGI(
      TAG,
      "Auxiliary signal: GPIO39"
  );
}


/*
 * Loop
 */
void BusSniffer::loop() {

  if (sample_count == 0) {
    return;
  }


  const uint32_t now =
      (uint32_t)esp_timer_get_time();


  /*
   * Noch Aktivität?
   */
  const uint32_t last =
      last_edge;

  if ((uint32_t)(now - last) <
      FRAME_TIMEOUT_US) {

    return;
  }


  /*
   * Wir übernehmen die Samples atomar.
   *
   * Die lokale Kopie ist static, damit sie NICHT
   * auf dem FreeRTOS-Stack liegt.
   */
  static Sample local[MAX_SAMPLES];


  uint16_t count;


  portENTER_CRITICAL(&sample_mux);

  count = sample_count;

  if (count > MAX_SAMPLES) {
    count = MAX_SAMPLES;
  }


  /*
   * Explizit kopieren.
   *
   * Kein:
   *
   *   local[i] = samples[i];
   *
   * weil samples[] volatile ist.
   */
  for (uint16_t i = 0; i < count; i++) {

    local[i].t =
        samples[i].t;

    local[i].value =
        samples[i].value;
  }


  sample_count = 0;

  portEXIT_CRITICAL(&sample_mux);


  if (count == 0) {
    return;
  }


  ESP_LOGI(
      TAG,
      "=============================="
  );

  ESP_LOGI(
      TAG,
      "Captured %u samples",
      count
  );


  if (count >= MAX_SAMPLES) {

    ESP_LOGW(
        TAG,
        "Buffer overflow - frame truncated"
    );
  }


  /*
   * Allgemeine Statistik.
   */
  analyze_statistics(
      local,
      count
  );


  /*
   * GPIO39 unabhängig untersuchen.
   */
  analyze_aux(
      local,
      count
  );


  /*
   * I2C aus GPIO38/40 dekodieren.
   */
  ESP_LOGI(
      TAG,
      "I2C decode:"
  );

  decode_i2c(
      local,
      count
  );


  /*
   * Rohdaten nur bei kleinen Frames vollständig
   * ausgeben.
   *
   * Bei den großen 1024-Sample-Frames würden wir
   * sonst den ESPHome-Logger massiv belasten.
   */
  if (count <= 300) {

    dump_samples(
        local,
        count
    );

  } else {

    ESP_LOGI(
        TAG,
        "Raw data omitted (%u samples)",
        count
    );

    ESP_LOGI(
        TAG,
        "Use I2C decoder + GPIO39 statistics above."
    );
  }


  ESP_LOGI(
      TAG,
      "=============================="
  );
}

}  // namespace bus_sniffer
}  // namespace esphome

