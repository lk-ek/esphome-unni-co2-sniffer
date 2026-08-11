#include "sensirion_ble.h"

#include "esphome/core/log.h"
#include "esphome/components/esp32_ble/ble.h"
#include "esp_mac.h"
#include <esp_gap_ble_api.h>

#include <span>
#include <vector>

namespace esphome {
namespace bus_sniffer {

static const char *TAG = "bus_sniffer";

/*
 * ============================================================================
 * BLE identity test
 * ============================================================================
 *
 * Give this firmware a deliberately different Bluetooth identity so that
 * MyAmbience/CoreBluetooth cannot identify it through the original ESP32-C3
 * BT MAC / GATT System ID.
 *
 * Original observed BT identity / 0x2A23:
 *   80:F1:B2:61:67:3A
 *
 * Test identity:
 *   82:F1:B2:61:68:3A
 *
 * 0x82 has the locally-administered bit set and the multicast bit clear, so it
 * is a valid private test MAC.  esp_iface_mac_addr_set(ESP_MAC_BT) is executed
 * from a C++ constructor, i.e. before ESPHome initializes Wi-Fi/BLE in
 * app_main().  Consequently:
 *
 *   - the BLE controller uses the new identity address,
 *   - esp_read_mac(..., ESP_MAC_BT) returns the new address,
 *   - GATT characteristic 0x2A23 from the YAML returns ... 68 3A,
 *   - sensirion_ble_get_device_id() derives gadget ID 0x683A.
 *
 * This intentionally changes no sensor/BLE payload logic apart from identity.
 */
static constexpr uint8_t SENSIRION_TEST_BT_MAC[6] = {
    0x82, 0xF1, 0xB2, 0x61, 0x68, 0x3A};

static esp_err_t sensirion_test_bt_mac_set_result = ESP_FAIL;

__attribute__((constructor))
static void sensirion_set_test_bt_identity_early()
{
  sensirion_test_bt_mac_set_result = esp_iface_mac_addr_set(
      SENSIRION_TEST_BT_MAC,
      ESP_MAC_BT);
}


/*
 * ============================================================================
 * Sensirion MyCO2-compatible BLE live advertisement
 * ============================================================================
 *
 * Manufacturer data layout used by Sensirion UPT:
 *
 *   [0]  0xD5   Sensirion company ID low byte (Company ID 0x06D5)
 *   [1]  0x06   Sensirion company ID high byte
 *   [2]  0x00   live/sample advertisement type
 *   [3]  0x08   SampleType 8 = T_RH_CO2_ALT / MyCO2
 *   [4]  dev_id high
 *   [5]  dev_id low
 *   [6..7]   temperature raw uint16
 *   [8..9]   relative humidity raw uint16
 *   [10..11] CO2 ppm uint16
 *
 * Note: Sensirion's UPT BLE_example treats the manufacturer-data header
 * specially: company ID 0xD506 is serialized as D5 06. Sample payload
 * encoding is handled separately by the UPT signal encoders.
 *
 * This is deliberately advertising-only.  No GATT server/history buffer is
 * needed for live readings in scanner applications.
 *
 * v22 additionally forces the configured local name "S" into the
 * primary advertising packet.  Manufacturer data + flags + this 8-byte name
 * fit within the 31-byte legacy BLE advertising payload.
 *
 * v23 enabled ESPHome's native GATT server.
 * v24 corrects the Sensirion manufacturer-data header back to the byte order
 * used by the current official UPT BLE_example: D5 06.
 * v25 uses T_RH_CO2_ALT integer signal encoding: T*200 (signed),
 * RH*100, CO2 direct ppm; sample uint16 fields remain little-endian.
 * The rest of the over-the-air payload remains unchanged and enables ESPHome's native
 * GATT server from YAML.  This deliberately avoids mixing NimBLE-Arduino with
 * ESPHome's ESP-IDF BLE stack.
 * v26 restores the official Gadget-library company-ID byte order 06 D5,
 * while keeping the v25 UPT sample encoding and the native GATT server.
 * v27 follows Sensirion's documented MyAmbience DIY-discovery rules:
 * complete local name begins with 'S' (S) and the assigned
 * Company Identifier 0x06D5 is serialized little-endian as D5 06.
 * Sensor decoding, calibration, UPT sample scaling and GATT server stay unchanged.
 * v28 completes the Device Information Service in YAML: firmware 1.0.0
 * and System ID 0x2A23 containing the six BLE-MAC bytes.
 * v30 changed SampleType from 8 to 10.
 * v31 matches the uploaded working Sensirion CO2-Gadget reference:
 *   Local Name = "S"
 *   DataType T_RH_CO2 => SampleType 10, 6 sample bytes
 *   T = encodeTemperatureV1, RH = encodeHumidityV1, CO2 = encodeSimple
 *   all 16-bit sample fields little-endian.
 * Decoder/FSM and sensor calibration remain unchanged.
 * v32 follows uploaded Sensirion Example2 SCD30 exactly for advertisement data:
 *   T_RH_CO2_ALT => SampleType 8, 8-byte sample; trailing 00 00 reserved.
 *   T/RH use V1 encoders, CO2 simple uint16, all sample words little-endian.
 *   Local Name remains exactly "S".
 */

static constexpr uint8_t SENSIRION_BLE_COMPANY_HI = 0xD5;
static constexpr uint8_t SENSIRION_BLE_COMPANY_LO = 0x06;
static constexpr uint8_t SENSIRION_BLE_SAMPLE_ADV_TYPE = 0x00;
static constexpr uint8_t SENSIRION_BLE_SAMPLE_TYPE_MYCO2 = 0x08;

static bool sensirion_ble_have_temperature = false;
static bool sensirion_ble_have_humidity = false;
static bool sensirion_ble_have_co2 = false;

static float sensirion_ble_temperature_c = 0.0f;
static float sensirion_ble_humidity_percent = 0.0f;
static uint16_t sensirion_ble_co2_ppm = 0;

static bool sensirion_ble_device_id_ready = false;
static uint16_t sensirion_ble_device_id = 0;

// We own the actual over-the-air advertising parameters because ESPHome's
// standard connectable advertiser defaults to ~20..40 ms.  The payload is
// still the same Sensirion legacy advertisement, but the default interval is
// deliberately much slower to reduce RF duty cycle.
static uint32_t sensirion_ble_advertising_interval_ms = 2000;
static std::vector<uint8_t> sensirion_ble_raw_advertisement;
static uint32_t sensirion_ble_payload_version = 0;
static uint32_t sensirion_ble_configured_version = 0;

enum class SensirionAdvState : uint8_t {
  IDLE,
  CONFIGURING,
  STARTING,
  ADVERTISING,
  STOPPING,
};

static SensirionAdvState sensirion_ble_adv_state = SensirionAdvState::IDLE;

static esp_ble_adv_params_t sensirion_ble_adv_params = {};

static void sensirion_ble_configure_raw_advertisement();

static void sensirion_ble_request_refresh()
{
  if (sensirion_ble_raw_advertisement.empty() ||
      esp32_ble::global_ble == nullptr ||
      !esp32_ble::global_ble->is_active())
    return;

  if (sensirion_ble_adv_state == SensirionAdvState::ADVERTISING) {
    sensirion_ble_adv_state = SensirionAdvState::STOPPING;
    const esp_err_t err = esp_ble_gap_stop_advertising();
    if (err != ESP_OK) {
      // Advertising may already have been stopped by a GATT connection.
      sensirion_ble_adv_state = SensirionAdvState::IDLE;
      sensirion_ble_configure_raw_advertisement();
    }
  } else if (sensirion_ble_adv_state == SensirionAdvState::IDLE) {
    sensirion_ble_configure_raw_advertisement();
  }
}

static void sensirion_ble_configure_raw_advertisement()
{
  if (sensirion_ble_raw_advertisement.empty())
    return;

  sensirion_ble_adv_state = SensirionAdvState::CONFIGURING;
  sensirion_ble_configured_version = sensirion_ble_payload_version;
  const esp_err_t err = esp_ble_gap_config_adv_data_raw(
      sensirion_ble_raw_advertisement.data(),
      sensirion_ble_raw_advertisement.size());
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "esp_ble_gap_config_adv_data_raw failed: %s", esp_err_to_name(err));
    sensirion_ble_adv_state = SensirionAdvState::IDLE;
  }
}

void sensirion_ble_set_advertising_interval(uint32_t interval_ms)
{
  if (interval_ms < 20)
    interval_ms = 20;
  if (interval_ms > 10240)
    interval_ms = 10240;

  sensirion_ble_advertising_interval_ms = interval_ms;
  const uint16_t units = static_cast<uint16_t>((interval_ms * 1000ULL + 624) / 625);
  sensirion_ble_adv_params.adv_int_min = units;
  sensirion_ble_adv_params.adv_int_max = units;
}

void sensirion_ble_gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
  switch (event) {
    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
      if (sensirion_ble_adv_state == SensirionAdvState::CONFIGURING) {
        if (param->adv_data_raw_cmpl.status != ESP_BT_STATUS_SUCCESS) {
          ESP_LOGW(TAG, "raw BLE adv data setup failed: %d", param->adv_data_raw_cmpl.status);
          sensirion_ble_adv_state = SensirionAdvState::IDLE;
          break;
        }
        sensirion_ble_adv_state = SensirionAdvState::STARTING;
        const esp_err_t err = esp_ble_gap_start_advertising(&sensirion_ble_adv_params);
        if (err != ESP_OK) {
          ESP_LOGW(TAG, "slow BLE advertising start failed: %s", esp_err_to_name(err));
          sensirion_ble_adv_state = SensirionAdvState::IDLE;
        }
      }
      break;

    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
      if (sensirion_ble_adv_state == SensirionAdvState::STARTING) {
        if (param->adv_start_cmpl.status == ESP_BT_STATUS_SUCCESS) {
          sensirion_ble_adv_state = SensirionAdvState::ADVERTISING;
          if (sensirion_ble_configured_version != sensirion_ble_payload_version)
            sensirion_ble_request_refresh();
        } else {
          sensirion_ble_adv_state = SensirionAdvState::IDLE;
        }
      } else if (!sensirion_ble_raw_advertisement.empty() &&
                 param->adv_start_cmpl.status == ESP_BT_STATUS_SUCCESS) {
        // ESPHome's BLE server may restart its default 20..40 ms advertiser
        // after a disconnect. Replace it with our low-duty-cycle advertiser.
        sensirion_ble_adv_state = SensirionAdvState::STOPPING;
        const esp_err_t err = esp_ble_gap_stop_advertising();
        if (err != ESP_OK) {
          sensirion_ble_adv_state = SensirionAdvState::IDLE;
          sensirion_ble_configure_raw_advertisement();
        }
      }
      break;

    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
      if (sensirion_ble_adv_state == SensirionAdvState::STOPPING) {
        sensirion_ble_adv_state = SensirionAdvState::IDLE;
        sensirion_ble_configure_raw_advertisement();
      }
      break;

    default:
      break;
  }
}



uint16_t sensirion_ble_encode_temperature(float value)
{
  /*
   * Exact Sensirion UPT BLEProtocol::encodeTemperatureV1():
   * uint16_t(((((T + 45) / 175) * 65535) + 0.5)).
   */
  return static_cast<uint16_t>(
      ((((value + 45.0f) / 175.0f) * 65535.0f) + 0.5f));
}


uint16_t sensirion_ble_encode_humidity(float value)
{
  /*
   * Exact Sensirion UPT BLEProtocol::encodeHumidityV1():
   * uint16_t((((RH / 100) * 65535) + 0.5)).
   */
  return static_cast<uint16_t>(
      (((value / 100.0f) * 65535.0f) + 0.5f));
}


static inline void sensirion_ble_put_u16_le(
    std::vector<uint8_t> &data,
    size_t offset,
    uint16_t value)
{
  data[offset] =
      static_cast<uint8_t>(value & 0xFF);
  data[offset + 1] =
      static_cast<uint8_t>(value >> 8);
}


uint16_t sensirion_ble_get_device_id()
{
  if (sensirion_ble_device_id_ready)
    return sensirion_ble_device_id;

  uint8_t mac[6] = {0};

  if (esp_read_mac(mac, ESP_MAC_BT) != ESP_OK) {
    // Stable fallback; should normally never be needed on ESP32-C3.
    sensirion_ble_device_id = 0xC301;
  } else {
    // Sensirion's manufacturer payload carries a 16-bit gadget ID.
    // Derive it deterministically from the board's Bluetooth MAC.
    sensirion_ble_device_id =
        (static_cast<uint16_t>(mac[4]) << 8) |
        static_cast<uint16_t>(mac[5]);

    if (sensirion_ble_device_id == 0)
      sensirion_ble_device_id = 0xC301;
  }

  sensirion_ble_device_id_ready = true;

  return sensirion_ble_device_id;
}


static void update_sensirion_ble_advertisement()
{
  if (!sensirion_ble_have_temperature ||
      !sensirion_ble_have_humidity ||
      !sensirion_ble_have_co2)
    return;

  if (esp32_ble::global_ble == nullptr ||
      !esp32_ble::global_ble->is_active())
    return;

  const uint16_t raw_temperature =
      sensirion_ble_encode_temperature(
          sensirion_ble_temperature_c);

  const uint16_t raw_humidity =
      sensirion_ble_encode_humidity(
          sensirion_ble_humidity_percent);

  const uint16_t device_id =
      sensirion_ble_get_device_id();

  std::vector<uint8_t> data(18, 0);

  // Sensirion Bluetooth SIG Company Identifier is 0x06D5.
  // BLE Manufacturer Specific Data carries the 16-bit company ID
  // little-endian, hence bytes D5 06 on the air.
  data[0] = SENSIRION_BLE_COMPANY_HI;
  data[1] = SENSIRION_BLE_COMPANY_LO;
  data[2] = SENSIRION_BLE_SAMPLE_ADV_TYPE;
  data[3] = SENSIRION_BLE_SAMPLE_TYPE_MYCO2;

  // UPT BLE_example serializes the 16-bit device ID MSB first.
  data[4] =
      static_cast<uint8_t>(device_id >> 8);
  data[5] =
      static_cast<uint8_t>(device_id & 0xFF);

  // Measurement sample data.
  sensirion_ble_put_u16_le(
      data, 6, raw_temperature);
  sensirion_ble_put_u16_le(
      data, 8, raw_humidity);
  sensirion_ble_put_u16_le(
      data, 10, sensirion_ble_co2_ppm);

  // T_RH_CO2_ALT uses the first 8 sample bytes.  The legacy Sensirion
  // Gadget BLE library nevertheless advertises its complete 12-byte Sample
  // backing store, so bytes 12..17 remain zero.  Keeping the 18-byte total
  // manufacturer payload makes this probe byte-for-byte compatible in length.
  data[12] = 0x00;
  data[13] = 0x00;
  data[14] = 0x00;
  data[15] = 0x00;
  data[16] = 0x00;
  data[17] = 0x00;

  // Build one complete 26-byte legacy advertising packet ourselves:
  // Flags + Manufacturer Specific Data + complete local name "S".
  // This lets us control the *real* GAP advertising interval; ESPHome's
  // advertising_cycle_time only controls rotation between advertisement sets.
  sensirion_ble_raw_advertisement.clear();
  sensirion_ble_raw_advertisement.reserve(26);
  sensirion_ble_raw_advertisement.push_back(0x02);
  sensirion_ble_raw_advertisement.push_back(ESP_BLE_AD_TYPE_FLAG);
  sensirion_ble_raw_advertisement.push_back(
      ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT);
  sensirion_ble_raw_advertisement.push_back(static_cast<uint8_t>(data.size() + 1));
  sensirion_ble_raw_advertisement.push_back(ESP_BLE_AD_MANUFACTURER_SPECIFIC_TYPE);
  sensirion_ble_raw_advertisement.insert(
      sensirion_ble_raw_advertisement.end(), data.begin(), data.end());
  sensirion_ble_raw_advertisement.push_back(0x02);
  sensirion_ble_raw_advertisement.push_back(ESP_BLE_AD_TYPE_NAME_CMPL);
  sensirion_ble_raw_advertisement.push_back('S');
  sensirion_ble_payload_version++;
  sensirion_ble_request_refresh();

  ESP_LOGD(
      TAG,
      "Sensirion BLE HISTORY/T_RH_CO2_ALT [S]: %.2f C / %.1f %% / %u ppm, "
      "device 0x%04X, payload "
      "%02X %02X %02X %02X %02X %02X "
      "%02X %02X %02X %02X %02X %02X %02X %02X "
      "%02X %02X %02X %02X",
      sensirion_ble_temperature_c,
      sensirion_ble_humidity_percent,
      static_cast<unsigned>(
          sensirion_ble_co2_ppm),
      static_cast<unsigned>(device_id),
      data[0], data[1], data[2], data[3],
      data[4], data[5], data[6], data[7],
      data[8], data[9], data[10], data[11],
      data[12], data[13], data[14], data[15],
      data[16], data[17]);
}


void sensirion_ble_set_temperature_humidity(
    float temperature_c,
    float humidity_percent)
{
  sensirion_ble_temperature_c = temperature_c;
  sensirion_ble_humidity_percent = humidity_percent;
  sensirion_ble_have_temperature = true;
  sensirion_ble_have_humidity = true;

  update_sensirion_ble_advertisement();
}


void sensirion_ble_set_co2(uint16_t ppm)
{
  sensirion_ble_co2_ppm = ppm;
  sensirion_ble_have_co2 = true;

  update_sensirion_ble_advertisement();
}



void sensirion_ble_setup()
{
  sensirion_ble_adv_params = {
      .adv_int_min = 0,
      .adv_int_max = 0,
      .adv_type = ADV_TYPE_IND,
      .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
      .peer_addr = {0, 0, 0, 0, 0, 0},
      .peer_addr_type = BLE_ADDR_TYPE_PUBLIC,
      .channel_map = ADV_CHNL_ALL,
      .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
  };
  sensirion_ble_set_advertising_interval(sensirion_ble_advertising_interval_ms);

  uint8_t bt_mac[6] = {0};
  const esp_err_t bt_read_err = esp_read_mac(bt_mac, ESP_MAC_BT);

  if (sensirion_test_bt_mac_set_result == ESP_OK && bt_read_err == ESP_OK) {
    ESP_LOGI(TAG,
             "BLE identity test active: BT MAC/System ID "
             "%02X:%02X:%02X:%02X:%02X:%02X, Sensirion ID 0x%02X%02X",
             bt_mac[0], bt_mac[1], bt_mac[2], bt_mac[3], bt_mac[4], bt_mac[5],
             bt_mac[4], bt_mac[5]);
  } else {
    ESP_LOGE(TAG,
             "BLE identity test FAILED: set=%d read=%d; actual BT MAC "
             "%02X:%02X:%02X:%02X:%02X:%02X",
             static_cast<int>(sensirion_test_bt_mac_set_result),
             static_cast<int>(bt_read_err),
             bt_mac[0], bt_mac[1], bt_mac[2], bt_mac[3], bt_mac[4], bt_mac[5]);
  }

  ESP_LOGI(TAG, "Sensirion BLE advertising interval: %u ms",
           static_cast<unsigned>(sensirion_ble_advertising_interval_ms));
}

bool sensirion_ble_has_temperature() { return sensirion_ble_have_temperature; }
bool sensirion_ble_has_humidity() { return sensirion_ble_have_humidity; }
bool sensirion_ble_has_co2() { return sensirion_ble_have_co2; }
float sensirion_ble_temperature() { return sensirion_ble_temperature_c; }
float sensirion_ble_humidity() { return sensirion_ble_humidity_percent; }
uint16_t sensirion_ble_co2() { return sensirion_ble_co2_ppm; }

}  // namespace bus_sniffer
}  // namespace esphome
