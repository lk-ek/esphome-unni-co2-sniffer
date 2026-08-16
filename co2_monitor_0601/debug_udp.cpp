// SPDX-FileCopyrightText: 2026 The esphome-unni-co2-sniffer contributors
// SPDX-License-Identifier: GPL-3.0-or-later
#include "debug_udp.h"

#include "esphome/core/log.h"

#include "lwip/inet.h"
#include "lwip/sockets.h"
#include "esp_wifi.h"
#include "esphome/core/hal.h"

#include <cerrno>
#include <cstring>

namespace esphome {
namespace co2_monitor_0601 {
namespace debug_udp {

static const char *TAG = "debug_udp";
static int udp_socket = -1;
static sockaddr_in destination{};
static bool configured = false;
static uint32_t last_send_ms = 0;
static constexpr uint32_t MIN_SEND_GAP_MS = 5;

struct __attribute__((packed)) PacketHeader {
  char magic[4];          // "UND1"
  uint8_t version;        // 1
  uint8_t type;           // PacketType
  uint16_t flags;
  uint32_t capture_id;
  uint16_t packet_index;  // zero based
  uint16_t packet_count;
  uint16_t payload_length;
  uint16_t reserved;
};
static_assert(sizeof(PacketHeader) == 20, "Unexpected UDP debug header size");

bool setup(const char *host, uint16_t port) {
  configured = false;
  if (host == nullptr || host[0] == '\0' || port == 0) return false;

  if (udp_socket >= 0) {
    lwip_close(udp_socket);
    udp_socket = -1;
  }

  sockaddr_in dest{};
  dest.sin_family = AF_INET;
  dest.sin_port = htons(port);
  if (inet_pton(AF_INET, host, &dest.sin_addr) != 1) {
    ESP_LOGE(TAG, "Invalid IPv4 collector address '%s'", host);
    return false;
  }

  // Do not create a lwIP socket during this component's setup(). ESPHome sets
  // this component up before Wi-Fi, so the TCP/IP synchronization primitives
  // may not exist yet. Creating a socket here can trip FreeRTOS'
  // xQueueSemaphoreTake(pxQueue) assertion. Store only the destination now and
  // lazily create the socket on the first packet send, which happens from the
  // normal loop after application setup has completed.
  destination = dest;
  configured = true;
  ESP_LOGI(TAG, "UDP debug export configured: %s:%u, payload <= %u bytes (socket opens lazily)",
           host, static_cast<unsigned>(port), static_cast<unsigned>(MAX_PAYLOAD));
  return true;
}

bool enabled() { return configured; }

static bool ensure_socket() {
  if (!configured) return false;
  if (udp_socket >= 0) return true;

  const int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (sock < 0) {
    ESP_LOGW(TAG, "lazy socket() failed: errno=%d", errno);
    return false;
  }
  udp_socket = sock;
  ESP_LOGI(TAG, "UDP debug socket opened lazily");
  return true;
}

bool send_packet(PacketType type, uint32_t capture_id, uint16_t packet_index,
                 uint16_t packet_count, const uint8_t *payload, uint16_t payload_length,
                 uint16_t flags) {
  if (!enabled() || payload == nullptr || payload_length > MAX_PAYLOAD || packet_count == 0 ||
      packet_index >= packet_count)
    return false;

  // Do not touch lwIP until the station is actually associated. During boot the
  // sniffer can finish its first capture a few hundred milliseconds before DHCP
  // completes; keeping the capture pending is cheaper and safer than provoking a
  // send on a half-initialized network path.
  wifi_ap_record_t ap{};
  if (esp_wifi_sta_get_ap_info(&ap) != ESP_OK) return false;

  // One datagram at a time. This deliberately back-pressures the capture
  // exporters instead of filling lwIP pbuf/mailbox queues and hitting ENOMEM.
  const uint32_t now_ms = millis();
  if (last_send_ms != 0 && static_cast<uint32_t>(now_ms - last_send_ms) < MIN_SEND_GAP_MS)
    return false;

  if (!ensure_socket()) return false;

  static uint8_t datagram[sizeof(PacketHeader) + MAX_PAYLOAD];
  PacketHeader header{{'U', 'N', 'D', '1'}, 1, static_cast<uint8_t>(type), flags,
                      capture_id, packet_index, packet_count, payload_length, 0};
  memcpy(datagram, &header, sizeof(header));
  memcpy(datagram + sizeof(header), payload, payload_length);

  const size_t total = sizeof(header) + payload_length;
  const int sent = sendto(udp_socket, datagram, total, MSG_DONTWAIT,
                          reinterpret_cast<const sockaddr *>(&destination), sizeof(destination));
  if (sent != static_cast<int>(total)) {
    static uint32_t last_error_log_ms = 0;
    if (last_error_log_ms == 0 || static_cast<uint32_t>(now_ms - last_error_log_ms) >= 1000U) {
      last_error_log_ms = now_ms;
      ESP_LOGW(TAG, "UDP send deferred for type=%u capture=%lu packet=%u/%u: sent=%d errno=%d",
               static_cast<unsigned>(type), static_cast<unsigned long>(capture_id),
               static_cast<unsigned>(packet_index + 1), static_cast<unsigned>(packet_count), sent, errno);
    }
    return false;
  }
  last_send_ms = now_ms;
  return true;
}

}  // namespace debug_udp
}  // namespace co2_monitor_0601
}  // namespace esphome
