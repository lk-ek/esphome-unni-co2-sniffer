// SPDX-FileCopyrightText: 2026 The esphome-unni-co2-sniffer contributors
// SPDX-License-Identifier: GPL-3.0-or-later
#include "debug_udp.h"

#include "esphome/core/log.h"

#include "lwip/inet.h"
#include "lwip/sockets.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_heap_caps.h"
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
static constexpr uint32_t ENOMEM_RETRY_BACKOFF_MS = 50;
static uint32_t retry_not_before_ms = 0;
static uint32_t enomem_count = 0;

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

  // Do not touch the UDP socket until the STA has a real IPv4 address.
  // Association alone is not sufficient: esp_wifi_sta_get_ap_info() can already
  // succeed while DHCP is still running, and sendto() in that window can block
  // the single-core ESP32-C3 for hundreds of milliseconds. Keep the capture
  // pending until the ESP-NETIF station interface reports a non-zero address.
  esp_netif_t *sta = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
  if (sta == nullptr || !esp_netif_is_netif_up(sta)) return false;
  esp_netif_ip_info_t ip_info{};
  if (esp_netif_get_ip_info(sta, &ip_info) != ESP_OK || ip_info.ip.addr == 0) return false;

  // One datagram at a time. This deliberately back-pressures the capture
  // exporters instead of filling lwIP pbuf/mailbox queues and hitting ENOMEM.
  const uint32_t now_ms = millis();
  if (retry_not_before_ms != 0 && static_cast<int32_t>(now_ms - retry_not_before_ms) < 0)
    return false;
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
    const int send_errno = errno;
    if (send_errno == ENOMEM) {
      enomem_count++;
      // A failed nonblocking send can itself mean lwIP had no pbuf/mailbox
      // resources available. Do not hammer sendto() again every component loop;
      // leave the exporter pending and let the network stack recover first.
      retry_not_before_ms = now_ms + ENOMEM_RETRY_BACKOFF_MS;
    }

    static uint32_t last_error_log_ms = 0;
    if (last_error_log_ms == 0 || static_cast<uint32_t>(now_ms - last_error_log_ms) >= 1000U) {
      last_error_log_ms = now_ms;
      const size_t free_8 = heap_caps_get_free_size(MALLOC_CAP_8BIT);
      const size_t largest_8 = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
      const size_t min_8 = heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT);
      const size_t free_internal =
          heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
      const size_t largest_internal =
          heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
      ESP_LOGW(TAG,
               "UDP send deferred type=%u capture=%lu packet=%u/%u: sent=%d errno=%d "
               "heap8=%u/%u min=%u internal=%u/%u enomem=%lu backoff=%u ms",
               static_cast<unsigned>(type), static_cast<unsigned long>(capture_id),
               static_cast<unsigned>(packet_index + 1), static_cast<unsigned>(packet_count),
               sent, send_errno, static_cast<unsigned>(free_8),
               static_cast<unsigned>(largest_8), static_cast<unsigned>(min_8),
               static_cast<unsigned>(free_internal), static_cast<unsigned>(largest_internal),
               static_cast<unsigned long>(enomem_count),
               send_errno == ENOMEM ? static_cast<unsigned>(ENOMEM_RETRY_BACKOFF_MS) : 0U);
    }
    return false;
  }
  retry_not_before_ms = 0;
  last_send_ms = now_ms;
  return true;
}

}  // namespace debug_udp
}  // namespace co2_monitor_0601
}  // namespace esphome
