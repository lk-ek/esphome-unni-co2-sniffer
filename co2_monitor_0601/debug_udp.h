// SPDX-FileCopyrightText: 2026 The esphome-unni-co2-sniffer contributors
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <cstddef>
#include <cstdint>

namespace esphome {
namespace co2_monitor_0601 {
namespace debug_udp {

enum class PacketType : uint8_t {
  I2C_LA02 = 1,
  RTRH_RAW = 2,
  RTRH_TIMING = 3,
};

// Keep debug datagrams comfortably below a full Ethernet MTU and, more
// importantly on ESP32-C3, reduce the transient lwIP pbuf allocation needed
// by sendto(). Captures are already packetized, so smaller packets only trade
// a few extra datagrams for substantially lower allocation pressure.
static constexpr size_t MAX_PAYLOAD = 512;

bool setup(const char *host, uint16_t port);
bool enabled();
bool ready_for_export();
bool sustained_resource_pressure();
void reset_after_resource_pressure();
bool send_packet(PacketType type, uint32_t capture_id, uint16_t packet_index,
                 uint16_t packet_count, const uint8_t *payload, uint16_t payload_length,
                 uint16_t flags = 0);

}  // namespace debug_udp
}  // namespace co2_monitor_0601
}  // namespace esphome
