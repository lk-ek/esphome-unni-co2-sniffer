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

static constexpr size_t MAX_PAYLOAD = 1000;

bool setup(const char *host, uint16_t port);
bool enabled();
bool send_packet(PacketType type, uint32_t capture_id, uint16_t packet_index,
                 uint16_t packet_count, const uint8_t *payload, uint16_t payload_length,
                 uint16_t flags = 0);

}  // namespace debug_udp
}  // namespace co2_monitor_0601
}  // namespace esphome
