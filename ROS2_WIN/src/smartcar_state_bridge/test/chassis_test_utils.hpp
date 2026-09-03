#pragma once

#include "smartcar_state_bridge/srp_v4_chassis.hpp"

#include <array>
#include <cstdint>
#include <cstring>
#include <vector>

namespace smartcar_state_bridge::test {

inline constexpr std::array<uint8_t, kSrpV4ChassisFrameBytes>
    kGoldenChassisFrame{{
        0xAAU, 0x55U, 0x18U, 0x00U, 0x00U, 0x2AU, 0x15U, 0x02U, 0x01U,
        0x04U, 0x00U, 0x00U, 0xE8U, 0x03U, 0x00U, 0x00U, 0x00U, 0x00U,
        0x7AU, 0x44U, 0x00U, 0x00U, 0xFAU, 0xC3U, 0x00U, 0x00U, 0x33U,
        0x43U, 0x00U, 0x00U, 0x48U, 0x41U, 0x7FU, 0xC0U, 0x0DU, 0x0AU,
    }};

inline std::vector<uint8_t> goldenFrame() {
  return {kGoldenChassisFrame.begin(), kGoldenChassisFrame.end()};
}

inline void writeU32Le(std::vector<uint8_t> &frame, std::size_t offset,
                       uint32_t value) {
  frame[offset] = static_cast<uint8_t>(value & 0xFFU);
  frame[offset + 1U] = static_cast<uint8_t>((value >> 8U) & 0xFFU);
  frame[offset + 2U] = static_cast<uint8_t>((value >> 16U) & 0xFFU);
  frame[offset + 3U] = static_cast<uint8_t>((value >> 24U) & 0xFFU);
}

inline void writeU16Le(std::vector<uint8_t> &frame, std::size_t offset,
                       uint16_t value) {
  frame[offset] = static_cast<uint8_t>(value & 0xFFU);
  frame[offset + 1U] = static_cast<uint8_t>((value >> 8U) & 0xFFU);
}

inline void writeF32Le(std::vector<uint8_t> &frame, std::size_t offset,
                       float value) {
  uint32_t bits = 0U;
  static_assert(sizeof(bits) == sizeof(value));
  std::memcpy(&bits, &value, sizeof(value));
  writeU32Le(frame, offset, bits);
}

inline void repairCrc(std::vector<uint8_t> &frame) {
  const uint16_t payload_length = static_cast<uint16_t>(frame[2U]) |
                                  static_cast<uint16_t>(frame[3U] << 8U);
  const std::size_t crc_offset = 8U + payload_length;
  const uint16_t crc =
      SrpV4Decoder::crc16CcittFalse(frame.data() + 2U, 6U + payload_length);
  frame[crc_offset] = static_cast<uint8_t>(crc & 0xFFU);
  frame[crc_offset + 1U] = static_cast<uint8_t>((crc >> 8U) & 0xFFU);
}

inline std::vector<uint8_t> makeFrame(uint32_t timestamp_ms, float x_mm,
                                      float y_mm, float yaw_deg,
                                      float total_dist_m,
                                      uint8_t status_flags = 0x04U,
                                      uint8_t sequence = 0x2AU) {
  auto frame = goldenFrame();
  frame[5U] = sequence;
  frame[9U] = status_flags;
  writeU32Le(frame, 12U, timestamp_ms);
  writeF32Le(frame, 16U, x_mm);
  writeF32Le(frame, 20U, y_mm);
  writeF32Le(frame, 24U, yaw_deg);
  writeF32Le(frame, 28U, total_dist_m);
  repairCrc(frame);
  return frame;
}

inline std::vector<uint8_t> makeSrpFrame(uint8_t message_id,
                                         uint8_t sequence,
                                         uint16_t payload_length,
                                         uint8_t priority = 2U,
                                         uint8_t header_flags = 0U) {
  std::vector<uint8_t> frame(
      static_cast<std::size_t>(payload_length) + kSrpV4FrameOverheadBytes,
      0U);
  frame[0U] = 0xAAU;
  frame[1U] = 0x55U;
  writeU16Le(frame, 2U, payload_length);
  frame[4U] = header_flags;
  frame[5U] = sequence;
  frame[6U] = message_id;
  frame[7U] = priority;
  for (std::size_t index = 0U; index < payload_length; ++index) {
    frame[8U + index] = static_cast<uint8_t>(index + 1U);
  }
  frame[frame.size() - 2U] = 0x0DU;
  frame[frame.size() - 1U] = 0x0AU;
  repairCrc(frame);
  return frame;
}

}  // namespace smartcar_state_bridge::test
