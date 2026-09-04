#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace smartcar_motion_gateway {

inline constexpr uint16_t kMotionMagic = 0x4D52U;
inline constexpr uint8_t kMotionVersion = 1U;
inline constexpr uint16_t kAuthPresentFlag = 0x0001U;
inline constexpr std::size_t kMotionHeaderBytes = 24U;
inline constexpr std::size_t kMotionAuthBytes = 16U;
inline constexpr std::size_t kMotionCrcBytes = 2U;
inline constexpr std::size_t kMotionMinFrameBytes =
    kMotionHeaderBytes + kMotionAuthBytes + kMotionCrcBytes;
inline constexpr std::size_t kMotionMaxPayloadBytes = 214U;
inline constexpr std::size_t kMotionMaxFrameBytes = 256U;
inline constexpr uint16_t kMinLeaseTtlMs = 20U;
inline constexpr uint16_t kMaxLeaseTtlMs = 220U;
inline constexpr float kMaxLinearMps = 0.10F;
inline constexpr float kMaxAngularRps = 0.30F;

enum class MessageType : uint8_t {
  kHello = 0x01U,
  kHelloAck = 0x02U,
  kLeaseRequest = 0x03U,
  kLeaseResponse = 0x04U,
  kMotionCommand = 0x05U,
  kStop = 0x06U,
  kStatus = 0x07U,
  kError = 0x08U,
  kHeartbeat = 0x09U,
};

enum class DecodeStatus {
  kAccepted,
  kSizeError,
  kMagicError,
  kVersionError,
  kFlagsError,
  kLengthError,
  kReservedError,
  kAuthError,
  kCrcError,
};

struct MotionFrame {
  MessageType type{MessageType::kError};
  uint32_t session_id{0U};
  uint32_t sequence{0U};
  uint32_t lease_id{0U};
  uint16_t ttl_ms{0U};
  std::vector<uint8_t> payload;
};

struct VelocityCommand {
  float linear_mps{0.0F};
  float angular_rps{0.0F};
};

struct DecodeResult {
  DecodeStatus status{DecodeStatus::kSizeError};
  MotionFrame frame{};
  std::string reason;

  bool accepted() const noexcept { return status == DecodeStatus::kAccepted; }
};

// Protocol-only codec. Callers own the key and must not log or persist it.
class MotionProtocol final {
 public:
  static std::vector<uint8_t> encode(const MotionFrame &frame,
                                     const std::vector<uint8_t> &psk);
  static DecodeResult decode(const uint8_t *data, std::size_t size,
                             const std::vector<uint8_t> &psk) noexcept;

  static uint16_t crc16CcittFalse(const uint8_t *data,
                                  std::size_t size) noexcept;
  static std::array<uint8_t, kMotionAuthBytes> hmacSha256Truncated(
      const uint8_t *data, std::size_t size,
      const std::vector<uint8_t> &psk) noexcept;
};

// Bounded TCP stream reassembler. It never stores more than one maximum-size
// frame and discards malformed data while searching for the next RM magic.
class StreamParser final {
 public:
  std::vector<DecodeResult> push(const uint8_t *data, std::size_t size,
                                 const std::vector<uint8_t> &psk);
  void clear() noexcept { buffer_.clear(); }

 private:
  std::vector<uint8_t> buffer_;
};

bool sequenceIsNewer(uint32_t candidate, uint32_t previous) noexcept;
bool validLeaseTtl(uint16_t ttl_ms) noexcept;
bool validVelocity(float linear_mps, float angular_rps) noexcept;
std::vector<uint8_t> encodeVelocityPayload(float linear_mps,
                                           float angular_rps);
bool decodeVelocityPayload(const std::vector<uint8_t> &payload,
                           float &linear_mps, float &angular_rps) noexcept;
const char *toString(DecodeStatus status) noexcept;

}  // namespace smartcar_motion_gateway
