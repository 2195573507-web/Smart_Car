#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace smartcar_state_bridge {

inline constexpr std::size_t kSrpV4FrameOverheadBytes = 12U;
inline constexpr std::size_t kSrpV4ChassisPayloadBytes = 24U;
inline constexpr std::size_t kSrpV4ChassisFrameBytes = 36U;
inline constexpr uint8_t kSrpV4ChassisPriority = 2U;
inline constexpr uint8_t kSrpV4ImuMessageId = 0x10U;
inline constexpr uint8_t kSrpV4WheelMessageId = 0x14U;
inline constexpr uint8_t kSrpV4ChassisMessageId = 0x15U;
inline constexpr uint8_t kSrpV4ChassisType = kSrpV4ChassisMessageId;
inline constexpr uint8_t kSrpV4ChassisSchema = 1U;
inline constexpr uint8_t kSrpV4ChassisAllowedFlags = 0x0FU;
inline constexpr uint8_t kSrpV4OdometryValid = 0x04U;

enum class SrpV4FrameDecodeStatus {
  kNotAttempted,
  kAccepted,
  kSizeError,
  kMagicError,
  kLengthError,
  kCrcError,
  kEofError,
};

struct SrpV4FrameHeader {
  uint16_t payload_length{0U};
  uint8_t priority{0U};
  uint8_t message_id{0U};
  uint8_t sequence{0U};
  uint8_t flags{0U};
};

struct SrpV4FrameDecodeResult {
  SrpV4FrameDecodeStatus status{SrpV4FrameDecodeStatus::kNotAttempted};
  SrpV4FrameHeader header{};
  bool header_available{false};
  uint16_t expected_crc{0U};
  uint16_t received_crc{0U};
  std::string reason;

  bool accepted() const noexcept {
    return status == SrpV4FrameDecodeStatus::kAccepted;
  }
};

// Public, transport-independent decoder for one complete SRP v4 envelope.
// It validates common framing and CRC only; message-specific schema checks
// remain in dedicated decoders.
class SrpV4Decoder final {
 public:
  SrpV4FrameDecodeResult decode(const uint8_t *data,
                                std::size_t size) const noexcept;

  SrpV4FrameDecodeResult decode(
      const std::vector<uint8_t> &frame) const noexcept {
    return decode(frame.data(), frame.size());
  }

  static uint16_t crc16CcittFalse(const uint8_t *data,
                                  std::size_t size) noexcept;
};

enum class SrpV4DecodeStatus {
  kNotAttempted,
  kAccepted,
  kSizeError,
  kMagicError,
  kLengthError,
  kPriorityError,
  kTypeError,
  kHeaderFlagsError,
  kCrcError,
  kEofError,
  kSchemaError,
  kPayloadFlagsError,
  kReservedError,
  kOdometryInvalid,
  kNonFinite,
};

struct SrpV4ChassisSample {
  uint8_t schema{0U};
  uint8_t status_flags{0U};
  uint8_t sequence{0U};
  uint32_t timestamp_ms{0U};
  double x_m{0.0};
  double y_m{0.0};
  double yaw_rad{0.0};
  double total_dist_m{0.0};
};

struct SrpV4DecodeResult {
  SrpV4DecodeStatus status{SrpV4DecodeStatus::kNotAttempted};
  SrpV4ChassisSample sample{};
  uint16_t expected_crc{0U};
  uint16_t received_crc{0U};
  std::string reason;

  bool accepted() const noexcept {
    return status == SrpV4DecodeStatus::kAccepted;
  }
};

// Pure in-memory decoder for one complete SRP v4 chassis-state frame. It has
// no transport, ROS, or firmware dependency.
class SrpV4ChassisDecoder final {
 public:
  SrpV4DecodeResult decode(const uint8_t *data, std::size_t size) const noexcept;

  SrpV4DecodeResult decode(const SrpV4FrameDecodeResult &frame,
                           const uint8_t *data,
                           std::size_t size) const noexcept;

  SrpV4DecodeResult decode(const std::vector<uint8_t> &frame) const noexcept {
    return decode(frame.data(), frame.size());
  }

  static uint16_t crc16CcittFalse(const uint8_t *data,
                                  std::size_t size) noexcept {
    return SrpV4Decoder::crc16CcittFalse(data, size);
  }
};

const char *toString(SrpV4FrameDecodeStatus status) noexcept;
const char *toString(SrpV4DecodeStatus status) noexcept;

}  // namespace smartcar_state_bridge
