#include "smartcar_state_bridge/srp_v4_chassis.hpp"

#include <cmath>
#include <cstring>
#include <limits>
#include <utility>

namespace smartcar_state_bridge {

namespace {

constexpr std::size_t kLengthOffset = 2U;
constexpr std::size_t kHeaderOffset = 4U;
constexpr std::size_t kPayloadOffset = 8U;
constexpr double kPi = 3.14159265358979323846;

uint16_t readU16Le(const uint8_t *data) noexcept {
  return static_cast<uint16_t>(data[0]) |
         static_cast<uint16_t>(static_cast<uint16_t>(data[1]) << 8U);
}

uint32_t readU32Le(const uint8_t *data) noexcept {
  return static_cast<uint32_t>(data[0]) |
         (static_cast<uint32_t>(data[1]) << 8U) |
         (static_cast<uint32_t>(data[2]) << 16U) |
         (static_cast<uint32_t>(data[3]) << 24U);
}

float readF32Le(const uint8_t *data) noexcept {
  static_assert(sizeof(float) == sizeof(uint32_t),
                "SRP v4 requires a 32-bit float");
  static_assert(std::numeric_limits<float>::is_iec559,
                "SRP v4 requires IEEE-754 float representation");
  const uint32_t bits = readU32Le(data);
  float value = 0.0F;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

SrpV4DecodeResult reject(SrpV4DecodeStatus status,
                         const char *reason) noexcept {
  SrpV4DecodeResult result;
  result.status = status;
  result.reason = reason;
  return result;
}

SrpV4FrameDecodeResult rejectFrame(SrpV4FrameDecodeResult result,
                                   SrpV4FrameDecodeStatus status,
                                   const char *reason) noexcept {
  result.status = status;
  result.reason = reason;
  return result;
}

SrpV4DecodeStatus chassisStatusForFrameFailure(
    SrpV4FrameDecodeStatus status, std::size_t size) noexcept {
  if (size != kSrpV4ChassisFrameBytes) {
    return SrpV4DecodeStatus::kSizeError;
  }
  switch (status) {
    case SrpV4FrameDecodeStatus::kMagicError:
      return SrpV4DecodeStatus::kMagicError;
    case SrpV4FrameDecodeStatus::kLengthError:
      return SrpV4DecodeStatus::kLengthError;
    case SrpV4FrameDecodeStatus::kCrcError:
      return SrpV4DecodeStatus::kCrcError;
    case SrpV4FrameDecodeStatus::kEofError:
      return SrpV4DecodeStatus::kEofError;
    case SrpV4FrameDecodeStatus::kSizeError:
    case SrpV4FrameDecodeStatus::kNotAttempted:
    case SrpV4FrameDecodeStatus::kAccepted:
      return SrpV4DecodeStatus::kSizeError;
  }
  return SrpV4DecodeStatus::kSizeError;
}

}  // namespace

uint16_t SrpV4Decoder::crc16CcittFalse(
    const uint8_t *data, std::size_t size) noexcept {
  uint16_t crc = 0xFFFFU;
  for (std::size_t index = 0U; index < size; ++index) {
    crc ^= static_cast<uint16_t>(data[index]) << 8U;
    for (unsigned bit = 0U; bit < 8U; ++bit) {
      crc = (crc & 0x8000U) != 0U
                ? static_cast<uint16_t>((crc << 1U) ^ 0x1021U)
                : static_cast<uint16_t>(crc << 1U);
    }
  }
  return crc;
}

SrpV4FrameDecodeResult SrpV4Decoder::decode(
    const uint8_t *data, std::size_t size) const noexcept {
  SrpV4FrameDecodeResult result;
  if (data == nullptr || size < kSrpV4FrameOverheadBytes) {
    return rejectFrame(std::move(result), SrpV4FrameDecodeStatus::kSizeError,
                       "SRP v4 frame is shorter than the common envelope");
  }
  if (data[0] != 0xAAU || data[1] != 0x55U) {
    return rejectFrame(std::move(result), SrpV4FrameDecodeStatus::kMagicError,
                       "SRP v4 magic must be AA 55");
  }

  result.header.payload_length = readU16Le(data + kLengthOffset);
  const uint32_t logical_header = readU32Le(data + kHeaderOffset);
  result.header.priority = static_cast<uint8_t>(logical_header >> 24U);
  result.header.message_id =
      static_cast<uint8_t>((logical_header >> 16U) & 0xFFU);
  result.header.sequence =
      static_cast<uint8_t>((logical_header >> 8U) & 0xFFU);
  result.header.flags = static_cast<uint8_t>(logical_header & 0xFFU);
  result.header_available = true;

  const std::size_t expected_size =
      static_cast<std::size_t>(result.header.payload_length) +
      kSrpV4FrameOverheadBytes;
  if (size != expected_size) {
    return rejectFrame(
        std::move(result), SrpV4FrameDecodeStatus::kLengthError,
        "SRP v4 declared payload length does not match the complete frame");
  }

  const std::size_t crc_offset = kPayloadOffset + result.header.payload_length;
  const std::size_t eof_offset = crc_offset + 2U;
  if (data[eof_offset] != 0x0DU || data[eof_offset + 1U] != 0x0AU) {
    return rejectFrame(std::move(result), SrpV4FrameDecodeStatus::kEofError,
                       "SRP v4 EOF must be 0D 0A");
  }

  result.received_crc = readU16Le(data + crc_offset);
  result.expected_crc = crc16CcittFalse(
      data + kLengthOffset, 6U + result.header.payload_length);
  if (result.received_crc != result.expected_crc) {
    return rejectFrame(std::move(result), SrpV4FrameDecodeStatus::kCrcError,
                       "SRP v4 CRC16-CCITT-FALSE mismatch");
  }

  result.status = SrpV4FrameDecodeStatus::kAccepted;
  return result;
}

SrpV4DecodeResult SrpV4ChassisDecoder::decode(
    const uint8_t *data, std::size_t size) const noexcept {
  const SrpV4FrameDecodeResult frame = SrpV4Decoder{}.decode(data, size);
  if (!frame.accepted()) {
    return reject(chassisStatusForFrameFailure(frame.status, size),
                  frame.reason.c_str());
  }
  return decode(frame, data, size);
}

SrpV4DecodeResult SrpV4ChassisDecoder::decode(
    const SrpV4FrameDecodeResult &frame, const uint8_t *data,
    std::size_t size) const noexcept {
  if (!frame.accepted()) {
    return reject(chassisStatusForFrameFailure(frame.status, size),
                  frame.reason.c_str());
  }
  if (data == nullptr || size != kSrpV4ChassisFrameBytes) {
    return reject(SrpV4DecodeStatus::kSizeError,
                  "SRP chassis frame must contain exactly 36 bytes");
  }
  if (frame.header.payload_length != kSrpV4ChassisPayloadBytes) {
    return reject(SrpV4DecodeStatus::kLengthError,
                  "SRP chassis payload length must be 24");
  }
  if (frame.header.priority != kSrpV4ChassisPriority) {
    return reject(SrpV4DecodeStatus::kPriorityError,
                  "SRP chassis priority must be 2");
  }
  if (frame.header.message_id != kSrpV4ChassisMessageId) {
    return reject(SrpV4DecodeStatus::kTypeError,
                  "SRP chassis type must be 0x15");
  }
  if (frame.header.flags != 0U) {
    return reject(SrpV4DecodeStatus::kHeaderFlagsError,
                  "SRP chassis logical-header flags must be zero");
  }

  SrpV4DecodeResult result;
  result.received_crc = frame.received_crc;
  result.expected_crc = frame.expected_crc;

  const uint8_t *payload = data + kPayloadOffset;
  const uint8_t schema = payload[0];
  const uint8_t status_flags = payload[1];
  if (schema != kSrpV4ChassisSchema) {
    return reject(SrpV4DecodeStatus::kSchemaError,
                  "SRP chassis schema must be 1");
  }
  if ((status_flags & static_cast<uint8_t>(~kSrpV4ChassisAllowedFlags)) !=
      0U) {
    return reject(SrpV4DecodeStatus::kPayloadFlagsError,
                  "SRP chassis flags contain unsupported bits");
  }
  if (readU16Le(payload + 2U) != 0U) {
    return reject(SrpV4DecodeStatus::kReservedError,
                  "SRP chassis reserved field must be zero");
  }
  if ((status_flags & kSrpV4OdometryValid) == 0U) {
    return reject(SrpV4DecodeStatus::kOdometryInvalid,
                  "SRP chassis ODOMETRY_VALID is clear");
  }

  const float x_mm = readF32Le(payload + 8U);
  const float y_mm = readF32Le(payload + 12U);
  const float yaw_deg = readF32Le(payload + 16U);
  const float total_dist_m = readF32Le(payload + 20U);
  if (!std::isfinite(x_mm) || !std::isfinite(y_mm) ||
      !std::isfinite(yaw_deg) || !std::isfinite(total_dist_m)) {
    return reject(SrpV4DecodeStatus::kNonFinite,
                  "SRP chassis payload contains NaN or infinity");
  }

  result.status = SrpV4DecodeStatus::kAccepted;
  result.sample.schema = schema;
  result.sample.status_flags = status_flags;
  result.sample.sequence = frame.header.sequence;
  result.sample.timestamp_ms = readU32Le(payload + 4U);
  result.sample.x_m = static_cast<double>(x_mm) / 1000.0;
  result.sample.y_m = static_cast<double>(y_mm) / 1000.0;
  result.sample.yaw_rad = static_cast<double>(yaw_deg) * kPi / 180.0;
  result.sample.total_dist_m = static_cast<double>(total_dist_m);
  return result;
}

const char *toString(SrpV4FrameDecodeStatus status) noexcept {
  switch (status) {
    case SrpV4FrameDecodeStatus::kNotAttempted:
      return "not_attempted";
    case SrpV4FrameDecodeStatus::kAccepted:
      return "accepted";
    case SrpV4FrameDecodeStatus::kSizeError:
      return "size_error";
    case SrpV4FrameDecodeStatus::kMagicError:
      return "magic_error";
    case SrpV4FrameDecodeStatus::kLengthError:
      return "length_error";
    case SrpV4FrameDecodeStatus::kCrcError:
      return "crc_error";
    case SrpV4FrameDecodeStatus::kEofError:
      return "eof_error";
  }
  return "unknown";
}

const char *toString(SrpV4DecodeStatus status) noexcept {
  switch (status) {
    case SrpV4DecodeStatus::kNotAttempted:
      return "not_attempted";
    case SrpV4DecodeStatus::kAccepted:
      return "accepted";
    case SrpV4DecodeStatus::kSizeError:
      return "size_error";
    case SrpV4DecodeStatus::kMagicError:
      return "magic_error";
    case SrpV4DecodeStatus::kLengthError:
      return "length_error";
    case SrpV4DecodeStatus::kPriorityError:
      return "priority_error";
    case SrpV4DecodeStatus::kTypeError:
      return "type_error";
    case SrpV4DecodeStatus::kHeaderFlagsError:
      return "header_flags_error";
    case SrpV4DecodeStatus::kCrcError:
      return "crc_error";
    case SrpV4DecodeStatus::kEofError:
      return "eof_error";
    case SrpV4DecodeStatus::kSchemaError:
      return "schema_error";
    case SrpV4DecodeStatus::kPayloadFlagsError:
      return "payload_flags_error";
    case SrpV4DecodeStatus::kReservedError:
      return "reserved_error";
    case SrpV4DecodeStatus::kOdometryInvalid:
      return "odometry_invalid";
    case SrpV4DecodeStatus::kNonFinite:
      return "non_finite";
  }
  return "unknown";
}

}  // namespace smartcar_state_bridge
