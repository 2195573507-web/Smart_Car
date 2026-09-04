#include "smartcar_motion_gateway/motion_protocol.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>

namespace smartcar_motion_gateway {
namespace {

constexpr uint8_t kMagicByte0 = 0x52U;
constexpr uint8_t kMagicByte1 = 0x4DU;
constexpr std::size_t kVersionOffset = 2U;
constexpr std::size_t kTypeOffset = 3U;
constexpr std::size_t kFlagsOffset = 4U;
constexpr std::size_t kPayloadLengthOffset = 6U;
constexpr std::size_t kSessionOffset = 8U;
constexpr std::size_t kSequenceOffset = 12U;
constexpr std::size_t kLeaseOffset = 16U;
constexpr std::size_t kTtlOffset = 20U;
constexpr std::size_t kReservedOffset = 22U;

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

void writeU16Le(std::vector<uint8_t> &out, std::size_t offset,
                uint16_t value) {
  out[offset] = static_cast<uint8_t>(value & 0xFFU);
  out[offset + 1U] = static_cast<uint8_t>(value >> 8U);
}

void writeU32Le(std::vector<uint8_t> &out, std::size_t offset,
                uint32_t value) {
  for (unsigned index = 0U; index < 4U; ++index) {
    out[offset + index] = static_cast<uint8_t>(value >> (index * 8U));
  }
}

void writeF32Le(std::vector<uint8_t> &out, std::size_t offset,
                float value) {
  static_assert(sizeof(float) == sizeof(uint32_t), "float32 is required");
  uint32_t bits = 0U;
  std::memcpy(&bits, &value, sizeof(bits));
  writeU32Le(out, offset, bits);
}

float readF32Le(const uint8_t *data) noexcept {
  static_assert(sizeof(float) == sizeof(uint32_t), "float32 is required");
  static_assert(std::numeric_limits<float>::is_iec559,
                "IEEE-754 float32 is required");
  const uint32_t bits = readU32Le(data);
  float value = 0.0F;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

DecodeResult reject(DecodeStatus status, const char *reason) noexcept {
  DecodeResult result;
  result.status = status;
  result.reason = reason;
  return result;
}

bool constantTimeEqual(const uint8_t *left, const uint8_t *right,
                       std::size_t size) noexcept {
  uint8_t difference = 0U;
  for (std::size_t index = 0U; index < size; ++index) {
    difference |= static_cast<uint8_t>(left[index] ^ right[index]);
  }
  return difference == 0U;
}

uint32_t rotr(uint32_t value, unsigned amount) noexcept {
  return (value >> amount) | (value << (32U - amount));
}

std::array<uint8_t, 32U> sha256(const uint8_t *data,
                                std::size_t size) noexcept {
  constexpr std::array<uint32_t, 64U> kRoundConstants{{
      0x428a2f98U, 0x71374491U, 0xb5c0fbcfU, 0xe9b5dba5U,
      0x3956c25bU, 0x59f111f1U, 0x923f82a4U, 0xab1c5ed5U,
      0xd807aa98U, 0x12835b01U, 0x243185beU, 0x550c7dc3U,
      0x72be5d74U, 0x80deb1feU, 0x9bdc06a7U, 0xc19bf174U,
      0xe49b69c1U, 0xefbe4786U, 0x0fc19dc6U, 0x240ca1ccU,
      0x2de92c6fU, 0x4a7484aaU, 0x5cb0a9dcU, 0x76f988daU,
      0x983e5152U, 0xa831c66dU, 0xb00327c8U, 0xbf597fc7U,
      0xc6e00bf3U, 0xd5a79147U, 0x06ca6351U, 0x14292967U,
      0x27b70a85U, 0x2e1b2138U, 0x4d2c6dfcU, 0x53380d13U,
      0x650a7354U, 0x766a0abbU, 0x81c2c92eU, 0x92722c85U,
      0xa2bfe8a1U, 0xa81a664bU, 0xc24b8b70U, 0xc76c51a3U,
      0xd192e819U, 0xd6990624U, 0xf40e3585U, 0x106aa070U,
      0x19a4c116U, 0x1e376c08U, 0x2748774cU, 0x34b0bcb5U,
      0x391c0cb3U, 0x4ed8aa4aU, 0x5b9cca4fU, 0x682e6ff3U,
      0x748f82eeU, 0x78a5636fU, 0x84c87814U, 0x8cc70208U,
      0x90befffaU, 0xa4506cebU, 0xbef9a3f7U, 0xc67178f2U,
  }};
  std::array<uint32_t, 8U> state{{
      0x6a09e667U, 0xbb67ae85U, 0x3c6ef372U, 0xa54ff53aU,
      0x510e527fU, 0x9b05688cU, 0x1f83d9abU, 0x5be0cd19U,
  }};

  const uint64_t bit_size = static_cast<uint64_t>(size) * 8U;
  const std::size_t padded_size = ((size + 9U + 63U) / 64U) * 64U;
  std::vector<uint8_t> padded(padded_size, 0U);
  if (size != 0U) std::memcpy(padded.data(), data, size);
  padded[size] = 0x80U;
  for (unsigned index = 0U; index < 8U; ++index) {
    padded[padded.size() - 1U - index] =
        static_cast<uint8_t>(bit_size >> (index * 8U));
  }

  for (std::size_t offset = 0U; offset < padded.size(); offset += 64U) {
    std::array<uint32_t, 64U> words{};
    for (std::size_t index = 0U; index < 16U; ++index) {
      const std::size_t base = offset + index * 4U;
      words[index] = (static_cast<uint32_t>(padded[base]) << 24U) |
                     (static_cast<uint32_t>(padded[base + 1U]) << 16U) |
                     (static_cast<uint32_t>(padded[base + 2U]) << 8U) |
                     static_cast<uint32_t>(padded[base + 3U]);
    }
    for (std::size_t index = 16U; index < words.size(); ++index) {
      const uint32_t s0 = rotr(words[index - 15U], 7U) ^
                          rotr(words[index - 15U], 18U) ^
                          (words[index - 15U] >> 3U);
      const uint32_t s1 = rotr(words[index - 2U], 17U) ^
                          rotr(words[index - 2U], 19U) ^
                          (words[index - 2U] >> 10U);
      words[index] = words[index - 16U] + s0 + words[index - 7U] + s1;
    }
    uint32_t a = state[0];
    uint32_t b = state[1];
    uint32_t c = state[2];
    uint32_t d = state[3];
    uint32_t e = state[4];
    uint32_t f = state[5];
    uint32_t g = state[6];
    uint32_t h = state[7];
    for (std::size_t index = 0U; index < words.size(); ++index) {
      const uint32_t sum1 = rotr(e, 6U) ^ rotr(e, 11U) ^ rotr(e, 25U);
      const uint32_t choice = (e & f) ^ ((~e) & g);
      const uint32_t temp1 = h + sum1 + choice + kRoundConstants[index] +
                             words[index];
      const uint32_t sum0 = rotr(a, 2U) ^ rotr(a, 13U) ^ rotr(a, 22U);
      const uint32_t majority = (a & b) ^ (a & c) ^ (b & c);
      const uint32_t temp2 = sum0 + majority;
      h = g;
      g = f;
      f = e;
      e = d + temp1;
      d = c;
      c = b;
      b = a;
      a = temp1 + temp2;
    }
    state[0] += a;
    state[1] += b;
    state[2] += c;
    state[3] += d;
    state[4] += e;
    state[5] += f;
    state[6] += g;
    state[7] += h;
  }

  std::array<uint8_t, 32U> digest{};
  for (std::size_t index = 0U; index < state.size(); ++index) {
    for (unsigned byte = 0U; byte < 4U; ++byte) {
      digest[index * 4U + byte] = static_cast<uint8_t>(
          state[index] >> ((3U - byte) * 8U));
    }
  }
  return digest;
}

}  // namespace

uint16_t MotionProtocol::crc16CcittFalse(const uint8_t *data,
                                         std::size_t size) noexcept {
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

std::array<uint8_t, kMotionAuthBytes> MotionProtocol::hmacSha256Truncated(
    const uint8_t *data, std::size_t size,
    const std::vector<uint8_t> &psk) noexcept {
  std::array<uint8_t, 64U> key_block{};
  if (psk.size() > key_block.size()) {
    const auto key_hash = sha256(psk.data(), psk.size());
    std::copy(key_hash.begin(), key_hash.end(), key_block.begin());
  } else if (!psk.empty()) {
    std::copy(psk.begin(), psk.end(), key_block.begin());
  }
  std::array<uint8_t, 64U> inner_pad{};
  std::array<uint8_t, 64U> outer_pad{};
  for (std::size_t index = 0U; index < key_block.size(); ++index) {
    inner_pad[index] = static_cast<uint8_t>(key_block[index] ^ 0x36U);
    outer_pad[index] = static_cast<uint8_t>(key_block[index] ^ 0x5CU);
  }
  std::vector<uint8_t> inner;
  inner.reserve(inner_pad.size() + size);
  inner.insert(inner.end(), inner_pad.begin(), inner_pad.end());
  if (size != 0U) inner.insert(inner.end(), data, data + size);
  const auto inner_hash = sha256(inner.data(), inner.size());
  std::array<uint8_t, 96U> outer{};
  std::copy(outer_pad.begin(), outer_pad.end(), outer.begin());
  std::copy(inner_hash.begin(), inner_hash.end(), outer.begin() + 64U);
  const auto digest = sha256(outer.data(), outer.size());
  std::array<uint8_t, kMotionAuthBytes> result{};
  std::copy_n(digest.begin(), result.size(), result.begin());
  return result;
}

std::vector<uint8_t> MotionProtocol::encode(const MotionFrame &frame,
                                            const std::vector<uint8_t> &psk) {
  if (psk.empty() || frame.payload.size() > kMotionMaxPayloadBytes) {
    return {};
  }
  const std::size_t auth_offset = kMotionHeaderBytes + frame.payload.size();
  const std::size_t crc_offset = auth_offset + kMotionAuthBytes;
  std::vector<uint8_t> encoded(crc_offset + kMotionCrcBytes, 0U);
  encoded[0] = kMagicByte0;
  encoded[1] = kMagicByte1;
  encoded[kVersionOffset] = kMotionVersion;
  encoded[kTypeOffset] = static_cast<uint8_t>(frame.type);
  writeU16Le(encoded, kFlagsOffset, kAuthPresentFlag);
  writeU16Le(encoded, kPayloadLengthOffset,
             static_cast<uint16_t>(frame.payload.size()));
  writeU32Le(encoded, kSessionOffset, frame.session_id);
  writeU32Le(encoded, kSequenceOffset, frame.sequence);
  writeU32Le(encoded, kLeaseOffset, frame.lease_id);
  writeU16Le(encoded, kTtlOffset, frame.ttl_ms);
  if (!frame.payload.empty()) {
    std::copy(frame.payload.begin(), frame.payload.end(),
              encoded.begin() + kMotionHeaderBytes);
  }
  const auto auth = hmacSha256Truncated(encoded.data() + kVersionOffset,
                                        kMotionHeaderBytes - kVersionOffset +
                                            frame.payload.size(),
                                        psk);
  std::copy(auth.begin(), auth.end(), encoded.begin() + auth_offset);
  writeU16Le(encoded, crc_offset,
             crc16CcittFalse(encoded.data() + kVersionOffset,
                              crc_offset - kVersionOffset));
  return encoded;
}

DecodeResult MotionProtocol::decode(const uint8_t *data, std::size_t size,
                                    const std::vector<uint8_t> &psk) noexcept {
  if (data == nullptr || size < kMotionMinFrameBytes ||
      size > kMotionMaxFrameBytes) {
    return reject(DecodeStatus::kSizeError, "frame size is outside v1 bounds");
  }
  if (data[0] != kMagicByte0 || data[1] != kMagicByte1) {
    return reject(DecodeStatus::kMagicError, "magic must be RM");
  }
  if (data[kVersionOffset] != kMotionVersion) {
    return reject(DecodeStatus::kVersionError, "unsupported protocol version");
  }
  if (readU16Le(data + kFlagsOffset) != kAuthPresentFlag) {
    return reject(DecodeStatus::kFlagsError, "AUTH_PRESENT must be the only flag");
  }
  const uint16_t payload_size = readU16Le(data + kPayloadLengthOffset);
  if (payload_size > kMotionMaxPayloadBytes ||
      size != kMotionMinFrameBytes + payload_size) {
    return reject(DecodeStatus::kLengthError,
                  "declared payload length does not match frame size");
  }
  if (readU16Le(data + kReservedOffset) != 0U) {
    return reject(DecodeStatus::kReservedError, "reserved field must be zero");
  }
  const std::size_t auth_offset = kMotionHeaderBytes + payload_size;
  const std::size_t crc_offset = auth_offset + kMotionAuthBytes;
  if (psk.empty()) {
    return reject(DecodeStatus::kAuthError, "authentication key is unavailable");
  }
  const auto expected_auth = hmacSha256Truncated(
      data + kVersionOffset,
      kMotionHeaderBytes - kVersionOffset + payload_size, psk);
  if (!constantTimeEqual(expected_auth.data(), data + auth_offset,
                         expected_auth.size())) {
    return reject(DecodeStatus::kAuthError, "HMAC verification failed");
  }
  if (readU16Le(data + crc_offset) !=
      crc16CcittFalse(data + kVersionOffset, crc_offset - kVersionOffset)) {
    return reject(DecodeStatus::kCrcError, "CRC16-CCITT-FALSE verification failed");
  }

  DecodeResult result;
  result.status = DecodeStatus::kAccepted;
  result.frame.type = static_cast<MessageType>(data[kTypeOffset]);
  result.frame.session_id = readU32Le(data + kSessionOffset);
  result.frame.sequence = readU32Le(data + kSequenceOffset);
  result.frame.lease_id = readU32Le(data + kLeaseOffset);
  result.frame.ttl_ms = readU16Le(data + kTtlOffset);
  result.frame.payload.assign(data + kMotionHeaderBytes, data + auth_offset);
  return result;
}

std::vector<DecodeResult> StreamParser::push(const uint8_t *data,
                                              std::size_t size,
                                              const std::vector<uint8_t> &psk) {
  std::vector<DecodeResult> results;
  if (data == nullptr && size != 0U) {
    results.push_back(reject(DecodeStatus::kSizeError, "null TCP input"));
    return results;
  }
  for (std::size_t incoming = 0U; incoming < size; ++incoming) {
    if (buffer_.size() == kMotionMaxFrameBytes) {
      results.push_back(reject(DecodeStatus::kLengthError,
                               "TCP reassembly exceeded 256-byte frame limit"));
      buffer_.clear();
    }
    buffer_.push_back(data[incoming]);
    for (;;) {
      if (buffer_.size() < 2U) break;
      // Find the marker directly to retain a partial RM across recv()
      // boundaries without retaining an unbounded noisy prefix.
      std::size_t magic_index = buffer_.size();
      for (std::size_t index = 0U; index + 1U < buffer_.size(); ++index) {
        if (buffer_[index] == kMagicByte0 && buffer_[index + 1U] == kMagicByte1) {
          magic_index = index;
          break;
        }
      }
      if (magic_index == buffer_.size()) {
        const bool retain_prefix = buffer_.back() == kMagicByte0;
        buffer_.assign(retain_prefix ? 1U : 0U,
                       retain_prefix ? buffer_.back() : 0U);
        results.push_back(reject(DecodeStatus::kMagicError,
                                 "TCP stream does not start with RM magic"));
        break;
      }
      if (magic_index != 0U) {
        buffer_.erase(buffer_.begin(), buffer_.begin() + magic_index);
        results.push_back(reject(DecodeStatus::kMagicError,
                                 "discarded bytes before next RM magic"));
      }
      if (buffer_.size() < kMotionHeaderBytes) break;
      const uint16_t payload_size = readU16Le(buffer_.data() + kPayloadLengthOffset);
      if (payload_size > kMotionMaxPayloadBytes) {
        results.push_back(reject(DecodeStatus::kLengthError,
                                 "declared payload exceeds 214 bytes"));
        buffer_.erase(buffer_.begin());
        continue;
      }
      const std::size_t total_size = kMotionMinFrameBytes + payload_size;
      if (buffer_.size() < total_size) break;
      results.push_back(MotionProtocol::decode(buffer_.data(), total_size, psk));
      buffer_.erase(buffer_.begin(), buffer_.begin() + total_size);
    }
  }
  return results;
}

bool sequenceIsNewer(uint32_t candidate, uint32_t previous) noexcept {
  return static_cast<int32_t>(candidate - previous) > 0;
}

bool validLeaseTtl(uint16_t ttl_ms) noexcept {
  return ttl_ms >= kMinLeaseTtlMs && ttl_ms <= kMaxLeaseTtlMs;
}

bool validVelocity(float linear_mps, float angular_rps) noexcept {
  return std::isfinite(linear_mps) && std::isfinite(angular_rps) &&
         std::abs(linear_mps) <= kMaxLinearMps &&
         std::abs(angular_rps) <= kMaxAngularRps;
}

std::vector<uint8_t> encodeVelocityPayload(float linear_mps,
                                           float angular_rps) {
  if (!validVelocity(linear_mps, angular_rps)) return {};
  std::vector<uint8_t> payload(8U, 0U);
  writeF32Le(payload, 0U, linear_mps);
  writeF32Le(payload, 4U, angular_rps);
  return payload;
}

bool decodeVelocityPayload(const std::vector<uint8_t> &payload,
                           float &linear_mps, float &angular_rps) noexcept {
  if (payload.size() != 8U) return false;
  linear_mps = readF32Le(payload.data());
  angular_rps = readF32Le(payload.data() + 4U);
  return validVelocity(linear_mps, angular_rps);
}

const char *toString(DecodeStatus status) noexcept {
  switch (status) {
    case DecodeStatus::kAccepted: return "accepted";
    case DecodeStatus::kSizeError: return "size_error";
    case DecodeStatus::kMagicError: return "magic_error";
    case DecodeStatus::kVersionError: return "version_error";
    case DecodeStatus::kFlagsError: return "flags_error";
    case DecodeStatus::kLengthError: return "length_error";
    case DecodeStatus::kReservedError: return "reserved_error";
    case DecodeStatus::kAuthError: return "auth_error";
    case DecodeStatus::kCrcError: return "crc_error";
  }
  return "unknown";
}

}  // namespace smartcar_motion_gateway
