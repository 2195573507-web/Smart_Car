#include "s3_ydlidar_bridge/framing.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <utility>

namespace s3_ydlidar_bridge {

namespace {
uint64_t steadyNowNs() {
  return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());
}

uint16_t readU16(const std::vector<uint8_t> &buffer, size_t offset) {
  return static_cast<uint16_t>(buffer[offset]) |
         static_cast<uint16_t>(buffer[offset + 1U] << 8U);
}

uint32_t readU32(const std::vector<uint8_t> &buffer, size_t offset) {
  return static_cast<uint32_t>(buffer[offset]) |
         (static_cast<uint32_t>(buffer[offset + 1U]) << 8U) |
         (static_cast<uint32_t>(buffer[offset + 2U]) << 16U) |
         (static_cast<uint32_t>(buffer[offset + 3U]) << 24U);
}

uint16_t crc16Modbus(const std::vector<uint8_t> &buffer, size_t begin,
                     size_t end) {
  uint16_t crc = 0xFFFFU;
  for (size_t index = begin; index < end; ++index) {
    crc ^= buffer[index];
    for (unsigned bit = 0; bit < 8U; ++bit) {
      crc = (crc & 1U) != 0U ? static_cast<uint16_t>((crc >> 1U) ^ 0xA001U)
                              : static_cast<uint16_t>(crc >> 1U);
    }
  }
  return crc;
}
}  // namespace

ExtractStatus S3FrameExtractor::extract(const std::vector<uint8_t> &buffer,
                                         size_t &consumed,
                                         ReceivedFrame &frame) {
  constexpr size_t kMagicBytes = 4U;
  constexpr size_t kHeaderBytes = 26U;
  constexpr size_t kCrcBytes = 2U;
  constexpr size_t kYdlidarHeaderBytes = 10U;
  constexpr uint8_t kMagic[] = {'S', '3', 'R', 'D'};
  consumed = 0U;
  frame = ReceivedFrame{};
  if (buffer.size() < kMagicBytes) {
    return ExtractStatus::kNeedMore;
  }
  if (!std::equal(std::begin(kMagic), std::end(kMagic), buffer.begin())) {
    ++magic_errors_;
    consumed = 1U;
    return ExtractStatus::kInvalid;
  }
  if (buffer.size() < kHeaderBytes) {
    return ExtractStatus::kNeedMore;
  }

  const uint8_t version = buffer[4];
  const uint8_t message_type = buffer[5];
  const uint16_t flags = readU16(buffer, 6U);
  const uint32_t device_id = readU32(buffer, 8U);
  const uint32_t stream_id = readU32(buffer, 12U);
  const uint32_t sequence = readU32(buffer, 16U);
  const uint32_t timestamp_ms = readU32(buffer, 20U);
  const size_t payload_length = readU16(buffer, 24U);
  const auto policy = policyForMessageType(message_type);
  const size_t minimum_payload =
      policy.has_value() && *policy == S3MessageTypePolicy::kRawYdlidar
          ? std::max(kYdlidarHeaderBytes, config_.min_payload_bytes)
          : config_.opaque_min_payload_bytes;
  // The envelope length bound is common to every message type.  The minimum
  // bound is policy-specific: opaque payloads are intentionally not assumed to
  // contain a YDLIDAR header.
  if (payload_length > config_.max_payload_bytes ||
      (policy.has_value() && payload_length < minimum_payload)) {
    ++length_errors_;
    consumed = 1U;
    return ExtractStatus::kInvalid;
  }
  const size_t frame_length = kHeaderBytes + payload_length + kCrcBytes;
  if (buffer.size() < frame_length) {
    return ExtractStatus::kNeedMore;
  }
  if (version != config_.expected_version) {
    ++version_errors_;
    consumed = frame_length;
    return ExtractStatus::kInvalid;
  }
  if (!policy.has_value()) {
    ++type_errors_;
    consumed = frame_length;
    return ExtractStatus::kInvalid;
  }
  const uint16_t expected_crc = crc16Modbus(buffer, 4U, kHeaderBytes +
                                                    payload_length);
  const uint16_t received_crc = readU16(buffer, kHeaderBytes + payload_length);
  if (expected_crc != received_crc) {
    ++crc_errors_;
    consumed = frame_length;
    return ExtractStatus::kInvalid;
  }
  const bool unknown_flags = (flags & ~config_.allowed_flags_mask) != 0U;
  const bool raw_payload = *policy == S3MessageTypePolicy::kRawYdlidar;
  const bool ct_mismatch =
      raw_payload &&
      ((flags & 0x0001U) !=
       (buffer[kHeaderBytes + 2U] & 0x01U));
  if (unknown_flags || ct_mismatch) {
    ++flags_errors_;
    consumed = frame_length;
    return ExtractStatus::kInvalid;
  }
  if (device_id != config_.expected_device_id ||
      stream_id != config_.expected_stream_id) {
    ++identity_errors_;
    consumed = frame_length;
    return ExtractStatus::kInvalid;
  }

  frame.version = version;
  frame.message_type = message_type;
  frame.flags = flags;
  frame.device_id = device_id;
  frame.stream_id = stream_id;
  frame.sequence = sequence;
  frame.timestamp_ms = timestamp_ms;
  frame.payload_length = static_cast<uint16_t>(payload_length);
  frame.payload_kind = raw_payload ? S3FramePayloadKind::kRawYdlidar
                                   : S3FramePayloadKind::kOpaque;
  frame.payload.assign(buffer.begin() + static_cast<ptrdiff_t>(kHeaderBytes),
                       buffer.begin() +
                           static_cast<ptrdiff_t>(kHeaderBytes + payload_length));
  frame.zero_packet = raw_payload && frame.payload.size() > 2U &&
                      (frame.payload[2] & 0x01U) != 0U;
  frame.metadata.version = version;
  frame.metadata.message_type = message_type;
  frame.metadata.flags = flags;
  frame.metadata.device_id = device_id;
  frame.metadata.stream_id = stream_id;
  frame.metadata.sequence = sequence;
  frame.metadata.timestamp_ms = timestamp_ms;
  frame.metadata.payload_length = static_cast<uint16_t>(payload_length);
  frame.metadata.payload_kind = frame.payload_kind;
  consumed = frame_length;
  ++accepted_frames_;
  if (raw_payload) {
    ++raw_frames_;
  } else {
    ++opaque_frames_;
  }
  return ExtractStatus::kFrameReady;
}

S3ProtocolCounters S3FrameExtractor::counters() const noexcept {
  return S3ProtocolCounters{accepted_frames_.load(), magic_errors_.load(),
                            version_errors_.load(), type_errors_.load(),
                            flags_errors_.load(), identity_errors_.load(),
                            length_errors_.load(), crc_errors_.load(),
                            raw_frames_.load(), opaque_frames_.load()};
}

std::optional<S3MessageTypePolicy> S3FrameExtractor::policyForMessageType(
    uint8_t message_type) const noexcept {
  if (!config_.message_type_rules.empty()) {
    for (const auto &rule : config_.message_type_rules) {
      if (rule.message_type == message_type) {
        return rule.policy;
      }
    }
    return std::nullopt;
  }
  if (message_type == config_.expected_message_type) {
    return config_.message_type_policy;
  }
  for (const uint8_t opaque_type : config_.opaque_message_types) {
    if (opaque_type == message_type) {
      return S3MessageTypePolicy::kOpaque;
    }
  }
  return std::nullopt;
}

TcpChunkAssembler::TcpChunkAssembler(std::shared_ptr<FrameExtractor> extractor,
                                      size_t max_buffer_bytes,
                                      size_t max_ready_frames)
    : extractor_(std::move(extractor)),
      max_buffer_bytes_(max_buffer_bytes),
      max_ready_frames_(max_ready_frames) {
  protocol_configured_ = extractor_ != nullptr;
}

bool TcpChunkAssembler::feed(const uint8_t *data, size_t size) {
  if (data == nullptr || size == 0 || extractor_ == nullptr) {
    return false;
  }

  if (max_buffer_bytes_ == 0U) {
    dropped_bytes_ += buffer_.size() + size;
    buffer_.clear();
    return false;
  }

  // Trim before insertion so one oversized TCP read cannot create a
  // temporary allocation larger than the configured bound.
  if (size >= max_buffer_bytes_) {
    dropped_bytes_ += buffer_.size() + (size - max_buffer_bytes_);
    buffer_.clear();
    data += size - max_buffer_bytes_;
    size = max_buffer_bytes_;
  } else if (size > max_buffer_bytes_ - buffer_.size()) {
    const size_t overflow = size - (max_buffer_bytes_ - buffer_.size());
    buffer_.erase(buffer_.begin(),
                  buffer_.begin() + static_cast<ptrdiff_t>(overflow));
    dropped_bytes_ += overflow;
  }
  buffer_.insert(buffer_.end(), data, data + size);

  bool produced = false;
  for (;;) {
    size_t consumed = 0;
    ReceivedFrame frame;
    const ExtractStatus result = extractor_->extract(buffer_, consumed, frame);
    if (result == ExtractStatus::kNotConfigured) {
      protocol_configured_ = false;
      break;
    }
    if (result == ExtractStatus::kNeedMore) {
      break;
    }
    if (result == ExtractStatus::kInvalid || consumed == 0 ||
        consumed > buffer_.size()) {
      ++invalid_frames_;
      // An extractor must normally leave at least one byte to resynchronize.
      // Stop if it reports invalid input for an empty buffer so a faulty
      // implementation cannot make this loop spin forever.
      if (buffer_.empty()) {
        break;
      }
      const size_t drop = consumed > 0U && consumed <= buffer_.size()
                              ? consumed
                              : 1U;
      buffer_.erase(buffer_.begin(),
                    buffer_.begin() + static_cast<ptrdiff_t>(drop));
      dropped_bytes_ += drop;
      continue;
    }

    buffer_.erase(buffer_.begin(),
                  buffer_.begin() + static_cast<ptrdiff_t>(consumed));
    if (frame.received_steady_ns == 0U) {
      frame.received_steady_ns = steadyNowNs();
    }
    if (max_ready_frames_ == 0U) {
      // A zero-capacity queue is useful as an explicit drop-all guard for
      // callers that want to keep parsing/diagnostics without buffering.
      ++dropped_ready_;
      ++ready_overflow_;
    } else {
      if (ready_.size() >= max_ready_frames_) {
        ready_.pop_front();
        ++dropped_ready_;
        ++ready_overflow_;
      }
      ready_.push_back(std::move(frame));
    }
    produced = true;
  }
  return produced;
}

std::vector<ReceivedFrame> TcpChunkAssembler::takeAll() {
  std::vector<ReceivedFrame> result;
  result.reserve(ready_.size());
  while (!ready_.empty()) {
    result.push_back(std::move(ready_.front()));
    ready_.pop_front();
  }
  return result;
}

ReadyQueueStats TcpChunkAssembler::readyStats() const noexcept {
  const size_t depth = ready_.size();
  return ReadyQueueStats{depth,
                         max_ready_frames_,
                         dropped_ready_,
                         ready_overflow_,
                         dropped_ready_,
                         ready_overflow_};
}

void SequenceTracker::beginConnection(uint64_t connection_epoch) noexcept {
  reset();
  connection_epoch_ = connection_epoch;
}

void SequenceTracker::endConnection(uint64_t connection_epoch) noexcept {
  if (connection_epoch_ == connection_epoch) {
    reset();
  }
}

SequenceStatus SequenceTracker::observe(uint64_t sequence, uint16_t flags) noexcept {
  if (!have_last_) {
    have_last_ = true;
    last_ = sequence;
    first_sequence_ = sequence;
    last_flags_ = flags;
    return SequenceStatus::kFirst;
  }
  if (sequence == last_) {
    return SequenceStatus::kDuplicate;
  }

  // S3RD carries a uint32 sequence. Compare in its natural modulo-2^32
  // domain so a forward producer-side skip across 0xffffffff remains forward.
  const uint32_t previous = static_cast<uint32_t>(last_);
  const uint32_t current = static_cast<uint32_t>(sequence);
  const uint32_t forward_delta = current - previous;
  if (forward_delta >= 0x80000000U) {
    return SequenceStatus::kOutOfOrder;
  }

  last_ = current;
  last_flags_ = flags;
  if (current < previous) {
    return SequenceStatus::kWrap;
  }
  if (forward_delta != 1U) {
    return SequenceStatus::kJump;
  }
  return SequenceStatus::kInOrder;
}

void SequenceTracker::reset() noexcept {
  have_last_ = false;
  last_ = 0;
  first_sequence_.reset();
  last_flags_.reset();
}

SequenceSnapshot SequenceTracker::snapshot() const noexcept {
  SequenceSnapshot result;
  result.connection_epoch = connection_epoch_;
  result.first_sequence = first_sequence_;
  if (have_last_) {
    result.last_sequence = last_;
  }
  result.last_flags = last_flags_;
  return result;
}

}  // namespace s3_ydlidar_bridge
