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
  if (payload_length < kYdlidarHeaderBytes ||
      payload_length < config_.min_payload_bytes ||
      payload_length > config_.max_payload_bytes) {
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
  if (message_type != config_.expected_message_type) {
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
  const uint8_t payload_ct = buffer[kHeaderBytes + 2U];
  const bool unknown_flags = (flags & ~config_.allowed_flags_mask) != 0U;
  const bool ct_mismatch = (flags & 0x0001U) != (payload_ct & 0x01U);
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
  frame.payload.assign(buffer.begin() + static_cast<ptrdiff_t>(kHeaderBytes),
                       buffer.begin() +
                           static_cast<ptrdiff_t>(kHeaderBytes + payload_length));
  frame.zero_packet = frame.payload.size() > 2U &&
                      (frame.payload[2] & 0x01U) != 0U;
  consumed = frame_length;
  ++accepted_frames_;
  return ExtractStatus::kFrameReady;
}

S3ProtocolCounters S3FrameExtractor::counters() const noexcept {
  return S3ProtocolCounters{accepted_frames_.load(), magic_errors_.load(),
                            version_errors_.load(), type_errors_.load(),
                            flags_errors_.load(), identity_errors_.load(),
                            length_errors_.load(), crc_errors_.load()};
}

TcpChunkAssembler::TcpChunkAssembler(std::shared_ptr<FrameExtractor> extractor,
                                     size_t max_buffer_bytes,
                                     size_t)
    : extractor_(std::move(extractor)),
      max_buffer_bytes_(max_buffer_bytes) {
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
    ready_.push_back(std::move(frame));
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
