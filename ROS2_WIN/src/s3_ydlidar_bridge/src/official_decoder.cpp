#include "s3_ydlidar_bridge/official_decoder.hpp"

#include "core/base/datatype.h"
#include "core/common/ChannelDevice.h"
#include "core/common/ydlidar_def.h"

#include <algorithm>
#include <cstring>

namespace s3_ydlidar_bridge {

namespace {
class ReadOnlyMemoryChannel final : public ydlidar::core::common::ChannelDevice {
 public:
  ReadOnlyMemoryChannel(const uint8_t *data, size_t size)
      : data_(data), size_(size) {}

  bool open() override { return true; }
  bool isOpen() override { return true; }
  void closePort() override {}
  size_t available() override { return size_ - offset_; }
  void flush() override {}

  int waitfordata(size_t data_count, uint32_t,
                 size_t *returned_size) override {
    const size_t available_bytes = available();
    if (returned_size != nullptr) {
      *returned_size = std::min(data_count, available_bytes);
    }
    return available_bytes >= data_count ? RESULT_OK : RESULT_TIMEOUT;
  }

  std::string readSize(size_t size) override {
    const size_t count = std::min(size, available());
    std::string result(reinterpret_cast<const char *>(data_ + offset_), count);
    offset_ += count;
    return result;
  }

  size_t writeData(const uint8_t *, size_t) override { return 0; }

  size_t readData(uint8_t *destination, size_t size) override {
    if (destination == nullptr || available() < size) {
      return 0;
    }
    std::memcpy(destination, data_ + offset_, size);
    offset_ += size;
    return size;
  }

 private:
  const uint8_t *data_;
  size_t size_;
  size_t offset_{0};
};
}  // namespace

OfficialDecoder::OfficialDecoder() : driver_(YDLIDAR_TYPE_SERIAL) {
  driver_.setIntensities(false);
  driver_.setAutoReconnect(false);
}

void OfficialDecoder::setIntensities(bool enabled) {
  driver_.setIntensities(enabled);
}

bool OfficialDecoder::decode(const uint8_t *data, size_t size,
                             std::vector<::node_info> &nodes,
                             std::string &error) {
  nodes.clear();
  error.clear();
  if (data == nullptr || size == 0) {
    error = "empty YDLIDAR payload";
    return false;
  }

  ReadOnlyMemoryChannel channel(data, size);
  const size_t capacity =
      ydlidar::core::common::DriverInterface::MAX_SCAN_NODES;
  nodes.resize(capacity);
  size_t count = capacity;
  const result_t result = driver_.parseMemoryChannel(&channel, nodes.data(), count);
  nodes.resize(count);
  if (!IS_OK(result)) {
    error = "official YDLIDAR parser rejected payload";
    nodes.clear();
    return false;
  }
  return true;
}

}  // namespace s3_ydlidar_bridge
