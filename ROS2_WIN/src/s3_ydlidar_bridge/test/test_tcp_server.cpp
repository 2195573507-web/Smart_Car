#include "s3_ydlidar_bridge/transport.hpp"

#include <gtest/gtest.h>

#include <arpa/inet.h>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <vector>

namespace {
void appendU16(std::vector<uint8_t> &bytes, uint16_t value) {
  bytes.push_back(static_cast<uint8_t>(value & 0xFFU));
  bytes.push_back(static_cast<uint8_t>(value >> 8U));
}

void appendU32(std::vector<uint8_t> &bytes, uint32_t value) {
  for (unsigned shift = 0U; shift < 32U; shift += 8U) {
    bytes.push_back(static_cast<uint8_t>(value >> shift));
  }
}

uint16_t crc16(const std::vector<uint8_t> &bytes, size_t begin, size_t end) {
  uint16_t crc = 0xFFFFU;
  for (size_t index = begin; index < end; ++index) {
    crc ^= bytes[index];
    for (unsigned bit = 0U; bit < 8U; ++bit) {
      crc = (crc & 1U) != 0U ? static_cast<uint16_t>((crc >> 1U) ^ 0xA001U)
                              : static_cast<uint16_t>(crc >> 1U);
    }
  }
  return crc;
}

std::vector<uint8_t> frame(uint32_t sequence) {
  const std::vector<uint8_t> payload(10U, 0xAAU);
  std::vector<uint8_t> result{'S', '3', 'R', 'D', 1U, 1U, 0U, 0U};
  appendU32(result, 1U);
  appendU32(result, 1U);
  appendU32(result, sequence);
  appendU32(result, 1U);
  appendU16(result, static_cast<uint16_t>(payload.size()));
  result.insert(result.end(), payload.begin(), payload.end());
  appendU16(result, crc16(result, 4U, result.size()));
  return result;
}

int connectTo(uint16_t port) {
  const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
  if (fd < 0) {
    return -1;
  }
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_port = htons(port);
  (void)::inet_pton(AF_INET, "127.0.0.1", &address.sin_addr);
  for (int attempt = 0; attempt < 50; ++attempt) {
    if (::connect(fd, reinterpret_cast<sockaddr *>(&address),
                  sizeof(address)) == 0) {
      return fd;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ::close(fd);
  return -1;
}
}  // namespace

TEST(TcpServer, AcceptsSplitStickyFramesAndReconnects) {
  constexpr uint16_t kPort = 18765U;
  s3_ydlidar_bridge::TcpServerConfig config;
  config.listen_address = "127.0.0.1";
  config.listen_port = kPort;
  config.max_buffer_bytes = 4096U;
  config.max_ready_frames = 8U;

  std::mutex mutex;
  std::condition_variable condition;
  std::vector<s3_ydlidar_bridge::ReceivedFrame> frames;
  std::vector<s3_ydlidar_bridge::ConnectionEvent> connection_events;
  s3_ydlidar_bridge::TcpServerTransport transport(config);
  transport.setConnectionCallback(
      [&](s3_ydlidar_bridge::ConnectionEvent event) {
        std::lock_guard<std::mutex> lock(mutex);
        connection_events.push_back(event);
        condition.notify_all();
      });
  std::string error;
  ASSERT_TRUE(transport.start(
      [&](s3_ydlidar_bridge::ReceivedFrame received) {
        std::lock_guard<std::mutex> lock(mutex);
        frames.push_back(std::move(received));
        condition.notify_all();
      },
      error)) << error;

  const auto first = frame(100U);
  const auto jump = frame(103U);
  const auto duplicate = frame(103U);
  const auto old = frame(102U);
  const int client = connectTo(kPort);
  ASSERT_GE(client, 0);
  ASSERT_EQ(::send(client, first.data(), 7U, 0), 7);
  ASSERT_EQ(::send(client, first.data() + 7U, first.size() - 7U, 0),
            static_cast<ssize_t>(first.size() - 7U));
  std::vector<uint8_t> sticky = jump;
  sticky.insert(sticky.end(), duplicate.begin(), duplicate.end());
  sticky.insert(sticky.end(), old.begin(), old.end());
  ASSERT_EQ(::send(client, sticky.data(), sticky.size(), 0),
            static_cast<ssize_t>(sticky.size()));
  (void)::shutdown(client, SHUT_RDWR);
  ::close(client);

  const int reconnect = connectTo(kPort);
  ASSERT_GE(reconnect, 0);
  const auto restarted = frame(1U);
  ASSERT_EQ(::send(reconnect, restarted.data(), restarted.size(), 0),
            static_cast<ssize_t>(restarted.size()));
  (void)::shutdown(reconnect, SHUT_RDWR);
  ::close(reconnect);

  {
    std::unique_lock<std::mutex> lock(mutex);
    ASSERT_TRUE(condition.wait_for(lock, std::chrono::seconds(2), [&] {
      return frames.size() >= 5U && connection_events.size() >= 4U;
    }));
  }
  transport.stop();
  ASSERT_EQ(frames.size(), 5U);
  EXPECT_EQ(frames[0].sequence, 100U);
  EXPECT_EQ(frames[1].sequence, 103U);
  EXPECT_EQ(frames[2].sequence, 103U);
  EXPECT_EQ(frames[3].sequence, 102U);
  EXPECT_EQ(frames[4].sequence, 1U);
  EXPECT_EQ(frames[0].connection_epoch, 1U);
  EXPECT_EQ(frames[1].connection_epoch, 1U);
  EXPECT_EQ(frames[2].connection_epoch, 1U);
  EXPECT_EQ(frames[3].connection_epoch, 1U);
  EXPECT_EQ(frames[4].connection_epoch, 2U);

  ASSERT_EQ(connection_events.size(), 4U);
  EXPECT_EQ(connection_events[0].type,
            s3_ydlidar_bridge::ConnectionEventType::kOpened);
  EXPECT_EQ(connection_events[0].connection_epoch, 1U);
  EXPECT_EQ(connection_events[1].type,
            s3_ydlidar_bridge::ConnectionEventType::kClosed);
  EXPECT_EQ(connection_events[1].connection_epoch, 1U);
  EXPECT_EQ(connection_events[2].type,
            s3_ydlidar_bridge::ConnectionEventType::kOpened);
  EXPECT_EQ(connection_events[2].connection_epoch, 2U);
  EXPECT_EQ(connection_events[3].type,
            s3_ydlidar_bridge::ConnectionEventType::kClosed);
  EXPECT_EQ(connection_events[3].connection_epoch, 2U);

  const auto stats = transport.stats();
  EXPECT_EQ(stats.connection_epoch, 2U);
  EXPECT_GE(stats.accepted_connections, 2U);
  EXPECT_GE(stats.disconnects, 2U);
}
