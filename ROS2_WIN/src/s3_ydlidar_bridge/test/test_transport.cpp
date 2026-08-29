#include "s3_ydlidar_bridge/transport.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>
#include <utility>
#include <vector>

namespace {
std::filesystem::path uniqueFixturePath() {
  const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
  return std::filesystem::temp_directory_path() /
         ("s3_ydlidar_bridge_replay_" + std::to_string(stamp) + ".bin");
}
}  // namespace

TEST(Transport, ReplayEmitsExactlyOneCompletePayload) {
  const auto path = uniqueFixturePath();
  {
    std::ofstream output(path, std::ios::binary);
    ASSERT_TRUE(output.good());
    const std::vector<uint8_t> payload{0xAAU, 0x55U, 0x01U, 0x02U};
    output.write(reinterpret_cast<const char *>(payload.data()),
                 static_cast<std::streamsize>(payload.size()));
  }

  s3_ydlidar_bridge::ReplayTransport transport(path.string());
  std::vector<s3_ydlidar_bridge::ReceivedFrame> frames;
  std::string error;
  ASSERT_TRUE(transport.start(
      [&frames](s3_ydlidar_bridge::ReceivedFrame frame) {
        frames.push_back(std::move(frame));
      },
      error)) << error;
  ASSERT_EQ(frames.size(), 1U);
  EXPECT_EQ(frames.front().payload,
            (std::vector<uint8_t>{0xAAU, 0x55U, 0x01U, 0x02U}));
  EXPECT_EQ(frames.front().sequence, 1U);
  EXPECT_GT(frames.front().received_steady_ns, 0U);
  std::filesystem::remove(path);
}

TEST(Transport, ReplayRejectsMissingOrEmptyFile) {
  s3_ydlidar_bridge::ReplayTransport transport(uniqueFixturePath().string());
  std::string error;
  EXPECT_FALSE(transport.start([](s3_ydlidar_bridge::ReceivedFrame) {}, error));
  EXPECT_FALSE(error.empty());
}

TEST(Transport, UnconfiguredTransportRefusesLiveMode) {
  s3_ydlidar_bridge::UnconfiguredTransport transport;
  std::string error;
  EXPECT_FALSE(transport.start([](s3_ydlidar_bridge::ReceivedFrame) {}, error));
  EXPECT_NE(error.find("not frozen"), std::string::npos);
}
