#include "s3_ydlidar_bridge/official_decoder.hpp"
#include "s3_ydlidar_bridge/scan_mapper.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <vector>

namespace {
void appendU16(std::vector<uint8_t> &bytes, uint16_t value) {
  bytes.push_back(static_cast<uint8_t>(value & 0xFFU));
  bytes.push_back(static_cast<uint8_t>(value >> 8U));
}

std::vector<uint8_t> makeTrianglePacket(bool corrupt_checksum) {
  const uint16_t sample_count = 4U;
  const uint8_t ct = 0x00U;  // normal packet; ring-start packets carry one sample
  const uint16_t fsa = static_cast<uint16_t>((0U * 64U) << 1U | 1U);
  const uint16_t lsa = static_cast<uint16_t>((3U * 64U) << 1U | 1U);
  const uint16_t samples[] = {4000U, 8000U, 0U, 12000U};
  uint16_t checksum = 0x55AAU;
  checksum ^= static_cast<uint16_t>(ct) | (sample_count << 8U);
  checksum ^= fsa;
  checksum ^= lsa;
  for (uint16_t sample : samples) {
    checksum ^= sample;
  }
  if (corrupt_checksum) {
    checksum ^= 0x0001U;
  }

  std::vector<uint8_t> frame{0xAAU, 0x55U, ct,
                             static_cast<uint8_t>(sample_count)};
  appendU16(frame, fsa);
  appendU16(frame, lsa);
  appendU16(frame, checksum);
  for (uint16_t sample : samples) {
    appendU16(frame, sample);
  }
  return frame;
}

std::vector<uint8_t> makeTriangleIntensityPacket() {
  const uint16_t sample_count = 2U;
  const uint8_t ct = 0x00U;
  const uint16_t fsa = static_cast<uint16_t>((0U * 64U) << 1U | 1U);
  const uint16_t lsa = static_cast<uint16_t>((2U * 64U) << 1U | 1U);
  const uint8_t qualities[] = {24U, 36U};
  const uint16_t distances[] = {4000U, 8000U};
  uint16_t checksum = 0x55AAU;
  checksum ^= static_cast<uint16_t>(ct) | (sample_count << 8U);
  checksum ^= fsa;
  checksum ^= lsa;
  for (size_t index = 0U; index < sample_count; ++index) {
    checksum ^= qualities[index];
    checksum ^= distances[index];
  }
  std::vector<uint8_t> frame{0xAAU, 0x55U, ct,
                             static_cast<uint8_t>(sample_count)};
  appendU16(frame, fsa);
  appendU16(frame, lsa);
  appendU16(frame, checksum);
  for (size_t index = 0U; index < sample_count; ++index) {
    frame.push_back(qualities[index]);
    appendU16(frame, distances[index]);
  }
  return frame;
}
}  // namespace

TEST(OfficialDecoder, UsesOfficialTriangleParserForMemoryFrame) {
  s3_ydlidar_bridge::OfficialDecoder decoder;
  const auto frame = makeTrianglePacket(false);
  std::vector<::node_info> nodes;
  std::string error;
  ASSERT_TRUE(decoder.decode(frame.data(), frame.size(), nodes, error)) << error;
  ASSERT_EQ(nodes.size(), 4U);
  EXPECT_EQ(nodes[0].distance_q2, 4000U);
  EXPECT_EQ(nodes[1].distance_q2, 8000U);
  EXPECT_EQ(nodes[2].distance_q2, 0U);
  EXPECT_EQ(nodes[3].distance_q2, 12000U);

  s3_ydlidar_bridge::ScanMapper mapper(s3_ydlidar_bridge::ScanMapperConfig{});
  const auto scan = mapper.map(nodes, rclcpp::Time(1));
  EXPECT_TRUE(std::any_of(scan.ranges.begin(), scan.ranges.end(),
                          [](float value) { return value == 1.0F; }));
}

TEST(OfficialDecoder, RejectsChecksumError) {
  s3_ydlidar_bridge::OfficialDecoder decoder;
  const auto frame = makeTrianglePacket(true);
  std::vector<::node_info> nodes;
  std::string error;
  EXPECT_FALSE(decoder.decode(frame.data(), frame.size(), nodes, error));
  EXPECT_TRUE(nodes.empty());
  EXPECT_FALSE(error.empty());
}

TEST(OfficialDecoder, UsesOfficialIntensityParserWhenConfigured) {
  s3_ydlidar_bridge::OfficialDecoder decoder;
  decoder.setIntensities(true);
  const auto frame = makeTriangleIntensityPacket();
  std::vector<::node_info> nodes;
  std::string error;
  ASSERT_TRUE(decoder.decode(frame.data(), frame.size(), nodes, error)) << error;
  ASSERT_EQ(nodes.size(), 2U);
  EXPECT_EQ(nodes[0].distance_q2, 4000U);
  EXPECT_EQ(nodes[1].distance_q2, 8000U);
  EXPECT_EQ(nodes[0].sync_quality, 24U);
  EXPECT_EQ(nodes[1].sync_quality, 36U);
}
