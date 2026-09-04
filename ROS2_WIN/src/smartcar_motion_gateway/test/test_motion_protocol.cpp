#include "smartcar_motion_gateway/motion_protocol.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <regex>
#include <string>
#include <vector>

namespace {

std::vector<uint8_t> fromHex(const std::string &hex) {
  if ((hex.size() % 2U) != 0U) return {};
  const auto value = [](char character) -> int {
    if (character >= '0' && character <= '9') return character - '0';
    if (character >= 'a' && character <= 'f') return character - 'a' + 10;
    if (character >= 'A' && character <= 'F') return character - 'A' + 10;
    return -1;
  };
  std::vector<uint8_t> bytes;
  bytes.reserve(hex.size() / 2U);
  for (std::size_t index = 0U; index < hex.size(); index += 2U) {
    const int high = value(hex[index]);
    const int low = value(hex[index + 1U]);
    if (high < 0 || low < 0) return {};
    bytes.push_back(static_cast<uint8_t>((high << 4U) | low));
  }
  return bytes;
}

std::string goldenText() {
  const char *path = std::getenv("ROS_MOTION_CONTROL_GOLDEN_VECTORS");
  if (path == nullptr) return {};
  std::ifstream input(path);
  return std::string(std::istreambuf_iterator<char>(input),
                     std::istreambuf_iterator<char>());
}

std::string section(const std::string &text, const std::string &begin,
                    const std::string &end) {
  const std::size_t first = text.find(begin);
  if (first == std::string::npos) return {};
  const std::size_t last = text.find(end, first + begin.size());
  return last == std::string::npos ? std::string{} :
      text.substr(first, last - first);
}

std::string firstHexValue(const std::string &text, const char *field) {
  const std::regex pattern(std::string("\\\"") + field +
      "\\\"\\s*:\\s*\\\"([0-9a-fA-F]+)\\\"");
  std::smatch match;
  return std::regex_search(text, match, pattern) ? match[1].str() : std::string{};
}

std::vector<std::vector<uint8_t>> framesIn(const std::string &text) {
  const std::regex pattern("\\\"frame_hex\\\"\\s*:\\s*\\\"([0-9a-fA-F]+)\\\"");
  std::vector<std::vector<uint8_t>> frames;
  for (std::sregex_iterator it(text.begin(), text.end(), pattern), end;
       it != end; ++it) {
    frames.push_back(fromHex((*it)[1].str()));
  }
  return frames;
}

}  // namespace

TEST(MotionProtocolGoldenVectors, DecodesAndReencodesAllNineValidFrames) {
  const std::string text = goldenText();
  ASSERT_FALSE(text.empty()) << "golden vectors must be mounted read-only";
  const auto key = fromHex(firstHexValue(text, "test_hmac_key_hex"));
  ASSERT_FALSE(key.empty());
  const auto frames = framesIn(section(text, "\"valid_frames\"", "\"invalid_frames\""));
  ASSERT_EQ(frames.size(), 9U);

  smartcar_motion_gateway::StreamParser fragmented_parser;
  for (const auto &frame : frames) {
    const auto decoded = smartcar_motion_gateway::MotionProtocol::decode(
        frame.data(), frame.size(), key);
    ASSERT_TRUE(decoded.accepted()) << decoded.reason;
    EXPECT_EQ(smartcar_motion_gateway::MotionProtocol::encode(decoded.frame, key), frame);

    const auto first = fragmented_parser.push(frame.data(), 3U, key);
    EXPECT_TRUE(first.empty());
    const auto complete = fragmented_parser.push(frame.data() + 3U,
                                                  frame.size() - 3U, key);
    ASSERT_EQ(complete.size(), 1U);
    EXPECT_TRUE(complete.front().accepted()) << complete.front().reason;
  }
}

TEST(MotionProtocolGoldenVectors, RejectsAllSixInvalidFramesAtTheRequiredGate) {
  const std::string text = goldenText();
  ASSERT_FALSE(text.empty()) << "golden vectors must be mounted read-only";
  const auto key = fromHex(firstHexValue(text, "test_hmac_key_hex"));
  ASSERT_FALSE(key.empty());
  const auto frames = framesIn(section(text, "\"invalid_frames\"", "\n}"));
  ASSERT_EQ(frames.size(), 6U);

  EXPECT_EQ(smartcar_motion_gateway::MotionProtocol::decode(
      frames[0].data(), frames[0].size(), key).status,
      smartcar_motion_gateway::DecodeStatus::kCrcError);
  EXPECT_EQ(smartcar_motion_gateway::MotionProtocol::decode(
      frames[1].data(), frames[1].size(), key).status,
      smartcar_motion_gateway::DecodeStatus::kAuthError);

  const auto old_sequence = smartcar_motion_gateway::MotionProtocol::decode(
      frames[2].data(), frames[2].size(), key);
  ASSERT_TRUE(old_sequence.accepted());
  EXPECT_FALSE(smartcar_motion_gateway::sequenceIsNewer(
      old_sequence.frame.sequence, old_sequence.frame.sequence));

  const auto wrong_session = smartcar_motion_gateway::MotionProtocol::decode(
      frames[3].data(), frames[3].size(), key);
  ASSERT_TRUE(wrong_session.accepted());
  EXPECT_NE(wrong_session.frame.session_id, 0x12345678U);

  const auto over_speed = smartcar_motion_gateway::MotionProtocol::decode(
      frames[4].data(), frames[4].size(), key);
  ASSERT_TRUE(over_speed.accepted());
  float linear = 0.0F;
  float angular = 0.0F;
  EXPECT_FALSE(smartcar_motion_gateway::decodeVelocityPayload(
      over_speed.frame.payload, linear, angular));

  const auto expired = smartcar_motion_gateway::MotionProtocol::decode(
      frames[5].data(), frames[5].size(), key);
  ASSERT_TRUE(expired.accepted());
  ASSERT_TRUE(smartcar_motion_gateway::validLeaseTtl(expired.frame.ttl_ms));
  const uint64_t deadline_ns = static_cast<uint64_t>(expired.frame.ttl_ms) * 1000000U;
  EXPECT_LT(deadline_ns, 25000000U);
}

TEST(MotionProtocol, ParserRejectsOversizeLengthAndFindsNextMagic) {
  const std::vector<uint8_t> key{0x01U};
  smartcar_motion_gateway::StreamParser parser;
  std::vector<uint8_t> malformed(25U, 0U);
  malformed[0] = 0x00U;
  malformed[1] = 0x52U;
  malformed[2] = 0x4DU;
  malformed[3] = 0x01U;
  malformed[4] = 0x01U;
  malformed[5] = 0x01U;
  malformed[7] = 0xFFU;
  const auto results = parser.push(malformed.data(), malformed.size(), key);
  ASSERT_FALSE(results.empty());
  EXPECT_NE(std::find_if(results.begin(), results.end(), [](const auto &result) {
    return result.status == smartcar_motion_gateway::DecodeStatus::kLengthError;
  }), results.end());
}
