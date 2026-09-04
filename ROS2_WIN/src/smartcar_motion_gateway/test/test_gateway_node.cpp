#include "smartcar_motion_gateway/gateway_node.hpp"

#include "chassis_test_utils.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

#include <gtest/gtest.h>

#include <arpa/inet.h>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <future>
#include <functional>
#include <memory>
#include <netinet/in.h>
#include <poll.h>
#include <string>
#include <sys/socket.h>
#include <thread>
#include <tuple>
#include <unistd.h>
#include <vector>

namespace {

constexpr uint16_t kTestPort = 18766U;
constexpr uint32_t kSessionId = 0x12345678U;
constexpr char kTestKeyText[] = "00112233445566778899aabbccddeeff"
                                "102132435465768798a9bacbdcedfe0f";
static_assert(sizeof(kTestKeyText) - 1U == 64U);
const std::vector<uint8_t> kTestKey(
    kTestKeyText, kTestKeyText + sizeof(kTestKeyText) - 1U);
const std::vector<uint8_t> kHexDecodedTestKey{
    0x00U, 0x11U, 0x22U, 0x33U, 0x44U, 0x55U, 0x66U, 0x77U,
    0x88U, 0x99U, 0xAAU, 0xBBU, 0xCCU, 0xDDU, 0xEEU, 0xFFU,
    0x10U, 0x21U, 0x32U, 0x43U, 0x54U, 0x65U, 0x76U, 0x87U,
    0x98U, 0xA9U, 0xBAU, 0xCBU, 0xDCU, 0xEDU, 0xFEU, 0x0FU,
};

std::string keyPath() {
  return "/tmp/smartcar_motion_gateway_test_" + std::to_string(getpid()) + ".psk";
}

void writeTestKey(const std::string &contents = std::string(kTestKeyText) + '\n') {
  std::ofstream output(keyPath(), std::ios::out | std::ios::trunc | std::ios::binary);
  ASSERT_TRUE(output.is_open());
  output.write(contents.data(), static_cast<std::streamsize>(contents.size()));
  ASSERT_TRUE(output.good());
}

bool sendAll(int fd, const std::vector<uint8_t> &bytes) {
  std::size_t offset = 0U;
  while (offset < bytes.size()) {
    const ssize_t sent = send(fd, bytes.data() + offset, bytes.size() - offset, MSG_NOSIGNAL);
    if (sent <= 0) return false;
    offset += static_cast<std::size_t>(sent);
  }
  return true;
}

int connectClient() {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (std::chrono::steady_clock::now() < deadline) {
    const int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) return -1;
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(kTestPort);
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (connect(fd, reinterpret_cast<const sockaddr *>(&address), sizeof(address)) == 0) {
      return fd;
    }
    close(fd);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  return -1;
}

std::vector<smartcar_motion_gateway::MotionFrame> receiveFrames(int fd,
                                                                  std::size_t expected) {
  smartcar_motion_gateway::StreamParser parser;
  std::vector<smartcar_motion_gateway::MotionFrame> frames;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (frames.size() < expected && std::chrono::steady_clock::now() < deadline) {
    pollfd descriptor{};
    descriptor.fd = fd;
    descriptor.events = POLLIN;
    if (poll(&descriptor, 1, 100) <= 0) continue;
    std::array<uint8_t, smartcar_motion_gateway::kMotionMaxFrameBytes> buffer{};
    const ssize_t received = recv(fd, buffer.data(), buffer.size(), 0);
    if (received <= 0) break;
    for (const auto &result : parser.push(buffer.data(), static_cast<std::size_t>(received),
                                          kTestKey)) {
      EXPECT_TRUE(result.accepted()) << result.reason;
      if (result.accepted()) frames.push_back(result.frame);
    }
  }
  return frames;
}

bool waitForPeerClose(int fd) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  std::array<uint8_t, 256U> buffer{};
  while (std::chrono::steady_clock::now() < deadline) {
    pollfd descriptor{};
    descriptor.fd = fd;
    descriptor.events = POLLIN;
    if (poll(&descriptor, 1, 100) <= 0) continue;
    if (recv(fd, buffer.data(), buffer.size(), 0) == 0) return true;
  }
  return false;
}

bool peerStaysConnectedWithoutFrames(int fd, std::chrono::milliseconds duration) {
  pollfd descriptor{};
  descriptor.fd = fd;
  descriptor.events = POLLIN;
  const int ready = poll(&descriptor, 1, static_cast<int>(duration.count()));
  return ready == 0 && (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) == 0;
}

bool hasFrameType(const std::vector<smartcar_motion_gateway::MotionFrame> &frames,
                  smartcar_motion_gateway::MessageType type) {
  for (const auto &frame : frames) {
    if (frame.type == type) return true;
  }
  return false;
}

smartcar_motion_gateway::MotionFrame hello(uint32_t session = kSessionId) {
  smartcar_motion_gateway::MotionFrame frame;
  frame.type = smartcar_motion_gateway::MessageType::kHello;
  frame.session_id = session;
  frame.sequence = 1U;
  return frame;
}

smartcar_motion_gateway::MotionFrame leaseResponse(uint32_t lease_id,
                                                    uint32_t sequence = 2U,
                                                    uint16_t ttl_ms = smartcar_motion_gateway::kMaxLeaseTtlMs) {
  smartcar_motion_gateway::MotionFrame frame;
  frame.type = smartcar_motion_gateway::MessageType::kLeaseResponse;
  frame.session_id = kSessionId;
  frame.sequence = sequence;
  frame.lease_id = lease_id;
  frame.ttl_ms = ttl_ms;
  frame.payload = {0U, static_cast<uint8_t>(ttl_ms & 0xFFU),
                   static_cast<uint8_t>(ttl_ms >> 8U)};
  return frame;
}

class GatewayHarness final {
 public:
  explicit GatewayHarness(const std::string &key_file = std::string(kTestKeyText) + '\n',
                          bool enable_motion = false) {
    writeTestKey(key_file);
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        rclcpp::Parameter("psk_config_path", keyPath()),
        rclcpp::Parameter("listen_port", static_cast<int64_t>(kTestPort)),
        rclcpp::Parameter("enable_motion", enable_motion),
        rclcpp::Parameter("protocol_ready", true),
        rclcpp::Parameter("enable_status_odom", true),
        rclcpp::Parameter("publish_status_tf", true),
        rclcpp::Parameter("safe_cmd_topic", "/motion_gateway_test_cmd"),
    });
    gateway_ = std::make_shared<smartcar_motion_gateway::GatewayNode>(options);
    observer_ = std::make_shared<rclcpp::Node>("motion_gateway_test_observer");
    odom_subscription_ = observer_->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", rclcpp::QoS(10), [this](nav_msgs::msg::Odometry::SharedPtr) {
          ++odom_count_;
        });
    tf_subscription_ = observer_->create_subscription<tf2_msgs::msg::TFMessage>(
        "/tf", rclcpp::QoS(10), [this](tf2_msgs::msg::TFMessage::SharedPtr message) {
          for (const auto &transform : message->transforms) {
            if (transform.header.frame_id == "odom" &&
                transform.child_frame_id == "base_link") {
              ++status_tf_count_;
              return;
            }
          }
        });
    safe_command_subscription_ = observer_->create_subscription<geometry_msgs::msg::Twist>(
        "/motion_gateway_test_cmd", rclcpp::QoS(10), [this](geometry_msgs::msg::Twist::SharedPtr message) {
          if (message->linear.x != 0.0 || message->angular.z != 0.0) ++nonzero_command_count_;
        });
    executor_.add_node(gateway_);
    executor_.add_node(observer_);
    spin_thread_ = std::thread([this] { executor_.spin(); });
  }

  ~GatewayHarness() {
    executor_.cancel();
    if (spin_thread_.joinable()) spin_thread_.join();
    executor_.remove_node(observer_);
    executor_.remove_node(gateway_);
    gateway_.reset();
    observer_.reset();
    (void)unlink(keyPath().c_str());
  }

  int connect() const { return connectClient(); }

  std::vector<smartcar_motion_gateway::MotionFrame> handshake(
      int fd, std::size_t expected = 1U) const {
    EXPECT_TRUE(sendAll(fd, smartcar_motion_gateway::MotionProtocol::encode(hello(), kTestKey)));
    return receiveFrames(fd, expected);
  }

  bool setMotionEnabled(bool enabled) {
    auto client = observer_->create_client<std_srvs::srv::SetBool>(
        "/smartcar_motion_gateway/set_motion_enabled");
    if (!client->wait_for_service(std::chrono::seconds(2))) return false;
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = enabled;
    auto result = client->async_send_request(request);
    if (result.wait_for(std::chrono::seconds(2)) != std::future_status::ready) return false;
    return result.get()->success;
  }

  bool stopMotion() {
    auto client = observer_->create_client<std_srvs::srv::Trigger>(
        "/smartcar_motion_gateway/stop");
    if (!client->wait_for_service(std::chrono::seconds(2))) return false;
    auto result = client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    if (result.wait_for(std::chrono::seconds(2)) != std::future_status::ready) return false;
    return result.get()->success;
  }

  bool waitForOdom() const {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (std::chrono::steady_clock::now() < deadline) {
      if (odom_count_.load() > 0U) return true;
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return false;
  }

  bool waitForStatusTf() const {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (std::chrono::steady_clock::now() < deadline) {
      if (status_tf_count_.load() > 0U) return true;
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return false;
  }

  uint64_t nonzeroCommandCount() const { return nonzero_command_count_.load(); }

 private:
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::shared_ptr<smartcar_motion_gateway::GatewayNode> gateway_;
  std::shared_ptr<rclcpp::Node> observer_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr safe_command_subscription_;
  std::thread spin_thread_;
  std::atomic<uint64_t> odom_count_{0U};
  std::atomic<uint64_t> status_tf_count_{0U};
  std::atomic<uint64_t> nonzero_command_count_{0U};
};

void closeClient(int fd) {
  if (fd >= 0) {
    shutdown(fd, SHUT_RDWR);
    close(fd);
  }
}

}  // namespace

class GatewayNodeTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    int argc = 0;
    char **argv = nullptr;
    rclcpp::init(argc, argv);
  }

  static void TearDownTestSuite() { rclcpp::shutdown(); }
};

TEST_F(GatewayNodeTest, TelemetryOnlyHelloAcknowledgesWithoutLeaseRequest) {
  GatewayHarness harness;
  const int fd = harness.connect();
  ASSERT_GE(fd, 0);
  const auto frames = harness.handshake(fd);
  ASSERT_EQ(frames.size(), 1U);
  EXPECT_EQ(frames[0].type, smartcar_motion_gateway::MessageType::kHelloAck);
  EXPECT_EQ(frames[0].session_id, kSessionId);
  EXPECT_NE(frames[0].sequence, 0U);
  EXPECT_EQ(frames[0].lease_id, 0U);
  EXPECT_EQ(frames[0].ttl_ms, 0U);
  EXPECT_TRUE(frames[0].payload.empty());
  EXPECT_TRUE(peerStaysConnectedWithoutFrames(
      fd, std::chrono::milliseconds(smartcar_motion_gateway::kMaxLeaseTtlMs + 100U)));
  closeClient(fd);
}

TEST_F(GatewayNodeTest, MotionAuthorizedHelloRequestsNonzeroLease) {
  GatewayHarness harness(std::string(kTestKeyText) + '\n', true);
  const int fd = harness.connect();
  ASSERT_GE(fd, 0);
  const auto frames = harness.handshake(fd, 2U);
  ASSERT_EQ(frames.size(), 2U);
  EXPECT_EQ(frames[0].type, smartcar_motion_gateway::MessageType::kHelloAck);
  EXPECT_EQ(frames[0].lease_id, 0U);
  EXPECT_EQ(frames[1].type, smartcar_motion_gateway::MessageType::kLeaseRequest);
  EXPECT_EQ(frames[1].session_id, kSessionId);
  EXPECT_NE(frames[1].sequence, 0U);
  EXPECT_NE(frames[1].lease_id, 0U);
  EXPECT_GT(frames[1].sequence, frames[0].sequence);
  closeClient(fd);
}

TEST_F(GatewayNodeTest, RawAsciiPskUsesAllSixtyFourTextBytesForHmac) {
  ASSERT_EQ(kTestKey.size(), 64U);
  ASSERT_EQ(kHexDecodedTestKey.size(), 32U);

  GatewayHarness harness;
  const int fd = harness.connect();
  ASSERT_GE(fd, 0);
  const auto frames = harness.handshake(fd);
  ASSERT_EQ(frames.size(), 1U);

  const auto hello_frame = smartcar_motion_gateway::MotionProtocol::encode(hello(), kTestKey);
  ASSERT_FALSE(hello_frame.empty());
  EXPECT_TRUE(smartcar_motion_gateway::MotionProtocol::decode(
      hello_frame.data(), hello_frame.size(), kTestKey).accepted());
  EXPECT_EQ(smartcar_motion_gateway::MotionProtocol::decode(
      hello_frame.data(), hello_frame.size(), kHexDecodedTestKey).status,
      smartcar_motion_gateway::DecodeStatus::kAuthError);
  closeClient(fd);
}

TEST_F(GatewayNodeTest, PskFinalLfAndCrlfLoadTheSameRawAsciiKey) {
  {
    GatewayHarness harness(std::string(kTestKeyText) + '\n');
    const int fd = harness.connect();
    ASSERT_GE(fd, 0);
    EXPECT_EQ(harness.handshake(fd).size(), 1U);
    closeClient(fd);
  }
  {
    GatewayHarness harness(std::string(kTestKeyText) + "\r\n");
    const int fd = harness.connect();
    ASSERT_GE(fd, 0);
    EXPECT_EQ(harness.handshake(fd).size(), 1U);
    closeClient(fd);
  }
}

TEST(GatewayPskText, RejectsWhitespaceExtraLinesNulAndNonAscii) {
  const std::vector<std::string> invalid_files{
      "",
      " " + std::string(kTestKeyText),
      std::string(kTestKeyText) + " ",
      "\t" + std::string(kTestKeyText),
      std::string(kTestKeyText) + "\t",
      std::string(kTestKeyText) + "\nsecond-line",
      std::string(kTestKeyText) + "\n\n",
      std::string(kTestKeyText) + std::string(1U, '\0'),
      std::string(kTestKeyText) + std::string(1U, static_cast<char>(0x80U)),
  };

  for (const auto &contents : invalid_files) {
    std::vector<uint8_t> parsed{0xFFU};
    EXPECT_FALSE(smartcar_motion_gateway::parseRawAsciiPsk(contents, parsed));
    EXPECT_TRUE(parsed.empty());
  }
}

TEST_F(GatewayNodeTest, StatusChassisTelemetryIsAcceptedBeforeLeaseAndPublishesOdom) {
  GatewayHarness harness;
  const int fd = harness.connect();
  ASSERT_GE(fd, 0);
  ASSERT_EQ(harness.handshake(fd).size(), 1U);
  ASSERT_TRUE(peerStaysConnectedWithoutFrames(
      fd, std::chrono::milliseconds(smartcar_motion_gateway::kMaxLeaseTtlMs + 100U)));

  for (const auto &[sequence, timestamp_ms, x_mm, chassis_sequence] :
       {std::tuple<uint32_t, uint32_t, float, uint8_t>{2U, 1000U, 0.0F, 1U},
        std::tuple<uint32_t, uint32_t, float, uint8_t>{3U, 1100U, 100.0F, 2U}}) {
    auto payload = smartcar_state_bridge::test::makeFrame(
        timestamp_ms, x_mm, 0.0F, 0.0F, 0.1F, 0x04U, chassis_sequence);
    payload.insert(payload.begin(), smartcar_state_bridge::kSrpV4ChassisMessageId);
    smartcar_motion_gateway::MotionFrame status;
    status.type = smartcar_motion_gateway::MessageType::kStatus;
    status.session_id = kSessionId;
    status.sequence = sequence;
    status.lease_id = 0U;
    status.payload = std::move(payload);
    ASSERT_TRUE(sendAll(fd, smartcar_motion_gateway::MotionProtocol::encode(status, kTestKey)));
  }
  EXPECT_TRUE(harness.waitForOdom());
  EXPECT_TRUE(harness.waitForStatusTf());
  closeClient(fd);
}

TEST_F(GatewayNodeTest, TelemetryOnlyMotionCommandClosesAndNeverPublishesNonzeroCommand) {
  GatewayHarness harness;
  const int fd = harness.connect();
  ASSERT_GE(fd, 0);
  ASSERT_EQ(harness.handshake(fd).size(), 1U);
  smartcar_motion_gateway::MotionFrame command;
  command.type = smartcar_motion_gateway::MessageType::kMotionCommand;
  command.session_id = kSessionId;
  command.sequence = 2U;
  command.ttl_ms = smartcar_motion_gateway::kMaxLeaseTtlMs;
  command.payload = smartcar_motion_gateway::encodeVelocityPayload(0.01F, 0.0F);
  ASSERT_TRUE(sendAll(fd, smartcar_motion_gateway::MotionProtocol::encode(command, kTestKey)));
  EXPECT_TRUE(waitForPeerClose(fd));
  EXPECT_EQ(harness.nonzeroCommandCount(), 0U);
  closeClient(fd);
}

TEST_F(GatewayNodeTest, RuntimeArmingRequestsLeaseAndRequiresAValidResponse) {
  GatewayHarness harness;
  const int fd = harness.connect();
  ASSERT_GE(fd, 0);
  ASSERT_EQ(harness.handshake(fd).size(), 1U);
  EXPECT_TRUE(peerStaysConnectedWithoutFrames(fd, std::chrono::milliseconds(100)));

  ASSERT_TRUE(harness.setMotionEnabled(true));
  const auto lease_frames = receiveFrames(fd, 1U);
  ASSERT_EQ(lease_frames.size(), 1U);
  ASSERT_EQ(lease_frames[0].type, smartcar_motion_gateway::MessageType::kLeaseRequest);
  ASSERT_NE(lease_frames[0].lease_id, 0U);
  EXPECT_TRUE(peerStaysConnectedWithoutFrames(fd, std::chrono::milliseconds(100)));

  const auto response = leaseResponse(lease_frames[0].lease_id);
  ASSERT_TRUE(sendAll(fd, smartcar_motion_gateway::MotionProtocol::encode(response, kTestKey)));
  const auto outbound = receiveFrames(fd, 1U);
  ASSERT_EQ(outbound.size(), 1U);
  EXPECT_EQ(outbound[0].type, smartcar_motion_gateway::MessageType::kHeartbeat);
  EXPECT_TRUE(harness.stopMotion());
  const auto after_stop = receiveFrames(fd, 2U);
  EXPECT_TRUE(hasFrameType(after_stop, smartcar_motion_gateway::MessageType::kStop));
  EXPECT_EQ(harness.nonzeroCommandCount(), 0U);
  closeClient(fd);
}

TEST_F(GatewayNodeTest, MotionHeartbeatRefreshesTheLeaseBeforeItsTtlExpires) {
  GatewayHarness harness(std::string(kTestKeyText) + '\n', true);
  const int fd = harness.connect();
  ASSERT_GE(fd, 0);
  const auto frames = harness.handshake(fd, 2U);
  ASSERT_EQ(frames.size(), 2U);
  const auto response = leaseResponse(frames[1].lease_id);
  ASSERT_TRUE(sendAll(fd, smartcar_motion_gateway::MotionProtocol::encode(response, kTestKey)));
  const auto heartbeats = receiveFrames(fd, 6U);
  ASSERT_EQ(heartbeats.size(), 6U);
  for (const auto &heartbeat : heartbeats) {
    EXPECT_EQ(heartbeat.type, smartcar_motion_gateway::MessageType::kHeartbeat);
    EXPECT_EQ(heartbeat.lease_id, frames[1].lease_id);
  }
  EXPECT_EQ(harness.nonzeroCommandCount(), 0U);
  closeClient(fd);
}

TEST_F(GatewayNodeTest, InvalidAuthCrcSessionAndSequenceFramesCloseTheConnection) {
  GatewayHarness harness;
  const auto expect_close = [&harness](std::function<void(int)> send_frame) {
    const int fd = harness.connect();
    EXPECT_GE(fd, 0);
    if (fd < 0) return;
    send_frame(fd);
    EXPECT_TRUE(waitForPeerClose(fd));
    closeClient(fd);
  };

  expect_close([](int fd) {
    auto frame = smartcar_motion_gateway::MotionProtocol::encode(hello(), kTestKey);
    frame[smartcar_motion_gateway::kMotionHeaderBytes] ^= 0x01U;
    EXPECT_TRUE(sendAll(fd, frame));
  });
  expect_close([](int fd) {
    auto frame = smartcar_motion_gateway::MotionProtocol::encode(hello(), kTestKey);
    frame.back() ^= 0x01U;
    EXPECT_TRUE(sendAll(fd, frame));
  });
  expect_close([&harness](int fd) {
    ASSERT_EQ(harness.handshake(fd).size(), 1U);
    auto status = hello(kSessionId + 1U);
    status.type = smartcar_motion_gateway::MessageType::kStatus;
    status.sequence = 2U;
    status.payload = {0U};
    EXPECT_TRUE(sendAll(fd, smartcar_motion_gateway::MotionProtocol::encode(status, kTestKey)));
  });
  expect_close([&harness](int fd) {
    ASSERT_EQ(harness.handshake(fd).size(), 1U);
    auto status = hello();
    status.type = smartcar_motion_gateway::MessageType::kStatus;
    status.sequence = 1U;
    status.payload = {0U};
    EXPECT_TRUE(sendAll(fd, smartcar_motion_gateway::MotionProtocol::encode(status, kTestKey)));
  });
}
