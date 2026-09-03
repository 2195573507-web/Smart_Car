#pragma once

#include "s3_ydlidar_bridge/framing.hpp"

#include <functional>
#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>

namespace s3_ydlidar_bridge {

using FrameCallback = std::function<void(ReceivedFrame)>;

enum class ConnectionEventType {
  kOpened,
  kClosed,
};

struct ConnectionEvent {
  ConnectionEventType type{ConnectionEventType::kClosed};
  uint64_t connection_epoch{0};
};

using ConnectionCallback = std::function<void(ConnectionEvent)>;

struct TransportStats {
  std::string connection_state{"stopped"};
  uint64_t connection_epoch{0};
  uint64_t accepted_connections{0};
  uint64_t disconnects{0};
  uint64_t recv_bytes{0};
  // Frames parsed successfully but evicted because the bounded ready queue
  // was full.  `overflow` is the short diagnostic key used by the node;
  // descriptive aliases are kept in the API for callers that prefer them.
  uint64_t dropped_ready{0};
  uint64_t overflow{0};
  uint64_t dropped_ready_frames{0};
  uint64_t ready_queue_overflows{0};
  S3ProtocolCounters protocol;
};

class FrameTransport {
 public:
  virtual ~FrameTransport() = default;
  virtual bool start(FrameCallback callback, std::string &error) = 0;
  virtual void stop() noexcept = 0;
  virtual TransportStats stats() const = 0;
  virtual void setConnectionCallback(ConnectionCallback) {}
};

class UnconfiguredTransport final : public FrameTransport {
 public:
  bool start(FrameCallback, std::string &error) override {
    error = "S3 gateway protocol is not frozen; live transport is disabled";
    return false;
  }
  void stop() noexcept override {}
  TransportStats stats() const override {
    TransportStats result;
    result.connection_state = "unconfigured";
    return result;
  }
};

class ReplayTransport final : public FrameTransport {
 public:
  explicit ReplayTransport(std::string path) : path_(std::move(path)) {}

  bool start(FrameCallback callback, std::string &error) override;
  void stop() noexcept override { stopped_ = true; }
  TransportStats stats() const override {
    TransportStats result;
    result.connection_state = stopped_ ? "stopped" : "replay";
    result.protocol.accepted_frames = emitted_;
    return result;
  }

 private:
  std::string path_;
  bool stopped_{false};
  uint64_t emitted_{0};
};

struct TcpServerConfig {
  std::string listen_address{"0.0.0.0"};
  uint16_t listen_port{8765};
  size_t max_buffer_bytes{256U * 1024U};
  size_t max_ready_frames{64U};
  S3ProtocolConfig protocol;
};

class TcpServerTransport final : public FrameTransport {
 public:
  explicit TcpServerTransport(TcpServerConfig config);
  ~TcpServerTransport() override;

  bool start(FrameCallback callback, std::string &error) override;
  void stop() noexcept override;
  TransportStats stats() const override;
  void setConnectionCallback(ConnectionCallback callback) override;

 private:
  void run();
  void closeSockets() noexcept;
  void setState(const std::string &state);

  TcpServerConfig config_;
  FrameCallback callback_;
  ConnectionCallback connection_callback_;
  std::shared_ptr<S3FrameExtractor> extractor_;
  std::atomic<bool> running_{false};
  std::thread thread_;
  mutable std::mutex socket_mutex_;
  mutable std::mutex state_mutex_;
  int server_fd_{-1};
  int client_fd_{-1};
  std::string connection_state_{"stopped"};
  std::atomic<uint64_t> accepted_connections_{0};
  std::atomic<uint64_t> disconnects_{0};
  std::atomic<uint64_t> recv_bytes_{0};
  std::atomic<uint64_t> connection_epoch_{0};
  std::atomic<uint64_t> dropped_ready_{0};
  std::atomic<uint64_t> ready_overflow_{0};
};

}  // namespace s3_ydlidar_bridge
