#include "s3_ydlidar_bridge/transport.hpp"

#include <array>
#include <chrono>
#include <cerrno>
#include <cstring>
#include <fstream>
#include <iterator>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <utility>

namespace s3_ydlidar_bridge {

bool ReplayTransport::start(FrameCallback callback, std::string &error) {
  if (!callback || path_.empty()) {
    error = "replay_file is empty";
    return false;
  }

  std::ifstream input(path_, std::ios::binary);
  if (!input) {
    error = "cannot open replay_file: " + path_;
    return false;
  }
  std::vector<uint8_t> payload((std::istreambuf_iterator<char>(input)),
                               std::istreambuf_iterator<char>());
  if (payload.empty()) {
    error = "replay_file is empty: " + path_;
    return false;
  }
  if (!stopped_) {
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    const auto ns = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(now).count());
    ReceivedFrame frame;
    frame.payload = std::move(payload);
    frame.sequence = 1U;
    frame.received_steady_ns = ns;
    callback(std::move(frame));
    ++emitted_;
  }
  return true;
}

TcpServerTransport::TcpServerTransport(TcpServerConfig config)
    : config_(std::move(config)),
      extractor_(std::make_shared<S3FrameExtractor>(config_.protocol)) {}

TcpServerTransport::~TcpServerTransport() { stop(); }

void TcpServerTransport::setConnectionCallback(ConnectionCallback callback) {
  connection_callback_ = std::move(callback);
}

bool TcpServerTransport::start(FrameCallback callback, std::string &error) {
  if (!callback) {
    error = "TCP callback is empty";
    return false;
  }
  if (config_.listen_port == 0U || config_.max_buffer_bytes == 0U) {
    error = "TCP port and max_buffer_bytes must be non-zero";
    return false;
  }
  if (running_.exchange(true)) {
    error = "TCP transport is already running";
    return false;
  }
  callback_ = std::move(callback);
  const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
  if (fd < 0) {
    running_ = false;
    error = std::string("socket: ") + std::strerror(errno);
    return false;
  }
  int reuse = 1;
  (void)::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_port = htons(config_.listen_port);
  if (::inet_pton(AF_INET, config_.listen_address.c_str(), &address.sin_addr) !=
      1) {
    ::close(fd);
    running_ = false;
    error = "listen_address must be an IPv4 address";
    return false;
  }
  if (::bind(fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0) {
    error = std::string("bind ") + config_.listen_address + ":" +
            std::to_string(config_.listen_port) + ": " + std::strerror(errno);
    ::close(fd);
    running_ = false;
    return false;
  }
  if (::listen(fd, 1) < 0) {
    error = std::string("listen: ") + std::strerror(errno);
    ::close(fd);
    running_ = false;
    return false;
  }
  {
    std::lock_guard<std::mutex> lock(socket_mutex_);
    server_fd_ = fd;
  }
  setState("listening");
  thread_ = std::thread(&TcpServerTransport::run, this);
  return true;
}

void TcpServerTransport::stop() noexcept {
  if (!running_.exchange(false)) {
    return;
  }
  closeSockets();
  if (thread_.joinable()) {
    thread_.join();
  }
  setState("stopped");
}

void TcpServerTransport::closeSockets() noexcept {
  std::lock_guard<std::mutex> lock(socket_mutex_);
  if (client_fd_ >= 0) {
    (void)::shutdown(client_fd_, SHUT_RDWR);
    (void)::close(client_fd_);
    client_fd_ = -1;
  }
  if (server_fd_ >= 0) {
    (void)::shutdown(server_fd_, SHUT_RDWR);
    (void)::close(server_fd_);
    server_fd_ = -1;
  }
}

void TcpServerTransport::setState(const std::string &state) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  connection_state_ = state;
}

void TcpServerTransport::run() {
  std::array<uint8_t, 8192> receive_buffer{};
  while (running_) {
    int server_fd = -1;
    {
      std::lock_guard<std::mutex> lock(socket_mutex_);
      server_fd = server_fd_;
    }
    if (server_fd < 0) {
      break;
    }
    const int client = ::accept(server_fd, nullptr, nullptr);
    if (client < 0) {
      if (running_) {
        setState("listening");
        continue;
      }
      break;
    }
    {
      std::lock_guard<std::mutex> lock(socket_mutex_);
      client_fd_ = client;
    }
    const uint64_t connection_epoch = connection_epoch_.fetch_add(1U) + 1U;
    ++accepted_connections_;
    setState("connected");
    if (connection_callback_) {
      connection_callback_(
          ConnectionEvent{ConnectionEventType::kOpened, connection_epoch});
    }
    TcpChunkAssembler assembler(extractor_, config_.max_buffer_bytes,
                                config_.max_ready_frames);
    while (running_) {
      const ssize_t count = ::recv(client, receive_buffer.data(),
                                   receive_buffer.size(), 0);
      if (count <= 0) {
        break;
      }
      recv_bytes_ += static_cast<uint64_t>(count);
      (void)assembler.feed(receive_buffer.data(),
                           static_cast<size_t>(count));
      for (auto frame : assembler.takeAll()) {
        if (!running_) {
          break;
        }
        frame.connection_epoch = connection_epoch;
        callback_(std::move(frame));
      }
    }
    {
      std::lock_guard<std::mutex> lock(socket_mutex_);
      if (client_fd_ == client) {
        (void)::shutdown(client_fd_, SHUT_RDWR);
        (void)::close(client_fd_);
        client_fd_ = -1;
      }
    }
    ++disconnects_;
    if (connection_callback_) {
      connection_callback_(
          ConnectionEvent{ConnectionEventType::kClosed, connection_epoch});
    }
    if (running_) {
      setState("listening");
    }
  }
}

TransportStats TcpServerTransport::stats() const {
  TransportStats result;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    result.connection_state = connection_state_;
  }
  result.connection_epoch = connection_epoch_.load();
  result.accepted_connections = accepted_connections_.load();
  result.disconnects = disconnects_.load();
  result.recv_bytes = recv_bytes_.load();
  if (extractor_) {
    result.protocol = extractor_->counters();
  }
  return result;
}

}  // namespace s3_ydlidar_bridge
