#include <array>
#include <atomic>
#include <cerrno>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <chrono>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

#include <fcntl.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <dds/dds.hpp>

#include "d1_transport_ipc.hpp"
#include "msg/ArmString_.hpp"
#include "msg/PubServoInfo_.hpp"

namespace
{
constexpr char kCommandTopic[] = "rt/arm_Command";
constexpr char kServoTopic[] = "current_servo_angle";
constexpr char kFeedbackTopic[] = "arm_Feedback";
constexpr std::chrono::milliseconds kLoopSleep{10};

std::atomic_bool g_should_exit{false};

void HandleSignal(int)
{
  g_should_exit.store(true);
}

void RegisterSignalHandlers()
{
  std::signal(SIGINT, HandleSignal);
  std::signal(SIGTERM, HandleSignal);
}

std::string SocketPathFromEnvironment()
{
  const char * env_value = std::getenv("D1_TRANSPORT_SOCKET_PATH");
  if (env_value != nullptr && env_value[0] != '\0') {
    return std::string(env_value);
  }
  return go2w_d1_arm::ipc::kDefaultSocketPath;
}

int DomainIdFromEnvironment()
{
  const char * env_value = std::getenv("ROS_DOMAIN_ID");
  if (env_value == nullptr || env_value[0] == '\0') {
    return 0;
  }

  try {
    return std::stoi(env_value);
  } catch (const std::exception &) {
    throw std::runtime_error("ROS_DOMAIN_ID must be an integer");
  }
}

void SetNonBlocking(int file_descriptor)
{
  const int flags = fcntl(file_descriptor, F_GETFL, 0);
  if (flags < 0) {
    throw std::runtime_error("fcntl(F_GETFL) failed");
  }
  if (fcntl(file_descriptor, F_SETFL, flags | O_NONBLOCK) < 0) {
    throw std::runtime_error("fcntl(F_SETFL) failed");
  }
}

class UnixSocketServer
{
public:
  explicit UnixSocketServer(std::string socket_path)
  : socket_path_(std::move(socket_path))
  {
    listen_fd_ = socket(AF_UNIX, SOCK_STREAM, 0);
    if (listen_fd_ < 0) {
      throw std::runtime_error("failed to create UNIX socket");
    }

    ::unlink(socket_path_.c_str());

    sockaddr_un address{};
    address.sun_family = AF_UNIX;
    if (socket_path_.size() >= sizeof(address.sun_path)) {
      throw std::runtime_error("socket path is too long: " + socket_path_);
    }
    std::strncpy(address.sun_path, socket_path_.c_str(), sizeof(address.sun_path) - 1);

    if (bind(listen_fd_, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0) {
      throw std::runtime_error("failed to bind UNIX socket: " + socket_path_);
    }
    if (listen(listen_fd_, 1) < 0) {
      throw std::runtime_error("failed to listen on UNIX socket");
    }

    SetNonBlocking(listen_fd_);
  }

  ~UnixSocketServer()
  {
    CloseClient();
    if (listen_fd_ >= 0) {
      close(listen_fd_);
    }
    ::unlink(socket_path_.c_str());
  }

  void AcceptClientIfAvailable()
  {
    if (client_fd_ >= 0) {
      return;
    }

    const int accepted = accept(listen_fd_, nullptr, nullptr);
    if (accepted < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        return;
      }
      throw std::runtime_error("accept() failed");
    }

    try {
      SetNonBlocking(accepted);
    } catch (const std::exception &) {
      close(accepted);
      throw;
    }

    client_fd_ = accepted;
    client_buffer_.clear();
    std::cout << "[go2w_d1_arm] transport client connected" << std::endl;
  }

  template<typename Callback>
  void PollCommands(Callback callback)
  {
    if (client_fd_ < 0) {
      return;
    }

    char buffer[4096];
    while (true) {
      const ssize_t received = recv(client_fd_, buffer, sizeof(buffer), 0);
      if (received > 0) {
        client_buffer_.append(buffer, static_cast<std::size_t>(received));
        DrainBufferedLines(callback);
        continue;
      }

      if (received == 0) {
        std::cerr << "[go2w_d1_arm] transport client disconnected" << std::endl;
        CloseClient();
        return;
      }

      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        return;
      }
      if (errno == EINTR) {
        continue;
      }

      std::cerr << "[go2w_d1_arm] transport recv failed: " << std::strerror(errno) << std::endl;
      CloseClient();
      return;
    }
  }

  void SendLine(const std::string & line)
  {
    if (client_fd_ < 0) {
      return;
    }

    const std::string message = line + "\n";
    std::size_t sent = 0;
    while (sent < message.size()) {
      const ssize_t result =
        send(client_fd_, message.data() + sent, message.size() - sent, MSG_NOSIGNAL);
      if (result > 0) {
        sent += static_cast<std::size_t>(result);
        continue;
      }

      if (result < 0 && errno == EINTR) {
        continue;
      }

      if (result < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
        std::cerr << "[go2w_d1_arm] transport client is not draining socket data" << std::endl;
      } else {
        std::cerr << "[go2w_d1_arm] transport send failed: " << std::strerror(errno) << std::endl;
      }
      CloseClient();
      return;
    }
  }

private:
  template<typename Callback>
  void DrainBufferedLines(Callback callback)
  {
    std::size_t newline_position = client_buffer_.find('\n');
    while (newline_position != std::string::npos) {
      std::string line = client_buffer_.substr(0, newline_position);
      if (!line.empty() && line.back() == '\r') {
        line.pop_back();
      }
      client_buffer_.erase(0, newline_position + 1);

      if (!line.empty()) {
        callback(line);
      }
      newline_position = client_buffer_.find('\n');
    }
  }

  void CloseClient()
  {
    if (client_fd_ >= 0) {
      close(client_fd_);
      client_fd_ = -1;
    }
    client_buffer_.clear();
  }

  std::string socket_path_;
  int listen_fd_ = -1;
  int client_fd_ = -1;
  std::string client_buffer_;
};

class D1ArmTransport
{
public:
  D1ArmTransport(int domain_id, std::string socket_path)
  : participant_(static_cast<uint32_t>(domain_id)),
    publisher_(participant_),
    subscriber_(participant_),
    command_topic_(participant_, kCommandTopic),
    servo_topic_(participant_, kServoTopic),
    feedback_topic_(participant_, kFeedbackTopic),
    command_writer_(publisher_, command_topic_),
    servo_reader_(subscriber_, servo_topic_),
    feedback_reader_(subscriber_, feedback_topic_),
    socket_server_(std::move(socket_path))
  {
  }

  void Run()
  {
    while (!g_should_exit.load()) {
      socket_server_.AcceptClientIfAvailable();
      socket_server_.PollCommands(
        [this](const std::string & line) {
          HandleCommandLine(line);
        });
      PumpServoReader();
      PumpFeedbackReader();
      std::this_thread::sleep_for(kLoopSleep);
    }
  }

private:
  void HandleCommandLine(const std::string & line)
  {
    std::string payload;
    if (!go2w_d1_arm::ipc::ParseCommandLine(line, &payload)) {
      std::cerr << "[go2w_d1_arm] ignoring malformed transport command line: " << line << std::endl;
      return;
    }

    unitree_arm::msg::dds_::ArmString_ message;
    message.data_() = payload;
    command_writer_.write(message);
  }

  void PumpServoReader()
  {
    dds::sub::LoanedSamples<unitree_arm::msg::dds_::PubServoInfo_> samples = servo_reader_.take();
    if (samples.length() <= 0) {
      return;
    }

    for (auto iter = samples.begin(); iter < samples.end(); ++iter) {
      if (!iter->info().valid()) {
        continue;
      }

      const auto & message = iter->data();
      const std::array<double, go2w_d1_arm::ipc::kJointCount> positions_deg = {
        static_cast<double>(message.servo0_data_()),
        static_cast<double>(message.servo1_data_()),
        static_cast<double>(message.servo2_data_()),
        static_cast<double>(message.servo3_data_()),
        static_cast<double>(message.servo4_data_()),
        static_cast<double>(message.servo5_data_()),
        static_cast<double>(message.servo6_data_()),
      };
      socket_server_.SendLine(go2w_d1_arm::ipc::BuildServoLine(positions_deg));
    }
  }

  void PumpFeedbackReader()
  {
    dds::sub::LoanedSamples<unitree_arm::msg::dds_::ArmString_> samples = feedback_reader_.take();
    if (samples.length() <= 0) {
      return;
    }

    for (auto iter = samples.begin(); iter < samples.end(); ++iter) {
      if (!iter->info().valid()) {
        continue;
      }

      socket_server_.SendLine(go2w_d1_arm::ipc::BuildFeedbackLine(iter->data().data_()));
    }
  }

  dds::domain::DomainParticipant participant_;
  dds::pub::Publisher publisher_;
  dds::sub::Subscriber subscriber_;
  dds::topic::Topic<unitree_arm::msg::dds_::ArmString_> command_topic_;
  dds::topic::Topic<unitree_arm::msg::dds_::PubServoInfo_> servo_topic_;
  dds::topic::Topic<unitree_arm::msg::dds_::ArmString_> feedback_topic_;
  dds::pub::DataWriter<unitree_arm::msg::dds_::ArmString_> command_writer_;
  dds::sub::DataReader<unitree_arm::msg::dds_::PubServoInfo_> servo_reader_;
  dds::sub::DataReader<unitree_arm::msg::dds_::ArmString_> feedback_reader_;
  UnixSocketServer socket_server_;
};
}  // namespace

int main()
{
  RegisterSignalHandlers();

  try {
    const int domain_id = DomainIdFromEnvironment();
    const std::string socket_path = SocketPathFromEnvironment();

    std::cout << "[go2w_d1_arm] starting raw D1 transport" << std::endl;
    std::cout << "[go2w_d1_arm] transport domain id=" << domain_id << std::endl;
    std::cout << "[go2w_d1_arm] transport socket=" << socket_path << std::endl;
    std::cout << "[go2w_d1_arm] transport interface=" <<
      (std::getenv("UNITREE_NETWORK_INTERFACE") != nullptr ? std::getenv("UNITREE_NETWORK_INTERFACE") : "<unset>")
              << std::endl;

    D1ArmTransport transport(domain_id, socket_path);
    transport.Run();
    return 0;
  } catch (const std::exception & ex) {
    std::cerr << "[go2w_d1_arm] transport fatal: " << ex.what() << std::endl;
    return 1;
  }
}
