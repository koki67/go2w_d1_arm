#include <algorithm>
#include <array>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "d1_transport_ipc.hpp"
#include "go2w_d1_arm/msg/d1_arm_status.hpp"
#include "go2w_d1_arm/srv/single_joint_command.hpp"

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr std::chrono::milliseconds kTransportConnectRetrySleep{100};
constexpr std::chrono::seconds kTransportConnectTimeout{5};

double DegreesToRadians(double degrees)
{
  return degrees * kPi / 180.0;
}

double RadiansToDegrees(double radians)
{
  return radians * 180.0 / kPi;
}

std::vector<std::string> DefaultJointNames()
{
  return {
    "d1_joint_0",
    "d1_joint_1",
    "d1_joint_2",
    "d1_joint_3",
    "d1_joint_4",
    "d1_joint_5",
    "d1_joint_6",
  };
}

std::string DefaultTransportSocketPath()
{
  const char * env_value = std::getenv("D1_TRANSPORT_SOCKET_PATH");
  if (env_value != nullptr && env_value[0] != '\0') {
    return std::string(env_value);
  }
  return go2w_d1_arm::ipc::kDefaultSocketPath;
}

std::optional<std::string> ExtractJsonScalar(const std::string & json, const std::string & key)
{
  const std::string quoted_key = "\"" + key + "\"";
  const auto key_pos = json.find(quoted_key);
  if (key_pos == std::string::npos) {
    return std::nullopt;
  }

  const auto colon_pos = json.find(':', key_pos + quoted_key.size());
  if (colon_pos == std::string::npos) {
    return std::nullopt;
  }

  std::size_t value_pos = colon_pos + 1;
  while (value_pos < json.size() && std::isspace(static_cast<unsigned char>(json[value_pos]))) {
    ++value_pos;
  }
  if (value_pos >= json.size()) {
    return std::nullopt;
  }

  if (json[value_pos] == '"') {
    ++value_pos;
    const auto end_quote = json.find('"', value_pos);
    if (end_quote == std::string::npos) {
      return std::nullopt;
    }
    return json.substr(value_pos, end_quote - value_pos);
  }

  std::size_t value_end = value_pos;
  while (value_end < json.size()) {
    const char current = json[value_end];
    if (current == ',' || current == '}' || std::isspace(static_cast<unsigned char>(current))) {
      break;
    }
    ++value_end;
  }

  if (value_end == value_pos) {
    return std::nullopt;
  }

  return json.substr(value_pos, value_end - value_pos);
}

std::optional<int> ExtractJsonInt(const std::string & json, const std::string & key)
{
  const auto token = ExtractJsonScalar(json, key);
  if (!token) {
    return std::nullopt;
  }

  try {
    std::size_t consumed = 0;
    const int value = std::stoi(*token, &consumed);
    if (consumed != token->size()) {
      return std::nullopt;
    }
    return value;
  } catch (const std::exception &) {
    return std::nullopt;
  }
}

class D1TransportClient
{
public:
  using ServoCallback =
    std::function<void(const std::array<double, go2w_d1_arm::ipc::kJointCount> &)>;
  using FeedbackCallback = std::function<void(const std::string &)>;

  D1TransportClient(std::string socket_path, rclcpp::Logger logger)
  : socket_path_(std::move(socket_path)),
    logger_(std::move(logger))
  {
  }

  ~D1TransportClient()
  {
    Stop();
  }

  void ConnectOrThrow(std::chrono::milliseconds timeout)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;

    while (std::chrono::steady_clock::now() < deadline) {
      if (TryConnect()) {
        return;
      }
      std::this_thread::sleep_for(kTransportConnectRetrySleep);
    }

    throw std::runtime_error(
            "timed out waiting for D1 transport socket at " + socket_path_);
  }

  void Start(ServoCallback servo_callback, FeedbackCallback feedback_callback)
  {
    servo_callback_ = std::move(servo_callback);
    feedback_callback_ = std::move(feedback_callback);
    stop_requested_.store(false);
    reader_thread_ = std::thread(&D1TransportClient::ReaderLoop, this);
  }

  void Stop()
  {
    stop_requested_.store(true);
    ShutdownSocket();

    if (reader_thread_.joinable()) {
      reader_thread_.join();
    }

    CloseSocket();
  }

  bool SendCommand(const std::string & payload, std::string * error_message)
  {
    std::lock_guard<std::mutex> lock(write_mutex_);

    const int socket_fd = socket_fd_.load();
    if (socket_fd < 0) {
      if (error_message != nullptr) {
        *error_message = "transport socket is not connected";
      }
      return false;
    }

    const std::string wire_message = go2w_d1_arm::ipc::BuildCommandLine(payload) + "\n";
    std::size_t written = 0;
    while (written < wire_message.size()) {
      const ssize_t result = send(
        socket_fd,
        wire_message.data() + written,
        wire_message.size() - written,
        MSG_NOSIGNAL);
      if (result > 0) {
        written += static_cast<std::size_t>(result);
        continue;
      }
      if (result < 0 && errno == EINTR) {
        continue;
      }

      if (error_message != nullptr) {
        *error_message = result < 0 ? std::strerror(errno) : "send() failed";
      }
      CloseSocketLocked();
      return false;
    }

    return true;
  }

private:
  bool TryConnect()
  {
    const int fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) {
      throw std::runtime_error("failed to create UNIX socket");
    }

    sockaddr_un address{};
    address.sun_family = AF_UNIX;
    if (socket_path_.size() >= sizeof(address.sun_path)) {
      close(fd);
      throw std::runtime_error("transport socket path is too long: " + socket_path_);
    }
    std::strncpy(address.sun_path, socket_path_.c_str(), sizeof(address.sun_path) - 1);

    if (connect(fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0) {
      close(fd);
      if (errno == ENOENT || errno == ECONNREFUSED) {
        return false;
      }
      throw std::runtime_error(
              "failed to connect to transport socket '" + socket_path_ + "': " + std::strerror(errno));
    }

    socket_fd_.store(fd);
    read_buffer_.clear();
    return true;
  }

  void ShutdownSocket()
  {
    std::lock_guard<std::mutex> lock(write_mutex_);
    const int socket_fd = socket_fd_.load();
    if (socket_fd >= 0) {
      shutdown(socket_fd, SHUT_RDWR);
    }
  }

  void CloseSocket()
  {
    std::lock_guard<std::mutex> lock(write_mutex_);
    CloseSocketLocked();
  }

  void CloseSocketLocked()
  {
    const int socket_fd = socket_fd_.load();
    if (socket_fd >= 0) {
      close(socket_fd);
      socket_fd_.store(-1);
    }
    read_buffer_.clear();
  }

  void ReaderLoop()
  {
    while (!stop_requested_.load()) {
      char buffer[4096];
      const int socket_fd = socket_fd_.load();
      if (socket_fd < 0) {
        return;
      }
      const ssize_t received = recv(socket_fd, buffer, sizeof(buffer), 0);
      if (received > 0) {
        read_buffer_.append(buffer, static_cast<std::size_t>(received));
        DrainBufferedLines();
        continue;
      }

      if (received == 0) {
        if (!stop_requested_.load()) {
          RCLCPP_ERROR(logger_, "D1 transport socket closed");
        }
        CloseSocket();
        return;
      }

      if (errno == EINTR) {
        continue;
      }
      if (errno == EBADF || errno == ENOTCONN) {
        return;
      }

      if (!stop_requested_.load()) {
        RCLCPP_ERROR(logger_, "D1 transport recv failed: %s", std::strerror(errno));
      }
      CloseSocket();
      return;
    }
  }

  void DrainBufferedLines()
  {
    std::size_t newline_position = read_buffer_.find('\n');
    while (newline_position != std::string::npos) {
      std::string line = read_buffer_.substr(0, newline_position);
      if (!line.empty() && line.back() == '\r') {
        line.pop_back();
      }
      read_buffer_.erase(0, newline_position + 1);

      if (!line.empty()) {
        DispatchLine(line);
      }
      newline_position = read_buffer_.find('\n');
    }
  }

  void DispatchLine(const std::string & line)
  {
    std::array<double, go2w_d1_arm::ipc::kJointCount> positions_deg{};
    if (go2w_d1_arm::ipc::ParseServoLine(line, &positions_deg)) {
      if (servo_callback_) {
        servo_callback_(positions_deg);
      }
      return;
    }

    std::string feedback;
    if (go2w_d1_arm::ipc::ParseFeedbackLine(line, &feedback)) {
      if (feedback_callback_) {
        feedback_callback_(feedback);
      }
      return;
    }

    RCLCPP_WARN(logger_, "Ignoring malformed D1 transport message: %s", line.c_str());
  }

  std::string socket_path_;
  rclcpp::Logger logger_;
  std::atomic_int socket_fd_{-1};
  std::atomic_bool stop_requested_{false};
  std::mutex write_mutex_;
  std::thread reader_thread_;
  std::string read_buffer_;
  ServoCallback servo_callback_;
  FeedbackCallback feedback_callback_;
};
}  // namespace

class D1ArmBridgeNode : public rclcpp::Node
{
public:
  D1ArmBridgeNode()
  : Node("d1_arm_bridge"),
    joint_names_(declare_parameter<std::vector<std::string>>("joint_names", DefaultJointNames())),
    transport_socket_path_(
      declare_parameter<std::string>("transport_socket_path", DefaultTransportSocketPath())),
    require_enable_before_motion_(declare_parameter<bool>("require_enable_before_motion", true)),
    enable_on_start_(declare_parameter<bool>("enable_on_start", false)),
    zero_on_start_(declare_parameter<bool>("zero_on_start", false)),
    multi_joint_mode_(declare_parameter<int>("multi_joint_mode", 1)),
    lock_force_(declare_parameter<int>("lock_force", 80000)),
    next_seq_(1),
    motion_authorized_(!require_enable_before_motion_)
  {
    motor_online_.fill(false);
    ValidateParameters();

    joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    arm_status_pub_ = create_publisher<go2w_d1_arm::msg::D1ArmStatus>("arm_status", 10);
    raw_feedback_pub_ = create_publisher<std_msgs::msg::String>("raw_feedback", 10);

    joint_command_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "joint_command",
      10,
      std::bind(&D1ArmBridgeNode::HandleJointCommand, this, std::placeholders::_1));

    enable_srv_ = create_service<std_srvs::srv::Trigger>(
      "enable",
      std::bind(&D1ArmBridgeNode::HandleEnable, this, std::placeholders::_1, std::placeholders::_2));
    disable_srv_ = create_service<std_srvs::srv::Trigger>(
      "disable",
      std::bind(&D1ArmBridgeNode::HandleDisable, this, std::placeholders::_1, std::placeholders::_2));
    power_on_srv_ = create_service<std_srvs::srv::Trigger>(
      "power_on",
      std::bind(&D1ArmBridgeNode::HandlePowerOn, this, std::placeholders::_1, std::placeholders::_2));
    power_off_srv_ = create_service<std_srvs::srv::Trigger>(
      "power_off",
      std::bind(&D1ArmBridgeNode::HandlePowerOff, this, std::placeholders::_1, std::placeholders::_2));
    zero_srv_ = create_service<std_srvs::srv::Trigger>(
      "zero",
      std::bind(&D1ArmBridgeNode::HandleZero, this, std::placeholders::_1, std::placeholders::_2));
    single_joint_srv_ = create_service<go2w_d1_arm::srv::SingleJointCommand>(
      "single_joint_command",
      std::bind(
        &D1ArmBridgeNode::HandleSingleJointCommand,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    transport_client_ = std::make_unique<D1TransportClient>(transport_socket_path_, get_logger());
    transport_client_->ConnectOrThrow(kTransportConnectTimeout);
    transport_client_->Start(
      std::bind(&D1ArmBridgeNode::HandleServoFeedback, this, std::placeholders::_1),
      std::bind(&D1ArmBridgeNode::HandleRawFeedback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Bridge ready. transport_socket='%s', require_enable_before_motion=%s, multi_joint_mode=%d, lock_force=%d",
      transport_socket_path_.c_str(),
      require_enable_before_motion_ ? "true" : "false",
      multi_joint_mode_,
      lock_force_);

    if (enable_on_start_) {
      std::string error_message;
      if (WriteCommand(BuildEnableCommand(lock_force_), &error_message)) {
        motion_authorized_.store(true);
        RCLCPP_INFO(get_logger(), "Sent enable command on startup");
      } else {
        RCLCPP_ERROR(get_logger(), "Failed to send enable command on startup: %s", error_message.c_str());
      }
    }

    if (zero_on_start_) {
      std::string error_message;
      if (!MotionAllowed()) {
        RCLCPP_WARN(get_logger(), "Skipping zero_on_start because motion is not yet authorized");
      } else if (WriteCommand(BuildZeroCommand(), &error_message)) {
        RCLCPP_INFO(get_logger(), "Sent zero command on startup");
      } else {
        RCLCPP_ERROR(get_logger(), "Failed to send zero command on startup: %s", error_message.c_str());
      }
    }
  }

  ~D1ArmBridgeNode() override
  {
    transport_client_.reset();
  }

private:
  void ValidateParameters() const
  {
    if (joint_names_.size() != go2w_d1_arm::ipc::kJointCount) {
      throw std::runtime_error("joint_names parameter must contain exactly 7 entries");
    }
    if (transport_socket_path_.empty()) {
      throw std::runtime_error("transport_socket_path parameter must not be empty");
    }
    if (multi_joint_mode_ != 0 && multi_joint_mode_ != 1) {
      throw std::runtime_error("multi_joint_mode must be 0 or 1");
    }
    if (lock_force_ < 0 || lock_force_ > 80000) {
      throw std::runtime_error("lock_force must be in the range [0, 80000]");
    }
  }

  uint32_t NextSequence()
  {
    return next_seq_.fetch_add(1);
  }

  std::string BuildEnableCommand(int mode)
  {
    std::ostringstream payload;
    payload
      << "{\"seq\":" << NextSequence()
      << ",\"address\":1,\"funcode\":5,\"data\":{\"mode\":" << mode << "}}";
    return payload.str();
  }

  std::string BuildPowerCommand(bool powered)
  {
    std::ostringstream payload;
    payload
      << "{\"seq\":" << NextSequence()
      << ",\"address\":1,\"funcode\":6,\"data\":{\"power\":" << (powered ? 1 : 0) << "}}";
    return payload.str();
  }

  std::string BuildZeroCommand()
  {
    std::ostringstream payload;
    payload << "{\"seq\":" << NextSequence() << ",\"address\":1,\"funcode\":7}";
    return payload.str();
  }

  std::string BuildSingleJointCommand(uint8_t joint_id, double angle_rad, int32_t delay_ms)
  {
    std::ostringstream payload;
    payload << std::fixed;
    payload.precision(6);
    payload
      << "{\"seq\":" << NextSequence()
      << ",\"address\":1,\"funcode\":1,\"data\":{\"id\":" << static_cast<int>(joint_id)
      << ",\"angle\":" << RadiansToDegrees(angle_rad)
      << ",\"delay_ms\":" << delay_ms << "}}";
    return payload.str();
  }

  std::string BuildMultiJointCommand(const std::vector<double> & positions_rad)
  {
    std::ostringstream payload;
    payload << std::fixed;
    payload.precision(6);
    payload
      << "{\"seq\":" << NextSequence()
      << ",\"address\":1,\"funcode\":2,\"data\":{\"mode\":" << multi_joint_mode_;
    for (std::size_t i = 0; i < go2w_d1_arm::ipc::kJointCount; ++i) {
      payload << ",\"angle" << i << "\":" << RadiansToDegrees(positions_rad.at(i));
    }
    payload << "}}";
    return payload.str();
  }

  bool WriteCommand(const std::string & payload, std::string * error_message)
  {
    if (!transport_client_) {
      if (error_message != nullptr) {
        *error_message = "transport client is not initialized";
      }
      return false;
    }

    return transport_client_->SendCommand(payload, error_message);
  }

  bool MotionAllowed() const
  {
    return !require_enable_before_motion_ || motion_authorized_.load();
  }

  void PublishArmStatus()
  {
    go2w_d1_arm::msg::D1ArmStatus status;
    status.header.stamp = now();
    status.header.frame_id = "d1_arm";

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      status.enabled = enabled_;
      status.powered = powered_;
      status.healthy = healthy_;
      status.motor_online.assign(motor_online_.begin(), motor_online_.end());
    }

    arm_status_pub_->publish(status);
  }

  void HandleServoFeedback(
    const std::array<double, go2w_d1_arm::ipc::kJointCount> & positions_deg)
  {
    sensor_msgs::msg::JointState joint_state;
    joint_state.header.stamp = now();
    joint_state.header.frame_id = "d1_arm";
    joint_state.name = joint_names_;
    joint_state.position.resize(go2w_d1_arm::ipc::kJointCount);
    for (std::size_t i = 0; i < go2w_d1_arm::ipc::kJointCount; ++i) {
      joint_state.position[i] = DegreesToRadians(positions_deg[i]);
    }
    joint_state_pub_->publish(joint_state);
  }

  void HandleRawFeedback(const std::string & payload)
  {
    std_msgs::msg::String raw_feedback;
    raw_feedback.data = payload;
    raw_feedback_pub_->publish(raw_feedback);

    const auto address = ExtractJsonInt(raw_feedback.data, "address");
    const auto funcode = ExtractJsonInt(raw_feedback.data, "funcode");
    if (!address || !funcode) {
      return;
    }

    if (*address == 2 && *funcode == 3) {
      const auto enable_status = ExtractJsonInt(raw_feedback.data, "enable_status");
      const auto power_status = ExtractJsonInt(raw_feedback.data, "power_status");
      const auto error_status = ExtractJsonInt(raw_feedback.data, "error_status");
      if (!enable_status || !power_status || !error_status) {
        RCLCPP_WARN(get_logger(), "Failed to parse arm status feedback: %s", raw_feedback.data.c_str());
        return;
      }

      bool enabled = false;
      bool powered = false;
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        enabled_ = *enable_status != 0;
        powered_ = *power_status != 0;
        healthy_ = *error_status != 0;
        enabled = enabled_;
        powered = powered_;
      }

      if (!powered || !enabled) {
        motion_authorized_.store(false);
      } else {
        motion_authorized_.store(true);
      }

      PublishArmStatus();
      return;
    }

    if (*address == 2 && *funcode == 4) {
      std::array<bool, go2w_d1_arm::ipc::kJointCount> online{};
      for (std::size_t i = 0; i < go2w_d1_arm::ipc::kJointCount; ++i) {
        const auto value = ExtractJsonInt(raw_feedback.data, "motor" + std::to_string(i) + "_status");
        if (!value) {
          RCLCPP_WARN(get_logger(), "Failed to parse motor status feedback: %s", raw_feedback.data.c_str());
          return;
        }
        online[i] = *value != 0;
      }

      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        motor_online_ = online;
      }

      PublishArmStatus();
      return;
    }

    if (*address == 3 && *funcode == 1) {
      const auto recv_status = ExtractJsonInt(raw_feedback.data, "recv_status");
      if (recv_status && *recv_status == 0) {
        RCLCPP_WARN(get_logger(), "Arm reported command parse failure: %s", raw_feedback.data.c_str());
      }
      return;
    }

    if (*address == 3 && *funcode == 2) {
      const auto exec_status = ExtractJsonInt(raw_feedback.data, "exec_status");
      if (exec_status && *exec_status == 0) {
        RCLCPP_WARN(get_logger(), "Arm reported command execution failure: %s", raw_feedback.data.c_str());
      }
    }
  }

  void HandleJointCommand(const sensor_msgs::msg::JointState::SharedPtr message)
  {
    if (message->position.size() != go2w_d1_arm::ipc::kJointCount) {
      RCLCPP_WARN(
        get_logger(),
        "Joint command ignored: expected %zu positions but received %zu",
        go2w_d1_arm::ipc::kJointCount,
        message->position.size());
      return;
    }

    if (!MotionAllowed()) {
      RCLCPP_WARN(get_logger(), "Joint command ignored: motion is gated until enable succeeds");
      return;
    }

    std::string error_message;
    if (!WriteCommand(BuildMultiJointCommand(message->position), &error_message)) {
      RCLCPP_ERROR(get_logger(), "Failed to send joint command: %s", error_message.c_str());
    }
  }

  void HandleEnable(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request;
    std::string error_message;
    response->success = WriteCommand(BuildEnableCommand(lock_force_), &error_message);
    if (response->success) {
      motion_authorized_.store(true);
      response->message = "Sent funcode=5 enable command";
    } else {
      response->message = error_message;
    }
  }

  void HandleDisable(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request;
    std::string error_message;
    response->success = WriteCommand(BuildEnableCommand(0), &error_message);
    if (response->success) {
      motion_authorized_.store(false);
      response->message = "Sent funcode=5 disable command";
    } else {
      response->message = error_message;
    }
  }

  void HandlePowerOn(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request;
    std::string error_message;
    response->success = WriteCommand(BuildPowerCommand(true), &error_message);
    response->message = response->success ? "Sent funcode=6 power on command" : error_message;
  }

  void HandlePowerOff(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request;
    std::string error_message;
    response->success = WriteCommand(BuildPowerCommand(false), &error_message);
    if (response->success) {
      motion_authorized_.store(false);
      response->message = "Sent funcode=6 power off command";
    } else {
      response->message = error_message;
    }
  }

  void HandleZero(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request;
    if (!MotionAllowed()) {
      response->success = false;
      response->message = "Motion is gated until enable succeeds";
      return;
    }

    std::string error_message;
    response->success = WriteCommand(BuildZeroCommand(), &error_message);
    response->message = response->success ? "Sent funcode=7 zero command" : error_message;
  }

  void HandleSingleJointCommand(
    const std::shared_ptr<go2w_d1_arm::srv::SingleJointCommand::Request> request,
    std::shared_ptr<go2w_d1_arm::srv::SingleJointCommand::Response> response)
  {
    if (request->joint_id >= go2w_d1_arm::ipc::kJointCount) {
      response->accepted = false;
      response->message = "joint_id must be in the range [0, 6]";
      return;
    }
    if (!std::isfinite(request->angle_rad)) {
      response->accepted = false;
      response->message = "angle_rad must be finite";
      return;
    }
    if (!MotionAllowed()) {
      response->accepted = false;
      response->message = "Motion is gated until enable succeeds";
      return;
    }

    std::string error_message;
    response->accepted = WriteCommand(
      BuildSingleJointCommand(request->joint_id, request->angle_rad, request->delay_ms),
      &error_message);
    response->message = response->accepted ? "Sent funcode=1 single-joint command" : error_message;
  }

  std::vector<std::string> joint_names_;
  std::string transport_socket_path_;
  bool require_enable_before_motion_;
  bool enable_on_start_;
  bool zero_on_start_;
  int multi_joint_mode_;
  int lock_force_;
  std::atomic<uint32_t> next_seq_;
  std::atomic_bool motion_authorized_;

  std::mutex state_mutex_;
  bool enabled_ = false;
  bool powered_ = false;
  bool healthy_ = false;
  std::array<bool, go2w_d1_arm::ipc::kJointCount> motor_online_{};

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<go2w_d1_arm::msg::D1ArmStatus>::SharedPtr arm_status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr raw_feedback_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_command_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr enable_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disable_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr power_on_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr power_off_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr zero_srv_;
  rclcpp::Service<go2w_d1_arm::srv::SingleJointCommand>::SharedPtr single_joint_srv_;

  std::unique_ptr<D1TransportClient> transport_client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  int exit_code = 0;
  try {
    auto node = std::make_shared<D1ArmBridgeNode>();
    rclcpp::spin(node);
  } catch (const std::exception & ex) {
    RCLCPP_FATAL(rclcpp::get_logger("d1_arm_bridge"), "%s", ex.what());
    exit_code = 1;
  }

  rclcpp::shutdown();
  return exit_code;
}
