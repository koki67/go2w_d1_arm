#include <algorithm>
#include <array>
#include <atomic>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <functional>
#include <memory>
#include <mutex>
#include <net/if.h>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include "go2w_d1_arm/msg/d1_arm_status.hpp"
#include "go2w_d1_arm/srv/single_joint_command.hpp"
#include "msg/ArmString_.hpp"
#include "msg/PubServoInfo_.hpp"

namespace
{
constexpr char kCommandTopic[] = "rt/arm_Command";
constexpr char kServoTopic[] = "current_servo_angle";
constexpr char kFeedbackTopic[] = "arm_Feedback";
constexpr std::size_t kJointCount = 7;
constexpr double kPi = 3.14159265358979323846;

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

std::vector<std::string> AvailableNetworkInterfaces()
{
  std::vector<std::string> interfaces;
  struct if_nameindex * indexed_interfaces = ::if_nameindex();
  if (indexed_interfaces == nullptr) {
    return interfaces;
  }

  for (struct if_nameindex * entry = indexed_interfaces;
    entry->if_index != 0 || entry->if_name != nullptr;
    ++entry)
  {
    if (entry->if_name != nullptr) {
      interfaces.emplace_back(entry->if_name);
    }
  }

  if_freenameindex(indexed_interfaces);
  std::sort(interfaces.begin(), interfaces.end());
  interfaces.erase(std::unique(interfaces.begin(), interfaces.end()), interfaces.end());
  return interfaces;
}

std::string JoinStrings(const std::vector<std::string> & values)
{
  std::ostringstream stream;
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (i > 0) {
      stream << ", ";
    }
    stream << values[i];
  }
  return stream.str();
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
}  // namespace

class D1ArmBridgeNode : public rclcpp::Node
{
public:
  D1ArmBridgeNode()
  : Node("d1_arm_bridge"),
    joint_names_(declare_parameter<std::vector<std::string>>("joint_names", DefaultJointNames())),
    require_enable_before_motion_(declare_parameter<bool>("require_enable_before_motion", true)),
    enable_on_start_(declare_parameter<bool>("enable_on_start", false)),
    zero_on_start_(declare_parameter<bool>("zero_on_start", false)),
    multi_joint_mode_(declare_parameter<int>("multi_joint_mode", 1)),
    lock_force_(declare_parameter<int>("lock_force", 80000)),
    network_interface_(ResolveNetworkInterface()),
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

    InitializeDds();

    RCLCPP_INFO(
      get_logger(),
      "Bridge ready. unitree_interface='%s', require_enable_before_motion=%s, multi_joint_mode=%d, lock_force=%d",
      network_interface_.empty() ? "<auto>" : network_interface_.c_str(),
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
    feedback_subscriber_.reset();
    servo_subscriber_.reset();
    command_publisher_.reset();
    unitree::robot::ChannelFactory::Instance()->Release();
  }

private:
  void ValidateParameters() const
  {
    if (joint_names_.size() != kJointCount) {
      throw std::runtime_error("joint_names parameter must contain exactly 7 entries");
    }
    if (multi_joint_mode_ != 0 && multi_joint_mode_ != 1) {
      throw std::runtime_error("multi_joint_mode must be 0 or 1");
    }
    if (lock_force_ < 0 || lock_force_ > 80000) {
      throw std::runtime_error("lock_force must be in the range [0, 80000]");
    }
  }

  std::string ResolveNetworkInterface()
  {
    const auto configured_value = declare_parameter<std::string>("network_interface", "");
    std::string selected_interface = configured_value;
    if (selected_interface.empty()) {
      const char * env_value = std::getenv("UNITREE_NETWORK_INTERFACE");
      selected_interface = env_value == nullptr ? std::string{} : std::string(env_value);
    }

    if (selected_interface.empty()) {
      return selected_interface;
    }

    const auto available_interfaces = AvailableNetworkInterfaces();
    if (
      std::find(
        available_interfaces.begin(),
        available_interfaces.end(),
        selected_interface) == available_interfaces.end())
    {
      std::ostringstream message;
      message
        << "configured network interface '" << selected_interface
        << "' was not found. Available interfaces: " << JoinStrings(available_interfaces);
      throw std::runtime_error(message.str());
    }

    return selected_interface;
  }

  void InitializeDds()
  {
    try {
      if (network_interface_.empty()) {
        unitree::robot::ChannelFactory::Instance()->Init(0);
      } else {
        unitree::robot::ChannelFactory::Instance()->Init(0, network_interface_);
      }

      command_publisher_ =
        std::make_unique<unitree::robot::ChannelPublisher<unitree_arm::msg::dds_::ArmString_>>(
        kCommandTopic);
      command_publisher_->InitChannel();

      servo_subscriber_ =
        std::make_unique<unitree::robot::ChannelSubscriber<unitree_arm::msg::dds_::PubServoInfo_>>(
        kServoTopic);
      servo_subscriber_->InitChannel(
        std::bind(&D1ArmBridgeNode::HandleServoFeedback, this, std::placeholders::_1),
        1);

      feedback_subscriber_ =
        std::make_unique<unitree::robot::ChannelSubscriber<unitree_arm::msg::dds_::ArmString_>>(
        kFeedbackTopic);
      feedback_subscriber_->InitChannel(
        std::bind(&D1ArmBridgeNode::HandleRawFeedback, this, std::placeholders::_1),
        1);
    } catch (const std::exception & ex) {
      std::ostringstream message;
      message << "DDS/channel initialization failed";
      if (!network_interface_.empty()) {
        message << " on interface '" << network_interface_ << "'";
      }
      message << ": " << ex.what();
      throw std::runtime_error(message.str());
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
    for (std::size_t i = 0; i < kJointCount; ++i) {
      payload << ",\"angle" << i << "\":" << RadiansToDegrees(positions_rad.at(i));
    }
    payload << "}}";
    return payload.str();
  }

  bool WriteCommand(const std::string & payload, std::string * error_message)
  {
    std::lock_guard<std::mutex> lock(command_mutex_);

    if (!command_publisher_) {
      if (error_message != nullptr) {
        *error_message = "command publisher is not initialized";
      }
      return false;
    }

    try {
      unitree_arm::msg::dds_::ArmString_ message;
      message.data_() = payload;
      const bool success = command_publisher_->Write(message);
      if (!success && error_message != nullptr) {
        *error_message = "failed to write to rt/arm_Command";
      }
      return success;
    } catch (const std::exception & ex) {
      if (error_message != nullptr) {
        *error_message = ex.what();
      }
      return false;
    }
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

  void HandleServoFeedback(const void * message)
  {
    const auto * servo_info = static_cast<const unitree_arm::msg::dds_::PubServoInfo_ *>(message);
    if (servo_info == nullptr) {
      return;
    }

    sensor_msgs::msg::JointState joint_state;
    joint_state.header.stamp = now();
    joint_state.header.frame_id = "d1_arm";
    joint_state.name = joint_names_;
    joint_state.position.resize(kJointCount);
    joint_state.position[0] = DegreesToRadians(servo_info->servo0_data_());
    joint_state.position[1] = DegreesToRadians(servo_info->servo1_data_());
    joint_state.position[2] = DegreesToRadians(servo_info->servo2_data_());
    joint_state.position[3] = DegreesToRadians(servo_info->servo3_data_());
    joint_state.position[4] = DegreesToRadians(servo_info->servo4_data_());
    joint_state.position[5] = DegreesToRadians(servo_info->servo5_data_());
    joint_state.position[6] = DegreesToRadians(servo_info->servo6_data_());
    joint_state_pub_->publish(joint_state);
  }

  void HandleRawFeedback(const void * message)
  {
    const auto * feedback = static_cast<const unitree_arm::msg::dds_::ArmString_ *>(message);
    if (feedback == nullptr) {
      return;
    }

    std_msgs::msg::String raw_feedback;
    raw_feedback.data = feedback->data_();
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
      std::array<bool, kJointCount> online{};
      for (std::size_t i = 0; i < kJointCount; ++i) {
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
    if (message->position.size() != kJointCount) {
      RCLCPP_WARN(
        get_logger(),
        "Joint command ignored: expected %zu positions but received %zu",
        kJointCount,
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
    if (request->joint_id >= kJointCount) {
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
  bool require_enable_before_motion_;
  bool enable_on_start_;
  bool zero_on_start_;
  int multi_joint_mode_;
  int lock_force_;
  std::string network_interface_;
  std::atomic<uint32_t> next_seq_;
  std::atomic_bool motion_authorized_;

  std::mutex command_mutex_;
  std::mutex state_mutex_;
  bool enabled_ = false;
  bool powered_ = false;
  bool healthy_ = false;
  std::array<bool, kJointCount> motor_online_{};

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

  std::unique_ptr<unitree::robot::ChannelPublisher<unitree_arm::msg::dds_::ArmString_>>
    command_publisher_;
  std::unique_ptr<unitree::robot::ChannelSubscriber<unitree_arm::msg::dds_::PubServoInfo_>>
    servo_subscriber_;
  std::unique_ptr<unitree::robot::ChannelSubscriber<unitree_arm::msg::dds_::ArmString_>>
    feedback_subscriber_;
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
