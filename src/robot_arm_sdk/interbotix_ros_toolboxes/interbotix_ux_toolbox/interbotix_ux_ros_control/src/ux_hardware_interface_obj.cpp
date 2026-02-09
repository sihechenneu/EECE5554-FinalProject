#include "interbotix_ux_ros_control/ux_hardware_interface_obj.h"
#include <chrono>
#include <thread>

namespace interbotix_ux_ros_control
{

UXHardwareInterface::UXHardwareInterface(const rclcpp::NodeOptions & options)
: Node("ux_hardware_interface", options),
  curr_state_(0),
  curr_mode_(0),
  curr_err_(0),
  dof_(6),
  control_rate_(50.0),
  gripper_cmd_prev_(0.0),
  use_gripper_(false)
{
  declare_parameter<int>("DOF", 6);
  declare_parameter<std::string>("xarm_robot_ip", "");
  declare_parameter<std::vector<std::string>>("joint_names", std::vector<std::string>{});
  declare_parameter<bool>("hardware_interface.use_gripper", false);
  declare_parameter<double>("hardware_interface.control_rate", 50.0);
  declare_parameter<std::string>("hardware_interface.joint_states_topic", "arm/joint_states");
  declare_parameter<std::string>("hardware_interface.robot_state_topic", "xarm_states");
  declare_parameter<std::string>("hardware_interface.gripper_name", "gripper");
  declare_parameter<double>("hardware_interface.gripper_pulse_vel", 1500.0);

  dof_ = get_parameter("DOF").as_int();
  joint_names_ = get_parameter("joint_names").as_string_array();
  if (joint_names_.empty()) {
    for (int i = 0; i < dof_; i++) {
      joint_names_.push_back("joint" + std::to_string(i + 1));
    }
  }
  use_gripper_ = get_parameter("hardware_interface.use_gripper").as_bool();
  control_rate_ = get_parameter("hardware_interface.control_rate").as_double();
  gripper_name_ = get_parameter("hardware_interface.gripper_name").as_string();
  std::string js_topic = get_parameter("hardware_interface.joint_states_topic").as_string();
  std::string robot_state_topic = get_parameter("hardware_interface.robot_state_topic").as_string();

  sub_joint_states_ = create_subscription<sensor_msgs::msg::JointState>(
    js_topic, 1, std::bind(&UXHardwareInterface::joint_state_cb, this, std::placeholders::_1));
  sub_robot_states_ = create_subscription<xarm_msgs::msg::RobotMsg>(
    robot_state_topic, 1, std::bind(&UXHardwareInterface::robot_state_cb, this, std::placeholders::_1));
}

bool UXHardwareInterface::init()
{
  uxarm_.init(shared_from_this());

  if (use_gripper_) {
    joint_names_.push_back(gripper_name_);
    float pulse_vel = static_cast<float>(get_parameter("hardware_interface.gripper_pulse_vel").as_double());
    uxarm_.gripperConfig(pulse_vel);
  }

  while (!joint_states_ && rclcpp::ok()) {
    rclcpp::spin_some(get_node_base_interface());
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  if (!joint_states_ || joint_states_->position.empty()) {
    RCLCPP_ERROR(get_logger(), "No joint_states received. Check joint_states_topic.");
    return false;
  }

  size_t n = joint_names_.size();
  joint_positions_.resize(n);
  joint_velocities_.resize(n);
  joint_efforts_.resize(n);
  joint_position_commands_.resize(n);
  joint_position_commands_float_.resize(dof_);
  joint_position_commands_float_prev_.resize(dof_);

  for (int i = 0; i < dof_; i++) {
    joint_position_commands_[i] = joint_states_->position[i];
    joint_position_commands_float_[i] = static_cast<float>(joint_states_->position[i]);
    joint_position_commands_float_prev_[i] = static_cast<float>(joint_states_->position[i]);
  }
  if (use_gripper_) {
    gripper_cmd_prev_ = joint_states_->position[dof_];
    joint_position_commands_[dof_] = gripper_cmd_prev_;
  }

  (void)uxarm_.motionEnable(1);
  (void)uxarm_.setMode(XARM_MODE::SERVO);
  int ret3 = uxarm_.setState(XARM_STATE::START);
  if (ret3 != 0) {
    RCLCPP_ERROR(get_logger(), "Xarm may not be connected or hardware error (ret=%d). Check xarm_robot_ip.", ret3);
    return false;
  }

  double period = 1.0 / control_rate_;
  timer_ = create_wall_timer(
    std::chrono::duration<double>(period),
    std::bind(&UXHardwareInterface::control_timer_cb, this));

  RCLCPP_INFO(get_logger(), "ux_hardware_interface started (dof=%d use_gripper=%d)", dof_, use_gripper_);
  return true;
}

UXHardwareInterface::~UXHardwareInterface()
{
  uxarm_.setMode(XARM_MODE::POSE);
}

void UXHardwareInterface::joint_state_cb(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  joint_states_ = msg;
}

void UXHardwareInterface::robot_state_cb(const xarm_msgs::msg::RobotMsg::SharedPtr msg)
{
  curr_mode_ = msg->mode;
  curr_state_ = msg->state;
  curr_err_ = msg->err;
}

void UXHardwareInterface::control_timer_cb()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!joint_states_ || joint_states_->position.empty()) return;
    for (size_t j = 0; j < joint_names_.size(); j++) {
      joint_positions_[j] = joint_states_->position[j];
      joint_velocities_[j] = j < joint_states_->velocity.size() ? joint_states_->velocity[j] : 0.0;
      joint_efforts_[j] = j < joint_states_->effort.size() ? joint_states_->effort[j] : 0.0;
    }
  }

  if (need_reset()) {
    std::lock_guard<std::mutex> lock(mutex_);
    for (int k = 0; k < dof_; k++)
      joint_position_commands_float_[k] = static_cast<float>(joint_positions_[k]);
    return;
  }

  for (int k = 0; k < dof_; k++) {
    float cmd = static_cast<float>(joint_position_commands_[k]);
    if (std::abs(joint_position_commands_float_[k] - cmd) * control_rate_ > 3.14f * 1.25f)
      RCLCPP_WARN(get_logger(), "joint %d abnormal command! previous: %f, this: %f", k + 1, joint_position_commands_float_[k], cmd);
    joint_position_commands_float_[k] = cmd;
  }

  if (joint_position_commands_float_ != joint_position_commands_float_prev_) {
    uxarm_.setServoJ(joint_position_commands_float_);
    joint_position_commands_float_prev_ = joint_position_commands_float_;
  }

  if (use_gripper_ && (std::abs(gripper_cmd_prev_ - joint_position_commands_.back()) > 1e-6)) {
    float pulse = 850.0f - static_cast<float>(joint_position_commands_.back() * 1000.0);
    uxarm_.gripperMove(pulse);
    gripper_cmd_prev_ = joint_position_commands_.back();
  }
}

bool UXHardwareInterface::need_reset() const
{
  return (curr_state_ == 4 || curr_state_ == 5 || curr_err_ != 0);
}

}  // namespace interbotix_ux_ros_control
