#include "interbotix_xs_ros_control/xs_hardware_interface_obj.h"
#include <chrono>

namespace interbotix_xs_ros_control
{

XSHardwareInterface::XSHardwareInterface(const rclcpp::NodeOptions & options)
: Node("xs_hardware_interface", options)
{
  this->declare_parameter<double>("hardware_interface.loop_hz", 10.0);
  this->declare_parameter<std::string>("hardware_interface.group_name", "arm");
  this->declare_parameter<std::string>("hardware_interface.gripper_name", "gripper");
  this->declare_parameter<std::string>("hardware_interface.joint_states_topic", "joint_states");

  loop_hz_ = this->get_parameter("hardware_interface.loop_hz").as_double();
  group_name_ = this->get_parameter("hardware_interface.group_name").as_string();
  gripper_name_ = this->get_parameter("hardware_interface.gripper_name").as_string();
  joint_states_topic_ = this->get_parameter("hardware_interface.joint_states_topic").as_string();

  srv_robot_info_ = this->create_client<interbotix_xs_msgs::srv::RobotInfo>("get_robot_info");
  while (!srv_robot_info_->wait_for_service(std::chrono::seconds(1)) && rclcpp::ok()) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for get_robot_info...");
  }

  auto req_group = std::make_shared<interbotix_xs_msgs::srv::RobotInfo::Request>();
  req_group->cmd_type = "group";
  req_group->name = group_name_;
  auto req_gripper = std::make_shared<interbotix_xs_msgs::srv::RobotInfo::Request>();
  req_gripper->cmd_type = "single";
  req_gripper->name = gripper_name_;

  auto result_group = srv_robot_info_->async_send_request(req_group);
  auto result_gripper = srv_robot_info_->async_send_request(req_gripper);
  rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_group, std::chrono::seconds(5));
  rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_gripper, std::chrono::seconds(5));

  auto res_group = result_group.get();
  auto res_gripper = result_gripper.get();
  if (!res_group || !res_gripper) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get robot info");
    return;
  }

  num_joints_ = res_group->num_joints + 1;
  joint_names_ = res_group->joint_names;
  joint_names_.push_back(res_gripper->joint_names.at(0));
  joint_state_indices_ = res_group->joint_state_indices;
  joint_state_indices_.push_back(res_gripper->joint_state_indices.at(0));

  joint_positions_.resize(num_joints_);
  joint_velocities_.resize(num_joints_);
  joint_efforts_.resize(num_joints_);
  joint_position_commands_.resize(num_joints_);
  joint_commands_prev_.resize(num_joints_);

  pub_group_ = this->create_publisher<interbotix_xs_msgs::msg::JointGroupCommand>(
    "commands/joint_group", 1);
  pub_gripper_ = this->create_publisher<interbotix_xs_msgs::msg::JointSingleCommand>(
    "commands/joint_single", 1);
  sub_joint_states_ = this->create_subscription<sensor_msgs::msg::JointState>(
    joint_states_topic_, 1, std::bind(&XSHardwareInterface::joint_state_cb, this, std::placeholders::_1));

  double period = 1.0 / loop_hz_;
  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(period),
    std::bind(&XSHardwareInterface::control_timer_cb, this));

  RCLCPP_INFO(this->get_logger(), "xs_hardware_interface started (group=%s gripper=%s)",
    group_name_.c_str(), gripper_name_.c_str());
}

void XSHardwareInterface::joint_state_cb(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  joint_states_ = msg;
}

void XSHardwareInterface::control_timer_cb()
{
  if (!joint_states_ || joint_states_->position.empty()) {
    return;
  }

  for (size_t i = 0; i < num_joints_; i++) {
    size_t idx = static_cast<size_t>(joint_state_indices_[i]);
    if (idx < joint_states_->position.size()) {
      joint_positions_[i] = joint_states_->position[idx];
      joint_velocities_[i] = (idx < joint_states_->velocity.size()) ? joint_states_->velocity[idx] : 0.0;
      joint_efforts_[i] = (idx < joint_states_->effort.size()) ? joint_states_->effort[idx] : 0.0;
    }
  }

  if (joint_position_commands_.empty()) {
    joint_position_commands_ = joint_positions_;
    joint_commands_prev_ = std::vector<float>(joint_position_commands_.begin(), joint_position_commands_.end());
    gripper_cmd_prev_ = joint_position_commands_.back() * 2.0f;
  }

  interbotix_xs_msgs::msg::JointGroupCommand group_msg;
  group_msg.name = group_name_;
  for (size_t i = 0; i < num_joints_ - 1; i++) {
    group_msg.cmd.push_back(static_cast<float>(joint_position_commands_[i]));
  }

  interbotix_xs_msgs::msg::JointSingleCommand gripper_msg;
  gripper_msg.name = gripper_name_;
  gripper_msg.cmd = static_cast<float>(joint_position_commands_.back() * 2.0);

  bool group_changed = (joint_commands_prev_.size() != group_msg.cmd.size());
  if (!group_changed) {
    for (size_t i = 0; i < group_msg.cmd.size(); i++) {
      if (std::abs(joint_commands_prev_[i] - group_msg.cmd[i]) > 1e-6f) {
        group_changed = true;
        break;
      }
    }
  }
  if (group_changed) {
    pub_group_->publish(group_msg);
    for (size_t i = 0; i < group_msg.cmd.size(); i++) {
      joint_commands_prev_[i] = group_msg.cmd[i];
    }
  }
  if (std::abs(gripper_cmd_prev_ - gripper_msg.cmd) > 1e-6f) {
    pub_gripper_->publish(gripper_msg);
    gripper_cmd_prev_ = gripper_msg.cmd;
  }
}

}  // namespace interbotix_xs_ros_control
