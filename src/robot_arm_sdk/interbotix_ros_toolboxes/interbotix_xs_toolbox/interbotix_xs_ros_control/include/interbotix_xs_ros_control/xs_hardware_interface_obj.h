#ifndef INTERBOTIX_XS_ROS_CONTROL__XS_HARDWARE_INTERFACE_OBJ_H_
#define INTERBOTIX_XS_ROS_CONTROL__XS_HARDWARE_INTERFACE_OBJ_H_

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <interbotix_xs_msgs/msg/joint_group_command.hpp>
#include <interbotix_xs_msgs/msg/joint_single_command.hpp>
#include <interbotix_xs_msgs/srv/robot_info.hpp>

namespace interbotix_xs_ros_control
{

class XSHardwareInterface : public rclcpp::Node
{
public:
  explicit XSHardwareInterface(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~XSHardwareInterface() = default;

private:
  void joint_state_cb(const sensor_msgs::msg::JointState::SharedPtr msg);
  void control_timer_cb();

  double loop_hz_;
  std::string group_name_;
  std::string gripper_name_;
  std::string joint_states_topic_;

  std::vector<double> joint_positions_;
  std::vector<double> joint_velocities_;
  std::vector<double> joint_efforts_;
  std::vector<double> joint_position_commands_;
  std::vector<float> joint_commands_prev_;
  float gripper_cmd_prev_;

  size_t num_joints_;
  std::vector<int16_t> joint_state_indices_;
  std::vector<std::string> joint_names_;

  sensor_msgs::msg::JointState::SharedPtr joint_states_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_states_;
  rclcpp::Publisher<interbotix_xs_msgs::msg::JointGroupCommand>::SharedPtr pub_group_;
  rclcpp::Publisher<interbotix_xs_msgs::msg::JointSingleCommand>::SharedPtr pub_gripper_;
  rclcpp::Client<interbotix_xs_msgs::srv::RobotInfo>::SharedPtr srv_robot_info_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace interbotix_xs_ros_control

#endif  // INTERBOTIX_XS_ROS_CONTROL__XS_HARDWARE_INTERFACE_OBJ_H_
