#ifndef INTERBOTIX_UX_ROS_CONTROL__UX_HARDWARE_INTERFACE_OBJ_H_
#define INTERBOTIX_UX_ROS_CONTROL__UX_HARDWARE_INTERFACE_OBJ_H_

#include <mutex>
#include <string>
#include <vector>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <xarm_msgs/msg/robot_msg.hpp>
#include <xarm_ros_client.h>
#include <xarm/instruction/uxbus_cmd_config.h>

namespace interbotix_ux_ros_control
{

class UXHardwareInterface : public rclcpp::Node
{
public:
  explicit UXHardwareInterface(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~UXHardwareInterface();
  /** Call after construction to init xarm client and start control timer. */
  bool init();

private:
  void joint_state_cb(const sensor_msgs::msg::JointState::SharedPtr msg);
  void robot_state_cb(const xarm_msgs::msg::RobotMsg::SharedPtr msg);
  void control_timer_cb();
  bool need_reset() const;

  std::vector<double> joint_positions_;
  std::vector<double> joint_velocities_;
  std::vector<double> joint_efforts_;
  std::vector<double> joint_position_commands_;
  std::vector<float> joint_position_commands_float_;
  std::vector<float> joint_position_commands_float_prev_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_states_;
  rclcpp::Subscription<xarm_msgs::msg::RobotMsg>::SharedPtr sub_robot_states_;
  rclcpp::TimerBase::SharedPtr timer_;

  sensor_msgs::msg::JointState::SharedPtr joint_states_;
  xarm_api::XArmROSClient uxarm_;

  int curr_state_;
  int curr_mode_;
  int curr_err_;
  int dof_;
  double control_rate_;
  double gripper_cmd_prev_;
  std::string gripper_name_;
  std::vector<std::string> joint_names_;
  mutable std::mutex mutex_;
  bool use_gripper_;
};

}  // namespace interbotix_ux_ros_control

#endif  // INTERBOTIX_UX_ROS_CONTROL__UX_HARDWARE_INTERFACE_OBJ_H_
