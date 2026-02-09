#ifndef INTERBOTIX_SLATE_DRIVER__SLATE_BASE_HPP_
#define INTERBOTIX_SLATE_DRIVER__SLATE_BASE_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "interbotix_slate_driver/base_driver.h"
#include "interbotix_slate_driver/serial_driver.h"
#include "interbotix_slate_msgs/srv/set_string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace slate_base
{

#define CMD_TIME_OUT 300  // ms
#define PORT "chassis"

class SlateBase
{
public:

  /**
   * @brief Constructor for the SlateBase
   * @param node ROS 2 node
   */
  explicit SlateBase(rclcpp::Node * node);

  /// @brief Destructor for the SlateBase
  ~SlateBase() = default;

  /// @brief Process velocity commands and update robot state
  void update();

private:
  rclcpp::Node * node_;

  float cmd_vel_x_;
  float cmd_vel_z_;
  rclcpp::Time cmd_vel_time_last_update_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr pub_battery_state_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;

  rclcpp::Service<interbotix_slate_msgs::srv::SetString>::SharedPtr srv_info_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_motor_torque_status_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_enable_charing_;

  std::string odom_frame_name_;
  std::string base_frame_name_;

  int cnt_;
  float x_;
  float y_;
  float theta_;
  float x_vel_;
  float z_omega_;
  bool is_first_odom_;
  float pose_[3];
  float right_motor_c_;
  float left_motor_c_;
  SystemState chassis_state_;
  bool publish_tf_;
  float max_vel_x_ = 1.0f;
  float max_vel_z_ = 1.0f;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_odom_;

  rclcpp::Time current_time_;
  rclcpp::Time last_time_;
  rclcpp::Duration cmd_vel_timeout_;

  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

  void set_text_callback(
    const std::shared_ptr<interbotix_slate_msgs::srv::SetString::Request> req,
    std::shared_ptr<interbotix_slate_msgs::srv::SetString::Response> res);

  void motor_torque_status_callback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
    std::shared_ptr<std_srvs::srv::SetBool::Response> res);

  void enable_charing_callback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
    std::shared_ptr<std_srvs::srv::SetBool::Response> res);

  float wrap_angle(float angle);
};

}  // namespace slate_base

#endif  // INTERBOTIX_SLATE_DRIVER__SLATE_BASE_HPP_
