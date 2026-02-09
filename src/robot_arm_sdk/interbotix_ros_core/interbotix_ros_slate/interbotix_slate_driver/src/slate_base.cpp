#include "interbotix_slate_driver/slate_base.h"
#include <cmath>

namespace slate_base
{

SlateBase::SlateBase(rclcpp::Node * node_handle)
  : node_(node_handle),
    cmd_vel_x_(0.0f),
    cmd_vel_z_(0.0f),
    cnt_(0),
    x_(0.0f),
    y_(0.0f),
    theta_(0.0f),
    x_vel_(0.0f),
    z_omega_(0.0f),
    is_first_odom_(true),
    pose_{0.0f, 0.0f, 0.0f},
    right_motor_c_(0.0f),
    left_motor_c_(0.0f),
    chassis_state_(SystemState::SYS_INIT),
    publish_tf_(false),
    max_vel_x_(1.0f),
    max_vel_z_(1.0f),
    current_time_(node_->get_clock()->now()),
    last_time_(node_->get_clock()->now()),
    cmd_vel_timeout_(0, static_cast<int64_t>(CMD_TIME_OUT * 1e6))
{
  node_->declare_parameter<bool>("publish_tf", false);
  node_->declare_parameter<std::string>("odom_frame_name", "odom");
  node_->declare_parameter<std::string>("base_frame_name", "base_link");

  node_->get_parameter("publish_tf", publish_tf_);
  node_->get_parameter("odom_frame_name", odom_frame_name_);
  node_->get_parameter("base_frame_name", base_frame_name_);

  pub_odom_ = node_->create_publisher<nav_msgs::msg::Odometry>("odom", 1);
  pub_battery_state_ = node_->create_publisher<sensor_msgs::msg::BatteryState>("battery_state", 1);

  sub_cmd_vel_ = node_->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 1, std::bind(&SlateBase::cmd_vel_callback, this, std::placeholders::_1));

  srv_info_ = node_->create_service<interbotix_slate_msgs::srv::SetString>(
    "set_text", std::bind(&SlateBase::set_text_callback, this, std::placeholders::_1, std::placeholders::_2));
  srv_motor_torque_status_ = node_->create_service<std_srvs::srv::SetBool>(
    "set_motor_torque_status", std::bind(&SlateBase::motor_torque_status_callback, this, std::placeholders::_1, std::placeholders::_2));
  srv_enable_charing_ = node_->create_service<std_srvs::srv::SetBool>(
    "enable_charging", std::bind(&SlateBase::enable_charing_callback, this, std::placeholders::_1, std::placeholders::_2));

  tf_broadcaster_odom_ = std::make_unique<tf2_ros::TransformBroadcaster>(*node_);

  std::string dev;
  if (!base_driver::chassisInit(dev)) {
    RCLCPP_FATAL(node_->get_logger(), "Failed to initialize base.");
    std::exit(EXIT_FAILURE);
  }
  RCLCPP_INFO(node_->get_logger(), "Initialized base at port '%s'.", dev.c_str());
  char version[32] = {0};
  if (base_driver::getVersion(version)) {
    RCLCPP_INFO(node_->get_logger(), "Base version: %s", version);
  }
}

void SlateBase::update()
{
  current_time_ = node_->get_clock()->now();

  cnt_++;
  sensor_msgs::msg::BatteryState state;
  if (cnt_ % 10 == 0) {
    state.header.stamp = current_time_;
    int percentage = 0;
    if (
      base_driver::getBatteryInfo(state.voltage, state.current, percentage) &&
      base_driver::getChassisState(chassis_state_))
    {
      if (state.current < 0) {
        int c = static_cast<int>(-state.current);
        int right_c = (c % 1000);
        int left_c = (c - right_c) / 1000;
        right_motor_c_ = right_c * 0.1f;
        left_motor_c_ = left_c * 0.1f;
        state.current = -(right_motor_c_ + left_motor_c_);
      }
      state.percentage = static_cast<float>(percentage);
      state.power_supply_status = static_cast<int8_t>(chassis_state_);
      pub_battery_state_->publish(state);
    }
  }

  if ((current_time_ - cmd_vel_time_last_update_).nanoseconds() > cmd_vel_timeout_.nanoseconds()) {
    cmd_vel_x_ = 0.0f;
    cmd_vel_z_ = 0.0f;
  }

  cmd_vel_x_ = std::min(max_vel_x_, std::max(-max_vel_x_, cmd_vel_x_));
  cmd_vel_z_ = std::min(max_vel_z_, std::max(-max_vel_z_, cmd_vel_z_));

  base_driver::getChassisInfo(x_vel_, z_omega_);
  base_driver::getChassisOdom(x_, y_, theta_);
  base_driver::chassisControl(cmd_vel_x_, cmd_vel_z_);

  if (is_first_odom_) {
    pose_[0] = x_;
    pose_[1] = y_;
    pose_[2] = theta_;
    is_first_odom_ = false;
  }

  geometry_msgs::msg::Quaternion odom_quat = tf2::toMsg(
    tf2::Quaternion(tf2::Vector3(0, 0, 1), wrap_angle(theta_ - pose_[2])));

  geometry_msgs::msg::TransformStamped odom_trans;
  odom_trans.header.stamp = current_time_;
  odom_trans.header.frame_id = odom_frame_name_;
  odom_trans.child_frame_id = base_frame_name_;

  odom_trans.transform.translation.x =
      cos(-pose_[2]) * (x_ - pose_[0]) - sin(-pose_[2]) * (y_ - pose_[1]);
  odom_trans.transform.translation.y =
      sin(-pose_[2]) * (x_ - pose_[0]) + cos(-pose_[2]) * (y_ - pose_[1]);
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = odom_quat;

  if (publish_tf_) {
    tf_broadcaster_odom_->sendTransform(odom_trans);
  }

  nav_msgs::msg::Odometry odom;
  odom.header.stamp = current_time_;
  odom.header.frame_id = odom_frame_name_;

  odom.pose.pose.position.x = odom_trans.transform.translation.x;
  odom.pose.pose.position.y = odom_trans.transform.translation.y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;
  odom.pose.covariance[0] = (chassis_state_ == SystemState::SYS_ESTOP) ? -1.0 : 1.0;

  odom.child_frame_id = base_frame_name_;
  odom.twist.twist.linear.x = x_vel_;
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.angular.z = z_omega_;

  pub_odom_->publish(odom);
  last_time_ = current_time_;
}

void SlateBase::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  cmd_vel_x_ = msg->linear.x;
  cmd_vel_z_ = msg->angular.z;
  cmd_vel_time_last_update_ = node_->get_clock()->now();
}

void SlateBase::set_text_callback(
  const std::shared_ptr<interbotix_slate_msgs::srv::SetString::Request> req,
  std::shared_ptr<interbotix_slate_msgs::srv::SetString::Response> res)
{
  res->success = base_driver::setText(req->data.c_str());
  if (res->success) {
    res->message = "Successfully set text to: '" + req->data + "'.";
    RCLCPP_INFO(node_->get_logger(), "%s", res->message.c_str());
  } else {
    res->message = "Failed to set text to: '" + req->data + "'.";
    RCLCPP_ERROR(node_->get_logger(), "%s", res->message.c_str());
  }
}

void SlateBase::motor_torque_status_callback(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
  std::shared_ptr<std_srvs::srv::SetBool::Response> res)
{
  res->success = base_driver::motorCtrl(!req->data);
  std::string enabled_disabled = req->data ? "enable" : "disable";
  if (res->success) {
    res->message = "Successfully " + enabled_disabled + "d motor torque.";
    RCLCPP_INFO(node_->get_logger(), "%s", res->message.c_str());
  } else {
    res->message = "Failed to " + enabled_disabled + " motor torque.";
    RCLCPP_ERROR(node_->get_logger(), "%s", res->message.c_str());
  }
}

void SlateBase::enable_charing_callback(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
  std::shared_ptr<std_srvs::srv::SetBool::Response> res)
{
  res->success = base_driver::setCharge(req->data);
  std::string enabled_disabled = req->data ? "enable" : "disable";
  if (res->success) {
    res->message = "Successfully " + enabled_disabled + "d charging.";
    RCLCPP_INFO(node_->get_logger(), "%s", res->message.c_str());
  } else {
    res->message = "Failed to " + enabled_disabled + " charging.";
    RCLCPP_ERROR(node_->get_logger(), "%s", res->message.c_str());
  }
}

float SlateBase::wrap_angle(float angle)
{
  const float pi = static_cast<float>(M_PI);
  if (angle > pi) {
    angle = angle - 2.0f * pi;
  } else if (angle < -pi) {
    angle = angle + 2.0f * pi;
  }
  return angle;
}

}  // namespace slate_base
