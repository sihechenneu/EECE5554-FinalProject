#pragma once

#ifndef Q_MOC_RUN
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#endif

#include <QLineEdit>
#include <QPushButton>
#include <QTimer>

#include <interbotix_xs_msgs/srv/reboot.hpp>
#include <interbotix_xs_msgs/srv/robot_info.hpp>
#include <interbotix_xs_msgs/srv/torque_enable.hpp>
#include <interbotix_xs_msgs/srv/operating_modes.hpp>
#include <interbotix_xs_msgs/srv/register_values.hpp>
#include <interbotix_xs_msgs/msg/joint_group_command.hpp>
#include "interbotix_xs_rviz/xs_register_descriptions.h"

namespace Ui { class InterbotixControlPanelUI; }

namespace interbotix_xs_rviz
{

class InterbotixControlPanel : public rviz_common::Panel
{
  Q_OBJECT
public:
  explicit InterbotixControlPanel(QWidget * parent = nullptr);
  ~InterbotixControlPanel() override;

  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

public Q_SLOTS:
  bool set_robot_namespace(const QString & robot_namespace);
  void send_torque_enable_call(bool enable);
  void send_home_sleep_call();
  void send_reboot_call();
  void send_opmodes_call();
  void send_getregval_call();

protected Q_SLOTS:
  void update_robot_namespace();
  void update_robot_info();
  void torque_change_cmd_type_single();
  void torque_change_cmd_type_group();
  void torque_change_name();
  void torque_enable_torque();
  void torque_disable_torque();
  void torque_init();
  void homesleep_go_to_home();
  void homesleep_go_to_sleep();
  void homesleep_init();
  void reboot_change_cmd_type_single();
  void reboot_change_cmd_type_group();
  void reboot_change_name();
  void reboot_change_smartreboot(bool checked);
  void reboot_change_enable(bool checked);
  void reboot_init();
  void opmodes_change_cmd_type_group();
  void opmodes_change_cmd_type_single();
  void opmodes_change_name();
  void opmodes_change_mode(int);
  void opmodes_change_profile_type(int);
  void opmodes_change_profile_vel();
  void opmodes_change_profile_acc();
  void opmodes_init();
  void getregval_change_cmd_type_group();
  void getregval_change_cmd_type_single();
  void getregval_change_name();
  void getregval_change_reg_name(int);
  void getregval_display(const interbotix_xs_msgs::srv::RegisterValues::Response & resp);
  void getregval_init();
  void estop_button_pressed();
  void spin_some();

protected:
  Ui::InterbotixControlPanelUI * ui_;
  rclcpp::Node::SharedPtr node_;
  QTimer spin_timer_;
  std::string robot_namespace_;
  interbotix_xs_msgs::srv::RobotInfo::Response::SharedPtr robot_info_resp_;
  std::vector<std::string> robot_arm_joints_;
  QStringList qrobot_arm_joints_;
  std::vector<std::string> robot_groups_;
  QStringList qrobot_groups_;

  rclcpp::Client<interbotix_xs_msgs::srv::TorqueEnable>::SharedPtr srv_torque_enable_;
  rclcpp::Client<interbotix_xs_msgs::srv::OperatingModes>::SharedPtr srv_operating_modes_;
  rclcpp::Client<interbotix_xs_msgs::srv::Reboot>::SharedPtr srv_reboot_motors_;
  rclcpp::Client<interbotix_xs_msgs::srv::RegisterValues>::SharedPtr srv_get_motor_registers_;
  rclcpp::Client<interbotix_xs_msgs::srv::RobotInfo>::SharedPtr srv_robot_info_;
  rclcpp::Publisher<interbotix_xs_msgs::msg::JointGroupCommand>::SharedPtr pub_joint_group_cmd_;

  std::vector<float> homesleep_homevec_;
  std::vector<float> homesleep_sleepvec_;
  bool loaded_ = false;

  void enable_elements(bool enable);
  void send_system_call();
};

}  // namespace interbotix_xs_rviz
