#include "interbotix_xs_rviz/interbotix_control_panel.hpp"
#include "ui_interbotix_control_panel.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <pluginlib/class_list_macros.hpp>

#include <chrono>

namespace interbotix_xs_rviz
{

InterbotixControlPanel::InterbotixControlPanel(QWidget * parent)
: rviz_common::Panel(parent),
  ui_(new Ui::InterbotixControlPanelUI())
{
  ui_->setupUi(this);
  node_ = std::make_shared<rclcpp::Node>("interbotix_control_panel");
  spin_timer_.setInterval(50);
  connect(&spin_timer_, &QTimer::timeout, this, &InterbotixControlPanel::spin_some);
  spin_timer_.start();

  connect(ui_->pushbutton_robot_namespace_, &QPushButton::clicked, this, &InterbotixControlPanel::update_robot_namespace);
  connect(ui_->combobox_torque_name_, &QComboBox::currentTextChanged, this, &InterbotixControlPanel::torque_change_name);
  connect(ui_->button_torque_enable_, &QPushButton::clicked, this, &InterbotixControlPanel::torque_enable_torque);
  connect(ui_->button_torque_disable_, &QPushButton::clicked, this, &InterbotixControlPanel::torque_disable_torque);
  connect(ui_->radiobutton_torque_group_, &QRadioButton::toggled, this, &InterbotixControlPanel::torque_change_cmd_type_group);
  connect(ui_->radiobutton_torque_single_, &QRadioButton::toggled, this, &InterbotixControlPanel::torque_change_cmd_type_single);
  connect(ui_->button_gotohome_, &QPushButton::clicked, this, &InterbotixControlPanel::homesleep_go_to_home);
  connect(ui_->button_gotosleep_, &QPushButton::clicked, this, &InterbotixControlPanel::homesleep_go_to_sleep);
  connect(ui_->combobox_reboot_name_, &QComboBox::currentTextChanged, this, &InterbotixControlPanel::reboot_change_name);
  connect(ui_->radiobutton_reboot_group_, &QRadioButton::toggled, this, &InterbotixControlPanel::reboot_change_cmd_type_group);
  connect(ui_->radiobutton_reboot_single_, &QRadioButton::toggled, this, &InterbotixControlPanel::reboot_change_cmd_type_single);
  connect(ui_->checkbox_smart_reboot_, &QCheckBox::toggled, this, &InterbotixControlPanel::reboot_change_smartreboot);
  connect(ui_->checkbox_reboot_enable_, &QCheckBox::toggled, this, &InterbotixControlPanel::reboot_change_enable);
  connect(ui_->button_reboot_reboot_, &QPushButton::clicked, this, &InterbotixControlPanel::send_reboot_call);
  connect(ui_->combobox_opmodes_name_, &QComboBox::currentTextChanged, this, &InterbotixControlPanel::opmodes_change_name);
  connect(ui_->radiobutton_opmodes_group_, &QRadioButton::toggled, this, &InterbotixControlPanel::opmodes_change_cmd_type_group);
  connect(ui_->radiobutton_opmodes_single_, &QRadioButton::toggled, this, &InterbotixControlPanel::opmodes_change_cmd_type_single);
  connect(ui_->combobox_opmodes_mode_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &InterbotixControlPanel::opmodes_change_mode);
  connect(ui_->combobox_opmodes_profile_type_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &InterbotixControlPanel::opmodes_change_profile_type);
  connect(ui_->lineedit_opmodes_profile_vel_, &QLineEdit::editingFinished, this, &InterbotixControlPanel::opmodes_change_profile_vel);
  connect(ui_->lineedit_opmodes_profile_acc_, &QLineEdit::editingFinished, this, &InterbotixControlPanel::opmodes_change_profile_acc);
  connect(ui_->button_opmodes_set_, &QPushButton::clicked, this, &InterbotixControlPanel::send_opmodes_call);
  connect(ui_->combobox_getregval_name_, &QComboBox::currentTextChanged, this, &InterbotixControlPanel::getregval_change_name);
  connect(ui_->radiobutton_getregval_group_, &QRadioButton::toggled, this, &InterbotixControlPanel::getregval_change_cmd_type_group);
  connect(ui_->radiobutton_getregval_single_, &QRadioButton::toggled, this, &InterbotixControlPanel::getregval_change_cmd_type_single);
  connect(ui_->combobox_getregval_reg_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &InterbotixControlPanel::getregval_change_reg_name);
  connect(ui_->button_getregval_val_, &QPushButton::clicked, this, &InterbotixControlPanel::send_getregval_call);
  connect(ui_->button_estop_, &QPushButton::clicked, this, &InterbotixControlPanel::estop_button_pressed);

  torque_init();
  homesleep_init();
  reboot_init();
  opmodes_init();
  getregval_init();
}

InterbotixControlPanel::~InterbotixControlPanel()
{
  delete ui_;
}

void InterbotixControlPanel::spin_some()
{
  if (node_) rclcpp::spin_some(node_);
}

void InterbotixControlPanel::update_robot_namespace()
{
  if (set_robot_namespace(ui_->lineedit_robot_namespace_->text())) {
    enable_elements(true);
    update_robot_info();
    Q_EMIT configChanged();
  } else {
    enable_elements(false);
  }
}

bool InterbotixControlPanel::set_robot_namespace(const QString & robot_namespace)
{
  robot_namespace_ = robot_namespace.toStdString();
  if (robot_namespace_.empty()) return false;
  std::string prefix = "/" + robot_namespace_;
  srv_torque_enable_ = node_->create_client<interbotix_xs_msgs::srv::TorqueEnable>(prefix + "/torque_enable");
  srv_operating_modes_ = node_->create_client<interbotix_xs_msgs::srv::OperatingModes>(prefix + "/set_operating_modes");
  srv_reboot_motors_ = node_->create_client<interbotix_xs_msgs::srv::Reboot>(prefix + "/reboot_motors");
  srv_get_motor_registers_ = node_->create_client<interbotix_xs_msgs::srv::RegisterValues>(prefix + "/get_motor_registers");
  srv_robot_info_ = node_->create_client<interbotix_xs_msgs::srv::RobotInfo>(prefix + "/get_robot_info");
  pub_joint_group_cmd_ = node_->create_publisher<interbotix_xs_msgs::msg::JointGroupCommand>(prefix + "/commands/joint_group", 1);

  auto timeout = loaded_ ? std::chrono::seconds(2) : std::chrono::milliseconds(100);
  if (!srv_torque_enable_->wait_for_service(timeout)) {
    return false;
  }
  return true;
}

void InterbotixControlPanel::update_robot_info()
{
  auto request = std::make_shared<interbotix_xs_msgs::srv::RobotInfo::Request>();
  request->cmd_type = "group";
  request->name = "all";
  auto result = srv_robot_info_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node_, result, std::chrono::seconds(2)) != rclcpp::FutureReturnCode::SUCCESS)
    return;
  robot_info_resp_ = result.get();
  if (!robot_info_resp_) return;

  homesleep_homevec_.resize(robot_info_resp_->num_joints, 0.0f);
  homesleep_sleepvec_ = robot_info_resp_->joint_sleep_positions;

  robot_arm_joints_.clear();
  qrobot_arm_joints_.clear();
  robot_groups_.clear();
  qrobot_groups_.clear();
  for (const auto & n : robot_info_resp_->joint_names) {
    robot_arm_joints_.push_back(n);
    qrobot_arm_joints_ << QString::fromStdString(n);
  }
  for (const auto & n : robot_info_resp_->name) {
    robot_groups_.push_back(n);
    qrobot_groups_ << QString::fromStdString(n);
  }
}

void InterbotixControlPanel::enable_elements(bool enable)
{
  ui_->button_torque_enable_->setEnabled(enable);
  ui_->button_torque_disable_->setEnabled(enable);
  ui_->radiobutton_torque_single_->setEnabled(enable);
  ui_->radiobutton_torque_group_->setEnabled(enable);
  ui_->combobox_torque_name_->setEnabled(enable);
  ui_->button_gotohome_->setEnabled(enable);
  ui_->button_gotosleep_->setEnabled(enable);
  ui_->radiobutton_reboot_single_->setEnabled(enable);
  ui_->radiobutton_reboot_group_->setEnabled(enable);
  ui_->button_reboot_reboot_->setEnabled(enable);
  ui_->checkbox_smart_reboot_->setEnabled(enable);
  ui_->checkbox_reboot_enable_->setEnabled(enable);
  ui_->combobox_reboot_name_->setEnabled(enable);
  ui_->radiobutton_opmodes_single_->setEnabled(enable);
  ui_->radiobutton_opmodes_group_->setEnabled(enable);
  ui_->combobox_opmodes_mode_->setEnabled(enable);
  ui_->combobox_opmodes_profile_type_->setEnabled(enable);
  ui_->lineedit_opmodes_profile_vel_->setEnabled(enable);
  ui_->lineedit_opmodes_profile_acc_->setEnabled(enable);
  ui_->button_opmodes_set_->setEnabled(enable);
  ui_->combobox_opmodes_name_->setEnabled(enable);
  ui_->radiobutton_getregval_single_->setEnabled(enable);
  ui_->radiobutton_getregval_group_->setEnabled(enable);
  ui_->combobox_getregval_reg_->setEnabled(enable);
  ui_->button_getregval_val_->setEnabled(enable);
  ui_->lineedit_getregval_val_->setEnabled(enable);
  ui_->combobox_getregval_name_->setEnabled(enable);
  ui_->button_estop_->setEnabled(enable);
}

void InterbotixControlPanel::torque_init() { }
void InterbotixControlPanel::torque_change_cmd_type_group()
{
  if (ui_->radiobutton_torque_group_->isChecked()) {
    ui_->combobox_torque_name_->clear();
    ui_->combobox_torque_name_->addItems(qrobot_groups_);
  }
}
void InterbotixControlPanel::torque_change_cmd_type_single()
{
  if (ui_->radiobutton_torque_single_->isChecked()) {
    ui_->combobox_torque_name_->clear();
    ui_->combobox_torque_name_->addItems(qrobot_arm_joints_);
  }
}
void InterbotixControlPanel::torque_change_name() { }
void InterbotixControlPanel::torque_enable_torque() { send_torque_enable_call(true); }
void InterbotixControlPanel::torque_disable_torque() { send_torque_enable_call(false); }

void InterbotixControlPanel::send_torque_enable_call(bool enable)
{
  auto req = std::make_shared<interbotix_xs_msgs::srv::TorqueEnable::Request>();
  req->cmd_type = ui_->radiobutton_torque_group_->isChecked() ? "group" : "single";
  req->name = ui_->combobox_torque_name_->currentText().toStdString();
  req->enable = enable;
  srv_torque_enable_->async_send_request(req);
}

void InterbotixControlPanel::homesleep_init() { }
void InterbotixControlPanel::homesleep_go_to_home()
{
  interbotix_xs_msgs::msg::JointGroupCommand msg;
  msg.name = "all";
  msg.cmd = homesleep_homevec_;
  pub_joint_group_cmd_->publish(msg);
}
void InterbotixControlPanel::homesleep_go_to_sleep()
{
  interbotix_xs_msgs::msg::JointGroupCommand msg;
  msg.name = "all";
  msg.cmd = homesleep_sleepvec_;
  pub_joint_group_cmd_->publish(msg);
}

void InterbotixControlPanel::reboot_init() { }
void InterbotixControlPanel::reboot_change_cmd_type_group()
{
  if (ui_->radiobutton_reboot_group_->isChecked()) {
    ui_->combobox_reboot_name_->clear();
    ui_->combobox_reboot_name_->addItems(qrobot_groups_);
  }
}
void InterbotixControlPanel::reboot_change_cmd_type_single()
{
  if (ui_->radiobutton_reboot_single_->isChecked()) {
    ui_->combobox_reboot_name_->clear();
    ui_->combobox_reboot_name_->addItems(qrobot_arm_joints_);
  }
}
void InterbotixControlPanel::reboot_change_name() { }
void InterbotixControlPanel::reboot_change_smartreboot(bool) { }
void InterbotixControlPanel::reboot_change_enable(bool) { }
void InterbotixControlPanel::send_reboot_call()
{
  auto req = std::make_shared<interbotix_xs_msgs::srv::Reboot::Request>();
  req->cmd_type = ui_->radiobutton_reboot_group_->isChecked() ? "group" : "single";
  req->name = ui_->combobox_reboot_name_->currentText().toStdString();
  req->smart_reboot = ui_->checkbox_smart_reboot_->isChecked();
  req->enable = ui_->checkbox_reboot_enable_->isChecked();
  srv_reboot_motors_->async_send_request(req);
}

void InterbotixControlPanel::opmodes_init() { }
void InterbotixControlPanel::opmodes_change_cmd_type_group()
{
  if (ui_->radiobutton_opmodes_group_->isChecked()) {
    ui_->combobox_opmodes_name_->clear();
    ui_->combobox_opmodes_name_->addItems(qrobot_groups_);
  }
}
void InterbotixControlPanel::opmodes_change_cmd_type_single()
{
  if (ui_->radiobutton_opmodes_single_->isChecked()) {
    ui_->combobox_opmodes_name_->clear();
    ui_->combobox_opmodes_name_->addItems(qrobot_arm_joints_);
  }
}
void InterbotixControlPanel::opmodes_change_name() { }
void InterbotixControlPanel::opmodes_change_mode(int) { }
void InterbotixControlPanel::opmodes_change_profile_type(int) { }
void InterbotixControlPanel::opmodes_change_profile_vel() { }
void InterbotixControlPanel::opmodes_change_profile_acc() { }
void InterbotixControlPanel::send_opmodes_call()
{
  auto req = std::make_shared<interbotix_xs_msgs::srv::OperatingModes::Request>();
  req->cmd_type = ui_->radiobutton_opmodes_group_->isChecked() ? "group" : "single";
  req->name = ui_->combobox_opmodes_name_->currentText().toStdString();
  req->mode = ui_->combobox_opmodes_mode_->currentText().toStdString();
  req->profile_type = ui_->combobox_opmodes_profile_type_->currentText().toStdString();
  req->profile_velocity = static_cast<int32_t>(ui_->lineedit_opmodes_profile_vel_->text().toInt());
  req->profile_acceleration = static_cast<int32_t>(ui_->lineedit_opmodes_profile_acc_->text().toInt());
  srv_operating_modes_->async_send_request(req);
}

void InterbotixControlPanel::getregval_init()
{
  ui_->label_getregval_desc_->setText(QString::fromStdString(
    xs_register_descriptions::descriptions.at("Operating_Mode").description));
}
void InterbotixControlPanel::getregval_change_cmd_type_group()
{
  if (ui_->radiobutton_getregval_group_->isChecked()) {
    ui_->combobox_getregval_name_->clear();
    ui_->combobox_getregval_name_->addItems(qrobot_groups_);
  }
}
void InterbotixControlPanel::getregval_change_cmd_type_single()
{
  if (ui_->radiobutton_getregval_single_->isChecked()) {
    ui_->combobox_getregval_name_->clear();
    ui_->combobox_getregval_name_->addItems(qrobot_arm_joints_);
  }
}
void InterbotixControlPanel::getregval_change_name() { }
void InterbotixControlPanel::getregval_change_reg_name(int)
{
  std::string reg = ui_->combobox_getregval_reg_->currentText().toStdString();
  auto it = xs_register_descriptions::descriptions.find(reg);
  if (it != xs_register_descriptions::descriptions.end())
    ui_->label_getregval_desc_->setText(QString::fromStdString(it->second.description));
}
void InterbotixControlPanel::send_getregval_call()
{
  auto req = std::make_shared<interbotix_xs_msgs::srv::RegisterValues::Request>();
  req->cmd_type = ui_->radiobutton_getregval_group_->isChecked() ? "group" : "single";
  req->name = ui_->combobox_getregval_name_->currentText().toStdString();
  req->reg = ui_->combobox_getregval_reg_->currentText().toStdString();
  auto result = srv_get_motor_registers_->async_send_request(req);
  if (rclcpp::spin_until_future_complete(node_, result, std::chrono::seconds(2)) == rclcpp::FutureReturnCode::SUCCESS &&
      result.get()) {
    getregval_display(*result.get());
  }
}
void InterbotixControlPanel::getregval_display(const interbotix_xs_msgs::srv::RegisterValues::Response & resp)
{
  std::stringstream s;
  for (size_t i = 0; i < resp.values.size(); ++i) {
    if (i > 0) s << ",";
    s << resp.values[i];
  }
  ui_->lineedit_getregval_val_->setText(QString::fromStdString(s.str()));
}

void InterbotixControlPanel::estop_button_pressed() { send_system_call(); }
void InterbotixControlPanel::send_system_call()
{
  (void)robot_namespace_;
  RCLCPP_WARN(node_->get_logger(), "E-stop: implement driver shutdown for ROS 2 (e.g. lifecycle or service)");
}

void InterbotixControlPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("RobotNameSpace", QString::fromStdString(robot_namespace_));
}

void InterbotixControlPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
  QString ns;
  if (config.mapGetString("RobotNameSpace", &ns)) {
    ui_->lineedit_robot_namespace_->setText(ns);
    set_robot_namespace(ns);
  }
  loaded_ = true;
}

}  // namespace interbotix_xs_rviz

PLUGINLIB_EXPORT_CLASS(interbotix_xs_rviz::InterbotixControlPanel, rviz_common::Panel)
