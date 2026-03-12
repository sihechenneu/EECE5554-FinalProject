#ifndef XS_SDK_OBJ_H_
#define XS_SDK_OBJ_H_

#include <rclcpp/rclcpp.hpp>
#include <urdf/model.h>
#include <unordered_map>
#include <yaml-cpp/yaml.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>
#include <interbotix_xs_msgs/srv/reboot.hpp>
#include <interbotix_xs_msgs/srv/robot_info.hpp>
#include <interbotix_xs_msgs/srv/motor_gains.hpp>
#include <interbotix_xs_msgs/srv/torque_enable.hpp>
#include <interbotix_xs_msgs/srv/operating_modes.hpp>
#include <interbotix_xs_msgs/srv/register_values.hpp>
#include <interbotix_xs_msgs/msg/joint_group_command.hpp>
#include <interbotix_xs_msgs/msg/joint_single_command.hpp>
#include <interbotix_xs_msgs/msg/joint_trajectory_command.hpp>

#define BAUDRATE 1000000
#define PORT "/dev/ttyDXL"
#define SYNC_WRITE_HANDLER_FOR_GOAL_POSITION 0
#define SYNC_WRITE_HANDLER_FOR_GOAL_VELOCITY 1
#define SYNC_WRITE_HANDLER_FOR_GOAL_CURRENT 2
#define SYNC_WRITE_HANDLER_FOR_GOAL_PWM 3
#define SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT 0

static const std::string OP_MODE = "position";
static const std::string PROFILE_TYPE = "velocity";
static const int32_t PROFILE_VELOCITY = 0;
static const int32_t PROFILE_ACCELERATION = 0;
static const bool TORQUE_ENABLE = true;

struct JointGroup
{
  std::vector<std::string> joint_names;
  std::vector<uint8_t> joint_ids;
  uint8_t joint_num;
  std::string mode;
  std::string profile_type;
  int32_t profile_velocity;
  int32_t profile_acceleration;
};

struct MotorState
{
  uint8_t motor_id;
  std::string mode;
  std::string profile_type;
  int32_t profile_velocity;
  int32_t profile_acceleration;
};

struct Gripper
{
  size_t js_index;
  float horn_radius;
  float arm_length;
  std::string left_finger;
  std::string right_finger;
};

struct MotorInfo
{
  uint8_t motor_id;
  std::string reg;
  int32_t value;
};

enum ReadFailureBehavior
{
  PUB_DXL_WB = 0,
  PUB_NAN = 1
};

class InterbotixRobotXS
{
public:
  explicit InterbotixRobotXS(rclcpp::Node* node_handle, bool& success);

  ~InterbotixRobotXS();

  void robot_set_operating_modes(std::string const& cmd_type, std::string const& name, std::string const& mode, std::string const& profile_type = PROFILE_TYPE, int32_t profile_velocity = PROFILE_VELOCITY, int32_t profile_acceleration = PROFILE_ACCELERATION);
  void robot_set_joint_operating_mode(std::string const& name, std::string const& mode, std::string const& profile_type = PROFILE_TYPE, int32_t profile_velocity = PROFILE_VELOCITY, int32_t profile_acceleration = PROFILE_ACCELERATION);
  void robot_torque_enable(std::string const& cmd_type, std::string const& name, bool const& enable);
  void robot_reboot_motors(std::string const& cmd_type, std::string const& name, bool const& enable, bool const& smart_reboot);
  void robot_write_commands(std::string const& name, std::vector<float> commands);
  void robot_write_joint_command(std::string const& name, float command);
  void robot_set_motor_pid_gains(std::string const& cmd_type, std::string const& name, std::vector<int32_t> const& gains);
  void robot_set_motor_registers(std::string const& cmd_type, std::string const& name, std::string const& reg, int32_t const& value);
  void robot_get_motor_registers(std::string const& cmd_type, std::string const& name, std::string const& reg, std::vector<int32_t>& values);
  void robot_get_joint_states(std::string const& name, std::vector<float>* positions = nullptr, std::vector<float>* velocities = nullptr, std::vector<float>* effort = nullptr);
  void robot_get_joint_state(std::string const& name, float* position = nullptr, float* velocity = nullptr, float* effort = nullptr);
  float robot_convert_linear_position_to_radian(std::string const& name, float const& linear_position);
  float robot_convert_angular_position_to_linear(std::string const& name, float const& angular_position);

private:
  int timer_hz;
  bool pub_states;
  bool execute_joint_traj;
  bool load_configs = true;
  JointGroup* all_ptr;
  DynamixelWorkbench dxl_wb;
  YAML::Node motor_configs;
  YAML::Node mode_configs;

  rclcpp::Node* node_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_states_;
  rclcpp::Subscription<interbotix_xs_msgs::msg::JointGroupCommand>::SharedPtr sub_command_group_;
  rclcpp::Subscription<interbotix_xs_msgs::msg::JointSingleCommand>::SharedPtr sub_command_single_;
  rclcpp::Subscription<interbotix_xs_msgs::msg::JointTrajectoryCommand>::SharedPtr sub_command_traj_;
  rclcpp::Service<interbotix_xs_msgs::srv::TorqueEnable>::SharedPtr srv_torque_enable_;
  rclcpp::Service<interbotix_xs_msgs::srv::Reboot>::SharedPtr srv_reboot_motors_;
  rclcpp::Service<interbotix_xs_msgs::srv::RobotInfo>::SharedPtr srv_get_robot_info_;
  rclcpp::Service<interbotix_xs_msgs::srv::OperatingModes>::SharedPtr srv_operating_modes_;
  rclcpp::Service<interbotix_xs_msgs::srv::MotorGains>::SharedPtr srv_motor_gains_;
  rclcpp::Service<interbotix_xs_msgs::srv::RegisterValues>::SharedPtr srv_set_registers_;
  rclcpp::Service<interbotix_xs_msgs::srv::RegisterValues>::SharedPtr srv_get_registers_;
  rclcpp::TimerBase::SharedPtr tmr_joint_states_;
  rclcpp::TimerBase::SharedPtr tmr_joint_traj_;
  sensor_msgs::msg::JointState joint_states;
  interbotix_xs_msgs::msg::JointTrajectoryCommand joint_traj_cmd;

  std::string port;
  std::string js_topic;
  std::vector<MotorInfo> motor_info_vec;
  std::vector<std::string> gripper_order;
  std::unordered_map<std::string, const ControlItem*> control_items;
  std::unordered_map<std::string, float> sleep_map;
  std::unordered_map<std::string, JointGroup> group_map;
  std::unordered_map<std::string, MotorState> motor_map;
  std::unordered_map<std::string, std::vector<std::string>> shadow_map;
  std::unordered_map<std::string, std::vector<std::string>> sister_map;
  std::unordered_map<std::string, Gripper> gripper_map;
  std::unordered_map<std::string, size_t> js_index_map;

  ReadFailureBehavior read_failure_behavior{ReadFailureBehavior::PUB_DXL_WB};

  bool robot_get_motor_configs(void);
  bool robot_init_port(void);
  bool robot_ping_motors(void);
  bool robot_load_motor_configs(void);
  void robot_init_controlItems(void);
  void robot_init_SDK_handlers(void);
  void robot_init_operating_modes(void);
  void robot_init_publishers(void);
  void robot_init_subscribers(void);
  void robot_init_services(void);
  void robot_init_timers(void);
  void robot_wait_for_joint_states(void);

  void robot_sub_command_group(const interbotix_xs_msgs::msg::JointGroupCommand::SharedPtr msg);
  void robot_sub_command_single(const interbotix_xs_msgs::msg::JointSingleCommand::SharedPtr msg);
  void robot_sub_command_traj(const interbotix_xs_msgs::msg::JointTrajectoryCommand::SharedPtr msg);

  bool robot_srv_torque_enable(const std::shared_ptr<interbotix_xs_msgs::srv::TorqueEnable::Request> req, std::shared_ptr<interbotix_xs_msgs::srv::TorqueEnable::Response> res);
  bool robot_srv_reboot_motors(const std::shared_ptr<interbotix_xs_msgs::srv::Reboot::Request> req, std::shared_ptr<interbotix_xs_msgs::srv::Reboot::Response> res);
  bool robot_srv_get_robot_info(const std::shared_ptr<interbotix_xs_msgs::srv::RobotInfo::Request> req, std::shared_ptr<interbotix_xs_msgs::srv::RobotInfo::Response> res);
  bool robot_srv_set_operating_modes(const std::shared_ptr<interbotix_xs_msgs::srv::OperatingModes::Request> req, std::shared_ptr<interbotix_xs_msgs::srv::OperatingModes::Response> res);
  bool robot_srv_set_motor_pid_gains(const std::shared_ptr<interbotix_xs_msgs::srv::MotorGains::Request> req, std::shared_ptr<interbotix_xs_msgs::srv::MotorGains::Response> res);
  bool robot_srv_set_motor_registers(const std::shared_ptr<interbotix_xs_msgs::srv::RegisterValues::Request> req, std::shared_ptr<interbotix_xs_msgs::srv::RegisterValues::Response> res);
  bool robot_srv_get_motor_registers(const std::shared_ptr<interbotix_xs_msgs::srv::RegisterValues::Request> req, std::shared_ptr<interbotix_xs_msgs::srv::RegisterValues::Response> res);

  bool robot_srv_validate(const std::string& cmd_type, std::string& name);

  void robot_execute_trajectory();
  void robot_update_joint_states();
};

#endif
