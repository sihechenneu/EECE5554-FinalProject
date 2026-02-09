#ifndef __XARM_ROS_CLIENT_H__
#define __XARM_ROS_CLIENT_H__

#include <rclcpp/rclcpp.hpp>
#include <xarm_msgs/srv/set_axis.hpp>
#include <xarm_msgs/srv/set_int16.hpp>
#include <xarm_msgs/srv/tcp_offset.hpp>
#include <xarm_msgs/srv/set_load.hpp>
#include <xarm_msgs/srv/move.hpp>
#include <xarm_msgs/srv/clear_err.hpp>
#include <xarm_msgs/srv/get_err.hpp>
#include <xarm_msgs/srv/gripper_config.hpp>
#include <xarm_msgs/srv/gripper_move.hpp>
#include <xarm_msgs/srv/gripper_state.hpp>
#include <xarm_msgs/srv/config_tool_modbus.hpp>
#include <xarm_msgs/srv/set_tool_modbus.hpp>

namespace xarm_api {

class XArmROSClient
{
public:
    XArmROSClient() = default;
    void init(rclcpp::Node::SharedPtr node);
    ~XArmROSClient() = default;

    int motionEnable(short en);
    int setState(short state);
    int setMode(short mode);
    int clearErr(void);
    int getErr(void);
    int setTCPOffset(const std::vector<float>& tcp_offset);
    int setLoad(float mass, const std::vector<float>& center_of_mass);
    int setServoJ(const std::vector<float>& joint_cmd);
    int setServoCartisian(const std::vector<float>& cart_cmd);
    int goHome(float jnt_vel_rad, float jnt_acc_rad = 15);
    int moveJoint(const std::vector<float>& joint_cmd, float jnt_vel_rad, float jnt_acc_rad = 15);
    int moveLine(const std::vector<float>& cart_cmd, float cart_vel_mm, float cart_acc_mm = 500);
    int moveLineB(int num_of_pnts, const std::vector<float> cart_cmds[], float cart_vel_mm, float cart_acc_mm = 500, float radii = 0);
    int getGripperState(float *curr_pulse, int *curr_err);
    int gripperConfig(float pulse_vel);
    int gripperMove(float pulse);
    int config_tool_modbus(int baud_rate, int time_out_ms);
    int send_tool_modbus(unsigned char* data, int send_len, unsigned char* recv_data = nullptr, int recv_len = 0);

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<xarm_msgs::srv::SetAxis>::SharedPtr motion_ctrl_client_;
    rclcpp::Client<xarm_msgs::srv::SetInt16>::SharedPtr set_mode_client_;
    rclcpp::Client<xarm_msgs::srv::SetInt16>::SharedPtr set_state_client_;
    rclcpp::Client<xarm_msgs::srv::Move>::SharedPtr go_home_client_;
    rclcpp::Client<xarm_msgs::srv::Move>::SharedPtr move_lineb_client_;
    rclcpp::Client<xarm_msgs::srv::Move>::SharedPtr move_servoj_client_;
    rclcpp::Client<xarm_msgs::srv::Move>::SharedPtr move_servo_cart_client_;
    rclcpp::Client<xarm_msgs::srv::Move>::SharedPtr move_line_client_;
    rclcpp::Client<xarm_msgs::srv::Move>::SharedPtr move_joint_client_;
    rclcpp::Client<xarm_msgs::srv::TCPOffset>::SharedPtr set_tcp_offset_client_;
    rclcpp::Client<xarm_msgs::srv::SetLoad>::SharedPtr set_load_client_;
    rclcpp::Client<xarm_msgs::srv::ClearErr>::SharedPtr clear_err_client_;
    rclcpp::Client<xarm_msgs::srv::GetErr>::SharedPtr get_err_client_;
    rclcpp::Client<xarm_msgs::srv::ConfigToolModbus>::SharedPtr config_modbus_client_;
    rclcpp::Client<xarm_msgs::srv::SetToolModbus>::SharedPtr send_modbus_client_;
    rclcpp::Client<xarm_msgs::srv::GripperMove>::SharedPtr gripper_move_client_;
    rclcpp::Client<xarm_msgs::srv::GripperConfig>::SharedPtr gripper_config_client_;
    rclcpp::Client<xarm_msgs::srv::GripperState>::SharedPtr gripper_state_client_;
};

}

#endif
