#ifndef __XARM_DRIVER_H
#define __XARM_DRIVER_H

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <xarm_msgs/srv/set_int16.hpp>
#include <xarm_msgs/srv/tcp_offset.hpp>
#include <xarm_msgs/srv/set_load.hpp>
#include <xarm_msgs/srv/set_axis.hpp>
#include <xarm_msgs/srv/move.hpp>
#include <xarm_msgs/msg/robot_msg.hpp>
#include <xarm_msgs/msg/io_state.hpp>
#include <xarm_msgs/srv/set_digital_io.hpp>
#include <xarm_msgs/srv/get_digital_io.hpp>
#include <xarm_msgs/srv/get_controller_digital_io.hpp>
#include <xarm_msgs/srv/get_analog_io.hpp>
#include <xarm_msgs/srv/clear_err.hpp>
#include <xarm_msgs/srv/get_err.hpp>
#include <xarm_msgs/srv/gripper_config.hpp>
#include <xarm_msgs/srv/gripper_move.hpp>
#include <xarm_msgs/srv/gripper_state.hpp>
#include <xarm_msgs/srv/set_tool_modbus.hpp>
#include <xarm_msgs/srv/config_tool_modbus.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <xarm/common/data_type.h>
#include <xarm/linux/thread.h>
#include "xarm/connect.h"
#include "xarm/report_data.h"

namespace xarm_api
{
    class XARMDriver
    {
        public:
            XARMDriver(rclcpp::Node* node);
            ~XARMDriver();
            void XARMDriverInit(char *server_ip);
            void Heartbeat(void);
            bool isConnectionOK(void);

            bool MotionCtrlCB(const std::shared_ptr<xarm_msgs::srv::SetAxis::Request> req, std::shared_ptr<xarm_msgs::srv::SetAxis::Response> res);
            bool SetModeCB(const std::shared_ptr<xarm_msgs::srv::SetInt16::Request> req, std::shared_ptr<xarm_msgs::srv::SetInt16::Response> res);
            bool SetStateCB(const std::shared_ptr<xarm_msgs::srv::SetInt16::Request> req, std::shared_ptr<xarm_msgs::srv::SetInt16::Response> res);
            bool SetTCPOffsetCB(const std::shared_ptr<xarm_msgs::srv::TCPOffset::Request> req, std::shared_ptr<xarm_msgs::srv::TCPOffset::Response> res);
            bool SetLoadCB(const std::shared_ptr<xarm_msgs::srv::SetLoad::Request> req, std::shared_ptr<xarm_msgs::srv::SetLoad::Response> res);
            bool SetDigitalIOCB(const std::shared_ptr<xarm_msgs::srv::SetDigitalIO::Request> req, std::shared_ptr<xarm_msgs::srv::SetDigitalIO::Response> res);
            bool GetDigitalIOCB(const std::shared_ptr<xarm_msgs::srv::GetDigitalIO::Request> req, std::shared_ptr<xarm_msgs::srv::GetDigitalIO::Response> res);
            bool GetAnalogIOCB(const std::shared_ptr<xarm_msgs::srv::GetAnalogIO::Request> req, std::shared_ptr<xarm_msgs::srv::GetAnalogIO::Response> res);
            bool ClearErrCB(const std::shared_ptr<xarm_msgs::srv::ClearErr::Request> req, std::shared_ptr<xarm_msgs::srv::ClearErr::Response> res);
            bool MoveitClearErrCB(const std::shared_ptr<xarm_msgs::srv::ClearErr::Request> req, std::shared_ptr<xarm_msgs::srv::ClearErr::Response> res);
            bool GetErrCB(const std::shared_ptr<xarm_msgs::srv::GetErr::Request> req, std::shared_ptr<xarm_msgs::srv::GetErr::Response> res);
            bool GoHomeCB(const std::shared_ptr<xarm_msgs::srv::Move::Request> req, std::shared_ptr<xarm_msgs::srv::Move::Response> res);
            bool MoveJointCB(const std::shared_ptr<xarm_msgs::srv::Move::Request> req, std::shared_ptr<xarm_msgs::srv::Move::Response> res);
            bool MoveLinebCB(const std::shared_ptr<xarm_msgs::srv::Move::Request> req, std::shared_ptr<xarm_msgs::srv::Move::Response> res);
            bool MoveLineCB(const std::shared_ptr<xarm_msgs::srv::Move::Request> req, std::shared_ptr<xarm_msgs::srv::Move::Response> res);
            bool MoveLineToolCB(const std::shared_ptr<xarm_msgs::srv::Move::Request> req, std::shared_ptr<xarm_msgs::srv::Move::Response> res);
            bool MoveServoJCB(const std::shared_ptr<xarm_msgs::srv::Move::Request> req, std::shared_ptr<xarm_msgs::srv::Move::Response> res);
            bool MoveServoCartCB(const std::shared_ptr<xarm_msgs::srv::Move::Request> req, std::shared_ptr<xarm_msgs::srv::Move::Response> res);
            bool GripperConfigCB(const std::shared_ptr<xarm_msgs::srv::GripperConfig::Request> req, std::shared_ptr<xarm_msgs::srv::GripperConfig::Response> res);
            bool GripperMoveCB(const std::shared_ptr<xarm_msgs::srv::GripperMove::Request> req, std::shared_ptr<xarm_msgs::srv::GripperMove::Response> res);
            bool GripperStateCB(const std::shared_ptr<xarm_msgs::srv::GripperState::Request> req, std::shared_ptr<xarm_msgs::srv::GripperState::Response> res);
            bool VacuumGripperCB(const std::shared_ptr<xarm_msgs::srv::SetInt16::Request> req, std::shared_ptr<xarm_msgs::srv::SetInt16::Response> res);
            bool SetModbusCB(const std::shared_ptr<xarm_msgs::srv::SetToolModbus::Request> req, std::shared_ptr<xarm_msgs::srv::SetToolModbus::Response> res);
            bool ConfigModbusCB(const std::shared_ptr<xarm_msgs::srv::ConfigToolModbus::Request> req, std::shared_ptr<xarm_msgs::srv::ConfigToolModbus::Response> res);
            bool SetControllerDOutCB(const std::shared_ptr<xarm_msgs::srv::SetDigitalIO::Request> req, std::shared_ptr<xarm_msgs::srv::SetDigitalIO::Response> res);
            bool GetControllerDInCB(const std::shared_ptr<xarm_msgs::srv::GetControllerDigitalIO::Request> req, std::shared_ptr<xarm_msgs::srv::GetControllerDigitalIO::Response> res);
            void SleepTopicCB(const std_msgs::msg::Float32::SharedPtr msg);

            void pub_robot_msg(const xarm_msgs::msg::RobotMsg& rm_msg);
            void pub_joint_state(const sensor_msgs::msg::JointState& js_msg);
            void pub_io_state();

            int get_frame(void);
            int get_rich_data(ReportDataNorm &norm_data);

        private:
            rclcpp::Node* node_;
            SocketPort *arm_report_;
            ReportDataNorm norm_data_;
            UxbusCmd *arm_cmd_;
            std::vector<unsigned char> rx_data_;
            std::string ip;
            pthread_t thread_id_;
            int dof_;
            int curr_state_;
            int curr_err_;
            xarm_msgs::msg::IOState io_msg;

            rclcpp::Service<xarm_msgs::srv::SetAxis>::SharedPtr motion_ctrl_server_;
            rclcpp::Service<xarm_msgs::srv::SetInt16>::SharedPtr set_state_server_;
            rclcpp::Service<xarm_msgs::srv::SetInt16>::SharedPtr set_mode_server_;
            rclcpp::Service<xarm_msgs::srv::Move>::SharedPtr go_home_server_;
            rclcpp::Service<xarm_msgs::srv::Move>::SharedPtr move_joint_server_;
            rclcpp::Service<xarm_msgs::srv::Move>::SharedPtr move_lineb_server_;
            rclcpp::Service<xarm_msgs::srv::Move>::SharedPtr move_line_server_;
            rclcpp::Service<xarm_msgs::srv::Move>::SharedPtr move_line_tool_server_;
            rclcpp::Service<xarm_msgs::srv::Move>::SharedPtr move_servoj_server_;
            rclcpp::Service<xarm_msgs::srv::Move>::SharedPtr move_servo_cart_server_;
            rclcpp::Service<xarm_msgs::srv::TCPOffset>::SharedPtr set_tcp_offset_server_;
            rclcpp::Service<xarm_msgs::srv::SetLoad>::SharedPtr set_load_server_;
            rclcpp::Service<xarm_msgs::srv::SetDigitalIO>::SharedPtr set_end_io_server_;
            rclcpp::Service<xarm_msgs::srv::GetDigitalIO>::SharedPtr get_digital_in_server_;
            rclcpp::Service<xarm_msgs::srv::GetAnalogIO>::SharedPtr get_analog_in_server_;
            rclcpp::Service<xarm_msgs::srv::ClearErr>::SharedPtr clear_err_server_;
            rclcpp::Service<xarm_msgs::srv::ClearErr>::SharedPtr moveit_clear_err_server_;
            rclcpp::Service<xarm_msgs::srv::GetErr>::SharedPtr get_err_server_;
            rclcpp::Service<xarm_msgs::srv::GripperConfig>::SharedPtr gripper_config_server_;
            rclcpp::Service<xarm_msgs::srv::GripperMove>::SharedPtr gripper_move_server_;
            rclcpp::Service<xarm_msgs::srv::GripperState>::SharedPtr gripper_state_server_;
            rclcpp::Service<xarm_msgs::srv::SetInt16>::SharedPtr set_vacuum_gripper_server_;
            rclcpp::Service<xarm_msgs::srv::SetToolModbus>::SharedPtr set_modbus_server_;
            rclcpp::Service<xarm_msgs::srv::ConfigToolModbus>::SharedPtr config_modbus_server_;
            rclcpp::Service<xarm_msgs::srv::SetDigitalIO>::SharedPtr set_controller_dout_server_;
            rclcpp::Service<xarm_msgs::srv::GetControllerDigitalIO>::SharedPtr get_controller_din_server_;

            rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_;
            rclcpp::Publisher<xarm_msgs::msg::RobotMsg>::SharedPtr robot_rt_state_;
            rclcpp::Publisher<xarm_msgs::msg::IOState>::SharedPtr end_input_state_;

            rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sleep_sub_;

            int wait_for_finish();
    };
}

#endif
