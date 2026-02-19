/* Copyright 2018 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jason Peng <jason@ufactory.cc>
 ============================================================================*/
#include <xarm_driver.h>
#include "xarm/instruction/uxbus_cmd_config.h"
#include "xarm/linux/thread.h"
#include <thread>

#define CMD_HEARTBEAT_US 30000000  // 30s

void* cmd_heart_beat(void* args)
{
    xarm_api::XARMDriver *my_driver = (xarm_api::XARMDriver *) args;
    while(true)
    {
        usleep(CMD_HEARTBEAT_US);
        my_driver->Heartbeat();
    }
    pthread_exit(0);
}

namespace xarm_api
{
    XARMDriver::XARMDriver(rclcpp::Node* node) : node_(node), dof_(7), curr_state_(0), curr_err_(0)
    {
        rx_data_.resize(1280);
    }

    XARMDriver::~XARMDriver()
    {
        arm_cmd_->set_mode(XARM_MODE::POSE);
        arm_cmd_->close();
    }

    void XARMDriver::XARMDriverInit(char *server_ip)
    {
        node_->declare_parameter<int>("DOF", 7);
        node_->get_parameter("DOF", dof_);

        arm_report_ = connext_tcp_report_norm(server_ip);
        arm_cmd_ = connect_tcp_control(server_ip);
        if (arm_cmd_ == NULL)
            RCLCPP_ERROR(node_->get_logger(), "Xarm Connection Failed!");
        else
        {
            int dbg_msg[16] = {0};
            arm_cmd_->servo_get_dbmsg(dbg_msg);

            for(int i=0; i<dof_; i++)
            {
                if((dbg_msg[i*2]==1)&&(dbg_msg[i*2+1]==40))
                {
                    arm_cmd_->clean_err();
                    RCLCPP_WARN(node_->get_logger(), "Cleared low-voltage error of joint %d", i+1);
                }
                else if((dbg_msg[i*2]==1))
                {
                    arm_cmd_->clean_err();
                    RCLCPP_WARN(node_->get_logger(), "There is servo error code:(0x%x) in joint %d, trying to clear it..", dbg_msg[i*2+1], i+1);
                }
            }
        }

        motion_ctrl_server_ = node_->create_service<xarm_msgs::srv::SetAxis>("motion_ctrl", std::bind(&XARMDriver::MotionCtrlCB, this, std::placeholders::_1, std::placeholders::_2));
        set_mode_server_ = node_->create_service<xarm_msgs::srv::SetInt16>("set_mode", std::bind(&XARMDriver::SetModeCB, this, std::placeholders::_1, std::placeholders::_2));
        set_state_server_ = node_->create_service<xarm_msgs::srv::SetInt16>("set_state", std::bind(&XARMDriver::SetStateCB, this, std::placeholders::_1, std::placeholders::_2));
        set_tcp_offset_server_ = node_->create_service<xarm_msgs::srv::TCPOffset>("set_tcp_offset", std::bind(&XARMDriver::SetTCPOffsetCB, this, std::placeholders::_1, std::placeholders::_2));
        set_load_server_ = node_->create_service<xarm_msgs::srv::SetLoad>("set_load", std::bind(&XARMDriver::SetLoadCB, this, std::placeholders::_1, std::placeholders::_2));

        go_home_server_ = node_->create_service<xarm_msgs::srv::Move>("go_home", std::bind(&XARMDriver::GoHomeCB, this, std::placeholders::_1, std::placeholders::_2));
        move_joint_server_ = node_->create_service<xarm_msgs::srv::Move>("move_joint", std::bind(&XARMDriver::MoveJointCB, this, std::placeholders::_1, std::placeholders::_2));
        move_lineb_server_ = node_->create_service<xarm_msgs::srv::Move>("move_lineb", std::bind(&XARMDriver::MoveLinebCB, this, std::placeholders::_1, std::placeholders::_2));
        move_line_server_ = node_->create_service<xarm_msgs::srv::Move>("move_line", std::bind(&XARMDriver::MoveLineCB, this, std::placeholders::_1, std::placeholders::_2));
        move_line_tool_server_ = node_->create_service<xarm_msgs::srv::Move>("move_line_tool", std::bind(&XARMDriver::MoveLineToolCB, this, std::placeholders::_1, std::placeholders::_2));
        move_servoj_server_ = node_->create_service<xarm_msgs::srv::Move>("move_servoj", std::bind(&XARMDriver::MoveServoJCB, this, std::placeholders::_1, std::placeholders::_2));
        move_servo_cart_server_ = node_->create_service<xarm_msgs::srv::Move>("move_servo_cart", std::bind(&XARMDriver::MoveServoCartCB, this, std::placeholders::_1, std::placeholders::_2));
        clear_err_server_ = node_->create_service<xarm_msgs::srv::ClearErr>("clear_err", std::bind(&XARMDriver::ClearErrCB, this, std::placeholders::_1, std::placeholders::_2));
        moveit_clear_err_server_ = node_->create_service<xarm_msgs::srv::ClearErr>("moveit_clear_err", std::bind(&XARMDriver::MoveitClearErrCB, this, std::placeholders::_1, std::placeholders::_2));
        get_err_server_ = node_->create_service<xarm_msgs::srv::GetErr>("get_err", std::bind(&XARMDriver::GetErrCB, this, std::placeholders::_1, std::placeholders::_2));

        set_end_io_server_ = node_->create_service<xarm_msgs::srv::SetDigitalIO>("set_digital_out", std::bind(&XARMDriver::SetDigitalIOCB, this, std::placeholders::_1, std::placeholders::_2));
        get_digital_in_server_ = node_->create_service<xarm_msgs::srv::GetDigitalIO>("get_digital_in", std::bind(&XARMDriver::GetDigitalIOCB, this, std::placeholders::_1, std::placeholders::_2));
        get_analog_in_server_ = node_->create_service<xarm_msgs::srv::GetAnalogIO>("get_analog_in", std::bind(&XARMDriver::GetAnalogIOCB, this, std::placeholders::_1, std::placeholders::_2));
        config_modbus_server_ = node_->create_service<xarm_msgs::srv::ConfigToolModbus>("config_tool_modbus", std::bind(&XARMDriver::ConfigModbusCB, this, std::placeholders::_1, std::placeholders::_2));
        set_modbus_server_ = node_->create_service<xarm_msgs::srv::SetToolModbus>("set_tool_modbus", std::bind(&XARMDriver::SetModbusCB, this, std::placeholders::_1, std::placeholders::_2));

        gripper_config_server_ = node_->create_service<xarm_msgs::srv::GripperConfig>("gripper_config", std::bind(&XARMDriver::GripperConfigCB, this, std::placeholders::_1, std::placeholders::_2));
        gripper_move_server_ = node_->create_service<xarm_msgs::srv::GripperMove>("gripper_move", std::bind(&XARMDriver::GripperMoveCB, this, std::placeholders::_1, std::placeholders::_2));
        gripper_state_server_ = node_->create_service<xarm_msgs::srv::GripperState>("gripper_state", std::bind(&XARMDriver::GripperStateCB, this, std::placeholders::_1, std::placeholders::_2));
        set_vacuum_gripper_server_ = node_->create_service<xarm_msgs::srv::SetInt16>("vacuum_gripper_set", std::bind(&XARMDriver::VacuumGripperCB, this, std::placeholders::_1, std::placeholders::_2));

        set_controller_dout_server_ = node_->create_service<xarm_msgs::srv::SetDigitalIO>("set_controller_dout", std::bind(&XARMDriver::SetControllerDOutCB, this, std::placeholders::_1, std::placeholders::_2));
        get_controller_din_server_ = node_->create_service<xarm_msgs::srv::GetControllerDigitalIO>("get_controller_din", std::bind(&XARMDriver::GetControllerDInCB, this, std::placeholders::_1, std::placeholders::_2));

        rclcpp::QoS qos(10);
        qos.transient_local();
        joint_state_ = node_->create_publisher<sensor_msgs::msg::JointState>("joint_states", qos);
        robot_rt_state_ = node_->create_publisher<xarm_msgs::msg::RobotMsg>("xarm_states", qos);

        sleep_sub_ = node_->create_subscription<std_msgs::msg::Float32>("sleep_sec", 1, std::bind(&XARMDriver::SleepTopicCB, this, std::placeholders::_1));
    }

    void XARMDriver::Heartbeat(void)
    {
        int cmd_num;
        arm_cmd_->get_cmdnum(&cmd_num);
    }

    bool XARMDriver::isConnectionOK(void)
    {
        return !arm_report_->is_ok();
    }

    void XARMDriver::SleepTopicCB(const std_msgs::msg::Float32::SharedPtr msg)
    {
        if(msg->data > 0)
            arm_cmd_->sleep_instruction(msg->data);
    }

    bool XARMDriver::ClearErrCB(const std::shared_ptr<xarm_msgs::srv::ClearErr::Request> req, std::shared_ptr<xarm_msgs::srv::ClearErr::Response> res)
    {
        (void)req;
        arm_cmd_->gripper_modbus_clean_err();
        arm_cmd_->clean_war();
        arm_cmd_->clean_err();
        res->ret = arm_cmd_->motion_en(8, 1);
        if(res->ret)
            res->message = "clear err, ret = " + std::to_string(res->ret);
        return true;
    }

    bool XARMDriver::MoveitClearErrCB(const std::shared_ptr<xarm_msgs::srv::ClearErr::Request> req, std::shared_ptr<xarm_msgs::srv::ClearErr::Response> res)
    {
        if(ClearErrCB(req, res))
        {
            arm_cmd_->set_mode(1);
            int ret = arm_cmd_->set_state(0);
            return (ret == 0);
        }
        return false;
    }

    bool XARMDriver::GetErrCB(const std::shared_ptr<xarm_msgs::srv::GetErr::Request> req, std::shared_ptr<xarm_msgs::srv::GetErr::Response> res)
    {
        (void)req;
        res->err = curr_err_;
        res->message = "current error code = " + std::to_string(res->err);
        return true;
    }

    bool XARMDriver::MotionCtrlCB(const std::shared_ptr<xarm_msgs::srv::SetAxis::Request> req, std::shared_ptr<xarm_msgs::srv::SetAxis::Response> res)
    {
        res->ret = arm_cmd_->motion_en(req->id, req->data);
        res->message = (req->data == 1) ? "motion enable, ret = " + std::to_string(res->ret) : "motion disable, ret = " + std::to_string(res->ret);
        return true;
    }

    bool XARMDriver::SetModeCB(const std::shared_ptr<xarm_msgs::srv::SetInt16::Request> req, std::shared_ptr<xarm_msgs::srv::SetInt16::Response> res)
    {
        res->ret = arm_cmd_->set_mode(req->data);
        switch(req->data)
        {
            case XARM_MODE::POSE: res->message = "pose mode, ret = " + std::to_string(res->ret); break;
            case XARM_MODE::SERVO: res->message = "servo mode, ret = " + std::to_string(res->ret); break;
            case XARM_MODE::TEACH_CART: res->message = "cartesian teach, ret = " + std::to_string(res->ret); break;
            case XARM_MODE::TEACH_JOINT: res->message = "joint teach , ret = " + std::to_string(res->ret); break;
            default: res->message = "the failed mode, ret = " + std::to_string(res->ret);
        }
        return true;
    }

    bool XARMDriver::SetStateCB(const std::shared_ptr<xarm_msgs::srv::SetInt16::Request> req, std::shared_ptr<xarm_msgs::srv::SetInt16::Response> res)
    {
        res->ret = arm_cmd_->set_state(req->data);
        switch(req->data)
        {
            case XARM_STATE::START: res->message = "start, ret = " + std::to_string(res->ret); break;
            case XARM_STATE::PAUSE: res->message = "pause, ret = " + std::to_string(res->ret); break;
            case XARM_STATE::STOP: res->message = "stop, ret = " + std::to_string(res->ret); break;
            default: res->message = "the failed state, ret = " + std::to_string(res->ret);
        }
        return true;
    }

    bool XARMDriver::SetTCPOffsetCB(const std::shared_ptr<xarm_msgs::srv::TCPOffset::Request> req, std::shared_ptr<xarm_msgs::srv::TCPOffset::Response> res)
    {
        float offsets[6] = {req->x, req->y, req->z, req->roll, req->pitch, req->yaw};
        res->ret = arm_cmd_->set_tcp_offset(offsets) | arm_cmd_->save_conf();
        res->message = "set tcp offset: ret = " + std::to_string(res->ret);
        return true;
    }

    bool XARMDriver::SetLoadCB(const std::shared_ptr<xarm_msgs::srv::SetLoad::Request> req, std::shared_ptr<xarm_msgs::srv::SetLoad::Response> res)
    {
        float CoM[3] = {req->xc, req->yc, req->zc};
        res->ret = arm_cmd_->set_tcp_load(req->mass, CoM) | arm_cmd_->save_conf();
        res->message = "set load: ret = " + std::to_string(res->ret);
        return true;
    }

    bool XARMDriver::SetControllerDOutCB(const std::shared_ptr<xarm_msgs::srv::SetDigitalIO::Request> req, std::shared_ptr<xarm_msgs::srv::SetDigitalIO::Response> res)
    {
        if(req->io_num >= 1 && req->io_num <= 8)
        {
            res->ret = arm_cmd_->cgpio_set_auxdigit(req->io_num - 1, req->value);
            res->message = "set Controller digital Output " + std::to_string(req->io_num) + " to " + std::to_string(req->value) + " : ret = " + std::to_string(res->ret);
            return true;
        }
        RCLCPP_WARN(node_->get_logger(), "Controller IO io_num: from 1 to 8");
        return false;
    }

    bool XARMDriver::GetControllerDInCB(const std::shared_ptr<xarm_msgs::srv::GetControllerDigitalIO::Request> req, std::shared_ptr<xarm_msgs::srv::GetControllerDigitalIO::Response> res)
    {
        if(req->io_num >= 1 && req->io_num <= 8)
        {
            int all_status;
            res->ret = arm_cmd_->cgpio_get_auxdigit(&all_status);
            res->value = (all_status >> (req->io_num - 1)) & 0x0001;
            res->message = "get Controller digital Input ret = " + std::to_string(res->ret);
            return true;
        }
        RCLCPP_WARN(node_->get_logger(), "Controller IO io_num: from 1 to 8");
        return false;
    }

    bool XARMDriver::SetDigitalIOCB(const std::shared_ptr<xarm_msgs::srv::SetDigitalIO::Request> req, std::shared_ptr<xarm_msgs::srv::SetDigitalIO::Response> res)
    {
        res->ret = arm_cmd_->tgpio_set_digital(req->io_num, req->value);
        res->message = "set Digital port " + std::to_string(req->io_num) + " to " + std::to_string(req->value) + " : ret = " + std::to_string(res->ret);
        return true;
    }

    bool XARMDriver::GetDigitalIOCB(const std::shared_ptr<xarm_msgs::srv::GetDigitalIO::Request> req, std::shared_ptr<xarm_msgs::srv::GetDigitalIO::Response> res)
    {
        (void)req;
        res->ret = arm_cmd_->tgpio_get_digital(&res->digital_1, &res->digital_2);
        res->message = "get Digital port ret = " + std::to_string(res->ret);
        return true;
    }

    bool XARMDriver::GetAnalogIOCB(const std::shared_ptr<xarm_msgs::srv::GetAnalogIO::Request> req, std::shared_ptr<xarm_msgs::srv::GetAnalogIO::Response> res)
    {
        switch (req->port_num)
        {
            case 1: res->ret = arm_cmd_->tgpio_get_analog1(&res->analog_value); break;
            case 2: res->ret = arm_cmd_->tgpio_get_analog2(&res->analog_value); break;
            default:
                res->message = "GetAnalogIO Fail: port number incorrect !";
                return false;
        }
        res->message = "get analog port ret = " + std::to_string(res->ret);
        return true;
    }

    bool XARMDriver::SetModbusCB(const std::shared_ptr<xarm_msgs::srv::SetToolModbus::Request> req, std::shared_ptr<xarm_msgs::srv::SetToolModbus::Response> res)
    {
        int send_len = static_cast<int>(req->send_data.size());
        int recv_len = req->respond_len;
        std::vector<unsigned char> tx_data(req->send_data.begin(), req->send_data.end());
        std::vector<unsigned char> rx_data(recv_len, 0);
        res->ret = arm_cmd_->tgpio_set_modbus(tx_data.data(), send_len, rx_data.data());
        res->respond_data.assign(rx_data.begin(), rx_data.end());
        return true;
    }

    bool XARMDriver::ConfigModbusCB(const std::shared_ptr<xarm_msgs::srv::ConfigToolModbus::Request> req, std::shared_ptr<xarm_msgs::srv::ConfigToolModbus::Response> res)
    {
        res->message = "";
        if(curr_err_)
        {
            arm_cmd_->set_state(0);
            RCLCPP_WARN(node_->get_logger(), "Cleared Existing Error Code %d", curr_err_);
        }
        int ret = arm_cmd_->set_modbus_baudrate(req->baud_rate);
        if(ret)
        {
            res->message = "set baud_rate, ret = " + std::to_string(ret);
            if(ret == 1) res->message += " controller error exists, please check and run clear_err service first!";
        }
        ret = arm_cmd_->set_modbus_timeout(req->timeout_ms);
        if(ret)
        {
            res->message += (std::string(" set timeout, ret = ") + std::to_string(ret));
            if(ret == 1) res->message += " controller error exists, please check and run clear_err service first!";
        }
        res->ret = res->message.empty() ? 0 : -1;
        if(res->message.empty()) res->message = "Modbus configuration OK";
        return true;
    }

    bool XARMDriver::GoHomeCB(const std::shared_ptr<xarm_msgs::srv::Move::Request> req, std::shared_ptr<xarm_msgs::srv::Move::Response> res)
    {
        res->ret = arm_cmd_->move_gohome(req->mvvelo, req->mvacc, req->mvtime);
        if(!res->ret) res->ret = wait_for_finish();
        res->message = "go home, ret = " + std::to_string(res->ret);
        return true;
    }

    bool XARMDriver::MoveJointCB(const std::shared_ptr<xarm_msgs::srv::Move::Request> req, std::shared_ptr<xarm_msgs::srv::Move::Response> res)
    {
        float joint[1][7] = {0};
        if(static_cast<int>(req->pose.size()) != dof_)
        {
            res->ret = static_cast<int>(req->pose.size());
            res->message = "pose parameters incorrect! Expected: " + std::to_string(dof_);
            return true;
        }
        for(int index = 0; index < 7; index++)
            joint[0][index] = (index < static_cast<int>(req->pose.size())) ? req->pose[index] : 0.0f;

        res->ret = arm_cmd_->move_joint(joint[0], req->mvvelo, req->mvacc, req->mvtime);
        if(!res->ret) res->ret = wait_for_finish();
        res->message = "move joint, ret = " + std::to_string(res->ret);
        return true;
    }

    bool XARMDriver::MoveLineCB(const std::shared_ptr<xarm_msgs::srv::Move::Request> req, std::shared_ptr<xarm_msgs::srv::Move::Response> res)
    {
        if(req->pose.size() != 6) { res->ret = -1; res->message = "parameters incorrect!"; return true; }
        float pose[6];
        for(int i = 0; i < 6; i++) pose[i] = req->pose[i];
        res->ret = arm_cmd_->move_line(pose, req->mvvelo, req->mvacc, req->mvtime);
        if(!res->ret) res->ret = wait_for_finish();
        res->message = "move line, ret = " + std::to_string(res->ret);
        return true;
    }

    bool XARMDriver::MoveLineToolCB(const std::shared_ptr<xarm_msgs::srv::Move::Request> req, std::shared_ptr<xarm_msgs::srv::Move::Response> res)
    {
        if(req->pose.size() != 6) { res->ret = -1; res->message = "parameters incorrect!"; return true; }
        float pose[6];
        for(int i = 0; i < 6; i++) pose[i] = req->pose[i];
        res->ret = arm_cmd_->move_line_tool(pose, req->mvvelo, req->mvacc, req->mvtime);
        if(!res->ret) res->ret = wait_for_finish();
        res->message = "move line tool, ret = " + std::to_string(res->ret);
        return true;
    }

    bool XARMDriver::MoveLinebCB(const std::shared_ptr<xarm_msgs::srv::Move::Request> req, std::shared_ptr<xarm_msgs::srv::Move::Response> res)
    {
        if(req->pose.size() != 6) { res->ret = -1; res->message = "parameters incorrect!"; return true; }
        float pose[6];
        for(int i = 0; i < 6; i++) pose[i] = req->pose[i];
        res->ret = arm_cmd_->move_lineb(pose, req->mvvelo, req->mvacc, req->mvtime, req->mvradii);
        res->message = "move lineb, ret = " + std::to_string(res->ret);
        return true;
    }

    bool XARMDriver::MoveServoJCB(const std::shared_ptr<xarm_msgs::srv::Move::Request> req, std::shared_ptr<xarm_msgs::srv::Move::Response> res)
    {
        float pose[1][7] = {0};
        if(static_cast<int>(req->pose.size()) != dof_) { res->ret = static_cast<int>(req->pose.size()); res->message = "pose parameters incorrect! Expected: " + std::to_string(dof_); return true; }
        for(int i = 0; i < 7; i++) pose[0][i] = (i < static_cast<int>(req->pose.size())) ? req->pose[i] : 0.0f;
        res->ret = arm_cmd_->move_servoj(pose[0], req->mvvelo, req->mvacc, req->mvtime);
        res->message = "move servoj, ret = " + std::to_string(res->ret);
        return true;
    }

    bool XARMDriver::MoveServoCartCB(const std::shared_ptr<xarm_msgs::srv::Move::Request> req, std::shared_ptr<xarm_msgs::srv::Move::Response> res)
    {
        if(req->pose.size() != 6) { res->ret = -1; res->message = "MoveServoCartCB parameters incorrect!"; return true; }
        float pose[6];
        for(int i = 0; i < 6; i++) pose[i] = req->pose[i];
        res->ret = arm_cmd_->move_servo_cartesian(pose, req->mvvelo, req->mvacc, req->mvtime);
        res->message = "move servo_cartesian, ret = " + std::to_string(res->ret);
        return true;
    }

    bool XARMDriver::GripperConfigCB(const std::shared_ptr<xarm_msgs::srv::GripperConfig::Request> req, std::shared_ptr<xarm_msgs::srv::GripperConfig::Response> res)
    {
        float pulse_vel = req->pulse_vel;
        if(pulse_vel > 5000) pulse_vel = 5000;
        else if(pulse_vel < 0) pulse_vel = 0;
        int ret1 = arm_cmd_->gripper_modbus_set_mode(0);
        int ret2 = arm_cmd_->gripper_modbus_set_en(1);
        int ret3 = arm_cmd_->gripper_modbus_set_posspd(static_cast<int>(pulse_vel));
        res->ret = (ret1 || ret2 || ret3) ? ret3 : 0;
        res->message = "gripper_config, ret = " + std::to_string(res->ret);
        return true;
    }

    bool XARMDriver::GripperMoveCB(const std::shared_ptr<xarm_msgs::srv::GripperMove::Request> req, std::shared_ptr<xarm_msgs::srv::GripperMove::Response> res)
    {
        float pulse_pos = req->pulse_pos;
        if(pulse_pos > 850) pulse_pos = 850;
        else if(pulse_pos < -100) pulse_pos = -100;
        res->ret = arm_cmd_->gripper_modbus_set_pos(static_cast<int>(pulse_pos));
        res->message = "gripper_move, ret = " + std::to_string(res->ret);
        return true;
    }

    bool XARMDriver::GripperStateCB(const std::shared_ptr<xarm_msgs::srv::GripperState::Request> req, std::shared_ptr<xarm_msgs::srv::GripperState::Response> res)
    {
        (void)req;
        int err_code = 0;
        float pos_now = 0;
        if(arm_cmd_->gripper_modbus_get_errcode(&err_code)) return false;
        if(arm_cmd_->gripper_modbus_get_pos(&pos_now)) return false;
        res->err_code = err_code;
        res->curr_pos = pos_now;
        return true;
    }

    bool XARMDriver::VacuumGripperCB(const std::shared_ptr<xarm_msgs::srv::SetInt16::Request> req, std::shared_ptr<xarm_msgs::srv::SetInt16::Response> res)
    {
        if(req->data)
        {
            res->ret = arm_cmd_->tgpio_set_digital(1, 1);
            arm_cmd_->tgpio_set_digital(2, 0);
        }
        else
        {
            res->ret = arm_cmd_->tgpio_set_digital(1, 0);
            arm_cmd_->tgpio_set_digital(2, 1);
        }
        res->message = "set vacuum gripper: " + std::to_string(req->data) + " ret = " + std::to_string(res->ret);
        return true;
    }

    void XARMDriver::pub_robot_msg(const xarm_msgs::msg::RobotMsg& rm_msg)
    {
        curr_err_ = rm_msg.err;
        curr_state_ = rm_msg.state;
        robot_rt_state_->publish(rm_msg);
    }

    void XARMDriver::pub_joint_state(const sensor_msgs::msg::JointState& js_msg)
    {
        joint_state_->publish(js_msg);
    }

    void XARMDriver::pub_io_state()
    {
        arm_cmd_->tgpio_get_digital(&io_msg.digital_1, &io_msg.digital_2);
        arm_cmd_->tgpio_get_analog1(&io_msg.analog_1);
        arm_cmd_->tgpio_get_analog2(&io_msg.analog_2);
        if(end_input_state_) end_input_state_->publish(io_msg);
    }

    int XARMDriver::get_frame(void)
    {
        return arm_report_->read_frame(rx_data_.data());
    }

    int XARMDriver::get_rich_data(ReportDataNorm &norm_data)
    {
        int ret = norm_data_.flush_data(rx_data_.data());
        norm_data = norm_data_;
        return ret;
    }

    int XARMDriver::wait_for_finish()
    {
        bool wait = false;
        node_->declare_parameter<bool>("wait_for_finish", false);
        node_->get_parameter("wait_for_finish", wait);
        if(!wait) return 0;

        rclcpp::sleep_for(std::chrono::milliseconds(200));
        const auto period = std::chrono::milliseconds(100);
        while(curr_state_ == 1)
        {
            if(curr_err_) return UXBUS_STATE::ERR_CODE;
            std::this_thread::sleep_for(period);
        }
        int err_warn[2] = {0};
        arm_cmd_->get_err_code(err_warn);
        if(err_warn[0])
        {
            RCLCPP_ERROR(node_->get_logger(), "XARM ERROR CODE: %d ", err_warn[0]);
            return UXBUS_STATE::ERR_CODE;
        }
        return 0;
    }
}
