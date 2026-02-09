/* Copyright 2018 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jason Peng <jason@ufactory.cc>
 ============================================================================*/
#include <xarm_ros_client.h>
#include <chrono>

namespace xarm_api {

void XArmROSClient::init(rclcpp::Node::SharedPtr node)
{
    node_ = node;
    motion_ctrl_client_ = node_->create_client<xarm_msgs::srv::SetAxis>("motion_ctrl");
    set_mode_client_ = node_->create_client<xarm_msgs::srv::SetInt16>("set_mode");
    set_state_client_ = node_->create_client<xarm_msgs::srv::SetInt16>("set_state");
    set_tcp_offset_client_ = node_->create_client<xarm_msgs::srv::TCPOffset>("set_tcp_offset");
    set_load_client_ = node_->create_client<xarm_msgs::srv::SetLoad>("set_load");
    clear_err_client_ = node_->create_client<xarm_msgs::srv::ClearErr>("clear_err");
    get_err_client_ = node_->create_client<xarm_msgs::srv::GetErr>("get_err");
    go_home_client_ = node_->create_client<xarm_msgs::srv::Move>("go_home");
    move_lineb_client_ = node_->create_client<xarm_msgs::srv::Move>("move_lineb");
    move_line_client_ = node_->create_client<xarm_msgs::srv::Move>("move_line");
    move_joint_client_ = node_->create_client<xarm_msgs::srv::Move>("move_joint");
    move_servoj_client_ = node_->create_client<xarm_msgs::srv::Move>("move_servoj");
    move_servo_cart_client_ = node_->create_client<xarm_msgs::srv::Move>("move_servo_cart");
    gripper_move_client_ = node_->create_client<xarm_msgs::srv::GripperMove>("gripper_move");
    gripper_config_client_ = node_->create_client<xarm_msgs::srv::GripperConfig>("gripper_config");
    gripper_state_client_ = node_->create_client<xarm_msgs::srv::GripperState>("gripper_state");
    config_modbus_client_ = node_->create_client<xarm_msgs::srv::ConfigToolModbus>("config_tool_modbus");
    send_modbus_client_ = node_->create_client<xarm_msgs::srv::SetToolModbus>("set_tool_modbus");

    const auto timeout = std::chrono::seconds(5);
    if (!motion_ctrl_client_->wait_for_service(timeout))
        RCLCPP_ERROR(node_->get_logger(), "motion_ctrl service not available");
    if (!set_state_client_->wait_for_service(timeout))
        RCLCPP_ERROR(node_->get_logger(), "set_state service not available");
    if (!set_mode_client_->wait_for_service(timeout))
        RCLCPP_ERROR(node_->get_logger(), "set_mode service not available");
    if (!move_servoj_client_->wait_for_service(timeout))
        RCLCPP_ERROR(node_->get_logger(), "move_servoj service not available");
}

int XArmROSClient::motionEnable(short en)
{
    auto request = std::make_shared<xarm_msgs::srv::SetAxis::Request>();
    request->id = 8;
    request->data = en;
    auto future = motion_ctrl_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(5)) != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to call service motion_ctrl");
        return 1;
    }
    auto response = future.get();
    RCLCPP_INFO(node_->get_logger(), "%s", response->message.c_str());
    return response->ret;
}

int XArmROSClient::setState(short state)
{
    auto request = std::make_shared<xarm_msgs::srv::SetInt16::Request>();
    request->data = state;
    auto future = set_state_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(5)) != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to call service set_state");
        return 1;
    }
    auto response = future.get();
    RCLCPP_INFO(node_->get_logger(), "%s", response->message.c_str());
    return response->ret;
}

int XArmROSClient::setMode(short mode)
{
    auto request = std::make_shared<xarm_msgs::srv::SetInt16::Request>();
    request->data = mode;
    auto future = set_mode_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(5)) != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to call service set_mode");
        return 1;
    }
    auto response = future.get();
    RCLCPP_INFO(node_->get_logger(), "%s", response->message.c_str());
    return response->ret;
}

int XArmROSClient::clearErr()
{
    auto request = std::make_shared<xarm_msgs::srv::ClearErr::Request>();
    auto future = clear_err_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(5)) != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to call service clear_err");
        return 1;
    }
    auto response = future.get();
    RCLCPP_INFO(node_->get_logger(), "%s", response->message.c_str());
    return response->ret;
}

int XArmROSClient::getErr()
{
    auto request = std::make_shared<xarm_msgs::srv::GetErr::Request>();
    auto future = get_err_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(5)) != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to call service get_err");
        return 1;
    }
    auto response = future.get();
    RCLCPP_INFO(node_->get_logger(), "%s", response->message.c_str());
    return response->err;
}

int XArmROSClient::setServoJ(const std::vector<float>& joint_cmd)
{
    auto request = std::make_shared<xarm_msgs::srv::Move::Request>();
    request->mvvelo = 0;
    request->mvacc = 0;
    request->mvtime = 0;
    request->pose = joint_cmd;
    auto future = move_servoj_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(5)) != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to call service move_servoj");
        return 1;
    }
    return future.get()->ret;
}

int XArmROSClient::setServoCartisian(const std::vector<float>& cart_cmd)
{
    auto request = std::make_shared<xarm_msgs::srv::Move::Request>();
    request->mvvelo = 0;
    request->mvacc = 0;
    request->mvtime = 0;
    request->pose = cart_cmd;
    auto future = move_servo_cart_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(5)) != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to call service move_servo_cart");
        return 1;
    }
    return future.get()->ret;
}

int XArmROSClient::setTCPOffset(const std::vector<float>& tcp_offset)
{
    if (tcp_offset.size() != 6)
    {
        RCLCPP_ERROR(node_->get_logger(), "Set tcp offset service parameter should be 6-element Cartesian offset!");
        return 1;
    }
    auto request = std::make_shared<xarm_msgs::srv::TCPOffset::Request>();
    request->x = tcp_offset[0];
    request->y = tcp_offset[1];
    request->z = tcp_offset[2];
    request->roll = tcp_offset[3];
    request->pitch = tcp_offset[4];
    request->yaw = tcp_offset[5];
    auto future = set_tcp_offset_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(5)) != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to call service set_tcp_offset");
        return 1;
    }
    return future.get()->ret;
}

int XArmROSClient::setLoad(float mass, const std::vector<float>& center_of_mass)
{
    auto request = std::make_shared<xarm_msgs::srv::SetLoad::Request>();
    request->mass = mass;
    request->xc = center_of_mass[0];
    request->yc = center_of_mass[1];
    request->zc = center_of_mass[2];
    auto future = set_load_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(5)) != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to call service set_load");
        return 1;
    }
    return future.get()->ret;
}

int XArmROSClient::goHome(float jnt_vel_rad, float jnt_acc_rad)
{
    auto request = std::make_shared<xarm_msgs::srv::Move::Request>();
    request->mvvelo = jnt_vel_rad;
    request->mvacc = jnt_acc_rad;
    request->mvtime = 0;
    auto future = go_home_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(5)) != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to call service go_home");
        return 1;
    }
    return future.get()->ret;
}

int XArmROSClient::moveJoint(const std::vector<float>& joint_cmd, float jnt_vel_rad, float jnt_acc_rad)
{
    auto request = std::make_shared<xarm_msgs::srv::Move::Request>();
    request->mvvelo = jnt_vel_rad;
    request->mvacc = jnt_acc_rad;
    request->mvtime = 0;
    request->pose = joint_cmd;
    auto future = move_joint_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(5)) != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to call service move_joint");
        return 1;
    }
    return future.get()->ret;
}

int XArmROSClient::moveLine(const std::vector<float>& cart_cmd, float cart_vel_mm, float cart_acc_mm)
{
    auto request = std::make_shared<xarm_msgs::srv::Move::Request>();
    request->mvvelo = cart_vel_mm;
    request->mvacc = cart_acc_mm;
    request->mvtime = 0;
    request->pose = cart_cmd;
    auto future = move_line_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(5)) != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to call service move_line");
        return 1;
    }
    return future.get()->ret;
}

int XArmROSClient::moveLineB(int num_of_pnts, const std::vector<float> cart_cmds[], float cart_vel_mm, float cart_acc_mm, float radii)
{
    auto request = std::make_shared<xarm_msgs::srv::Move::Request>();
    request->mvvelo = cart_vel_mm;
    request->mvacc = cart_acc_mm;
    request->mvtime = 0;
    request->mvradii = radii;
    for (int i = 0; i < num_of_pnts; i++)
    {
        request->pose = cart_cmds[i];
        auto future = move_lineb_client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(5)) != rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed to call service move_lineb");
            return 1;
        }
        if (future.get()->ret) return 1;
    }
    return 0;
}

int XArmROSClient::config_tool_modbus(int baud_rate, int time_out_ms)
{
    auto request = std::make_shared<xarm_msgs::srv::ConfigToolModbus::Request>();
    request->baud_rate = baud_rate;
    request->timeout_ms = time_out_ms;
    auto future = config_modbus_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(5)) != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to call service config_tool_modbus");
        return 1;
    }
    return future.get()->ret;
}

int XArmROSClient::send_tool_modbus(unsigned char* data, int send_len, unsigned char* recv_data, int recv_len)
{
    auto request = std::make_shared<xarm_msgs::srv::SetToolModbus::Request>();
    for (int i = 0; i < send_len; i++)
        request->send_data.push_back(data[i]);
    request->respond_len = recv_len;

    auto future = send_modbus_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(5)) != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to call service set_tool_modbus");
        return 1;
    }
    auto response = future.get();
    if (recv_data && recv_len > 0 && static_cast<int>(response->respond_data.size()) >= recv_len)
    {
        for (int j = 0; j < recv_len; j++)
            recv_data[j] = response->respond_data[static_cast<size_t>(j)];
    }
    return 0;
}

int XArmROSClient::gripperMove(float pulse)
{
    auto request = std::make_shared<xarm_msgs::srv::GripperMove::Request>();
    request->pulse_pos = pulse;
    auto future = gripper_move_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(5)) != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to call service gripper_move");
        return 1;
    }
    RCLCPP_INFO(node_->get_logger(), "gripper_move: %f", pulse);
    return future.get()->ret;
}

int XArmROSClient::gripperConfig(float pulse_vel)
{
    auto request = std::make_shared<xarm_msgs::srv::GripperConfig::Request>();
    request->pulse_vel = pulse_vel;
    auto future = gripper_config_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(5)) != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to call service gripper_config");
        return 1;
    }
    return future.get()->ret;
}

int XArmROSClient::getGripperState(float *curr_pulse, int *curr_err)
{
    auto request = std::make_shared<xarm_msgs::srv::GripperState::Request>();
    auto future = gripper_state_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(5)) != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to call service gripper_state");
        return 1;
    }
    auto resp = future.get();
    *curr_pulse = resp->curr_pos;
    *curr_err = resp->err_code;
    return 0;
}

}
