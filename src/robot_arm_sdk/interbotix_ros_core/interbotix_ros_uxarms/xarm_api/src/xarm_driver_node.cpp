/* Copyright 2018 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jason Peng <jason@ufactory.cc>
 ============================================================================*/
#include <xarm_driver.h>
#include <xarm/linux/thread.h>
#include <signal.h>
#include "xarm/connect.h"
#include "xarm/report_data.h"
#include <chrono>
#include <thread>

void exit_sig_handler(int signum)
{
    (void)signum;
    fprintf(stderr, "[xarm_driver] Ctrl-C caught, exit process...\n");
    exit(-1);
}

class XarmRTConnection
{
public:
    XarmRTConnection(rclcpp::Node* node, char *server_ip, xarm_api::XARMDriver* drv)
    {
        node_ = node;
        joint_num_ = node_->declare_parameter<int>("DOF", 7);
        node_->get_parameter("DOF", joint_num_);
        node_->declare_parameter("joint_names", std::vector<std::string>({}));
        node_->get_parameter("joint_names", joint_name_);
        if (joint_name_.empty()) {
            joint_name_.resize(static_cast<size_t>(joint_num_));
            for (int i = 0; i < joint_num_; i++)
                joint_name_[i] = "joint_" + std::to_string(i + 1);
        }
        ip = server_ip;
        xarm_driver = drv;
        xarm_driver->XARMDriverInit(server_ip);
        rclcpp::sleep_for(std::chrono::milliseconds(500));
        thread_id = thread_init(thread_proc, reinterpret_cast<void*>(this));
    }

    void thread_run(void)
    {
        int ret;
        int err_num = 0;
        int rxcnt = 0;
        int first_cycle = 1;
        std::vector<double> prev_angle(static_cast<size_t>(joint_num_), 0.0);

        const auto period = std::chrono::milliseconds(1000 / static_cast<int>(REPORT_RATE_HZ));

        while (xarm_driver->isConnectionOK())
        {
            std::this_thread::sleep_for(period);
            ret = xarm_driver->get_frame();
            if (ret != 0) continue;

            ret = xarm_driver->get_rich_data(norm_data);
            if (ret == 0)
            {
                rxcnt++;

                js_msg.header.stamp = node_->now();
                js_msg.header.frame_id = "real-time data";
                js_msg.name.resize(static_cast<size_t>(joint_num_));
                js_msg.position.resize(static_cast<size_t>(joint_num_));
                js_msg.velocity.resize(static_cast<size_t>(joint_num_));
                js_msg.effort.resize(static_cast<size_t>(joint_num_));

                for (int i = 0; i < joint_num_; i++)
                {
                    double d = static_cast<double>(norm_data.angle_[i]);
                    js_msg.name[static_cast<size_t>(i)] = joint_name_[static_cast<size_t>(i)];
                    js_msg.position[static_cast<size_t>(i)] = d;

                    if (first_cycle)
                    {
                        js_msg.velocity[static_cast<size_t>(i)] = 0;
                        first_cycle = 0;
                    }
                    else
                    {
                        js_msg.velocity[static_cast<size_t>(i)] = (js_msg.position[static_cast<size_t>(i)] - prev_angle[static_cast<size_t>(i)]) * REPORT_RATE_HZ;
                    }

                    js_msg.effort[static_cast<size_t>(i)] = static_cast<double>(norm_data.tau_[i]);
                    prev_angle[static_cast<size_t>(i)] = d;
                }

                xarm_driver->pub_joint_state(js_msg);

                rm_msg.state = norm_data.runing_;
                rm_msg.mode = norm_data.mode_;
                rm_msg.cmdnum = norm_data.cmdnum_;
                rm_msg.err = norm_data.err_;
                rm_msg.warn = norm_data.war_;
                rm_msg.mt_brake = norm_data.mt_brake_;
                rm_msg.mt_able = norm_data.mt_able_;
                rm_msg.angle.resize(static_cast<size_t>(joint_num_));

                for (int i = 0; i < joint_num_; i++)
                {
                    double d = static_cast<double>(norm_data.angle_[i]);
                    double rnd;
                    char str[8];
                    sprintf(str, "%0.3f", d);
                    sscanf(str, "%lf", &rnd);
                    rm_msg.angle[static_cast<size_t>(i)] = static_cast<float>(rnd);
                }
                for (int i = 0; i < 6; i++)
                {
                    rm_msg.pose[static_cast<size_t>(i)] = norm_data.pose_[i];
                    rm_msg.offset[static_cast<size_t>(i)] = norm_data.tcp_offset_[i];
                }
                xarm_driver->pub_robot_msg(rm_msg);
            }
            else
            {
                printf("Error: real_data.flush_data failed, ret = %d\n", ret);
                err_num++;
            }
        }
        RCLCPP_ERROR(node_->get_logger(), "xArm Connection Failed! Please Shut Down (Ctrl-C) and Retry ...");
    }

    static void* thread_proc(void *arg)
    {
        XarmRTConnection* p = reinterpret_cast<XarmRTConnection*>(arg);
        p->thread_run();
        pthread_exit(0);
    }

    rclcpp::Node* node_;
    pthread_t thread_id;
    char *ip;
    ReportDataNorm norm_data;
    sensor_msgs::msg::JointState js_msg;
    xarm_api::XARMDriver* xarm_driver;
    xarm_msgs::msg::RobotMsg rm_msg;

    int joint_num_;
    std::vector<std::string> joint_name_;
    static constexpr double REPORT_RATE_HZ = 10;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("xarm_driver_node");

    std::string robot_ip = "192.168.1.121";
    node->declare_parameter<std::string>("xarm_robot_ip", robot_ip);
    if (!node->get_parameter("xarm_robot_ip", robot_ip))
    {
        RCLCPP_ERROR(node->get_logger(), "No param named 'xarm_robot_ip'; use default 192.168.1.121");
    }

    char server_ip[20] = {0};
    strncpy(server_ip, robot_ip.c_str(), sizeof(server_ip) - 1);

    xarm_api::XARMDriver driver(node.get());
    RCLCPP_INFO(node->get_logger(), "start xarm driver");

    XarmRTConnection rt_connect(node.get(), server_ip, &driver);

    signal(SIGINT, exit_sig_handler);
    rclcpp::spin(node);

    printf("end");
    return 0;
}
