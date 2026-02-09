#include <rclcpp/rclcpp.hpp>
#include "interbotix_ux_ros_control/ux_hardware_interface_obj.h"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<interbotix_ux_ros_control::UXHardwareInterface>();
  if (!node->init()) {
    return 1;
  }
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
