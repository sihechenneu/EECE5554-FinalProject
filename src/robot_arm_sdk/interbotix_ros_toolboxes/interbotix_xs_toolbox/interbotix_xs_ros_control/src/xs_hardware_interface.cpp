#include <rclcpp/rclcpp.hpp>
#include "interbotix_xs_ros_control/xs_hardware_interface_obj.h"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<interbotix_xs_ros_control::XSHardwareInterface>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
