#include "interbotix_moveit_interface/moveit_interface_obj.h"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("moveit_interface");
  InterbotixMoveItInterface interface(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
