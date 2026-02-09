#include "interbotix_slate_driver/slate_base.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("slate_base");
  slate_base::SlateBase driver(node.get());

  rclcpp::Rate r(20);
  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    driver.update();
    r.sleep();
  }
  rclcpp::shutdown();
  return 0;
}
