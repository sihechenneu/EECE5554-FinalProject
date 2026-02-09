#include "interbotix_xs_sdk/xs_sdk_obj.h"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("xs_sdk");
  bool success = true;
  InterbotixRobotXS bot(node.get(), success);
  if (success)
    rclcpp::spin(node);
  else
  {
    RCLCPP_FATAL(node->get_logger(),
      "[xs_sdk] For troubleshooting, please see "
      "'https://docs.trossenrobotics.com/interbotix_xsarms_docs/troubleshooting.html'");
    rclcpp::shutdown();
  }
  return 0;
}
