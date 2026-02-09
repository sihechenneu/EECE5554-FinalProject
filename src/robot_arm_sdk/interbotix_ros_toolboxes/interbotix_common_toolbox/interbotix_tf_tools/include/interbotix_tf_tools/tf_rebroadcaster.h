#ifndef INTERBOTIX_TF_TOOLS__TF_REBROADCASTER_H_
#define INTERBOTIX_TF_TOOLS__TF_REBROADCASTER_H_

#include <string>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "yaml-cpp/yaml.h"

struct Frame
{
  std::string parent_frame_id;
  std::string child_frame_id;
  std::string prefix;
  bool logged = false;
};

class TFRebroadcaster : public rclcpp::Node
{
public:
  explicit TFRebroadcaster();

private:
  void tf_cb(const tf2_msgs::msg::TFMessage::SharedPtr msg);

  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr sub_tf_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr pub_tf_;

  std::string filepath_config_;
  std::string topic_from_;
  std::string topic_to_;
  bool use_incoming_time_;
  YAML::Node config_;
  std::vector<Frame> frames_;
};

#endif  // INTERBOTIX_TF_TOOLS__TF_REBROADCASTER_H_
