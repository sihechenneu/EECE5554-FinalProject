#include "interbotix_tf_tools/tf_rebroadcaster.h"

TFRebroadcaster::TFRebroadcaster()
: Node("tf_rebroadcaster")
{
  declare_parameter<std::string>("filepath_config", "");
  declare_parameter<std::string>("topic_to", "");
  declare_parameter<std::string>("topic_from", "");
  declare_parameter<bool>("use_incoming_time", true);

  get_parameter("filepath_config", filepath_config_);
  get_parameter("topic_to", topic_to_);
  get_parameter("topic_from", topic_from_);
  get_parameter("use_incoming_time", use_incoming_time_);

  try {
    config_ = YAML::LoadFile(filepath_config_.c_str());
  } catch (const YAML::BadFile & error) {
    RCLCPP_FATAL(
      get_logger(),
      "[tf_rebroadcaster] Config file at '%s' was not found or has a bad format. Shutting down...",
      filepath_config_.c_str());
    RCLCPP_FATAL(get_logger(), "YAML Error: '%s'", error.what());
    return;
  }

  RCLCPP_INFO(
    get_logger(),
    "[tf_rebroadcaster] Will broadcast TFs from topic '%s' to topic '%s'.",
    topic_from_.c_str(),
    topic_to_.c_str());

  YAML::Node all_frames = config_["frames"];
  for (YAML::const_iterator frame_itr = all_frames.begin();
       frame_itr != all_frames.end();
       frame_itr++)
  {
    Frame frame = Frame();
    frame.parent_frame_id = frame_itr->first.as<std::string>();
    frame.child_frame_id = frame_itr->second["child_frame_id"].as<std::string>();
    frame.prefix = frame_itr->second["prefix"].as<std::string>();
    frames_.push_back(frame);

    if (frame.prefix == "") {
      RCLCPP_INFO(
        get_logger(),
        "[tf_rebroadcaster] Will broadcast TF from frame '%s' to frame '%s'.",
        frame.child_frame_id.c_str(),
        frame.prefix.c_str());
    } else {
      RCLCPP_INFO(
        get_logger(),
        "[tf_rebroadcaster] Will broadcast TF from frame '%s' to frame '%s', prepending prefix '%s'.",
        frame.parent_frame_id.c_str(),
        frame.child_frame_id.c_str(),
        frame.prefix.c_str());
    }
  }

  sub_tf_ = create_subscription<tf2_msgs::msg::TFMessage>(
    topic_from_, 10, std::bind(&TFRebroadcaster::tf_cb, this, std::placeholders::_1));
  pub_tf_ = create_publisher<tf2_msgs::msg::TFMessage>(topic_to_, 10);
}

void TFRebroadcaster::tf_cb(const tf2_msgs::msg::TFMessage::SharedPtr msg)
{
  for (auto & frame : frames_) {
    for (auto & tf : msg->transforms) {
      if (
        frame.parent_frame_id == tf.header.frame_id &&
        frame.child_frame_id == tf.child_frame_id)
      {
        geometry_msgs::msg::TransformStamped tf_ = tf;
        tf2_msgs::msg::TFMessage rebroadcast_tf;
        if (frame.prefix != "") {
          tf_.child_frame_id = frame.prefix + tf.child_frame_id;
          tf_.header.frame_id = frame.prefix + tf.header.frame_id;
        }
        if (!use_incoming_time_) {
          tf_.header.stamp = now();
        }
        rebroadcast_tf.transforms.push_back(tf_);
        pub_tf_->publish(rebroadcast_tf);
        if (!frame.logged) {
          RCLCPP_INFO(
            get_logger(),
            "Broadcasted TF from frame '%s' to frame '%s'. This will only log once.",
            tf_.child_frame_id.c_str(),
            tf_.header.frame_id.c_str());
          frame.logged = true;
        }
      }
    }
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TFRebroadcaster>());
  rclcpp::shutdown();
  return 0;
}
