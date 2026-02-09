#ifndef INTERBOTIX_MOVEIT_INTERFACE__MOVEIT_INTERFACE_OBJ_H_
#define INTERBOTIX_MOVEIT_INTERFACE__MOVEIT_INTERFACE_OBJ_H_

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <interbotix_moveit_interface/srv/move_it_plan.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

namespace moveit::core {
enum MoveItErrorCode : int;
}

class InterbotixMoveItInterface
{
public:
  static constexpr int8_t CMD_PLAN_POSE = 1;
  static constexpr int8_t CMD_PLAN_POSITION = 2;
  static constexpr int8_t CMD_PLAN_ORIENTATION = 3;
  static constexpr int8_t CMD_EXECUTE = 4;

  explicit InterbotixMoveItInterface(const rclcpp::Node::SharedPtr & node);
  ~InterbotixMoveItInterface();

  bool moveit_plan_joint_positions(const std::vector<double> & joint_group_positions);
  bool moveit_plan_ee_pose(const geometry_msgs::msg::Pose & pose);
  bool moveit_plan_ee_position(double x, double y, double z);
  bool moveit_plan_ee_orientation(const geometry_msgs::msg::Quaternion & quat);
  bool moveit_plan_cartesian_path(const std::vector<geometry_msgs::msg::Pose> & waypoints);
  bool moveit_execute_plan();
  void moveit_set_path_constraint(const std::string & constrained_link, const std::string & reference_link,
    const geometry_msgs::msg::Quaternion & quat, double tolerance);
  void moveit_clear_path_constraints();
  geometry_msgs::msg::Pose moveit_get_ee_pose();
  void moveit_scale_ee_velocity(double factor);

private:
  void moveit_planner(
    const std::shared_ptr<interbotix_moveit_interface::srv::MoveItPlan::Request> request,
    std::shared_ptr<interbotix_moveit_interface::srv::MoveItPlan::Response> response);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Service<interbotix_moveit_interface::srv::MoveItPlan>::SharedPtr srv_moveit_plan_;
  moveit::planning_interface::MoveGroupInterface move_group_;
  const moveit::core::JointModelGroup * joint_model_group_;
  std::unique_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools_;
  moveit::planning_interface::MoveGroupInterface::Plan saved_plan_;
  Eigen::Isometry3d text_pose_;
};

#endif  // INTERBOTIX_MOVEIT_INTERFACE__MOVEIT_INTERFACE_OBJ_H_
