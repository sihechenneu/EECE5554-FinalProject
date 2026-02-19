#include "interbotix_moveit_interface/moveit_interface_obj.h"
#include <moveit/planning_interface/planning_interface.hpp>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>

namespace
{
const std::string PLANNING_GROUP = "interbotix_arm";
}

InterbotixMoveItInterface::InterbotixMoveItInterface(const rclcpp::Node::SharedPtr & node)
: node_(node),
  move_group_(node, PLANNING_GROUP),
  joint_model_group_(nullptr),
  text_pose_(Eigen::Isometry3d::Identity())
{
  joint_model_group_ = move_group_.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  visual_tools_ = std::make_unique<moveit_visual_tools::MoveItVisualTools>(node_, move_group_.getPlanningFrame(), "moveit_visual_tools");
  visual_tools_->deleteAllMarkers();
  text_pose_.translation().z() = 1.0;
  visual_tools_->publishText(text_pose_, "InterbotixMoveItInterface", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
  visual_tools_->trigger();

  srv_moveit_plan_ = node_->create_service<interbotix_moveit_interface::srv::MoveItPlan>(
    "moveit_plan",
    [this](const std::shared_ptr<interbotix_moveit_interface::srv::MoveItPlan::Request> req,
           std::shared_ptr<interbotix_moveit_interface::srv::MoveItPlan::Response> res) {
      moveit_planner(req, res);
    });

  RCLCPP_INFO(node_->get_logger(), "Reference frame: %s", move_group_.getPlanningFrame().c_str());
  RCLCPP_INFO(node_->get_logger(), "End effector link: %s", move_group_.getEndEffectorLink().c_str());
}

InterbotixMoveItInterface::~InterbotixMoveItInterface() = default;

bool InterbotixMoveItInterface::moveit_plan_joint_positions(const std::vector<double> & joint_group_positions)
{
  visual_tools_->deleteAllMarkers();
  move_group_.setJointValueTarget(joint_group_positions);
  moveit::core::MoveItErrorCode err = move_group_.plan(saved_plan_);
  bool success = (err == moveit::core::MoveItErrorCode::SUCCESS);
  visual_tools_->publishText(text_pose_, "Joint Space Goal", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
  if (success && saved_plan_.trajectory)
    visual_tools_->publishTrajectoryLine(*saved_plan_.trajectory, joint_model_group_);
  visual_tools_->trigger();
  return success;
}

bool InterbotixMoveItInterface::moveit_plan_ee_pose(const geometry_msgs::msg::Pose & pose)
{
  visual_tools_->deleteAllMarkers();
  move_group_.setPoseTarget(pose);
  moveit::core::MoveItErrorCode err = move_group_.plan(saved_plan_);
  bool success = (err == moveit::core::MoveItErrorCode::SUCCESS);
  visual_tools_->publishAxisLabeled(pose, "ee_pose");
  visual_tools_->publishText(text_pose_, "Pose Goal", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
  if (success && saved_plan_.trajectory)
    visual_tools_->publishTrajectoryLine(*saved_plan_.trajectory, joint_model_group_);
  visual_tools_->trigger();
  return success;
}

bool InterbotixMoveItInterface::moveit_plan_ee_position(double x, double y, double z)
{
  visual_tools_->deleteAllMarkers();
  move_group_.setPositionTarget(x, y, z);
  moveit::core::MoveItErrorCode err = move_group_.plan(saved_plan_);
  bool success = (err == moveit::core::MoveItErrorCode::SUCCESS);
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  visual_tools_->publishAxisLabeled(pose, "ee_pose");
  visual_tools_->publishText(text_pose_, "Position Goal", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
  if (success && saved_plan_.trajectory)
    visual_tools_->publishTrajectoryLine(*saved_plan_.trajectory, joint_model_group_);
  visual_tools_->trigger();
  return success;
}

bool InterbotixMoveItInterface::moveit_plan_ee_orientation(const geometry_msgs::msg::Quaternion & quat)
{
  visual_tools_->deleteAllMarkers();
  move_group_.setOrientationTarget(quat.x, quat.y, quat.z, quat.w);
  moveit::core::MoveItErrorCode err = move_group_.plan(saved_plan_);
  bool success = (err == moveit::core::MoveItErrorCode::SUCCESS);
  geometry_msgs::msg::Pose pose = moveit_get_ee_pose();
  pose.orientation = quat;
  visual_tools_->publishAxisLabeled(pose, "ee_pose");
  visual_tools_->publishText(text_pose_, "Orientation Goal", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
  if (success && saved_plan_.trajectory)
    visual_tools_->publishTrajectoryLine(*saved_plan_.trajectory, joint_model_group_);
  visual_tools_->trigger();
  return success;
}

bool InterbotixMoveItInterface::moveit_plan_cartesian_path(const std::vector<geometry_msgs::msg::Pose> & waypoints)
{
  moveit_msgs::msg::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = move_group_.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  RCLCPP_INFO(node_->get_logger(), "Cartesian path (%.2f%% achieved)", fraction * 100.0);
  visual_tools_->deleteAllMarkers();
  visual_tools_->publishText(text_pose_, "Cartesian Path", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
  visual_tools_->publishPath(waypoints, rviz_visual_tools::LIME_GREEN, rviz_visual_tools::SMALL);
  for (size_t i = 0; i < waypoints.size(); ++i)
    visual_tools_->publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rviz_visual_tools::SMALL);
  visual_tools_->trigger();

  saved_plan_.trajectory = std::make_shared<moveit_msgs::msg::RobotTrajectory>(trajectory);
  bool success = (1.0 - fraction < 0.1);
  return success;
}

bool InterbotixMoveItInterface::moveit_execute_plan()
{
  moveit::core::MoveItErrorCode err = move_group_.execute(saved_plan_);
  return (err == moveit::core::MoveItErrorCode::SUCCESS);
}

void InterbotixMoveItInterface::moveit_set_path_constraint(const std::string & constrained_link,
  const std::string & reference_link, const geometry_msgs::msg::Quaternion & quat, double tolerance)
{
  moveit_msgs::msg::OrientationConstraint ocm;
  ocm.link_name = constrained_link;
  ocm.header.frame_id = reference_link;
  ocm.orientation = quat;
  ocm.absolute_x_axis_tolerance = tolerance;
  ocm.absolute_y_axis_tolerance = tolerance;
  ocm.absolute_z_axis_tolerance = tolerance;
  ocm.weight = 1.0;
  moveit_msgs::msg::Constraints constraints;
  constraints.orientation_constraints.push_back(ocm);
  move_group_.setPathConstraints(constraints);
  move_group_.setPlanningTime(30.0);
}

void InterbotixMoveItInterface::moveit_clear_path_constraints()
{
  move_group_.clearPathConstraints();
  move_group_.setPlanningTime(5.0);
}

geometry_msgs::msg::Pose InterbotixMoveItInterface::moveit_get_ee_pose()
{
  return move_group_.getCurrentPose().pose;
}

void InterbotixMoveItInterface::moveit_scale_ee_velocity(double factor)
{
  move_group_.setMaxVelocityScalingFactor(factor);
}

void InterbotixMoveItInterface::moveit_planner(
  const std::shared_ptr<interbotix_moveit_interface::srv::MoveItPlan::Request> request,
  std::shared_ptr<interbotix_moveit_interface::srv::MoveItPlan::Response> response)
{
  bool success = false;
  std::string service_type;
  if (request->cmd == CMD_PLAN_POSE) {
    success = moveit_plan_ee_pose(request->ee_pose);
    service_type = "Planning EE pose";
  } else if (request->cmd == CMD_PLAN_POSITION) {
    success = moveit_plan_ee_position(
      request->ee_pose.position.x, request->ee_pose.position.y, request->ee_pose.position.z);
    service_type = "Planning EE position";
  } else if (request->cmd == CMD_PLAN_ORIENTATION) {
    success = moveit_plan_ee_orientation(request->ee_pose.orientation);
    service_type = "Planning EE orientation";
  } else if (request->cmd == CMD_EXECUTE) {
    success = moveit_execute_plan();
    service_type = "Execution";
  }
  response->success = success;
  response->msg.data = success ? (service_type + " was successful!") : (service_type + " was not successful.");
}
