#include "moveit_msgs/msg/motion_plan_response.hpp"
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <memory>

using moveit::planning_interface::MoveGroupInterface;

class PlanExecutionNode : public rclcpp::Node 
{
public:
  PlanExecutionNode();

  void initMoveit();

private:
  const std::string PLANNING_GROUP = "manipulator";
  std::unique_ptr<MoveGroupInterface> move_group_interface;
  rclcpp::Subscription<moveit_msgs::msg::MotionPlanResponse>::SharedPtr pose_subscriber;
};
