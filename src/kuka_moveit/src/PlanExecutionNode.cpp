#include "kuka_moveit/PlanExecutionNode.hpp"

PlanExecutionNode::PlanExecutionNode()
    : Node("plan_execution_node")
{
  pose_subscriber = this->create_subscription<moveit_msgs::msg::MotionPlanResponse>("/desired_plan", 10,
      [this](const moveit_msgs::msg::MotionPlanResponse &plan_msg)
      {
        MoveGroupInterface::Plan plan;
        plan.planning_time = plan_msg.planning_time;
        plan.trajectory = plan_msg.trajectory;
        plan.start_state = plan_msg.trajectory_start;
        move_group_interface->execute(plan);
      });
}

void PlanExecutionNode::initMoveit()
{
  move_group_interface = std::make_unique<MoveGroupInterface>(shared_from_this(), PLANNING_GROUP);
}

