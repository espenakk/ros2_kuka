#include "kuka_moveit/TrajectoryPlanningNode.hpp"
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_msgs/msg/motion_plan_response.hpp>
#include <thread>
#include <chrono>

TrajectoryPlanningNode::TrajectoryPlanningNode()
    : Node("trajectory_planning_node",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
{
  pose_subscriber = this->create_subscription<geometry_msgs::msg::Pose>("/desired_pose", 10,
      [this](const geometry_msgs::msg::Pose &pose)
      {
        // move_group_interface->setPlanningPipelineId("pilz_industrial_motion_planner");
        // move_group_interface->setPlannerId("PTP");
        move_group_interface->setPoseTarget(pose);
        move_group_interface->setMaxAccelerationScalingFactor(0.01);
        move_group_interface->setMaxVelocityScalingFactor(0.1);
        MoveGroupInterface::Plan plan;
        bool success = (move_group_interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (success)
        {
          moveit_msgs::msg::MotionPlanResponse plan_msg;
          plan_msg.planning_time = plan.planning_time;
          plan_msg.trajectory = plan.trajectory;
          plan_msg.trajectory_start = plan.start_state;
          plan_msg.group_name = PLANNING_GROUP;
          plan_publisher->publish(plan_msg);
          std::this_thread::sleep_for(std::chrono::milliseconds(30));
        }
        else
        {
          RCLCPP_INFO(this->get_logger(), "Planning failed");
        }
      });
  plan_publisher = this->create_publisher<moveit_msgs::msg::MotionPlanResponse>("/desired_plan", 10);

  joint_subscriber = create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10,
      [this](const sensor_msgs::msg::JointState &state)
      {
        velocities = state.velocity;
      });
}

void TrajectoryPlanningNode::initMoveit()
{
  move_group_interface = std::make_unique<MoveGroupInterface>(shared_from_this(), PLANNING_GROUP);
  move_group_interface->startStateMonitor();
  psm = std::make_unique<PlanningSceneMonitor>(shared_from_this(), "robot_description");
  psm->startStateMonitor();
  psm->startSceneMonitor();
  psm->startWorldGeometryMonitor();
}

