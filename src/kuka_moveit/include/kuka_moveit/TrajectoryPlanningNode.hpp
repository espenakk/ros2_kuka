#include "geometry_msgs/msg/pose.hpp"
#include "moveit_msgs/msg/motion_plan_response.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.hpp>
#include <moveit/robot_state/robot_state.hpp>

using moveit::planning_interface::MoveGroupInterface;
using planning_scene_monitor::PlanningSceneMonitor;
using planning_scene_monitor::CurrentStateMonitor;

class TrajectoryPlanningNode : public rclcpp::Node 
{
public:
  TrajectoryPlanningNode();

  void initMoveit();

private:
  const std::string PLANNING_GROUP = "manipulator";
  std::unique_ptr<MoveGroupInterface> move_group_interface;
  std::unique_ptr<PlanningSceneMonitor> psm;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_subscriber;
  rclcpp::Publisher<moveit_msgs::msg::MotionPlanResponse>::SharedPtr plan_publisher;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_subscriber;
  std::vector<double> velocities;
};
