#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <moveit/planning_scene_monitor/planning_scene_monitor.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Eigen>

using planning_scene_monitor::PlanningSceneMonitor;
using moveit::core::RobotModelConstPtr;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;

class PointToLegalPoseNode : public rclcpp::Node
{
public:
  PointToLegalPoseNode();

  void initMoveit();

private:
  const std::string PLANNING_GROUP = "manipulator";
  std::unique_ptr<PlanningSceneMonitor> planning_scene_monitor;
  rclcpp::Subscription<Point>::SharedPtr ball_subscriber;
  rclcpp::Publisher<Pose>::SharedPtr pose_publisher;
};
