#include "kuka_moveit/PointToLegalPoseNode.hpp"

PointToLegalPoseNode::PointToLegalPoseNode()
  : Node("point_to_legal_pose_node")
{
  ball_subscriber = this->create_subscription<Point>("/ball/predicted_position", 10, 
      [this](const Point &point)
      {
        Pose pose;
        Eigen::Vector3d vec;
        vec = { point.x, point.y, point.z };
        
        double reach = 0;
        
        const moveit::core::RobotModelConstPtr robotModel = planning_scene_monitor->getRobotModel();
        // Retrieves reach in inches (why?)
        // Multiply by 0.9 since extent is a little larger than actual reach
        reach = robotModel->getMaximumExtent() * 0.0254 * 0.9;
        //RCLCPP_INFO(get_logger(), "Vector frobenius norm: %f", vec.norm());

        if (vec.norm() > reach)
        {
          vec.normalize();
          vec *= reach;
        }
        pose.position.x = vec.x();
        pose.position.y = vec.y();
        // Always go above ground level
        pose.position.z = 0.3;

        pose.orientation.w = 1;
        pose.orientation.x = 0;
        pose.orientation.y = 0;
        pose.orientation.z = 0;

        pose_publisher->publish(pose);
      });

  pose_publisher = this->create_publisher<Pose>("/desired_pose", 10);
}

void PointToLegalPoseNode::initMoveit()
{
  planning_scene_monitor = std::make_unique<PlanningSceneMonitor>(shared_from_this(), "robot_description");
  planning_scene_monitor->startSceneMonitor();
  planning_scene_monitor->startStateMonitor();
  planning_scene_monitor->startWorldGeometryMonitor();
}
