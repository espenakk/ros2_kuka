#include "ball_trajectory_prediction_node.hpp"

BallTrajectoryPredictionNode::BallTrajectoryPredictionNode()
    : Node("ball_trajectory_prediction_node"), is_initialized_(false)
{
  ball_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
      "/ball/position", 10, std::bind(&BallTrajectoryPredictionNode::ballCallback, this, std::placeholders::_1));

  pred_pub_ = this->create_publisher<geometry_msgs::msg::Point>("/ball/predicted_position", 10);

  clock_ = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  last_time_ = clock_->now();
  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("ball_prediction_markers", 10);
}

void BallTrajectoryPredictionNode::ballCallback(const geometry_msgs::msg::Point::SharedPtr msg)
{
  rclcpp::Time now = clock_->now();
  double dt = (now - last_time_).seconds();
  if (dt == 0.0)
    dt = 0.01; // fallback to small dt

  Eigen::Vector3d measurement(msg->x, msg->y, msg->z);

  if (!is_initialized_)
  {
    kf_.init(measurement);
    is_initialized_ = true;
  }
  else
  {
    kf_.predict(dt);
    kf_.update(measurement);

    // Predict 0.5s ahead
    Eigen::Vector3d pred = kf_.predictFuturePosition(0.5);

    geometry_msgs::msg::Point pred_msg;
    pred_msg.x = pred.x();
    pred_msg.y = pred.y();
    pred_msg.z = pred.z();
    pred_pub_->publish(pred_msg);
  }

  last_time_ = now;

  visualization_msgs::msg::MarkerArray marker_array;

  // Marker for current ball position
  visualization_msgs::msg::Marker ball_marker;
  ball_marker.header.frame_id = "world"; // <-- Make sure your RViz Fixed Frame matches
  ball_marker.header.stamp = now;
  ball_marker.ns = "ball";
  ball_marker.id = 0;
  ball_marker.type = visualization_msgs::msg::Marker::SPHERE;
  ball_marker.action = visualization_msgs::msg::Marker::ADD;
  ball_marker.pose.position.x = measurement.x();
  ball_marker.pose.position.y = measurement.y();
  ball_marker.pose.position.z = measurement.z();
  ball_marker.scale.x = 0.1;
  ball_marker.scale.y = 0.1;
  ball_marker.scale.z = 0.1;
  ball_marker.color.a = 1.0;
  ball_marker.color.r = 0.0;
  ball_marker.color.g = 1.0;
  ball_marker.color.b = 0.0;

  marker_array.markers.push_back(ball_marker);

  // Markers for future predicted points
  std::vector<double> future_times = {0.05, 0.1, 0.15, 0.2, 0.25}; // prediction times

  // Predicted points
  int id = 1;
  for (size_t i = 0; i < future_times.size(); ++i)
  {
    double t_future = future_times[i];
    Eigen::Vector3d pred_point = kf_.predictFuturePosition(t_future);

    visualization_msgs::msg::Marker pred_marker;
    pred_marker.header.frame_id = "world";
    pred_marker.header.stamp = now;
    pred_marker.ns = "predictions";
    pred_marker.id = id++;
    pred_marker.type = visualization_msgs::msg::Marker::SPHERE;
    pred_marker.action = visualization_msgs::msg::Marker::ADD;
    pred_marker.pose.position.x = pred_point.x();
    pred_marker.pose.position.y = pred_point.y();
    pred_marker.pose.position.z = pred_point.z();
    pred_marker.scale.x = 0.05;
    pred_marker.scale.y = 0.05;
    pred_marker.scale.z = 0.05;

    // Color: early points green, late points red
    double t_ratio = static_cast<double>(i) / static_cast<double>(future_times.size());
    pred_marker.color.a = 1.0;
    pred_marker.color.r = t_ratio;
    pred_marker.color.g = 1.0 - t_ratio;
    marker_array.markers.push_back(pred_marker);
  }
  marker_pub_->publish(marker_array);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BallTrajectoryPredictionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}