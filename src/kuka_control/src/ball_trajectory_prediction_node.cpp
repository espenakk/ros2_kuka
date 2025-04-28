#include "ball_trajectory_prediction_node.hpp"

BallTrajectoryPredictionNode::BallTrajectoryPredictionNode()
    : Node("ball_trajectory_prediction_node"), is_initialized_(false)
{
  ball_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
      "/ball/position", 10, std::bind(&BallTrajectoryPredictionNode::ballCallback, this, std::placeholders::_1));

  pred_pub_ = this->create_publisher<geometry_msgs::msg::Point>("/ball/predicted_position", 10);

  clock_ = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  last_time_ = clock_->now();
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
    kf_.init(measurement, dt);
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
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BallTrajectoryPredictionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}