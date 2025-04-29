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

void BallTrajectoryPredictionNode::ballCallback(
  const geometry_msgs::msg::Point::SharedPtr msg)
{
/* ── time step ─────────────────────────────────────────────────────────── */
rclcpp::Time now = clock_->now();
double dt = (now - last_time_).seconds();
if (dt <= 0.0) dt = 0.01;

/* measurement as Eigen --------------------------------------------------- */
Eigen::Vector3d z(msg->x, msg->y, msg->z);

/* ── initialisation (need two samples for v0) ──────────────────────────── */
if (!is_initialized_)
{
  if (!prev_meas_) {               // first sample → store & wait
    prev_meas_ = z;
    last_time_ = now;
    return;
  }
  Eigen::Vector3d v0 = (z - *prev_meas_) / dt;
  kf_.init(z, v0);                 // init with position + velocity guess
  is_initialized_ = true;
}
else
{
  /* ── normal Kalman loop ──────────────────────────────────────────────── */
  kf_.predict(dt);
  kf_.update(z);

  // 0.5 s-ahead point for external use
  Eigen::Vector3d pred = kf_.predictFuture(0.5);
  geometry_msgs::msg::Point out;
  out.x = pred.x();  out.y = pred.y();  out.z = pred.z();
  pred_pub_->publish(out);
}

/* ── RViz markers -------------------------------------------------------- */
visualization_msgs::msg::MarkerArray arr;

// green ball (measurement)
visualization_msgs::msg::Marker ball;
ball.header.frame_id = "world";
ball.header.stamp    = now;
ball.ns = "ball";  ball.id = 0;
ball.type = visualization_msgs::msg::Marker::SPHERE;
ball.action = visualization_msgs::msg::Marker::ADD;
ball.pose.position.x = z.x();
ball.pose.position.y = z.y();
ball.pose.position.z = z.z();
ball.scale.x = ball.scale.y = ball.scale.z = 0.10;
ball.color.a = 1.0;  ball.color.r = 0.0;  ball.color.g = 1.0;  ball.color.b = 0.0;
arr.markers.push_back(ball);

// future prediction spheres
std::vector<double> future{0.05,0.10,0.15,0.20,0.25};
int id = 1;
for (size_t i = 0; i < future.size(); ++i)
{
  Eigen::Vector3d p = kf_.predictFuture(future[i]);

  visualization_msgs::msg::Marker m;
  m.header.frame_id = "world";
  m.header.stamp    = now;
  m.ns  = "predictions";
  m.id  = id++;
  m.type = visualization_msgs::msg::Marker::SPHERE;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.pose.position.x = p.x();
  m.pose.position.y = p.y();
  m.pose.position.z = p.z();
  m.scale.x = m.scale.y = m.scale.z = 0.05;

  double ratio = static_cast<double>(i) / future.size();
  m.color.a = 1.0;
  m.color.r = ratio;
  m.color.g = 1.0 - ratio;
  m.color.b = 0.0;

  arr.markers.push_back(m);
}
marker_pub_->publish(arr);

/* ── bookkeeping --------------------------------------------------------- */
prev_meas_ = z;
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