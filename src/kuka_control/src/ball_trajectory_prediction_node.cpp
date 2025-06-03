#include "ball_trajectory_prediction_node.hpp"

BallTrajectoryPredictionNode::BallTrajectoryPredictionNode()
    : Node("ball_trajectory_prediction_node"), is_initialized_(false)
{
  ball_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
      "/ball/position", 10, std::bind(&BallTrajectoryPredictionNode::ballCallback, this, std::placeholders::_1));

  pred_pub_ = this->create_publisher<geometry_msgs::msg::Point>("/ball/predicted_position", 10);
  ground_pub_ = this->create_publisher<geometry_msgs::msg::Point>("/ball/ground_hit", 10);

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
  if (dt <= 0.0)
    dt = 0.01;

  /* measurement as Eigen --------------------------------------------------- */
  Eigen::Vector3d z(msg->x, msg->y, msg->z);

  // ── detect restart (ball jumped upward > 5 m) ─────────────────────────
  if (is_initialized_ &&
      prev_meas_ &&
      (z.z() - prev_meas_->z()) > 5.0)
  {
    is_initialized_ = false;
    prev_meas_.reset(); // forget old sample

    kf_.reset(); // clear filter internals ❶
    last_time_ = now;
    return; // wait for next sample to seed v₀
  }

  visualization_msgs::msg::MarkerArray arr;

  /* ── initialisation (need two samples for v0) ──────────────────────────── */
  if (!is_initialized_)
  {
    if (!prev_meas_)
    { // first sample → store & wait
      prev_meas_ = z;
      last_time_ = now;
      return;
    }
    Eigen::Vector3d v0 = (z - *prev_meas_) / dt;
    kf_.init(z, v0); // init with position + velocity guess
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
    out.x = pred.x();
    out.y = pred.y();
    out.z = pred.z();
    pred_pub_->publish(out);
    /* ---------- predict ground-impact point ----------------------------- */
    const double g = 9.81;
    Eigen::Vector3d pos = kf_.position();
    Eigen::Vector3d vel = kf_.velocity();

    /* solve ½ g t² − v_z t − z = 0  (we keep positive root) */
    double a = 0.5 * g;
    double b = -vel.z();
    double c = -pos.z(); // ground is z=0
    double disc = b * b - 4 * a * c;

    bool hit_valid = disc >= 0.0 && a > 0.0;
    double t_hit = (-b + std::sqrt(disc)) / (2 * a); // positive root

    if (hit_valid && t_hit > 0.0 && t_hit < 5.0) // ignore absurd hits
    {
      Eigen::Vector3d hit;
      hit.x() = pos.x() + vel.x() * t_hit;
      hit.y() = pos.y() + vel.y() * t_hit;
      hit.z() = 0.0;

      /* publish on /ball/ground_hit */
      geometry_msgs::msg::Point p;
      p.x = hit.x();
      p.y = hit.y();
      p.z = hit.z();
      ground_pub_->publish(p);

      /* add RViz marker */
      visualization_msgs::msg::Marker m;
      m.header.frame_id = "world";
      m.header.stamp = now;
      m.ns = "ground";
      m.id = 999;
      m.type = visualization_msgs::msg::Marker::SPHERE;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x = hit.x();
      m.pose.position.y = hit.y();
      m.pose.position.z = hit.z();
      m.scale.x = m.scale.y = m.scale.z = 0.25;
      m.color.a = 1.0;
      m.color.r = 0.0;
      m.color.g = 0.5;
      m.color.b = 1.0;
      arr.markers.push_back(m);
    }
  }

  /* ── RViz markers -------------------------------------------------------- */

  // green ball (measurement)
  visualization_msgs::msg::Marker ball;
  ball.header.frame_id = "world";
  ball.header.stamp = now;
  ball.ns = "ball";
  ball.id = 0;
  ball.type = visualization_msgs::msg::Marker::SPHERE;
  ball.action = visualization_msgs::msg::Marker::ADD;
  ball.pose.position.x = z.x();
  ball.pose.position.y = z.y();
  ball.pose.position.z = z.z();
  ball.scale.x = ball.scale.y = ball.scale.z = 0.20;
  ball.color.a = 1.0;
  ball.color.r = 0.0;
  ball.color.g = 1.0;
  ball.color.b = 0.0;
  arr.markers.push_back(ball);

  // future prediction spheres
  std::vector<double> future{0.05, 0.10, 0.15, 0.20, 0.25};
  int id = 1;
  for (size_t i = 0; i < future.size(); ++i)
  {
    Eigen::Vector3d p = kf_.predictFuture(future[i]);

    visualization_msgs::msg::Marker m;
    m.header.frame_id = "world";
    m.header.stamp = now;
    m.ns = "predictions";
    m.id = id++;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position.x = p.x();
    m.pose.position.y = p.y();
    m.pose.position.z = p.z();
    m.scale.x = m.scale.y = m.scale.z = 0.12;

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