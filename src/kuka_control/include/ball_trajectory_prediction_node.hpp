#pragma once

#include "kalman_filter.hpp"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>


class BallTrajectoryPredictionNode : public rclcpp::Node
{
public:
    BallTrajectoryPredictionNode();

private:
    void ballCallback(const geometry_msgs::msg::Point::SharedPtr msg);
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr ball_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pred_pub_;

    KalmanFilter kf_;
    bool is_initialized_;
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Time last_time_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

};