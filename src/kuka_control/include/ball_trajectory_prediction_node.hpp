#pragma once

#include "kalman_filter.hpp"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>

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

};