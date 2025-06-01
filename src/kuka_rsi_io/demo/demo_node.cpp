#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <Eigen/Core>
#include "rsi/kukarsiinterface.h"

#include <cmath>     // For M_PI, fabs, std::abs
#include <algorithm> // For std::clamp
#include <vector>    // For std::vector
#include <chrono>    // For std::chrono::milliseconds
#include <atomic>    // For std::atomic

// Define DEG2RAD if not available elsewhere.
// It's often in math utility headers or could be part of your rsi includes.
#ifndef DEG2RAD
#define DEG2RAD (M_PI / 180.0)
#endif

// Helper function to get the sign of a number
template <typename T>
int sign(T val) {
    return (T(0) < val) - (val < T(0));
}

using std::placeholders::_1;

class KukaRsiRos2Node : public rclcpp::Node
{
public:
    KukaRsiRos2Node()
    : Node("kuka_rsi_ros2_node"),
      iface_("192.168.1.50", 49153, 6, std::bind(&KukaRsiRos2Node::on_joint_state, this, std::placeholders::_1))
    {
        // Initialize Eigen arrays (6 joints)
        current_commanded_positions_rad_ = Eigen::ArrayXd::Zero(6);
        target_positions_rad_ = Eigen::ArrayXd::Zero(6);
        current_joint_velocities_rad_per_cycle_ = Eigen::ArrayXd::Zero(6);

        // Publisher for joint states
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

        // Subscriber for joint position commands (in radians)
        joint_cmd_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "joint_position_command", 10,
            std::bind(&KukaRsiRos2Node::on_joint_command, this, _1));

        // Start RSI interface
        iface_.start();

        // Timer for motion control loop
        motion_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(CYCLE_TIME_S_ * 1000)),
            std::bind(&KukaRsiRos2Node::update_motion, this));
        
        RCLCPP_INFO(this->get_logger(), "KukaRsiRos2Node initialized. Waiting for initial joint states from robot...");
    }

    ~KukaRsiRos2Node() {
        iface_.stop();
    }

private:
    void on_joint_state(const Eigen::ArrayXd &joint_positions_rad)
    {
        if (!initial_positions_set_.load(std::memory_order_acquire)) {
            if (joint_positions_rad.size() == 6) {
                current_commanded_positions_rad_ = joint_positions_rad;
                target_positions_rad_ = joint_positions_rad; // Initialize target to current actual
                current_joint_velocities_rad_per_cycle_.setZero();
                initial_positions_set_.store(true, std::memory_order_release);
                RCLCPP_INFO(this->get_logger(), "Initial joint positions received from robot and set.");
            } else {
                RCLCPP_WARN(this->get_logger(), "Received initial joint_state with unexpected size: %ld. Expected 6.", joint_positions_rad.size());
                return;
            }
        }

        auto msg = sensor_msgs::msg::JointState();
        msg.header.stamp = this->now();
        msg.name = {"joint_a1", "joint_a2", "joint_a3", "joint_a4", "joint_a5", "joint_a6"};
        if (joint_positions_rad.size() == 6) {
            msg.position.resize(6);
            for (size_t i = 0; i < 6; ++i) {
                msg.position[i] = joint_positions_rad[i];
            }
        } else {
            // Handle error or unexpected size if necessary, though RSI should provide 6
            RCLCPP_DEBUG(this->get_logger(), "Joint state update with size %ld, expected 6.", joint_positions_rad.size());
            return; // Don't publish if data is not as expected
        }
        joint_state_pub_->publish(msg);
    }

    void on_joint_command(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (msg->data.size() != 6) {
            RCLCPP_WARN(this->get_logger(), "Expected 6 joint positions, got %zu. Command ignored.", msg->data.size());
            return;
        }
        if (!initial_positions_set_.load(std::memory_order_acquire)) {
            RCLCPP_WARN(this->get_logger(), "Initial joint positions not yet set from robot. Command ignored for now. Please wait.");
            return;
        }
        
        RCLCPP_DEBUG(this->get_logger(), "on_joint_command: Received command data:");
        for (size_t i = 0; i < 6; ++i) {
            target_positions_rad_[i] = msg->data[i];
            RCLCPP_DEBUG(this->get_logger(), "  J%zu raw_cmd: %.4f", i, msg->data[i]);
        }
        
        // Log the state of target_positions_rad_ after update
        std::string targets_str = "on_joint_command: New target_positions_rad_ set to [";
        for (size_t i = 0; i < 6; ++i) {
            targets_str += std::to_string(target_positions_rad_[i]) + (i == 5 ? "" : ", ");
        }
        targets_str += "]";
        RCLCPP_INFO(this->get_logger(), "%s", targets_str.c_str());
    }

    void update_motion()
    {
        if (!initial_positions_set_.load(std::memory_order_acquire)) {
            // RCLCPP_DEBUG(this->get_logger(), "update_motion: Waiting for initial positions."); // Can be too verbose
            return; // Wait for initial state from robot
        }

        // Log current targets and commanded positions at the start of update_motion
        std::string current_targets_str = "update_motion: Targets [";
        std::string current_commanded_str = "update_motion: Commanded [";
        for (int i = 0; i < 6; ++i) {
            current_targets_str += std::to_string(target_positions_rad_[i]) + (i == 5 ? "" : ", ");
            current_commanded_str += std::to_string(current_commanded_positions_rad_[i]) + (i == 5 ? "" : ", ");
        }
        current_targets_str += "]";
        current_commanded_str += "]";
        RCLCPP_DEBUG(this->get_logger(), "%s", current_targets_str.c_str());
        RCLCPP_DEBUG(this->get_logger(), "%s", current_commanded_str.c_str());

        bool all_joints_at_target = true;

        for (int i = 0; i < 6; ++i) {
            double current_pos_rad = current_commanded_positions_rad_[i];
            double target_pos_rad = target_positions_rad_[i];
            double current_vel_rad_per_cycle = current_joint_velocities_rad_per_cycle_[i];
            
            double error_rad = target_pos_rad - current_pos_rad;

            // Tolerance for being "at target" (e.g., ~0.005 degrees)
            if (std::abs(error_rad) < 0.0001) {
                current_joint_velocities_rad_per_cycle_[i] = 0.0;
                current_commanded_positions_rad_[i] = target_pos_rad; // Snap to target
                continue; 
            }

            all_joints_at_target = false; 

            int direction_to_target = sign(error_rad);

            // Calculate distance needed to stop from current_vel_rad_per_cycle
            double dist_to_stop = 0.0;
            if (accel_change_in_velocity_per_cycle_ > 1e-9) { // Avoid division by zero if acceleration is effectively zero
                dist_to_stop = (current_vel_rad_per_cycle * current_vel_rad_per_cycle) / (2.0 * accel_change_in_velocity_per_cycle_);
            }
            
            bool should_decelerate = false;
            double deceleration_distance_multiplier = 1.8; // Default multiplier
            if (i == 1) { // Joint A2 (index 1) is more sensitive
                deceleration_distance_multiplier = 5.0; // Increase multiplier for earlier deceleration
            }

            // If moving towards target (velocity sign matches error sign) AND close enough to start decelerating
            if (sign(current_vel_rad_per_cycle) == direction_to_target && direction_to_target != 0) {
                if (std::abs(error_rad) <= std::abs(dist_to_stop) * deceleration_distance_multiplier) { // Use adjusted multiplier
                    should_decelerate = true;
                }
            } 
            // If moving away from target, or stationary but not at target and need to start moving
            else if (direction_to_target != 0) { 
                 if (sign(current_vel_rad_per_cycle) != 0 && sign(current_vel_rad_per_cycle) != direction_to_target) { // Actively moving away
                    should_decelerate = true; // Need to brake
                 }
                 // If stationary (vel=0), it will accelerate, not decelerate.
            }

            double new_velocity_rad_per_cycle;
            if (should_decelerate) {
                // Decelerate: change velocity towards zero
                new_velocity_rad_per_cycle = current_vel_rad_per_cycle - sign(current_vel_rad_per_cycle) * accel_change_in_velocity_per_cycle_;
                // If deceleration crosses zero, cap at zero if we were moving towards target
                if (sign(current_vel_rad_per_cycle) == direction_to_target && sign(new_velocity_rad_per_cycle) != sign(current_vel_rad_per_cycle) && sign(current_vel_rad_per_cycle) != 0) {
                    new_velocity_rad_per_cycle = 0.0;
                }
            } else { // Accelerate or cruise
                double effective_acceleration_rate = accel_change_in_velocity_per_cycle_;
                if (i == 1) { // Joint A2 (index 1) is sensitive during startup
                    effective_acceleration_rate /= 3.0; // Reduce acceleration for joint A2 by half
                }
                new_velocity_rad_per_cycle = current_vel_rad_per_cycle + direction_to_target * effective_acceleration_rate;
            }

            // Clamp velocity to max speed per cycle
            new_velocity_rad_per_cycle = std::clamp(new_velocity_rad_per_cycle, 
                                                    -max_increment_rad_per_cycle_, 
                                                    max_increment_rad_per_cycle_);
            
            current_joint_velocities_rad_per_cycle_[i] = new_velocity_rad_per_cycle;

            // Update position: pos_new = pos_old + vel_new * dt (dt is 1 cycle here, so vel is delta_pos)
            double tentative_new_position = current_pos_rad + new_velocity_rad_per_cycle;

            // Check for overshoot and snap to target if crossed
            if ( (direction_to_target > 0 && tentative_new_position > target_pos_rad) ||
                 (direction_to_target < 0 && tentative_new_position < target_pos_rad) ) 
            {
                current_commanded_positions_rad_[i] = target_pos_rad;
                current_joint_velocities_rad_per_cycle_[i] = 0.0; // Stop at target
            } else {
                current_commanded_positions_rad_[i] = tentative_new_position;
            }
        }

        iface_.setJointPositionsRad(current_commanded_positions_rad_);
        
        // Log what's being sent to RSI
        std::string sent_to_rsi_str = "update_motion: Sent to RSI [";
        for (int i = 0; i < 6; ++i) {
            sent_to_rsi_str += std::to_string(current_commanded_positions_rad_[i]) + (i == 5 ? "" : ", ");
        }
        sent_to_rsi_str += "]";
        RCLCPP_DEBUG(this->get_logger(), "%s", sent_to_rsi_str.c_str());


        // Optional: Log when all joints reach target
        // if (all_joints_at_target && !prev_all_joints_at_target_) {
        //     RCLCPP_INFO(this->get_logger(), "All joints reached target positions.");
        // }
        // prev_all_joints_at_target_ = all_joints_at_target;
    }

    // Member variables
    rsi::KukaRsiInterface iface_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joint_cmd_sub_;
    rclcpp::TimerBase::SharedPtr motion_timer_;

    Eigen::ArrayXd current_commanded_positions_rad_; // Current positions being sent to RSI
    Eigen::ArrayXd target_positions_rad_;            // Goal positions from ROS topic
    Eigen::ArrayXd current_joint_velocities_rad_per_cycle_; // Velocity for each joint in rad/cycle

    std::atomic<bool> initial_positions_set_ {false};
    // bool prev_all_joints_at_target_ = true; // For logging when target is reached

    // --- Motion parameters ---
    // CYCLE_TIME_S_ should match the timer period (e.g., 10ms -> 100Hz)
    static constexpr double CYCLE_TIME_S_ = 0.004; 
    // Max velocity: e.g., 300 deg/s 
    static constexpr double MAX_VELOCITY_RAD_PER_S_ = 300.0 * DEG2RAD; // approx 0.52 rad/s
    // Max acceleration: e.g., 100 deg/s^2
    static constexpr double MAX_ACCELERATION_RAD_PER_S2_ = 10.0 * DEG2RAD; // approx 1.74 rad/s^2

    // --- Derived per-cycle parameters ---
    // Max change in position per cycle (max speed per cycle)
    static constexpr double max_increment_rad_per_cycle_ = MAX_VELOCITY_RAD_PER_S_ * CYCLE_TIME_S_;
    // Max change in velocity per cycle (acceleration effect on velocity per cycle)
    static constexpr double accel_change_in_velocity_per_cycle_ = MAX_ACCELERATION_RAD_PER_S2_ * CYCLE_TIME_S_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KukaRsiRos2Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}