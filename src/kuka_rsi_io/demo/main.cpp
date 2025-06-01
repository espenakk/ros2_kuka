#include "rsi/kukarsiinterface.h"

#include <spdlog/spdlog.h>

int main()
{
    Eigen::ArrayXd home_position_deg(6u);
    home_position_deg << 0.0, -90.0, 90.0, 0.0, 0.0, 0.0;
    bool hasR = false;

    rsi::KukaRsiInterface iface("192.168.1.50", 49153, 6u, [&](const Eigen::ArrayXd &joint_positions_rad)
    {
        // Debug
        // std::stringstream ss;
        // ss << "Received joint positions: " << joint_positions_rad.transpose() << std::endl;
        // spdlog::info(ss.str());
        hasR = true;
        // Create a ROS publisher here for the joint positions.
        // !!!!!! Be very mindful of conventions for degrees and radians !!!!!!
    });
    iface.start();
    iface.setJointPositionsRad(home_position_deg * DEG2RAD);

    // Do the work here, update the joint position setpoints as needed.
    // Subscribe to desired joint positions through a ROS topic and call setJointPositionsRad() when a message has been received
    // Should be part of a ROS node which will keep the main thread alive through rclcpp::spin() or similar.
    // !!!!!! Be very mindful of conventions for degrees and radians !!!!!!

    // ...existing code...
    // !!!!!! Be very mindful of conventions for degrees and radians !!!!!!

    double a = 0.0;
    bool fwd = true;
    double current_increment = 0.0;
    const double max_increment = 0.1; // Max speed/increment per cycle
    const double acceleration_step = 0.0006; // How much to change speed per cycle
    const double limit_deg = 35.0; // Target +/- limit for 'a'

    // Calculate the distance needed to decelerate from max_increment to 0
    // Formula: distance = max_speed^2 / (2 * acceleration_rate)
    const double deceleration_distance_needed = (max_increment * max_increment) / (2.0 * acceleration_step);

    while(true)
    {
        
        if (hasR){

            Eigen::ArrayXd home_position_deg2(6u);
            home_position_deg2 << 0.0+a, -90.0+fabs(a), 90.0+a, 0.0+a*0.4, 0.0+a*0.1, 0.0+a*0.1;
            // home_position_deg2 << 0.0, -90.0, 90.0+a, 0.0, 0.0, 0.0;
            //Add an infinite while-loop to test communication with the robot controller.
            // It should be configured to look for the IP address on line 10 by default.
            // If nothing crashes, the robot starts with a click, no errors on the teachpad, and things seem to run - then it works!
            // "All that remains" is to interface this into the rest of your system.
            
            bool turn_around = false;
            // Check if 'a' has reached or passed the limits
            if (fwd && a >= limit_deg) {
                fwd = false;
                turn_around = true;
                a = limit_deg; // Clamp position precisely at the limit
            } else if (!fwd && a <= -limit_deg) {
                fwd = true;
                turn_around = true;
                a = -limit_deg; // Clamp position precisely at the limit
            }

            if (turn_around) {
                current_increment = 0.0; // Stop movement and prepare for acceleration in the new direction
            } else {
                // Not turning around yet, so either accelerate, cruise, or decelerate.
                bool needs_to_decelerate = false;
                if (fwd) { // Moving towards positive limit
                    // Start decelerating if 'a' is within the deceleration distance from the positive limit
                    if (a >= limit_deg - (deceleration_distance_needed * 1.5)) { // Start decelerating earlier
                        needs_to_decelerate = true;
                    }
                } else { // Moving towards negative limit
                    // Start decelerating if 'a' is within the deceleration distance from the negative limit
                    if (a <= -limit_deg + (deceleration_distance_needed * 1.5)) { // Start decelerating earlier
                        needs_to_decelerate = true;
                    }
                }

                if (needs_to_decelerate) {
                    // Decelerate
                    current_increment -= acceleration_step;
                    if (current_increment < 0.0) {
                        current_increment = 0.0; // Speed cannot be negative
                    }
                } else {
                    // Accelerate or maintain max speed
                    if (current_increment < max_increment) {
                        current_increment += acceleration_step;
                        if (current_increment > max_increment) {
                            current_increment = max_increment;
                        }
                    }
                }
            }

            // Update position 'a' based on current direction and increment
            if (fwd) {
                a += current_increment;
            } else {
                a -= current_increment;
            }
            
            // Final clamping, mainly as a safeguard if logic above has tiny overshoots
            // before turn_around is set in the *next* cycle.
            if (a > limit_deg && fwd) a = limit_deg;
            if (a < -limit_deg && !fwd) a = -limit_deg;

            iface.setJointPositionsRad(home_position_deg2 * DEG2RAD);
        }else{
            // std::this_thread::sleep_for(std::chrono::milliseconds(3000));
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return 0;
}