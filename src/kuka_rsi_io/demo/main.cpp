#include "rsi/kukarsiinterface.h"

#include <spdlog/spdlog.h>

int main()
{
    Eigen::ArrayXd home_position_deg(6u);
    home_position_deg << 0.0, -90.0, 90.0, 0.0, 0.0, 0.0;
    bool hasR = false;

    rsi::KukaRsiInterface iface("192.168.1.50", 49153, 6u, [&](const Eigen::ArrayXd &joint_positions_rad)
    {
        std::stringstream ss;
        ss << "Received joint positions: " << joint_positions_rad.transpose() << std::endl;
        spdlog::info(ss.str());
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

    double a = 0.0;
    bool fwd = true;

    while(true)
    {
        
        if (hasR){

            Eigen::ArrayXd home_position_deg2(6u);
            home_position_deg2 << 0.0+a*2.0, -90.0+fabs(a)*1.6, 90.0+a*1.6, 0.0+a, 0.0+a*2.5, 0.0+a*2.5;
            // home_position_deg2 << 0.0, -90.0, 90.0+a, 0.0, 0.0, 0.0;
            //Add an infinite while-loop to test communication with the robot controller.
            // It should be configured to look for the IP address on line 10 by default.
            // If nothing crashes, the robot starts with a click, no errors on the teachpad, and things seem to run - then it works!
            // "All that remains" is to interface this into the rest of your system.
            if (a > 30)
                fwd = false;

            if (a < -30)
                fwd = true;

            if (!fwd){
                a-=0.010;
            }else {
                a+=0.009;
            }
            iface.setJointPositionsRad(home_position_deg2 * DEG2RAD);
        }else{
            // std::this_thread::sleep_for(std::chrono::milliseconds(3000));
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return 0;
}
