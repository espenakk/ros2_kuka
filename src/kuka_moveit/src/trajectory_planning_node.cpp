#include "kuka_moveit/TrajectoryPlanningNode.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryPlanningNode>();
    node->initMoveit();
    rclcpp::spin(node);
    rclcpp::shutdown();
}
