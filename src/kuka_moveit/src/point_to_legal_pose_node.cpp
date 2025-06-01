#include "kuka_moveit/PointToLegalPoseNode.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointToLegalPoseNode>();
    node->initMoveit();
    rclcpp::spin(node);
    rclcpp::shutdown();
}
