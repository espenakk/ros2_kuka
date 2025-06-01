#include "kuka_moveit/PlanExecutionNode.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PlanExecutionNode>();
    node->initMoveit();
    rclcpp::spin(node);
    rclcpp::shutdown();
}
