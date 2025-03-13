#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **args)
{
    rclcpp::init(argc, args);
    auto node = std::make_shared<rclcpp::Node>("ris", "hello_world");
    node->get_logger().set_level(rclcpp::Logger::Level::Debug);
    RCLCPP_INFO(node->get_logger(), "/hello_world node started");

    // Do what you need to do to set up the application here.

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
