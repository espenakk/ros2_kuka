#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <ris_msgs/msg/hello.hpp>

std::atomic<bool> run = true;
std::shared_ptr<rclcpp::Node> node;

void hello_message_received(std::shared_ptr<ris_msgs::msg::Hello> msg)
{
    RCLCPP_INFO(node->get_logger(), "/hello_subscriber received msg: %s", msg->text.c_str());
}

int main(int argc, char **args)
{
    rclcpp::init(argc, args);
    node = std::make_shared<rclcpp::Node>("ris", "hello_subscriber");
    node->get_logger().set_level(rclcpp::Logger::Level::Debug);
    RCLCPP_INFO(node->get_logger(), "/hello_subscriber node started");
    auto subscriber = node->create_subscription<ris_msgs::msg::Hello>("/hello_publisher/message", 10u, &hello_message_received);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
