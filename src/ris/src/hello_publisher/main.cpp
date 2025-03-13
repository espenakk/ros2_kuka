#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <ris_msgs/msg/hello.hpp>

std::atomic<bool> run = true;
std::shared_ptr<rclcpp::Node> node;

void publish_hello(std::shared_ptr<rclcpp::Node> node)
{
    auto publisher = node->create_publisher<ris_msgs::msg::Hello>("message", 10u);
    uint32_t index = 0u;
    while(run)
    {
        ris_msgs::msg::Hello hello;
        hello.text = "Hello nr: " + std::to_string(index++);
        RCLCPP_INFO(node->get_logger(), "/hello_publisher/message: publishing %s", hello.text.c_str());
        publisher->publish(hello);
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

int main(int argc, char **args)
{
    rclcpp::init(argc, args);
    node = std::make_shared<rclcpp::Node>("ris", "hello_publisher");
    node->get_logger().set_level(rclcpp::Logger::Level::Debug);
    RCLCPP_INFO(node->get_logger(), "/hello_publisher node started");

    std::thread worker([&]() { publish_hello(node); });

    rclcpp::spin(node);
    run = false;
    worker.join();
    rclcpp::shutdown();
    return 0;
}
