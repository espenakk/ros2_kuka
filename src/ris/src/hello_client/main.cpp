#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <ris_msgs/msg/hello.hpp>

#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <ris_msgs/srv/detail/ping_pong__builder.hpp>

std::atomic<bool> run = true;
std::shared_ptr<rclcpp::Node> node;

void send_hello(std::shared_ptr<rclcpp::Node> node)
{
    using namespace std::chrono_literals;

    auto client = node->create_client<ris_msgs::srv::PingPong>("/hello_server/ping", 10u);
    uint32_t index = 0u;
    while(run)
    {
        auto request = std::make_shared<ris_msgs::srv::PingPong::Request>();
        request->input = "Ping index " + std::to_string(index++);
        auto future = client->async_send_request(request);
        if(future.wait_for(10ms) == std::future_status::ready)
            RCLCPP_INFO(node->get_logger(), "/hello_client received: \"%s\"", future.get()->output.c_str());
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

int main(int argc, char **args)
{
    rclcpp::init(argc, args);
    node = std::make_shared<rclcpp::Node>("ris", "hello_client");
    node->get_logger().set_level(rclcpp::Logger::Level::Debug);
    RCLCPP_INFO(node->get_logger(), "/hello_client node started");

    std::thread worker([&]() { send_hello(node); });

    rclcpp::spin(node);
    run = false;
    rclcpp::shutdown();
    return 0;
}
