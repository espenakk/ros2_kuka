#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <ris_msgs/srv/ping_pong.hpp>

std::atomic<bool> run = true;
std::shared_ptr<rclcpp::Node> node;

void serve_pingpong(ris_msgs::srv::PingPong::Request::Ptr request, ris_msgs::srv::PingPong::Response::Ptr response)
{
    response->output = "Pong: " + request->input;
    RCLCPP_INFO(node->get_logger(), "/hello_server received: \"%s\"", request->input.c_str());
}

int main(int argc, char **args)
{
    rclcpp::init(argc, args);
    node = std::make_shared<rclcpp::Node>("ris", "hello_server");
    node->get_logger().set_level(rclcpp::Logger::Level::Debug);
    RCLCPP_INFO(node->get_logger(), "/hello_server node started");

    auto server = node->create_service<ris_msgs::srv::PingPong>("ping", &serve_pingpong);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
