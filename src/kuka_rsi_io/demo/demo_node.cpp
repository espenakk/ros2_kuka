#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <Eigen/Core>
#include "rsi/kukarsiinterface.h"

using std::placeholders::_1;

class KukaRsiRos2Node : public rclcpp::Node
{
public:
    KukaRsiRos2Node()
    : Node("kuka_rsi_ros2_node"),
      iface_("192.168.1.50", 49153, 6, std::bind(&KukaRsiRos2Node::on_joint_state, this, std::placeholders::_1))
    {
        // Publisher for joint states
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

        // Subscriber for joint position commands (in radians)
        joint_cmd_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "joint_position_command", 10,
            std::bind(&KukaRsiRos2Node::on_joint_command, this, _1));

        // Start RSI interface
        iface_.start();
    }

    ~KukaRsiRos2Node() {
        iface_.stop();
    }

private:
    // Callback from RSI interface (robot joint positions)
    void on_joint_state(const Eigen::ArrayXd &joint_positions_rad)
    {
        auto msg = sensor_msgs::msg::JointState();
        msg.header.stamp = this->now();
        msg.name = {"joint_a1", "joint_a2", "joint_a3", "joint_a4", "joint_a5", "joint_a6"};
        msg.position.resize(joint_positions_rad.size());
        for (int i = 0; i < joint_positions_rad.size(); ++i)
            msg.position[i] = joint_positions_rad[i];
        joint_state_pub_->publish(msg);
    }

    // Callback from ROS topic (desired joint positions)
    void on_joint_command(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (msg->data.size() != 6) {
            RCLCPP_WARN(this->get_logger(), "Expected 6 joint positions, got %zu", msg->data.size());
            return;
        }
        Eigen::ArrayXd setpoints(6);
        for (size_t i = 0; i < 6; ++i)
            setpoints[i] = msg->data[i];
        iface_.setJointPositionsRad(setpoints);
    }

    rsi::KukaRsiInterface iface_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joint_cmd_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KukaRsiRos2Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}