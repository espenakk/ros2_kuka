#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>

class TestBallPublisher : public rclcpp::Node
{
public:
    TestBallPublisher()
    : Node("test_ball_publisher"), t_(0.0)
    {
        pub_ = create_publisher<geometry_msgs::msg::Point>("/ball/position", 10);
        timer_ = create_wall_timer(std::chrono::milliseconds(50), std::bind(&TestBallPublisher::timerCallback, this));
    }

private:
    void timerCallback()
    {
        geometry_msgs::msg::Point msg;
        msg.x = 0.5 * t_;          // move slowly in x
        msg.y = 0.0;               // no y movement
        msg.z = 5.0 - 0.5 * 9.81 * t_ * t_; // fall under gravity

        pub_->publish(msg);

        t_ += 0.05; // simulate 20 Hz publishing
    }

    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    double t_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TestBallPublisher>());
    rclcpp::shutdown();
    return 0;
}
