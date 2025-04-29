#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <cmath>

class TestBallPublisher : public rclcpp::Node
{
public:
    TestBallPublisher()
        : Node("test_ball_publisher"), t_(0.0)
    {
        pub_ = create_publisher<geometry_msgs::msg::Point>("/ball/position", 10);

        using namespace std::chrono_literals;
        period_ = 50ms; // 20 Hz  →  dt = 0.05 s
        timer_ = create_wall_timer(period_,
                                   std::bind(&TestBallPublisher::timerCallback, this));
    }

private:
    void timerCallback()
    {
        geometry_msgs::msg::Point p;
        p.x = 30.0 - 21.0 * t_;                    // ≈10 m over the 1.4 s fall
        p.y = 0.3 * std::cos(0.5 * t_);    // small sideways wave
        p.z = 10.0 - 0.5 * 9.81 * t_ * t_; // free fall from 10 m

        // restart when the ball hits the floor
        if (p.z <= -10.0)
        {
            t_ = 0.0;
            p.z = 10.0; // reset height immediately
        }

        pub_->publish(p);
        t_ += std::chrono::duration<double>(period_).count(); // == 0.05 s
    }

    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::chrono::milliseconds period_;
    double t_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TestBallPublisher>());
    rclcpp::shutdown();
    return 0;
}
