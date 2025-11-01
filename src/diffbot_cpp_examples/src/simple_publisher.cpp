#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

class SimplePublisher : public rclcpp::Node
{
public:
    SimplePublisher() : Node("simple_publisher"), counter_(0)
    {
        pub_ = create_publisher<std_msgs::msg::String>("chatter", 10);

        timer_ = create_wall_timer(1s, std::bind(&SimplePublisher::timerCallback, this));
    }
private:
    unsigned int counter_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void timerCallback()
    {
        auto msg = std_msgs::msg::String();
        msg.data = "Hello ROS2 - counter: " + std::to_string(counter_++);
        pub_->publish(msg);
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto simple_publisher = std::make_shared<SimplePublisher>();
    rclcpp::spin(simple_publisher);
    rclcpp::shutdown();
    return 0;
}

