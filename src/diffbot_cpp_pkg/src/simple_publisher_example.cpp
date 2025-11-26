#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

class SimplePublisherNode : public rclcpp::Node
{
public:
    SimplePublisherNode() : Node("simple_publisher")
    {
        simple_publisher_ = create_publisher<std_msgs::msg::String>("chatter", 10);
        timer_ = create_wall_timer(1s, std::bind(&SimplePublisherNode::publishCallback, this));
    }
private:

    void publishCallback()
    {
        std_msgs::msg::String msg;
        msg.data = "Hi Ros2 : counter " + std::to_string(counter_++);
        simple_publisher_->publish(msg);
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr simple_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    unsigned int counter_;

};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto simple_publisher_node = std::make_shared<SimplePublisherNode>();
    rclcpp::spin(simple_publisher_node);
    rclcpp::shutdown();
    return 0;
}