#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class SimpleSubscriberNode : public rclcpp::Node
{
public:
    SimpleSubscriberNode() : Node("simple_subscribe")
    {
        simple_subscriber_ = create_subscription<std_msgs::msg::String>("chatter", 10, std::bind(&SimpleSubscriberNode::msgCallback, this, std::placeholders::_1)); 
    }

private:
    void msgCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO_STREAM(get_logger(), "I heard: " << msg->data);
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr simple_subscriber_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto simple_subscriber_node = std::make_shared<SimpleSubscriberNode>();
    rclcpp::spin(simple_subscriber_node);
    rclcpp::shutdown();
    return 0;
}