#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class SimpleSubscriber : public rclcpp::Node
{
public:
    SimpleSubscriber() : Node("simple_subscriber")
    {
        sub_ = create_subscription<std_msgs::msg::String>("chatter", 10,
            std::bind(&SimpleSubscriber::msgCallback, this, std::placeholders::_1));
    }
private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
    void msgCallback(const std_msgs::msg::String &msg ) const
    {
        RCLCPP_INFO_STREAM(get_logger(), "I Heard : " << msg.data.c_str());
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto simple_subscriber = std::make_shared<SimpleSubscriber>();
    rclcpp::spin(simple_subscriber);
    rclcpp::shutdown();
    return 0;
}