#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include <Eigen/Core>

class SimpleController : public rclcpp::Node
{
public:
    SimpleController(const std::string &name);
private:
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_cmd_pub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vel_sub_;

    double wheel_radius_;
    double wheel_separation_;

    void velCallback(const geometry_msgs::msg::TwistStamped & msg);

    Eigen::Matrix2d speed_conversion_;
};