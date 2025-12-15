#include "diffbot_cpp_pkg/simple_tf_kinematics.hpp"

SimpleTfKinematics::SimpleTfKinematics(const std::string &name)
: Node(name)
{
    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    static_transform_stamped_.header.stamp = get_clock()->now();
    static_transform_stamped_.header.frame_id = "diffbot_base";
    static_transform_stamped_.child_frame_id = "diffbot_top";
    static_transform_stamped_.transform.translation.x = 0.0;
    static_transform_stamped_.transform.translation.y = 0.0;
    static_transform_stamped_.transform.translation.z = 0.2;
    static_transform_stamped_.transform.rotation.x = 0.0;
    static_transform_stamped_.transform.rotation.y = 0.0;
    static_transform_stamped_.transform.rotation.z = 0.0;
    static_transform_stamped_.transform.rotation.w = 1.0;

    static_tf_broadcaster_->sendTransform(static_transform_stamped_);

    RCLCPP_INFO_STREAM(get_logger(), "Publishing static transform between " << static_transform_stamped_.header.frame_id << " and " << static_transform_stamped_.child_frame_id);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto simple_tf_kinematics = std::make_shared<SimpleTfKinematics>("simple_tf_kinematics");
    rclcpp::spin(simple_tf_kinematics);
    rclcpp::shutdown();
    return 0;
}