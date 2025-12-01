#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

class SimpleParameterNode : public rclcpp::Node
{
public:
    SimpleParameterNode() : Node("simple_parameter")
    {
        declare_parameter<int>("simple_int_param", 10);
        declare_parameter<std::string>("simple_str_param", "Diffbot Ros");

        param_callback_handle_ = add_on_set_parameters_callback(std::bind(&SimpleParameterNode::paramChangeCallback, this, std::placeholders::_1));
    }
private:
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    rcl_interfaces::msg::SetParametersResult paramChangeCallback(const std::vector<rclcpp::Parameter> & parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;

        for(const auto& param: parameters)
        {
            if (param.get_name() == "simple_int_param" and param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
            {
                RCLCPP_INFO_STREAM(get_logger(), "Param simple_int_param have Changed! New Value is : " << param.as_int());
                result.successful = true;
            }

            if(param.get_name() == "simple_str_param" and param.get_type() == rclcpp::ParameterType::PARAMETER_STRING)
            {
                RCLCPP_INFO_STREAM(get_logger(), "Param simple_str_param have Changed! New Values is : " << param.as_string());
                result.successful = true;
            }
        }
        return result;
    }

};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto simple_parameter = std::make_shared<SimpleParameterNode>();
    rclcpp::spin(simple_parameter);
    rclcpp::shutdown();
    return 0;
}