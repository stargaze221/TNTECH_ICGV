#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class CppNode : public rclcpp::Node
{
public:
    CppNode()
    : Node("cpp_node")
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic_name", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&CppNode::timer_callback, this));
    }

private:
    void timer_callback()
    {
        auto message = std_msgs::msg::String();
        message.data = "Hello from C++!";
        publisher_->publish(message);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CppNode>());
    rclcpp::shutdown();
    return 0;
}
