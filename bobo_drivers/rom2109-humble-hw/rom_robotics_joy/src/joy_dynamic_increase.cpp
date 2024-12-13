#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>

class DynamicScalingNode : public rclcpp::Node {
public:
    DynamicScalingNode()
        : Node("dynamic_scaling_node"), linear_scale_(0.7), angular_scale_(0.4), increment_(0.1) {
        
        // Declare and initialize parameters
        this->declare_parameter("linear_increase_button", 5);  // Example: "Y" button
        this->declare_parameter("linear_decrease_button", 1);  // Example: "X" button
        this->declare_parameter("rotate_increase_button", 6);  // Example: "Y" button
        this->declare_parameter("rotate_decrease_button", 7);  // Example: "X" button
        this->declare_parameter("axis_linear", 1);      // Axis for linear motion
        this->declare_parameter("axis_angular", 2);     // Axis for angular motion

        // Get parameter values
        this->get_parameter("linear_increase_button", linear_increase_button_);
        this->get_parameter("linear_decrease_button", linear_decrease_button_);
        this->get_parameter("rotate_increase_button", rotate_increase_button_);
        this->get_parameter("rotate_decrease_button", rotate_decrease_button_);
        this->get_parameter("axis_linear", axis_linear_);
        this->get_parameter("axis_angular", axis_angular_);

        // Subscribe to joystick messages
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&DynamicScalingNode::joyCallback, this, std::placeholders::_1));

        // Publisher for velocity commands
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_joy", 10);

        RCLCPP_INFO(this->get_logger(), "DynamicScalingNode initialized");
    }

private:
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        // Adjust scaling based on button presses
        if (msg->buttons[linear_increase_button_]) {
            linear_scale_ += increment_;
            RCLCPP_INFO(this->get_logger(), "Increased scaling: linear=%.2f", linear_scale_);
        }
        if (msg->buttons[linear_decrease_button_]) {
            linear_scale_ = std::max(0.0, linear_scale_ - increment_);
            RCLCPP_INFO(this->get_logger(), "Decresed scaling: linear=%.2f", linear_scale_);
        }
        if (msg->buttons[rotate_increase_button_]) {
            angular_scale_ += increment_;
            RCLCPP_INFO(this->get_logger(), "Increased scaling: angular=%.2f", angular_scale_);
        }
        if (msg->buttons[rotate_decrease_button_]) {
            angular_scale_ = std::max(0.0, angular_scale_ - increment_);
            RCLCPP_INFO(this->get_logger(), "Increased scaling: angular=%.2f", angular_scale_);
        }
        

        // Generate Twist message based on joystick input
        auto twist = geometry_msgs::msg::Twist();
        twist.linear.x = linear_scale_ * msg->axes[axis_linear_];
        twist.angular.z = angular_scale_ * msg->axes[axis_angular_];

        // Publish the twist message
        cmd_vel_pub_->publish(twist);
    }

    // Scaling parameters
    double linear_scale_;
    double angular_scale_;
    const double increment_;

    // Joystick mappings
    int linear_increase_button_;
    int linear_decrease_button_;
    int rotate_increase_button_;
    int rotate_decrease_button_;
    int axis_linear_;
    int axis_angular_;

    // ROS interfaces
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DynamicScalingNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
