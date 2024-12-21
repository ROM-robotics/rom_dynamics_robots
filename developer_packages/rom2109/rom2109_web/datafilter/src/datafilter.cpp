#include <cinttypes>
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class datafilter : public rclcpp::Node
{
  public:
    datafilter()
    : Node("datafilter")
    {
      cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel_web_to_twist", 10, std::bind(&datafilter::listener_callback, this, _1));
      cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("/diff_controller/cmd_vel_unstamped", 10);
    //   publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    //   timer_ = this->create_wall_timer(
    //   500ms, std::bind(&datafilter::timer_callback, this));
    }

  private:
    void listener_callback(const geometry_msgs::msg::Twist & raw_msg) const{
        double x = 0;
        double z = 0;
        if(raw_msg.linear.x > 0){x = 0.10000; z = 0.000000;}
        else if(raw_msg.linear.x < 0 && raw_msg.angular.z < 0){x = 0.00000; z = -0.70000;}
        else if(raw_msg.linear.x < 0 && raw_msg.angular.z > 0){x = 0.00000; z = 0.70000;}
        else{x = 0; z = 0;}
        // if(raw_msg.angular.x < 0.8){raw_msg.angular.x = 0;}
        RCLCPP_INFO(this->get_logger(), "I heard: x:%.3f z:%.3f\n", x, z);

        auto message = geometry_msgs::msg::Twist();
        message.linear.x = x;
        message.angular.z = z;
        cmd_vel_pub->publish(message);

    }
    // void timer_callback()
    // {
    //   auto message = std_msgs::msg::String();
    //   message.data = "Hello, world! " + std::to_string(count_++);
    //   RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    //   publisher_->publish(message);
    // }
    rclcpp::TimerBase::SharedPtr timer_;
    //rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
    
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<datafilter>());
  rclcpp::shutdown();
  return 0;
}