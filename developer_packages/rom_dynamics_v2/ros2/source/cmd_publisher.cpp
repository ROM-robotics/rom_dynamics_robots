#include "cmd_publisher.h"

Publisher::Publisher(const std::string &topic_name) : Node("qt_twist_publisher")
{
    message_ = std::make_shared<geometry_msgs::msg::Twist>();

    drive_command_ = "stop";

    auto publish_Message =
        [this]() -> void
        {
            //時刻更新
            //auto now = std::chrono::system_clock::now();
            //std::time_t now_c = std::chrono::system_clock::to_time_t(now);
            //std::string time_str = std::ctime(&now_c);
            
            static int count = 0;

            if (this->drive_command_ == "forward")
            {
                message_->linear.x = 0.400;
                message_->angular.z = 0.000;
                publish_->publish(*message_);

                count = 0;
                return;
            } 
            else if (this->drive_command_ == "left")
            {
                message_->linear.x = 0.000;
                message_->angular.z = 0.400;
                publish_->publish(*message_);

                count = 0;
                return;
            } 
            else if (this->drive_command_ == "right")
            {
                message_->linear.x = 0.000;
                message_->angular.z = -0.400;
                publish_->publish(*message_);

                count = 0;
                return;
            }
            else if(this->drive_command_ == "stop")
            {
                if(count < 5)
                {
                    message_->linear.x = 0.0000;
                    message_->angular.z = 0.0000;
                    publish_->publish(*message_);

                    count ++;
                    return;
                } else { return; }
            } 
            
        };

    publish_ = create_publisher<geometry_msgs::msg::Twist>(topic_name, 2);
    timer_ = create_wall_timer(std::chrono::milliseconds(100), publish_Message);
}

void Publisher::setForward(){this->drive_command_ = "forward";}
void Publisher::setLeft()   {this->drive_command_ = "left";}
void Publisher::setRight()  {this->drive_command_ = "right";}
void Publisher::setStop()   {this->drive_command_ = "stop";}