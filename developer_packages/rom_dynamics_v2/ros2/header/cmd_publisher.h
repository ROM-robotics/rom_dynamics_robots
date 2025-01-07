#ifndef PUBLISHER_H
#define PUBLISHER_H

#include <rclcpp/rclcpp.hpp>
//#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <QObject>
#include <QtWidgets/QApplication>

class Publisher : public QObject, public rclcpp::Node
{
    Q_OBJECT

    public:
        explicit Publisher(const std::string &topic_name);

    public slots:
        //void DisplaySubscription(const QString &log);
        void setForward();
        void setLeft();
        void setRight();
        void setStop();

    private:
        std::shared_ptr<geometry_msgs::msg::Twist> message_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publish_;
        rclcpp::TimerBase::SharedPtr timer_;

        std::string drive_command_;
    };

#endif // PUBLISHER_H
