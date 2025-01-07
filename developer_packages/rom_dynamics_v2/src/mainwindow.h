
#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QtWidgets>
#include <QtWidgets/QMainWindow>
#include <QLabel>

#include "ui_mainwindow.h"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

QT_BEGIN_NAMESPACE
namespace Ui
{
    class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

    public:
        explicit MainWindow(QWidget *parent = nullptr);
        //~MainWindow() { delete ui;}
        std::shared_ptr<Ui::MainWindow> getUi() { return ui; }
        void mousePressEvent(QMouseEvent *event);
    
    public slots:
        void DisplaySubscription(const QString &log);
        void updateMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
        
        void sendMappingMode();
        void sendNavigationMode();
        void sendRemappingMode();
        
    private slots:
        void on_shutdownBtn_clicked();
        void on_btnEstop_clicked();
        
    
    private:
        std::shared_ptr<Ui::MainWindow> ui = nullptr;
        QLabel* label = nullptr;

        rclcpp::Node::SharedPtr node_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_publisher_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;


        std::string current_mode_;

        // map
        double map_origin_x_ = 0;
        double map_origin_y_ = 0;
        double map_resolution_ = 0;
        // int map_width_ = 0;
        // int map_height_ = 0;
};

#endif // MAINWINDOW_H
