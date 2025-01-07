#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <cmath> // For atan2 and M_PI

QPointF initialScenePoint; // Store the initial point
auto clicked_pose = geometry_msgs::msg::PoseStamped();


extern void shutdown_thread();

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent)
{
    setWindowTitle("ROM Dynamics Company's Robot Suite");
    ui = std::make_shared<Ui::MainWindow>();
    ui->setupUi(this);

    //label = ui->subscribedText; // to delete
    clicked_pose.header.frame_id = "map";

    node_ = rclcpp::Node::make_shared("rom_qt");
    mode_publisher_ = node_->create_publisher<std_msgs::msg::String>("switch_mode", 10);
    pose_publisher_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("pose", 10);

    connect(ui->mappingBtn, &QPushButton::clicked, this, &MainWindow::sendMappingMode);
    connect(ui->navigationBtn, &QPushButton::clicked, this, &MainWindow::sendNavigationMode);
    connect(ui->remappingBtn, &QPushButton::clicked, this, &MainWindow::sendRemappingMode);

    ui->shutdownBtn->setStyleSheet(
    "QPushButton:pressed {"
    "    background-color: rgb(255, 200, 200);"        // Background color when pressed
    "}"
    );
    ui->btnEstop->setStyleSheet(
    "QPushButton:pressed {"
    "    background-color: rgb(255, 200, 200);"        // Background color when pressed
    "}"
    );
    ui->btnStop->setStyleSheet(
    "QPushButton:pressed {"
    "    background-color: rgb(255, 200, 200);"        // Background color when pressed
    "}"
    );

    ui->btnForward->setStyleSheet(
    "QPushButton:pressed {"
    "    background-color: rgb(200, 255, 200);"        // Background color when pressed
    "}"
    );
    ui->btnLeft->setStyleSheet(
    "QPushButton:pressed {"
    "    background-color: rgb(200, 255, 200);"        // Background color when pressed
    "}"
    );
    ui->btnRight->setStyleSheet(
    "QPushButton:pressed {"
    "    background-color: rgb(200, 255, 200);"        // Background color when pressed
    "}"
    );

    current_mode_ = "navi";

    qRegisterMetaType<nav_msgs::msg::OccupancyGrid::SharedPtr>("nav_msgs::msg::OccupancyGrid::SharedPtr");
}


void MainWindow::mousePressEvent(QMouseEvent *event) {
    if (!ui->graphicsView) {
        ui->statusLabel->setText("GraphicsView is null.");
        return;
    }

    if (!ui->graphicsView->scene()) {
        ui->statusLabel->setText("GraphicsView has no scene."); 
        return;
    }
    // Get the position of the click relative to the QGraphicsView
    QPoint viewPoint = ui->graphicsView->mapFrom(this, event->pos());

    // Map the QGraphicsView coordinates to the QGraphicsScene coordinates
    QPointF scenePoint = ui->graphicsView->mapToScene(viewPoint);

    // Check if the scenePoint is within the scene's bounds
    QRectF sceneBounds = ui->graphicsView->scene()->sceneRect();
    if (!sceneBounds.contains(scenePoint)) {
        // If the point is outside the scene, ignore it
        ui->statusLabel->setText("Click outside scene bounds ignored.");
        //return;
    }
    else {
        ui->statusLabel->setText("Click inside scene bounds.");

        // Convert the scene coordinates to map coordinates
    double mapX = (scenePoint.x() * this->map_resolution_) + this->map_origin_x_;
    double mapY = (scenePoint.y() * this->map_resolution_) + this->map_origin_y_;

    // Publish the pose
    
    clicked_pose.header.stamp = rclcpp::Clock().now();
    clicked_pose.pose.position.x = mapX;
    clicked_pose.pose.position.y = mapY;
    clicked_pose.pose.orientation.w = 1.0; // Neutral orientation
    pose_publisher_->publish(clicked_pose);

    // Update the status label with the calculated position
    ui->statusLabel->setText(QString::asprintf("x: %.5f, y: %.5f, phi: %d", mapX, mapY, 0));
    }
}



void MainWindow::updateMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) 
{
    double map_origin_x   = msg->info.origin.position.x;
    double map_origin_y   = msg->info.origin.position.y;
    double map_resolution = msg->info.resolution;

    // Convert OccupancyGrid to an image
    QImage mapImage(msg->info.width, msg->info.height, QImage::Format_RGB888);
    for (size_t y = 0; y < msg->info.height; ++y) {
        for (size_t x = 0; x < msg->info.width; ++x) {
            int index = y * msg->info.width + x;
            int value = msg->data[index];
            QColor color = (value == 0) ? Qt::white : (value == 100) ? Qt::black : Qt::gray;
            mapImage.setPixel(x, y, color.rgb());
        }
    }
    this->map_resolution_ = map_resolution;
    this->map_origin_x_ = map_origin_x;
    this->map_origin_y_ = map_origin_y;

    QGraphicsScene *scene = new QGraphicsScene(this);
    scene->addPixmap(QPixmap::fromImage(mapImage));
    ui->graphicsView->setScene(scene);
    
    ui->graphicsView->fitInView(scene->sceneRect(), Qt::KeepAspectRatio);
}


void MainWindow::DisplaySubscription(const QString &log)
{
    //ui->statusLabel->setText("hello"); there is no publisher
}


void MainWindow::sendMappingMode() {

    if( current_mode_ == "mapping" ) {
        return;
    } else 
    {
        current_mode_ = "mapping";

        auto msg = std_msgs::msg::String();
        msg.data = "mapping";

        mode_publisher_->publish(msg);
        ui->statusLabel->setText("Changing Mapping Mode...");
        ui->mappingBtn->setStyleSheet("background-color: green;");
        ui->navigationBtn->setStyleSheet("background-color: none;");
        ui->remappingBtn->setStyleSheet("background-color: none;");
    }
}


void MainWindow::sendNavigationMode() {
    if( current_mode_ == "navi" ) {
        return;
    } else 
    {
        current_mode_ = "navi";

        auto msg = std_msgs::msg::String();
        msg.data = "navi";

        mode_publisher_->publish(msg);
        ui->statusLabel->setText("Changing Navigation Mode...");
        ui->mappingBtn->setStyleSheet("background-color: none;");
        ui->navigationBtn->setStyleSheet("background-color: green;");
        ui->remappingBtn->setStyleSheet("background-color: none;");
    }
}


void MainWindow::sendRemappingMode() {
    if(current_mode_ == "remapping") {
        return;
    } else 
    {
        current_mode_ = "remapping";
    
        auto msg = std_msgs::msg::String();
        msg.data = "remapping";

        mode_publisher_->publish(msg);
        ui->statusLabel->setText("Changing Remapping Mode...");
        ui->mappingBtn->setStyleSheet("background-color: none;");
        ui->navigationBtn->setStyleSheet("background-color: none;");
        ui->remappingBtn->setStyleSheet("background-color: green;");
    }
}


void MainWindow::on_shutdownBtn_clicked()
{
    ui->statusLabel->setText("Shutdown 0 ...");

    shutdown_thread();

    // Perform any additional cleanup if needed
    rclcpp::shutdown(); // Stop the ROS 2 node and executor

    QApplication::quit(); // Exit the application
}


void MainWindow::on_btnEstop_clicked()
{
  ui->statusLabel->setText("Activating E-Stop ...");
}


