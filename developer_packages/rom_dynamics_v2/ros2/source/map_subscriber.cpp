#include <map_subscriber.h>

MapSubscriber::MapSubscriber(QObject* parent) : QObject(parent), Node("map_handler") 
{
    map_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 10, std::bind(&MapSubscriber::mapCallback, this, std::placeholders::_1));
}


void MapSubscriber::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) 
{
    /*
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
    }*/

    //emit updateMap(mapImage, map_origin_x, map_origin_y, map_resolution); // Emit updated map as QImage
    emit updateMap(msg);

}
   