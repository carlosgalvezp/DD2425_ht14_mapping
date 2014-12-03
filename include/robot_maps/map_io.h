#ifndef MAP_IO_H
#define MAP_IO_H

#include <fstream>
#include <ras_utils/ras_names.h>
// ROS
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

// OpenCV
#include <opencv2/highgui/highgui.hpp>
class Map_IO
{
public:
    Map_IO();

    bool saveMap(const std::string &path_img,
                 const std::string &path_metadata,
                 nav_msgs::OccupancyGrid &map);

    bool loadMap(const std::string &path_img,
                 const std::string &path_metadata,
                       nav_msgs::OccupancyGrid &map);
};
#endif // MAP_IO_H
