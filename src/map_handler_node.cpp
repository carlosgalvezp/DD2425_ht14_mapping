#include <ros/ros.h>
#include <robot_maps/cell_map.h>
#include <nav_msgs/OccupancyGrid.h>
#include <vector>

class MapHandlerNode
{
public:
    MapHandlerNode();
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "map_handler_node");

    ros::NodeHandle n;

    ros::Publisher map_pub = n.advertise<nav_msgs::OccupancyGrid>("/map/occ_grid", 1);

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
      nav_msgs::OccupancyGrid msg;

      std::vector<int8_t> test(100);
      for(int i = 0; i < 33; i++) {
          test[i] = 100;
      }
      msg.data = test;
      msg.info.height = 10;
      msg.info.width = 10;
      msg.info.resolution = 10;

      map_pub.publish(msg);

      ros::spinOnce();

      loop_rate.sleep();
    }

    return 0;
}
