#include <ros/ros.h>
#include <ras_utils/ras_names.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ras_utils/basic_node.h>
#include <mapping/map_io.h>

#define QUEUE_SIZE      1
#define PUBLISH_RATE    10

class MapLoaderNode : rob::BasicNode
{
public:

    MapLoaderNode();
    void run();
private:

    // ** Publishers and subscribers
    ros::Publisher map_pub_thick_;
    nav_msgs::OccupancyGrid map_msg_;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "map_loader_node");

    MapLoaderNode mln;

    mln.run();

    return 0;
}

// ===============================================================================
// ===============================================================================
MapLoaderNode::MapLoaderNode()
{
    // Publisher
    map_pub_thick_ = n.advertise<nav_msgs::OccupancyGrid>(TOPIC_MAP_OCC_GRID_THICK, QUEUE_SIZE);

    // Read map from disk
    Map_IO map_io;
    map_io.loadMap(RAS_Names::THICK_MAP_DATA_PATH,
                   RAS_Names::THICK_MAP_METADATA_PATH, map_msg_);
}

void MapLoaderNode::run()
{
    ros::Rate loop_rate(PUBLISH_RATE);
    while(ros::ok())
    {
        map_pub_thick_.publish(map_msg_);
        // ** Sleep
        ros::spinOnce();
        loop_rate.sleep();
    }
}
