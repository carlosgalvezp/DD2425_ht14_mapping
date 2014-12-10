#include <ros/ros.h>
#include <ras_utils/ras_names.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ras_utils/basic_node.h>
#include <std_msgs/Int64MultiArray.h>
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
    ros::Publisher map_pub_thick_,map_pub_raw_, map_pub_cost_;

    // ** This subscribers will only grab the first message read from the bag file.
    // After that, this message will be published to the same topic by the previous publishers
    ros::Subscriber map_sub_thick_, map_sub_raw_, map_sub_cost_;

    nav_msgs::OccupancyGrid::ConstPtr thick_map_msg_, raw_map_msg_;
    std_msgs::Int64MultiArray::ConstPtr cost_msg_;

    // ** Callbacks
    void rawMapCallback(const nav_msgs::OccupancyGridConstPtr &msg);
    void thickMapCallback(const nav_msgs::OccupancyGridConstPtr &msg);
    void costMapCallback(const std_msgs::Int64MultiArrayConstPtr &msg);
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
    map_pub_raw_   = n.advertise<nav_msgs::OccupancyGrid>(TOPIC_MAP_OCC_GRID, QUEUE_SIZE);
    map_pub_cost_  = n.advertise<std_msgs::Int64MultiArray>(TOPIC_MAP_COST, QUEUE_SIZE);

    // Subscriber
    map_sub_thick_ = n.subscribe(TOPIC_MAP_OCC_GRID_THICK_BAG, QUEUE_SIZE, &MapLoaderNode::thickMapCallback, this);
    map_sub_raw_   = n.subscribe(TOPIC_MAP_OCC_GRID_BAG,       QUEUE_SIZE, &MapLoaderNode::rawMapCallback, this);
    map_sub_cost_  = n.subscribe(TOPIC_MAP_COST_BAG,           QUEUE_SIZE, &MapLoaderNode::costMapCallback, this);

}

void MapLoaderNode::run()
{
    ros::Rate loop_rate(PUBLISH_RATE);
    while(ros::ok())
    {
        if(raw_map_msg_ != 0)
        {
            map_pub_raw_.publish(raw_map_msg_);
        }

        if(thick_map_msg_ != 0)
        {
            map_pub_thick_.publish(thick_map_msg_);
        }

        if(cost_msg_ != 0)
        {
            map_pub_cost_.publish(cost_msg_);
        }

        // ** Sleep
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void MapLoaderNode::rawMapCallback(const nav_msgs::OccupancyGridConstPtr &msg)
{
    this->raw_map_msg_ = msg;

    // Stop subscribing
    this->map_sub_raw_.shutdown();
}

void MapLoaderNode::thickMapCallback(const nav_msgs::OccupancyGridConstPtr &msg)
{
    this->thick_map_msg_ = msg;

    // Stop subscribing
    this->map_sub_thick_.shutdown();
}

void MapLoaderNode::costMapCallback(const std_msgs::Int64MultiArrayConstPtr &msg)
{
    this->cost_msg_ = msg;

    // Stop subscribing
    this->map_sub_cost_.shutdown();

}
