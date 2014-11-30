#include <ros/ros.h>
#include <ras_utils/ras_names.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Bool.h>
#include <ras_utils/basic_node.h>
#include <ras_utils/occupancy_map_utils.h>
#include <sstream>
#include <cstdlib>


#define QUEUE_SIZE      1


class MapSaverNode : rob::BasicNode
{
public:

    MapSaverNode()
    {
        // Subscriber
        map_save_sub_ = n.subscribe<std_msgs::Bool>(TOPIC_MAP_SAVE, QUEUE_SIZE,&MapSaverNode::saveMapCallback, this);

        last_saving_time_ = ros::WallTime::now();
        map_counter_ = 0;
    }

private:

    // ** Publishers and subscribers
    ros::Subscriber map_save_sub_;

    int map_counter_;
    ros::WallTime last_saving_time_;

    void saveMapCallback(const std_msgs::Bool::ConstPtr &msg)
    {

        if (msg->data)
        {
            ROS_ERROR("SAVING MAP");
            // ** Save raw map
            std::stringstream ss;
            ss << "rosrun map_server map_saver map:="<<TOPIC_MAP_OCC_GRID<<" -f "<<RAS_Names::MAP_ROOT_PATH<<"map_raw_"<<map_counter_;
            system(ss.str().c_str());

            ros::Duration(5.0).sleep();

            // ** Save thick map
            std::stringstream ss2;
            ss2 << "rosrun map_server map_saver map:="<<TOPIC_MAP_OCC_GRID_THICK<<" -f "<<RAS_Names::MAP_ROOT_PATH<<"map_thick_"<<map_counter_;
            system(ss2.str().c_str());

            ++map_counter_;
        }
    }

};

int main(int argc, char **argv) {
    ros::init(argc, argv, "map_handler_node");

    MapSaverNode msn;
    ros::spin();

    return 0;
}
