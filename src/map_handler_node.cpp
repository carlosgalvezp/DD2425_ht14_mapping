#include <ros/ros.h>
#include <robot_maps/map_handler.h>
#include <ras_utils/ras_names.h>
#include <robot_maps/map.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose2D.h>
#include <ras_utils/basic_node.h>
#include <ras_arduino_msgs/ADConverter.h>
#include <ras_utils/occupancy_map_utils.h>
#include <sstream>
#include <cstdlib>

#include <ras_utils/occupancy_map_utils.h>

#define QUEUE_SIZE      1
#define PUBLISH_RATE    10

// ** Map specific constants
#define MAP_HEIGHT      1000    // Corresponds to amount of cells
#define MAP_WIDTH       1000    // Corresponds to amount of cells
#define MAP_CELL_SIZE   1.0     // Width and Height in // Corresponds to amount of cells CM per Cell

#define TIME_SAVE_MAP   30.0    // [s] After this time interval passes, we save the map. E.g.: save map every 30 seconds

class MapHandlerNode : rob::BasicNode
{
public:

    MapHandlerNode() : mapHandler(Map(MAP_HEIGHT, MAP_WIDTH, MAP_CELL_SIZE))
    {
        // Publisher
        map_pub_ = n.advertise<nav_msgs::OccupancyGrid>(TOPIC_MAP_OCC_GRID, QUEUE_SIZE);
        map_pub_thick_ = n.advertise<nav_msgs::OccupancyGrid>(TOPIC_MAP_OCC_GRID_THICK, QUEUE_SIZE);
        save_map_pub_ = n.advertise<std_msgs::Bool>(TOPIC_MAP_SAVE,QUEUE_SIZE);
        // Subscriber
        odo_sub_ = n.subscribe(TOPIC_ODOMETRY, 1,  &MapHandlerNode::odoCallback, this);
        adc_sub_ = n.subscribe(TOPIC_ARDUINO_ADC, 1,  &MapHandlerNode::adcCallback, this);

        last_saving_time_ = ros::WallTime::now();
        map_counter_ = 0;
    }

    void run()
    {
        ros::Rate loop_rate(PUBLISH_RATE);
        while(ros::ok())
        {
            if(odo_data_ != nullptr && adc_data_ != nullptr) {
                mapHandler.update(odo_data_, adc_data_);

                {


                    // ** Publish
                    nav_msgs::OccupancyGrid msg_raw, msg_thick;
                    // Raw map
                    msg_raw.header.frame_id = COORD_FRAME_WORLD;
                    msg_raw.info.origin.position.x = - (mapHandler.getWidth() / 100) / 2.0;
                    msg_raw.info.origin.position.y = - (mapHandler.getHeight() / 100) / 2.0;
                    msg_raw.data = (&mapHandler.getMap())[0];
                    msg_raw.info.height = mapHandler.getHeight();
                    msg_raw.info.width = mapHandler.getWidth();
                    msg_raw.info.resolution = mapHandler.getCellSize() / 100.0;
                    map_pub_.publish(msg_raw);

                    // Thick map
                    msg_thick.header.frame_id = COORD_FRAME_WORLD;
                    msg_thick.info.origin.position.x = - (mapHandler.getWidth() / 100) / 2.0;
                    msg_thick.info.origin.position.y = - (mapHandler.getHeight() / 100) / 2.0;
                    msg_thick.data = (&mapHandler.getThickMap())[0];
                    msg_thick.info.height = mapHandler.getHeight();
                    msg_thick.info.width = mapHandler.getWidth();
                    msg_thick.info.resolution = mapHandler.getCellSize() / 100.0;
                    map_pub_thick_.publish(msg_thick);

                    // ** Save the map if necessary
                    ros::WallTime current_t = ros::WallTime::now();
                    if( (current_t.toSec() - last_saving_time_.toSec()) > TIME_SAVE_MAP)
                    {
                        ROS_INFO(" ===== PUBLISHING SAVE MAP ==== ");
                        std_msgs::Bool msg;
                        msg.data = true;
                        save_map_pub_.publish(msg);
                        last_saving_time_ = current_t;
                    }

                }
            }

            // ** Sleep
            ros::spinOnce();
            loop_rate.sleep();
        }
        std::cout << "Exiting...\n";
    }

private:

    // ** Publishers and subscribers
    ros::Publisher map_pub_;
    ros::Publisher map_pub_thick_;
    ros::Publisher save_map_pub_;

    ros::Subscriber odo_sub_;
    ros::Subscriber adc_sub_;

    // ** Recieved messages from subscribers
    geometry_msgs::Pose2D::ConstPtr odo_data_;
    ras_arduino_msgs::ADConverter::ConstPtr adc_data_;

    // ** The actual map
    MapHandler mapHandler;

    // ** Variables to help saving the map every X seconds
    int map_counter_;
    ros::WallTime last_saving_time_;

    void odoCallback(const geometry_msgs::Pose2D::ConstPtr& msg) {
        odo_data_ = msg;
    }

    void adcCallback(const ras_arduino_msgs::ADConverter::ConstPtr& msg)
    {
        adc_data_ = msg;
    }

};

int main(int argc, char **argv) {
    ros::init(argc, argv, "map_handler_node");

    MapHandlerNode mhn;

    mhn.run();

    return 0;
}
