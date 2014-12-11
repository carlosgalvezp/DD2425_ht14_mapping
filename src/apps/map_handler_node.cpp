#include <ros/ros.h>
#include <mapping/map_handler.h>
#include <ras_utils/ras_names.h>
#include <mapping/map.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose2D.h>
#include <ras_utils/basic_node.h>
#include <ras_arduino_msgs/ADConverter.h>
#include <ras_utils/occupancy_map_utils.h>
#include <sstream>
#include <std_msgs/Int64MultiArray.h>
#include <ras_srv_msgs/LaserScanner.h>
#include <ras_srv_msgs/IRData.h>

#include <mapping/map_io.h>
#include <ras_utils/occupancy_map_utils.h>

#include <boost/filesystem.hpp>

#include <opencv2/highgui/highgui.hpp>

#define QUEUE_SIZE      1
#define PUBLISH_RATE    10

// ** Map specific constants
#define MAP_HEIGHT      1200    // Corresponds to amount of cells
#define MAP_WIDTH       1200    // Corresponds to amount of cells
#define MAP_CELL_SIZE   1.0     // Width and Height in // Corresponds to amount of cells CM per Cell

#define TIME_SAVE_MAP   10.0    // [s] After this time interval passes, we save the map. E.g.: save map every 30 seconds

#define TIME_BEFORE_END_SAVE_MAP 10.0    // [s] We will use the map 5.0s before we made Ctrl + C

class MapHandlerNode : rob::BasicNode
{
public:

    MapHandlerNode() : new_adc_data_recieved_(false), new_laser_data_recieved_(false), mapHandler(Map(MAP_HEIGHT, MAP_WIDTH, MAP_CELL_SIZE)),
        publish_bag_counter_(0)

    {
        // Publisher
        map_pub_ = n.advertise<nav_msgs::OccupancyGrid>(TOPIC_MAP_OCC_GRID, QUEUE_SIZE);
        map_pub_thick_ = n.advertise<nav_msgs::OccupancyGrid>(TOPIC_MAP_OCC_GRID_THICK, QUEUE_SIZE);
        map_pub_cost_ = n.advertise<std_msgs::Int64MultiArray>(TOPIC_MAP_COST, QUEUE_SIZE);

        map_pub_bag_       = n.advertise<nav_msgs::OccupancyGrid>(TOPIC_MAP_OCC_GRID_BAG, QUEUE_SIZE);
        map_pub_thick_bag_ = n.advertise<nav_msgs::OccupancyGrid>(TOPIC_MAP_OCC_GRID_THICK_BAG, QUEUE_SIZE);
        map_pub_cost_bag_  = n.advertise<std_msgs::Int64MultiArray>(TOPIC_MAP_COST_BAG, QUEUE_SIZE);

        // Subscriber
        odo_sub_ = n.subscribe(TOPIC_ODOMETRY, 1,  &MapHandlerNode::odoCallback, this);
        adc_sub_ = n.subscribe(TOPIC_ARDUINO_ADC_FILTERED, 1,  &MapHandlerNode::adcCallback, this);
        las_sub_ = n.subscribe(TOPIC_OBSTACLE_LASER_MAP, 1,  &MapHandlerNode::laserCallback, this);
        obj_sub_ = n.subscribe(TOPIC_OBJECT_AS_OBSTACLE, 1, &MapHandlerNode::objCallbak, this);

        // Reset map directory

        /*
        boost::filesystem::remove_all(RAS_Names::MAP_ROOT_PATH); // This also removes the /raw/ folder
        boost::filesystem::create_directory(RAS_Names::MAP_ROOT_PATH); // Re-create it
*/
    }

    void run()
    {
        ros::Rate loop_rate(PUBLISH_RATE);
        while(ros::ok())
        {
            if(odo_data_ != nullptr  && adc_data_ != nullptr){

                //ros::WallTime temp_time = ros::WallTime::now();

                mapHandler.update(odo_data_, adc_data_, las_data_, new_adc_data_recieved_, new_laser_data_recieved_);
                new_laser_data_recieved_= false;
                new_adc_data_recieved_ = false;  // needed for removing duplicate data

                {
                    // ** Publish
                    nav_msgs::OccupancyGrid msg_raw, msg_thick;
                    std_msgs::Int64MultiArray msg_cost;
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

                    // Cost Map
                    msg_cost.data = (&mapHandler.getCostMap())[0];
                    map_pub_cost_.publish(msg_cost);

                    if(++publish_bag_counter_ > PUBLISH_RATE)
                    {
                        // ** Publish to the recording every second instead, since the bag weighs A LOT
                        map_pub_bag_.publish(msg_raw);
                        map_pub_thick_bag_.publish(msg_thick);
                        map_pub_cost_bag_.publish(msg_cost);

                        publish_bag_counter_ = 0;
                    }
                 //   std::cout << "map time: " << RAS_Utils::time_diff_ms(temp_time, ros::WallTime::now()) << std::endl;
                }
            }

            // ** Sleep
            ros::spinOnce();
            loop_rate.sleep();
        }
    }

private:

    // ** Publishers and subscribers
    ros::Publisher map_pub_, map_pub_thick_, map_pub_cost_;
    ros::Publisher map_pub_bag_, map_pub_thick_bag_, map_pub_cost_bag_;

    ros::Subscriber odo_sub_;
    ros::Subscriber adc_sub_;
    ros::Subscriber las_sub_;
    ros::Subscriber obj_sub_;

    // ** Recieved messages from subscribers
    geometry_msgs::Pose2D::ConstPtr odo_data_;
    ras_srv_msgs::IRDataConstPtr adc_data_;
    ras_srv_msgs::LaserScanner::ConstPtr las_data_;

    // ** The actual map
    MapHandler mapHandler;


    void odoCallback(const geometry_msgs::Pose2D::ConstPtr& msg) {
        odo_data_ = msg;
    }

    bool new_adc_data_recieved_;

    void adcCallback(const ras_srv_msgs::IRDataConstPtr& msg)
    {
        new_adc_data_recieved_ = true; // needed for removing duplicate data
        adc_data_ = msg;
    }

    bool new_laser_data_recieved_;

    void laserCallback(const ras_srv_msgs::LaserScanner::ConstPtr& msg)
    {
        new_laser_data_recieved_ = true;
        las_data_ = msg;
    }


    void objCallbak(const geometry_msgs::Point::ConstPtr& msg)
    {
        mapHandler.objectDetected(*msg);
    }

    void fixLinesRawMap(nav_msgs::OccupancyGrid & map);

    int publish_bag_counter_;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "map_handler_node");

    MapHandlerNode mhn;
    ros::WallTime t1(ros::WallTime::now());

    mhn.run();

    // ** Save time
    int timeStart = 0.001 * RAS_Utils::time_diff_ms(t1, ros::WallTime::now()) - TIME_BEFORE_END_SAVE_MAP;
    std::ofstream file(RAS_Names::FILE_TIME_START_MAP_ROSBAG);
    file << timeStart;
    file.close();

    return 0;
}


