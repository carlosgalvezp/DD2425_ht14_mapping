#include <ros/ros.h>
#include <robot_maps/map_handler.h>
#include <robot_maps/map.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose2D.h>
#include <ras_utils/basic_node.h>
#include <ras_arduino_msgs/ADConverter.h>

#define QUEUE_SIZE      1000
#define PUBLISH_RATE    10

// ** Map specific constants
#define MAP_HEIGHT      1400    // Corresponds to amount of cells
#define MAP_WIDTH       1400    // Corresponds to amount of cells
#define MAP_CELL_SIZE   1.0     // Width and Height in // Corresponds to amount of cells CM per Cell


class MapHandlerNode : rob::BasicNode
{
public:

    MapHandlerNode() : mapHandler(Map(MAP_HEIGHT, MAP_WIDTH, MAP_CELL_SIZE))
    {
        // Publisher
        map_pub_ = n.advertise<nav_msgs::OccupancyGrid>(TOPIC_MAP_OCC_GRID, QUEUE_SIZE);
        // Subscriber
        odo_sub_ = n.subscribe(TOPIC_ODOMETRY, 1,  &MapHandlerNode::odoCallback, this);
        adc_sub_ = n.subscribe(TOPIC_ARDUINO_ADC, 1,  &MapHandlerNode::adcCallback, this);

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
                    nav_msgs::OccupancyGrid msg;
                    msg.data = (&mapHandler.getMap())[0];
                    msg.info.height = mapHandler.getHeight();
                    msg.info.width = mapHandler.getWidth();
                    msg.info.resolution = mapHandler.getCellSize();
                    map_pub_.publish(msg);
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
    ros::Subscriber odo_sub_;
    ros::Subscriber adc_sub_;

    // ** Recieved messages from subscribers
    geometry_msgs::Pose2D::ConstPtr odo_data_;
    ras_arduino_msgs::ADConverter::ConstPtr adc_data_;

    // ** The actual map
    MapHandler mapHandler;

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
