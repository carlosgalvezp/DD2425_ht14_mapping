#include <ros/ros.h>
#include <ras_utils/ras_names.h>
#include <ras_utils/occupancy_map_utils.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ras_utils/basic_node.h>
#include <std_msgs/Int64MultiArray.h>
#include <mapping/map_io.h>

#define QUEUE_SIZE      1
#define PUBLISH_RATE    10

// ** Map specific constants
#define MAP_HEIGHT      1000    // Corresponds to amount of cells
#define MAP_WIDTH       1000    // Corresponds to amount of cells

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

    bool got_map_raw, got_map_thick, got_map_cost;

    nav_msgs::OccupancyGrid thick_map_msg_, raw_map_msg_;
    std_msgs::Int64MultiArray cost_msg_;

    // ** Callbacks
    void rawMapCallback(const nav_msgs::OccupancyGridConstPtr &msg);
    void thickMapCallback(const nav_msgs::OccupancyGridConstPtr &msg);
    void costMapCallback(const std_msgs::Int64MultiArrayConstPtr &msg);

    void fixLinesRawMap(nav_msgs::OccupancyGrid &map);
};

int main(int argc, char **argv) {    
    // ** Launch normal node
    ros::init(argc, argv, "map_loader_node");

    MapLoaderNode mln;

    mln.run();

    return 0;
}

// ===============================================================================
// ===============================================================================
MapLoaderNode::MapLoaderNode()
    : got_map_raw(false), got_map_thick(false), got_map_cost(false)
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
        if(got_map_raw)
        {
            map_pub_raw_.publish(raw_map_msg_);
        }

        if(got_map_thick)
        {
            map_pub_thick_.publish(thick_map_msg_);
        }

        if(got_map_cost)
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
    this->raw_map_msg_ = *msg;
    ROS_WARN("GOT RAW MAP");
    // Fix thin lines
    fixLinesRawMap(this->raw_map_msg_);

    // Stop subscribing
    got_map_raw = true;
    this->map_sub_raw_.shutdown();
}

void MapLoaderNode::thickMapCallback(const nav_msgs::OccupancyGridConstPtr &msg)
{
    this->thick_map_msg_ = *msg;
    ROS_WARN("GOT THICK MAP");

    // Stop subscribing
    got_map_thick = true;
    this->map_sub_thick_.shutdown();
}

void MapLoaderNode::costMapCallback(const std_msgs::Int64MultiArrayConstPtr &msg)
{
    this->cost_msg_ = *msg;
    ROS_WARN("GOT COST MAP");

    // Stop subscribing
    got_map_cost = true;
    this->map_sub_cost_.shutdown();
}

void MapLoaderNode::fixLinesRawMap(nav_msgs::OccupancyGrid &map)
{
    // ** Convert to cv::Mat
    cv::Mat img_raw = cv::Mat(MAP_HEIGHT, MAP_WIDTH, CV_8UC1, &(map.data[0]));
    cv::threshold(img_raw, img_raw, 50, OCC_GRID_SIMPLE_BLOCKED_AREA, CV_THRESH_BINARY);
//    cv::imshow("RAW", img_raw);
    // ** Erode and dilate
    int size = 3;
    cv::Mat element = cv::getStructuringElement( cv::MORPH_RECT,cv::Size( 2*size + 1, 2*size+1 ),cv::Point( size, size ) );
    cv::dilate( img_raw, img_raw, element );
//    imshow("After dilation", img_raw );
    cv::erode( img_raw, img_raw, element );
//    imshow("After erosion", img_raw );
//    cv::waitKey();
    // ** Convert back to vector
    cv::Mat img_raw2 = img_raw.reshape(0,1);
    map.data = std::vector<int8_t>(img_raw2);
}
