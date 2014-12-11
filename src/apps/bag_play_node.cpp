#include <ros/ros.h>
#include <fstream>
#include <ras_utils/ras_names.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "bag_play_node");

    // ** Save time
    std::ifstream file(RAS_Names::FILE_TIME_START_MAP_ROSBAG);
    int tStart;
    file >> tStart;
    std::stringstream ss;
    ss << "rosbag play -q -d 2 -s "<< tStart<<" dd2425_ht14_G6_phase1_map.bag";

    system(ss.str().c_str());

    return 0;
}


