#include <ros/ros.h>
#include <ras_utils/graph/graph.h>
#include <ras_utils/ras_names.h>
#include <ras_utils/occupancy_map_utils.h>
#include <ras_utils/ras_utils.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <ras_utils/basic_node.h>
#include <mapping/map_io.h>
#include <ras_utils/genetic/genetic_algorithm.h>

#include <fstream>
#include <signal.h>

#define QUEUE_SIZE      1
#define PUBLISH_RATE    10

#define MIN_OBJECT_DISTANCE 0.3 // [m]

// ** Global variables and functions to be called by the SIGINT handle.
// ** I am sorry but there is no other option!! :(

    void computeObjectsGraph(const nav_msgs::OccupancyGrid::ConstPtr &map_msg,
                            const std_msgs::Int64MultiArray::ConstPtr &map_cost_msg,
                            const std::vector<geometry_msgs::Point> &object_vector,
                            std::vector<Node> &objects_path);

    void saveObjectsPath(const std::string& path, const std::vector<Node> &objects_path);
    nav_msgs::OccupancyGrid::ConstPtr map_msg_;
    std_msgs::Int64MultiArray::ConstPtr map_cost_msg_;
    std::vector<geometry_msgs::Point> object_vector_;

    // ** Callbacks
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg);
    void objectsCallback(const geometry_msgs::Point::ConstPtr &msg);
    void mapCostCallback(const std_msgs::Int64MultiArray::ConstPtr &msg);

    // ** Help functions
    bool pathContainsIntermediateNodes(const Node& start, const Node &end,
                                       const std::vector<geometry_msgs::Point> &path,
                                       const std::vector<Node> &nodes);

void mySigintHandler(int sig)
{
    // ** Before exiting, compute Graph path and save it to disk
    std::cout <<"[ObjectMapNode] Computing TSP path for objects" << std::endl;
    std::vector<Node> objects_path;
    computeObjectsGraph(map_msg_, map_cost_msg_, object_vector_, objects_path);

    // ** Call another node to solve the TSP problem
    system("rosrun mapping tsp_solver_node");

    // All the default sigint handler does is call shutdown()
    ros::shutdown();
}

void mySigTermHandler(int sig)
{
}

int main(int argc, char **argv) {    
    ros::init(argc, argv, "map_loader_node");
    ros::NodeHandle nh;

    // ** Subscribers
    ros::Subscriber map_sub_, object_position_sub_, map_cost_sub_;
    map_sub_ = nh.subscribe(TOPIC_MAP_OCC_GRID_THICK, QUEUE_SIZE, &mapCallback);
    map_cost_sub_ = nh.subscribe(TOPIC_MAP_COST, QUEUE_SIZE, &mapCostCallback);
    object_position_sub_ = nh.subscribe(TOPIC_ROBOT_OBJECT_POSITION, QUEUE_SIZE, &objectsCallback);


    // Override the default ros sigint handler.
    // This must be set after the first NodeHandle is created.
    signal(SIGINT, mySigintHandler);
    signal(SIGTERM, mySigTermHandler); // Even if ROS wants to kill me, it wont be able to :)

    ros::spin();

    return 0;
}

// ===============================================================================
// ===============================================================================
void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    map_msg_ = msg;
}

void objectsCallback(const geometry_msgs::Point::ConstPtr &msg)
{
    object_vector_.push_back(*msg);
}

void mapCostCallback(const std_msgs::Int64MultiArray::ConstPtr &msg)
{
    map_cost_msg_ = msg;
}


void computeObjectsGraph(const nav_msgs::OccupancyGrid::ConstPtr &map_msg,
                        const std_msgs::Int64MultiArray::ConstPtr &map_cost_msg,
                        const std::vector<geometry_msgs::Point> &object_vector,
                        std::vector<Node> &objects_path)
{    
    if(map_msg!= 0 || object_vector.size() != 0)
    {

        std::cout << "============== COMPUTING OBJECT PATH================"<<std::endl;

        std::vector<Node> nodes;
        std::vector<Edge> edges;
        nodes.push_back(Node(0.0,0.0,0)); // Starting position

        // ** Get nodes
        for(std::size_t i = 0; i < object_vector.size(); ++i)
        {
            const geometry_msgs::Point &p = object_vector[i];
            Node n(p.x, p.y,nodes.size());
            nodes.push_back(n);
        }

        // ** Get edges and cost
        for(std::size_t i = 0; i < nodes.size(); ++i)
        {
            for(std::size_t j = i+1; j < nodes.size(); ++j)
            {
                const Node &n1 = nodes[i];
                const Node &n2 = nodes[j];

                const std::vector<geometry_msgs::Point> &path =
                        RAS_Utils::occ_grid::bfs_search::getPathFromTo(*map_msg, *map_cost_msg, n1.getPosition().x_,n1.getPosition().y_,
                                                                                                n2.getPosition().x_,n2.getPosition().y_);

                double cost  = path.size() != 0? path.size() : 100000;
                Edge e(n1, n2, cost);
                edges.push_back(e);
                std::cout <<"EDGE BETWEEN "<<n1.getID()<<" and "<<n2.getID() <<" with cost "<<cost<<std::endl;
            }
        }

        // ** Create graph
        Graph graph(nodes, edges);
        std::cout << "Created graph with "<<nodes.size() << " and " << edges.size()<<std::endl;

        // ** Save to file
        std::ofstream file(RAS_Names::OBJECT_GRAPH_PATH);
        file << graph;
        file.close();
    }
    else
    {
        std::cout << "[ObjectMapNode]: map_msgs = 0 or object_msg = 0" <<std::endl;
    }
}
void saveObjectsPath(const std::string &path, const std::vector<Node> &objects_path)
{
    if(objects_path.size() != 0)
    {
        // ** Save path into a text file
        std::ofstream file;
        file.open(path);
        for(std::size_t i = 0; i < objects_path.size(); ++i)
        {
            const Node &n = objects_path[i];
            file << n.getID() << " " << n.getPosition().x_ << " " << n.getPosition().y_ << std::endl;
        }
        file.close();
    }
}

bool pathContainsIntermediateNodes(const Node& start, const Node &end,
                                   const std::vector<geometry_msgs::Point> &path,
                                   const std::vector<Node> &nodes)
{
    // ** Analize the path and check whether the points are not close to other object nodes
    for(std::size_t i = 0; i < path.size(); ++i)
    {
        const geometry_msgs::Point &p  = path[i];

        for(std::size_t j = 0; j < nodes.size(); ++j)
        {
            const Node &n = nodes[j];
            if(n.getID() != start.getID() && n.getID() != end.getID())
            {
                double d = RAS_Utils::euclidean_distance(n.getPosition().x_, n.getPosition().y_,
                                                         p.x, p.y);
                if( d < MIN_OBJECT_DISTANCE )
                    return true;
            }
        }
    }
    return false;
}
