#include <ros/ros.h>
#include <ras_utils/graph/graph.h>
#include <ras_utils/ras_names.h>
#include <ras_utils/occupancy_map_utils.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <ras_utils/basic_node.h>
#include <mapping/map_io.h>
#include <ras_utils/genetic/genetic_algorithm.h>

#include <fstream>

#define QUEUE_SIZE      1
#define PUBLISH_RATE    10

class ObjectMapNode : rob::BasicNode
{
public:

    ObjectMapNode();
    void computeObjectsPath(std::vector<Node> &objects_path);
    void saveObjectsPath(const std::string& path, const std::vector<Node> &objects_path);
private:
    nav_msgs::OccupancyGrid::ConstPtr map_msg_;
    visualization_msgs::MarkerArray::ConstPtr object_msg_;

    // ** Subscribers
    ros::Subscriber map_sub_, object_position_sub_;

    // ** Callbacks
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg);
    void objectsCallback(const visualization_msgs::MarkerArray::ConstPtr &msg);
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "map_loader_node");

    ObjectMapNode o;
    ros::spin();

    // ** Before exiting, compute TSP path and save it to disk
    ROS_INFO("[ObjectMapNode] Computing TSP path for objects");
    std::vector<Node> objects_path;
    o.computeObjectsPath(objects_path);
    o.saveObjectsPath(RAS_Names::OBJECT_BEST_PATH_PATH, objects_path);

    return 0;
}

// ===============================================================================
// ===============================================================================
ObjectMapNode::ObjectMapNode()
{
    // Subscribers
    map_sub_ = n.subscribe(TOPIC_MAP_OCC_GRID_THICK, QUEUE_SIZE, &ObjectMapNode::mapCallback, this);
    object_position_sub_ = n.subscribe(TOPIC_MARKERS, QUEUE_SIZE, &ObjectMapNode::objectsCallback, this);
}

void ObjectMapNode::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    this->map_msg_ = msg;
}

void ObjectMapNode::objectsCallback(const visualization_msgs::MarkerArray::ConstPtr &msg)
{
    this->object_msg_ = msg;
}

void ObjectMapNode::computeObjectsPath(std::vector<Node> &objects_path)
{
    std::vector<Node> nodes;
    std::vector<Edge> edges;
    nodes.push_back(Node(0,0,0)); // Starting position

    // ** Get nodes
    for(std::size_t i = 0; object_msg_->markers.size(); ++i)
    {
        const visualization_msgs::Marker &marker = object_msg_->markers[i];
        if(marker.ns == RVIZ_MARKER_NS_OBJECT && marker.type == visualization_msgs::Marker::CUBE)
        {
            Node n(marker.pose.position.x, marker.pose.position.y,nodes.size());
            nodes.push_back(n);
        }
    }
    // Add the start node again, so that we come back after fetching objects
    nodes.push_back(Node(0,0,nodes.size()));

    // ** Get edges and cost
    for(std::size_t i = 0; i < nodes.size(); ++i)
    {
        for(std::size_t j = i+1; j < nodes.size(); ++j)
        {
            const Node &n1 = nodes[i];
            const Node &n2 = nodes[j];

            const std::vector<geometry_msgs::Point> &path =
                    RAS_Utils::occ_grid::bfs_search::getPathFromTo(*map_msg_, n1.getPosition().x_,n1.getPosition().y_,
                                                                              n2.getPosition().x_,n2.getPosition().y_);
            double cost  = path.size();
            Edge e(n1, n2, cost);
            edges.push_back(e);
        }
    }

    // ** Create graph
    Graph graph(nodes, edges);

    // ** Solve TSP problem
    GeneticAlgorithm gm(graph);
    gm.computeSolution(objects_path);
}
void ObjectMapNode::saveObjectsPath(const std::string &path, const std::vector<Node> &objects_path)
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

