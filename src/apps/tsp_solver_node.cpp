#include <ras_utils/ras_names.h>
#include <ras_utils/ras_utils.h>
#include <ras_utils/genetic/genetic_algorithm.h>
#include <fstream>
#include <signal.h>
void mySigintHandler(int)
{
    std::cout << "You will not ever kill me :)";
}

void mySigTermHandler(int)
{
    std::cout << "You will need something better than that :)";
}

int main(int argc, char **argv)
{
    signal(SIGINT, mySigintHandler);
    signal(SIGTERM, mySigTermHandler); // Even you want to kill me, you wont be able to :)

    std::cout << "===== TSP SOLVER ========"<<std::endl;

    // ** Read graph from file
    std::ifstream file(RAS_Names::OBJECT_GRAPH_PATH);
    Graph g;
    file >> g;
    file.close();

    // Solve TSP with Genetic Algorithm
    std::cout << "=== BEFORE CREATING GA====" <<std::endl;
    GeneticAlgorithm ga(g);
    std::vector<Node> objects_path;
    std::cout << "=== BEFORE COMPUTE SOLUTION====" <<std::endl;
    ga.computeSolution(objects_path);
    std::cout << "==== AFTER COMPUTE SOLUTION ====" <<std::endl;

    // Save to file
    if(objects_path.size() != 0)
    {
        std::ofstream file(RAS_Names::OBJECT_BEST_PATH_PATH);
        for(std::size_t i = 0; i < objects_path.size(); ++i)
        {
            const Node &n = objects_path[i];
            file << n.getID() << " " << n.getPosition().x_ << " " << n.getPosition().y_ << std::endl;
        }
    }
    std::cout << "======= PATH SUCCESSFULLY COMPUTED ========" << std::endl;
    return 0;
}
