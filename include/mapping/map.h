#ifndef MAP_H
#define MAP_H

#include <mapping/cell.h>
#include <vector>

#define SIMPLE_UNKNOWN_AREA 50
#define SIMPLE_FREE_AREA    0
#define SIMPLE_BLOCKED_AREA 100

#define THICK_WALL_COUNTER_LIMIT 5
#define THICK_WALL_SIZE 10

#define COST_MAP_LIMIT                  20
#define COST_MAP_DEFAULT                100
#define COST_MAP_EXPO_STATIC_INCREASER  5.0    // Lowest possible value to be used with expo. Makes the cost map more distinct, less changing of path
#define COST_MAP_EXPO_VALUE             3     // The expo value for cost calculation

class Map {
public:
    typedef std::vector< Cell > CellVector;
    typedef std::vector< CellVector > CellMatrix;

    Map() {}

    Map(int height, int width, double cell_size) :
        height_(height),
        width_(width),
        cell_size_(cell_size),
        simple_map_vector_(height*width, SIMPLE_UNKNOWN_AREA),
        simple_thick_map_vector_(height*width, SIMPLE_UNKNOWN_AREA),
        thick_wall_counter_(height*width, 0),
        map_matrix_(width, CellVector(height)),
        cost_map_(width * height, COST_MAP_DEFAULT)

    {
        for(int i = 0; i < width; i++)
        {
            for(int j = 0; j < height; j++)
            {
                setUnknown(i, j);
            }
        }

        preCalculateCost();
        preCalculateThickWall();
    }


    bool setFreeIfNotFreeAllready(double x, double y)
    {
        int i, j;
        convertToIndexValue(x, y, i, j);
        if(!getMapMatrix()[i][j].isFree())
        {
            setFree(i, j);
            return true;
        }
        return false;
    }

    bool setFreeIfUnknown(double x, double y)
    {
        int i, j;
        convertToIndexValue(x, y, i, j);
        if(getMapMatrix()[i][j].isUnknown())
        {
            setFree(i, j);
            return true;
        }
        return false;
    }


    bool setBlockedIfNotBlockedAllready(double x, double y)
    {
        int i, j;
        convertToIndexValue(x, y, i, j);
        if(!getMapMatrix()[i][j].isBlocked())
        {
            // We block and then make sure to incrase the cost for surronding cells
            setBlocked(i, j);
            return true;
        }
        return false;
    }



    Cell & getCell(double x, double y)
    {
        int i, j;
        convertToIndexValue(x, y, i, j);
        return getMapMatrix()[i][j];
    }

    std::vector<int8_t> & getSimpleMapVector()
    {
        return simple_map_vector_;
    }

    std::vector<int8_t> & getSimpleThickMapVector()
    {
        return simple_thick_map_vector_;
    }

    std::vector<long> & getCostVector()
    {
        return cost_map_;
    }

    CellMatrix & getMapMatrix()
    {
        return map_matrix_;
    }

    int getHeight()
    {
        return height_;
    }

    int getWidth()
    {
        return width_;
    }

    double getCellSize()
    {
        return cell_size_;
    }

    int getIndexPosition(int i, int j)
    {
        return getIndexPosition(i, j, getWidth());
    }

    int getIndexPosition(int i, int j, int size)
    {
        return i + j * size;
    }


private:


    int height_;
    int width_;
    double cell_size_;

    CellMatrix map_matrix_;

    std::vector<int8_t> simple_map_vector_;
    std::vector<int8_t> simple_thick_map_vector_;
    std::vector<long> cost_map_;

    std::vector<int> thick_wall_counter_;

    std::vector<long> pre_calculated_costs;
    std::vector<bool> pre_calculated_thick_wall;

    void setCell(Cell cell)
    {
        if(cell.getI() < 0 || cell.getI() >= getWidth() || cell.getJ() < 0 || cell.getJ() > getHeight())
        {
            ROS_ERROR(" *** Trying to set cell outside of bounds! Check code!!! ***");
            ROS_ERROR("X = %i : Y = %i", cell.getI(), cell.getJ());
            return;
        }


        if(cell.isFree()) {
            simple_map_vector_[getIndexPosition(cell.getI(), cell.getJ())] = SIMPLE_FREE_AREA;


            if(map_matrix_[cell.getI()][cell.getJ()].isBlocked())
            {
                // Going specifically from blocked to free
                updateThickMapVectorWall(cell.getI(), cell.getJ(), false);
                updateCellCost(cell.getI(), cell.getJ(), false);
            } else
            {
                updateThickMapVectorFree(cell.getI(), cell.getJ());
            }

        } else if(cell.isBlocked() || cell.isObject()){
            simple_map_vector_[getIndexPosition(cell.getI(), cell.getJ())] = SIMPLE_BLOCKED_AREA;
            updateThickMapVectorWall(cell.getI(), cell.getJ(), true);
            updateCellCost(cell.getI(), cell.getJ(), true);
        } else {
            simple_map_vector_[getIndexPosition(cell.getI(), cell.getJ())] = SIMPLE_UNKNOWN_AREA;
        }
        map_matrix_[cell.getI()][cell.getJ()] = cell;
    }


    void preCalculateCost()
    {

        auto costCalculator = [](int i, int j, int max_dist)
        {
            double dist = sqrt(pow(i, 2) + pow(j, 2));
            long value;
            if(dist >= max_dist)
            {
                value = 0;
            } else
            {
                value = pow((double)max_dist - dist + COST_MAP_EXPO_STATIC_INCREASER, COST_MAP_EXPO_VALUE) + 0.5;
            }

            return value;
        };

        int size_limit = COST_MAP_LIMIT;


        pre_calculated_costs = std::vector<long>(pow(size_limit*2 + 1, 2));



        for(int i = - size_limit; i <= size_limit; i++)
        {
            for(int j = - size_limit; j <= size_limit; j++)
            {
                int index = getIndexPosition(i + size_limit, j + size_limit, (size_limit*2 + 1));
                pre_calculated_costs[index] = costCalculator(i, j, size_limit);
            }
        }
    }


    void preCalculateThickWall()
    {

        pre_calculated_thick_wall = std::vector<bool>(pow(THICK_WALL_SIZE*2 + 1, 2));
        for(int i = -THICK_WALL_SIZE; i <= THICK_WALL_SIZE; i++)
        {
            for(int j = -THICK_WALL_SIZE; j <= THICK_WALL_SIZE; j++)
            {
                double dist = sqrt(pow(i, 2) + pow(j, 2)) + 0.5;
                int index_pos = getIndexPosition(i + THICK_WALL_SIZE, (j + THICK_WALL_SIZE), (THICK_WALL_SIZE*2 + 1));
                if(dist < THICK_WALL_SIZE)
                {
                    pre_calculated_thick_wall[index_pos] = true;
                } else {
                    pre_calculated_thick_wall[index_pos] = false;
                }
            }
        }
    }

    void updateCellCost(int cell_i, int cell_j, bool increase)
    {

        int sign = (increase) ? 1: -1;

        int size_limit = COST_MAP_LIMIT;

        for(int i = - size_limit; i <= size_limit; i++)
        {
            for(int j = - size_limit; j <= size_limit; j++)
            {
                long cost = pre_calculated_costs[getIndexPosition(i + size_limit, j + size_limit, size_limit*2 + 1)];
                cost_map_[getIndexPosition(i + cell_i, j + cell_j)] += cost * sign;
            }
        }
    }

    void updateThickMapVectorFree(int cell_i, int cell_j)
    {
        int index = getIndexPosition(cell_i, cell_j);
        if(simple_thick_map_vector_[index] != SIMPLE_BLOCKED_AREA)
        {
            simple_thick_map_vector_[index] = SIMPLE_FREE_AREA;
        }
    }

    void updateThickMapVectorWall(int cell_i, int cell_j, bool added_wall)
    {
        int sign = (added_wall) ? 1 : -1;
        int index;

        for(int i = -THICK_WALL_SIZE; i <= THICK_WALL_SIZE; i++)
        {
            for(int j = -THICK_WALL_SIZE; j <= THICK_WALL_SIZE; j++)
            {
                if(pre_calculated_thick_wall[j + THICK_WALL_SIZE + (i + THICK_WALL_SIZE) * (THICK_WALL_SIZE*2 + 1)])
                {
                    index = getIndexPosition(i + cell_i, j + cell_j);

                    thick_wall_counter_[index] += sign;
                    if(thick_wall_counter_[index] >= THICK_WALL_COUNTER_LIMIT)
                    {
                        // Counter reached, fill with wall
                        simple_thick_map_vector_[index] = SIMPLE_BLOCKED_AREA;
                    } else
                    {
                        // It should not be willed with wall just yet
                        if(simple_thick_map_vector_[index] == SIMPLE_BLOCKED_AREA)
                        {
                            //IF it was filled, change to not filled
                            simple_thick_map_vector_[index] = SIMPLE_FREE_AREA;
                        } else if(simple_map_vector_[index] != SIMPLE_UNKNOWN_AREA)
                        {
                            // The "normal" map is not unknown here, so set it to known for the thick map
                            simple_thick_map_vector_[index] = SIMPLE_FREE_AREA;
                        }
                    }
                }
            }
        }

    }


    void convertToIndexValue(double x, double y, int &i, int &j)
    {
        i = (x/getCellSize()) + 0.5;
        j = (y/getCellSize()) + 0.5;
    }

    void setBlocked(int i, int j)
    {
        setCell(Cell(i, j, Cell::BLOCKED));
    }

    void setFree(int i, int j)
    {
        setCell(Cell(i, j, Cell::FREE));
    }

    void setUnknown(int i, int j)
    {
        setCell(Cell(i, j, Cell::UNKNOWN));
    }

    void setObject(int i, int j, std::string object_name)
    {
        setCell(ObjectCell(i, j, object_name));
    }
};


#endif // MAP_H
