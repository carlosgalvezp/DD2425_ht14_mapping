#ifndef MAP_H
#define MAP_H

#include <robot_maps/cell.h>
#include <vector>

#define SIMPLE_UNKNOWN_AREA 50
#define SIMPLE_FREE_AREA    0
#define SIMPLE_BLOCKED_AREA 100

class Map {
public:
    typedef std::vector< Cell > CellVector;
    typedef std::vector< CellVector > CellMatrix;

    Map(int height, int width, double cell_size) :
        height_(height),
        width_(width),
        cell_size_(cell_size),
        simple_map_vector_(height*width, SIMPLE_UNKNOWN_AREA),
        map_matrix_(width, CellVector(height))

    {
        for(int i = 0; i < width; i++)
        {
            for(int j = 0; j < height; j++)
            {
                setUnknown(i, j);
            }
        }
    }

    Cell & getCell(double x, double y) {
        int i, j;
        convertToIndexValue(x, y, i, j);
        return getMapMatrix()[i][j];
    }

    void setBlocked(double x, double y) {
        int i, j;
        convertToIndexValue(x, y, i, j);
        setCell(Cell(i, j, Cell::BLOCKED, map_matrix_[i][j].getCost()));
    }

    void setFree(double x, double y) {
        int i, j;
        convertToIndexValue(x, y, i, j);
        setCell(Cell(i, j, Cell::FREE, map_matrix_[i][j].getCost()));
    }

    void setUnknown(double x, double y) {
        int i, j;
        convertToIndexValue(x, y, i, j);
        setCell(Cell(i, j, Cell::UNKNOWN, map_matrix_[i][j].getCost()));
    }

    void setObject(double x, double y, std::string object_name) {
        int i, j;
        convertToIndexValue(x, y, i, j);
        setCell(ObjectCell(i, j, object_name, map_matrix_[i][j].getCost()));
    }

    void setCost(double x, double y, int cost)
    {
        int i, j;
        convertToIndexValue(x, y, i, j);
        setCell(Cell(i, j, map_matrix_[i][j].getType(), cost));
    }

    std::vector<int8_t> & getSimpleMapVector() {
        return simple_map_vector_;
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


private:


    int height_;
    int width_;
    double cell_size_;

    std::vector<int8_t> simple_map_vector_;

    CellMatrix map_matrix_;

    void setCell(Cell cell)
    {
        if(cell.getI() < 0 || cell.getI() >= getWidth() || cell.getJ() < 0 || cell.getJ() > getHeight())
        {
            ROS_ERROR(" *** Trying to set cell outside of bounds! Check code!!! ***");
            ROS_ERROR("X = %i : Y = %i", cell.getI(), cell.getJ());
            return;
        }
        if(cell.isFree()) {
            simple_map_vector_[cell.getI() + getWidth() * cell.getJ()] = SIMPLE_FREE_AREA + cell.getCost();
        } else if(cell.isBlocked() || cell.isObject()){
            simple_map_vector_[cell.getI() + getWidth() * cell.getJ()] = SIMPLE_BLOCKED_AREA;
        } else {
            simple_map_vector_[cell.getI() + getWidth() * cell.getJ()] = SIMPLE_UNKNOWN_AREA;
        }
        map_matrix_[cell.getI()][cell.getJ()] = cell;
    }

    void convertToIndexValue(double x, double y, int &i, int &j)
    {
        i = (x/getCellSize()) + 0.5;
        j = (y/getCellSize()) + 0.5;
    }


};


#endif // MAP_H
