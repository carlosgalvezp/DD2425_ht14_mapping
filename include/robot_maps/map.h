#ifndef MAP_H
#define MAP_H

#include <robot_maps/cell.h>
#include <vector>

#define SIMPLE_UNKNOWN_AREA 0
#define SIMPLE_FREE_AREA    127
#define SIMPLE_BLOCKED_AREA 255

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

    void setBlocked(double x, double y) {
        int i, j;
        convertToIndexValue(x, y, i, j);
        setCell(Cell(i, j, Cell::BLOCKED), SIMPLE_BLOCKED_AREA);
    }

    void setFree(double x, double y) {
        int i, j;
        convertToIndexValue(x, y, i, j);
        setCell(Cell(i, j, Cell::FREE), SIMPLE_FREE_AREA);
    }

    void setUnknown(double x, double y) {
        int i, j;
        convertToIndexValue(x, y, i, j);
        setCell(Cell(i, j, Cell::UNKNOWN), SIMPLE_UNKNOWN_AREA);
    }

    void setObject(double x, double y, std::string object_name) {
        int i, j;
        convertToIndexValue(x, y, i, j);
        setCell(ObjectCell(i, j, object_name), SIMPLE_BLOCKED_AREA);
    }

    std::vector<int8_t> & getSimpleMapVector() {
        return simple_map_vector_;
    }

    const CellMatrix & getMapMatrix()
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

    void setCell(Cell cell, int8_t simple_value)
    {
        if(cell.getI() < 0 || cell.getI() >= getWidth() || cell.getJ() < 0 || cell.getJ() > getHeight())
        {
            ROS_ERROR(" *** Trying to set cell outside of bounds! Check code!!! ***");
            ROS_ERROR("X = %i : Y = %i", cell.getI(), cell.getJ());
            return;
        }
        simple_map_vector_[cell.getI() + getWidth() * cell.getJ()] = simple_value;
        map_matrix_[cell.getI()][cell.getJ()] = cell;
    }

    void convertToIndexValue(double x, double y, int &i, int &j)
    {
        i = (x/getCellSize()) + 0.5;
        j = (y/getCellSize()) + 0.5;
    }


};


#endif // MAP_H
