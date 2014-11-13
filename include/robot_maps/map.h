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
        simple_map_vector_(height*width, SIMPLE_UNKNOWN_AREA)

    {
        for(int i = 0; i < width; i++)
        {
            for(int j = 0; j < height; j++)
            {
                setUnknown(i, j);
            }
        }
    }

    void setBlocked(int x, int y) {
        setCellSimpleVector(x, y, SIMPLE_BLOCKED_AREA);
        setCell(Cell(x, y, Cell::BLOCKED));
    }

    void setFree(int x, int y) {
        setCellSimpleVector(x, y, SIMPLE_FREE_AREA);
        setCell(Cell(x, y, Cell::FREE));
    }

    void setUnknown(int x, int y) {
        setCellSimpleVector(x, y, SIMPLE_UNKNOWN_AREA);
        setCell(Cell(x, y, Cell::UNKNOWN));
    }

    void setObject(int x, int y, std::string object_name) {
        setCellSimpleVector(x, y, SIMPLE_BLOCKED_AREA);
        setCell(ObjectCell(x, y, object_name));
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

    int getCellSize()
    {
        return cell_size_;
    }


private:


    int height_;
    int width_;
    int cell_size_;

    std::vector<int8_t> simple_map_vector_;

    CellMatrix map_matrix_;

    void setCellSimpleVector(int x, int y, int8_t value)
    {
        simple_map_vector_[x + width_*y] = value;
    }

    void setCell(Cell cell)
    {
        map_matrix_[cell.getX()][cell.getY()] = cell;
    }


};


#endif // MAP_H
