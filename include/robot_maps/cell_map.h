#ifndef CELL_MAP_H
#define CELL_MAP_H

#include <robot_maps/cell.h>
#include <robot_maps/physical_object.h>
#include <vector>

class CellMap {
public:

    CellMap(int rows, int column) : map(rows, CellVector(column, Cell(Cell::UNKNOWN))) {
    }

    void setBlocked(int x, int y) {
        setCell(x, y, Cell(Cell::BLOCKED));
    }

    void setFree(int x, int y) {
        setCell(x, y, Cell(Cell::FREE));
    }

    void setUnknown(int x, int y) {
        setCell(x, y, Cell(Cell::UNKNOWN));
    }

    void setObject(int x, int y, PhysicalObject * physicalObject) {
        setCell(x, y, ObjectCell(physicalObject));
    }


private:
    typedef std::vector< std::vector<Cell> > CellMatrix;
    typedef std::vector<Cell> CellVector;

    void setCell(int x, int y, Cell newCell ) {
        map[x][y] = newCell;
    }

    CellMatrix map;
};


#endif // CELL_MAP_H
