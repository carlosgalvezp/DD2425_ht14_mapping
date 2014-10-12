#ifndef CELL_H
#define CELL_H

#include <robot_maps/physical_object.h>



class Cell {
public:

    enum Type {
        UNKNOWN, FREE, BLOCKED
    };

    Cell(Type cellValue) : type(cellValue){}

    virtual bool isObject() {
        return false;
    }

    bool isBlocked() {
        return type == BLOCKED;
    }

    bool isFree() {
        return type == FREE;
    }

    bool isUnknown() {
        return type == UNKNOWN;
    }

private:
    Type type;
};

class ObjectCell : public Cell {
public:
    ObjectCell() : ObjectCell(nullptr) {}

    ObjectCell(PhysicalObject * physicalObject) : Cell(Cell::BLOCKED), physicalObject(physicalObject) {}

    bool isObject() {
        return true;
    }

    void setPhysicalObject(PhysicalObject * physicalObject) {
        this->physicalObject = physicalObject;
    }

private:
    PhysicalObject * physicalObject;
};

#endif // CELL_H
