#ifndef CELL_H
#define CELL_H

#include <string>


class Cell {
public:

    enum Type {
        UNKNOWN, FREE, BLOCKED
    };


    Cell(int x, int y, Type cellValue) : x(x), y(y), type(cellValue){}

    virtual bool isObject() {
        return false;
    }

    virtual std::string toString() {
        switch (type)
        {
        case UNKNOWN:
            return "unknown";
        case FREE:
            return "free";
        case BLOCKED:
            return "blocked";
        }
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

    int getX()
    {
        return x;
    }

    int getY()
    {
        return y;
    }

private:
    Type type;
    int x;
    int y;
};


class ObjectCell : public Cell {
public:
    ObjectCell(int x, int y, std::string object_name) : Cell(x, y, Cell::BLOCKED), object_name_(object_name) {}

    bool isObject() {
        return true;
    }

    std::string toString()
    {
        return object_name_;
    }

private:
    const std::string object_name_;
};

#endif // CELL_H
