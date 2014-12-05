#ifndef CELL_H
#define CELL_H

#include <string>

#define FREE_STANDARD_VALUE 10

class Cell {
public:

    enum Type {
        UNKNOWN, FREE, BLOCKED
    };

    Cell() : cost_(FREE_STANDARD_VALUE){}

    Cell(int i, int j, Type cellValue, int cost) : cost_(cost), i(i), j(j), type(cellValue){}

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

    Type getType()
    {
        return type;
    }

    int getI()
    {
        return i;
    }

    int getJ()
    {
        return j;
    }

    int getCost()
    {
        return cost_;
    }

private:
    Type type;
    int i;
    int j;
    int cost_;
};


class ObjectCell : public Cell {
public:
    ObjectCell(int i, int j, std::string object_name, int cost) : Cell(i, j, Cell::BLOCKED, cost), object_name_(object_name) {}

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
