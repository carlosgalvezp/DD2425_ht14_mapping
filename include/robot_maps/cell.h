#ifndef CELL_H
#define CELL_H

#include <string>


class Cell {
public:

    enum Type {
        UNKNOWN, FREE, BLOCKED
    };


    Cell(int i, int j, Type cellValue) : i(i), j(j), type(cellValue){}

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

    int getI()
    {
        return i;
    }

    int getJ()
    {
        return j;
    }

private:
    Type type;
    int i;
    int j;
};


class ObjectCell : public Cell {
public:
    ObjectCell(int i, int j, std::string object_name) : Cell(j, j, Cell::BLOCKED), object_name_(object_name) {}

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
