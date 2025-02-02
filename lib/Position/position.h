#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <cmath>
class Position{

private:
    float x, y, z;
    bool compareValues(float t1, float t2, float df);

public:

    Position(float x, float y, float z);
    Position();

    float getX();
    float getY();
    float getZ();

    void setX(float x);
    void setY(float y);
    void setZ(float z);

    bool isOnPosition(Position pos);
    bool isOnPosition(Position pos, float dx, float dy, float dz);
};

#endif