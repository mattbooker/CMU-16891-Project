#ifndef UTILS_H
#define UTILS_H


struct Point3
{
    float x, y, z;
};

struct BoundingBox
{
    Point3 min;
    Point3 max;
};

#endif