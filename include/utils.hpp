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

struct Collision
{
    int agent1;
    int agent2;
    int t;

    std::pair<Point3, Point3> location;
};

struct Constraint
{
    int agentNum;
    int t;
    Point3 location;
};

#endif