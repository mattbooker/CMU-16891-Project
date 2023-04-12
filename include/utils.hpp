#ifndef UTILS_H
#define UTILS_H

#include <utility>
#include <vector>

struct Point3
{
    float x, y, z;

    bool operator== (const Point3& rhs) const
    {
        return x == rhs.x && y == rhs.y && z == rhs.z;
    }
};

struct BoundingBox
{
    Point3 min;
    Point3 max;
};

struct MAPFInstance
{
    int numAgents;
    BoundingBox env;

    std::vector<Point3> startLocs;
    std::vector<Point3> goalLocs;
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