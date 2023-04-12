#ifndef UTILS_H
#define UTILS_H

#include <utility>
#include <vector>
#include <eigen3/Eigen/Core>

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

    std::pair<Eigen::Vector3d, Eigen::Vector3d> location;
};

struct Constraint
{
    int agentNum;
    int t;
    Eigen::Vector3d location;
};

#endif