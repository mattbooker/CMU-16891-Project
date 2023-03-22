#include <stdio.h>
#include "utils.h"
#include "ShapeDecomposer.hpp"

int main()
{
    const int num_of_agents = 7;
    const BoundingBox env_bbox = {{-5, -5, -5}, {5, 5, 5}};

    // Presteps: Get as input a shape

    // 1. Get endpoints for N number of agents from shape decomposer
    ShapeDecomposer decomposer(num_of_agents, env_bbox);
    std::vector<Point3> endpoints = decomposer.decomposeShape();

    // 2. Determine assignment using Anonymous MAPF (min cut max flow)

    // 3. Solve for the trajectories of each agent
    return 0;
}