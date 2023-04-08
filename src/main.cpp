#include <stdio.h>
#include "utils.hpp"
#include "ShapeDecomposer.hpp"
#include "TrajectoryOptimizer.hpp"

#include <casadi/casadi.hpp>

using namespace casadi;

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

    TrajectoryOptimizer trajOpt;

    DM start = DM::zeros(6);
    DM goal = DM::zeros(6);
    goal(1) = goal(2) = goal(0) = 2;
    trajOpt.solveDoubleIntegrator(start, goal);

    return 0;
}