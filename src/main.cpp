#include <stdio.h>
#include "utils.hpp"
#include "ShapeDecomposer.hpp"
#include "TrajectoryOptimizer.hpp"
#include "CBSSolver.hpp"

using namespace casadi;

int main()
{
    const int numAgents = 3;
    const BoundingBox envBbox = {{-5, -5, -5}, {5, 5, 5}};

    // Presteps: Get as input a shape

    // 1. Get endpoints for N number of agents from shape decomposer
    ShapeDecomposer decomposer(numAgents, envBbox);
    std::vector<Point3> endpoints = decomposer.decomposeShape();

    // 2. Determine assignment using Anonymous MAPF (min cut max flow)

    // 3. Solve for the trajectories of each agent
    std::vector<Point3> starts = {{-3, -3, -3}, {1, -3, -3}, {2, -3, -3}};

    CBSSolver solver;

    MAPFInstance problem = {numAgents, envBbox, starts, endpoints};

    solver.solve(problem);

    return 0;
}