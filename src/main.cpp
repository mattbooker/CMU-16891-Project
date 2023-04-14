#include <stdio.h>
#include "utils.hpp"
#include "ShapeDecomposer.hpp"
#include "TrajectoryOptimizer.hpp"
#include "CBSSolver.hpp"

using namespace casadi;

int main()
{
    const int numAgents = 8;
    const BoundingBox envBbox = {{-5, -5, -5}, {5, 5, 5}};

    // Presteps: Get as input a shape

    // 1. Get endpoints for N number of agents from shape decomposer
    ShapeDecomposer decomposer(numAgents, envBbox);
    std::vector<Point3> endpoints = decomposer.decomposeShape();

    // 2. Determine assignment using Anonymous MAPF (min cut max flow)

    // 3. Solve for the trajectories of each agent
    std::vector<Point3> starts;

    for (int i = 0; i < numAgents; i++)
    {
        starts.push_back({-4 + i, -4, -4});
    }

    CBSSolver solver;

    MAPFInstance problem = {numAgents, envBbox, starts, endpoints};

    std::vector<QuadTrajectory> answer = solver.solve(problem);

    for (int i = 0; i < numAgents; i++)
    {
        printf("(%f, %f, %f) -> (%f, %f, %f)\n", starts[i].x, starts[i].y, starts[i].z, endpoints[i].x, endpoints[i].y, endpoints[i].z);
    }

    for (int i = 0; i < answer.size(); i++)
    {
        for (int j = 0; j < answer[i].size(); j++)
        {
            printf("%f, %f, %f\n", answer[i][j](0), answer[i][j](1), answer[i][j](2));
        }

        printf("---\n");
    }

    return 0;
}