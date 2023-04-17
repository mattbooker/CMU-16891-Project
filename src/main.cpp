#include <stdio.h>
#include "utils.hpp"
#include "ShapeDecomposer.hpp"
#include "TrajectoryOptimizer.hpp"
#include "CBSSolver.hpp"

using namespace casadi;

int main(int argv, char *argc[])
{
    const int numAgents = 16;
    const BoundingBox envBbox = {{-5, -5, -5}, {5, 5, 5}};

    // Presteps: Get as input a shape

    // 1. Get endpoints for N number of agents from shape decomposer
    ShapeDecomposer decomposer(numAgents, envBbox);
    std::vector<Point3> endpoints = decomposer.decomposeShape();

    std::vector<Point3> starts;

    for (int i = -2; i < 2; i++)
    {
        for (int j = -2; j < 2; j++)
        {
            starts.push_back({2*i, 2*j, 0});
        }
    }

    // 2. Determine assignment using Anonymous MAPF (min cut max flow)

    // 3. Solve for the trajectories of each agent

    CBSSolver solver;

    MAPFInstance problem = {numAgents, envBbox, starts, endpoints};

    auto startTime = std::chrono::high_resolution_clock::now();

    std::vector<QuadTrajectory> answer = solver.solve(problem);

    auto stopTime = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stopTime - startTime);
    std::cout << "Time = " << duration.count() * 1e-6 << std::endl;

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