#include <stdio.h>
#include "utils.hpp"
#include "ShapeDecomposer.hpp"
#include "TrajectoryOptimizer.hpp"
#include "CBSSolver.hpp"

using namespace casadi;

Timer* Timer::timer_ = nullptr;

int main(int argv, char *argc[])
{
    const int numAgents = 100;
    const BoundingBox envBbox = {{-15, -15, -15}, {15, 15, 15}};

    // Presteps: Get as input a shape

    // 1. Get endpoints for N number of agents from shape decomposer
    ShapeDecomposer decomposer(numAgents, envBbox);
    std::vector<Point3> endpoints = decomposer.decomposeShape();

    std::vector<Point3> starts;

    for (int i = -5; i < 5; i++)
    {
        for (int j = -5; j < 5; j++)
        {
            starts.push_back({2*i, 2*j, 0});
        }
    }

    // 2. Determine assignment using Anonymous MAPF (min cut max flow)

    // 3. Solve for the trajectories of each agent

    CBSSolver solver;

    MAPFInstance problem = {numAgents, envBbox, endpoints, starts};

    Timer* test = Timer::getInstance();
    test->start("Full");
    std::vector<QuadTrajectory> answer = solver.solve(problem);
    test->stop("Full");
    test->printAllTimers();

    Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "", "");

    for (int i = 0; i < answer.size(); i++)
    {
        for (int j = 0; j < answer[i].size(); j++)
        {
          std::cout << answer[i][j].format(CommaInitFmt) << std::endl;
        }

        printf("---\n");
    }

    return 0;
}
