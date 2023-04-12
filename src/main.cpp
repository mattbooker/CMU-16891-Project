#include <stdio.h>
#include "utils.hpp"
#include "ShapeDecomposer.hpp"
#include "TrajectoryOptimizer.hpp"
#include "CBSSolver.hpp"

using namespace casadi;

int main()
{
    const int numAgents = 7;
    const BoundingBox envBbox = {{-5, -5, -5}, {5, 5, 5}};

    // Presteps: Get as input a shape

    // 1. Get endpoints for N number of agents from shape decomposer
    ShapeDecomposer decomposer(numAgents, envBbox);
    std::vector<Point3> endpoints = decomposer.decomposeShape();

    // 2. Determine assignment using Anonymous MAPF (min cut max flow)

    // 3. Solve for the trajectories of each agent
    std::vector<Point3> starts;
    float x = envBbox.min.x;
    float y = envBbox.min.y;
    float z = envBbox.min.z;

    for (int i = 0; i < numAgents; i++)
    {
        starts.push_back({x, y, z});
    }

    CBSSolver solver;

    MAPFInstance problem = {numAgents, envBbox, starts, endpoints};

    solver.solve(problem);

    // TrajectoryOptimizer trajOpt;
    // Params params;
    // params.start = {0, 0, 0};
    // params.goal = {2, 2, 2};
    // params.tf = 3.0;
    // params.dt = 0.1;
    // params.N = int(params.tf / params.dt);
    // params.colRadiusSq = pow(0.8 + 0.2, 2.0);

    // QuadTrajectory x;
    // QuadControls u;
    // Constraint b1 = {0, 0, {1, 1, 1}};
    // Constraint b2 = {0, 0, {1, 1, 2}};
    // Constraint b3 = {0, 0, {1, 1, 0}};
    // std::vector<Constraint> constraints = {b1, b2, b3};


    // trajOpt.solveQuadcopter(params, constraints, x, u);

    // for (int i = 0; i < x.size(); i++)
    // {
    //     printf("%f, %f, %f\n", x[i](0), x[i](1), x[i](2));
    // }

    return 0;
}