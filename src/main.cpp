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
    Params params;
    params.start = {0, 0, 0};
    params.goal = {2, 2, 2};
    params.tf = 3.0;
    params.dt = 0.1;
    params.N = int(params.tf / params.dt);
    params.colRadiusSq = pow(0.8 + 0.2, 2.0);

    QuadTrajectory x;
    QuadControls u;
    Constraint b1 = {MX::ones(3)};
    Constraint b2 = {MX::ones(3)};
    Constraint b3 = {MX::ones(3)};
    b2.location(2) = 2;
    b3.location(2) = 0;
    std::vector<Constraint> constraints = {b1, b2, b3};


    trajOpt.solveQuadcopter(params, constraints, x, u);

    for (int i = 0; i < x.size(); i++)
    {
        printf("%f, %f, %f\n", x[i](0), x[i](1), x[i](2));
        // printf("%f, %f, %f, %f\n", u[i](0), u[i](1), u[i](2), u[i](3));
    }

    return 0;
}