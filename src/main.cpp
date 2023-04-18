#include <stdio.h>
#include "utils.hpp"
#include "ShapeDecomposerSphere.hpp"
#include "ShapeDecomposer.hpp"
#include "TrajectoryOptimizer.hpp"
#include "CBSSolver.hpp"

using namespace casadi;

int main(int argv, char *argc[])
{
    const int numAgents = 144;
    const BoundingBox envBbox = {{-5, -5, -5}, {5, 5, 5}};
    const float sphereRadius = 5.0;

    // Presteps: Get as input a shape

    // 1. Get endpoints for N number of agents from shape decomposer
    // ShapeDecomposer decomposer(numAgents, envBbox);
    // std::vector<Point3> endpoints = decomposer.decomposeShape();

    //Spherical shape decomposer
    ShapeDecomposerSphere decomposer(numAgents, envBbox, sphereRadius);
    std::vector<Point3> endpoints = decomposer.decomposeShapeSphere();

    // std::vector<Point3> starts = {{0, 0, 1}, {0, 0, 2}, {0, 0, 3}, {0, 0, 4}};
    // std::vector<Point3> starts = {{0, 0, -3}, {0, 0, -2}, {0, 0, -1}, {0, 0, 0}, {0, 0, 1}, {0, 0, 2}, {0, 0, 3}, {0, 0, 4}, {0, 0, 5}};
    // std::vector<Point3> endpoints = {{0, 2, -1}, {0, 2, -2}};

    
    std::vector<Point3> starts;

    for (int i = -6; i < 6; i++)
    {
        for (int j = -6; j < 6; j++)
        {
            starts.push_back({2*i, 2*j, -6});
        }
    }
    printf("Num of starts: %d \n", starts.size());
    printf("Num of ends: %d\n", endpoints.size());

    for (int i = 0; i<endpoints.size(); i++)
    {
        printf("%f, %f, %f \n", endpoints[i].x, endpoints[i].y, endpoints[i].z);
    }

    // 2. Determine assignment using Anonymous MAPF (min cut max flow)

    // 3. Solve for the trajectories of each agent
    // float width = envBbox.max.x - envBbox.min.x;

    // for (int i = 0; i < numAgents; i++)
    // {
    //     while (true)
    //     {
    //         bool found = false;
    //         float x = ((float) rand()/RAND_MAX) * width - width/2;
    //         float y = (float) rand()/RAND_MAX * width - width/2;
    //         float z = (float) rand()/RAND_MAX * width - width/2;

    //         for (int j = 0; j < starts.size(); j++)
    //         {
    //             float dist = pow((x - starts[j].x), 2) + pow((y - starts[j].y), 2) + pow((z - starts[j].z), 2);
    //             if (dist > 0.2)
    //             {
    //                 found = true;
    //                 break;
    //             }
    //         }

    //         if (found || starts.size() == 0)
    //         {
    //             starts.push_back({x, y, z});
    //             break;
    //         }
    //     }
    // }

    CBSSolver solver;

    MAPFInstance problem = {numAgents, envBbox, starts, endpoints};

    std::vector<QuadTrajectory> answer = solver.solve(problem);

    // for (int i = 0; i < numAgents; i++)
    // {
    //     printf("(%f, %f, %f) -> (%f, %f, %f)\n", starts[i].x, starts[i].y, starts[i].z, endpoints[i].x, endpoints[i].y, endpoints[i].z);
    // }

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