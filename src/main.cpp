#include <stdio.h>
#include <iostream>
#include <chrono>
#include "utils.hpp"
#include "ShapeDecomposerSphere.hpp"
#include "ShapeDecomposer.hpp"
#include "TrajectoryOptimizer.hpp"
#include "CBSSolver.hpp"
#include "goalAllocator.hpp"
#include <random>

using namespace casadi;

int main(int argv, char *argc[])
{
    // srand(time(0));
    const int numAgents = 16;
    const BoundingBox envBbox = {{-5, -5, -5}, {5, 5, 5}};
    const float sphereRadius = 5.0;

    //Spherical shape decomposer
    ShapeDecomposerSphere decomposer(numAgents, envBbox, sphereRadius);
    std::vector<Point3> endpoints = decomposer.decomposeShapeSphere();

    
    std::vector<Point3> starts;

    for (int i = -2; i < 2; i++)
    {
        for (int j = -2; j < 2; j++)
        {
            starts.push_back({2*i, 2*j, -2});
        }
    }
    printf("Num of starts: %d \n", starts.size());
    printf("Num of ends: %d\n", endpoints.size());

    std::vector<std::vector<int>> assignment;
    std::ifstream file_in("../assignments/16_agents_assignment.txt");
    if (!file_in) {/*error*/}

    std::string line;
    while (std::getline(file_in, line)) // Read next line to `line`, stop if no more lines.
    {
        // Construct so called 'string stream' from `line`, see while loop below for usage.
        std::istringstream ss(line);

        assignment.push_back({}); // Add one more empty vector (of vectors) to `vec`.

        int x;
        while (ss >> x) // Read next int from `ss` to `x`, stop if no more ints.
            assignment.back().push_back(x); // Add it to the last sub-vector of `vec`.
    }

    std::vector<Point3> endpoints2;
    // std::vector<int> idx;
    for (int i = 0; i < numAgents; i++){
        for (int j = 0; j < numAgents; j++){
            // printf("%d\n", assignment[i][j]);
            if (assignment[i][j] ==1){
                endpoints2.push_back(endpoints[j]);
                // idx.push_back(j);
            }
        }
    }

    printf("Reassigned endpoints size: %d\n", endpoints2.size());

    auto beg = std::chrono::high_resolution_clock::now();
    CBSSolver solver;

    MAPFInstance problem = {numAgents, envBbox, starts, endpoints2};

    std::vector<QuadTrajectory> answer = solver.solve(problem);

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - beg);
 
    // Displaying the elapsed time
    std::cout << "Elapsed Time: " << duration.count() * 1e-6 << std::endl;

    // Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "", "");

    // for (int i = 0; i < answer.size(); i++)
    // {
    //     for (int j = 0; j < answer[i].size(); j++)
    //     {
    //       std::cout << answer[i][j].format(CommaInitFmt) << std::endl;
    //     }

    //     printf("---\n");
    // }

    return 0;
}