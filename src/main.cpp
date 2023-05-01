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
    auto beg = std::chrono::high_resolution_clock::now();

    const int numAgents = 196;
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

    for (int i = -7; i < 7; i++)
    {
        for (int j = -7; j < 7; j++)
        {
            starts.push_back({2*i, 2*j, -2});
        }
    }
    printf("Num of starts: %d \n", starts.size());
    printf("Num of ends: %d\n", endpoints.size());

    //random starts
    // int max_x = envBbox.max.x;
    // int max_y = envBbox.max.y;
    // int max_z = envBbox.max.z;
    // int min_x = envBbox.min.x;
    // int min_y = envBbox.min.y;
    // int min_z = envBbox.min.z;
    // for (int i = 0; i < numAgents; i++)
    // {
    //     int rand_x = rand()%(max_x-min_x + 1) + min_x;
    //     int rand_y = rand()%(max_y-min_y + 1) + min_y;
    //     int rand_z = rand()%(max_z-min_z + 1) + min_z;
    //     starts.push_back({rand_x, rand_y, rand_z});
    // }

    // std::vector<Point3> endpoints;

    // for (int i = 0; i < numAgents; i++)
    // {
    //     int rand_x = rand()%(max_x-min_x + 1) + min_x;
    //     int rand_y = rand()%(max_y-min_y + 1) + min_y;
    //     int rand_z = rand()%(max_z-min_z + 1) + min_z;
    //     endpoints.push_back({rand_x, rand_y, rand_z});
    // }


    std::vector<std::vector<int>> costMatrix;
    for (int i = 0; i < numAgents; i++){
        costMatrix.push_back(std::vector<int>());
        for (int j = 0; j < endpoints.size(); j++){
            costMatrix[i].push_back(int(powf(powf(endpoints[j].x - starts[i].x, 2) 
                                    + powf(endpoints[j].y - starts[i].y, 2) 
                                    + powf(endpoints[j].z - starts[i].z, 2), 0.5)));
            
        }
    } 

    for (int i = 0; i < numAgents; i++){
        printf("{");
        for (int j = 0; j < endpoints.size(); j++){
            if (j == endpoints.size()-1)
                printf("%f", costMatrix[i][j]);
            else
                printf("%f, ", costMatrix[i][j]);
        }
        printf("},\n");
    }

    // for (int i = 0; i<endpoints.size(); i++)
    // {
    //     printf("%f, %f, %f \n", endpoints[i].x, endpoints[i].y, endpoints[i].z);
    // }

    std::vector<std::vector<int>> assignment;
    std::ifstream file_in("../assignments/196_agents_assignment.txt");
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

    // goalAllocator hungarian(costMatrix, numAgents, numAgents);
    // printf("Calling solve \n");
    // std::vector<std::vector<int>> assignment = hungarian.solve();

    // std::vector< std::vector<int> >::iterator row;
    // std::vector<int>::iterator col;
    // for (row = assignment.begin(); row != assignment.end(); row++) {
    //     for (col = row->begin(); col != row->end(); col++) {
    //         printf("%d ", col);
    //     }
    //     printf("\n");
    // }

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

    
    // printf("Endpoints size: %d\n", endpoints.size());
    // for (int i = 0; i < idx.size(); i++)
    // {
    //     printf("Idx i: %d\n", idx[i]);
    //     endpoints.erase(endpoints.begin() + idx[i]);
    // }

    // printf("Remaining endpoints: %d\n", endpoints.size());
    // int count = 0;
    // while(endpoints.size() > 0)
    // {
    //     endpoints2.push_back(endpoints[count]);
    //     endpoints.erase(endpoints.begin() + count);
    //     count++;
    // }

    printf("Reassigned endpoints size: %d\n", endpoints2.size());

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

    MAPFInstance problem = {numAgents, envBbox, starts, endpoints2};

    std::vector<QuadTrajectory> answer = solver.solve(problem);

    // for (int i = 0; i < numAgents; i++)
    // {
    //     printf("(%f, %f, %f) -> (%f, %f, %f)\n", starts[i].x, starts[i].y, starts[i].z, endpoints[i].x, endpoints[i].y, endpoints[i].z);
    // }

    // for (int i = 0; i < answer.size(); i++)
    // {
    //     for (int j = 0; j < answer[i].size(); j++)
    //     {
    //         printf("%f, %f, %f\n", answer[i][j](0), answer[i][j](1), answer[i][j](2));
    //     }

    //     printf("---\n");
    // }

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - beg);
 
    // Displaying the elapsed time
    std::cout << "Elapsed Time: " << duration.count();

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