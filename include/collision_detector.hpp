#include "TrajectoryOptimizer.hpp"
#include "QuadCopter.hpp"
#include "utils.hpp"
#include<iostream>
#include<cmath>

using namespace std;



class CollisionDetector
{
    public: 
    
        CollisionDetector(const Params &params, vector<QuadTrajectory> &InstancePlan);

        void get_collisions(Params &params, vector<QuadTrajectory> &InstancePlan);
        void get_collisons_agents(Params &params, QuadStateVector agent1, QuadStateVector agent2
                                    int a1, int a2); //returns collisions between two agents
        void get_line_eq(vector<float> pos1, vector<float> pos2, Eigen::Vector3d line);
        pair<double, double> find_solution(Eigen::Vector3d a1_pos1, Eigen::Vector3d a1_pos2, Eigen::Vector3d line1, Eigen::Vector3d line2);

        Eigen::Vector3d crossP; //cross product between two lines
        float shortest_dist_sq; //shortest distance between two lines
        vector<Collision> collisions; //returns collisions between all agents
        Eigen::Vector3d line1;
        Eigen::Vector3d line2;
};

