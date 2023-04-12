#include "collision_detector.hpp"


void CollisionDetector::get_collisions(Params &params, vector<QuadTrajectory> &InstancePlan)
{
    for (int i= 0; i<InstancePlan.size()-1; i++){
        for (int j=i+1; j<InstancePlan.size()-1; j++){
            QuadTrajectory agent1 = InstancePlan[i];
            QuadTrajectory agent2 = InstancePlan[j];
            get_collisons_agents(&params, agent1, agent2, i, j)

        }
    }
}

void CollisionDetector::get_collisons_agents(Params &params, QuadTrajectory agent1, QuadTrajectory agent2, int a1, int a2)
{
    for (int i=0; i<agent1.size()-2; i++)
    {
        Eigen::Vector3d a1_pos1 = agent1[i][0:3];
        Eigen::Vector3d a1_pos2 = agent1[i+1][0:3];

        get_line_eq(a1_pos1, a1_pos2, line1);

        Eigen::Vector3d a2_pos1 = agent2[i][0:3];
        Eigen::Vector3d a2_pos2 = agent2[i+1][0:3];

        get_line_eq(a2_pos1, a2_pos2, line2);

        auto ans = find_solution(a1_pos1, a2_pos1);
        Eigen::Vector3d points1 = a1_pos1 + ans.first * line1;
        Eigen::Vector3d points2 = a2_pos1 + y.second * line2;

        shortest_dist_sq = pow(points2[0] - points1[0], 2) + pow(points2[1] - points1[1], 2) + pow(points2[2] - points1[2], 2);

        if (shortest_dist_sq < params.colRadiusSq)
        {
            collisions.push_back({a1, a2, i, make_pair(points1, points2)});
        }
    }
}

void CollisionDetector::get_line_eq(vector<float> pos1, vector<float> pos2, Eigen::Vector3d &line)
{
    //get line equation between points
    vector<float> line; 
    line[0] = pos2[0] - pos1[0]; 
    line[1] = pos2[1] - pos1[1]; 
    line[2] = pos2[2] - pos1[2];

}

pair<double, double> CollisionDetector::find_solution(Eigen::Vector3d a1_pos1, Eigen::Vector3d a2_pos1)
{
    double A[2][2] = {
        {pow(line1[0], 2) + pow(line1[1], 2) + pow(line1[2], 2), -(line1[0]*line2[0] + line1[1]*line2[1] + line1[2]*line2[2])},
        {line1[0]*line2[0] + line1[1]*line2[1] + line1[2]*line2[2], -(pow(line2[0], 2) + pow(line2[1], 2) + pow(line2[2], 2))}
    };
    
    double B[2][1] = {
        {-(a1_pos1[0]*line1[0] - a2_pos1[0]*line1[0] + a1_pos1[1]*line1[1] - a2_pos1[1]*line1[1] + a1_pos1[2]*line1[2] - a2_pos1[2]*line1[2])},
        {-(a1_pos1[0]*line2[0] - a2_pos1[0]*line2[0] + a1_pos1[1]*line2[1] - a2_pos1[1]*line2[1] + a1_pos1[2]*line2[2] - a2_pos1[2]*line2[2])}
    };

    double det_A = (A[0][0] * A[1][1]) - (A[0][1] * A[1][0]);
    double det_X = (B[0][0] * A[1][1]) - (B[1][0] * A[0][1]);
    double det_Y = (A[0][0] * B[1][0]) - (A[1][0] * B[0][0]);

    double x = det_X / det_A;
    if (x > 1){
        x = 1;
    }
    else if (x < 0){
        x = 0;
    }
    double y = det_Y / det_A;
    if (y > 1){
        y = 1;
    }
    else if (y < 0){
        y = 0;
    }

    return make_pair(x, y);
}

