#include "CBSSolver.hpp"
#include <queue>

std::vector<QuadTrajectory> CBSSolver::solve(const MAPFInstance &instance)
{
    printf("Starting CBS Solver\n");

    // Initialize low level solver
    TrajectoryOptimizer lowLevelSolver;
    Params params;
    params.tf = 5.0;
    params.dt = 0.1;
    params.N = int(params.tf / params.dt);
    params.colDistSq = colDistSq;

    // Create priority queue
    std::priority_queue<CTNodeSharedPtr,
                        std::vector<CTNodeSharedPtr>,
                        CTNodeComparator>
        pq;

    CTNodeSharedPtr root = std::make_shared<CTNode>();
    root->paths.resize(instance.numAgents);
    root->id = numNodesGenerated++;

    // for (int i = 0; i < instance.numAgents; i++)
    // {
    //     for (int j = 0; j < instance.numAgents; j++)
    //     {
    //         if (j == i) continue;

    //         Eigen::Vector3f con;
    //         con << instance.goalLocs[j].x, instance.goalLocs[j].y, instance.goalLocs[j].z;
    //         root->constraintList.push_back({i, params.N, con}); 
    //     }
    // }

    // Create paths for all agents
    for (int i = 0; i < instance.startLocs.size(); i++)
    {
        params.start = instance.startLocs[i];
        params.goal = instance.goalLocs[i];
        bool found = lowLevelSolver.solveQuadcopter(params, root->constraintList, QuadTrajectory(), root->paths[i], i);

        if (!found)
            throw NoSolutionException();
    }

    root->cost = computeCost(root->paths);
    detectCollisions(root->paths, root->collisionList);

    pq.push(root);

    while (!pq.empty())
    {
        CTNodeSharedPtr cur = pq.top();
        pq.pop();

        // If no collisions in the node then return solution
        if (cur->collisionList.size() == 0)
        {
            std::cout << "Num Cons = " << cur->constraintList.size() << std::endl;
            return cur->paths;
        }

        // Get first collision and create two nodes (each containing a new plan for the two agents in the collision)
        for (Constraint &c : resolveCollision(cur->collisionList[0]))
        {
            // Add new constraint
            CTNodeSharedPtr child = std::make_shared<CTNode>();
            child->constraintList = cur->constraintList;
            child->constraintList.push_back(c);
            child->paths = cur->paths;

            // Replan only for the agent that has the new constraint

            QuadTrajectory prev = child->paths[c.agentNum];
            child->paths[c.agentNum].clear();
            params.start = instance.startLocs[c.agentNum];
            params.goal = instance.goalLocs[c.agentNum];

            bool success = lowLevelSolver.solveQuadcopter(params, child->constraintList, prev, child->paths[c.agentNum], c.agentNum);

            if (success)
            {
                // Update cost and find collisions
                child->cost = computeCost(child->paths);
                detectCollisions(child->paths, child->collisionList);

                // Set id and increment numNodesGenerated
                child->id = numNodesGenerated++;

                // Add to search queue
                pq.push(child);
            }
        }
    }

    throw NoSolutionException();
}

float inline CBSSolver::computeCost(const std::vector<QuadTrajectory> &paths)
{
    float result = 0;

    // Cost of a node is the total length of all paths
    for (int i = 0; i < paths.size(); i++)
    {
        for (int j = 1; j < paths[i].size(); j++)
        {
            result += (paths[i][j](Eigen::seq(0,2)) - paths[i][j - 1](Eigen::seq(0,2))).squaredNorm();
        }
    }

    return result;
}

void CBSSolver::detectCollisions(const std::vector<QuadTrajectory> &paths, std::vector<Collision> &collisionList)
{
    Collision col;
    collisionList.clear();

    for (int i = 0; i < paths.size() - 1; i++)
    {
        for (int j = i + 1; j < paths.size(); j++)
        {
            getCollisionsAgents(paths[i], paths[j], i, j, collisionList);
        }
    }
}

inline std::vector<Constraint> CBSSolver::resolveCollision(const Collision &col)
{
    return std::vector<Constraint>{Constraint{col.agent1, col.t, col.location.second}, Constraint{col.agent2, col.t, col.location.first}};
}

void CBSSolver::getCollisionsAgents(QuadTrajectory agent1, QuadTrajectory agent2, int a1, int a2, std::vector<Collision> &collisionList)
{
    Eigen::Vector3f line1;
    Eigen::Vector3f line2;

    for (int i = 0; i < agent1.size() - 2; i++)
    {
        Eigen::Vector3f a1_pos1 = agent1[i](Eigen::seq(0, 2));
        Eigen::Vector3f a1_pos2 = agent1[i + 1](Eigen::seq(0, 2));

        getLineEq(a1_pos1, a1_pos2, line1);

        Eigen::Vector3f a2_pos1 = agent2[i](Eigen::seq(0, 2));
        Eigen::Vector3f a2_pos2 = agent2[i + 1](Eigen::seq(0, 2));

        getLineEq(a2_pos1, a2_pos2, line2);

        auto ans = findSolution(a1_pos1, a2_pos1, line1, line2);
        Eigen::Vector3f points1 = a1_pos1 + ans.first * line1;
        Eigen::Vector3f points2 = a2_pos1 + ans.second * line2;

        float shortestDistSq = powf(points2[0] - points1[0], 2) + powf(points2[1] - points1[1], 2) + powf(points2[2] - points1[2], 2);

        if (shortestDistSq <= colDistSq)
        {
            collisionList.push_back({a1, a2, i, std::make_pair(points1, points2)});
        }
    }
}

void inline CBSSolver::getLineEq(Eigen::Vector3f pos1, Eigen::Vector3f pos2, Eigen::Vector3f &line)
{
    // get line equation between points
    line[0] = pos2[0] - pos1[0];
    line[1] = pos2[1] - pos1[1];
    line[2] = pos2[2] - pos1[2];
}

std::pair<float, float> CBSSolver::findSolution(Eigen::Vector3f a1_pos1, Eigen::Vector3f a2_pos1, Eigen::Vector3f line1, Eigen::Vector3f line2)
{
    float A[2][2] = {
        {powf(line1[0], 2) + powf(line1[1], 2) + powf(line1[2], 2), -(line1[0] * line2[0] + line1[1] * line2[1] + line1[2] * line2[2])},
        {line1[0] * line2[0] + line1[1] * line2[1] + line1[2] * line2[2], -(powf(line2[0], 2) + powf(line2[1], 2) + powf(line2[2], 2))}};

    float B[2][1] = {
        {-(a1_pos1[0] * line1[0] - a2_pos1[0] * line1[0] + a1_pos1[1] * line1[1] - a2_pos1[1] * line1[1] + a1_pos1[2] * line1[2] - a2_pos1[2] * line1[2])},
        {-(a1_pos1[0] * line2[0] - a2_pos1[0] * line2[0] + a1_pos1[1] * line2[1] - a2_pos1[1] * line2[1] + a1_pos1[2] * line2[2] - a2_pos1[2] * line2[2])}};

    float det_A = (A[0][0] * A[1][1]) - (A[0][1] * A[1][0]);
    float det_X = (B[0][0] * A[1][1]) - (B[1][0] * A[0][1]);
    float det_Y = (A[0][0] * B[1][0]) - (A[1][0] * B[0][0]);

    float x = det_X / det_A;
    if (x > 1)
    {
        x = 1;
    }
    else if (x < 0)
    {
        x = 0;
    }
    float y = det_Y / det_A;
    if (y > 1)
    {
        y = 1;
    }
    else if (y < 0)
    {
        y = 0;
    }

    return std::make_pair(x, y);
}