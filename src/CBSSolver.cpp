#include "CBSSolver.hpp"
#include <queue>

CBSSolver::CBSSolver()
    : numNodesGenerated(0), colRadiusSq(1.0)
{
}

std::vector<QuadTrajectory> CBSSolver::solve(const MAPFInstance &instance)
{
    printf("Starting CBS Solver\n");

    // Initialize low level solver
    TrajectoryOptimizer lowLevelSolver;
    Params params;
    params.tf = 3.0;
    params.dt = 0.1;
    params.N = int(params.tf / params.dt);
    params.colRadiusSq = powf(colRadiusSq, 2.0);

    // Create priority queue
    std::priority_queue<CTNodeSharedPtr,
                        std::vector<CTNodeSharedPtr>,
                        CTNodeComparator>
        pq;

    CTNodeSharedPtr root = std::make_shared<CTNode>();
    root->paths.resize(instance.numAgents);
    root->id = numNodesGenerated++;

    // Create paths for all agents
    for (int i = 0; i < instance.startLocs.size(); i++)
    {
        params.start = instance.startLocs[i];
        params.goal = instance.goalLocs[i];
        QuadControls unused;
        bool found = lowLevelSolver.solveQuadcopter(params, root->constraintList, root->paths[i], unused);

        if (!found)
            throw NoSolutionException();
    }

    root->cost = computeCost(root->paths);
    printf("a\n");
    detectCollisions(root->paths, root->collisionList);
    printf("b\n");

    pq.push(root);

    while (!pq.empty())
    {
        CTNodeSharedPtr cur = pq.top();
        pq.pop();

        printf("a\n");

        // If no collisions in the node then return solution
        if (cur->collisionList.size() == 0)
        {
            return cur->paths;
        }
        printf("b\n");

        // Get first collision and create two nodes (each containing a new plan for the two agents in the collision)
        for (Constraint &c : resolveCollision(cur->collisionList[0]))
        {
            printf("c\n");
            // Add new constraint
            CTNodeSharedPtr child = std::make_shared<CTNode>();
            child->constraintList = cur->constraintList;
            child->constraintList.push_back(c);
            child->paths = cur->paths;
            // printf("New cons = %d (%d,%d) %d @ %d\n", c.agentNum, c.location.first.x, c.location.first.y, c.isVertexConstraint, c.t);

            printf("d\n");
            // Replan only for the agent that has the new constraint
            // printf("Before = %d %d\n", child->paths[c.agentNum].size(), computeCost(child->paths));
            child->paths[c.agentNum].clear();
            params.start = instance.startLocs[c.agentNum];
            params.goal = instance.startLocs[c.agentNum];
            QuadControls unused;
            bool success = lowLevelSolver.solveQuadcopter(params, child->constraintList, child->paths[c.agentNum], unused);
            // printf("After = %d %d\n", child->paths[c.agentNum].size(), computeCost(child->paths));
            printf("e\n");

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

int inline CBSSolver::computeCost(const std::vector<QuadTrajectory> &paths)
{
    int result = 0;

    for (int i = 0; i < paths.size(); i++)
    {
        result += paths[i].size() - 1;
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
        printf("x\n");
        Eigen::Vector3f a1_pos1 = agent1[i](Eigen::seq(0, 3));
        Eigen::Vector3f a1_pos2 = agent1[i + 1](Eigen::seq(0, 3));
        printf("y\n");

        getLineEq(a1_pos1, a1_pos2, line1);

        Eigen::Vector3f a2_pos1 = agent2[i](Eigen::seq(0, 3));
        Eigen::Vector3f a2_pos2 = agent2[i + 1](Eigen::seq(0, 3));

        getLineEq(a2_pos1, a2_pos2, line2);

        auto ans = findSolution(a1_pos1, a2_pos1, line1, line2);
        Eigen::Vector3f points1 = a1_pos1 + ans.first * line1;
        Eigen::Vector3f points2 = a2_pos1 + ans.second * line2;

        float shortest_dist_sq = powf(points2[0] - points1[0], 2) + powf(points2[1] - points1[1], 2) + powf(points2[2] - points1[2], 2);

        if (shortest_dist_sq <= colRadiusSq)
        {
            printf("before\n");
            collisionList.push_back({a1, a2, i, std::make_pair(points1, points2)});
            printf("after\n");
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