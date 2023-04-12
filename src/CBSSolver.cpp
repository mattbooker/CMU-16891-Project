#include "CBSSolver.hpp"
#include "TrajectoryOptimizer.hpp"
#include <queue>

CBSSolver::CBSSolver()
: numNodesGenerated(0)
{
}

std::vector<QuadTrajectory> CBSSolver::solve(const MAPFInstance& instance)
{
    printf("Starting CBS Solver\n");

    // Initialize low level solver
    TrajectoryOptimizer lowLevelSolver;
    Params params;
    params.tf = 3.0;
    params.dt = 0.1;
    params.N = int(params.tf / params.dt);
    params.colRadiusSq = pow(0.8 + 0.2, 2.0);

    // Create priority queue
    std::priority_queue <CTNodeSharedPtr, 
                         std::vector<CTNodeSharedPtr>, 
                         CTNodeComparator 
                        > pq;

    CTNodeSharedPtr root = std::make_shared<CTNode>();
    root->paths.resize(instance.numAgents);
    root->id = numNodesGenerated++;

    // Create paths for all agents
    for (int i = 0; i < instance.startLocs.size(); i++)
    {
        params.start = instance.startLocs[i];
        params.goal = instance.startLocs[i];
        QuadControls unused;
        bool found = lowLevelSolver.solveQuadcopter(params, root->constraintList, root->paths[i], unused);
        
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
            return cur->paths;
        }

        printf("%f\n", cur->cost);

        // Get first collision and create two nodes (each containing a new plan for the two agents in the collision)
        for (Constraint &c : resolveCollision(cur->collisionList[0]))
        {
            // Add new constraint
            CTNodeSharedPtr child = std::make_shared<CTNode>();
            child->constraintList = cur->constraintList;
            child->constraintList.push_back(c);
            child->paths = cur->paths;
            // printf("New cons = %d (%d,%d) %d @ %d\n", c.agentNum, c.location.first.x, c.location.first.y, c.isVertexConstraint, c.t);

            // Replan only for the agent that has the new constraint
            // printf("Before = %d %d\n", child->paths[c.agentNum].size(), computeCost(child->paths));
            child->paths[c.agentNum].clear();
            params.start = instance.startLocs[c.agentNum];
            params.goal = instance.startLocs[c.agentNum];
            QuadControls unused;
            bool success = lowLevelSolver.solveQuadcopter(params, child->constraintList, child->paths[c.agentNum], unused);
            // printf("After = %d %d\n", child->paths[c.agentNum].size(), computeCost(child->paths));

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

    // N^2 alg - seems like there should be a better way to detect collisions
    for (int i = 0; i < paths.size() - 1; i++)
    {
        for (int j = i + 1; j < paths.size(); j++)
        {
            if (detectCollision(i, j, paths[i], paths[j], col))
                collisionList.push_back(col);
        }
    }
}

inline bool CBSSolver::detectCollision(int agent1, int agent2, const QuadTrajectory &pathA, const QuadTrajectory &pathB, Collision &col)
{
    // TODO
    // int maxTime = std::max(pathA.size(), pathB.size());

    // for (int t = 0; t < maxTime; t++)
    // {
    //     if (getLocation(pathA, t) == getLocation(pathB, t))
    //     {
    //         col = createVertexCollision(agent1, agent2, t, getLocation(pathA, t));
    //         return true;
    //     }

    //     if (getLocation(pathA, t) == getLocation(pathB, t + 1) && getLocation(pathA, t + 1) == getLocation(pathB, t))
    //     {
    //         col = createEdgeCollision(agent1, agent2, t + 1, getLocation(pathA, t), getLocation(pathA, t + 1));
    //         return true;
    //     }
    // }

    return false;
}

// inline Point3 CBSSolver::getLocation(const QuadTrajectory &path, int t)
// {
//     if (t >= path.size())
//         return path[path.size() - 1];
//     else
//         return path[t];
// }

inline std::vector<Constraint> CBSSolver::resolveCollision(const Collision& col)
{
    return std::vector<Constraint> {Constraint{col.agent1, col.t, col.location.second}, Constraint{col.agent2, col.t, col.location.first}};
}