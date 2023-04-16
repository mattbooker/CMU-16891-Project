#ifndef CBS_SOLVER_H
#define CBS_SOLVER_H

#include <memory>
#include <vector>
#include "utils.hpp"
#include "TrajectoryOptimizer.hpp"
#include "Quadcopter.hpp"
#include <utility>

class CBSSolver
{
public:
    CBSSolver()
        : numNodesGenerated(0), 
        colRadiusSq(0.25)
    {
    }

    std::vector<QuadTrajectory> solve(const MAPFInstance &instance);

private:
    int inline computeCost(const std::vector<QuadTrajectory> &paths);
    void detectCollisions(const std::vector<QuadTrajectory> &paths, std::vector<Collision> &collisionList);
    inline bool detectCollision(int agent1, int agent2, const QuadTrajectory &pathA, const QuadTrajectory &pathB, Collision &col);
    // inline Point3 getLocation(const QuadTrajectory &path, int t);
    inline std::vector<Constraint> resolveCollision(const Collision &col);

    void getCollisionsAgents(QuadTrajectory agent1, QuadTrajectory agent2, int a1, int a2, std::vector<Collision> &collisionList);
    void inline getLineEq(Eigen::Vector3f pos1, Eigen::Vector3f pos2, Eigen::Vector3f &line);
    std::pair<float, float> findSolution(Eigen::Vector3f a1_pos1, Eigen::Vector3f a2_pos1, Eigen::Vector3f line1, Eigen::Vector3f line2);

    struct CTNode
    {
        float cost;
        std::vector<QuadTrajectory> paths;
        std::vector<Collision> collisionList;
        std::vector<Constraint> constraintList;
        int id;
    };

    typedef std::shared_ptr<CTNode> CTNodeSharedPtr;

    class CTNodeComparator
    {
    public:
        bool operator()(const CTNodeSharedPtr &a, const CTNodeSharedPtr &b) const
        {
            if (a->cost == b->cost)
            {
                if (a->collisionList.size() == b->collisionList.size())
                {
                    return a->id > b->id;
                }

                return a->collisionList.size() > b->collisionList.size();
            }

            return a->cost > b->cost;
        }
    };

    class NoSolutionException : public std::exception
    {
        char *what()
        {
            return (char *)"No Solution exists for the given MAPF instance";
        }
    };

    int numNodesGenerated;
    float colRadiusSq;
};

#endif