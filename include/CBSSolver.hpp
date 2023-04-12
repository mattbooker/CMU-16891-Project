#ifndef CBS_SOLVER_H
#define CBS_SOLVER_H

#include <memory>
#include <vector>
#include "utils.hpp"
#include "Quadcopter.hpp"

class CBSSolver
{
public:
    CBSSolver();

    std::vector<QuadTrajectory> solve(const MAPFInstance& instance);

private:
    int inline computeCost(const std::vector<QuadTrajectory> &paths);
    void detectCollisions(const std::vector<QuadTrajectory> &paths, std::vector<Collision> &collisionList);
    inline bool detectCollision(int agent1, int agent2, const QuadTrajectory &pathA, const QuadTrajectory &pathB, Collision &col);
    // inline Point3 getLocation(const QuadTrajectory &path, int t);
    inline std::vector<Constraint> resolveCollision(const Collision &col);

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
            return (char*)"No Solution exists for the given MAPF instance";
        }
    };

    int numNodesGenerated;
};

#endif