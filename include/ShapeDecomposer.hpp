#ifndef SHAPE_DECOMPOSER_H
#define SHAPE_DECOMPOSER_H

#include <vector>
#include "utils.hpp"

class ShapeDecomposer
{
public:
    ShapeDecomposer(int num_of_agents, BoundingBox env_bbox);

    // This is what it should look like
    // std::vector<Point3> decomposeShape(Shape input_shape);

    // TODO: remove and replace with actual decomposition
    std::vector<Point3> decomposeShape();

private:
    int num_of_agents_;
    BoundingBox env_bbox_;
};

#endif