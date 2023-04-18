#ifndef SHAPE_DECOMPOSER_SPHERE_H
#define SHAPE_DECOMPOSER_SPHERE_H

#include <vector>
#include "utils.hpp"

class ShapeDecomposerSphere
{
public:
    ShapeDecomposerSphere(int num_of_agents, BoundingBox env_bbox, float sphereRadius);

    // This is what it should look like
    // std::vector<Point3> decomposeShape(Shape input_shape);

    // TODO: remove and replace with actual decomposition
    std::vector<Point3> decomposeShapeSphere();
    void decomposeShape(float radius, float x, int N);
    std::vector<Point3> result;

private:
    int num_of_agents_;
    BoundingBox env_bbox_;
    float sphereRadius_;
};

#endif