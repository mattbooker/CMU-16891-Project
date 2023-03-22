#include <math.h>

#include "ShapeDecomposer.hpp"

ShapeDecomposer::ShapeDecomposer(int num_of_agents, BoundingBox env_bbox)
: num_of_agents_(num_of_agents), env_bbox_(env_bbox)
{

}

std::vector<Point3> ShapeDecomposer::decomposeShape()
{
    // Generate a ring in the Z plane with num_of_agents evenly spaced points
    
    float x = (env_bbox_.min.x + env_bbox_.max.x) / 2;

    float y_scale = (env_bbox_.max.y - env_bbox_.min.y) / 2;
    float y_offset = (env_bbox_.min.y + env_bbox_.max.y) / 2;

    float z_scale = (env_bbox_.max.z - env_bbox_.min.z) / 2;
    float z_offset = (env_bbox_.min.z + env_bbox_.max.z) / 2;

    float angular_seperation = 2 * M_PI / num_of_agents_;

    std::vector<Point3> result;
    result.reserve(num_of_agents_);

    for (int i = 0; i < num_of_agents_; i++)
    {
        Point3 endpoint = {x, y_offset + y_scale * cos(i * angular_seperation), z_offset + z_scale * sin(i * angular_seperation)};
        result.push_back(endpoint);
    }

    return result;
}