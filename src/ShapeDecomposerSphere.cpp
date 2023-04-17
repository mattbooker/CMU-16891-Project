#include <math.h>

#include "ShapeDecomposerSphere.hpp"

ShapeDecomposerSphere::ShapeDecomposerSphere(int num_of_agents, BoundingBox env_bbox, float sphereRadius)
: num_of_agents_(num_of_agents), env_bbox_(env_bbox), sphereRadius_(sphereRadius)
{

}

std::vector<Point3> ShapeDecomposerSphere::decomposeShapeSphere()
{
    // Generate a ring in the Z plane with num_of_agents evenly spaced points
    float deltaR = 2*sphereRadius_ / num_of_agents_;
    float deltaPsi = 2*M_PI / num_of_agents_;
    std::vector<float> z;
    std::vector<float> vecPsi;
    float startR = -sphereRadius_;
    float startPsi = 0;

    std::vector<Point3> result;
    result.reserve(num_of_agents_);

    for (int i = 0; i< num_of_agents_; i++)
    {
        z.push_back(startR);
        vecPsi.push_back(startPsi);

        float x = powf(powf(sphereRadius_, 2) - powf(z[i], 2), 0.5) * cos(vecPsi[i]);
        float y = powf(powf(sphereRadius_, 2) - powf(z[i], 2), 0.5) * sin(vecPsi[i]);
        Point3 endpoint = {x, y, z[i]};
        result.push_back(endpoint);

        startR += deltaR;
        startPsi += deltaPsi;
    }

    return result;
}
