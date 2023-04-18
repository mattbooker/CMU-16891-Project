#include <math.h>

#include "ShapeDecomposerSphere.hpp"

ShapeDecomposerSphere::ShapeDecomposerSphere(int num_of_agents, BoundingBox env_bbox, float sphereRadius)
: num_of_agents_(num_of_agents), env_bbox_(env_bbox), sphereRadius_(sphereRadius)
{

}

std::vector<Point3> ShapeDecomposerSphere::decomposeShapeSphere()
{
    // Generate a ring in the Z plane with num_of_agents evenly spaced points
    
    // float deltaR = 2*sphereRadius_ / num_of_agents_;
    // float deltaPsi = 2*M_PI / num_of_agents_;
    // std::vector<float> z;
    // std::vector<float> vecPsi;
    // float startR = -sphereRadius_;
    // float startPsi = 0;

    // std::vector<Point3> result;
    // result.reserve(num_of_agents_);

    // for (int i = 0; i< num_of_agents_; i++)
    // {
    //     z.push_back(startR);
    //     vecPsi.push_back(startPsi);

    //     float x = powf(powf(sphereRadius_, 2) - powf(z[i], 2), 0.5) * cos(vecPsi[i]);
    //     float y = powf(powf(sphereRadius_, 2) - powf(z[i], 2), 0.5) * sin(vecPsi[i]);
    //     Point3 endpoint = {x, y, z[i]};
    //     result.push_back(endpoint);

    //     startR += deltaR;
    //     startPsi += deltaPsi;
    // }

    // return result;


    // float a = (4 * M_PI * powf(sphereRadius_, 2)) / num_of_agents_;
    // printf("a: %f \n", a);
    // float d = powf(a, 0.5);
    // printf("d: %f \n", d);
    // int Mv = round(M_PI / d);
    // printf("Mv: %d \n", Mv);
    // float d_v = M_PI / Mv;
    // printf("dv: %f \n", d_v);
    // float d_psi = a / d_v;
    // printf("dpsi: %f \n", d_psi);


    // for (int i = 0; i < Mv; i++)
    // {
    //     float v = M_PI * (i + 0.5) / Mv;
    //     int Mpsi = round((2*M_PI*sin(v)) / d_psi);
    //     for (int j = 0; j < Mpsi; j++)
    //     {
    //         float psi = (2*M_PI*j) / Mpsi;
    //         Point3 endpoint = {sphereRadius_*sin(v)*cos(psi), sphereRadius_*sin(v)*sin(psi), sphereRadius_*cos(v)};
    //         result.push_back(endpoint);
    //     }
    // }  
    // return result;

    result.reserve(num_of_agents_);

    int N = num_of_agents_;
    float collisionRadius = 0.5;
    float radius = sphereRadius_;
    printf("Radius: %f\n", radius);
    float deltaX = 1.0;
    float x = 0.0;

    while(N > 0)
    {
        int numOfAgentsInCircle = pow(floor(sqrt(M_PI / asin(collisionRadius / radius))), 2);
        printf("Num of agents in circle: %d\n", numOfAgentsInCircle);
        printf("Num of agents left: %d\n", N);
        if (N == num_of_agents_)
        {
            decomposeShape(radius, x, numOfAgentsInCircle);
            N -= numOfAgentsInCircle;
            x += deltaX;
            radius = sqrt(powf(sphereRadius_, 2) - powf(x, 2));
            printf("Radius: %f\n", radius);
        }
        else if (N < num_of_agents_)
        {
            printf("X = %f\n", x);
            if(x >= sphereRadius_)
            {
                printf("Could only fit %d agents in sphere! \n", (num_of_agents_ - N));
                break;
            }
            if(N > 2*numOfAgentsInCircle)
            {
                decomposeShape(radius, x, numOfAgentsInCircle);
                decomposeShape(radius, -x, numOfAgentsInCircle);
                N -= 2*numOfAgentsInCircle;
                x += deltaX;
                radius = sqrt(powf(sphereRadius_, 2) - powf(x, 2));
                printf("Radius: %f\n", radius);
            }
            else if(N > numOfAgentsInCircle)
            {
                decomposeShape(radius, x, numOfAgentsInCircle);
                N -= numOfAgentsInCircle;
                decomposeShape(radius, -x, N);
                N = 0;
            }
            else
            {
                decomposeShape(radius, x, N);
                N = 0;
            }
        }
    }
    printf("Result length = %d\n", result.size());
    return result;
}

void ShapeDecomposerSphere::decomposeShape(float radius, float x, int N)
{
    // Generate a ring in the Z plane with num_of_agents evenly spaced points
    
    // float x = (env_bbox_.min.x + env_bbox_.max.x) / 2;

    // float y_scale = (env_bbox_.max.y - env_bbox_.min.y) / 2;
    float y_scale = radius;
    float y_offset = (env_bbox_.min.y + env_bbox_.max.y) / 2;

    // float z_scale = (env_bbox_.max.z - env_bbox_.min.z) / 2;
    float z_scale = radius;
    float z_offset = (env_bbox_.min.z + env_bbox_.max.z) / 2;

    float angular_seperation = 2 * M_PI / N;

    for (int i = 0; i < N; i++)
    {
        Point3 endpoint = {x, y_offset + y_scale * cos(i * angular_seperation), z_offset + z_scale * sin(i * angular_seperation)};
        result.push_back(endpoint);
    }
}


