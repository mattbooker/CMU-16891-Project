#ifndef QUADCOPTER_H
#define QUADCOPTER_H

#include <casadi/casadi.hpp>

using namespace casadi;

struct QuadcopterModel
{
    float mass;
    DM J;
    DM gravity;
    float L;
    float kf;
    float km;
    float dt;
};

class Quadcopter
{
public:
    MX dynamics(const QuadcopterModel& model, const MX& x, const MX& u);
    MX rk4(const QuadcopterModel& model, const MX& x, const MX& u);

private:
    inline MX dcmFromMrp(const MX &vec)
    {
        MX p1 = vec(0);
        MX p2 = vec(1);
        MX p3 = vec(2);

        MX den = pow(pow(p1, 2) + pow(p2, 2) + pow(p3, 2) + 1, 2);
        MX a = (4 * pow(p1, 2) + 4 * pow(p2, 2) + 4 * pow(p3, 2) - 4);

        MX result = MX::zeros(3,3);

        result(0, 0) = -((8 * pow(p2, 2) + 8 * pow(p3, 2)) / den - 1) * den;
        result(0, 1) = 8 * p1 * p2 + p3 * a;
        result(0, 2) = 8 * p1 * p3 - p2 * a;
        result(1, 0) = 8 * p1 * p2 - p3 * a;
        result(1, 1) = -((8 * pow(p1, 2) + 8 * pow(p3, 2)) / den - 1) * den;
        result(1, 2) = 8 * p2 * p3 + p1 * a;
        result(2, 0) = 8 * p1 * p3 + p2 * a;
        result(2, 1) = 8 * p2 * p3 - p1 * a;
        result(2, 2) = -((8 * pow(p1, 2) + 8 * pow(p2, 2)) / den - 1) * den;

        return result / den;
    }
};

#endif