#ifndef DOUBLE_INTEGRATOR_H
#define DOUBLE_INTEGRATOR_H

#include <casadi/casadi.hpp>

using namespace casadi;

class DoubleIntegrator
{
public:
    static const int nx = 6;
    static const int nu = 3;

    MX dynamics(const MX &x, const MX &u);
    MX rk4(const MX &x, const MX &u, float dt);

    MX computeCost(const DM &Q, const DM &Qf, const DM &R, const MX &x, const MX &u, int N, float dt);
};

#endif