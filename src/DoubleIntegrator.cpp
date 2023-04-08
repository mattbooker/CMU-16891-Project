#include "DoubleIntegrator.hpp"

MX DoubleIntegrator::dynamics(const MX &x, const MX &u)
{
    MX qd = x(Slice(3, 6));

    return vertcat(qd, u);
}

MX DoubleIntegrator::rk4(const MX &x, const MX &u, float dt)
{
    MX k1 = dt * dynamics(x, u);
    MX k2 = dt * dynamics(x + k1 / 2.0, u);
    MX k3 = dt * dynamics(x + k2 / 2.0, u);
    MX k4 = dt * dynamics(x + k3, u);
    
    return x + (1.0 / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
}

MX DoubleIntegrator::computeCost(const DM &Q, const DM &Qf, const DM &R, const MX &x, const MX &u, int N, float dt)
{
    // Terminal cost
    MX cost = 0.5 * mtimes(x(Slice(), N - 1).T(), mtimes(Qf, x(Slice(), N - 1)));

    for (int i = 0; i < N - 1; i++)
    {
        cost += 0.5 * mtimes(x(Slice(), i).T(), mtimes(Q, x(Slice(), i))) + 0.5 * mtimes(u(Slice(), i).T(), mtimes(R, u(Slice(), i)));
    }

    return cost;
}