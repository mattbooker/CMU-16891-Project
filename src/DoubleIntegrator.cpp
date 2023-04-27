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


MX DoubleIntegrator::hermiteSimpson(const MX &x1, const MX &x2, const MX &u, float dt)
{
    MX x1Dot = dynamics(x1, u);
    MX x2Dot = dynamics(x2, u);

    MX xkHalf = (x1 + x2) / 2 + dt * (x1Dot - x2Dot) / 8;
    return x1 + dt * (x1Dot + 4 * dynamics(xkHalf, u) + x2Dot) / 6 - x2;
}

MX DoubleIntegrator::computeCost(const DM &Q, const DM &Qf, const DM &R, const MX &x, const MX &u, const DM &xGoal, int N, float dt)
{
    // Terminal cost
    MX cost = 0.5 * mtimes((x(Slice(), N - 1) - xGoal).T(), mtimes(Qf, (x(Slice(), N - 1) - xGoal)));

    for (int i = 0; i < N - 1; i++)
    {
        cost += 0.5 * mtimes((x(Slice(), i) - xGoal).T(), mtimes(Q, (x(Slice(), i) - xGoal))) + 0.5 * mtimes(u(Slice(), i).T(), mtimes(R, u(Slice(), i)));
    }

    return cost;
}