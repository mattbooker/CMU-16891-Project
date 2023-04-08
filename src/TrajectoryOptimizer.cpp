#include "TrajectoryOptimizer.hpp"
#include <chrono>

void TrajectoryOptimizer::solveDoubleIntegrator(const DM& start, const DM& goal)
{
    int N = 51;
    float dt = 0.1;

    DM Q = DM::eye(doubleIntegrator.nx);
    DM Qf = 10*DM::eye(doubleIntegrator.nx);
    DM R = 0.1*DM::eye(doubleIntegrator.nu);

    MX x = opti.variable(doubleIntegrator.nx, N);
    MX u = opti.variable(doubleIntegrator.nu, N - 1);

    opti.minimize(doubleIntegrator.computeCost(Q, Qf, R, x, u, N, dt));

    // Add start and goal constraints
    opti.subject_to(x(Slice(), 0) == start);
    opti.subject_to(x(Slice(), N-1) == goal);

    // Add controls constraint (limit acceleration)
    opti.subject_to(-accelLimit <= u <= accelLimit);

    // Add dynamics constraint
    for (int i = 0; i < N - 1; i++)
    {
        opti.subject_to(x(Slice(), i + 1) - doubleIntegrator.rk4(x(Slice(), i), u(Slice(), i), dt) == 0);
    }

    // TODO: Add sphere collision avoidance constraint
    // for (int i = 0; i < constraints.size(); i++)
    // {
    //     opti.subject_to(pow(quadRadius + inflationRadius, 2) <= x);
    // }

    opti.set_initial(x, 0.001 * DM::rand(doubleIntegrator.nx, N));
    opti.set_initial(u, 0.001 * DM::rand(doubleIntegrator.nu, N - 1));

    auto startTime = std::chrono::high_resolution_clock::now();
    OptiSol sol = opti.solve();
    auto stopTime = std::chrono::high_resolution_clock::now();
    
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stopTime - startTime);;
    std::cout << duration.count() * 1e-6 << std::endl;

    DM xSolution = sol.value(x);
    // sol.value(y)
}