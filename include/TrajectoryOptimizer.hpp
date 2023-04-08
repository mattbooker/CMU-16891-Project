#ifndef TRAJECTORY_OPTIMIZER_H
#define TRAJECTORY_OPTIMIZER_H

#include "casadi/casadi.hpp"
#include "Quadcopter.hpp"
#include "DoubleIntegrator.hpp"

using namespace casadi;

class TrajectoryOptimizer
{
public:
    TrajectoryOptimizer()
        : accelLimit(2.0),
          opti(Opti())
    {
        DM J = DM({{0.0023, 0.0, 0.0}, {0.0, 0.0023, 0.0}, {0.0, 0.0, 0.004}});
        DM gravity = DM({0.0, 0.0, -9.81});
        quadModel = {0.5, J, gravity, 0.1750, 1.0, 0.0245, 0.1};

        Dict options;
        options["ipopt.print_level"] = 0;
        options["print_time"] = false;
        options["verbose"] = false;
        options["ipopt.sb"] = "yes"; // Undocumented option that suppresses the IPOPT header from printing
        opti.solver("ipopt", options);
    }

    void solveDoubleIntegrator(const DM &start, const DM &goal);

private:
    DoubleIntegrator doubleIntegrator;
    Quadcopter quadcopter;
    QuadcopterModel quadModel;

    float accelLimit;
    Opti opti;
};

#endif