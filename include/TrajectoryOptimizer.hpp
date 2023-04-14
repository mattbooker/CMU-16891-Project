#ifndef TRAJECTORY_OPTIMIZER_H
#define TRAJECTORY_OPTIMIZER_H

#include "casadi/casadi.hpp"
#include "Quadcopter.hpp"
#include "DoubleIntegrator.hpp"
#include "utils.hpp"

#include <vector>

using namespace casadi;

struct Params
{
    Point3 start;
    Point3 goal;
    float tf;
    float dt;
    int N;
    float colRadiusSq;
};

struct iLQROptions
{
    int maxIters;
    float aTol;

    // Forward-pass params
    int maxLineSearchIters;
};

typedef Eigen::Matrix<float, Quadcopter::nx, Quadcopter::nx> nxByNxMatrix;
typedef Eigen::Matrix<float, Quadcopter::nu, Quadcopter::nu> nuByNuMatrix;
typedef Eigen::Matrix<float, Quadcopter::nx, Quadcopter::nu> nxByNuMatrix;
typedef Eigen::Matrix<float, Quadcopter::nu, Quadcopter::nx> nuByNxMatrix;

class TrajectoryOptimizer
{
public:
    TrajectoryOptimizer()
        : opti(Opti())
    {
        iLQROpt = {250, 1e-3, 20};

        Dict options;
        options["ipopt.print_level"] = 0;
        options["print_time"] = false;
        options["verbose"] = false;
        options["ipopt.sb"] = "yes"; // Undocumented option that suppresses the IPOPT header from printing
        opti.solver("ipopt", options);
    }

    void solveDoubleIntegrator(const Params &params, const std::vector<Constraint> &constraints, DM &xSolution, DM &uSolution);
    bool solveQuadcopter(const Params &params, const std::vector<Constraint> &constraints, QuadTrajectory &xOut, QuadControls &uOut);

private:
    void createReference(const Params &params,
                         const DM &xDoubleIntegrator,
                         QuadTrajectory &xRef,
                         QuadControls &uRef);

    bool runILQR(const Params &params,
                 QuadTrajectory &x,
                 QuadControls &u,
                 const QuadTrajectory &xRef,
                 const QuadControls &uRef);

    float backwardPassIlQR(const QuadTrajectory &x,
                           const QuadControls &u,
                           const QuadTrajectory &xRef,
                           const QuadControls &uRef,
                           const nxByNxMatrix &Q,
                           const nxByNxMatrix &Qf,
                           const nuByNuMatrix &R,
                           std::vector<QuadControlsVector> &d,
                           std::vector<nuByNxMatrix> &K);

    void forwardPassILQR(QuadTrajectory &x,
                         QuadControls &u,
                         const QuadTrajectory &xRef,
                         const QuadControls &uRef,
                         const nxByNxMatrix &Q,
                         const nxByNxMatrix &Qf,
                         const nuByNuMatrix &R,
                         std::vector<QuadControlsVector> &d,
                         std::vector<nuByNxMatrix> &K,
                         float deltaJ);

    float trajectoryCost(const QuadTrajectory &x,
                         const QuadControls &u,
                         const QuadTrajectory &xRef,
                         const QuadControls &uRef,
                         const nxByNxMatrix &Q,
                         const nxByNxMatrix &Qf,
                         const nuByNuMatrix &R);

    DoubleIntegrator doubleIntegrator;
    Quadcopter quadcopter;

    iLQROptions iLQROpt;

    Opti opti;
};

#endif