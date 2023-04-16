#include "TrajectoryOptimizer.hpp"

#include <eigen3/Eigen/Dense>
#include <eigen3/unsupported/Eigen/AutoDiff>

#include <exception>

bool TrajectoryOptimizer::solveDoubleIntegrator(const Params &params, const std::vector<Constraint> &constraints, DM &xSolution, DM &uSolution)
{
    int N = params.N;
    float dt = params.dt;

    DM start = DM::zeros(DoubleIntegrator::nx, 1);
    DM goal = DM::zeros(DoubleIntegrator::nx, 1);

    start(0) = params.start.x;
    start(1) = params.start.y;
    start(2) = params.start.z;

    goal(0) = params.goal.x;
    goal(1) = params.goal.y;
    goal(2) = params.goal.z;

    DM Q = DM::eye(DoubleIntegrator::nx);
    DM Qf = 10 * DM::eye(DoubleIntegrator::nx);
    DM R = 0.1 * DM::eye(DoubleIntegrator::nu);

    MX x = opti.variable(DoubleIntegrator::nx, N);
    MX u = opti.variable(DoubleIntegrator::nu, N - 1);

    printf("DI Start = (%f, %f, %f)\n", params.start.x, params.start.y, params.start.z);
    printf("DI Goal = (%f, %f, %f)\n", params.goal.x, params.goal.y, params.goal.z);

    opti.minimize(doubleIntegrator.computeCost(Q, Qf, R, x, u, goal, N, dt));

    // Add start and goal constraints
    opti.subject_to(x(Slice(), 0) == start);
    opti.subject_to(x(Slice(), N - 1) == goal);

    // Add controls constraint (limit acceleration)
    opti.subject_to(-quadcopter.model.accelLimit <= u <= quadcopter.model.accelLimit);

    // Add dynamics constraint
    for (int i = 0; i < N - 1; i++)
    {
        opti.subject_to(x(Slice(), i + 1) - doubleIntegrator.rk4(x(Slice(), i), u(Slice(), i), dt) == 0);
    }

    // Add sphere collision avoidance constraint
    for (const Constraint &c : constraints)
    {
        // TODO: Constraint shouldnt be for entire trajectory
        // int min_time = std::max(0, c.t - 10);
        // int max_time = std::min(params.N, c.t + 10);
        // for (int t = min_time; t <= max_time; t++)
        // {
        //     DM location = {c.location[0], c.location[1], c.location[2]};
        //     opti.subject_to(params.colRadiusSq <= sumsqr(x(Slice(0, 3), t) - location));
        // }

        for (int t = 0; t < N; t++)
        {
            DM location = {c.location[0], c.location[1], c.location[2]};
            opti.subject_to(params.colRadiusSq * 1.2 <= sumsqr(x(Slice(0, 3), t) - location));
        }

        std::cout << "Cons = " << c.location.transpose() << " @ " << c.t << std::endl;
    }

    DM initX = 0.01 * DM::rand(DoubleIntegrator::nx, N);
    initX(Slice(), 0) = start;
    initX(Slice(), N - 1) = goal;
    float stepX = (params.goal.x - params.start.x) / N;
    float stepY = (params.goal.y - params.start.y) / N;
    float stepZ = (params.goal.z - params.start.z) / N;

    for (int i = 1; i < N - 1; i++)
    {
        initX(0, i) = initX(0, i - 1) + stepX;
        initX(1, i) = initX(1, i - 1) + stepY;
        initX(2, i) = initX(2, i - 1) + stepZ;
    }

    // opti.set_initial(x, 0.01 * DM::rand(DoubleIntegrator::nx, N));
    opti.set_initial(x, initX);
    opti.set_initial(u, 0.01 * DM::rand(DoubleIntegrator::nu, N - 1));

    auto startTime = std::chrono::high_resolution_clock::now();

    try {

        OptiSol sol = opti.solve();
        xSolution = sol.value(x);
        uSolution = sol.value(u);
    }
    catch (CasadiException e)
    {
        std::cout << "DI Solver failed: " << e.what() << std::endl;
        return false;
    }
    auto stopTime = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stopTime - startTime);

    if(verbose)
        std::cout << duration.count() * 1e-6 << std::endl;

    return true;
}

bool TrajectoryOptimizer::solveQuadcopter(const Params &params, const std::vector<Constraint> &constraints, QuadTrajectory &xOut)
{
    // Solve for a reference trajectory (using double integrator)
    DM xDoubleIntegrator, uDoubleIntegrator;

    bool result = solveDoubleIntegrator(params, constraints, xDoubleIntegrator, uDoubleIntegrator);

    if (!result)
        return false;

    QuadTrajectory xRef(params.N, QuadStateVector::Zero());
    QuadControls uRef(params.N, QuadControlsVector::Ones());

    createReference(params, xDoubleIntegrator, xRef, uRef);

    QuadControls uOut;

    // Use iLQR to refine trajectory to be flyable by quadrotor
    return runILQR(params, xOut, uOut, xRef, uRef);
}

void TrajectoryOptimizer::createReference(const Params &params, const DM &xDoubleIntegrator, QuadTrajectory &xRef, QuadControls &uRef)
{
    std::vector<double> elements = xDoubleIntegrator.get_elements();
    for (int i = 0; i < params.N; i++)
    {
        // Copy positions and velocities
        xRef[i](0) = elements[6 * i];
        xRef[i](1) = elements[6 * i + 1];
        xRef[i](2) = elements[6 * i + 2];
        xRef[i](3) = elements[6 * i + 3];
        xRef[i](4) = elements[6 * i + 4];
        xRef[i](5) = elements[6 * i + 5];

        // Set all controls to hover
        uRef[i] *= (9.81 * quadcopter.model.mass / 4.0);
    }
}

bool TrajectoryOptimizer::runILQR(const Params &params, QuadTrajectory &x, QuadControls &u, const QuadTrajectory &xRef, const QuadControls &uRef)
{
    const int nx = Quadcopter::nx;
    const int nu = Quadcopter::nu;
    int N = params.N;

    // Setup cost matrices
    nxByNxMatrix Q = nxByNxMatrix::Identity();
    Q(3, 3) = 0.1;
    Q(4, 4) = 0.1;
    Q(5, 5) = 0.1;
    Q(9, 9) = 0.1;
    Q(10, 10) = 0.1;
    Q(11, 11) = 0.1;
    nxByNxMatrix Qf = 10 * nxByNxMatrix::Identity();
    nuByNuMatrix R = 0.1 * nuByNuMatrix::Identity();

    // Initial rollout
    x = QuadTrajectory(N, QuadStateVector::Zero());
    x[0] = xRef[0];

    for (int i = 0; i < params.N - 1; i++)
    {
        x[i + 1] = quadcopter.rk4(x[i], uRef[i]);
    }

    // Run iLQR
    float deltaJ = 0;
    u = uRef;
    std::vector<QuadControlsVector> d(N - 1, QuadControlsVector::Zero());
    std::vector<nuByNxMatrix> K(N - 1, nuByNxMatrix::Zero());

    std::cout << "Start = " << xRef[0](Eigen::seq(0, 2)).transpose() << std::endl;
    std::cout << "Goal = " << xRef[N - 1](Eigen::seq(0, 2)).transpose() << std::endl;

    for (int i = 0; i < iLQROpt.maxIters; i++)
    {
        deltaJ = backwardPassIlQR(x, u, xRef, uRef, Q, Qf, R, d, K);
        forwardPassILQR(x, u, xRef, uRef, Q, Qf, R, d, K, deltaJ);


        if (deltaJ < iLQROpt.aTol)
        {
            if (verbose)
                printf("iLQR Converged\n");
            return true;
        }
    }

    if (verbose)
        printf("iLQR Failed\n");
    return false;
}

float TrajectoryOptimizer::backwardPassIlQR(const QuadTrajectory &x,
                                            const QuadControls &u,
                                            const QuadTrajectory &xRef,
                                            const QuadControls &uRef,
                                            const nxByNxMatrix &Q,
                                            const nxByNxMatrix &Qf,
                                            const nuByNuMatrix &R,
                                            std::vector<QuadControlsVector> &d,
                                            std::vector<nuByNxMatrix> &K)
{
    float deltaJ = 0;
    const int nx = Quadcopter::nx;
    const int nu = Quadcopter::nu;
    int N = x.size();

    std::vector<nxByNxMatrix> P(N, nxByNxMatrix::Zero());
    std::vector<QuadStateVector> p(N, QuadStateVector::Zero());

    P.resize(N);
    p.resize(N);

    P[N - 1] = Qf;
    p[N - 1] = Qf * (x.back() - xRef.back());

    DM xCas = DM::zeros(nx, 1);
    DM uCas = DM::zeros(nu, 1);

    nxByNxMatrix A, A_T;
    nxByNuMatrix B;
    nuByNxMatrix B_T;
    std::vector<double> tempA, tempB;
    std::vector<float> Avals, Bvals;

    QuadStateVector gradJ_x, g_x;
    QuadControlsVector gradJ_u, g_u;

    nxByNxMatrix G_xx;
    nuByNuMatrix G_uu;
    nxByNuMatrix G_xu;
    nuByNxMatrix G_ux;
    nuByNuMatrix G_uu_inv;
    nxByNuMatrix K_T;

    for (int k = N - 2; k >= 0; k--)
    {
        // Fill out xCas
        for (int i = 0; i < nx; i++)
        {
            xCas(i) = x[k](i);
        }

        // Fill out uCas
        for (int i = 0; i < nu; i++)
        {
            uCas(i) = u[k](i);
        }

        std::vector<DM> ACas = quadcopter.jacA(std::vector<DM>{xCas, uCas});
        std::vector<DM> BCas = quadcopter.jacB(std::vector<DM>{xCas, uCas});

        // Hacky way to convert to Eigen
        tempA = ACas[0].get_elements();
        tempB = BCas[0].get_elements();
        Avals = std::vector<float>(tempA.begin(), tempA.end());
        Bvals = std::vector<float>(tempB.begin(), tempB.end());
        A = nxByNxMatrix(Avals.data());
        B = nxByNuMatrix(Bvals.data());
        A_T = A.transpose();
        B_T = B.transpose();

        // Compute values for iLQR
        gradJ_x = Q * (x[k] - xRef[k]);
        gradJ_u = R * (u[k] - uRef[k]);

        g_x = gradJ_x + A_T * p[k + 1];
        g_u = gradJ_u + B_T * p[k + 1];

        G_xx = Q + A_T * P[k + 1] * A;
        G_uu = R + B_T * P[k + 1] * B;
        G_xu = A_T * P[k + 1] * B;
        G_ux = B_T * P[k + 1] * A;

        G_uu_inv = G_uu.inverse();
        d[k] = G_uu_inv * g_u;
        K[k] = G_uu_inv * G_ux;
        K_T = K[k].transpose();
        P[k] = G_xx + K_T * G_uu * K[k] - G_xu * K[k] - K_T * G_ux;
        p[k] = g_x - K_T * g_u + K_T * G_uu * d[k] - G_xu * d[k];

        deltaJ += g_u.transpose() * d[k];
    }

    return deltaJ;
}

void TrajectoryOptimizer::forwardPassILQR(QuadTrajectory &x,
                                          QuadControls &u,
                                          const QuadTrajectory &xRef,
                                          const QuadControls &uRef,
                                          const nxByNxMatrix &Q,
                                          const nxByNxMatrix &Qf,
                                          const nuByNuMatrix &R,
                                          std::vector<QuadControlsVector> &d,
                                          std::vector<nuByNxMatrix> &K,
                                          float deltaJ)
{
    float alpha = 1.0;
    float originalCost = trajectoryCost(x, u, xRef, uRef, Q, Qf, R);
    float updatedCost = 0;
    float beta = 0.001;

    QuadTrajectory newX(x.size(), x[0]);
    QuadControls newU(x.size(), u[0]);

    for (int i = 0; i < iLQROpt.maxLineSearchIters; i++)
    {
        // Compute rollout
        for (int k = 0; k < x.size() - 1; k++)
        {
            newU[k] = u[k] - alpha * d[k] - K[k] * (newX[k] - x[k]);
            newX[k + 1] = quadcopter.rk4(newX[k], newU[k]);
        }

        alpha /= 2.0;
        updatedCost = trajectoryCost(newX, newU, xRef, uRef, Q, Qf, R);

        if (updatedCost < originalCost - beta * deltaJ)
        {
            // Copy over the new trajectory + controls
            x = newX;
            u = newU;
            return;
        }
    }

    throw std::logic_error("Forward pass line search failed");
}

float TrajectoryOptimizer::trajectoryCost(const QuadTrajectory &x,
                                          const QuadControls &u,
                                          const QuadTrajectory &xRef,
                                          const QuadControls &uRef,
                                          const nxByNxMatrix &Q,
                                          const nxByNxMatrix &Qf,
                                          const nuByNuMatrix &R)
{
    float totalCost = 0.5 * (x.back() - xRef.back()).transpose() * Qf * (x.back() - xRef.back());

    for (int i = 0; i < x.size() - 1; i++)
    {
        totalCost += 0.5 * (x[i] - xRef[i]).transpose() * Q * (x[i] - xRef[i]);
        totalCost += 0.5 * (u[i] - uRef[i]).transpose() * R * (u[i] - uRef[i]);
    }

    return totalCost;
}