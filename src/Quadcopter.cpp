#include "Quadcopter.hpp"

MX Quadcopter::dynamics(const MX &x, const MX &u)
{
    MX r = x(Slice(0, 3));
    MX v = x(Slice(3, 6));
    MX p = x(Slice(6, 9));
    MX w = x(Slice(9, 12));

    MX Q = dcmFromMrp(p);

    MX f1 = fmax(0.0, model.kf * u(0));
    MX f2 = fmax(0.0, model.kf * u(1));
    MX f3 = fmax(0.0, model.kf * u(2));
    MX f4 = fmax(0.0, model.kf * u(3));

    MX totalF = MX::zeros(3, 1);
    totalF(2) = f1 + f2 + f3 + f4;

    MX m1 = model.km * u(0);
    MX m2 = model.km * u(1);
    MX m3 = model.km * u(2);
    MX m4 = model.km * u(3);

    MX t = MX::zeros(3, 1);
    t(0) = model.L * (f2 - f4);
    t(1) = model.L * (f3 - f1);
    t(2) = m1 - m2 + m3 - m4;

    MX J = MX::zeros(3, 3);
    J(0, 0) = model.J[0];
    J(1, 1) = model.J[1];
    J(2, 2) = model.J[2];

    MX v_dot = (model.mass * gravity_casadi + mtimes(Q, totalF)) / model.mass;
    MX p_dot = mtimes(((1.0 + sumsqr(p)) / 4.0), mtimes((MX::eye(3) + 2.0 * (mpower(skew(p), 2) + skew(p)) / (1.0 + sumsqr(p))), w));
    MX w_dot = solve(J, (t - cross(w, mtimes(J, w))));

    MX x_dot = vertcat(v,
                       v_dot,
                       p_dot,
                       w_dot);
    return x_dot;
}

MX Quadcopter::rk4(const MX &x, const MX &u)
{
    MX k1 = model.dt * dynamics(x, u);
    MX k2 = model.dt * dynamics(x + k1 / 2.0, u);
    MX k3 = model.dt * dynamics(x + k2 / 2.0, u);
    MX k4 = model.dt * dynamics(x + k3, u);

    return x + (1.0 / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
}

QuadStateVector Quadcopter::dynamics(const QuadStateVector &x, const QuadControlsVector &u) const
{
    Eigen::Vector3f r = x(Eigen::seq(0, 2));
    Eigen::Vector3f v = x(Eigen::seq(3, 5));
    Eigen::Vector3f p = x(Eigen::seq(6, 8));
    Eigen::Vector3f w = x(Eigen::seq(9, 11));

    Eigen::Matrix3f Q = dcmFromMrp(p);

    float f1 = std::max(0.0f, model.kf * u(0));
    float f2 = std::max(0.0f, model.kf * u(1));
    float f3 = std::max(0.0f, model.kf * u(2));
    float f4 = std::max(0.0f, model.kf * u(3));

    Eigen::Vector3f totalF = Eigen::Vector3f::Zero();
    totalF(2) = f1 + f2 + f3 + f4;

    float m1 = model.km * u(0);
    float m2 = model.km * u(1);
    float m3 = model.km * u(2);
    float m4 = model.km * u(3);

    Eigen::Vector3f t = Eigen::Vector3f::Zero();
    t(0) = model.L * (f2 - f4);
    t(1) = model.L * (f3 - f1);
    t(2) = m1 - m2 + m3 - m4;

    Eigen::DiagonalMatrix<float, 3> J(model.J[0], model.J[1], model.J[2]);

    Eigen::Matrix3f pHat;
    pHat << 0.0f, -p(2), p(1),
        p(2), 0.0f, -p(0),
        -p(1), p(0), 0.0f;

    Eigen::Matrix3f wHat;
    wHat << 0.0f, -w(2), w(1),
        w(2), 0.0f, -w(0),
        -w(1), w(0), 0.0f;

    Eigen::Vector3f vDot = (model.mass * gravity_eigen + Q * totalF) / model.mass;
    Eigen::Vector3f pDot = ((1.0 + p.squaredNorm()) / 4.0) * (Eigen::Matrix3f::Identity() + 2.0 * (pHat * pHat + pHat) / (1.0 + p.squaredNorm())) * w;
    Eigen::Vector3f wDot = J.inverse() * (t - wHat * J * w);

    QuadStateVector xDot;
    xDot << v, vDot, pDot, wDot;

    return xDot;
}

QuadStateVector Quadcopter::rk4(const QuadStateVector &x, const QuadControlsVector &u) const
{
    QuadStateVector k1 = model.dt * dynamics(x, u);
    QuadStateVector k2 = model.dt * dynamics(x + k1 / 2.0, u);
    QuadStateVector k3 = model.dt * dynamics(x + k2 / 2.0, u);
    QuadStateVector k4 = model.dt * dynamics(x + k3, u);
    return x + (1.0 / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
}