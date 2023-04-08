#include "Quadcopter.hpp"

MX Quadcopter::dynamics(const QuadcopterModel& model, const MX& x, const MX& u) {
    MX r = x(Slice(0,3));
    MX v = x(Slice(3,6));
    MX p = x(Slice(6,9));
    MX w = x(Slice(9,12));

    MX Q = dcmFromMrp(p);

    MX f1 = fmax(0, model.kf * u(0));
    MX f2 = fmax(0, model.kf * u(1));
    MX f3 = fmax(0, model.kf * u(2));
    MX f4 = fmax(0, model.kf * u(3));

    MX totalF = MX::zeros(3, 1);
    totalF(2) = f1 + f2 + f3 + f4;

    MX m1 = model.km * u(0);
    MX m2 = model.km * u(1);
    MX m3 = model.km * u(2);
    MX m4 = model.km * u(3);

    MX t = MX::zeros(3, 1);
    t(0) = model.L * (f2-f4);
    t(1) = model.L * (f3-f1);
    t(2) = m1-m2+m3-m4;

    MX v_dot = (model.mass * model.gravity + mtimes(Q, totalF)) / model.mass;
    MX p_dot = mtimes(((1.0 + sumsqr(p)) / 4.0), mtimes((MX::eye(3) + 2.0 * (mpower(skew(p),2) + skew(p))/ (1.0 + sumsqr(p))), w));
    MX w_dot = mtimes(inv(model.J), (t - cross(w, mtimes(model.J, w))));

    MX x_dot = vertcat(v, 
                       v_dot, 
                       p_dot,
                       w_dot);
    return x_dot;
}

MX Quadcopter::rk4(const QuadcopterModel& model, const MX& x, const MX& u)
{
    MX k1 = model.dt * dynamics(model, x, u);
    MX k2 = model.dt * dynamics(model, x + k1/2, u);
    MX k3 = model.dt * dynamics(model, x + k2/2, u);
    MX k4 = model.dt * dynamics(model, x + k3, u);
    return x + (1/6)*(k1 + 2*k2 + 2*k3 + k4);
}