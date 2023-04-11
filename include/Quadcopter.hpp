#ifndef QUADCOPTER_H
#define QUADCOPTER_H

#include <casadi/casadi.hpp>
#include <eigen3/Eigen/Core>

using namespace casadi;

struct QuadcopterModel
{
    float mass;
    float J[3];
    float L;
    float kf;
    float km;
    float dt;
    float accelLimit;
};

typedef Eigen::Matrix<float, 12, 1> QuadStateVector;
typedef Eigen::Matrix<float, 4, 1> QuadControlsVector;

class Quadcopter
{
public:
    Quadcopter()
    {
        model = {0.5, {0.0023, 0.0023, 0.004}, 0.1750, 1.0, 0.0245, 0.1, 2.0};

        gravity_casadi = MX::zeros(3, 1);
        gravity_casadi(2) = -9.81;

        gravity_eigen = Eigen::Vector3f::Zero();
        gravity_eigen(2) = -9.81;

        
        MX dx = MX::sym("x", Quadcopter::nx, 1);
        MX du = MX::sym("u", Quadcopter::nu, 1);
        jacA = Function("jacA", {dx, du}, {jacobian(rk4(dx, du), dx)});
        jacB = Function("jacB", {dx, du}, {jacobian(rk4(dx, du), du)});
    }

    QuadStateVector dynamics(const QuadStateVector &x, const QuadControlsVector &u) const;
    QuadStateVector rk4(const QuadStateVector &x, const QuadControlsVector &u) const;
    MX dynamics(const MX& x, const MX& u);
    MX rk4(const MX& x, const MX& u);

    
    QuadcopterModel model;
    static const int nx = 12;
    static const int nu = 4;
    Function jacA;
    Function jacB;

private:
    Eigen::Vector3f gravity_eigen;
    MX gravity_casadi;

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

    inline Eigen::Matrix3f dcmFromMrp(const Eigen::Vector3f &vec) const
    {
        float p1 = vec(0);
        float p2 = vec(1);
        float p3 = vec(2);

        float den = pow(pow(p1, 2) + pow(p2, 2) + pow(p3, 2) + 1.0f, 2);
        float a = (4.0f * pow(p1, 2) + 4.0f * pow(p2, 2) + 4.0f * pow(p3, 2) - 4.0f);

        Eigen::Matrix3f result = Eigen::Matrix3f::Zero();

        result(0, 0) = -((8.0f * pow(p2, 2) + 8.0f * pow(p3, 2)) / den - 1.0f) * den;
        result(0, 1) = 8.0f * p1 * p2 + p3 * a;
        result(0, 2) = 8.0f * p1 * p3 - p2 * a;
        result(1, 0) = 8.0f * p1 * p2 - p3 * a;
        result(1, 1) = -((8.0f * pow(p1, 2) + 8.0f * pow(p3, 2)) / den - 1.0f) * den;
        result(1, 2) = 8.0f * p2 * p3 + p1 * a;
        result(2, 0) = 8.0f * p1 * p3 + p2 * a;
        result(2, 1) = 8.0f * p2 * p3 - p1 * a;
        result(2, 2) = -((8.0f * pow(p1, 2) + 8.0f * pow(p2, 2)) / den - 1.0f) * den;

        return result / den;
    }
};

template <short N_IN, short N_OUT>
class VectorFunction
{

public:
    VectorFunction()
    {
        quadModel = {0.5, {0.0023, 0.0023, 0.004}, 0.1750, 1.0, 0.0245, 0.1};
    }

    // Eigen needs this to determine no. of variables at compile time
    enum
    {
        InputsAtCompileTime = N_IN,
        ValuesAtCompileTime = N_OUT
    };

    // Also needed by Eigen
    typedef Eigen::Matrix<float, N_IN, 1> InputType;
    typedef Eigen::Matrix<float, N_OUT, 1> ValueType;

    // Vector function
    template <typename T>
    void operator()(
        const Eigen::Matrix<T, N_IN, 1> &vIn,
        Eigen::Matrix<T, N_OUT, 1> *vOut) const
    {
        
    }

    void setControl(const QuadControlsVector &newU)
    {
        u = newU;
    }

    // Quadcopter quad;
    QuadcopterModel quadModel;
    Quadcopter quad;
    QuadControlsVector u;
};

#endif