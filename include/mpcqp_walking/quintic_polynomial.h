#ifndef QUINTIC_POLYNOMIAL_HPP
#define QUINTIC_POLYNOMIAL_HPP

#include <stdlib.h>
#include <math.h>
#include <eigen3/Eigen/Dense>

namespace legged_robot
{

class QuinticPolynomial
{
public:
    QuinticPolynomial();
    QuinticPolynomial(const Eigen::Vector3d &start, const Eigen::Vector3d &end, const double &duration, const double &step_time);
    ~QuinticPolynomial();

    void Tick();
    Eigen::Vector3d Output();
    Eigen::Vector3d TickAndOutput();
    Eigen::Vector3d Output(const double &time);
    Eigen::MatrixXd Output(const Eigen::VectorXd &time);
    void ResetTime();

private:
    Eigen::MatrixXd coeffs_matrix_;
    double current_time_;
    double step_time_;
};

}

#endif
