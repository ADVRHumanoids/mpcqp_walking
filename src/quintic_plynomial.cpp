#include <mpcqp_walking/quintic_polynomial.h>

namespace legged_robot
{
using namespace Eigen;

QuinticPolynomial::QuinticPolynomial()
{
    coeffs_matrix_ = MatrixXd::Zero(3, 6);
    current_time_ = 0;
    step_time_ = 0;
}

QuinticPolynomial::QuinticPolynomial(const Vector3d &start, const Vector3d &end, const double &duration, const double &step_time)
{
    coeffs_matrix_ = MatrixXd::Zero(3, 6);
    current_time_ = 0;

    VectorXd coeffs(6);
    coeffs[0] = -0.5 * (12 * start[0] - 12 * end[0] + 6 * start[1] * duration + 6 * end[1] * duration + start[2] * pow(duration, 2) - end[2] * pow(duration, 2)) / pow(duration, 5);
    coeffs[1] = 0.5 * (30 * start[0] - 30 * end[0] + 16 * start[1] * duration + 14 * end[1] * duration + 3 * start[2] * pow(duration, 2) - 2 * end[2] * pow(duration, 2)) / pow(duration, 4);
    coeffs[2] = -0.5 * (20 * start[0] - 20 * end[0] + 12 * start[1] * duration + 8 * end[1] * duration + 3 * start[2] * pow(duration, 2) - end[2] * pow(duration, 2)) / pow(duration, 3);
    coeffs[3] = 0.5 * start[2];
    coeffs[4] = start[1];
    coeffs[5] = start[0];

    coeffs_matrix_.row(0) = coeffs.transpose();
    coeffs_matrix_.row(1) << 0, 5 * coeffs(0), 4 * coeffs(1), 3 * coeffs(2), 2 * coeffs(3), coeffs(4);
    coeffs_matrix_.row(2) << 0, 0, 20 * coeffs(0), 12 * coeffs(1), 6 * coeffs(2), 2 * coeffs(3);

    step_time_ = step_time;
}

QuinticPolynomial::~QuinticPolynomial()
{

}

void QuinticPolynomial::Tick()
{
    current_time_ += step_time_;
}

Vector3d QuinticPolynomial::Output()
{
    return Output(current_time_);
}

Vector3d QuinticPolynomial::TickAndOutput()
{
    Tick();
    return Output();
}


Vector3d QuinticPolynomial::Output(const double &time)
{
    Eigen::VectorXd t(6);

    for (unsigned int i = 0; i < 6; i++) {
        t[i] = pow(time, 5 - i);
    }

    return coeffs_matrix_ * t;
}

MatrixXd QuinticPolynomial::Output(const VectorXd &time)
{
    Eigen::MatrixXd t_matrix(6, time.size());

    for (unsigned int i = 0; i < 6; i++) {
        t_matrix.row(i) = (time.array().pow(5 - i)).transpose();
    }

    return coeffs_matrix_ * t_matrix;
}

void QuinticPolynomial::ResetTime()
{
    current_time_ = 0;
}


}
