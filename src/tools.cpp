/*
  Copyright (C) 2017 cogimon
  Author:  Yangwei You, Enrico Mingo Hoffman
  email: yangwei.you@iit.it, enrico.mingo@iit.it
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU Lesser General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU Lesser General Public License for more details.
  You should have received a copy of the GNU Lesser General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>
*/

#include <mpcqp_walking/tools.h>

void QuinticPolynomial(const Eigen::Vector3d &start, const Eigen::Vector3d &end, const double &duration, const Eigen::VectorXd &time, Eigen::MatrixXd &result)
{
    Eigen::VectorXd coeffs(6);

    double startPos = start[0];
    double startVel = start[1];
    double startAcc = start[2];
    double endPos = end[0];
    double endVel = end[1];
    double endAcc = end[2];

    coeffs(0) = -0.5 * (12 * startPos - 12 * endPos + 6 * startVel * duration + 6 * endVel * duration + startAcc * pow(duration, 2) - endAcc * pow(duration, 2)) / pow(duration, 5);
    coeffs(1) = 0.5 * (30 * startPos - 30 * endPos + 16 * startVel * duration + 14 * endVel * duration + 3 * startAcc * pow(duration, 2) - 2 * endAcc * pow(duration, 2)) / pow(duration, 4);
    coeffs(2) = -0.5 * (20 * startPos - 20 * endPos + 12 * startVel * duration + 8 * endVel * duration + 3 * startAcc * pow(duration, 2) - endAcc * pow(duration, 2)) / pow(duration, 3);
    coeffs(3) = 0.5 * startAcc;
    coeffs(4) = startVel;
    coeffs(5) = startPos;

    Eigen::MatrixXd t_matrix(6, time.size());
    Eigen::MatrixXd coeffs_matrix(3, 6);

    for (unsigned int i = 0; i < 6; i++) {
        t_matrix.row(i) = (time.array().pow(5 - i)).transpose();
    }

    coeffs_matrix.row(0) = coeffs.transpose();
    coeffs_matrix.row(1) << 0, 5 * coeffs(0), 4 * coeffs(1), 3 * coeffs(2), 2 * coeffs(3), coeffs(4);
    coeffs_matrix.row(2) << 0, 0, 20 * coeffs(0), 12 * coeffs(1), 6 * coeffs(2), 2 * coeffs(3);

    result.resize(3, time.size());
    result = coeffs_matrix * t_matrix;
}

void QuinticPolynomial(const Eigen::Vector3d &start, const Eigen::Vector3d &end, const double &duration, const double &time, Eigen::Vector3d &result)
{
    Eigen::VectorXd time_v(1);
    time_v << time;
    Eigen::MatrixXd result_m;
    QuinticPolynomial(start, end, duration, time_v, result_m);
    result << result_m;
}
