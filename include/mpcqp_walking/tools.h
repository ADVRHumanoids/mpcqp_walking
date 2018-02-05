/*
  Copyright (C) 2017 WALK-MAN, cogimon
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

#ifndef TOOLS_H
#define TOOLS_H

#include <Eigen/Dense>
#include "abstract_variable.h"

void QuinticPolynomial(const Eigen::Vector3d &start, const Eigen::Vector3d &end, const double &duration, const Eigen::VectorXd &time, Eigen::MatrixXd &result );

void QuinticPolynomial(const Eigen::Vector3d &start, const Eigen::Vector3d &end, const double &duration, const double &time, Eigen::Vector3d &result );

#endif
