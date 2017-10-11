/*
  Copyright (C) 2017 cogimon
  Author:  , Enrico Mingo Hoffman
  email:  , enrico.mingo@iit.it

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

#ifndef LINEAR_INVERTED_PENDULUM_MODEL_HPP
#define LINEAR_INVERTED_PENDULUM_MODEL_HPP

#include <stdlib.h>
#include <math.h>
#include <boost/concept_check.hpp>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include "defines.h"

/// \namespace legged_robot
namespace legged_robot
{

/// \class LIPM
///
/// \brief represent linear inverted pendulum model
///
/// \example
class LIPM
{
public:
    LIPM();

    /**
     * @brief LIPM constructor
     * @param knot_time time between two consecutive knots in the mpc
     * @param com_height z com
     * @param gravity
     */
    LIPM ( const double &knot_time,
           const double &com_height,
           const double &gravity = DEFAULT_GRAVITY );
    ~LIPM();

    /// \brief system model: Xnext = A*X + B*U; ZMP = C*X.
    struct System
    {
        Eigen::Matrix3d A;
        Eigen::Vector3d B;
        Eigen::Vector3d C;
    double knot_time;
    double com_height;
    double gravity;
    };

    void CalculateOneStep ( const Eigen::Vector3d &X, const double &Ux, Eigen::Vector3d &Xnext );

    void CalculateOneStep ( const Eigen::Vector3d &X, const Eigen::Vector3d &Y, const double &Ux, const double &Uy, Eigen::Vector3d &Xnext, Eigen::Vector3d &Ynext );

    void CalculateZMP ( const Eigen::Vector3d &X, double &ZMPx );

    void CalculateZMP ( const Eigen::Vector3d &X, const Eigen::Vector3d &Y, double &ZMPx, double &ZMPy );

    System GetModel();

    void ResetModel( const double &knot_time, const double &com_height, const double &gravity );

private:
    System system_;

};

}

#endif
