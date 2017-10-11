/*
  Copyright (C) 2017 cogimon
  Author:  Yangwei You
  email: yangwei.you@iit.it

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

#include <mpcqp_walking/linear_inverted_pendulum_model.h>


namespace legged_robot
{
using namespace Eigen;

LIPM::LIPM ( const double &step_time, const double &com_height, const double &gravity )
{
    ResetModel(step_time, com_height, gravity);
}

LIPM::~LIPM()
{

}

void LIPM::CalculateOneStep ( const Vector3d& X, const double& Ux, Vector3d& Xnext )
{
    Xnext = system_.A*X + system_.B*Ux;
}

void LIPM::CalculateOneStep ( const Vector3d& X, const Vector3d& Y, const double& Ux, const double& Uy, Vector3d& Xnext, Vector3d& Ynext )
{
    CalculateOneStep ( X, Ux, Xnext );
    CalculateOneStep ( Y, Uy, Ynext );
}

void LIPM::CalculateZMP ( const Vector3d& X, double& ZMPx )
{
    ZMPx = (system_.C.transpose()*X).value();
}

void LIPM::CalculateZMP ( const Vector3d& X, const Vector3d& Y, double& ZMPx, double& ZMPy )
{
    CalculateZMP ( X, ZMPx );
    CalculateZMP ( Y, ZMPy );
}


LIPM::System LIPM::GetModel()
{
    return system_;
}

void LIPM::ResetModel ( const double& step_time, const double& com_height, const double& gravity )
{
    system_.step_time = step_time;
    system_.com_height = com_height;
    system_.gravity = gravity;

    system_.A << 1, system_.step_time, pow ( system_.step_time, 2 ) /2,
              0, 1, system_.step_time,
              0, 0, 1;

    system_.B << pow ( system_.step_time, 3 ) /6, pow ( system_.step_time, 2 ) /2, system_.step_time;

    system_.C << 1, 0, -system_.com_height/system_.gravity;
}


}
