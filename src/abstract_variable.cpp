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

#include <mpcqp_walking/abstract_variable.h>

namespace legged_robot
{
using namespace Eigen;

OnePoint::OnePoint()
{
    pos.setZero();
    vel.setZero();
    acc.setZero();

    euler_pos.setZero();
    euler_vel.setZero();
    euler_acc.setZero();
}

OnePoint::~OnePoint()
{

}

AbstractVariable AbstractVariable::ToLocalCoordinate()
{
    return ToLocalCoordinate ( contact_state );
}

AbstractVariable AbstractVariable::ToLocalCoordinate ( const unsigned int &expected_contact_state, const bool &set_contact_state )
{
    OnePoint origin;
    AbstractVariable output = *this;

    switch ( expected_contact_state ) {
    case StateMachine::kLeftSupport:
        origin = lsole;
        break;
    case StateMachine::kRightSupport:
        origin = rsole;
        break;
    default:
        origin.pos = 0.5 * ( lsole.pos + rsole.pos );
    }

    if ( set_contact_state ) {
        output.contact_state = expected_contact_state;
    } else {
        output.contact_state = contact_state;
    }
//     output.current_phase_knot_num = current_phase_knot_num;
    output.com.pos = com.pos - origin.pos;
    output.pelvis.pos = pelvis.pos - origin.pos;
    output.lsole.pos = lsole.pos - origin.pos;
    output.rsole.pos = rsole.pos - origin.pos;

    return output;
}

}
