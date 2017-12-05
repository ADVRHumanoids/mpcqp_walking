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

#ifndef STATE_MACHINE_HPP
#define STATE_MACHINE_HPP

#include <stdlib.h>
#include <math.h>
#include <boost/concept_check.hpp>
#include <eigen3/Eigen/Dense>
#include "linear_inverted_pendulum_model.h"
#include "defines.h"

/// \namespace legged_robot
namespace legged_robot
{
/// \class StateMachine
///
/// \brief state machine for bipedal walking
///
/// \example
class StateMachine
{
public:
    enum ContactState {
        kNoneContact, kLeftSupport, kRightSupport, kDoubleSupport
    };

    /**
     * @brief StateMachine constructor
     * @param single_support_knot_num number of knots used for every single support phase
     * NOTE: the time of the single support is given by:
     *  single_support_knot_num * knot_time
     * @param double_support_knot_num number of knots used for every double support phase
     * NOTE: the time of the double support is given by:
     *  double_support_knot_num * knot_time
     */
    StateMachine(const unsigned int &single_support_knot_num,
                 const unsigned int &double_support_knot_num);
    ~StateMachine();

    unsigned int SetStartState(const unsigned int &contact_state, const unsigned int &current_phase_knot_num);

    unsigned int Next();

    unsigned int GetCurrentPhaseKnotNumber();

    unsigned int GetSingleSupportKnotNumber();

    unsigned int GetDoubleSupportKnotNumber();

    unsigned int GetPreviousSupportLeg();

    unsigned int GetCurrentSupportLeg();

    unsigned int GetContactState();

    void SetSingleSupportKnotNumber(const unsigned int &num);

    void SetDoubleSupportKnotNumber(const unsigned int &num);

    void SetPreviouSupportLeg( const unsigned int &previous_support_leg);

private:


    unsigned int contact_state_;

    unsigned int current_phase_knot_num_;

    unsigned int single_support_knot_num_;

    unsigned int double_support_knot_num_;

    unsigned int previous_support_leg_;

};

}

#endif
