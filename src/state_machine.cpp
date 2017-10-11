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

#include <mpcqp_walking/state_machine.h>

namespace legged_robot
{

StateMachine::StateMachine(const unsigned int &single_support_knot_num, const unsigned int &double_support_knot_num)
{
    current_phase_knot_num_ = double_support_knot_num;
    single_support_knot_num_ = single_support_knot_num;
    double_support_knot_num_ = double_support_knot_num;
    contact_state_ = kDoubleSupport;
    previous_support_leg_ = kRightSupport;
}

StateMachine::~StateMachine()
{

}

unsigned int StateMachine::SetStartState(const unsigned int &contact_state, const unsigned int &current_phase_knot_num)
{
    contact_state_ = contact_state;
    current_phase_knot_num_ = current_phase_knot_num;
}

unsigned int StateMachine::Next()
{
    current_phase_knot_num_ --;
    if (current_phase_knot_num_ <= 0) {
        switch (contact_state_) {
        case kLeftSupport:
            if (double_support_knot_num_ > 0) {
                contact_state_ = kDoubleSupport;
                current_phase_knot_num_ = double_support_knot_num_;
            } else {
                contact_state_ = kRightSupport;
                current_phase_knot_num_ = single_support_knot_num_;
            }
            previous_support_leg_ = kLeftSupport;
            break;
        case kRightSupport:
            if (double_support_knot_num_ > 0) {
                contact_state_ = kDoubleSupport;
                current_phase_knot_num_ = double_support_knot_num_;
            } else {
                contact_state_ = kLeftSupport;
                current_phase_knot_num_ = single_support_knot_num_;
            }
            previous_support_leg_ = kRightSupport;
            break;
        default:
        case kDoubleSupport:
            if (previous_support_leg_ == kLeftSupport) {
                contact_state_ = kRightSupport;
            } else {
                contact_state_ = kLeftSupport;
            }
            current_phase_knot_num_ = single_support_knot_num_;
            break;
        }
    }
    return contact_state_;
}

unsigned int StateMachine::GetCurrentPhaseKnotNumber()
{
    return current_phase_knot_num_;
}


unsigned int StateMachine::GetSingleSupportKnotNumber()
{
    return single_support_knot_num_;
}

unsigned int StateMachine::GetDoubleSupportKnotNumber()
{
    return double_support_knot_num_;
}

unsigned int StateMachine::GetPreviousSupportLeg()
{
    return previous_support_leg_;
}

unsigned int StateMachine::GetCurrentSupportLeg()
{
    if (previous_support_leg_ == kLeftSupport) {
        return kRightSupport;
    } else {
        return kLeftSupport;
    }
}

unsigned int StateMachine::GetContactState()
{
    return contact_state_;
}


void StateMachine::SetSingleSupportKnotNumber(const unsigned int &num)
{
    single_support_knot_num_ = num;
}

void StateMachine::SetDoubleSupportKnotNumber(const unsigned int &num)
{
    double_support_knot_num_ = num;
}

void StateMachine::SetPreviouSupportLeg(const unsigned int &previous_support_leg)
{
    if (previous_support_leg == kLeftSupport)
        previous_support_leg_ = kLeftSupport;
    else
        previous_support_leg_ = kRightSupport;
}

}
