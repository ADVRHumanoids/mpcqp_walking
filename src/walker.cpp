/*
  Copyright (C) 2017 cogimon
  Author:  Enrico Mingo Hoffman
  email: enrico.mingo@iit.it

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

#include <mpcqp_walking/walker.h>

namespace legged_robot{

Walker::Walker(const XBot::ModelInterface &robot, const double dT,
               const double single_support_phase_time, const double double_support_phase_time,
               const double horizontal_center_feet_distance, const Eigen::Vector2cd &foot_size,
               const std::string& l_foot_center_frame, const std::string& r_foot_center_frame,
               const std::string& pelvis_frame):
    _robot(robot),
    _dT(dT),
    _single_support_phase_time(single_support_phase_time),
    _double_support_phase_time(double_support_phase_time),
    _horizontal_center_feet_distance(horizontal_center_feet_distance),
    _foot_size(foot_size),
    _l_frame(l_foot_center_frame),
    _r_frame(r_foot_center_frame),
    _pelvis_frame(pelvis_frame)
{
    _robot.getCOM(_com);

    _robot_lipm.reset(new LIPM(dT, _com[2]));

    if(single_support_phase_time >= dT && double_support_phase_time >= dT)
        _sm.reset(new StateMachine(int(single_support_phase_time/dT),
                                   int(double_support_phase_time/dT)));
    else
        std::cout<<"ERROR! SINGLE/DOUBLE SUPPORT PHASE < dT!"<<std::endl;



    _mpc.reset(new MPC(_robot_lipm,
                       _horizontal_center_feet_distance,
                       _foot_size[0]/2., _foot_size[1]/2.,
                       _sm));


}

void Walker::setCurrentState(const AbstractVariable &state)
{
    _current_state = state;
}

void Walker::setCurrentState(const XBot::ModelInterface &robot)
{
    robot.getCOM(_current_state.com.pos);
    robot.getPose(_l_frame, _tmp_affine);
    _current_state.lsole.pos = _tmp_affine.translation(); ///TODO: add orientation!
    robot.getPose(_r_frame, _tmp_affine);
    _current_state.rsole.pos = _tmp_affine.translation(); ///TODO: add orientation!
    robot.getPose(_pelvis_frame, _tmp_affine);
    _current_state.pelvis.pos = _tmp_affine.translation(); ///TODO: add orientation!
}

Walker::~Walker()
{

}

}
