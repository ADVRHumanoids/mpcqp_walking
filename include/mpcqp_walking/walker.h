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

#ifndef _WALKER_HPP_
#define _WALKER_HPP_

#include "model_preview_control.h"
#include <XBotInterface/ModelInterface.h>

namespace legged_robot{

/**
 * @brief The Walker class wrap the MPC class with an interface to the ModelInterface
 */
class Walker
{
public:
    /**
     * @brief Walker
     * @param robot reference to a robot model
     * @param dT control loop in seconds
     * @param single_support_phase_time time in single support phase in seconds
     * @param double_support_phase_time time in double support phase in seconds
     * @param horizontal_center_feet_distance distance (along y axis) between the 2 center of the
     * @param foot_size size of a foot
     * @param l_foot_center_frame name of a frame in the center of the left foot
     * @param r_foot_center_frame name of a frame in the center of the right foot
     * @param pelvis_frame name of a frame in the pelvis
     * feet
     */
    Walker(XBot::ModelInterface& robot, const double dT,
           const double single_support_phase_time, const double double_support_phase_time,
           const Eigen::Vector2d& foot_size,
           const std::string& l_foot_center_frame,
           const std::string& r_foot_center_frame,
           const std::string& pelvis_frame);
    ~Walker();

    /**
     * @brief setCurrentState from a state
     * @param state a state
     */
    bool setCurrentState(const AbstractVariable& state);

    void setReference(const Eigen::Vector2d& xy_com_twist);

    void solve(legged_robot::AbstractVariable& new_state);

    const legged_robot::AbstractVariable& getCurrentState() const
    {
        return _current_state;
    }

    void log(XBot::MatLogger::Ptr logger, const std::string& id)
    {
        _current_state.log(logger, id+"_current_state");
    }

    double getDuration()
    {
        return _dT;
    }

    void setStepHeight(const double height)
    {
        if(height > 0.)
            _step_height = height;
    }

    void setFootSpan(const double foot_span)
    {
        if(foot_span > 0.)
            _mpc->SetFootSpan(foot_span);
    }

    double getFootSpan()
    {
        return _mpc->GetFootSpan();
    }



private:
    boost::shared_ptr<LIPM> _robot_lipm;
    boost::shared_ptr<StateMachine> _sm;
    boost::shared_ptr<MPC> _mpc;

    bool initFromRobot(const XBot::ModelInterface& robot,
                         const unsigned int contact_state);


    /**
     * @brief current_state wrt world frame
     */
    AbstractVariable _current_state;


    double _dT;
    double _single_support_phase_time;
    double _double_support_phase_time;
    double _step_height;

    Eigen::Vector2d _foot_size;

    Eigen::Affine3d _tmp_affine;

    std::string _l_frame;
    std::string _r_frame;
    std::string _pelvis_frame;

    Eigen::MatrixXd _com_desired_twist_window;
};

}

#endif
