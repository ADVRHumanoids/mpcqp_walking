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

#ifndef ABSTRACT_VARIABLE_HPP
#define ABSTRACT_VARIABLE_HPP

#include <stdlib.h>
#include <math.h>
#include <boost/concept_check.hpp>
#include <eigen3/Eigen/Dense>
#include "state_machine.h"
#include <XBotInterface/Logger.hpp>


/// \namespace legged_robot
namespace legged_robot
{

struct OnePoint {
    Eigen::Vector3d pos;
    Eigen::Vector3d vel;
    Eigen::Vector3d acc;

    Eigen::Vector3d euler_pos; //roll, pitch, yaw
    Eigen::Vector3d euler_vel;
    Eigen::Vector3d euler_acc;

    OnePoint();
    ~OnePoint();

    virtual void log(XBot::MatLogger::Ptr logger, const std::string& id)
    {
        logger->add(id + "_pos", pos);
        logger->add(id + "_vel", vel);
        logger->add(id + "_acc", acc);

        logger->add(id + "_euler_pos", euler_pos);
        logger->add(id + "_euler_vel", euler_vel);
        logger->add(id + "_euler_acc", euler_acc);
    }

};

struct AbstractVariable {
    OnePoint com;
    OnePoint pelvis;
    OnePoint lsole;
    OnePoint rsole;
    unsigned int contact_state;
    unsigned int current_phase_knot_num;

    virtual void log(XBot::MatLogger::Ptr logger, const std::string& id)
    {
        com.log(logger, id+"_com");
        pelvis.log(logger, id+"_pelvis");
        lsole.log(logger, id+"_lsole");
        rsole.log(logger, id+"_rsole");
    }

    AbstractVariable ToLocalCoordinate();
    AbstractVariable ToLocalCoordinate ( const unsigned int &expected_contact_state, const bool &set_contact_state = false );
};

}

#endif
