#ifndef INTEGRATOR_HPP
#define INTEGRATOR_HPP

#include <stdlib.h>
#include <math.h>
#include <eigen3/Eigen/Dense>
#include "abstract_variable.h"

/// \namespace legged_robot
namespace legged_robot
{

class Integrator
{
public:
    Integrator();
    Integrator(const AbstractVariable &start, const AbstractVariable &end, const double &duration, const double &step_time);
    ~Integrator();

    void Tick();
    AbstractVariable Output();
    AbstractVariable Output(const double &time);
    AbstractVariable TickAndOuput();
    bool IsExpired();
    void set(const AbstractVariable &start, const AbstractVariable &end, const double &duration, const double &step_time);

private:
    AbstractVariable current_value_;
    double duration_;
    double current_time_;
    double step_time_;
    Eigen::Matrix3d com_state_;
    Eigen::Matrix3d pelvis_state_;
    Eigen::Matrix3d lsole_state_;
    Eigen::Matrix3d rsole_state_;
    Eigen::Vector3d com_jerk_;
    Eigen::Vector3d pelvis_jerk_;
    Eigen::Vector3d lsole_jerk_;
    Eigen::Vector3d rsole_jerk_;
};

}

#endif
