#include <mpcqp_walking/integrator.h>

namespace legged_robot
{
using namespace Eigen;
using namespace std;

Integrator::Integrator()
{
    current_time_ = 0;
    duration_ = 0;
    step_time_ = 0;
}

Integrator::Integrator(const AbstractVariable &start, const AbstractVariable &end, const double &duration, const double &step_time)
{
    current_value_ = start;
    duration_ = duration;
    step_time_ = step_time;
    
    com_state_ << start.com.pos, start.com.vel, start.com.acc;
    pelvis_state_ << start.pelvis.pos, start.pelvis.vel, start.pelvis.acc;
    lsole_state_<< start.lsole.pos, start.lsole.vel, start.lsole.acc;
    rsole_state_ << start.rsole.pos, start.rsole.vel, start.rsole.acc;

    com_jerk_ = 1 / duration * (end.com.acc - start.com.acc);
    pelvis_jerk_ = 1 / duration * (end.pelvis.acc - start.pelvis.acc);
    lsole_jerk_ = 1 / duration * (end.lsole.acc - start.lsole.acc);
    rsole_jerk_ = 1 / duration * (end.rsole.acc - start.rsole.acc);
}


Integrator::~Integrator()
{

}


void Integrator::Tick()
{
    current_time_ += step_time_;
}

AbstractVariable Integrator::Output()
{
    return Output(current_time_);
}

AbstractVariable Integrator::Output(const double &time)
{
    Matrix3d A;
    Vector3d B;
    Matrix3d C;
    A << 1, time, pow(time, 2) / 2,
    0, 1, time,
    0, 0, 1;
    B << pow(time, 3)/6, pow(time, 2)/2, time;
    
    C = A*com_state_.transpose() + B*com_jerk_.transpose();
    current_value_.com.pos = C.row(0);
    current_value_.com.vel = C.row(1);
    current_value_.com.acc = C.row(2);

    C = A*pelvis_state_.transpose() + B*pelvis_jerk_.transpose();
    current_value_.pelvis.pos = C.row(0);
    current_value_.pelvis.vel = C.row(1);
    current_value_.pelvis.acc = C.row(2);
    
    C = A*lsole_state_.transpose() + B*lsole_jerk_.transpose();
    current_value_.lsole.pos = C.row(0);
    current_value_.lsole.vel = C.row(1);
    current_value_.lsole.acc = C.row(2);
    
    C = A*rsole_state_.transpose() + B*rsole_jerk_.transpose();
    current_value_.rsole.pos = C.row(0);
    current_value_.rsole.vel = C.row(1);
    current_value_.rsole.acc = C.row(2);
    
      return current_value_;
}

AbstractVariable Integrator::TickAndOuput()
{
    Tick();
    return Output();
}

bool Integrator::IsExpired()
{
    return (current_time_ > duration_);
}

}
