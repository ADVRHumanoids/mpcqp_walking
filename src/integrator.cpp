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
    poly_ = vector<QuinticPolynomial>(12);
}

Integrator::Integrator(const AbstractVariable &start, const AbstractVariable &end, const double &duration, const double &step_time)
{
    current_time_ = 0;
    step_time_ = step_time;
    poly_num_ = 12;
    MatrixXd start_state(3, poly_num_);
    start_state.row(0) << start.com.pos.transpose(), start.pelvis.pos.transpose(), start.lsole.pos.transpose(), start.rsole.pos.transpose();
    start_state.row(1) << start.com.vel.transpose(), start.pelvis.vel.transpose(), start.lsole.vel.transpose(), start.rsole.vel.transpose();
    start_state.row(2) << start.com.acc.transpose(), start.pelvis.acc.transpose(), start.lsole.acc.transpose(), start.rsole.acc.transpose();

    MatrixXd end_state(3, poly_num_);
    end_state.row(0) << end.com.pos.transpose(), end.pelvis.pos.transpose(), end.lsole.pos.transpose(), end.rsole.pos.transpose();
    end_state.row(1) << end.com.vel.transpose(), end.pelvis.vel.transpose(), end.lsole.vel.transpose(), end.rsole.vel.transpose();
    end_state.row(2) << end.com.acc.transpose(), end.pelvis.acc.transpose(), end.lsole.acc.transpose(), end.rsole.acc.transpose();

//     cout << "start state:\n" << start_state << endl;
//     cout << "end state:\n" << end_state << endl;

    for (unsigned int i = 0; i < poly_num_; i++) {
        poly_.push_back(QuinticPolynomial(start_state.col(i), end_state.col(i), duration, step_time_));
    }

    current_value_ = start;
    duration_ = duration;
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
    MatrixXd state(3, poly_num_);
    for (unsigned int i = 0; i < poly_num_; i++) {
        state.col(i) = poly_[i].Output(time);
    }

//     if (abs(time - 1) < 0.001) {
//       cout << "state:\n" << state << endl;
//     }

    current_value_.com.pos = state.block(0, 0, 1, 3).transpose();
    current_value_.com.vel = state.block(1, 0, 1, 3).transpose();
    current_value_.com.acc = state.block(2, 0, 1, 3).transpose();

    current_value_.pelvis.pos = state.block(0, 3, 1, 3).transpose();
    current_value_.pelvis.vel = state.block(1, 3, 1, 3).transpose();
    current_value_.pelvis.acc = state.block(2, 3, 1, 3).transpose();

    current_value_.lsole.pos = state.block(0, 6, 1, 3).transpose();
    current_value_.lsole.vel = state.block(1, 6, 1, 3).transpose();
    current_value_.lsole.acc = state.block(2, 6, 1, 3).transpose();

    current_value_.rsole.pos = state.block(0, 9, 1, 3).transpose();
    current_value_.rsole.vel = state.block(1, 9, 1, 3).transpose();
    current_value_.rsole.acc = state.block(2, 9, 1, 3).transpose();

    return current_value_;
}

AbstractVariable Integrator::TickAndOuput()
{
    Tick();
    return Output();
}

bool Integrator::IsExpired()
{
    return (current_time_ >= duration_ - 0.00001);
}

}
