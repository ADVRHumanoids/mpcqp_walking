#ifndef INTEGRATOR_HPP
#define INTEGRATOR_HPP

#include <stdlib.h>
#include <math.h>
#include <eigen3/Eigen/Dense>
#include "quintic_polynomial.h"
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

private:
    AbstractVariable current_value_;
    double duration_;
    double current_time_;
    double step_time_;
    std::vector<QuinticPolynomial> poly_;
    unsigned int poly_num_;
};

}

#endif
