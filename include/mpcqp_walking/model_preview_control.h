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

#ifndef MODEL_PREVIEW_CONTROL_HPP
#define MODEL_PREVIEW_CONTROL_HPP

#include <stdlib.h>
#include <math.h>
#include <algorithm>
#include <boost/concept_check.hpp>
#include <eigen3/Eigen/Dense>
#include "linear_inverted_pendulum_model.h"
#include "state_machine.h"
#include <iostream>
#include <eigen3/unsupported/Eigen/MatrixFunctions> // for using MatrixPower<MatrixXd> Apow(A) to calculate A^n
#include "eigen_quadsolve.hpp"
//#include "utils/saveEigen.h"
//#include "utils/tools.h"
#include "variable/abstract_variable.hpp"

/// \namespace legged_robot
namespace legged_robot
{

/// \class MPC
///
/// \brief realize model preview control
///
/// \example
class MPC
{
public:
    MPC(const double &com_height = 1,
    const double &foot_span = 0.15,
    const double &foot_half_length = 0.1,
    const double &foot_half_width = 0.05,
        const unsigned int &single_support_knot_num = 15,
        const unsigned int &double_support_knot_num = 1,
        const unsigned int &preview_walking_step = 3,
        const double &step_time = 0.1,
        const double &jerk_weight = 1e-3,
        const double &velocity_weight = 1e+1, // 1e+1
        const double &zmp_weight = 1e+3,
        const double &gravity = 9.81);

    /**
     * @brief MPC constructor, here we assume that the foot is symmetric
     * @param model a linear inverted pendulum model
     * @param foot_span minimum y half distance between the center of the feet
     * @param foot_half_length half lenght of the feet (from the center)
     * @param foot_half_width half width of the feet (from the center)
     * @param state_machine a state machine object
     * @param preview_walking_step number of steps to look forward for the optimization window,
     * at least 2 steps are needed
     * @param jerk_weight higher result in a smoother control input
     * @param velocity_weight higher result in small tracking error of com velocities
     * @param zmp_weight higher result in small deviation of the zmp wrt the feet center
     */
    MPC(const LIPM &model,
    const double &foot_span,
    const double &foot_half_length,
    const double &foot_half_width,
        const StateMachine &state_machine,
        const unsigned int &preview_walking_step = 3,
        const double &jerk_weight = 1e-3,
        const double &velocity_weight = 1e+1,
        const double &zmp_weight = 1e+3);

    ~MPC();

    AbstractVariable Next(AbstractVariable &current_state, const Eigen::VectorXd &dXkp1_ref, const Eigen::VectorXd &dYkp1_ref);

    unsigned int GetPreviewKnotNumber();

    unsigned int GetSingleSupportKnotNumber();

    unsigned int GetDoubleSupportKnotNumber();

    unsigned int GetPreviewWalkingStep();

    unsigned int GetOptimizationVariableNum();

    void GetWeight(double &jerk_weight, double &velocity_weight, double &zmp_weight);

    double GetFootSpan();

    LIPM GetInternalModel();

    unsigned int GetPreviousSupportLeg();

    void SetPreviewKnotNumber(const unsigned int &num);

    void SetSingleSupportKnotNumber(const unsigned int &num);

    void SetDoubleSupportKnotNumber(const unsigned int &num);

    void SetPreviewWalkingStep( const unsigned int &num);

    void SetWeight(const double &jerk_weight, const double &velocity_weight, const double &zmp_weight);

    void ResetModel(const double &step_time, const double &com_height, const double &gravity);

    void ResetModel(const LIPM &model);

    void SetPreviouSupportLeg(const unsigned int &previous_support_leg);

    void InversePreviouSupportLeg();

    void SetFootSpan(const double &foot_span);

    void SetFootSize(const double &foot_half_length, const double &foot_half_width);

private:
//     enum ContactState {
//         kNoneContact, kLeftSupport, kRightSupport, kDoubleSupport
//     };

    void GeneratePredictionMatrix();

    void GenerateUMatrix();

    void GenerateQMatrix();

    void GeneratePVector();

    void GenerateZMPConstrain();

    void GeneratePlacementConstrain();

    LIPM model_;
    StateMachine state_machine_;
    AbstractVariable current_state_;
    unsigned int preview_walking_step_;
    unsigned int preview_knot_num_;
    unsigned int optimization_variable_num_;
    unsigned int current_phase_knot_num_;
    Eigen::VectorXd dXkp1_ref_;
    Eigen::VectorXd dYkp1_ref_;
    double foot_span_;
    double foot_half_length_;
    double foot_half_width_;

    Eigen::MatrixXd Pps_;

    Eigen::MatrixXd Ppu_;

    Eigen::MatrixXd Pvs_;

    Eigen::MatrixXd Pvu_;

    Eigen::MatrixXd Pas_;

    Eigen::MatrixXd Pau_;

    Eigen::MatrixXd Pzs_;

    Eigen::MatrixXd Pzu_;


    Eigen::MatrixXd Un_;

    Eigen::MatrixXd Uc_;

    Eigen::MatrixXd Ukp1_;

    Eigen::MatrixXd Uckp1_;

    Eigen::MatrixXd Qk_;

    Eigen::MatrixXd Qk_prime_;

    Eigen::VectorXd pk_;

    Eigen::MatrixXd CI_ZMP_;

    Eigen::VectorXd ci0_ZMP_;

    Eigen::MatrixXd CI_place_;

    Eigen::VectorXd ci0_place_;

    Eigen::Vector2d Xkfc_;
    Eigen::Vector2d Ykfc_;

    Eigen::Vector3d Xk_hat_;
    Eigen::Vector3d Yk_hat_;

    double alpha_;

    double beta_;

    double gamma_;
};

}

#endif
