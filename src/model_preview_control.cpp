#include <mpcqp_walking/model_preview_control.h>
#include <mpcqp_walking/tools.h>
#include <Eigen/Dense>

namespace legged_robot
{
using namespace Eigen;

MPC::MPC(const double &com_height,
         const double &foot_span,
         const double &foot_half_length,
         const double &foot_half_width,
         const unsigned int &single_support_knot_num,
         const unsigned int &double_support_knot_num,
         const unsigned int &preview_walking_step,
         const double &knot_time,
         const double &jerk_weight,
         const double &velocity_weight,
         const double &zmp_weight,
         const double &gravity)
{
//     MPC(LIPM(knot_time, com_height, gravity),
// 	foot_span,
// 	StateMachine(single_support_knot_num, double_support_knot_num),
// 	preview_walking_step,
// 	jerk_weight,
// 	velocity_weight,
// 	zmp_weight
//        );

    model_ = LIPM(knot_time, com_height, gravity);
    state_machine_ = StateMachine(single_support_knot_num, double_support_knot_num);
    SetPreviewWalkingStep(preview_walking_step);
    preview_knot_num_ = preview_walking_step * (single_support_knot_num + double_support_knot_num);
    optimization_variable_num_ = 2 * (preview_knot_num_ + preview_walking_step);

    SetWeight(jerk_weight, velocity_weight, zmp_weight);
    SetFootSpan(foot_span);
    SetFootSize(foot_half_length, foot_half_width);
    GeneratePredictionMatrix();
}

MPC::MPC(const LIPM &model,
         const double &foot_span,
         const double &foot_half_length,
         const double &foot_half_width,
         const StateMachine &state_machine,
         const unsigned int &preview_walking_step,
         const double &jerk_weight,
         const double &velocity_weight,
         const double &zmp_weight)
{
    model_ = model;
    state_machine_ = state_machine;
    SetPreviewWalkingStep(preview_walking_step);
    preview_knot_num_ = preview_walking_step * (state_machine_.GetSingleSupportKnotNumber(), state_machine_.GetDoubleSupportKnotNumber());
    optimization_variable_num_ = 2 * (preview_knot_num_ + preview_walking_step);

    SetWeight(jerk_weight, velocity_weight, zmp_weight);
    SetFootSpan(foot_span);
    SetFootSize(foot_half_length, foot_half_width);
    GeneratePredictionMatrix();
}

MPC::~MPC()
{

}

void MPC::GeneratePredictionMatrix()
{
    Matrix3d Pssi;
    MatrixXd Pzsi(1, 3);
    MatrixXd Psui;
    MatrixXd Pzui;

    Psui.resize(3, preview_knot_num_);
    Pzui.resize(1, preview_knot_num_);

    Pps_.resize(preview_knot_num_, 3);
    Pvs_.resize(preview_knot_num_, 3);
    Pas_.resize(preview_knot_num_, 3);
    Pzs_.resize(preview_knot_num_, 3);

    Ppu_.resize(preview_knot_num_, preview_knot_num_);
    Pvu_.resize(preview_knot_num_, preview_knot_num_);
    Pau_.resize(preview_knot_num_, preview_knot_num_);
    Pzu_.resize(preview_knot_num_, preview_knot_num_);

    Matrix3d A = model_.GetModel().A; //!< use the temp value due to the requirement of MatrixPower
    MatrixPower<Matrix3d> Apow(A);

    for (unsigned int i = 1; i < preview_knot_num_ + 1; i++) {
        Pssi = Apow(i);
        Pzsi = model_.GetModel().C.transpose() * Apow(i);

        Psui.setZero();
        Pzui.setZero();

        for (unsigned int j = 1; j < i + 1; j++) {
            Psui.col(j - 1) = Apow(i - j) * model_.GetModel().B;
            Pzui(j - 1) = model_.GetModel().C.transpose() * Psui.col(j - 1);
        }

        Pps_.row(i - 1) = Pssi.row(0);
        Ppu_.row(i - 1) = Psui.row(0);

        Pvs_.row(i - 1) = Pssi.row(1);
        Pvu_.row(i - 1) = Psui.row(1);

        Pas_.row(i - 1) = Pssi.row(2);
        Pau_.row(i - 1) = Psui.row(2);

        Pzs_.row(i - 1) = Pzsi;
        Pzu_.row(i - 1) = Pzui;
    }

//     cout << "Pps: " << Pps_ << endl;
//     cout << "Ppu: " << Ppu_ << endl;
//     cout << "Pzs: " << Pzs_ << endl;
//     cout << "Pzu: " << Pzu_ << endl;
}

void MPC::GenerateUMatrix()
{
    MatrixXd U = MatrixXd::Zero(preview_knot_num_, 2 + preview_walking_step_);
    StateMachine temp_state_machine = state_machine_;
    unsigned int col = 0;
    unsigned int last_state = temp_state_machine.GetContactState();
    unsigned int current_state;
    for (unsigned int row = 0; row < preview_knot_num_; row++) {
        temp_state_machine.Next();
        current_state = temp_state_machine.GetContactState();
        if ((current_state != last_state) && (last_state != StateMachine::kDoubleSupport)) {
            col++;
        }
        if (current_state == StateMachine::kDoubleSupport) {
            U.block(row, col, 1, 2) << 0.5, 0.5;
        } else {
            U.block(row, col, 1, 2) << 0, 1;
        }
        last_state = current_state;
    }

    Uckp1_.resize(preview_knot_num_, 2);
    Ukp1_.resize(preview_knot_num_, preview_walking_step_);
    Uckp1_ = U.block(0, 0, preview_knot_num_, 2);
    Ukp1_ = U.block(0, 2, preview_knot_num_, preview_walking_step_);

//     cout << "-------------------------" << endl;
//     cout <<"Uc:\n" << Uckp1_ << endl;
//     cout <<"Uk:\n" << Ukp1_ << endl;
}

void MPC::GenerateQMatrix()
{
    Qk_prime_.resize(preview_knot_num_ + preview_walking_step_, preview_knot_num_ + preview_walking_step_);
    Qk_prime_.block(0, 0, preview_knot_num_, preview_knot_num_ + preview_walking_step_) << alpha_*MatrixXd::Identity(preview_knot_num_, preview_knot_num_) + beta_*Pvu_.transpose() *Pvu_ + gamma_*Pzu_.transpose() *Pzu_, -gamma_*Pzu_.transpose() *Ukp1_;
    Qk_prime_.block(preview_knot_num_, 0, preview_walking_step_, preview_knot_num_ + preview_walking_step_) << -gamma_*Ukp1_.transpose() *Pzu_, gamma_*Ukp1_.transpose() *Ukp1_;
    Qk_.resize(2 * Qk_prime_.rows(), 2 * Qk_prime_.cols());
    Qk_.setZero();
    Qk_.block(0, 0, Qk_prime_.rows(), Qk_prime_.cols()) = Qk_prime_;
    Qk_.block(Qk_prime_.rows(), Qk_prime_.cols(), Qk_prime_.rows(), Qk_prime_.cols()) = Qk_prime_;

//     cout << "Qk_prime:\n" << Qk_prime_ << endl;
}

void MPC::GeneratePVector()
{
//     Vector2d Xkfc;
//     Vector2d Ykfc;
    if (state_machine_.GetPreviousSupportLeg() == StateMachine::kLeftSupport) {
        Xkfc_[0] = current_state_.lsole.pos[0];
        Xkfc_[1] = current_state_.rsole.pos[0];
        Ykfc_[0] = current_state_.lsole.pos[1];
        Ykfc_[1] = current_state_.rsole.pos[1];
    } else {
        Xkfc_[0] = current_state_.rsole.pos[0];
        Xkfc_[1] = current_state_.lsole.pos[0];
        Ykfc_[0] = current_state_.rsole.pos[1];
        Ykfc_[1] = current_state_.lsole.pos[1];
    }

    Xk_hat_ << current_state_.com.pos[0], current_state_.com.vel[0], current_state_.com.acc[0];
    Yk_hat_ << current_state_.com.pos[1], current_state_.com.vel[1], current_state_.com.acc[1];
    pk_.resize(2 * (preview_knot_num_ + preview_walking_step_));
    pk_ << beta_*Pvu_.transpose() * (Pvs_ * Xk_hat_ - dXkp1_ref_) + gamma_*Pzu_.transpose() * (Pzs_ * Xk_hat_ - Uckp1_ * Xkfc_),
        -gamma_*Ukp1_.transpose() * (Pzs_ * Xk_hat_ - Uckp1_ * Xkfc_),
        beta_*Pvu_.transpose() * (Pvs_ * Yk_hat_ - dYkp1_ref_) + gamma_*Pzu_.transpose() * (Pzs_ * Yk_hat_ - Uckp1_ * Ykfc_),
        -gamma_*Ukp1_.transpose() * (Pzs_ * Yk_hat_ - Uckp1_ * Ykfc_);

//     cout << "pk: " << pk_.transpose() <<endl;
}

void MPC::GenerateZMPConstrain()
{
    double zmp_shrink = 0.0;
    double x_max = foot_half_length_ - zmp_shrink, x_min = -foot_half_length_ + zmp_shrink; // 0.12
    double y_max = foot_half_width_ - zmp_shrink, y_min = -foot_half_width_ + zmp_shrink; // 0.05
    double double_x_max = foot_half_length_, double_x_min = -foot_half_length_; //0.15
    double double_y_max = foot_span_, double_y_min = -foot_span_;
    VectorXd Xmax(preview_knot_num_), Xmin(preview_knot_num_);
    VectorXd Ymax(preview_knot_num_), Ymin(preview_knot_num_);

    CI_ZMP_.resize(preview_knot_num_ * 4, optimization_variable_num_);
    ci0_ZMP_.resize(preview_knot_num_ * 4);

    CI_ZMP_.setZero();
    ci0_ZMP_.setZero();

    CI_ZMP_.block(0, 0, preview_knot_num_, preview_knot_num_) = Pzu_;
    CI_ZMP_.block(0, preview_knot_num_, preview_knot_num_, preview_walking_step_) = -Ukp1_;
    CI_ZMP_.block(preview_knot_num_, preview_knot_num_ + preview_walking_step_, preview_knot_num_, preview_knot_num_) = Pzu_;
    CI_ZMP_.block(preview_knot_num_, 2 * preview_knot_num_ + preview_walking_step_, preview_knot_num_, preview_walking_step_) = -Ukp1_;

    CI_ZMP_.block(2 * preview_knot_num_, 0, 2 * preview_knot_num_, optimization_variable_num_) = -CI_ZMP_.block(0, 0, 2 * preview_knot_num_, optimization_variable_num_);

    bool current_double_support = true;
    StateMachine temp_state_machine = state_machine_;
    for (unsigned int i = 0; i < preview_knot_num_; i++) {
        temp_state_machine.Next();
        switch (temp_state_machine.GetContactState()) {
        case StateMachine::kDoubleSupport:
//             if (current_double_support = true) {
// 	      Xmax[i] = max(Xkfc_[0], Xkfc_[1])+foot_half_length_;
// 	      Xmin[i] = min(Xkfc_[0], Xkfc_[1])-foot_half_length_;
// 	      Ymax[i] = max(Ykfc_[0], Ykfc_[1])+foot_half_width_;
// 	      Ymin[i] = min(Ykfc_[0], Ykfc_[1])-foot_half_width_;
//             } else {
                Xmax[i] = double_x_max;
                Xmin[i] = double_x_min;
                Ymax[i] = double_y_max;
                Ymin[i] = double_y_min;
//             }
            break;
        default:
            Xmax[i] = x_max;
            Xmin[i] = x_min;
            Ymax[i] = y_max;
            Ymin[i] = y_min;
            current_double_support = false;
        }
    }
    ci0_ZMP_ << Xmax + Uckp1_*Xkfc_ - Pzs_*Xk_hat_, Ymax + Uckp1_*Ykfc_ - Pzs_*Yk_hat_, - (Xmin + Uckp1_ * Xkfc_ - Pzs_ * Xk_hat_), - (Ymin + Uckp1_ * Ykfc_ - Pzs_ * Yk_hat_);
}


void MPC::GeneratePlacementConstrain()
{
    MatrixXd S = MatrixXd::Zero(2, optimization_variable_num_);
    S(0, preview_knot_num_) = 1;
    S(1, 2 * preview_knot_num_ + preview_walking_step_) = 1;

    CI_place_.resize(4, optimization_variable_num_);
    CI_place_ << S, -S;

    double leg_limit = 0.5; // 0.5

    ci0_place_.resize(4);
    if (state_machine_.GetCurrentSupportLeg() == StateMachine::kLeftSupport) {
        ci0_place_ << current_state_.lsole.pos.head(2) + Vector2d(2 * leg_limit, -foot_span_), - (current_state_.lsole.pos.head(2) + Vector2d(-2 * leg_limit, -leg_limit - foot_span_));
    } else {
        ci0_place_ << current_state_.rsole.pos.head(2) + Vector2d(2 * leg_limit, leg_limit + foot_span_), - (current_state_.rsole.pos.head(2) + Vector2d(-2 * leg_limit, foot_span_));
    }
//     cout << "CI:\n" << CI_place_ << endl;
//     cout << "ci:\n" << ci0_place_ << endl;
}

void MPC::Next(AbstractVariable& next_state, AbstractVariable &current_state, const VectorXd &dXkp1_ref, const VectorXd &dYkp1_ref)
{
    current_phase_knot_num_ = current_state.current_phase_knot_num;
    current_state_ = current_state;
    state_machine_.SetStartState(current_state_.contact_state, current_phase_knot_num_);
    dXkp1_ref_ = dXkp1_ref.head(preview_knot_num_);
    dYkp1_ref_ = dYkp1_ref.head(preview_knot_num_);

    GenerateUMatrix();

    GenerateQMatrix();

    GeneratePVector();

    GenerateZMPConstrain();

    GeneratePlacementConstrain();

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // use eigen_quadsolve
    MatrixXd empty_matrix;
    VectorXd empty_vector;
    VectorXd X(optimization_variable_num_);
    Qk_ = Qk_ + 0.00000001 * MatrixXd::Identity(Qk_.rows(), Qk_.rows());  // 0.00000001: use a very small value to adjust the objective matrix to make optimization problem solvable.
    MatrixXd CI(CI_ZMP_.rows() + CI_place_.rows(), optimization_variable_num_);
    VectorXd ci0(CI.rows());
    CI << CI_ZMP_, CI_place_;
    ci0 << ci0_ZMP_, ci0_place_;

    solve_quadprog(Qk_, pk_, empty_matrix, empty_vector, -CI.transpose(), ci0, X);

//     cout.setf ( ios::fixed,ios::floatfield );
//     cout.precision ( 4 );
//     cout << "QK:\n" << Qk_ << endl;
//     cout << "pk_: " << pk_.transpose() << endl;
//     cout << "X: " << X.transpose() << endl;

//     MatrixXd Qk_x, Qk_y;
//     VectorXd pk_x, pk_y;
//     VectorXd X_x, X_y;
//     unsigned int half = optimization_variable_num_ / 2;
//     Qk_x = Qk_.block(0, 0, half, half);
//     Qk_y = Qk_.block(half, half, half, half);
//     pk_x = pk_.head(half);
//     pk_y = pk_.segment(half, half);
//     unsigned int half_ci = CI.rows() / 2;
//
//     MatrixXd CI_x(half_ci, half), CI_y(half_ci, half);
//     VectorXd ci0_x(half_ci), ci0_y(half_ci);
//     CI_x << CI_ZMP_.block(0, 0, CI_ZMP_.rows() / 4, half), CI_ZMP_.block(CI_ZMP_.rows() / 2, 0, CI_ZMP_.rows() / 4, half), CI_place_.block(0, 0, 1, half), CI_place_.block(2, 0, 1, half);
//     CI_y << CI_ZMP_.block(CI_ZMP_.rows() / 4, half, CI_ZMP_.rows() / 4, half), CI_ZMP_.block(3 * CI_ZMP_.rows() / 4, half, CI_ZMP_.rows() / 4, half), CI_place_.block(1, half, 1, half), CI_place_.block(3, half, 1, half);
//     ci0_x << ci0_ZMP_.head(CI_ZMP_.rows() / 4), ci0_ZMP_.segment(CI_ZMP_.rows() / 2, CI_ZMP_.rows() / 4), ci0_place_(0), ci0_place_(2);
//     ci0_y << ci0_ZMP_.segment(CI_ZMP_.rows() / 4, CI_ZMP_.rows() / 4), ci0_ZMP_.segment(3 * CI_ZMP_.rows() / 4, CI_ZMP_.rows() / 4), ci0_place_(1), ci0_place_(3);
//     solve_quadprog(Qk_x, pk_x, empty_matrix, empty_vector, -CI_x.transpose(), ci0_x, X_x);
//     solve_quadprog(Qk_y, pk_y, empty_matrix, empty_vector, -CI_y.transpose(), ci0_y, X_y);
//     X << X_x, X_y;

//     cout.setf(ios::fixed, ios::floatfield);
//     cout.precision(4);
//     cout << "QK_e:\n" << Qk_x -Qk_y << endl;
//     cout << "pk_e: " << (pk_x - pk_y).transpose() << endl;
//     cout << "CI_e:\n" << CI_x -CI_y << endl;
//     cout << "ci0_e: " << (ci0_x - ci0_y).transpose() << endl;

    Vector3d com_x(current_state.com.pos[0], current_state.com.vel[0], current_state.com.acc[0]);
    Vector3d com_y(current_state.com.pos[1], current_state.com.vel[1], current_state.com.acc[1]);
    VectorXd dddx = X.head(preview_knot_num_);
    VectorXd dddy = X.segment(preview_knot_num_ + preview_walking_step_, preview_knot_num_);
    VectorXd ddx = Pas_ * com_x + Pau_ * dddx;
    VectorXd ddy = Pas_ * com_y + Pau_ * dddy;
    VectorXd dx = Pvs_ * com_x + Pvu_ * dddx;
    VectorXd dy = Pvs_ * com_y + Pvu_ * dddy;
    VectorXd x = Pps_ * com_x + Ppu_ * dddx;
    VectorXd y = Pps_ * com_y + Ppu_ * dddy;
    VectorXd zmp_x = Pzs_ * com_x + Pzu_ * dddx;
    VectorXd zmp_y = Pzs_ * com_y + Pzu_ * dddy;

    next_state.com.pos << x[0], y[0], model_.GetModel().com_height;
    next_state.com.vel << dx[0], dy[0], 0;
    next_state.com.acc << ddx[0], ddy[0], 0;
    next_state.pelvis = next_state.com;

    // test for zmp
    next_state.pelvis.pos[0] = zmp_x[0];
    next_state.pelvis.pos[1] = zmp_y[0];

    next_state.contact_state = state_machine_.Next();
    next_state.current_phase_knot_num = state_machine_.GetCurrentPhaseKnotNumber();
    double ground_clearness = 0.15;
    unsigned int transfer_knot = 0;
    unsigned int xy_duration = current_state_.current_phase_knot_num - transfer_knot;
    unsigned int mid_phase = (state_machine_.GetSingleSupportKnotNumber() - transfer_knot) / 2;
    unsigned int z_duration;
    double frequency = 10;
    // TODO clean the dirty double support phase
//     if (current_state_.contact_state != next_state.contact_state && current_state_.contact_state != kDoubleSupport) {
//         current_state.contact_state = kDoubleSupport;
//         next_state.lsole = current_state_.lsole;
//         next_state.rsole = current_state_.rsole;
//     } else {
    switch (current_state_.contact_state) {
    case StateMachine::kLeftSupport: {
        next_state.lsole = current_state_.lsole;
//         next_state.rsole.pos << X[preview_knot_num_], X[preview_knot_num_ * 2 + preview_walking_step_], 0;
//         next_state.rsole.vel << dx[current_phase_knot_num_ - 1], dy[current_phase_knot_num_ - 1], 0;
//         next_state.rsole.acc << ddx[current_phase_knot_num_ - 1], ddy[current_phase_knot_num_ - 1], 0;

        Vector3d current_xstate(current_state_.rsole.pos[0], current_state_.rsole.vel[0], current_state_.rsole.acc[0]);
        Vector3d current_ystate(current_state_.rsole.pos[1], current_state_.rsole.vel[1], current_state_.rsole.acc[1]);
        Vector3d current_zstate(current_state_.rsole.pos[2], current_state_.rsole.vel[2], current_state_.rsole.acc[2]);
        Vector3d next_xstate(X[preview_knot_num_], 0, 0); // -dx[current_phase_knot_num_ - 1], -ddx[current_phase_knot_num_ - 1]);
        Vector3d next_ystate(X[preview_knot_num_ * 2 + preview_walking_step_], 0, 0); // -dy[current_phase_knot_num_ - 1], -ddy[current_phase_knot_num_ - 1]);
        Vector3d next_zstate;
//             unsigned int transfer_knot = 1;
//             unsigned int xy_duration = current_state_.current_phase_knot_num - transfer_knot;
//             unsigned int mid_phase = (state_machine_.GetSingleSupportKnotNumber() - transfer_knot) / 2;
//             unsigned int z_duration;
        if (xy_duration > mid_phase) {
            z_duration = xy_duration - mid_phase;
            next_zstate << ground_clearness, 0, 0;
        } else {
            z_duration = xy_duration;
            next_zstate << 0, 0, 0;
        }
        QuinticPolynomial(current_xstate, next_xstate, xy_duration*model_.GetModel().knot_time, model_.GetModel().knot_time, next_xstate);
        QuinticPolynomial(current_ystate, next_ystate, xy_duration*model_.GetModel().knot_time, model_.GetModel().knot_time, next_ystate);
        QuinticPolynomial(current_zstate, next_zstate, z_duration*model_.GetModel().knot_time, model_.GetModel().knot_time, next_zstate);
        next_state.rsole.pos << next_xstate[0], next_ystate[0], next_zstate[0];
        next_state.rsole.vel << next_xstate[1], next_ystate[1], next_zstate[1];
        next_state.rsole.acc << next_xstate[2], next_ystate[2], next_zstate[2];
        break;
    }
    case StateMachine::kRightSupport: {
        next_state.rsole = current_state_.rsole;
//         next_state.lsole.pos << X[preview_knot_num_], X[preview_knot_num_ * 2 + preview_walking_step_], 0;
//         next_state.lsole.vel << dx[current_phase_knot_num_ - 1], dy[current_phase_knot_num_ - 1], 0;
//         next_state.lsole.acc << ddx[current_phase_knot_num_ - 1], ddy[current_phase_knot_num_ - 1], 0;

        Vector3d current_xstate(current_state_.lsole.pos[0], current_state_.lsole.vel[0], current_state_.lsole.acc[0]);
        Vector3d current_ystate(current_state_.lsole.pos[1], current_state_.lsole.vel[1], current_state_.lsole.acc[1]);
        Vector3d current_zstate(current_state_.lsole.pos[2], current_state_.lsole.vel[2], current_state_.lsole.acc[2]);
        Vector3d next_xstate(X[preview_knot_num_], 0, 0); //-dx[current_phase_knot_num_ - 1], -ddx[current_phase_knot_num_ - 1]);
        Vector3d next_ystate(X[preview_knot_num_ * 2 + preview_walking_step_], 0, 0); // -dy[current_phase_knot_num_ - 1], -ddy[current_phase_knot_num_ - 1]);
        Vector3d next_zstate;
//             unsigned int xy_duration = current_state_.current_phase_knot_num;
//             unsigned int mid_phase = state_machine_.GetSingleSupportKnotNumber() / 2;
//             unsigned int z_duration;
        if (xy_duration > mid_phase) {
            z_duration = xy_duration - mid_phase;
            next_zstate << ground_clearness, 0, 0;
        } else {
            z_duration = xy_duration;
            next_zstate << 0, 0, 0;
        }
        QuinticPolynomial(current_xstate, next_xstate, xy_duration*model_.GetModel().knot_time, model_.GetModel().knot_time, next_xstate);
        QuinticPolynomial(current_ystate, next_ystate, xy_duration*model_.GetModel().knot_time, model_.GetModel().knot_time, next_ystate);
        QuinticPolynomial(current_zstate, next_zstate, z_duration*model_.GetModel().knot_time, model_.GetModel().knot_time, next_zstate);
        next_state.lsole.pos << next_xstate[0], next_ystate[0], next_zstate[0];
        next_state.lsole.vel << next_xstate[1], next_ystate[1], next_zstate[1];
        next_state.lsole.acc << next_xstate[2], next_ystate[2], next_zstate[2];
        break;
    }
    default: {
        next_state.lsole = current_state_.lsole;
        next_state.rsole = current_state_.rsole;
    }
    }
//     }

}

unsigned int MPC::GetSingleSupportKnotNumber()
{
    return state_machine_.GetSingleSupportKnotNumber();
}

unsigned int MPC::GetOptimizationVariableNum()
{
    return optimization_variable_num_;
}

unsigned int MPC::GetDoubleSupportKnotNumber()
{
    return state_machine_.GetDoubleSupportKnotNumber();
}

unsigned int MPC::GetPreviewWalkingStep()
{
    return preview_walking_step_;
}

unsigned int MPC::GetPreviousSupportLeg()
{
    return state_machine_.GetPreviousSupportLeg();
}

void MPC::GetWeight(double &jerk_weight, double &velocity_weight, double &zmp_weight)
{
    jerk_weight = alpha_;
    velocity_weight = beta_;
    zmp_weight = gamma_;
}

double MPC::GetFootSpan()
{
    return foot_span_;
}


void MPC::SetSingleSupportKnotNumber(const unsigned int &num)
{
    state_machine_.SetSingleSupportKnotNumber(num);
}

void MPC::SetDoubleSupportKnotNumber(const unsigned int &num)
{
    state_machine_.SetDoubleSupportKnotNumber(num);
}

void MPC::SetPreviewWalkingStep(const unsigned int &num)
{
    preview_walking_step_ = num;
}

void MPC::SetPreviouSupportLeg(const unsigned int &previous_support_leg)
{
    if (previous_support_leg == StateMachine::kLeftSupport)
        state_machine_.SetPreviouSupportLeg(StateMachine::kLeftSupport);
    else
        state_machine_.SetPreviouSupportLeg(StateMachine::kRightSupport);
}

void MPC::InversePreviouSupportLeg()
{
    if (state_machine_.GetPreviousSupportLeg() == StateMachine::kLeftSupport)
        state_machine_.SetPreviouSupportLeg(StateMachine::kRightSupport);
    else
        state_machine_.SetPreviouSupportLeg(StateMachine::kLeftSupport);
}

void MPC::SetWeight(const double &jerk_weight, const double &velocity_weight, const double &zmp_weight)
{
    alpha_ = jerk_weight;
    beta_ = velocity_weight;
    gamma_ = zmp_weight;
}

void MPC::SetFootSpan(const double &foot_span)
{
    foot_span_ = foot_span;
}

void MPC::SetFootSize(const double &foot_half_length, const double &foot_half_width)
{
    foot_half_length_ = foot_half_length;
    foot_half_width_ = foot_half_width;
}

}
