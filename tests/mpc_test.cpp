#include <gtest/gtest.h>
#include <mpcqp_walking/model_preview_control.h>
#include <boost/shared_ptr.hpp>

namespace {

class testMPC: public ::testing::Test {
public:
    testMPC():
        ctrl_loop(0.1), //good value
        com_z(0.5),
        foot_span(0.1),
        sm()
    {
        foot_size<<0.2,0.1;

        robot.reset(new legged_robot::LIPM(ctrl_loop, com_z));

        wpg.reset(new legged_robot::MPC(
                      *robot, foot_span/2., foot_size[0]/2., foot_size[1]/2., sm));
    }

    virtual ~testMPC() {

    }

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }

    void fillCoMTwist(const Eigen::Vector2d& twist)
    {
        xy_com_desired_twist<<Eigen::VectorXd::Constant(wpg->GetOptimizationVariableNum(), twist[0]),
                              Eigen::VectorXd::Constant(wpg->GetOptimizationVariableNum(), twist[1]);
    }

    void solve(legged_robot::AbstractVariable& new_state, legged_robot::AbstractVariable& current_state)
    {
        new_state = wpg->Next(current_state, xy_com_desired_twist.row(0), xy_com_desired_twist.row(1));
    }

    double ctrl_loop;
    double com_z;
    double foot_span;

    Eigen::Vector2d foot_size;

    boost::shared_ptr<legged_robot::MPC> wpg;
    boost::shared_ptr<legged_robot::LIPM> robot;
    legged_robot::StateMachine sm;

    legged_robot::AbstractVariable current_state;

    Eigen::MatrixXd xy_com_desired_twist;
};

TEST_F(testMPC, test_wpg)
{
    // Here we set thing wrt the world frame
    this->current_state.com.pos<<0.0, 0.0, this->com_z;
    this->current_state.lsole.pos<<0.0, this->foot_span, 0.0;
    this->current_state.rsole.pos<<0.0, -this->foot_span, 0.0;
    this->current_state.pelvis.pos<<0.0, 0.0, this->com_z;

    this->current_state.contact_state = legged_robot::kDoubleSupport;
    this->current_state.current_phase_knot_num = 10;

    int loops = 100; //10 secs

    legged_robot::AbstractVariable references;
    for(unsigned int i = 0; i < loops; ++i)
    {
        Eigen::Vector2d xy_com_desired_twist_instantaneous;
        xy_com_desired_twist_instantaneous<<0.1-i*1e-3, 0.0;

        fillCoMTwist(xy_com_desired_twist_instantaneous);

        solve(references, this->current_state);

        //IK

        this->current_state = references;
    }

}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
