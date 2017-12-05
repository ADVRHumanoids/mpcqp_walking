#include <gtest/gtest.h>
#include <mpcqp_walking/model_preview_control.h>
#include <mpcqp_walking/integrator.h>
#include <mpcqp_walking/abstract_variable.h>
#include <mpcqp_walking/state_machine.h>
#include <boost/shared_ptr.hpp>

namespace {

class testMPC2: public ::testing::Test {
public:
    testMPC2():
        ctrl_loop(0.1), //good value
        com_z(0.5),
        foot_span(0.2),
        sm()
    {
        foot_size<<0.2,0.1;

        robot.reset(new legged_robot::LIPM(ctrl_loop, com_z));

         wpg.reset(new legged_robot::MPC(*robot, foot_span, foot_size[0]/2., foot_size[1]/2., this->sm));

//        wpg.reset(new legged_robot::MPC(com_z, foot_span,
//                                        foot_size[0]/2., foot_size[1]/2.));
    }

    virtual ~testMPC2() {
        logger->FlushAll();

    }

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }

    void fillCoMTwist(const Eigen::Vector2d& twist)
    {
        xy_com_desired_twist.resize(2,wpg->GetOptimizationVariableNum());
        xy_com_desired_twist<<Eigen::MatrixXd::Constant(1,wpg->GetOptimizationVariableNum(), twist[0]),
                              Eigen::MatrixXd::Constant(1,wpg->GetOptimizationVariableNum(), twist[1]);
    }

    void solve(legged_robot::AbstractVariable& new_state, legged_robot::AbstractVariable& current_state)
    {
        wpg->Next(new_state,
                  current_state,
                  xy_com_desired_twist.row(0),
                  xy_com_desired_twist.row(1),
                  DEFAULT_GROUND_CLEARNESS);
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

    XBot::MatLogger::Ptr logger;
};

TEST_F(testMPC2, test_wpg)
{
    this->logger = XBot::MatLogger::getLogger("/tmp/test_wpg2");

    // Here we set thing wrt the world frame
    this->current_state.com.pos<<0.0, 0.0, this->com_z;
    this->current_state.lsole.pos<<0.0, 0.1, 0.0;
    this->current_state.rsole.pos<<0.0, -this->foot_span, 0.0;
    this->current_state.pelvis.pos<<0.0, 0.0, this->com_z;

    this->current_state.contact_state = legged_robot::StateMachine::kDoubleSupport;
    this->current_state.current_phase_knot_num = 10;

    int loops = 100; //10 secs

    int internal_loops = 10;// here we suppose that the internal control loop goes at 100 Hz

    legged_robot::AbstractVariable next_state;
    legged_robot::AbstractVariable references;

    for(unsigned int i = 0; i < loops; ++i)
    {
        Eigen::Vector2d xy_com_desired_twist_instantaneous;
        xy_com_desired_twist_instantaneous<<0.1-i*1e-3, 0.0;
        logger->add("xy_dcom_ref", xy_com_desired_twist_instantaneous);

        fillCoMTwist(xy_com_desired_twist_instantaneous);

        solve(next_state, this->current_state);
        this->current_state.log(this->logger, "current_state");
        next_state.log(this->logger, "next_state");


        legged_robot::Integrator integrator(this->current_state, next_state, this->ctrl_loop, this->ctrl_loop/internal_loops);
        for(unsigned int j = 0; j < internal_loops; ++j)
        {
            references = integrator.TickAndOuput();
            references.log(this->logger, "references");
            //IK
        }

        this->current_state = next_state; //state update


    }

}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
