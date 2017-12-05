#include <gtest/gtest.h>
#include <mpcqp_walking/model_preview_control.h>
#include <mpcqp_walking/integrator.h>
#include <mpcqp_walking/abstract_variable.h>
#include <mpcqp_walking/state_machine.h>
#include <boost/shared_ptr.hpp>
#include <XBotInterface/ModelInterface.h>
#include <mpcqp_walking/walker.h>

std::string robotology_root = std::getenv("ROBOTOLOGY_ROOT");
std::string relative_path = "/configs/ADVR_shared/cogimon/configs/config_cogimon.yaml";
std::string _path_to_cfg = robotology_root + relative_path;

namespace {

class testMPC: public ::testing::Test {
public:
    testMPC():
        ctrl_loop(0.1), //good value
        sm()
    {
        _model = XBot::ModelInterface::getModel(_path_to_cfg);

        _model->getCOM(com);
        com_z = com[2];

        Eigen::Affine3d tmp;
        _model->getPose("l_sole", "r_sole", tmp);
        foot_span = fabs(tmp.translation()[1]);

        foot_size<<0.2,0.1;

        robot.reset(new legged_robot::LIPM(ctrl_loop, com_z));
        
         wpg.reset(new legged_robot::MPC(*robot, foot_span, foot_size[0]/2., foot_size[1]/2., this->sm));

//        wpg.reset(new legged_robot::MPC(com_z, foot_span,
//                                        foot_size[0]/2., foot_size[1]/2.));
    }

    virtual ~testMPC() {
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
        wpg->Next(new_state, current_state,
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

    XBot::ModelInterface::Ptr _model;

    Eigen::Vector3d com;
};

TEST_F(testMPC, test_wpg)
{
    this->logger = XBot::MatLogger::getLogger("/tmp/test_wpg");

    // Here we set thing wrt the world frame
    this->current_state.com.pos<<this->com;
    Eigen::Affine3d tmp;
    this->_model->getPose("l_sole", tmp);
    this->current_state.lsole.pos<<tmp.translation();
    this->_model->getPose("r_sole", tmp);
    this->current_state.rsole.pos<<tmp.translation();
    this->_model->getPose("Waist", tmp);
    this->current_state.pelvis.pos<<tmp.translation();

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

bool checkState(const legged_robot::AbstractVariable& A,
                const legged_robot::AbstractVariable& B)
{
    if(!(A.com.pos == B.com.pos)) return false;
    if(!(A.com.euler_pos == B.com.euler_pos)) return false;
    if(!(A.lsole.pos == B.lsole.pos)) return false;
    if(!(A.lsole.euler_pos == B.lsole.euler_pos)) return false;
    if(!(A.rsole.pos == B.rsole.pos)) return false;
    if(!(A.rsole.euler_pos == B.rsole.euler_pos)) return false;
    if(!(A.pelvis.pos == B.pelvis.pos)) return false;
    if(!(A.pelvis.euler_pos == B.pelvis.euler_pos)) return false;

    if(!(A.contact_state == B.contact_state)) return false;
    if(!(A.current_phase_knot_num == B.current_phase_knot_num)) return false;

    return true;
}

TEST_F(testMPC, test_walker)
{
    this->logger = XBot::MatLogger::getLogger("/tmp/test_walker");

    // Here we set thing wrt the world frame
    this->current_state.com.pos<<this->com;
    Eigen::Affine3d tmp;
    this->_model->getPose("l_sole", tmp);
    this->current_state.lsole.pos<<tmp.translation();
    this->_model->getPose("r_sole", tmp);
    this->current_state.rsole.pos<<tmp.translation();
    this->_model->getPose("Waist", tmp);
    this->current_state.pelvis.pos<<tmp.translation();

    this->current_state.contact_state = legged_robot::StateMachine::kDoubleSupport;
    this->current_state.current_phase_knot_num = DEFAULT_CURRENT_PHASE_KNOT_NUM;

    int loops = 100; //10 secs

    int internal_loops = 10;// here we suppose that the internal control loop goes at 100 Hz

    legged_robot::AbstractVariable next_state;
    legged_robot::AbstractVariable references;
    std::vector<legged_robot::AbstractVariable> references_method1;

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
            references_method1.push_back(references);
            //IK
        }

        this->current_state = next_state; //state update

    }



    legged_robot::Walker walker(*(this->_model), this->ctrl_loop,
                                DEFAULT_SINGLE_SUPPORT_KNOT_NUM*this->ctrl_loop,
                                DEFAULT_DOUBLE_SUPPORT_KNOT_NUM*this->ctrl_loop,
                                this->foot_size,
                                "l_sole", "r_sole", "Waist");

    legged_robot::AbstractVariable next_state2;
    legged_robot::AbstractVariable references2;
    std::vector<legged_robot::AbstractVariable> references_method2;

    for(unsigned int i = 0; i < loops; ++i)
    {
        Eigen::Vector2d xy_com_desired_twist_instantaneous;
        xy_com_desired_twist_instantaneous<<0.1-i*1e-3, 0.0;
        logger->add("xy_dcom_ref", xy_com_desired_twist_instantaneous);

        walker.setReference(xy_com_desired_twist_instantaneous);

        walker.solve(next_state2);
        walker.log(this->logger, "walker");
        next_state.log(this->logger, "next_state");


        legged_robot::Integrator integrator(walker.getCurrentState(), next_state2, this->ctrl_loop, this->ctrl_loop/internal_loops);
        for(unsigned int j = 0; j < internal_loops; ++j)
        {
            references2 = integrator.TickAndOuput();
            references2.log(this->logger, "references");
            references_method2.push_back(references2);
            //IK
        }

        walker.setCurrentState(next_state2);
    }

    EXPECT_EQ(references_method1.size(), references_method2.size());

    for(unsigned int i = 0; i < references_method1.size(); ++i)
        EXPECT_TRUE(checkState(references_method1[i], references_method2[i]));


}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
