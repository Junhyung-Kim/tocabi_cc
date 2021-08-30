#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/algorithm/model.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"
#include "pinocchio/algorithm/compute-all-terms.hpp"
#include "pinocchio/algorithm/centroidal.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include <ct/core/core.h>
#include <ct/optcon/optcon.h>
#include <ct/optcon/constraint/constraint-impl.h>
#include <ct/optcon/solver/lqp/HPIPMInterface-impl.hpp>
#include <ct/optcon/solver/lqp/HPIPMInterface.hpp>
#include <ct/optcon/nlp/Nlp>
#include "cc.h"

using namespace TOCABI;
using namespace pinocchio;
using namespace ct;
using namespace ct::core;
using namespace ct::optcon;
using std::shared_ptr;

pinocchio::Model model;
pinocchio::Data model_data;


//MPC
const size_t state_dim = 5;
const size_t control_dim = 3;

std::shared_ptr<CostFunctionQuadratic<state_dim, control_dim>> createflywheelCostFunction(const core::StateVector<state_dim>& x_final);

class flywheelLinear : public core::LinearSystem<state_dim, control_dim>
{
        public:

            state_matrix_t A_;
            state_control_matrix_t B_;

            const state_matrix_t& getDerivativeState(const core::StateVector<state_dim>& x,
                const core::ControlVector<control_dim>& u,
                const double t = 0.0) override
            {
          //      A_<< 0, 1, 0, 0, 0,   w_, 0, -w_, 0, 0,    0, 0, 0, 0, 0,    0, 0, 0, 0, 1,   0, 0, 0, 0, 0;
                return A_;
            }

            const state_control_matrix_t& getDerivativeControl(const core::StateVector<state_dim>& x,
                const core::ControlVector<control_dim>& u,
                const double t = 0.0) override
            {
            //    B_ << 0, 0, 0,   0, 1/(m*G), 0,   1, 0, 0,    0,0,0,    0,1,0;
            
                return B_;
            }

            flywheelLinear* clone() const override { return new flywheelLinear(); };
};


CustomController::CustomController(RobotData &rd) : rd_(rd) //, wbc_(dc.wbc_)
{
    ControlVal_.setZero();    
    pinocchio::urdf::buildModel("/home/jhk/catkin_ws/src/dyros_tocabi/tocabi_description/robots/dyros_tocabi.urdf", model);
    pinocchio::Data data(model);
    model_data = data;
    q = randomConfiguration(model);
    qdot = Eigen::VectorXd::Zero(model.nv);
    qddot = Eigen::VectorXd::Zero(model.nv);
    qdot_ = Eigen::VectorXd::Zero(model.nv);
    qddot_ = Eigen::VectorXd::Zero(model.nv);

    CMM = pinocchio::computeCentroidalMap(model, data, q);
    pinocchio::crba(model, data, q);
    pinocchio::computeCoriolisMatrix(model, data, q, qdot);
    pinocchio::rnea(model, data, q, qdot_, qddot_);
}

Eigen::VectorQd CustomController::getControl()
{
    return ControlVal_;
}

// void CustomController::taskCommandToCC(TaskCommand tc_)
// {
//     tc = tc_;
// }

void CustomController::computeSlow()
{
    if (rd_.tc_.mode == 10)
    {
        if (rd_.tc_init)
        {
            //Initialize settings for Task Control! 

            rd_.tc_init = false;
            std::cout<<"cc mode 11"<<std::endl;

            //rd_.link_[COM_id].x_desired = rd_.link_[COM_id].x_init;
        }

        WBC::SetContact(rd_, 1, 1);

        rd_.J_task.setZero(9, MODEL_DOF_VIRTUAL);
        rd_.J_task.block(0, 0, 6, MODEL_DOF_VIRTUAL) = rd_.link_[COM_id].Jac();
        rd_.J_task.block(6, 0, 3, MODEL_DOF_VIRTUAL) = rd_.link_[Upper_Body].Jac().block(3, 0, 3, MODEL_DOF_VIRTUAL);

        rd_.link_[COM_id].x_desired = rd_.tc_.ratio * rd_.link_[Left_Foot].x_init + (1 - rd_.tc_.ratio) * rd_.link_[Right_Foot].x_init;
        rd_.link_[COM_id].x_desired(2) = rd_.tc_.height;

        rd_.link_[Upper_Body].rot_desired = DyrosMath::rotateWithX(rd_.tc_.roll) * DyrosMath::rotateWithY(rd_.tc_.pitch) * DyrosMath::rotateWithZ(rd_.tc_.yaw + rd_.link_[Pelvis].yaw_init);

        Eigen::VectorXd fstar;
        rd_.link_[COM_id].SetTrajectoryQuintic(rd_.control_time_, rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time);

        rd_.link_[Upper_Body].SetTrajectoryRotation(rd_.control_time_, rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time);

        fstar.setZero(9);
        fstar.segment(0, 6) = WBC::GetFstar6d(rd_.link_[COM_id]);
        fstar.segment(6, 3) = WBC::GetFstarRot(rd_.link_[Upper_Body]);

        rd_.torque_desired = WBC::ContactForceRedistributionTorque(rd_, WBC::GravityCompensationTorque(rd_) + WBC::TaskControlTorque(rd_, fstar));
    }
    else if(rd_.tc_.mode == 11)
    {
        if (rd_.tc_.walking_enable == 1.0)
        {
            q = rd_.q_;
            qdot = rd_.q_dot_;
            
            //Pinocchio model 
            CMM = pinocchio::computeCentroidalMap(model, model_data, q);
            pinocchio::crba(model, model_data, q);
            pinocchio::computeCoriolisMatrix(model, model_data, q, qdot);
            pinocchio::rnea(model, model_data, q, qdot_, qddot_);

            TorqueGrav = WBC::GravityCompensationTorque(rd_);
        //ContactForceRedistributionTorqueWalking(rd_, )
            /*if (walking_tickc > 0)
            {
                if (contactModec == 1.0)
                {
                    rd_.ee_[0].contact = 1.0;
                    rd_.ee_[1].contact = 1.0;
                }
                else if (contactModec == 2.0)
                {
                    rd_.ee_[0].contact = 1.0;
                    rd_.ee_[1].contact = 0.0;
                }
                else
                {
                    rd_.ee_[0].contact = 0.0;
                    rd_.ee_[1].contact = 1.0;
                }
            }
            else
            {
                if (contactModec == 1.0)
                {
                    rd_.ee_[0].contact = 1.0;
                    rd_.ee_[1].contact = 1.0;
                }
                else if (contactModec == 2.0)
                {
                    rd_.ee_[0].contact = 1.0;
                    rd_.ee_[1].contact = 0.0;
                }
                else
                {
                    rd_.ee_[0].contact = 0.0;
                    rd_.ee_[1].contact = 1.0;
                }
            }*/
        }

    }
}

void CustomController::computeFast()
{
    // if (tc.mode == 10)
    // {
    // }
    // else if (tc.mode == 11)
    // {
    // }
}

void CustomController::computePlanner()
{
    if (rd_.tc_.mode == 10)
    {
    

    }
    else if(rd_.tc_.mode == 11)
    {
        if (rd_.tc_.walking_enable == 1.0)
        {

        }
    }
}

void CustomController::copyRobotData(RobotData &rd_l)
{
    std::memcpy(&rd_cc_, &rd_l, sizeof(RobotData));
}

void CustomController::walkingCompute()
{
    getRobotInitState();
    footStepGenerator();

    if (walking_tick == 0)
    {
       // setCpPosition();
       // cpReferencePatternGeneration();
       // cptoComTrajectory();
    }
}

void CustomController::getRobotInitState()
{
    if (walking_tick == 0)
    {  
        contactMode = 1.0;
        RF_float_init.translation() = rd_.link_[Right_Foot].xpos;
        RFx_float_init.translation() = rd_.link_[Right_Foot].xipos;
        RF_float_init.linear() = rd_.link_[Right_Foot].rotm;
        LF_float_init.translation() = rd_.link_[Left_Foot].xpos;
        LF_float_init.linear() = rd_.link_[Left_Foot].rotm;

        COM_float_init.translation() = rd_.link_[COM_id].xpos;
        COM_float_init.linear() = rd_.link_[COM_id].rotm;

        PELV_float_init.translation() = rd_.link_[Pelvis].xipos;
        PELV_float_init.linear() = rd_.link_[Pelvis].rotm;

        PELV_float_init1.translation() = rd_.link_[Pelvis].xpos;
        PELV_float_init1.linear() = rd_.link_[Pelvis].rotm;

        HLR_float_init.translation() = rd_.link_[4].xpos;
        HLR_float_init.linear() = rd_.link_[4].rotm;

        HRR_float_init.translation() = rd_.link_[10].xpos;
        HRR_float_init.linear() = rd_.link_[10].rotm;

        Eigen::Isometry3d temp;
        temp.linear() = PELV_float_init.linear();
        temp.translation().setZero();
        foot_distance = temp.inverse() * (LF_float_init.translation() - RF_float_init.translation());
    }
}

void CustomController::footStepGenerator()
{
    if (target(0) == 0 && target(1) == 0 && target(3) == 0)
    {
        foot_step.resize(desired_foot_step_num, 7);
        foot_step.setZero();
        for (int i = 0; i < desired_foot_step_num / 2; i++)
        {
            if (foot_step_dir != 1)
            {
                foot_step(2 * i, 0) = (LF_float_init).translation()(0);
                foot_step(2 * i, 1) = (LF_float_init).translation()(1);
                foot_step(2 * i, 2) = 0.0;
                foot_step(2 * i, 6) = 0.5 + 0.5 * foot_step_dir;

                foot_step(2 * i + 1, 0) = (RF_float_init).translation()(0);
                foot_step(2 * i + 1, 1) = (RF_float_init).translation()(1);
                foot_step(2 * i + 1, 2) = 0.0;
                foot_step(2 * i + 1, 6) = 0.5 + 0.5 * (-1) * foot_step_dir;
            }
            else
            {
                foot_step(2 * i, 0) = (RF_float_init).translation()(0);
                foot_step(2 * i, 1) = (RF_float_init).translation()(1);
                foot_step(2 * i, 2) = 0.0;
                foot_step(2 * i, 6) = 0.5 + 0.5 * foot_step_dir;

                foot_step(2 * i + 1, 0) = (LF_float_init).translation()(0);
                foot_step(2 * i + 1, 1) = (LF_float_init).translation()(1);
                foot_step(2 * i + 1, 2) = 0.0;
                foot_step(2 * i + 1, 6) = 0.5 + 0.5 * (-1) * foot_step_dir;
            }
        }
    }
    else
    {
        footStepTotal();
    }
    
    total_step_num = foot_step.col(1).size();
}

void CustomController::footStepTotal()
{
    double initial_rot, initial_drot, final_rot, final_drot;

    initial_rot = atan2(target(1), target(0));
    if (initial_rot > 0.0)
        initial_drot = 10.0 * DEG2RAD;
    else
        initial_drot = -10.0 * DEG2RAD;

    unsigned int init_totalstep_num = initial_rot / initial_drot;
    double init_residual_angle = initial_rot - init_totalstep_num * initial_drot;

    final_rot = target(3) - initial_rot;
    if (final_rot > 0.0)
        final_drot = 10.0 * DEG2RAD;
    else
        final_drot = -10.0 * DEG2RAD;

    unsigned int final_total_step_num = final_rot / final_drot;
    double final_residual_angle = final_rot - final_total_step_num * final_drot;
    double l = sqrt(target(0) * target(0) + target(1) * target(1));
    double dlength = step_length_x; //거리 추가 필요
    int middle_total_step_num = l / dlength;
    double middle_residual_length = l - middle_total_step_num * dlength;
    int numberOfFootstep;
    int del_size = 1;
    numberOfFootstep = init_totalstep_num * del_size + middle_total_step_num * del_size + final_total_step_num * del_size;

    if (init_totalstep_num != 0 || abs(init_residual_angle) >= 0.0001)
    {
        if (init_totalstep_num % 2 == 0)
            numberOfFootstep = numberOfFootstep + 2;
        else
        {
            if (abs(init_residual_angle) >= 0.0001)
                numberOfFootstep = numberOfFootstep + 3;
            else
                numberOfFootstep = numberOfFootstep + 1;
        }
    }

    if (middle_total_step_num != 0 || abs(middle_residual_length) >= 0.0001)
    {
        if (middle_total_step_num % 2 == 0)
            numberOfFootstep = numberOfFootstep + 2;
        else
        {
            if (abs(middle_residual_length) >= 0.0001)
                numberOfFootstep = numberOfFootstep + 3;
            else
                numberOfFootstep = numberOfFootstep + 1;
        }
    }

    if (final_total_step_num != 0 || abs(final_residual_angle) >= 0.0001)
    {
        if (abs(final_residual_angle) >= 0.0001)
            numberOfFootstep = numberOfFootstep + 2;
        else
            numberOfFootstep = numberOfFootstep + 1;
    }

    numberOfFootstep = numberOfFootstep + 1;
    foot_step.resize(numberOfFootstep, 7);
    foot_step.setZero();

    int index = 0;

    int temp, temp2, temp3, is_right;

    if (foot_step_dir == 1)
        is_right = 1;
    else
        is_right = -1;

    temp = -is_right; //right foot will be first swingfoot
    temp2 = -is_right;
    temp3 = is_right;

    if (init_totalstep_num != 0 || abs(init_residual_angle) >= 0.0001)
    {
        for (int i = 0; i < init_totalstep_num; i++)
        {
            temp *= -1;
            foot_step(index, 0) = temp * foot_distance(1) / 2.0 * sin((i + 1) * initial_drot);
            foot_step(index, 1) = -temp * foot_distance(1) / 2.0 * cos((i + 1) * initial_drot);
            foot_step(index, 5) = (i + 1) * initial_drot;
            foot_step(index, 6) = 0.5 + 0.5 * temp;
            index++;
        }

        if (temp == is_right)
        {
            if (abs(init_residual_angle) >= 0.0001)
            {
                temp *= -1;

                foot_step(index, 0) = temp * foot_distance(1) / 2.0 * sin((init_totalstep_num)*initial_drot + init_residual_angle);
                foot_step(index, 1) = -temp * foot_distance(1) / 2.0 * cos((init_totalstep_num)*initial_drot + init_residual_angle);
                foot_step(index, 5) = (init_totalstep_num)*initial_drot + init_residual_angle;
                foot_step(index, 6) = 0.5 + 0.5 * temp;
                index++;

                temp *= -1;

                foot_step(index, 0) = temp * foot_distance(1) / 2.0 * sin((init_totalstep_num)*initial_drot + init_residual_angle);
                foot_step(index, 1) = -temp * foot_distance(1) / 2.0 * cos((init_totalstep_num)*initial_drot + init_residual_angle);
                foot_step(index, 5) = (init_totalstep_num)*initial_drot + init_residual_angle;
                foot_step(index, 6) = 0.5 + 0.5 * temp;
                index++;

                temp *= -1;

                foot_step(index, 0) = temp * foot_distance(1) / 2.0 * sin((init_totalstep_num)*initial_drot + init_residual_angle);
                foot_step(index, 1) = -temp * foot_distance(1) / 2.0 * cos((init_totalstep_num)*initial_drot + init_residual_angle);
                foot_step(index, 5) = (init_totalstep_num)*initial_drot + init_residual_angle;
                foot_step(index, 6) = 0.5 + 0.5 * temp;
                index++;
            }
            else
            {
                temp *= -1;

                foot_step(index, 0) = temp * foot_distance(1) / 2.0 * sin((init_totalstep_num)*initial_drot + init_residual_angle);
                foot_step(index, 1) = -temp * foot_distance(1) / 2.0 * cos((init_totalstep_num)*initial_drot + init_residual_angle);
                foot_step(index, 5) = (init_totalstep_num)*initial_drot + init_residual_angle;
                foot_step(index, 6) = 0.5 + 0.5 * temp;
                index++;
            }
        }
        else if (temp == -is_right)
        {
            temp *= -1;

            foot_step(index, 0) = temp * foot_distance(1) / 2.0 * sin((init_totalstep_num)*initial_drot + init_residual_angle);
            foot_step(index, 1) = -temp * foot_distance(1) / 2.0 * cos((init_totalstep_num)*initial_drot + init_residual_angle);
            foot_step(index, 5) = (init_totalstep_num)*initial_drot + init_residual_angle;
            foot_step(index, 6) = 0.5 + 0.5 * temp;
            index++;

            temp *= -1;

            foot_step(index, 0) = temp * foot_distance(1) / 2.0 * sin((init_totalstep_num)*initial_drot + init_residual_angle);
            foot_step(index, 1) = -temp * foot_distance(1) / 2.0 * cos((init_totalstep_num)*initial_drot + init_residual_angle);
            foot_step(index, 5) = (init_totalstep_num)*initial_drot + init_residual_angle;
            foot_step(index, 6) = 0.5 + 0.5 * temp;
            index++;
        }
    }

    if (middle_total_step_num != 0 || abs(middle_residual_length) >= 0.0001)
    {
        temp2 *= -1;

        foot_step(index, 0) = 0.0;
        foot_step(index, 1) = -temp2 * (foot_distance(1) / 2.0);
        foot_step(index, 5) = 0.0;
        foot_step(index, 6) = 0.5 + 0.5 * temp2;

        index++;

        for (int i = 0; i < middle_total_step_num; i++)
        {
            temp2 *= -1;

            foot_step(index, 0) = cos(initial_rot) * (dlength * (i + 1)) + temp2 * sin(initial_rot) * (foot_distance(1) / 2.0);
            foot_step(index, 1) = sin(initial_rot) * (dlength * (i + 1)) - temp2 * cos(initial_rot) * (foot_distance(1) / 2.0);
            foot_step(index, 5) = initial_rot;
            foot_step(index, 6) = 0.5 + 0.5 * temp2;
            index++;
        }

        if (temp2 == -is_right)
        {
            if (abs(middle_residual_length) >= 0.0001)
            {
                temp2 *= -1;

                foot_step(index, 0) = cos(initial_rot) * (dlength * (middle_total_step_num) + middle_residual_length) + temp2 * sin(initial_rot) * (foot_distance(1) / 2.0);
                foot_step(index, 1) = sin(initial_rot) * (dlength * (middle_total_step_num) + middle_residual_length) - temp2 * cos(initial_rot) * (foot_distance(1) / 2.0);
                foot_step(index, 5) = initial_rot;
                foot_step(index, 6) = 0.5 + 0.5 * temp2;
                index++;

                temp2 *= -1;

                foot_step(index, 0) = cos(initial_rot) * (dlength * (middle_total_step_num) + middle_residual_length) + temp2 * sin(initial_rot) * (foot_distance(1) / 2.0);
                foot_step(index, 1) = sin(initial_rot) * (dlength * (middle_total_step_num) + middle_residual_length) - temp2 * cos(initial_rot) * (foot_distance(1) / 2.0);
                foot_step(index, 5) = initial_rot;
                foot_step(index, 6) = 0.5 + 0.5 * temp2;
                index++;

                temp2 *= -1;

                foot_step(index, 0) = cos(initial_rot) * (dlength * (middle_total_step_num) + middle_residual_length) + temp2 * sin(initial_rot) * (foot_distance(1) / 2.0);
                foot_step(index, 1) = sin(initial_rot) * (dlength * (middle_total_step_num) + middle_residual_length) - temp2 * cos(initial_rot) * (foot_distance(1) / 2.0);
                foot_step(index, 5) = initial_rot;
                foot_step(index, 6) = 0.5 + 0.5 * temp2;
                index++;
            }
            else
            {
                temp2 *= -1;

                foot_step(index, 0) = cos(initial_rot) * (dlength * (middle_total_step_num) + middle_residual_length) + temp2 * sin(initial_rot) * (foot_distance(1) / 2.0);
                foot_step(index, 1) = sin(initial_rot) * (dlength * (middle_total_step_num) + middle_residual_length) - temp2 * cos(initial_rot) * (foot_distance(1) / 2.0);
                foot_step(index, 5) = initial_rot;
                foot_step(index, 6) = 0.5 + 0.5 * temp2;
                index++;
            }
        }
        else if (temp2 == is_right)
        {
            temp2 *= -1;

            foot_step(index, 0) = cos(initial_rot) * (dlength * (middle_total_step_num) + middle_residual_length) + temp2 * sin(initial_rot) * (foot_distance(1) / 2.0);
            foot_step(index, 1) = sin(initial_rot) * (dlength * (middle_total_step_num) + middle_residual_length) - temp2 * cos(initial_rot) * (foot_distance(1) / 2.0);
            foot_step(index, 5) = initial_rot;
            foot_step(index, 6) = 0.5 + 0.5 * temp2;
            index++;

            temp2 *= -1;

            foot_step(index, 0) = cos(initial_rot) * (dlength * (middle_total_step_num) + middle_residual_length) + temp2 * sin(initial_rot) * (foot_distance(1) / 2.0);
            foot_step(index, 1) = sin(initial_rot) * (dlength * (middle_total_step_num) + middle_residual_length) - temp2 * cos(initial_rot) * (foot_distance(1) / 2.0);
            foot_step(index, 5) = initial_rot;
            foot_step(index, 6) = 0.5 + 0.5 * temp2;
            index++;
        }
    }

    double final_position_x = cos(initial_rot) * (dlength * (middle_total_step_num) + middle_residual_length);
    double final_position_y = sin(initial_rot) * (dlength * (middle_total_step_num) + middle_residual_length);

    if (final_total_step_num != 0 || abs(final_residual_angle) >= 0.0001)
    {
        for (int i = 0; i < final_total_step_num; i++)
        {
            temp3 *= -1;

            foot_step(index, 0) = final_position_x + temp3 * foot_distance(1) / 2.0 * sin((i + 1) * final_drot + initial_rot);
            foot_step(index, 1) = final_position_y - temp3 * foot_distance(1) / 2.0 * cos((i + 1) * final_drot + initial_rot);
            foot_step(index, 5) = (i + 1) * final_drot + initial_rot;
            foot_step(index, 6) = 0.5 + 0.5 * temp3;
            index++;
        }

        if (abs(final_residual_angle) >= 0.0001)
        {
            temp3 *= -1;

            foot_step(index, 0) = final_position_x + temp3 * foot_distance(1) / 2.0 * sin(target(3));
            foot_step(index, 1) = final_position_y - temp3 * foot_distance(1) / 2.0 * cos(target(3));
            foot_step(index, 5) = target(3);
            foot_step(index, 6) = 0.5 + 0.5 * temp3;
            index++;

            temp3 *= -1;

            foot_step(index, 0) = final_position_x + temp3 * foot_distance(1) / 2.0 * sin(target(3));
            foot_step(index, 1) = final_position_y - temp3 * foot_distance(1) / 2.0 * cos(target(3));
            foot_step(index, 5) = target(3);
            foot_step(index, 6) = 0.5 + 0.5 * temp3;
            index++;
        }
        else
        {
            temp3 *= -1;

            foot_step(index, 0) = final_position_x + temp3 * foot_distance(1) / 2.0 * sin(target(3));
            foot_step(index, 1) = final_position_y - temp3 * foot_distance(1) / 2.0 * cos(target(3));
            foot_step(index, 5) = target(3);
            foot_step(index, 6) = 0.5 + 0.5 * temp3;
            index++;
        }
    }

    for (int i = 0; i < numberOfFootstep; i++)
    {
        if (foot_step(i, 6) == 1)
        {
            foot_step(i, 0) = foot_step(i, 0) + (RF_float_init).translation()(0);
            foot_step(i, 1) = (RF_float_init).translation()(1);
        }
        else
        {
            foot_step(i, 0) = foot_step(i, 0) + (LF_float_init).translation()(0);
            foot_step(i, 1) = (LF_float_init).translation()(1);
        }
    }
}

void CustomController::getRobotState()
{
    //////Real Robot Float Frame//////PELV_traejctory_float_id].xipos;
    COM_float_current.linear() = rd_.link_[COM_id].rotm;
    COMV_support_currentV = rd_.link_[COM_id].v;

    if (foot_step(current_step_num, 6) == 0)
    {
        SUF_float_current = RF_float_current;
        SWF_float_current = LF_float_current;
        for (int i = 0; i < 3; i++)
        {
            SUF_float_currentV(i) = SUF_float_current.translation()(i);
            SWF_float_currentV(i) = SWF_float_current.translation()(i);
        }
        for (int i = 0; i < 3; i++)
        {
            SUF_float_currentV(i + 3) = DyrosMath::rot2Euler(SUF_float_current.linear())(i);
            SWF_float_currentV(i + 3) = DyrosMath::rot2Euler(SWF_float_current.linear())(i);
        }
    }
    else
    {
        SUF_float_current = LF_float_current;
        SWF_float_current = RF_float_current;
        for (int i = 0; i < 3; i++)
        {
            SUF_float_currentV(i) = SUF_float_current.translation()(i);
            SWF_float_currentV(i) = SWF_float_current.translation()(i);
        }
        for (int i = 0; i < 3; i++)
        {
            SUF_float_currentV(i + 3) = DyrosMath::rot2Euler(SUF_float_current.linear())(i);
            SWF_float_currentV(i + 3) = DyrosMath::rot2Euler(SWF_float_current.linear())(i);
        }
    }

    //////Real Robot Support Foot Frame//////
    PELV_support_current = DyrosMath::inverseIsometry3d(SUF_float_current) * PELV_float_current;
    RF_support_current = DyrosMath::multiplyIsometry3d(PELV_support_current, RF_float_current);
    LF_support_current = DyrosMath::multiplyIsometry3d(PELV_support_current, LF_float_current);
    COM_support_current = DyrosMath::multiplyIsometry3d(PELV_support_current, COM_float_current);

/*    Ag_leg = rd_.Ag_.block(3, 0, 3, 12);
    Ag_armR = rd_.Ag_.block(3, 25, 3, 8);
    Ag_armL = rd_.Ag_.block(3, 15, 3, 8);
    Ag_waist = rd_.Ag_.block(3, 12, 3, 3);

    Agl_leg = rd_.Ag_.block(0, 0, 3, 12);
    Agl_armR = rd_.Ag_.block(0, 25, 3, 8);
    Agl_armL = rd_.Ag_.block(0, 15, 3, 8);
    Agl_waist = rd_.Ag_.block(0, 12, 3, 3);

    yx_vibm(0) = rd_.ZMP(0);
    yx_vibm(1) = rd_.link_[Pelvis].xipos(0);
    yx_vibm(2) = rd_.link_[Pelvis].v(0);

    yy_vibm(0) = rd_.ZMP(1);
    yy_vibm(1) = rd_.link_[COM_id].xipos(1);
    yy_vibm(2) = rd_.link_[COM_id].v(1);
*/
    calcRobotState();
}

void CustomController::calcRobotState()
{
    
}

void CustomController::setCpPosition()
{
    capturePoint_ox.resize(total_step_num + 3);
    capturePoint_oy.resize(total_step_num + 3);

    zmp_dx.resize(total_step_num + 2);
    zmp_dy.resize(total_step_num + 2);

    b.resize(total_step_num + 2);

    capturePoint_offsetx.resize(total_step_num + 3);
    capturePoint_offsety.resize(total_step_num + 3);

    /////// INITIALIZE //////

    capturePoint_offsetx.setZero();
    capturePoint_offsety.setZero();

    for (int i = 0; i < total_step_num + 2; i++)
    {
        b(i) = exp(lipm_w * t_total / Hz_);
    }

    for (int i = 0; i < total_step_num + 3; i++)
    {
        capturePoint_offsety(i) = 0.00;
        capturePoint_offsety(i) = 0.01;
        //    capturePoint_offsetx(i) = 0.04;
        capturePoint_offsetx(i) = 0.04;
    }

    if (com_control == 0)
    {
        capturePoint_ox(0) = (PELV_float_init).translation()(0);
        capturePoint_oy(0) = (PELV_float_init).translation()(1);
        capturePoint_ox(total_step_num + 1) = (foot_step(total_step_num - 1, 0) + foot_step(total_step_num - 2, 0)) / 2 + capturePoint_offsetx(total_step_num + 1);
        capturePoint_oy(total_step_num + 1) = (LF_float_init).translation()(1) - foot_distance(1) / 2;
        capturePoint_ox(total_step_num + 2) = (foot_step(total_step_num - 1, 0) + foot_step(total_step_num - 2, 0)) / 2 + capturePoint_offsetx(total_step_num + 2);
        capturePoint_oy(total_step_num + 2) = (LF_float_init).translation()(1) - foot_distance(1) / 2;
    }
    else
    {
        capturePoint_ox(0) = (COM_float_init).translation()(0);
        capturePoint_oy(0) = (COM_float_init).translation()(1);
        capturePoint_ox(total_step_num + 1) = (foot_step(total_step_num - 1, 0) + foot_step(total_step_num - 2, 0)) / 2 + capturePoint_offsetx(total_step_num + 1);
        capturePoint_oy(total_step_num + 1) = (LF_float_init).translation()(1) - foot_distance(1) / 2;
        capturePoint_ox(total_step_num + 2) = (foot_step(total_step_num - 1, 0) + foot_step(total_step_num - 2, 0)) / 2 + capturePoint_offsetx(total_step_num + 2);
        capturePoint_oy(total_step_num + 2) = (LF_float_init).translation()(1) - foot_distance(1) / 2;
    }

    for (int i = 0; i < total_step_num; i++)
    {
        if (foot_step(0, 6) == 0) //right support
        {
            if (i == 0)
            {
                if (com_control == 0)
                {
                    capturePoint_ox(1) = (PELV_float_init).translation()(0); // + capturePoint_offsetx(1);
                    capturePoint_oy(1) = (RF_float_init).translation()(1) + capturePoint_offsety(1);
                }
                else
                {
                    capturePoint_ox(1) = (COM_float_init).translation()(0); // + capturePoint_offsetx(1);
                    capturePoint_oy(1) = (RF_float_init).translation()(1) + capturePoint_offsety(1);
                }
            }
            else
            {
                if (i % 2 == 0)
                {
                    capturePoint_ox(i + 1) = foot_step(i - 1, 0) + capturePoint_offsetx(i + 1);
                    capturePoint_oy(i + 1) = foot_step(i - 1, 1) + capturePoint_offsety(i + 1);
                }
                else
                {
                    capturePoint_ox(i + 1) = foot_step(i - 1, 0) + capturePoint_offsetx(i + 1);
                    capturePoint_oy(i + 1) = foot_step(i - 1, 1) - capturePoint_offsety(i + 1);
                }
            }
        }
        else
        {
            if (i == 0)
            {
                if (com_control == 0)
                {
                    capturePoint_ox(1) = (PELV_float_init).translation()(0); // + capturePoint_offsetx(1);
                    capturePoint_oy(1) = (LF_float_init).translation()(1) - capturePoint_offsety(1);
                }
                else
                {
                    capturePoint_ox(1) = (COM_float_init).translation()(0); // + capturePoint_offsetx(1);
                    capturePoint_oy(1) = (LF_float_init).translation()(1) - capturePoint_offsety(1);
                }
            }
            else
            {
                if (i % 2 == 0)
                {
                    capturePoint_ox(i + 1) = foot_step(i - 1, 0) + capturePoint_offsetx(i + 1);
                    capturePoint_oy(i + 1) = foot_step(i - 1, 1) - capturePoint_offsety(i + 1);
                }
                else
                {
                    capturePoint_ox(i + 1) = foot_step(i - 1, 0) + capturePoint_offsetx(i + 1);
                    capturePoint_oy(i + 1) = foot_step(i - 1, 1) + capturePoint_offsety(i + 1);
                }
            }
        }
    }

    for (int i = 0; i < total_step_num + 2; i++)
    {
        zmp_dx(i) = capturePoint_ox(i + 1) / (1 - b(i)) - (b(i) * capturePoint_ox(i)) / (1 - b(i));
        zmp_dy(i) = capturePoint_oy(i + 1) / (1 - b(i)) - (b(i) * capturePoint_oy(i)) / (1 - b(i));
    }    
}