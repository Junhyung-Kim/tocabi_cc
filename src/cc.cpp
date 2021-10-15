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

#include "wholebody_functions.h"
#include "cc.h"

using namespace TOCABI;
using namespace pinocchio;
using std::shared_ptr;

pinocchio::Model model;
pinocchio::Data model_data;

CustomController::CustomController(RobotData &rd) : rd_(rd) //, wbc_(dc.wbc_)
{
    for (int i = 0; i < 2; i++)
    {
        file[i].open(FILE_NAMES[i].c_str(), std::ios_base::out);
    }

    ControlVal_.setZero();
    pinocchio::urdf::buildModel("/home/jhk/catkin_ws/src/dyros_tocabi_v2/tocabi_description/robots/dyros_tocabi_with_redhands.urdf", model);
    pinocchio::Data data(model);
    model_data = data;
    q_ = randomConfiguration(model);
    qdot = Eigen::VectorXd::Zero(model.nv);
    qddot = Eigen::VectorXd::Zero(model.nv);
    qdot_ = Eigen::VectorXd::Zero(model.nv);
    qddot_ = Eigen::VectorXd::Zero(model.nv);

    CMM = pinocchio::computeCentroidalMap(model, data, q_);
    pinocchio::crba(model, data, q_);
    pinocchio::computeCoriolisMatrix(model, data, q_, qdot);
    pinocchio::rnea(model, data, q_, qdot_, qddot_);

    mpc_on = false;
    wlk_on = false;
    velEst_f = false;

    mpcVariableInit();
    std::cout << "Custom Controller Init" << std::endl;
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
            std::cout << "cc mode 11" << std::endl;

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
    else if (rd_.tc_.mode == 11)
    {
        TorqueContact.setZero();
        rd_.torque_desired_walk.setZero();

        if (rd_.tc_.walking_enable == 1.0)
        {
            if (rd_.tc_init)
            {
                //Initialize settings for Task Control!
                rd_.tc_init = false;
                rd_.q_desired = rd_.q_;
            }

            //Pinocchio model
            CMM = pinocchio::computeCentroidalMap(model, model_data, rd_.q_);
            pinocchio::crba(model, model_data, rd_.q_);
            pinocchio::computeCoriolisMatrix(model, model_data, rd_.q_, qdot);
            pinocchio::rnea(model, model_data, rd_.q_, qdot_, qddot_);

            Cor_ = model_data.C;
            G_ = model_data.tau;

            jointVelocityEstimate();
            velEst_f = true;

            Vector12d fc_redis;
            double fc_ratio = 0.000;
            fc_redis.setZero();

            if (walking_tick >= 1 && wlk_on == true && current_step_num != total_step_num)
            {
                if (contactMode == 1.0)
                {
                    rd_.ee_[0].contact = 1.0;
                    rd_.ee_[1].contact = 1.0;
                }
                else if (contactMode == 2.0)
                {
                    rd_.ee_[0].contact = 1.0;
                    rd_.ee_[1].contact = 0.0;
                }
                else
                {
                    rd_.ee_[0].contact = 0.0;
                    rd_.ee_[1].contact = 1.0;
                }

                if (phaseChange == true && phaseChange1 == false)
                {
                    WBC::SetContact(rd_, rd_.ee_[0].contact, rd_.ee_[1].contact);
                    TorqueGrav = WBC::GravityCompensationTorque(rd_);

                    if (rd_.sim_mode == false)
                    {
                        TorqueGrav(1) = 1.2 * TorqueGrav(1);
                        TorqueGrav(7) = 1.2 * TorqueGrav(7);
                    }

                    rate = DyrosMath::cubic(walking_tick, double2Single_pre, double2Single, 1, 0, 0, 0);
                    TorqueContact = WBC::ContactForceRedistributionTorqueWalking(rd_, TorqueGrav, fc_ratio, rate, foot_step_mu(current_step_num, 6));
                    rd_.torque_desired_walk = TorqueGrav + TorqueContact;
                }
                else if (phaseChange == false && phaseChange1 == true)
                {
                    WBC::SetContact(rd_, rd_.ee_[0].contact, rd_.ee_[1].contact);
                    TorqueGrav = WBC::GravityCompensationTorque(rd_);

                    if (rd_.sim_mode == false)
                    {
                        TorqueGrav(1) = 1.2 * TorqueGrav(1);
                        TorqueGrav(7) = 1.2 * TorqueGrav(7);
                    }

                    rate = DyrosMath::cubic(walking_tick, single2Double_pre, single2Double, 0, 1, 0, 0);
                    if (current_step_num < total_step_num - 1)
                        TorqueContact = WBC::ContactForceRedistributionTorqueWalking(rd_, TorqueGrav, fc_ratio, rate, foot_step_mu(current_step_num, 6));
                    else
                        TorqueContact = WBC::ContactForceRedistributionTorqueWalking(rd_, TorqueGrav, fc_ratio, rate, foot_step_mu(total_step_num - 1, 6));

                    rd_.torque_desired_walk = TorqueGrav + TorqueContact;
                }
                else
                {
                    WBC::SetContact(rd_, rd_.ee_[0].contact, rd_.ee_[1].contact);
                    rate = 1.0;
                    TorqueGrav = WBC::GravityCompensationTorque(rd_);
                    if (current_step_num < total_step_num - 1)
                        TorqueContact = WBC::ContactForceRedistributionTorqueWalking(rd_, TorqueGrav, fc_ratio, rate, foot_step_mu(current_step_num + 1, 6));
                    else
                        TorqueContact = WBC::ContactForceRedistributionTorqueWalking(rd_, TorqueGrav, fc_ratio, rate, foot_step_mu(total_step_num - 2, 6));

                    rd_.torque_desired_walk = TorqueGrav + TorqueContact;
                }
                fc_redis = rd_.fc_redist_;
            }
            else
            {
                WBC::SetContact(rd_, 1, 1);
                rd_.torque_desired_walk = WBC::ContactForceRedistributionTorque(rd_, WBC::GravityCompensationTorque(rd_));
            }

            for (int i = 0; i < MODEL_DOF; i++)
            {
                rd_.torque_desired[i] = rd_.pos_kp_v[i] * (rd_.q_desired[i] - rd_.q_[i]) + rd_.pos_kv_v[i] * (-rd_.q_dot_[i]) + rd_.torque_desired_walk[i];
            }
        }
        else if (rd_.tc_.walking_enable == 3.0 || rd_.tc_.walking_enable == 2.0)
        {
            WBC::SetContact(rd_, 1, 1);
            rd_.torque_desired_walk = WBC::ContactForceRedistributionTorque(rd_, WBC::GravityCompensationTorque(rd_));
            for (int i = 0; i < MODEL_DOF; i++)
            {
                rd_.torque_desired[i] = rd_.pos_kp_v[i] * (rd_.q_desired[i] - rd_.q_[i]) + rd_.pos_kv_v[i] * (-rd_.q_dot_[i]) + rd_.torque_desired_walk[i];
            }
        }
    }
    //file[1] << walking_tick <<"\t"<< rd_.torque_desired[1] << "\t" << rd_.torque_desired_walk[1] << "\t" << TorqueContact(1) << "\t" << TorqueGrav(1) << "\t" << rd_.torque_desired[2] << "\t" << rd_.torque_desired_walk[2] << "\t" << TorqueContact(2) << "\t" << TorqueGrav(2) << "\t" << rd_.torque_desired[3] << "\t" << rd_.torque_desired_walk[3] << "\t" << TorqueContact(3) << "\t" << TorqueGrav(3) << "\t" << rd_.torque_desired[4] << "\t" << rd_.torque_desired_walk[4] << "\t" << TorqueContact(4) << "\t" << TorqueGrav(4) << "\t" << rd_.torque_desired[5] << "\t" << rd_.torque_desired_walk[5] << "\t" << TorqueContact(5) << "\t" << TorqueGrav(5) << "\t" << std::endl;
    //   file[0] <<walking_tick << "\t" <<current_step_num <<"\t" << total_step_num << "\t"<< com_refx.size()<<"\t"<< rd_.q_desired(0) << "\t"<< desired_leg_q_temp(0) <<  "\t"<< rd_.q_(0) << "\t" << rd_.q_desired(1)<< "\t"<< desired_leg_q_temp(1) << "\t" << rd_.q_(1) << "\t" << rd_.q_desired(2) << "\t"<< desired_leg_q_temp(2)<< "\t" << rd_.q_(2) << "\t" << rd_.q_desired(3) << "\t"<< desired_leg_q_temp(3)<< "\t" << rd_.q_(3) << "\t" << rd_.q_desired(4) << "\t"<< desired_leg_q_temp(4)<< "\t" << rd_.q_(4) << "\t" << rd_.q_desired(5) << "\t"<< desired_leg_q_temp(5)<< "\t" << rd_.q_(5) << std::endl;
}

void CustomController::computeFast()
{
    if (rd_.tc_.mode == 10)
    {
    }
    else if (rd_.tc_.mode == 11)
    {
        if (rd_.tc_.walking_enable == 1.0)
        {
            if (wlk_on == false && velEst_f == true)
            {
                wk_Hz = 1000;
                wk_dt = 1 / wk_Hz;
                walking_tick = 0;

                ik_mode = rd_.tc_.ik_mode;
                walking_pattern = rd_.tc_.pattern;
                com_control = rd_.tc_.pattern2;
                target(0) = rd_.tc_.x;
                target(1) = rd_.tc_.y;
                target(2) = rd_.tc_.z;
                target(3) = rd_.tc_.theta;
                height = rd_.tc_.height;
                step_length_x = rd_.tc_.step_length_x;
                step_length_y = rd_.tc_.step_length_y;
                dob = rd_.tc_.dob;
                imu = rd_.tc_.imu;
                rd_.tc_.walking_height;
                mom = rd_.tc_.mom;
                vibration_control = rd_.tc_.comcontrol;
                com_control_mode = true;
                gyro_frame_flag = false;

                if (rd_.tc_.first_foot_step == 0)
                {
                    foot_step_dir = 1.0;
                }
                else
                {
                    foot_step_dir = -1.0;
                }

                setWalkingParameter();

                //////InitModel//////
                getRobotInitState(rd_);

                /////FootStep//////
                footStepGenerator(rd_);
                saveFootTrajectory();

                setCpPosition();
                cpReferencePatternGeneration();
                cptoComTrajectory();

                mpcSoftVariable(rd_);
                mpcStateContraint(rd_);

                cc_mutex.lock();
                foot_step_mu = foot_step;
                LFvx_trajectory_float_mu = LFvx_trajectory_float;
                LFvy_trajectory_float_mu = LFvy_trajectory_float;
                LFvz_trajectory_float_mu = LFvz_trajectory_float;
                RFvx_trajectory_float_mu = RFvx_trajectory_float;
                RFvy_trajectory_float_mu = RFvy_trajectory_float;
                RFvz_trajectory_float_mu = RFvz_trajectory_float;
                LFx_trajectory_float_mu = LFx_trajectory_float;
                LFy_trajectory_float_mu = LFy_trajectory_float;
                LFz_trajectory_float_mu = LFz_trajectory_float;
                RFx_trajectory_float_mu = RFx_trajectory_float;
                RFy_trajectory_float_mu = RFy_trajectory_float;
                RFz_trajectory_float_mu = RFz_trajectory_float;
                COM_float_init_mu = COM_float_init;
                com_refx_mu = com_refx;
                com_refy_mu = com_refy;
                zmp_refx_mu = zmp_refx;
                zmp_refy_mu = zmp_refy;

                softCx_mu = softCx_s;
                softCy_mu = softCy_s;
                softBoundx_mu = softBoundx_s;
                softBoundy_mu = softBoundy_s;
                zmpx_mu = zmpx_s;
                zmpy_mu = zmpy_s;
                xL_mu = xL_s;
                xU_mu = xU_s;
                yL_mu = yL_s;
                yU_mu = yU_s;

                softCx1_mu = softCx_s;
                softCy1_mu = softCy_s;
                softBoundx1_mu = softBoundx_s;
                softBoundy1_mu = softBoundy_s;
                zmpx1_mu = zmpx_s;
                zmpy1_mu = zmpy_s;
                xL1_mu = xL_s;
                xU1_mu = xU_s;
                yL1_mu = yL_s;
                yU1_mu = yU_s;

                cc_mutex.unlock();

                wlk_on = true;
            }

            if (wlk_on == true)
            {
                walkingCompute(rd_);
                momentumControl(rd_);

                cc_mutex.lock();
                for (int i = 0; i < 12; i++)
                {
                    rd_.q_desired(i) = desired_leg_q(i);
                }
                for (int i = 12; i < MODEL_DOF; i++)
                {
                    rd_.q_desired(i) = desired_init_q(i);
                }
                cc_mutex.unlock();
            }
        }
        else if (rd_.tc_.walking_enable == 3.0)
        {
            wk_Hz = 1000;
            wk_dt = 1 / wk_Hz;
            setInitPose(rd_, desired_init_q);
            updateInitTime();

            cc_mutex.lock();
            for (int i = 0; i < MODEL_DOF; i++)
            {
                rd_.q_desired(i) = desired_init_q(i);
            }
            cc_mutex.unlock();
        }
    }
}

void CustomController::computePlanner()
{
    if (rd_.tc_.mode == 10)
    {
    }
    else if (rd_.tc_.mode == 11)
    {
        if (rd_.tc_.walking_enable == 1.0)
        {
            if (wlk_on == true && mpc_on == false)
            {
                mpcModelSetup();

                // maximum element in cost functions
                if (ns[1] > 0 | ns[N] > 0)
                    mu0 = 1000.0;
                else
                    mu0 = 2.0;

                //solver Setup
                int iter_max = 20;
                double alpha_min = 5e-3;
                double tol_stat = 5e-3;
                double tol_eq = 5e-3;
                double tol_ineq = 5e-3;
                double tol_comp = 5e-3;
                double reg_prim = 5e-3;
                int warm_start = 0;
                int pred_corr = 1;
                int ric_alg = 1;
                enum hpipm_mode mode = SPEED_ABS;

                dim_sizex = d_ocp_qp_dim_memsize(N);
                dim_memx = malloc(dim_sizex);
                d_ocp_qp_dim_create(N, &dimx, dim_memx);
                ipm_arg_sizex = d_ocp_qp_ipm_arg_memsize(&dimx);
                ipm_arg_memx = malloc(ipm_arg_sizex);
                d_ocp_qp_ipm_arg_create(&dimx, &argx, ipm_arg_memx);
                d_ocp_qp_ipm_arg_set_default(mode, &argx);
                d_ocp_qp_ipm_arg_set_mu0(&mu0, &argx);
                d_ocp_qp_ipm_arg_set_iter_max(&iter_max, &argx);
                d_ocp_qp_ipm_arg_set_tol_stat(&tol_stat, &argx);
                d_ocp_qp_ipm_arg_set_tol_eq(&tol_eq, &argx);
                d_ocp_qp_ipm_arg_set_tol_ineq(&tol_ineq, &argx);
                d_ocp_qp_ipm_arg_set_tol_comp(&tol_comp, &argx);
                d_ocp_qp_ipm_arg_set_reg_prim(&reg_prim, &argx);
                d_ocp_qp_ipm_arg_set_warm_start(&warm_start, &argx);
                d_ocp_qp_ipm_arg_set_ric_alg(&ric_alg, &argx);

                d_ocp_qp_dim_set_all(nx, nu, nbx, nbu, ng, nsbx, nsbu, nsg, &dimx);
                qp_sizex = d_ocp_qp_memsize(&dimx);
                qp_memx = malloc(qp_sizex);
                d_ocp_qp_create(&dimx, &qpx, qp_memx);
                qp_sol_sizex = d_ocp_qp_sol_memsize(&dimx);
                qp_sol_memx = malloc(qp_sol_sizex);
                ipm_sizex = d_ocp_qp_ipm_ws_memsize(&dimx, &argx);
                ipm_memx = malloc(ipm_sizex);
                d_ocp_qp_ipm_ws_create(&dimx, &argx, &workspacex, ipm_memx);

                dim_sizey = d_ocp_qp_dim_memsize(N);
                dim_memy = malloc(dim_sizey);
                d_ocp_qp_dim_create(N, &dimy, dim_memy);
                ipm_arg_sizey = d_ocp_qp_ipm_arg_memsize(&dimy);
                ipm_arg_memy = malloc(ipm_arg_sizey);
                d_ocp_qp_ipm_arg_create(&dimy, &argy, ipm_arg_memy);
                d_ocp_qp_ipm_arg_set_default(mode, &argy);
                d_ocp_qp_ipm_arg_set_mu0(&mu0, &argy);
                d_ocp_qp_ipm_arg_set_iter_max(&iter_max, &argy);
                d_ocp_qp_ipm_arg_set_tol_stat(&tol_stat, &argy);
                d_ocp_qp_ipm_arg_set_tol_eq(&tol_eq, &argy);
                d_ocp_qp_ipm_arg_set_tol_ineq(&tol_ineq, &argy);
                d_ocp_qp_ipm_arg_set_tol_comp(&tol_comp, &argy);
                d_ocp_qp_ipm_arg_set_reg_prim(&reg_prim, &argy);
                d_ocp_qp_ipm_arg_set_warm_start(&warm_start, &argy);
                d_ocp_qp_ipm_arg_set_ric_alg(&ric_alg, &argy);

                d_ocp_qp_dim_set_all(nx, nu, nbx, nbu, ng, nsbx, nsbu, nsg, &dimy);
                qp_sizey = d_ocp_qp_memsize(&dimy);
                qp_memy = malloc(qp_sizey);
                d_ocp_qp_create(&dimy, &qpy, qp_memy);
                qp_sol_sizey = d_ocp_qp_sol_memsize(&dimy);
                qp_sol_memy = malloc(qp_sol_sizey);
                ipm_sizey = d_ocp_qp_ipm_ws_memsize(&dimy, &argy);
                ipm_memy = malloc(ipm_sizey);
                d_ocp_qp_ipm_ws_create(&dimy, &argy, &workspacey, ipm_memy);

                d_ocp_qp_sol_create(&dimx, &qp_solx, qp_sol_memx);
                d_ocp_qp_sol_create(&dimy, &qp_soly, qp_sol_memy);
                mpc_on = true;
                std::cout << "MPC INIT" << std::endl;
            }

            if (wlk_on == true && mpc_s == true && mpc_on == true && walking_tick >= 0 && mpc_cycle < 1800) //(t_total * (total_step_num + 1) + t_temp - 1) / mpct - 1)
            {
                /************************************************
                *********box & general constraints**************
                ************************************************/

                auto t4 = std::chrono::steady_clock::now();
                mpc_variablex();
                mpc_variabley();
                solverx = std::async(std::launch::async, &CustomController::walking_x, this);
                solvery = std::async(std::launch::async, &CustomController::walking_y, this);
                solverx.wait();
                solvery.wait();
                auto t5 = std::chrono::steady_clock::now();
                auto d1 = std::chrono::duration_cast<std::chrono::microseconds>(t5 - t4).count();
                 
                 if(d1 >= 5000)
                 {
                     std::cout << d1 << std::endl;
                     std::cout <<"mpc " << mpc_cycle << std::endl;
                 }

                 //std::cout << "solvemicro " << d7 <<" "<<d8<<std::endl;
                /*     if (d7 >= 5000)
                {
                    std::cout << mpc_cycle << std::endl;
                    std::cout << "d7 " << d7 << std::endl;
                }

                if (d8 >= 5000)
                {
                    // std::cout << mpc_cycle<< std::endl;
                    //   std::cout <<"d8 " << d8 << std::endl;
                }
               */
                file[1] << (t_total * (total_step_num + 1) + t_temp - 1) << "\t" << mpc_cycle << "\t" << x11x[0] << "\t" << x11x[2] << "\t" << x11x[4] << "\t" << x11y[0] << "\t" << x11y[2] << "\t" << x11y[4] << "\t" << hd_lbxx[1][0] << "\t" << hd_ubxx[1][0] << "\t" << hd_lbxx[1][1] << "\t" << hd_ubxx[1][1] << "\t" << hd_lbxx[1][2] << "\t" << hd_ubxx[1][2] << "\t" << com_refx_mu(mpct * mpc_cycle) << "\t" << com_refy_mu(mpct * mpc_cycle) << "\t" << zmp_refx_mu(mpct * mpc_cycle) << "\t" << zmp_refy_mu(mpct * mpc_cycle) << std::endl;

                if (hpipm_statusx == 1)
                {
                    printf("\n -> x : Solver failed! Maximum number of iterations reached\n");
                    std::cout << "mpc" << mpc_cycle << std::endl;
                }
                else if (hpipm_statusx == 2)
                {
                    printf("\n -> x : Solver failed! Minimum step lenght reached\n");
                    std::cout << "mpc" << mpc_cycle << std::endl;
                }
                else if (hpipm_statusx == 2)
                {
                    printf("\n -> x : Solver failed! NaN in computations\n");
                    std::cout << "mpc" << mpc_cycle << std::endl;
                }

                if (hpipm_statusy == 1)
                {
                    printf("\n -> y : Solver failed! Maximum number of iterations reached\n");
                    std::cout << "mpc" << mpc_cycle << std::endl;
                }
                else if (hpipm_statusy == 2)
                {
                    printf("\n -> y : Solver failed! Minimum step lenght reached\n");

                    std::cout << "mpc" << mpc_cycle << std::endl;
                }
                else if (hpipm_statusy == 2)
                {
                    printf("\n -> y : Solver failed! NaN in computations\n");
                    std::cout << "mpc" << mpc_cycle << std::endl;
                }

                mpc_cycle++;
            }

            mpc_s = true;
        }
    }
}

void CustomController::mpc_variablex()
{
    std::copy(xL_mu + mpc_cycle, xL_mu + N + 1 + mpc_cycle, hd_lbxx);
    std::copy(xU_mu + mpc_cycle, xU_mu + N + 1 + mpc_cycle, hd_ubxx);

    if (mpc_cycle == 0)
    {
        d_lbx0x[0] = com_refx_mu(0);
        d_lbx0x[1] = 0.0;
        d_lbx0x[2] = com_refx_mu(0);
        d_lbx0x[3] = 0.0;
        d_lbx0x[4] = 0.0;
        d_ubx0x[0] = com_refx_mu(0);
        d_ubx0x[1] = 0.0;
        d_ubx0x[2] = com_refx_mu(0);
        d_ubx0x[3] = 0.0;
        d_ubx0x[4] = 0.0;

        hd_lbxx[0] = d_lbx0x;
        hd_ubxx[0] = d_ubx0x;
    }
    else
    {
        hd_lbxx[0] = x11x;
        hd_ubxx[0] = x11x;
    }
    std::copy(softCx_mu + mpc_cycle, softCx_mu + N + mpc_cycle, hCx);
    std::copy(softBoundx_mu + mpc_cycle, softBoundx_mu + N + mpc_cycle, hd_lgx);
    std::copy(softBoundx_mu + mpc_cycle, softBoundx_mu + N + mpc_cycle, hd_ugx);
    std::copy(zmpx_mu + mpc_cycle, zmpx_mu + N + 1 + mpc_cycle, hqx);
}

void CustomController::mpc_variabley()
{
    std::copy(yL_mu + mpc_cycle, yL_mu + N + 1 + mpc_cycle, hd_lbxy);
    std::copy(yU_mu + mpc_cycle, yU_mu + N + 1 + mpc_cycle, hd_ubxy);

    if (mpc_cycle == 0)
    {
        d_lbx0y[0] = 0.0; //com_refy_mu(2000);
        d_lbx0y[1] = 0.0; //com_refdy(2000);
        d_lbx0y[2] = 0.0; //zmp_refy_mu(2000);
        d_lbx0y[3] = 0.0;
        d_lbx0y[4] = 0.0;
        d_ubx0y[0] = 0.0; //com_refy_mu(2000);
        d_ubx0y[1] = 0.0; //com_refdy(2000);
        d_ubx0y[2] = 0.0; //zmp_refy_mu(2000);
        d_ubx0y[3] = 0.0;
        d_ubx0y[4] = 0.0;
        hd_lbxy[0] = d_lbx0y;
        hd_ubxy[0] = d_ubx0y;
    }
    else
    {
        hd_lbxy[0] = x11y;
        hd_ubxy[0] = x11y;
    }

    std::copy(softCy_mu + mpc_cycle, softCy_mu + N + mpc_cycle, hCy);
    std::copy(softBoundy_mu + mpc_cycle, softBoundy_mu + N + mpc_cycle, hd_lgy);
    std::copy(softBoundy_mu + mpc_cycle, softBoundy_mu + N + mpc_cycle, hd_ugy);
    std::copy(zmpy_mu + mpc_cycle, zmpy_mu + N + 1 + mpc_cycle, hqy);
}

void CustomController::walking_x()
{
    //MPC Setup
    d_ocp_qp_set_all(hAx, hBx, hbx, hQx, hSx, hRx, hqx, hrx, hidxbx, hd_lbxx, hd_ubxx, hidxbu, hd_lbux, hd_ubux, hCx, hDx, hd_lgx, hd_ugx, hZlx, hZux, hzlx, hzux, hidxs, hd_lsx, hd_usx, &qpx);
    d_ocp_qp_ipm_solve(&qpx, &qp_solx, &argx, &workspacex);
    d_ocp_qp_ipm_get_status(&workspacex, &hpipm_statusx);
    d_ocp_qp_sol_get_x(1, &qp_solx, x11x);
}

void CustomController::walking_y()
{
    d_ocp_qp_set_all(hAy, hBy, hby, hQy, hSy, hRy, hqy, hry, hidxbx, hd_lbxy, hd_ubxy, hidxbu, hd_lbuy, hd_ubuy, hCy, hDy, hd_lgy, hd_ugy, hZly, hZuy, hzly, hzuy, hidxs, hd_lsy, hd_usy, &qpy);
    d_ocp_qp_ipm_solve(&qpy, &qp_soly, &argy, &workspacey);
    d_ocp_qp_ipm_get_status(&workspacey, &hpipm_statusy);
    d_ocp_qp_sol_get_x(1, &qp_soly, x11y);
}

void CustomController::copyRobotData(RobotData &rd_l)
{
    std::memcpy(&rd_cc_, &rd_l, sizeof(RobotData));
}

void CustomController::jointVelocityEstimate()
{
    //Estimate joint velocity using state observer
    double dt;
    dt = 1 / 2000;
    Eigen::MatrixXd A_t, A_dt, B_t, B_dt, C, I, I_t;
    I.setZero(MODEL_DOF * 2, MODEL_DOF * 2);
    I.setIdentity();
    I_t.setZero(MODEL_DOF, MODEL_DOF);
    I_t.setIdentity();
    A_t.setZero(MODEL_DOF * 2, MODEL_DOF * 2);
    A_dt.setZero(MODEL_DOF * 2, MODEL_DOF * 2);
    B_t.setZero(MODEL_DOF * 2, MODEL_DOF);
    B_dt.setZero(MODEL_DOF * 2, MODEL_DOF);
    C.setZero(MODEL_DOF, MODEL_DOF * 2);

    A_t.topRightCorner(MODEL_DOF, MODEL_DOF);
    A_t.bottomRightCorner(MODEL_DOF, MODEL_DOF) = rd_.A_inv_.bottomRightCorner(MODEL_DOF, MODEL_DOF) * Cor_;
    A_t.topRightCorner(MODEL_DOF, MODEL_DOF) = I_t;
    B_t.bottomRightCorner(MODEL_DOF, MODEL_DOF) = rd_.A_inv_.bottomRightCorner(MODEL_DOF, MODEL_DOF);
    C.bottomLeftCorner(MODEL_DOF, MODEL_DOF) = I_t * dt;
    B_dt = B_t * dt;
    A_dt = I - dt * A_dt;

    double L, L1;
    L = 0.004;
    L1 = 0.004;

    if (velEst == false)
    {
        q_est = rd_.q_;
        q_dot_est = rd_.q_dot_;
        velEst = true;
    }

    if (velEst == true)
    {
        Eigen::VectorQd q_temp;
        // Eigen::VectorVQd q_dot_virtual;

        q_temp = q_est;

        q_est = q_est + dt * q_dot_est + L * (rd_.q_ - q_est);

        //q_dot_virtual.segment<MODEL_DOF>(6) = q_dot_est;

        q_dot_est = (q_temp - q_est) * 2000.0;

        Eigen::VectorQd tau_;
        tau_ = Cor_ * q_dot_est + G_;

        cc_mutex.lock();
        q_dot_est_mu = -(q_dot_est + B_dt.bottomRightCorner(MODEL_DOF, MODEL_DOF) * (rd_.torque_desired + L1 * (rd_.q_ - q_est) - tau_));
        Ag_ = CMM;
        cc_mutex.unlock();
        q_dot_est = q_dot_est_mu;
    }
}

void CustomController::flyWheelModel(double Ts, int nx, int nu, double *Ax, double *Bx, double *Ay, double *By)
{
    int ii;
    int nx2 = nx * nx;

    for (ii = 0; ii < nx * nx; ii++)
    {
        Ax[ii] = 0.0;
        Ay[ii] = 0.0;
    }

    Ax[0] = 1.0;
    Ax[6] = 1.0;
    Ax[12] = 1.0;
    Ax[18] = 1.0;
    Ax[24] = 1.0;

    Ax[1] = lipm_w * lipm_w * Ts;
    Ax[23] = 1.0 * Ts;
    Ax[11] = -lipm_w * lipm_w * Ts;
    Ax[5] = 1.0 * Ts;

    Ay[0] = 1.0;
    Ay[6] = 1.0;
    Ay[12] = 1.0;
    Ay[18] = 1.0;
    Ay[24] = 1.0;

    Ay[1] = lipm_w * lipm_w * Ts;
    Ay[23] = 1.0 * Ts;
    Ay[11] = -lipm_w * lipm_w * Ts;
    Ay[5] = 1.0 * Ts;

    for (ii = 0; ii < nx * nu; ii++)
    {
        Bx[ii] = 0.0;
        By[ii] = 0.0;
    }

    Bx[6] = 1.0 / (total_mass * zc) * Ts;
    Bx[2] = 1.00 * Ts;
    Bx[9] = 1.00 * Ts;

    By[6] = -1.0 / (total_mass * zc) * Ts;
    By[2] = 1.00 * Ts;
    By[9] = 1.00 * Ts;
}

void CustomController::mpcVariableInit()
{
    N = timeHorizon / Ts;
    nx_ = 5;
    nu_ = 2;

    //resize
    nx = (int *)malloc((N + 1) * sizeof(int));
    nu = (int *)malloc((N + 1) * sizeof(int));
    nbu = (int *)malloc((N + 1) * sizeof(int));
    nbx = (int *)malloc((N + 1) * sizeof(int));
    nb = (int *)malloc((N + 1) * sizeof(int));
    ng = (int *)malloc((N + 1) * sizeof(int));
    nsbx = (int *)malloc((N + 1) * sizeof(int));
    nsbu = (int *)malloc((N + 1) * sizeof(int));
    nsg = (int *)malloc((N + 1) * sizeof(int));
    ns = (int *)malloc((N + 1) * sizeof(int));

    hAx = (double **)malloc((N) * sizeof(double *));
    hBx = (double **)malloc((N) * sizeof(double *));
    hbx = (double **)malloc((N) * sizeof(double *));
    hQx = (double **)malloc((N + 1) * sizeof(double *));
    hSx = (double **)malloc((N + 1) * sizeof(double *));
    hRx = (double **)malloc((N + 1) * sizeof(double *));
    hqx = (double **)malloc((N + 1) * sizeof(double *));
    hrx = (double **)malloc((N + 1) * sizeof(double *));

    hAy = (double **)malloc((N) * sizeof(double *));
    hBy = (double **)malloc((N) * sizeof(double *));
    hby = (double **)malloc((N) * sizeof(double *));
    hQy = (double **)malloc((N + 1) * sizeof(double *));
    hSy = (double **)malloc((N + 1) * sizeof(double *));
    hRy = (double **)malloc((N + 1) * sizeof(double *));
    hqy = (double **)malloc((N + 1) * sizeof(double *));
    hry = (double **)malloc((N + 1) * sizeof(double *));

    hidxbx = (int **)malloc((N + 1) * sizeof(int *));
    hd_lbxx = (double **)malloc((N + 1) * sizeof(double *));
    hd_ubxx = (double **)malloc((N + 1) * sizeof(double *));
    hd_lbxy = (double **)malloc((N + 1) * sizeof(double *));
    hd_ubxy = (double **)malloc((N + 1) * sizeof(double *));
    hidxbu = (int **)malloc((N + 1) * sizeof(int *));
    hd_lbux = (double **)malloc((N + 1) * sizeof(double *));
    hd_ubux = (double **)malloc((N + 1) * sizeof(double *));
    hd_lbuy = (double **)malloc((N + 1) * sizeof(double *));
    hd_ubuy = (double **)malloc((N + 1) * sizeof(double *));
    hCx = (double **)malloc((N + 1) * sizeof(double *));
    hDx = (double **)malloc((N + 1) * sizeof(double *));
    hd_lgx = (double **)malloc((N + 1) * sizeof(double *));
    hd_ugx = (double **)malloc((N + 1) * sizeof(double *));
    hCy = (double **)malloc((N + 1) * sizeof(double *));
    hDy = (double **)malloc((N + 1) * sizeof(double *));
    hd_lgy = (double **)malloc((N + 1) * sizeof(double *));
    hd_ugy = (double **)malloc((N + 1) * sizeof(double *));
    hZlx = (double **)malloc((N + 1) * sizeof(double *));
    hZux = (double **)malloc((N + 1) * sizeof(double *));
    hzlx = (double **)malloc((N + 1) * sizeof(double *));
    hzux = (double **)malloc((N + 1) * sizeof(double *));
    hZly = (double **)malloc((N + 1) * sizeof(double *));
    hZuy = (double **)malloc((N + 1) * sizeof(double *));
    hzly = (double **)malloc((N + 1) * sizeof(double *));
    hzuy = (double **)malloc((N + 1) * sizeof(double *));
    hidxs = (int **)malloc((N + 1) * sizeof(int *));
    hd_lsx = (double **)malloc((N + 1) * sizeof(double *));
    hd_usx = (double **)malloc((N + 1) * sizeof(double *));
    hd_lsy = (double **)malloc((N + 1) * sizeof(double *));
    hd_usy = (double **)malloc((N + 1) * sizeof(double *));

    // stage-wise variant size
    for (ii = 1; ii <= N; ii++)
    {
        nx[ii] = nx_;
        nu[ii] = nu_;
        nbu[ii] = nu[ii];
        nbx[ii] = nx[ii];
        nb[ii] = nbu[ii] + nbx[ii];
        ng[ii] = 1;
        nsbx[ii] = 0;
    }

    nu[N] = 0;
    ng[N] = 1;
    nbu[N] = 0;
    nsbx[N] = 0;
    nu[0] = nu_;
    nbu[0] = nu_;
    nx[0] = nx_;
    nb[0] = nbu[0] + nbx[0];
    nbx[0] = nx[0];
    ng[0] = 0;
    nsbx[0] = 0;

    nsg[0] = 0;
    nsbu[0] = 0;
    ns[0] = nsbx[0] + nsbu[0] + nsg[0];

    for (ii = 1; ii <= N; ii++)
    {
        nsg[ii] = 1;
        nsbu[ii] = 0;
        ns[ii] = nsbx[ii] + nsbu[ii] + nsg[ii];
    }

    d_zeros(&Ax, nx_, nx_); // states update matrix
    d_zeros(&Bx, nx_, nu_); // inputs matrix
    d_zeros(&bx, nx_, 1);   // states offset
    d_zeros(&x0x, nx_, 1);  // initial state
    d_zeros(&Qx, nx_, nx_);
    d_zeros(&Rx, nu_, nu_);
    d_zeros(&Sx, nu_, nx_);
    d_zeros(&qx, nx_, 1);
    d_zeros(&rx, nu_, 1);
    d_zeros(&Ay, nx_, nx_); // states update matrix
    d_zeros(&By, nx_, nu_); // inputs matrix
    d_zeros(&by, nx_, 1);   // states offset
    d_zeros(&x0y, nx_, 1);  // initial state
    d_zeros(&Qy, nx_, nx_);
    d_zeros(&Ry, nu_, nu_);
    d_zeros(&Sy, nu_, nx_);
    d_zeros(&qy, nx_, 1);
    d_zeros(&ry, nu_, 1);

    d_zeros(&d_lbx1x, nbx[1], 1);
    d_zeros(&d_ubx1x, nbx[1], 1);
    d_zeros(&d_lbu1x, nbu[1], 1);
    d_zeros(&d_ubu1x, nbu[1], 1);
    d_zeros(&d_lg1x, ng[1], 1);
    d_zeros(&d_ug1x, ng[1], 1);
    d_zeros(&d_lbx0x, nbx[0], 1);
    d_zeros(&d_ubx0x, nbx[0], 1);
    d_zeros(&d_lbu0x, nbu[0], 1);
    d_zeros(&d_ubu0x, nbu[0], 1);
    d_zeros(&d_lg0x, ng[0], 1);
    d_zeros(&d_ug0x, ng[0], 1);
    d_zeros(&d_lbx1y, nbx[1], 1);
    d_zeros(&d_ubx1y, nbx[1], 1);
    d_zeros(&d_lbu1y, nbu[1], 1);
    d_zeros(&d_ubu1y, nbu[1], 1);
    d_zeros(&d_lg1y, ng[1], 1);
    d_zeros(&d_ug1y, ng[1], 1);
    d_zeros(&d_lbx0y, nbx[0], 1);
    d_zeros(&d_ubx0y, nbx[0], 1);
    d_zeros(&d_lbu0y, nbu[0], 1);
    d_zeros(&d_ubu0y, nbu[0], 1);
    d_zeros(&d_lg0y, ng[0], 1);
    d_zeros(&d_ug0y, ng[0], 1);
    int_zeros(&idxbx0, nbx[0], 1);
    int_zeros(&idxbx1, nbx[1], 1);
    int_zeros(&idxbu1, nbu[1], 1);
    int_zeros(&idxbu0, nbu[0], 1);
    int_zeros(&idxbxN, nbx[N], 1);

    d_zeros(&d_lbxNx, nbx[N], 1);
    d_zeros(&d_ubxNx, nbx[N], 1);
    d_zeros(&d_lgNx, ng[N], 1);
    d_zeros(&d_ugNx, ng[N], 1);
    d_zeros(&C0x, ng[0], nx[0]);
    d_zeros(&D0x, ng[0], nu[0]);
    d_zeros(&C1x, ng[1], nx[1]);
    d_zeros(&D1x, ng[1], nu[1]);
    d_zeros(&CNx, ng[N], nx[N]);
    d_zeros(&DNx, ng[N], nu[N]);
    d_zeros(&Zl0x, ns[0], 1);
    d_zeros(&Zu0x, ns[0], 1);
    d_zeros(&zl0x, ns[0], 1);
    d_zeros(&zu0x, ns[0], 1);
    d_zeros(&d_lbxNy, nbx[N], 1);
    d_zeros(&d_ubxNy, nbx[N], 1);
    d_zeros(&d_lgNy, ng[N], 1);
    d_zeros(&d_ugNy, ng[N], 1);
    d_zeros(&C0y, ng[0], nx[0]);
    d_zeros(&D0y, ng[0], nu[0]);
    d_zeros(&C1y, ng[1], nx[1]);
    d_zeros(&D1y, ng[1], nu[1]);
    d_zeros(&CNy, ng[N], nx[N]);
    d_zeros(&DNy, ng[N], nu[N]);
    d_zeros(&Zl0y, ns[0], 1);
    d_zeros(&Zu0y, ns[0], 1);
    d_zeros(&zl0y, ns[0], 1);
    d_zeros(&zu0y, ns[0], 1);

    int_zeros(&idxs0, ns[0], 1);
    d_zeros(&d_ls0x, ns[0], 1);
    d_zeros(&d_us0x, ns[0], 1);
    d_zeros(&Zl1x, ns[1], 1);
    d_zeros(&Zu1x, ns[1], 1);
    d_zeros(&zl1x, ns[1], 1);
    d_zeros(&zu1x, ns[1], 1);
    d_zeros(&d_ls0y, ns[0], 1);
    d_zeros(&d_us0y, ns[0], 1);
    d_zeros(&Zl1y, ns[1], 1);
    d_zeros(&Zu1y, ns[1], 1);
    d_zeros(&zl1y, ns[1], 1);
    d_zeros(&zu1y, ns[1], 1);
    int_zeros(&idxs1, ns[1], 1);
    d_zeros(&d_ls1x, ns[1], 1);
    d_zeros(&d_us1x, ns[1], 1);
    d_zeros(&ZlNx, ns[N], 1);
    d_zeros(&ZuNx, ns[N], 1);
    d_zeros(&zlNx, ns[N], 1);
    d_zeros(&zuNx, ns[N], 1);
    d_zeros(&d_ls1y, ns[1], 1);
    d_zeros(&d_us1y, ns[1], 1);
    d_zeros(&ZlNy, ns[N], 1);
    d_zeros(&ZuNy, ns[N], 1);
    d_zeros(&zlNy, ns[N], 1);
    d_zeros(&zuNy, ns[N], 1);
    int_zeros(&idxsN, ns[N], 1);
    d_zeros(&d_lsNx, ns[N], 1);
    d_zeros(&d_usNx, ns[N], 1);
    d_zeros(&d_lsNy, ns[N], 1);
    d_zeros(&d_usNy, ns[N], 1);

    for (jj = 0; jj < nx_; jj++)
    {
        x0x[jj] = 0.0;
        bx[jj] = 0.0;
        Qx[jj * (nx_ + 1)] = 3.0;
        qx[jj] = 0.0;
        x0y[jj] = 0.0;
        by[jj] = 0.0;
        Qy[jj * (nx_ + 1)] = 1.0;
        qy[jj] = 0.0;
    }

    Qy[0] = 0.5;
    Qy[1] = 0.5;
    Qy[2 * (nx_ + 1)] = 500000; //5000;

    Qx[0] = 3;
    Qx[2 * (nx_ + 1)] = 999000;

    for (ii = 0; ii < nu_; ii++)
    {
        rx[ii] = 0.0;
        Rx[ii * (nu_ + 1)] = 20.0;
        ry[ii] = 0.0;
        Ry[ii * (nu_ + 1)] = 10.0;
    }

    Rx[0] = 300.0;
    Ry[0] = 200.0;

    for (ii = 0; ii < nbu[0]; ii++)
    {
        idxbu0[ii] = ii;
    }

    for (ii = 0; ii < nbx[0]; ii++)
    {
        idxbx0[ii] = ii;
    }

    for (ii = 0; ii < nbu[1]; ii++)
    {
        idxbu1[ii] = ii;
    }

    for (ii = 0; ii < nbx[1]; ii++)
    {
        idxbx1[ii] = ii;
    }

    for (ii = 0; ii < nbx[N]; ii++)
    {
        idxbxN[ii] = ii;
    }

    //SOFT CONSTARINT
    for (ii = 0; ii < ns[0]; ii++)
    {
        Zl0x[ii] = 1000.0;
        Zu0x[ii] = 1000.0;
        zl0x[ii] = 0;
        zu0x[ii] = 0;
        Zl0y[ii] = 500.0;
        Zu0y[ii] = 500.0;
        zl0y[ii] = 0;
        zu0y[ii] = 0;
        idxs0[ii] = nu[0] + nx[0] + ii;
        d_ls0x[ii] = 0.00; //-1.0;
        d_us0x[ii] = 0.00;
        d_ls0y[ii] = 0.00; //-1.0;
        d_us0y[ii] = 0.00;
    }

    for (ii = 0; ii < ns[1]; ii++)
    {
        Zl1x[ii] = 1000.0;
        Zu1x[ii] = 1000.0;
        zl1x[ii] = 0;
        zu1x[ii] = 0;
        Zl1y[ii] = 500.0;
        Zu1y[ii] = 500.0;
        zl1y[ii] = 0;
        zu1y[ii] = 0;
        idxs1[ii] = nu[1] + nx[1] + ii;
        d_ls1x[ii] = 0.0; //-1.0;
        d_us1x[ii] = 0.0;
        d_ls1y[ii] = 0.0; //-1.0;
        d_us1y[ii] = 0.0;
        ZlNx[ii] = 1000.0;
        ZuNx[ii] = 1000.0;
        zlNx[ii] = 0;
        zuNx[ii] = 0;
        ZlNy[ii] = 500.0;
        ZuNy[ii] = 500.0;
        zlNy[ii] = 0;
        zuNy[ii] = 0;
        idxsN[ii] = nu[N] + nx[N] + ii;
        d_lsNx[ii] = 0.0; //-1.0;
        d_usNx[ii] = 0.0;
        d_lsNy[ii] = 0.0; //-1.0;
        d_usNy[ii] = 0.0;
    }

    nx_max = nx[0];
    for (ii = 1; ii <= N; ii++)
        if (nx[ii] > nx_max)
            nx_max = nx[ii];

    x11x = (double *)malloc(nx_max * sizeof(double));
    u11x = (double *)malloc(nx_max * sizeof(double));
    slx = (double *)malloc(1 * sizeof(double));
    sux = (double *)malloc(1 * sizeof(double));
    x11y = (double *)malloc(nx_max * sizeof(double));
    u11y = (double *)malloc(nx_max * sizeof(double));
    sly = (double *)malloc(1 * sizeof(double));
    suy = (double *)malloc(1 * sizeof(double));

    //Identify
    hidxbx[0] = idxbx0;
    hidxbu[0] = idxbu0;
    hidxs[0] = idxs0;
    hidxbx[N] = idxbxN;
    hidxs[N] = idxsN;
    for (ii = 1; ii < N; ii++)
    {
        hidxs[ii] = idxs1;
        hidxbu[ii] = idxbu1;
        hidxbx[ii] = idxbx1;
    }

    //INPUT CONSTRAINT
    d_lbu0x[0] = -0.3;
    d_lbu0x[1] = -10;
    d_ubu0x[0] = 0.3;
    d_ubu0x[1] = 10;

    d_lbu0y[0] = -1.3;
    d_lbu0y[1] = -8;
    d_ubu0y[0] = 1.3;
    d_ubu0y[1] = 8;

    d_lbu1x[0] = -0.3;
    d_lbu1x[1] = -10;
    d_ubu1x[0] = 0.3;
    d_ubu1x[1] = 10;

    d_lbu1y[0] = -1.3;
    d_lbu1y[1] = -8;
    d_ubu1y[0] = 1.3;
    d_ubu1y[1] = 8;

    hd_lbux[0] = d_lbu0x;
    hd_ubux[0] = d_ubu0x;
    hd_lbuy[0] = d_lbu0y;
    hd_ubuy[0] = d_ubu0y;

    for (ii = 1; ii < N; ii++)
    {
        hd_lbux[ii] = d_lbu1x;
        hd_ubux[ii] = d_ubu1x;
        hd_lbuy[ii] = d_lbu1y;
        hd_ubuy[ii] = d_ubu1y;
    }

    //SoftConstraint
    hd_lsx[0] = d_ls0x;
    hd_usx[0] = d_us0x;
    hd_lsy[0] = d_ls0y;
    hd_usy[0] = d_us0y;
    for (ii = 1; ii < N; ii++)
    {
        hd_lsx[ii] = d_ls1x;
        hd_usx[ii] = d_us1x;
        hd_lsy[ii] = d_ls1y;
        hd_usy[ii] = d_us1y;
    }
    hd_lsx[N] = d_lsNx;
    hd_usx[N] = d_usNx;
    hd_lsy[N] = d_lsNy;
    hd_usy[N] = d_usNy;

    //CostFunction
    hbx[0] = bx;
    hQx[0] = Qx;
    hSx[0] = Sx;
    hRx[0] = Rx;
    hqx[0] = qx;
    hrx[0] = rx;
    hby[0] = by;
    hQy[0] = Qy;
    hSy[0] = Sy;
    hRy[0] = Ry;
    hqy[0] = qy;
    hry[0] = ry;
    hZlx[0] = Zl0x;
    hZux[0] = Zu0x;
    hzlx[0] = zl0x;
    hzux[0] = zu0x;
    hZly[0] = Zl0y;
    hZuy[0] = Zu0y;
    hzly[0] = zl0y;
    hzuy[0] = zu0y;
    for (ii = 1; ii < N; ii++)
    {
        hbx[ii] = bx;
        hQx[ii] = Qx;
        hSx[ii] = Sx;
        hRx[ii] = Rx;
        hqx[ii] = qx;
        hrx[ii] = rx;
        hby[ii] = by;
        hQy[ii] = Qy;
        hSy[ii] = Sy;
        hRy[ii] = Ry;
        hqy[ii] = qy;
        hry[ii] = ry;
        hZlx[ii] = Zl1x;
        hZux[ii] = Zu1x;
        hzlx[ii] = zl1x;
        hzux[ii] = zu1x;
        hZly[ii] = Zl1y;
        hZuy[ii] = Zu1y;
        hzly[ii] = zl1y;
        hzuy[ii] = zu1y;
    }
    hQx[N] = Qx;
    hSx[N] = Sx;
    hRx[N] = Rx;
    hqx[N] = qx;
    hrx[N] = rx;
    hQy[N] = Qy;
    hSy[N] = Sy;
    hRy[N] = Ry;
    hqy[N] = qy;
    hry[N] = ry;
    hZlx[N] = ZlNx;
    hZux[N] = ZuNx;
    hzlx[N] = zlNx;
    hzux[N] = zuNx;
    hZly[N] = ZlNy;
    hZuy[N] = ZuNy;
    hzly[N] = zlNy;
    hzuy[N] = zuNy;

    //General Constraint X (Init, Final)
    hDx[0] = D0x;
    hDy[0] = D0y;
    hCx[0] = C0x;
    hCy[0] = C0y;

    for (ii = 1; ii < N; ii++)
    {
        hDx[ii] = D1x;
        hDy[ii] = D1y;
        hCx[ii] = C1x;
        hCy[ii] = C1y;
    }
    hDx[N] = DNx;
    hDy[N] = DNy;
    hCx[N] = CNx;
    hCy[N] = CNy;
    hd_lgx[0] = d_lg0x;
    hd_ugx[0] = d_ug0x;
    hd_lgy[0] = d_lg0y;
    hd_ugy[0] = d_ug0y;
    hd_lgx[N] = d_lgNx;
    hd_ugx[N] = d_ugNx;
    hd_lgy[N] = d_lgNy;
    hd_ugy[N] = d_ugNy;
}

void CustomController::mpcModelSetup()
{
    flyWheelModel(Ts, nx_, nu_, Ax, Bx, Ay, By);

    //MODEL
    hAx[0] = Ax;
    hBx[0] = Bx;
    hAy[0] = Ay;
    hBy[0] = By;

    for (ii = 1; ii < N; ii++)
    {
        hAx[ii] = Ax;
        hBx[ii] = Bx;
        hAy[ii] = Ay;
        hBy[ii] = By;
    }
}

void CustomController::momentumControl(RobotData &Robot)
{
    int variable_size, constraint_size;

    variable_size = 5;
    constraint_size = 5;

    if (walking_tick == 1)
        QP_m.InitializeProblemSize(variable_size, constraint_size);

    MatrixXd H, A, W;
    H.setZero(variable_size, variable_size);
    A.setZero(constraint_size, variable_size);
    VectorXd g, lb, ub, lbA, ubA;
    g.setZero(variable_size);

    lb.setZero(variable_size);
    ub.setZero(variable_size);
    lbA.setZero(constraint_size);
    ubA.setZero(constraint_size);

    Eigen::Vector3d q_waistd;
    Eigen::Vector8d q_rarmd, q_larmd;

    H_leg.setZero();
    H_leg = Ag_leg * q_dot_est_mu.head(12) + Ag_waist * q_dot_est_mu.segment(12, 3) + Ag_armL * q_dot_est_mu.segment(15, 8) + Ag_armR * q_dot_est_mu.segment(25, 8);

    Eigen::MatrixXd Ag_temp;
    Eigen::Matrix5d I;
    I.setIdentity();
    double alpha = 0.00;

    Ag_temp.resize(3, 5);
    Ag_temp.block<3, 3>(0, 0) = Ag_waist;
    Ag_temp.block<3, 1>(0, 3) = Ag_armL.block<3, 1>(0, 1);
    Ag_temp.block<3, 1>(0, 4) = Ag_armR.block<3, 1>(0, 1);

    H = Ag_temp.transpose() * Ag_temp;   // + alpha*I;
    g = 2 * Ag_temp.transpose() * H_leg; //- 2*alpha*qd_prev;

    A.setIdentity();

    for (int i = 0; i < 3; i++)
    {
        lbA(i) = (-0.5 - q_w(i)) * wk_Hz;
        ubA(i) = (0.5 - q_w(i)) * wk_Hz;
    }

    lbA(3) = (0.0 - q_w(3)) * wk_Hz;
    ubA(3) = (0.9 - q_w(3)) * wk_Hz;
    lbA(4) = (-0.9 - q_w(4)) * wk_Hz;
    ubA(4) = (0.0 - q_w(4)) * wk_Hz;

    for (int i = 0; i < variable_size; i++)
    {
        lb(i) = -5.0;
        ub(i) = 5.0;
    }

    lb(3) = -0.5;
    lb(4) = -0.5;

    ub(3) = 0.5;
    ub(4) = 0.5;

    QP_m.EnableEqualityCondition(0.001);
    QP_m.UpdateMinProblem(H, g);
    QP_m.UpdateSubjectToAx(A, lbA, ubA);
    QP_m.UpdateSubjectToX(lb, ub);

    debug_temp1 += H_leg(1) / wk_Hz;

    //   q_dm = QP_m.SolveQPoases(100);
    //qd_prev = q_dm;
}