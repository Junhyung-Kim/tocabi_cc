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

#include "cc.h"

using namespace TOCABI;
using namespace pinocchio;
using std::shared_ptr;

pinocchio::Model model;
pinocchio::Data model_data;

CustomController::CustomController(RobotData &rd) : rd_(rd) //, wbc_(dc.wbc_)
{
    ControlVal_.setZero();
    pinocchio::urdf::buildModel("/home/jhk/catkin_ws/src/dyros_tocabi_v2/tocabi_description/robots/dyros_tocabi.urdf", model);
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
        if (rd_.tc_.walking_enable == 1.0)
        {
            /*   q = rd_.q_;
            qdot = rd_.q_dot_;
            
            //Pinocchio model 
            CMM = pinocchio::computeCentroidalMap(model, model_data, q);
            pinocchio::crba(model, model_data, q);
            pinocchio::computeCoriolisMatrix(model, model_data, q, qdot);
            pinocchio::rnea(model, model_data, q, qdot_, qddot_);

            TorqueGrav = WBC::GravityCompensationTorque(rd_);
  */
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
    else if (rd_.tc_.mode == 11)
    {
        if (rd_.tc_.walking_enable == 1.0)
        {
            if (mpc_init == true)
            {
                // maximum element in cost functions
                if (ns[1] > 0 | ns[N] > 0)
                    mu0 = 1000.0;
                else
                    mu0 = 2.0;

                //solver Setup
                int iter_max = 30;
                double alpha_min = 1e-8;
                double tol_stat = 1e-6;
                double tol_eq = 1e-8;
                double tol_ineq = 1e-8;
                double tol_comp = 1e-8;
                double reg_prim = 1e-12;
                int warm_start = 0;
                int pred_corr = 1;
                int ric_alg = 0;
                int comp_res_exit = 1;
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
                
                d_ocp_qp_dim_set_all(nx, nu, nbx, nbu, ng, nsbx, nsbu, nsg, &dimy);
                qp_sizey = d_ocp_qp_memsize(&dimy);
                qp_memy = malloc(qp_sizey);
                d_ocp_qp_create(&dimy, &qpy, qp_memy);
                qp_sol_sizey = d_ocp_qp_sol_memsize(&dimy);
                qp_sol_memy = malloc(qp_sol_sizey);
                ipm_sizey = d_ocp_qp_ipm_ws_memsize(&dimy, &argy);
                ipm_memy = malloc(ipm_sizey);
                d_ocp_qp_ipm_ws_create(&dimy, &argy, &workspacey, ipm_memy);

                mpcModelSetup();
                mpc_init = false;
                std::cout << "MPC INIT" << std::endl;
            }

            start = clock();

            /************************************************
            * box & general constraints
            ************************************************/
            d_lbu0x[0] = -100;
            d_lbu0x[1] = -100;
            d_ubu0x[0] = 100;
            d_ubu0x[1] = 100;

            d_lbx0x[0] = 0.0;
            d_lbx0x[1] = 0.0;
            d_lbx0x[2] = 0.0;
            d_lbx0x[3] = 0.0;
            d_lbx0x[4] = 0.0;
            d_ubx0x[0] = 0.0;
            d_ubx0x[1] = 0.0;
            d_ubx0x[2] = 0.0;
            d_ubx0x[3] = 0.0;
            d_ubx0x[4] = 0.0;

            d_lbu0y[0] = -100;
            d_lbu0y[1] = -100;
            d_ubu0y[0] = 100;
            d_ubu0y[1] = 100;

            d_lbx0y[0] = 0.0;
            d_lbx0y[1] = 0.0;
            d_lbx0y[2] = 0.0;
            d_lbx0y[3] = 0.0;
            d_lbx0y[4] = 0.0;
            d_ubx0y[0] = 0.0;
            d_ubx0y[1] = 0.0;
            d_ubx0y[2] = 0.0;
            d_ubx0y[3] = 0.0;
            d_ubx0y[4] = 0.0;            

            for (ii = 0; ii < ng[0]; ii++)
            {
                if (ii < nu[0] - nb[0]) // input
                {
                    d_lg0x[ii] = -0.5; // umin
                    d_ug0x[ii] = 0.5;  // umax
                    d_lg0y[ii] = -0.5; // umin
                    d_ug0y[ii] = 0.5;  // umax
                }
                else // state
                {
                    d_lg0x[ii] = -4.0; // xmin
                    d_ug0x[ii] = 4.0;  // xmax
                    d_lg0y[ii] = -4.0; // xmin
                    d_ug0y[ii] = 4.0;  // xmax
                }
            }

            d_lbu1x[0] = -100;
            d_lbu1x[1] = -100;
            d_ubu1x[0] = 100;
            d_ubu1x[1] = 100;

            d_lbx1x[0] = -10;
            d_lbx1x[1] = -10;
            d_lbx1x[2] = -0.1;
            d_lbx1x[3] = -10;
            d_lbx1x[4] = -3;
            d_ubx1x[0] = 10;
            d_ubx1x[1] = 10;
            d_ubx1x[2] = 0.25;
            d_ubx1x[3] = 10;
            d_ubx1x[4] = 3;

            d_lbu1y[0] = -100;
            d_lbu1y[1] = -100;
            d_ubu1y[0] = 100;
            d_ubu1y[1] = 100;

            d_lbx1y[0] = -10;
            d_lbx1y[1] = -10;
            d_lbx1y[2] = -0.1;
            d_lbx1y[3] = -10;
            d_lbx1y[4] = -3;
            d_ubx1y[0] = 10;
            d_ubx1y[1] = 10;
            d_ubx1y[2] = 0.25;
            d_ubx1y[3] = 10;
            d_ubx1y[4] = 3;

            for (ii = 0; ii < ng[1]; ii++)
            {
                if (ii < nu[1] - nb[1]) // input
                {
                    d_lg1x[ii] = 0.0; // umin
                    d_ug1x[ii] = 0.0; // umax
                    d_lg1y[ii] = 0.0; // umin
                    d_ug1y[ii] = 0.0; // umax
                }
                else // state
                {
                    d_lg1x[ii] = 0.4; // xmin
                    d_ug1x[ii] = 0.4; // xmax
                    d_lg1y[ii] = 0.4; // xmin
                    d_ug1y[ii] = 0.4; // xmax
                }
            }

            d_lbxNx[0] = 0.1;
            d_lbxNx[1] = 0.1;
            d_lbxNx[2] = 0.1;
            d_lbxNx[3] = 0.1;
            d_lbxNx[4] = 0.1;
            d_ubxNx[0] = 0.1;
            d_ubxNx[1] = 0.1;
            d_ubxNx[2] = 0.1;
            d_ubxNx[3] = 0.1;
            d_ubxNx[4] = 0.1;

            d_lbxNy[0] = 0.1;
            d_lbxNy[1] = 0.1;
            d_lbxNy[2] = 0.1;
            d_lbxNy[3] = 0.1;
            d_lbxNy[4] = 0.1;
            d_ubxNy[0] = 0.1;
            d_ubxNy[1] = 0.1;
            d_ubxNy[2] = 0.1;
            d_ubxNy[3] = 0.1;
            d_ubxNy[4] = 0.1;

            for (ii = 0; ii < ng[N]; ii++)
            {
                d_lgNx[ii] = 0.2; // dmin
                d_ugNx[ii] = 0.2; // dmax
                d_lgNy[ii] = 0.2; // dmin
                d_ugNy[ii] = 0.2; // dmax            
            }

            D1x[1] = 0.0;
            C1x[3] = 1.0;
            C1x[4] = 1.0;
            DNx[1] = 0.0;
            CNx[3] = 1.0;
            CNx[4] = 1.0;

            D1y[1] = 0.0;
            C1y[3] = 1.0;
            C1y[4] = 1.0;
            DNy[1] = 0.0;
            CNy[3] = 1.0;
            CNy[4] = 1.0;

            hCx[0] = C0x;
            hDx[0] = D0x;
            hd_lgx[0] = d_lg0x;
            hd_ugx[0] = d_ug0x;

            hCy[0] = C0y;
            hDy[0] = D0y;
            hd_lgy[0] = d_lg0y;
            hd_ugy[0] = d_ug0y;

            for (ii = 1; ii < N; ii++)
            {
                hd_lgx[ii] = d_lg1x;
                hd_ugx[ii] = d_ug1x;
                hCx[ii] = C1x;
                hDx[ii] = D1x;
                hd_lgy[ii] = d_lg1y;
                hd_ugy[ii] = d_ug1y;
                hCy[ii] = C1y;
                hDy[ii] = D1y;
            }

            hCx[N] = CNx;
            hDx[N] = DNx;
            hd_lgx[N] = d_lgNx;
            hd_ugx[N] = d_ugNx;

            hCy[N] = CNy;
            hDy[N] = DNy;
            hd_lgy[N] = d_lgNy;
            hd_ugy[N] = d_ugNy;

            d_ocp_qp_set_all(hAx, hBx, hbx, hQx, hSx, hRx, hqx, hrx, hidxbx, hd_lbxx, hd_ubxx, hidxbu, hd_lbux, hd_ubux, hCx, hDx, hd_lgx, hd_ugx, hZlx, hZux, hzlx, hzux, hidxs, hd_lsx, hd_usx, &qpx);
            d_ocp_qp_set_all(hAy, hBy, hby, hQy, hSy, hRy, hqy, hry, hidxbx, hd_lbxy, hd_ubxy, hidxbu, hd_lbuy, hd_ubuy, hCy, hDy, hd_lgy, hd_ugy, hZly, hZuy, hzly, hzuy, hidxs, hd_lsy, hd_usy, &qpy);

            start1 = clock();

            d_ocp_qp_sol_create(&dimx, &qp_solx, qp_sol_memx);
            d_ocp_qp_ipm_solve(&qpx, &qp_solx, &argx, &workspacex);
            d_ocp_qp_ipm_get_status(&workspacex, &hpipm_statusx);
            d_ocp_qp_sol_create(&dimy, &qp_soly, qp_sol_memy);
            d_ocp_qp_ipm_solve(&qpy, &qp_soly, &argy, &workspacey);
            d_ocp_qp_ipm_get_status(&workspacey, &hpipm_statusy);
            endt = clock();

            double result, result1;
            result = (double)(endt - start) / CLOCKS_PER_SEC;
            result1 = (double)(endt - start1) / CLOCKS_PER_SEC;

            int nx_max = nx[0];
            for (ii = 1; ii <= N; ii++)
                if (nx[ii] > nx_max)
                    nx_max = nx[ii];
            double *x11x = (double *)malloc(nx_max * sizeof(double));
            double *slx = (double *)malloc(1 * sizeof(double));
            double *sux = (double *)malloc(1 * sizeof(double));

            std::cout << "time" << result << "time1 " << result1 << std::endl;

            if (hpipm_statusx == 0)
            {
                printf("\n -> QP solved!\n");
            }
            else if (hpipm_statusx == 1)
            {
                printf("\n -> Solver failed! Maximum number of iterations reached\n");
            }
            else if (hpipm_statusx == 2)
            {
                printf("\n -> Solver failed! Minimum step lenght reached\n");
            }
            else if (hpipm_statusx == 2)
            {
                printf("\n -> Solver failed! NaN in computations\n");
            }
            else
            {
                printf("\n -> Solver failed! Unknown return flag\n");
            }
/*
            for (ii = 1; ii <= N; ii++)
            {
                std::cout << " ii " << ii << std::endl;
                d_ocp_qp_sol_get_x(ii, &qp_solx, x11x);
                d_ocp_qp_sol_get_sl(ii, &qp_solx, slx);
                d_print_mat(1, nx[ii], x11x, 1);
                std::cout << "sl " << std::endl;
                d_print_mat(1, ns[ii], slx, 1);
            }*/
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

    b_offset.resize(total_step_num + 2);

    capturePoint_offsetx.resize(total_step_num + 3);
    capturePoint_offsety.resize(total_step_num + 3);

    /////// INITIALIZE //////

    capturePoint_offsetx.setZero();
    capturePoint_offsety.setZero();

    for (int i = 0; i < total_step_num + 2; i++)
    {
        b_offset(i) = exp(lipm_w * t_total / Hz_);
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
        zmp_dx(i) = capturePoint_ox(i + 1) / (1 - b_offset(i)) - (b_offset(i) * capturePoint_ox(i)) / (1 - b_offset(i));
        zmp_dy(i) = capturePoint_oy(i + 1) / (1 - b_offset(i)) - (b_offset(i) * capturePoint_oy(i)) / (1 - b_offset(i));
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

    Ax[1] = 2.77 * Ts;
    Ax[23] = 1.0 * Ts;
    Ax[11] = -2.77 * Ts;
    Ax[5] = 1.0 * Ts;

    Ay[0] = 1.0;
    Ay[6] = 1.0;
    Ay[12] = 1.0;
    Ay[18] = 1.0;
    Ay[24] = 1.0;

    Ay[1] = 2.77 * Ts;
    Ay[23] = 1.0 * Ts;
    Ay[11] = -2.77 * Ts;
    Ay[5] = 1.0 * Ts;

    for (ii = 0; ii < nx * nu; ii++)
    {
        Bx[ii] = 0.0;
        By[ii] = 0.0;
    }

    Bx[6] = 1 / (94.23 * 9.81) * Ts;
    Bx[2] = 1.00 * Ts;
    Bx[9] = 1.00 * Ts;

    By[6] = 1 / (94.23 * 9.81) * Ts;
    By[2] = 1.00 * Ts;
    By[9] = 1.00 * Ts;
}

void CustomController::mpcVariableInit()
{
    N = timeHorizon / Ts;
    nx_ = 5;
    nu_ = 2;

    //resize
    ux = (double **)malloc((N + 1) * sizeof(double *));
    xx = (double **)malloc((N + 1) * sizeof(double *));
    lsx = (double **)malloc((N + 1) * sizeof(double *));
    usx = (double **)malloc((N + 1) * sizeof(double *));
    pix = (double **)malloc((N + 1) * sizeof(double *));
    lam_lbx = (double **)malloc((N + 1) * sizeof(double *));
    lam_lgx = (double **)malloc((N + 1) * sizeof(double *));
    lam_ubx = (double **)malloc((N + 1) * sizeof(double *));
    lam_ugx = (double **)malloc((N + 1) * sizeof(double *));
    lam_lsx = (double **)malloc((N + 1) * sizeof(double *));
    lam_usx = (double **)malloc((N + 1) * sizeof(double *));
    
    uy = (double **)malloc((N + 1) * sizeof(double *));
    xy = (double **)malloc((N + 1) * sizeof(double *));
    lsy = (double **)malloc((N + 1) * sizeof(double *));
    usy = (double **)malloc((N + 1) * sizeof(double *));
    piy = (double **)malloc((N + 1) * sizeof(double *));
    lam_lby = (double **)malloc((N + 1) * sizeof(double *));
    lam_lgy = (double **)malloc((N + 1) * sizeof(double *));
    lam_uby = (double **)malloc((N + 1) * sizeof(double *));
    lam_ugy = (double **)malloc((N + 1) * sizeof(double *));
    lam_lsy = (double **)malloc((N + 1) * sizeof(double *));
    lam_usy = (double **)malloc((N + 1) * sizeof(double *));

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
    nbxe = (int *)malloc((N + 1) * sizeof(int));

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
        nbxe[ii] = 0;
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
    nbxe[0] = nx_;
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
        x0x[jj] = 0;
        bx[jj] = 0.0;
        Qx[ii * (nx_ + 1)] = 3.0;
        qx[ii] = 0.0;
        x0y[jj] = 0;
        by[jj] = 0.0;
        Qy[ii * (nx_ + 1)] = 3.0;
        qy[ii] = 0.0;
    }

    for (ii = 0; ii < nu_; ii++)
    {
        rx[ii] = 0.0;
        Rx[ii * (nu_ + 1)] = 50.0;
        ry[ii] = 0.0;
        Ry[ii * (nu_ + 1)] = 50.0;
    }

    for (ii = 0; ii <= N; ii++)
    {
        d_zeros(ux + ii, nu[ii], 1);
        d_zeros(xx + ii, nx[ii], 1);
        d_zeros(lsx + ii, ns[ii], 1);
        d_zeros(usx + ii, ns[ii], 1);
        d_zeros(pix + ii, nx[ii + 1], 1);
        d_zeros(lam_lbx + ii, nb[ii], 1);
        d_zeros(lam_ubx + ii, nb[ii], 1);
        d_zeros(lam_lgx + ii, ng[ii], 1);
        d_zeros(lam_ugx + ii, ng[ii], 1);
        d_zeros(lam_lsx + ii, ns[ii], 1);
        d_zeros(lam_usx + ii, ns[ii], 1);
        d_zeros(uy + ii, nu[ii], 1);
        d_zeros(xy + ii, nx[ii], 1);
        d_zeros(lsy + ii, ns[ii], 1);
        d_zeros(usy + ii, ns[ii], 1);
        d_zeros(piy + ii, nx[ii + 1], 1);
        d_zeros(lam_lby + ii, nb[ii], 1);
        d_zeros(lam_uby + ii, nb[ii], 1);
        d_zeros(lam_lgy + ii, ng[ii], 1);
        d_zeros(lam_ugy + ii, ng[ii], 1);
        d_zeros(lam_lsy + ii, ns[ii], 1);
        d_zeros(lam_usy + ii, ns[ii], 1);
    }

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
        Zl0x[ii] = 0e3;
        Zu0x[ii] = 0e3;
        zl0x[ii] = 1e2;
        zu0x[ii] = 1e2;
        Zl0y[ii] = 0e3;
        Zu0y[ii] = 0e3;
        zl0y[ii] = 1e2;
        zu0y[ii] = 1e2;
        idxs0[ii] = nu[0] + nx[0] + ii;
        d_ls0x[ii] = -0.05; //-1.0;
        d_us0x[ii] = 0.05;
        d_ls0y[ii] = -0.05; //-1.0;
        d_us0y[ii] = 0.05;
    }

    for (ii = 0; ii < ns[1]; ii++)
    {
        Zl1x[ii] = 0e3;
        Zu1x[ii] = 0e3;
        zl1x[ii] = 1e2;
        zu1x[ii] = 1e2;
        Zl1y[ii] = 0e3;
        Zu1y[ii] = 0e3;
        zl1y[ii] = 1e2;
        zu1y[ii] = 1e2;
        idxs1[ii] = nu[1] + nx[1] + ii;
        d_ls1x[ii] = 0.0; //-1.0;
        d_us1x[ii] = 0.0;
        d_ls1y[ii] = 0.0; //-1.0;
        d_us1y[ii] = 0.0;
        ZlNx[ii] = 0e3;
        ZuNx[ii] = 0e3;
        zlNx[ii] = 1e2;
        zuNx[ii] = 1e2;
        ZlNy[ii] = 0e3;
        ZuNy[ii] = 0e3;
        zlNy[ii] = 1e2;
        zuNy[ii] = 1e2;
        idxsN[ii] = nu[N] + nx[N] + ii;
        d_lsNx[ii] = 0.0; //-1.0;
        d_usNx[ii] = 0.0;
        d_lsNy[ii] = 0.0; //-1.0;
        d_usNy[ii] = 0.0;
    }
}

void CustomController::mpcModelSetup()
{
    flyWheelModel(Ts, nx_, nu_, Ax, Bx, Ay, By);

    //MODEL
    hAx[0] = Ax;
    hBx[0] = Bx;
    hbx[0] = bx;
    hQx[0] = Qx;
    hSx[0] = Sx;
    hRx[0] = Rx;
    hqx[0] = qx;
    hrx[0] = rx;
    hAy[0] = Ay;
    hBy[0] = By;
    hby[0] = by;
    hQy[0] = Qy;
    hSy[0] = Sy;
    hRy[0] = Ry;
    hqy[0] = qy;
    hry[0] = ry;
    hidxbx[0] = idxbx0;
    hd_lbxx[0] = d_lbx0x;
    hd_ubxx[0] = d_ubx0x;
    hd_lbxy[0] = d_lbx0y;
    hd_ubxy[0] = d_ubx0y;
    hidxbu[0] = idxbu0;
    hd_lbux[0] = d_lbu0x;
    hd_ubux[0] = d_ubu0x;
    hd_lbuy[0] = d_lbu0y;
    hd_ubuy[0] = d_ubu0y;
    hZlx[0] = Zl0x;
    hZux[0] = Zu0x;
    hzlx[0] = zl0x;
    hzux[0] = zu0x;
    hZly[0] = Zl0y;
    hZuy[0] = Zu0y;
    hzly[0] = zl0y;
    hzuy[0] = zu0y;
    hidxs[0] = idxs0;
    hd_lsx[0] = d_ls0x;
    hd_usx[0] = d_us0x;
    hd_lsy[0] = d_ls0y;
    hd_usy[0] = d_us0y;

    for (ii = 1; ii < N; ii++)
    {
        hAx[ii] = Ax;
        hBx[ii] = Bx;
        hbx[ii] = bx;
        hQx[ii] = Qx;
        hSx[ii] = Sx;
        hRx[ii] = Rx;
        hqx[ii] = qx;
        hrx[ii] = rx;
        hAy[ii] = Ay;
        hBy[ii] = By;
        hby[ii] = by;
        hQy[ii] = Qy;
        hSy[ii] = Sy;
        hRy[ii] = Ry;
        hqy[ii] = qy;
        hry[ii] = ry;
        hidxbx[ii] = idxbx1;
        hd_lbxx[ii] = d_lbx1x;
        hd_ubxx[ii] = d_ubx1x;
        hd_lbxy[ii] = d_lbx1y;
        hd_ubxy[ii] = d_ubx1y;
        hidxbu[ii] = idxbu1;
        hd_lbux[ii] = d_lbu1x;
        hd_ubux[ii] = d_ubu1x;
        hZlx[ii] = Zl1x;
        hZux[ii] = Zu1x;
        hzlx[ii] = zl1x;
        hzux[ii] = zu1x;
        hd_lbuy[ii] = d_lbu1y;
        hd_ubuy[ii] = d_ubu1y;
        hZly[ii] = Zl1y;
        hZuy[ii] = Zu1y;
        hzly[ii] = zl1y;
        hzuy[ii] = zu1y; 
        hidxs[ii] = idxs1;
        hd_lsx[ii] = d_ls1x;
        hd_usx[ii] = d_us1x;
        hd_lsy[ii] = d_ls1y;
        hd_usy[ii] = d_us1y;
    }

    hQx[N] = Qx;
    hSx[N] = Sx;
    hRx[N] = Rx;
    hqx[N] = qx;
    hrx[N] = rx;
    hQx[N] = Qy;
    hSx[N] = Sy;
    hRx[N] = Ry;
    hqx[N] = qy;
    hrx[N] = ry;

    hidxbx[N] = idxbxN;

    hd_lbxx[N] = d_lbxNx;
    hd_ubxx[N] = d_ubxNx;
    hd_lbxy[N] = d_lbxNy;
    hd_ubxy[N] = d_ubxNy;
    hZlx[N] = ZlNx;
    hZux[N] = ZuNx;
    hzlx[N] = zlNx;
    hzux[N] = zuNx;
    hZly[N] = ZlNy;
    hZuy[N] = ZuNy;
    hzly[N] = zlNy;
    hzuy[N] = zuNy;
    hidxs[N] = idxsN;
    hd_lsx[N] = d_lsNx;
    hd_usx[N] = d_usNx;
    hd_lsy[N] = d_lsNy;
    hd_usy[N] = d_usNy;
}