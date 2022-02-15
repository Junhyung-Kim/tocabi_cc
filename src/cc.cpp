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

pinocchio::Model model1;
pinocchio::Data model_data1;

CustomController::CustomController(RobotData &rd) : rd_(rd) //, wbc_(dc.wbc_)
{
    for (int i = 0; i < 2; i++)
    {
        file[i].open(FILE_NAMES[i].c_str(), std::ios_base::out);
    }
    mpc_cycle = 0;
    mpc_cycle_prev = 0;

    rd_.mujoco_dist = false;
    ros::NodeHandle nh;
    nh.getParam("/tocabi_controller/Qx1", Qx1_mpc);
    nh.getParam("/tocabi_controller/Qx2", Qx2_mpc);
    nh.getParam("/tocabi_controller/Qx3", Qx3_mpc);
    nh.getParam("/tocabi_controller/Qx4", Qx4_mpc);
    nh.getParam("/tocabi_controller/Qx5", Qx5_mpc);

    nh.getParam("/tocabi_controller/Rx1", Rx1_mpc);
    nh.getParam("/tocabi_controller/Rx2", Rx2_mpc);

    nh.getParam("/tocabi_controller/Zl0x", Zl0x_mpc);
    nh.getParam("/tocabi_controller/Zu0x", Zu0x_mpc);
    nh.getParam("/tocabi_controller/zl0x", zl0x_mpc);
    nh.getParam("/tocabi_controller/zu0x", zu0x_mpc);

    nh.getParam("/tocabi_controller/Zl1y", Zl1x_mpc);
    nh.getParam("/tocabi_controller/Zu1y", Zu1x_mpc);
    nh.getParam("/tocabi_controller/zl1y", zl1x_mpc);
    nh.getParam("/tocabi_controller/zu1y", zu1x_mpc);

    nh.getParam("/tocabi_controller/ZlNy", ZlNx_mpc);
    nh.getParam("/tocabi_controller/ZuNy", ZuNx_mpc);
    nh.getParam("/tocabi_controller/zlNy", zlNx_mpc);
    nh.getParam("/tocabi_controller/zuNy", zuNx_mpc);

    nh.getParam("/tocabi_controller/Qy1", Qy1_mpc);
    nh.getParam("/tocabi_controller/Qy2", Qy2_mpc);
    nh.getParam("/tocabi_controller/Qy3", Qy3_mpc);
    nh.getParam("/tocabi_controller/Qy4", Qy4_mpc);
    nh.getParam("/tocabi_controller/Qy5", Qy5_mpc);

    nh.getParam("/tocabi_controller/Ry1", Ry1_mpc);
    nh.getParam("/tocabi_controller/Ry2", Ry2_mpc);

    nh.getParam("/tocabi_controller/Zl0y", Zl0y_mpc);
    nh.getParam("/tocabi_controller/Zu0y", Zu0y_mpc);
    nh.getParam("/tocabi_controller/zl0y", zl0y_mpc);
    nh.getParam("/tocabi_controller/zu0y", zu0y_mpc);

    nh.getParam("/tocabi_controller/Zl1y", Zl1y_mpc);
    nh.getParam("/tocabi_controller/Zu1y", Zu1y_mpc);
    nh.getParam("/tocabi_controller/zl1y", zl1y_mpc);
    nh.getParam("/tocabi_controller/zu1y", zu1y_mpc);

    nh.getParam("/tocabi_controller/ZlNy", ZlNy_mpc);
    nh.getParam("/tocabi_controller/ZuNy", ZuNy_mpc);
    nh.getParam("/tocabi_controller/zlNy", zlNy_mpc);
    nh.getParam("/tocabi_controller/zuNy", zuNy_mpc);

    nh.getParam("/tocabi_controller/arp_dl", arp_dl);
    nh.getParam("/tocabi_controller/ark_dl", ark_dl);
    nh.getParam("/tocabi_controller/app_dl", app_dl);
    nh.getParam("/tocabi_controller/apk_dl", apk_dl);

    nh.getParam("/tocabi_controller/kc_r", kc_r);
    nh.getParam("/tocabi_controller/tc_r", tc_r);
    nh.getParam("/tocabi_controller/kc_p", kc_p);
    nh.getParam("/tocabi_controller/tc_p", tc_p);

    nh.getParam("/tocabi_controller/arp_sl", arp_sl);
    nh.getParam("/tocabi_controller/ark_sl", ark_sl);
    nh.getParam("/tocabi_controller/app_sl", app_sl);
    nh.getParam("/tocabi_controller/apk_sl", apk_sl);

    nh.getParam("/tocabi_controller/arp_dr", arp_dr);
    nh.getParam("/tocabi_controller/ark_dr", ark_dr);
    nh.getParam("/tocabi_controller/app_dr", app_dr);
    nh.getParam("/tocabi_controller/apk_dr", apk_dr);

    nh.getParam("/tocabi_controller/arp_sr", arp_sr);
    nh.getParam("/tocabi_controller/ark_sr", ark_sr);
    nh.getParam("/tocabi_controller/app_sr", app_sr);
    nh.getParam("/tocabi_controller/apk_sr", apk_sr);

    nh.getParam("/tocabi_controller/pelv_xp", pelv_xp);
    nh.getParam("/tocabi_controller/pelv_yp", pelv_yp);

    nh.getParam("/tocabi_controller/com_gain", com_gain1);

    nh.getParam("/tocabi_controller/zmp_xp", zmp_xp);
    nh.getParam("/tocabi_controller/zmp_yp", zmp_yp);

    nh.getParam("/tocabi_controller/mobgain1", mobgain1);
    nh.getParam("/tocabi_controller/mobgain2", mobgain2);
    nh.getParam("/tocabi_controller/mobgain3", mobgain3);
    nh.getParam("/tocabi_controller/mobgain4", mobgain4);
    nh.getParam("/tocabi_controller/mobgain5", mobgain5);
    nh.getParam("/tocabi_controller/mobgain6", mobgain6);

    nh.getParam("/tocabi_controller/K_fx", K_fx);
    nh.getParam("/tocabi_controller/T_fx", T_fx);

    nh.getParam("/tocabi_controller/K_fy", K_fy);
    nh.getParam("/tocabi_controller/T_fy", T_fy);

    nh.getParam("/tocabi_controller/K_fz", K_fz);
    nh.getParam("/tocabi_controller/T_fz", T_fz);

    nh.getParam("/tocabi_controller/lmom", lmom);

    nh.getParam("/tocabi_controller/dist", dist);

    mobgain.push_back(mobgain1);
    mobgain.push_back(mobgain2);
    mobgain.push_back(mobgain3);
    mobgain.push_back(mobgain4);
    mobgain.push_back(mobgain5);
    mobgain.push_back(mobgain6);

    nh.getParam("/tocabi_controller/ft", ft_ok);

    x11x_temp.resize(5, 100);
    x11y_temp.resize(5, 100);

    ROS_INFO("MPCQx");
    std::cout << Qx1_mpc << ", " << Qx2_mpc << ", " << Qx3_mpc << ", " << Qx4_mpc << ", " << Qx5_mpc << std::endl;
    ROS_INFO("MPCRx");
    std::cout << Rx1_mpc << ", " << Rx2_mpc << std::endl;
    ROS_INFO("MPCZl1x");
    std::cout << Zl1x_mpc << ", " << Zu1x_mpc << ", " << zl1x_mpc << ", " << zu1x_mpc << std::endl;

    ROS_INFO("MPCQy");
    std::cout << Qy1_mpc << ", " << Qy2_mpc << ", " << Qy3_mpc << ", " << Qy4_mpc << ", " << Qy5_mpc << std::endl;
    ROS_INFO("MPCRy");
    std::cout << Ry1_mpc << ", " << Ry2_mpc << std::endl;
    ROS_INFO("MPCZl1y");
    std::cout << Zl1y_mpc << ", " << Zu1y_mpc << ", " << zl1y_mpc << ", " << zu1y_mpc << std::endl;

    std::cout << "Robot total mass" << rd_.total_mass_ << std::endl;
    ControlVal_.setZero();
    pinocchio::urdf::buildModel("/home/jhk/catkin_ws/src/dyros_tocabi_v2/tocabi_description/robots/dyros_tocabi_with_redhands.urdf", model);
    pinocchio::Data data(model);
    model_data = data;
    q_ = randomConfiguration(model);
    qdot = Eigen::VectorXd::Zero(model.nv);
    qddot = Eigen::VectorXd::Zero(model.nv);
    qdot_ = Eigen::VectorXd::Zero(model.nv);
    qddot_ = Eigen::VectorXd::Zero(model.nv);

    pinocchio::JointModelFreeFlyer root_joint;
    pinocchio::Model model2;
    pinocchio::urdf::buildModel("/home/jhk/catkin_ws/src/dyros_tocabi_v2/tocabi_description/robots/dyros_tocabi_with_redhands.urdf", root_joint, model2);
    pinocchio::Data data1(model2);

    model1 = model2;

    model_data1 = data1;

    q_1 = Eigen::VectorXd::Zero(model1.nq);
    qdot1 = Eigen::VectorXd::Zero(model1.nv);
    qddot1 = Eigen::VectorXd::Zero(model1.nv);
    qdot_1 = Eigen::VectorXd::Zero(model1.nv);
    qddot_1 = Eigen::VectorXd::Zero(model1.nv);

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
            // CMM = pinocchio::computeCentroidalMap(model, model_data, rd_.q_);
            pinocchio::crba(model, model_data, rd_.q_);
            pinocchio::computeCoriolisMatrix(model, model_data, rd_.q_, rd_.q_dot_);
            pinocchio::rnea(model, model_data, rd_.q_, qdot_, qddot_);

            Cor_ = model_data.C;
            G_ = model_data.tau;
            M_ = model_data.M;
            M_.triangularView<Eigen::StrictlyLower>() = model_data.M.transpose().triangularView<Eigen::StrictlyLower>();

            for (int i = 0; i < 6; i++)
            {
                q_1(i) = rd_.q_virtual_(i);
                /*    qdot_1(i) = rd_.q_dot_virtual_(i);
                qddot_1(i) = rd_.q_ddot_virtual_(i);*/
            }

            q_1(6) = rd_.q_virtual_(MODEL_DOF_VIRTUAL);

            for (int i = 0; i < MODEL_DOF; i++)
            {
                q_1(i + 7) = rd_.q_virtual_(i + 6);
                /*  qdot_1(i + 6) = rd_.q_dot_virtual_(i + 6);
                qddot_1(i + 6) = rd_.q_ddot_virtual_(i + 6);*/
            }

            q_dot_virtual_lpf_ = DyrosMath::lpf(rd_.q_dot_virtual_, q_dot_virtual_lpf_, 2000, 3);
            CMM = pinocchio::computeCentroidalMap(model1, model_data1, q_1);

            jointVelocityEstimate();
            velEst_f = true;

            /*
            pinocchio::crba(model1, model_data1, q_1);
            pinocchio::computeCoriolisMatrix(model1, model_data1, q_1, qdot_1);
            pinocchio::computeGeneralizedGravity(model1, model_data1, q_1);
            pinocchio::nonLinearEffects(model1, model_data1, q_1, qdot_1);

            Cor_1 = model_data1.C;
            G_1 = model_data1.g;

            M_1 = model_data1.M;
            M_1.block(6, 6, MODEL_DOF, MODEL_DOF).triangularView<Eigen::StrictlyLower>() = model_data1.M.block(6, 6, MODEL_DOF, MODEL_DOF).transpose().triangularView<Eigen::StrictlyLower>();

            mob(rd_);
*/
            Vector12d fc_redis;
            double fc_ratio = 0.000;
            fc_redis.setZero();

            if (walking_tick >= 1 && wlk_on == true && current_step_num != total_step_num)
            {
                if (contactMode == 1)
                {
                    rd_.ee_[0].contact = 1.0;
                    rd_.ee_[1].contact = 1.0;
                }
                else if (contactMode == 2)
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

            if (wlk_on == true && (mpc_cycle > 0 || rd_.tc_.MPC == false))
            {
                /////ModelUpdate//////
                getRobotState(rd_);

                if (rd_.tc_.MPC == true)
                {
                    cp_mpcx = com_mpcx + com_mpcdx / lipm_w;
                    cp_mpcy = com_mpcy + com_mpcdy / lipm_w;

                    cp_meax = rd_.link_[COM_id].xpos(0) + rd_.link_[COM_id].v(0) / lipm_w;
                    cp_meay = rd_.link_[COM_id].xpos(1) + rd_.link_[COM_id].v(1) / lipm_w;

                    cp_errx = cp_meax - cp_mpcx;
                    cp_erry = cp_meay - cp_mpcy;

                    zmp_delx = 1.2 * cp_errx;
                    zmp_dely = 1.4 * cp_erry;

                    zmp_delx = 0.0;
                    zmp_dely = 0.0;

                    if (walking_tick > 500)
                    {
                        PELV_trajectory_float.translation()(0) = com_mpcx;
                    }
                    else
                    {
                        PELV_trajectory_float.translation()(0) = COM_float_init.translation()(0);
                    }
                    PELV_trajectory_float.translation()(1) = com_mpcy;
                    PELV_trajectory_float.translation()(2) = PELV_float_init.translation()(2);
                    PELV_trajectory_float.linear() = PELV_float_init.linear();
                }

                walkingCompute(rd_);

                zmpCalc(rd_);
                zmpControl(rd_);

                momentumControl(rd_);

                cc_mutex.lock();
                RT_mu = RT_l;
                LT_mu = LT_l;

                for (int i = 0; i < 12; i++)
                {
                    rd_.q_desired(i) = desired_leg_q(i);
                }
                if (rd_.ankleHybrid == true)
                {
                    rd_.q_desired(5) = rd_.q_desired(5);   // + control_input(1);
                    rd_.q_desired(11) = rd_.q_desired(11); // + control_input(3);
                }

                if (walking_tick == 0)
                {
                    for (int i = 12; i < MODEL_DOF; i++)
                    {
                        rd_.q_desired(i) = desired_init_q(i);
                    }
                }
                else
                {
                    if (rd_.tc_.mom == true && walking_tick > 2500)
                    {
                        rd_.q_desired(12) = rd_.q_desired(12) + q_dm(0) / wk_Hz;
                        rd_.q_desired(13) = rd_.q_desired(13) + q_dm(1) / wk_Hz;
                        rd_.q_desired(14) = rd_.q_desired(14) + q_dm(2) / wk_Hz;
                        rd_.q_desired(16) = rd_.q_desired(16) + q_dm(3) / wk_Hz;
                        rd_.q_desired(17) = rd_.q_desired(17) + q_dm(4) / wk_Hz;
                        rd_.q_desired(26) = rd_.q_desired(26) + q_dm(5) / wk_Hz;
                        rd_.q_desired(27) = rd_.q_desired(27) + q_dm(6) / wk_Hz;
                    }
                }
                cc_mutex.unlock();

                if (rd_.tc_.MPC == true)
                {
                    if (walking_tick % mpct == 0)
                    {
                        int mpc_temp;
                        if (walking_tick == 0)
                        {
                            walking_tick++;
                            mpc_temp = mpc_cycle;
                            mpc_cycle_prev = mpc_temp;
                        }
                        else
                        {
                            if (mpc_cycle > mpc_cycle_prev)
                            {
                                walking_tick++;
                                mpc_temp = mpc_cycle;
                                mpc_cycle_prev = mpc_temp;
                            }

                            if (mpc_cycle >= (t_total * (total_step_num + 1) + t_temp - 1 + 30 * N) / mpct)
                            {
                                walking_tick++;
                            }
                        }
                    }
                    else
                    {
                        walking_tick++;
                    }
                }
                else
                {
                    walking_tick++;
                }

           /*    if (walking_tick >= 4351 && walking_tick < 4420 && dist == 1)
                {
                    rd_.mujoco_dist = true;
                }
                else
                {
                    rd_.mujoco_dist = false;
                }
*/
                if(walking_tick >= 4205 && walking_tick < 4255 && dist == 1)
                {
                    rd_.mujoco_dist  = true;
                }
                else
                {
                    rd_.mujoco_dist = false;
                }

                //  file[1] << PELV_trajectory_float_c.translation()(0) << "\t" << PELV_trajectory_float_c.translation()(1) << "\t" << PELV_trajectory_float_c.translation()(2) << "\t" << RF_trajectory_float.translation()(0)<< "\t" << RF_trajectory_float.translation()(1)<< "\t" << RF_trajectory_float.translation()(2)<<std::endl;
                //if (walking_tick % 5 == 0)
                //     file[1] << walking_tick << "\t"<< mpc_cycle << "\t" << zmp_refx(walking_tick) <<"\t" << rd_.link_[COM_id].xpos(0) << "\t" << com_mpcx << "\t" << ZMP_FT_l(0) << "\t" << zmp_mpcx << "\t" <<xL[walking_tick][0]<<"\t" << xU[walking_tick][0] <<"\t"<< rd_.link_[COM_id].v(1) << "\t" << rd_.link_[COM_id].xpos(1) << "\t" <<hpipm_statusy<<"\t" << com_mpcy << "\t" << ZMP_FT_l(1) << "\t" << ZMP_FT(1) << "\t" << zmp_mpcy << "\t" << rd_.link_[COM_id].xpos(2) << "\t" << H_data(3) << "\t" << mom_mpcy << "\t" << H_data(4) << "\t" << mom_mpcx << "\t" << control_input(0) << "\t" << control_input(1) <<std::endl;

                if (walking_tick != walking_tick_prev)
                {
                    walking_tick_prev = walking_tick;
                    if (rd_.tc_.MPC == true)
                    {
                        file[1] << hpipm_statusx << "\t" << PELV_trajectory_float.translation()(0) << "\t" << com_sup(0) << "\t" << comR_sup(0) << "\t" << com_mpcx << "\t" << rd_.link_[COM_id].xpos(0) << "\t" << zmp_mpcx << "\t" << ZMP_FT_l_mu(0) << "\t" << mom_mpcx << "\t" << H_pitch<<"\t" << xL[walking_tick][2] << "\t" << xU[walking_tick][2] << "\t" << xL_mu[mpc_cycle][2] << "\t" << xU_mu[mpc_cycle][2] << "\t" << control_input(0) << "\t" << control_input(1) << "\t" << rd_.q_desired(0) << "\t" << rd_.q_(0) << "\t" << rd_.q_desired(1) << "\t" << rd_.q_(1) << "\t" << rd_.q_desired(2) << "\t" << rd_.q_(2) << "\t" << rd_.q_desired(3) << "\t" << rd_.q_(3) << "\t" << rd_.q_desired(4) << "\t" << rd_.q_(4) << "\t" << rd_.q_desired(5) << "\t" << rd_.q_(5) << std::endl;
                        file[0] << hpipm_statusy << "\t" << PELV_trajectory_float.translation()(0) << "\t" << com_sup(1) << "\t" << comR_sup(1) << "\t" << com_mpcy << "\t" << rd_.link_[COM_id].xpos(1) << "\t" << zmp_mpcy << "\t" << ZMP_FT_l_mu(1) << "\t" << mom_mpcy << "\t" << H_roll<<"\t"<< yL[walking_tick][2] << "\t" << yU[walking_tick][2] << "\t" << rd_.q_desired(0) << "\t" << rd_.q_desired(1) << "\t" << rd_.q_desired(2) << "\t" << rd_.q_desired(3) << "\t" << rd_.q_desired(4) << "\t" << rd_.q_desired(5) << std::endl;
                    }
                    else
                    {
                        file[0] << PELV_trajectory_float.translation()(0) << "\t" << com_refx(walking_tick) << "\t" << rd_.link_[COM_id].xpos(0) << "\t" << rd_.link_[Pelvis].xipos(0) << "\t" << PELV_trajectory_float.translation()(0) << "\t" << com_sup(0) << "\t" << zmp_refx(walking_tick) << "\t" << ZMP_FT_l(0) << std::endl;
                        file[1] << PELV_trajectory_float.translation()(1) << "\t" << com_refy(walking_tick) << "\t" << rd_.link_[COM_id].xpos(1) << "\t" << rd_.link_[Pelvis].xipos(1) << "\t" << PELV_trajectory_float.translation()(1) << "\t" << com_sup(1) << "\t" << zmp_refy(walking_tick) << "\t" << ZMP_FT_l(1) << "\t" << control_input(0) << "\t" << control_input(1) << "\t" << posture_input(0) << "\t" << posture_input(1) << "\t" << rd_.q_desired(0) << "\t" << rd_.q_(0) << "\t" << rd_.q_desired(1) << "\t" << rd_.q_(1) << "\t" << rd_.q_desired(2) << "\t" << rd_.q_(2) << "\t" << rd_.q_desired(3) << "\t" << rd_.q_(3) << "\t" << rd_.q_desired(4) << "\t" << rd_.q_(4) << "\t" << rd_.q_desired(5) << "\t" << rd_.q_(5) << std::endl;
                    }
                }
            }
        }
        else if (rd_.tc_.walking_enable == 3.0)
        {
            wk_Hz = 1000;
            wk_dt = 1 / wk_Hz;
            setInitPose(rd_, desired_init_q);

            cc_mutex.lock();
            for (int i = 0; i < 12; i++)
            {
                rd_.q_desired(i) = desired_leg_q(i);
            }
            if (walking_tick == 0)
            {
                for (int i = 12; i < MODEL_DOF; i++)
                {
                    rd_.q_desired(i) = desired_init_q(i);
                }
            }
            cc_mutex.unlock();

            //file[1] << rd_.link_[COM_id].xpos(0) << "\t" << rd_.link_[COM_id].xpos(1) << "\t" << rd_.link_[COM_id].v(0) << "\t" << rd_.link_[COM_id].v(1) << "\t" << ZMP_FT_l(0) << "\t" << ZMP_FT_l(1) << "\t" << COM_float_current.translation()(0) << "\t" << COM_float_current.translation()(1) << "\t" << COM_float_current.translation()(2) << "\t" << rd_.total_mass_ << "\t" << std::endl;
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
            if (wlk_on == true && mpc_on == false && rd_.tc_.MPC == true)
            {
                mpcModelSetup();

                // maximum element in cost functions
                if (ns[1] > 0 | ns[N] > 0)
                    mu0 = 1000.0;
                else
                    mu0 = 2.0;

                //solver Setup
                int iter_max = 100;
                double alpha_min = 0.01;
                double tol_stat = 0.01;
                double tol_eq = 0.01;   //8e-3;
                double tol_ineq = 0.01; //8e-3;
                double tol_comp = 0.01;
                double reg_prim = 0.01;
                int warm_start = 1;
                int ric_alg = 0;
                enum hpipm_mode mode = BALANCE;

                dim_sizex = d_ocp_qp_dim_memsize(N);
                dim_memx = malloc(dim_sizex);
                d_ocp_qp_dim_create(N, &dimx, dim_memx);
                ipm_arg_sizex = d_ocp_qp_ipm_arg_memsize(&dimx);
                ipm_arg_memx = malloc(ipm_arg_sizex);
                d_ocp_qp_ipm_arg_create(&dimx, &argx, ipm_arg_memx);
                d_ocp_qp_ipm_arg_set_default(mode, &argx);
                /*    d_ocp_qp_ipm_arg_set_mu0(&mu0, &argx);
                */d_ocp_qp_ipm_arg_set_iter_max(&iter_max, &argx);
                /*d_ocp_qp_ipm_arg_set_tol_stat(&tol_stat, &argx);
                d_ocp_qp_ipm_arg_set_tol_eq(&tol_eq, &argx);
                d_ocp_qp_ipm_arg_set_tol_ineq(&tol_ineq, &argx);
                d_ocp_qp_ipm_arg_set_tol_comp(&tol_comp, &argx);
                d_ocp_qp_ipm_arg_set_reg_prim(&reg_prim, &argx);*/
                //d_ocp_qp_ipm_arg_set_warm_start(&warm_start, &argx);
                // d_ocp_qp_ipm_arg_set_ric_alg(&ric_alg, &argx);
                //   d_ocp_qp_ipm_arg_set_split_step(&split_step, &argx);
                //argx.abs_form = 1.0;

                d_ocp_qp_dim_set_all(nx, nu, nbx, nbu, ng, nsbx, nsbu, nsg, &dimx);
                qp_sizex = d_ocp_qp_memsize(&dimx);
                qp_memx = malloc(qp_sizex);
                d_ocp_qp_create(&dimx, &qpx, qp_memx);
                qp_sol_sizex = d_ocp_qp_sol_memsize(&dimx);
                qp_sol_memx = malloc(qp_sol_sizex);
                ipm_sizex = d_ocp_qp_ipm_ws_memsize(&dimx, &argx);
                ipm_memx = malloc(ipm_sizex);
                d_ocp_qp_ipm_ws_create(&dimx, &argx, &workspacex, ipm_memx);

                if (ns[1] > 0 | ns[N] > 0)
                    mu0 = 100.0;
                else
                    mu0 = 2.0;

                mode = BALANCE;

                iter_max = 80;
                alpha_min = 1;
                tol_stat = 0.005;
                tol_eq = 0.01;
                tol_ineq = 0.01;
                tol_comp = 0.01;
                reg_prim = 0.01;
                warm_start = 0;
                ric_alg = 0;

                dim_sizey = d_ocp_qp_dim_memsize(N);
                dim_memy = malloc(dim_sizey);
                d_ocp_qp_dim_create(N, &dimy, dim_memy);
                ipm_arg_sizey = d_ocp_qp_ipm_arg_memsize(&dimy);
                ipm_arg_memy = malloc(ipm_arg_sizey);
                d_ocp_qp_ipm_arg_create(&dimy, &argy, ipm_arg_memy);
                d_ocp_qp_ipm_arg_set_default(mode, &argy);
               // d_ocp_qp_ipm_arg_set_mu0(&mu0, &argy);
               d_ocp_qp_ipm_arg_set_iter_max(&iter_max, &argy);
            /*    d_ocp_qp_ipm_arg_set_tol_stat(&tol_stat, &argy);
                d_ocp_qp_ipm_arg_set_tol_eq(&tol_eq, &argy);
                d_ocp_qp_ipm_arg_set_tol_ineq(&tol_ineq, &argy);
                d_ocp_qp_ipm_arg_set_tol_comp(&tol_comp, &argy);
                d_ocp_qp_ipm_arg_set_reg_prim(&reg_prim, &argy);
                d_ocp_qp_ipm_arg_set_warm_start(&warm_start, &argy);
                d_ocp_qp_ipm_arg_set_ric_alg(&ric_alg, &argy);
                */
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
                //d_ocp_qp_sol_create(&dimx, &qp_solx_temp, qp_sol_memx);
                d_ocp_qp_sol_create(&dimy, &qp_soly, qp_sol_memy);
                std::cout << "MPC INIT" << std::endl;
                mpc_on = true;
            }

            if (wlk_on == true && mpc_s == true && mpc_cycle < (t_total * (total_step_num + 1) + t_temp - 1 + 30 * N) / mpct && rd_.tc_.MPC == true && debug == false)
            {
                /************************************************
                *********box & general constraints**************
                ************************************************/
                if (walking_tick == 0 || (walking_tick % mpct == 0 && mpc_cycle * mpct <= walking_tick + mpct))
                {
                    auto t4 = std::chrono::steady_clock::now();
                    walking_tick_prev = walking_tick;

                    auto t1 = std::chrono::steady_clock::now();
                    mpc_variablex();
                    mpc_variabley();
                    solverx = std::async(std::launch::async, &CustomController::walking_x, this);
                    solvery = std::async(std::launch::async, &CustomController::walking_y, this);
                    solverx.wait();
                    solvery.wait();

                    if (hpipm_statusx != 0)
                    {
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
                        else
                        {
                            printf("\n -> x : Unknown Error\n");
                            std::cout << "mpc" << mpc_cycle << std::endl;
                        }
                    }

                    if (hpipm_statusy != 0)
                    {
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
                        else
                        {
                            printf("\n -> y : Unknown Error\n");
                            std::cout << "mpc" << mpc_cycle << std::endl;
                        }
                    }

                    if (mpc_cycle != 0)
                    {
                        mom_mpcx_prev = mom_mpcx;
                        mom_mpcy_prev = mom_mpcy;
                    }

                    auto t6 = std::chrono::steady_clock::now();

                    com_mpcx = x11x[0];
                    com_mpcdx = x11x[1];
                    zmp_mpcx = x11x[2];
                    
                    double ab;
                    ab = H_pitch;
                    mom_mpcx = x11x[4];

                    /*  if (mpct1 != 1 || mpct1_prev != 1)
                        mom_mpcx = (0.05 * (-softCx_s[mpc_cycle][0] * x11x[0] - softCx_s[mpc_cycle][1] * x11x[1] + softBoundx_s[mpc_cycle][0]) + 0.95 * H_pitch); //x11x[4];
                    else
                        mom_mpcx = x11x[4]; //-1.2*(softCx_s[mpc_cycle][0] * x11x[0] + softCx_s[mpc_cycle][1] * x11x[1] - softBoundx_s[mpc_cycle][0]);//x11x[4];
                    */
                    com_mpcy = x11y[0];
                    com_mpcdy = x11y[1];
                    zmp_mpcy = x11y[2];
                    mom_mpcy = x11y[4];

                    /*if (mpct2 != 1 || mpct2_prev != 1)
                        mom_mpcy = (0.05 * (-softCy_s[mpc_cycle][0] * x11y[0] - softCy_s[mpc_cycle][1] * x11y[1] + softBoundy_s[mpc_cycle][0]) + 0.95 * H_roll); //x11x[4];
                    else
                        mom_mpcy = x11y[4];*/

                    if (mpc_cycle == 0)
                    {
                        mom_mpcx_prev = mom_mpcx;
                        mom_mpcy_prev = mom_mpcy;
                    }

                    mot_mpcx = (mom_mpcx - mom_mpcx_prev) / Ts;
                    mot_mpcy = (mom_mpcy - mom_mpcy_prev) / Ts;

                    auto t5 = std::chrono::steady_clock::now();
                    auto d1 = std::chrono::duration_cast<std::chrono::microseconds>(t5 - t4).count();

                    if (d1 >= 10000)
                    {
                        //   std::cout << d1 << std::endl;
                        std::cout << "mpc " << mpc_cycle << " : " << d1 << std::endl;
                    }

                    mpc_cycle++;
                }
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
        ZMP_FT = WBC::GetZMPpos_fromFT(rd_);
        d_lbx0x[0] = rd_.link_[COM_id].xpos(0); //om_refx(0);//rd_.link_[COM_id].xpos(0);
        d_lbx0x[1] = 0.0;
        d_lbx0x[2] = ZMP_FT(0);
        d_lbx0x[3] = 0.0;
        d_lbx0x[4] = 0.0;
        d_ubx0x[0] = rd_.link_[COM_id].xpos(0); //com_refx(0);//rd_.link_[COM_id].xpos(0);
        d_ubx0x[1] = 0.0;
        d_ubx0x[2] = ZMP_FT(0);
        d_ubx0x[3] = 0.0;
        d_ubx0x[4] = 0.0;

        hd_lbxx[0] = d_lbx0x;
        hd_ubxx[0] = d_ubx0x;
    }
    else
    {
        x11x[0] = rd_.link_[COM_id].xpos(0);
        x11x[1] = rd_.link_[COM_id].v(0);
        x11x[2] = ZMP_FT_l_mu(0);
      /*  if(mpc_cycle > 300)
        {
            x11x[4] = 0.0;
        }
        else
        {*/
            x11x[4] = H_pitch;
        //}
        
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
        x11y[0] = rd_.link_[COM_id].xpos(1);
        x11y[1] = rd_.link_[COM_id].v(1);
        x11y[2] = ZMP_FT_l_mu(1);
        x11y[4] = H_roll;
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
    /*
    if (hpipm_statusx == 0 || mpc_cycle <= 100)
    {
        for (int i = 0; i < N; i++)
        {
            d_ocp_qp_sol_get_x(i, &qp_solx, x11x);
            for (int j = 0; j < 5; j++)
            {
                x11x_temp(j, i) = x11x[j];
            }
        }
        d_ocp_qp_sol_get_x(1, &qp_solx, x11x);
        if (mpct1 == 1)
        {
            mpct1_prev = 1;
        }
        mpct1 = 1;
    }
    else if (mpc_cycle > 100)
    {
        mpct1 = mpct1 + 1;
        for (int j = 0; j < 5; j++)
        {
            x11x[j] = x11x_temp(j, mpct1);
        }
        mpct1_prev = mpct1;
    }*/
}

void CustomController::walking_y()
{
    d_ocp_qp_set_all(hAy, hBy, hby, hQy, hSy, hRy, hqy, hry, hidxbx, hd_lbxy, hd_ubxy, hidxbu, hd_lbuy, hd_ubuy, hCy, hDy, hd_lgy, hd_ugy, hZly, hZuy, hzly, hzuy, hidxs, hd_lsy, hd_usy, &qpy);
    d_ocp_qp_ipm_solve(&qpy, &qp_soly, &argy, &workspacey);
    d_ocp_qp_ipm_get_status(&workspacey, &hpipm_statusy);
    d_ocp_qp_sol_get_x(1, &qp_soly, x11y);
    /*if(hpipm_statusy != 0)
    {
        mpct2 = mpct2 +1;
        mpct2_prev = mpct2;
    }
    else
    {
        if(mpct2 == 1)
        {
            mpct2_prev = 1;
        }
        mpct2 = 1;
    }
    if (hpipm_statusy == 0 || mpc_cycle <= 100)
    {
        for (int i = 0; i < N; i++)
        {
            d_ocp_qp_sol_get_x(i, &qp_soly, x11y);
            for (int j = 0; j < 5; j++)
            {
                x11y_temp(j, i) = x11y[j];
            }
        }
        d_ocp_qp_sol_get_x(1, &qp_soly, x11y);
        if (mpct2 == 1)
        {
            mpct2_prev = 1;
        }
        mpct2 = 1;
    }
    else if (mpc_cycle > 100)
    {
        mpct2 = mpct2 + 1;
        for (int j = 0; j < 5; j++)
        {
            x11y[j] = x11y_temp(j, mpct2);
        }
        mpct2_prev = mpct2;
    }
    */
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
    L = 0.0027;
    L1 = 0.0027;

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

    Bx[6] = -1.0 / (total_mass * zc) * Ts;
    Bx[2] = 1.00 * Ts;
    Bx[9] = 1.00 * Ts;

    By[6] = 1.0 / (total_mass * zc) * Ts;
    By[2] = 1.00 * Ts;
    By[9] = 1.00 * Ts;
}

void CustomController::mpcVariableInit()
{
    N = timeHorizon / Ts;
    std::cout << "N  :   " << N << std::endl;
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
        qx[jj] = 0.0;
        x0y[jj] = 0.0;
        by[jj] = 0.0;
        qy[jj] = 0.0;
    }

    Qy[0] = Qy1_mpc;
    Qy[1 * (nx_ + 1)] = Qy2_mpc;
    Qy[2 * (nx_ + 1)] = Qy3_mpc;
    Qy[3 * (nx_ + 1)] = Qy4_mpc;
    Qy[4 * (nx_ + 1)] = Qy5_mpc;

    Qx[0] = Qx1_mpc;
    Qx[1 * (nx_ + 1)] = Qx2_mpc;
    Qx[2 * (nx_ + 1)] = Qx3_mpc;
    Qx[3 * (nx_ + 1)] = Qx4_mpc;
    Qx[4 * (nx_ + 1)] = Qx5_mpc;

    for (ii = 0; ii < nu_; ii++)
    {
        rx[ii] = 0.0;
        ry[ii] = 0.0;
    }

    Rx[0] = Rx1_mpc;
    Rx[1 * (nu_ + 1)] = Rx2_mpc;
    Ry[0] = Ry1_mpc;
    Ry[1 * (nu_ + 1)] = Ry2_mpc;

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
        Zl0x[ii] = Zl0x_mpc;
        Zu0x[ii] = Zu0x_mpc;
        zl0x[ii] = zl0x_mpc;
        zu0x[ii] = zu0x_mpc;
        Zl0y[ii] = Zl0y_mpc;
        Zu0y[ii] = Zu0y_mpc;
        zl0y[ii] = zl0x_mpc;
        zu0y[ii] = zu0x_mpc;
        idxs0[ii] = nu[0] + nx[0] + ii;
        d_ls0x[ii] = 0.00;
        d_us0x[ii] = 0.00;
        d_ls0y[ii] = 0.00;
        d_us0y[ii] = 0.00;
    }

    for (ii = 0; ii < ns[1]; ii++)
    {
        Zl1x[ii] = Zl1x_mpc;
        Zu1x[ii] = Zu1x_mpc;
        zl1x[ii] = zl1x_mpc;
        zu1x[ii] = zu1x_mpc;
        Zl1y[ii] = Zl1y_mpc;
        Zu1y[ii] = Zu1y_mpc;
        zl1y[ii] = zl1y_mpc;
        zu1y[ii] = zu1y_mpc;
        idxs1[ii] = nu[1] + nx[1] + ii;
        d_ls1x[ii] = 0.0; //-1.0;
        d_us1x[ii] = 0.0;
        d_ls1y[ii] = 0.0; //-1.0;
        d_us1y[ii] = 0.0;
        ZlNx[ii] = ZlNx_mpc;
        ZuNx[ii] = ZuNx_mpc;
        zlNx[ii] = zlNx_mpc;
        zuNx[ii] = zuNx_mpc;
        ZlNy[ii] = ZlNy_mpc;
        ZuNy[ii] = ZuNy_mpc;
        zlNy[ii] = zlNy_mpc;
        zuNy[ii] = zuNy_mpc;
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
    d_lbu0x[0] = -1.0;
    d_lbu0x[1] = -10;
    d_ubu0x[0] = 1.0;
    d_ubu0x[1] = 10;

    d_lbu0y[0] = -3.2;
    d_lbu0y[1] = -15;
    d_ubu0y[0] = 3.2;
    d_ubu0y[1] = 15;

    d_lbu1x[0] = -1.0;
    d_lbu1x[1] = -10;
    d_ubu1x[0] = 1.0;
    d_ubu1x[1] = 10;

    d_lbu1y[0] = -3.2;
    d_lbu1y[1] = -15;
    d_ubu1y[0] = 3.2;
    d_ubu1y[1] = 15;

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

    variable_size = 7;
    constraint_size = 7;

    if (walking_tick == 0)
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

    Eigen::Vector6d H_leg_ref;

    H_leg.setZero();
    H_leg_ref.setZero();

    if (Robot.tc_.MPC == true)
    {
        H_leg_ref(3) = mom_mpcy;
        H_leg_ref(4) = mom_mpcx;

        H_leg_ref(0) = rd_.total_mass_ * com_mpcdx;
        H_leg_ref(1) = rd_.total_mass_ * com_mpcdy;

        Eigen::Vector6d H_leg_1;
        H_leg_1 = Ag_leg * q_dot_est_mu.head(12) + Ag_waist * q_dot_est_mu.segment(12, 3) + Ag_armL * q_dot_est_mu.segment(15, 8) + Ag_armR * q_dot_est_mu.segment(25, 8);
        Eigen::Vector6d h_temp;
        h_temp = Ag_v * q_dot_virtual_lpf_.segment<6>(0);

        H_data.segment<2>(0) = H_leg_1.segment<2>(0); // + h_temp.segment<2>(0);
        H_data.segment<3>(3) = H_leg_1.segment<3>(3) + h_temp.segment<3>(3);

        H_roll = H_leg_1(3);
        H_pitch = H_leg_1(4);

        H_leg1 = (Ag_waist * q_dot_est_mu.segment(12, 3) + Ag_armL * q_dot_est_mu.segment(15, 8) + Ag_armR * q_dot_est_mu.segment(25, 8)).segment<3>(3);

        // Hl_leg(0) = H_roll - H_leg_ref(0);
        // Hl_leg(1) = H_pitch - H_leg_ref(1);

        H_leg(0) = H_roll - H_leg_ref(3);
        H_leg(1) = H_pitch - H_leg_ref(4);
        H_leg(2) = H_leg_1(5);
    }
    else
    {
        // H_leg = Ag_leg * q_dot_est_mu.head(12) + Ag_waist * q_dot_est_mu.segment(12, 3) + Ag_armL * q_dot_est_mu.segment(15, 8) + Ag_armR * q_dot_est_mu.segment(25, 8);
    }

    F_ref(0) = lipm_w * lipm_w * Robot.total_mass_ * (com_mpcx - zmp_mpcx) - mom_mpcy / zc;
    F_ref(1) = lipm_w * lipm_w * Robot.total_mass_ * (com_mpcy - zmp_mpcy) + mom_mpcx / zc;

    F_cur(0) = lipm_w * lipm_w * Robot.total_mass_ * (Robot.link_[COM_id].xpos(0) - ZMP_FT(0)) - H_pitch / zc;
    F_cur(1) = lipm_w * lipm_w * Robot.total_mass_ * (Robot.link_[COM_id].xpos(1) - ZMP_FT(1)) + H_roll / zc;

    F_err = F_cur - F_ref;
    if (walking_tick == 0)
    {
        F_err_l = F_err.segment<2>(0);
    }

    for (int i = 0; i < 2; i++)
    {
        F_err_l(i) = DyrosMath::lowPassFilter(F_err(i), F_err_l(i), 1 / wk_Hz, 1 / (2 * 3.14 * 3));
    }

    Eigen::MatrixXd Ag_temp;
    Eigen::MatrixXd Agl_temp;
    Eigen::Matrix7d I;
    Eigen::Matrix7d alpha;
    Eigen::Vector7d qd_prev;

    I.setIdentity();
    alpha.setIdentity();
    alpha = 0.05 * alpha;
    alpha(3, 3) = 0.001;
    alpha(4, 4) = 0.001;
    alpha(5, 5) = 0.001;
    alpha(6, 6) = 0.001;

    Ag_temp.resize(3, variable_size);
    Agl_temp.resize(2, variable_size);
    Agl_temp.block<2, 3>(0, 0).setZero() = Ag_waist.block<2, 3>(0, 0);
    Agl_temp.block<2, 2>(0, 3).setZero() = Ag_armL.block<2, 2>(0, 2);
    Agl_temp.block<2, 2>(0, 5).setZero() = Ag_armR.block<2, 2>(0, 2);

    Ag_temp.block<3, 3>(0, 0) = Ag_waist.block<3, 3>(3, 0);
    Ag_temp.block<3, 2>(0, 3) = Ag_armL.block<3, 2>(3, 2);
    Ag_temp.block<3, 2>(0, 5) = Ag_armR.block<3, 2>(3, 2);

    if (walking_tick == 0)
    {
        qd_prev.setZero();
    }

    q_w(0) = Robot.q_desired(12);
    q_w(1) = Robot.q_desired(13);
    q_w(2) = Robot.q_desired(14);

    q_w(3) = Robot.q_desired(16);
    q_w(4) = Robot.q_desired(17);
    q_w(5) = Robot.q_desired(26);
    q_w(6) = Robot.q_desired(27);

    H = Ag_temp.transpose() * Ag_temp + alpha * I + lmom * Agl_temp.transpose() * Agl_temp;
    g = 2 * Ag_temp.transpose() * H_leg + lmom * 2 * Agl_temp.transpose() * Hl_leg; // - 2*alpha*qd_prev;

    A.setIdentity();

    for (int i = 0; i < 3; i++)
    {
        lbA(i) = (-0.3 - q_w(i)) * wk_Hz;
        ubA(i) = (0.3 - q_w(i)) * wk_Hz;
    }

    lbA(3) = (-0.6 - q_w(3)) * wk_Hz;
    ubA(3) = (1.2 - q_w(3)) * wk_Hz;
    lbA(4) = (1.2 - q_w(4)) * wk_Hz;
    ubA(4) = (1.8 - q_w(4)) * wk_Hz;
    lbA(5) = (-1.2 - q_w(5)) * wk_Hz;
    ubA(5) = (0.6 - q_w(5)) * wk_Hz;
    lbA(6) = (-1.8 - q_w(6)) * wk_Hz;
    ubA(6) = (-1.2 - q_w(6)) * wk_Hz;

    for (int i = 0; i < variable_size; i++)
    {
        lb(i) = -1.0;
        ub(i) = 1.0;
    }

    lb(3) = -1.0;
    lb(4) = -1.0;

    ub(3) = 1.0;
    ub(4) = 1.0;

    lb(5) = -1.0;
    lb(6) = -1.0;

    ub(5) = 1.0;
    ub(6) = 1.0;

    if (rd_.tc_.mom == true && walking_tick > 2500)
    {
        QP_m.EnableEqualityCondition(0.005);
        QP_m.UpdateMinProblem(H, g);
        QP_m.UpdateSubjectToAx(A, lbA, ubA);
        QP_m.UpdateSubjectToX(lb, ub);

        q_dm = QP_m.SolveQPoases(100);
        qd_prev = q_dm;
    }
}

void CustomController::zmpCalc(RobotData &Robot)
{
    if (walking_tick == 0)
    {
        pelv_tmp = rd_.link_[Pelvis].xpos(0);
        pelv_lp = rd_.link_[Pelvis].xipos;
        pelv_lp_prev = pelv_lp;
        Fl = Robot.LF_CF_FT;
        Fr = Robot.RF_CF_FT;
        ZMP_FT = WBC::GetZMPpos_fromFT(rd_);
        ZMP_FT_prev = ZMP_FT;
        Fr_prev = Fr;
        Fl_prev = Fl;
    }
    else
    {
        ZMP_FT = WBC::GetZMPpos_fromFT(rd_);
        Fl = Robot.LF_CF_FT;
        Fr = Robot.RF_CF_FT;
    }

    for (int i = 0; i < 3; i++)
        pelv_lp(i) = DyrosMath::lowPassFilter(rd_.link_[Pelvis].xipos(i), pelv_lp_prev(i), 1 / wk_Hz, 1 / (2 * 3.14 * 32));

    for (int i = 0; i < 3; i++)
        ZMP_FT_l(i) = DyrosMath::lowPassFilter(ZMP_FT(i), ZMP_FT_prev(i), 1 / wk_Hz, 1 / (2 * 3.14 * 3));

    for (int i = 0; i < 6; i++)
    {
        Fr_l(i) = DyrosMath::lowPassFilter(Fr(i), Fr_prev(i), 1 / wk_Hz, 1 / (2 * 3.14 * 6));
        Fl_l(i) = DyrosMath::lowPassFilter(Fl(i), Fl_prev(i), 1 / wk_Hz, 1 / (2 * 3.14 * 6));
    }

    Fr_prev = Fr_l;
    Fl_prev = Fl_l;
    ZMP_FT_prev = ZMP_FT_l;

    ZMP_FT(0) = ZMP_FT(0);
    ZMP_FT_l(0) = ZMP_FT_l(0);

    cc_mutex.lock();
    pelv_lp_mu = pelv_lp;
    ZMP_FT_mu = ZMP_FT;
    ZMP_FT_l_mu = ZMP_FT_l;
    Fr_mu = Fr;
    Fl_mu = Fl;
    cc_mutex.unlock();
}

void CustomController::zmpControl(RobotData &Robot)
{
    if (walking_tick > 1 && (mpc_cycle >= 1 || rd_.tc_.MPC == false))
    {
        pr(2) = 0.0;
        pl(2) = 0.0;

        for (int i = 0; i < 2; i++)
        {
            pr(i) = Robot.ee_[1].xpos_contact(i);
            pl(i) = Robot.ee_[0].xpos_contact(i);
        }

        double A, B, C, xi, yi, alpha, Lz, Lz1;
        Eigen::Vector3d desired_ankle_torque, pr_temp, pl_temp;

        int k;
        zmp_delx = 0.0;
        zmp_dely = 0.0;

        if (rd_.tc_.MPC == false)
        {
            if (contactMode == 1)
            {
                A = (pr(1) - pl(1)) / (pr(0) - pl(0));
                B = pl(1) - A * pl(0);
                C = (zmp_refx(walking_tick) + zmp_delx) / A + (zmp_refy(walking_tick) + zmp_dely);
                xi = (C - B) / (A + 1 / A);
                yi = A * xi + B;

                if (yi > pl(1))
                {
                    xi = pl(0);
                    yi = pl(1);
                }
                else if (yi < pr(1))
                {
                    xi = pr(0);
                    yi = pr(1);
                }

                pl_temp(0) = pl(0) - (zmp_refx(walking_tick) + zmp_delx); //zmp_refx(walking_tick);
                pl_temp(1) = pl(1) - (zmp_refy(walking_tick) + zmp_dely); //zmp_refy(walking_tick);
                pl_temp(2) = 0.0;

                pr_temp(0) = pr(0) - (zmp_refx(walking_tick) + zmp_delx); //zmp_refx(walking_tick);
                pr_temp(1) = pr(1) - (zmp_refy(walking_tick) + zmp_dely); //zmp_refy(walking_tick);
                pl_temp(2) = 0.0;

                Lz = sqrt((pr(0) - pl(0)) * (pr(0) - pl(0)) + (pr(1) - pl(1)) * (pr(1) - pl(1)));
                Lz1 = sqrt((xi - pl(0)) * (xi - pl(0)) + (yi - pl(1)) * (yi - pl(1)));
                alpha = Lz1 / Lz;

                if (alpha > 1)
                {
                    alpha = 1;
                }
                else if (alpha < 0)
                {
                    alpha = 0;
                }

                desired_ankle_torque = -DyrosMath::skew(pl_temp) * Fl_l.segment<3>(0) - DyrosMath::skew(pr_temp) * Fr_l.segment<3>(0);
            }
            else if (contactMode == 2)
            {
                alpha = 0.0;

                pl_temp(0) = pl(0) - (zmp_refx(walking_tick) + zmp_delx);
                pl_temp(1) = pl(1) - (zmp_refy(walking_tick) + zmp_dely);
                pl_temp(2) = 0.0;

                desired_ankle_torque = -DyrosMath::skew(pl_temp) * Fl_l.segment<3>(0);
            }
            else
            {
                alpha = 1.0;

                pr_temp(0) = pr(0) - (zmp_refx(walking_tick) + zmp_delx);
                pr_temp(1) = pr(1) - (zmp_refy(walking_tick) + zmp_dely);
                pr_temp(2) = 0.0;

                desired_ankle_torque = -DyrosMath::skew(pr_temp) * Fr_l.segment<3>(0);
            }
        }
        else
        {

            if (contactMode == 1)
            {
                A = (pr(1) - pl(1)) / (pr(0) - pl(0));
                B = pl(1) - A * pl(0);
                C = (zmp_mpcx + zmp_delx) / A + (zmp_mpcy + zmp_dely);
                xi = (C - B) / (A + 1 / A);
                yi = A * xi + B;

                if (yi > pl(1))
                {
                    xi = pl(0);
                    yi = pl(1);
                }
                else if (yi < pr(1))
                {
                    xi = pr(0);
                    yi = pr(1);
                }

                pl_temp(0) = pl(0) - (zmp_mpcx + zmp_delx); //zmp_refx(walking_tick);
                pl_temp(1) = pl(1) - (zmp_mpcy + zmp_dely); //zmp_refy(walking_tick);
                pl_temp(2) = 0.0;

                pr_temp(0) = pr(0) - (zmp_mpcx + zmp_delx); //zmp_refx(walking_tick);
                pr_temp(1) = pr(1) - (zmp_mpcy + zmp_dely); //zmp_refy(walking_tick);
                pl_temp(2) = 0.0;

                Lz = sqrt((pr(0) - pl(0)) * (pr(0) - pl(0)) + (pr(1) - pl(1)) * (pr(1) - pl(1)));
                Lz1 = sqrt((xi - pl(0)) * (xi - pl(0)) + (yi - pl(1)) * (yi - pl(1)));
                alpha = Lz1 / Lz;

                if (alpha > 1)
                {
                    alpha = 1;
                }
                else if (alpha < 0)
                {
                    alpha = 0;
                }

                desired_ankle_torque = -DyrosMath::skew(pl_temp) * Fl_l.segment<3>(0) - DyrosMath::skew(pr_temp) * Fr_l.segment<3>(0);
            }
            else if (contactMode == 2)
            {
                alpha = 0.0;

                pl_temp(0) = pl(0) - (zmp_mpcx + zmp_delx);
                pl_temp(1) = pl(1) - (zmp_mpcy + zmp_dely);
                pl_temp(2) = 0.0;

                desired_ankle_torque = -DyrosMath::skew(pl_temp) * Fl_l.segment<3>(0);
            }
            else
            {
                alpha = 1.0;

                pr_temp(0) = pr(0) - (zmp_mpcx + zmp_delx);
                pr_temp(1) = pr(1) - (zmp_mpcy + zmp_dely);
                pr_temp(2) = 0.0;

                desired_ankle_torque = -DyrosMath::skew(pr_temp) * Fr_l.segment<3>(0);
            }
        }

        for (int i = 0; i < 2; i++)
        {
            if (desired_ankle_torque(i) > 130)
            {
                desired_ankle_torque(i) = 130;
            }

            if (desired_ankle_torque(i) < -130)
            {
                desired_ankle_torque(i) = -130;
            }
        }

        LT(0) = (1 - alpha) * desired_ankle_torque(0);
        RT(0) = (alpha)*desired_ankle_torque(0);

        LT(1) = (1 - alpha) * desired_ankle_torque(1);
        RT(1) = (alpha)*desired_ankle_torque(1);

        dspForceControl(Robot, alpha);

        double arp_l, ark_l, app_l, apk_l, arp_r, ark_r, app_r, apk_r;

        if (contactMode == 1)
        {
            arp_l = arp_dl;
            ark_l = ark_dl;
            app_l = app_dl;
            apk_l = apk_dl;

            arp_r = arp_dr;
            ark_r = ark_dr;
            app_r = app_dr;
            apk_r = apk_dr;
        }
        else if (contactMode == 2)
        {
            arp_l = arp_sl;
            ark_l = ark_sl;
            app_l = app_sl;
            apk_l = apk_sl;

            arp_r = arp_sr;
            ark_r = ark_sr;
            app_r = app_sr;
            apk_r = apk_sr;
        }
        else
        {
            arp_l = arp_sl;
            ark_l = ark_sl;
            app_l = app_sl;
            apk_l = apk_sl;

            arp_r = arp_sr;
            ark_r = ark_sr;
            app_r = app_sr;
            apk_r = apk_sr;
        }

        if (walking_tick == 2)
        {
            LT_prev = LT;
            RT_prev = RT;
        }

        for (int i = 0; i < 2; i++)
        {
            LT_l(i) = DyrosMath::lowPassFilter(LT(i), LT_prev(i), 1 / 1000.0, 1 / (2.0 * 3.14 * 3.0));
            RT_l(i) = DyrosMath::lowPassFilter(RT(i), RT_prev(i), 1 / 1000.0, 1 / (2.0 * 3.14 * 3.0));
        }

        LT_prev = LT_l;
        RT_prev = RT_l;

        if (walking_tick == 2)
        {
            control_input.setZero();
        }

        //  if(walking_tick <= 4200 || walking_tick >=4300)
        //  {
        control_input(0) = apk_l / 1000.0 * (LT(1) - Fl_l(4)) + (1 - app_l / 1000.0) * control_input(0); //pitch
        control_input(1) = ark_l / 1000.0 * (LT(0) - Fl_l(3)) + (1 - arp_l / 1000.0) * control_input(1); //roll
        control_input(2) = apk_r / 1000.0 * (RT(1) - Fr_l(4)) + (1 - app_r / 1000.0) * control_input(2);
        control_input(3) = ark_r / 1000.0 * (RT(0) - Fr_l(3)) + (1 - arp_r / 1000.0) * control_input(3);
        //  }

        if (contactMode == 2)
        {
            control_input(2) = 0.0;
            control_input(3) = 0.0;
        }
        else if (contactMode == 3)
        {
            control_input(0) = 0.0;
            control_input(1) = 0.0;
        }

        posture_input(0) = kc_r / 1000.0 * (-Robot.roll) + (1 - tc_r / 1000.0) * posture_input(0);  //pitch
        posture_input(1) = kc_p / 1000.0 * (-Robot.pitch) + (1 - tc_p / 1000.0) * posture_input(1); //roll

        for (int i = 0; i < 4; i++)
        {
            if (contactMode == 1)
            {
                if (i == 1 || i == 3)
                {
                    if (control_input(i) > 0.05)
                    {
                        control_input(i) = 0.05;
                    }

                    if (control_input(i) < -0.05)
                    {
                        control_input(i) = -0.05;
                    }
                }
                else
                {
                    if (control_input(i) > 0.05)
                    {
                        control_input(i) = 0.05;
                    }

                    if (control_input(i) < -0.05)
                    {
                        control_input(i) = -0.05;
                    }
                }
            }
            else
            {
                if (i == 1 || i == 3)
                {
                    if (control_input(i) > 0.05)
                    {
                        control_input(i) = 0.05;
                    }

                    if (control_input(i) < -0.05)
                    {
                        control_input(i) = -0.05;
                    }
                }
                else
                {
                    if (control_input(i) > 0.05)
                    {
                        control_input(i) = 0.05;
                    }

                    if (control_input(i) < -0.05)
                    {
                        control_input(i) = -0.05;
                    }
                }
            }
        }

        for (int i = 0; i < 2; i++)
        {
            if (posture_input(i) > 0.05)
            {
                posture_input(i) = 0.05;
            }

            if (posture_input(i) < -0.05)
            {
                posture_input(i) = -0.05;
            }
        }
        // file[0] << posture_input(0) << "\t" << posture_input(1) << "\t" << Robot.roll << "\t" << Robot.pitch << std::endl;
        //    file[0] << F_diff(2) << "\t" << F_diff_m(2) << "\t" << contactMode << "\t" << control_input(1) << "\t" << LT(0) << "\t" << Fl_l(3) << "\t" << control_input(3) << "\t" << RT(0) << "\t" << Fr_l(3) << std::endl;
    }
}

void CustomController::dspForceControl(RobotData &Robot, double alpha)
{
    RF_d.setZero();
    LF_d.setZero();
    RF_d(2) = -1 * alpha * Robot.total_mass_ * GRAVITY;
    LF_d(2) = -1 * (1 - alpha) * Robot.total_mass_ * GRAVITY;

    double Kfx, Tfx, Kfy, Tfy, Kfz, Tfz;

    if (phaseChange2 == true && phaseChange3 == false)
    {
        Kfx = DyrosMath::cubic(walking_tick, double2Single_pre1, double2Single1, K_fx, 0.0, 0, 0);
        Tfx = DyrosMath::cubic(walking_tick, double2Single_pre1, double2Single1, T_fx, 0.0, 0, 0);
        Kfy = DyrosMath::cubic(walking_tick, double2Single_pre1, double2Single1, K_fy, 0.0, 0, 0);
        Tfy = DyrosMath::cubic(walking_tick, double2Single_pre1, double2Single1, T_fy, 0.0, 0, 0);
        Kfz = DyrosMath::cubic(walking_tick, double2Single_pre1, double2Single1, K_fz, 0.0, 0, 0);
        Tfz = DyrosMath::cubic(walking_tick, double2Single_pre1, double2Single1, T_fz, 0.0, 0, 0);
    }
    else
    {
        if (contactMode == 1)
        {
            Kfx = K_fx;
            Tfx = T_fx;
            Kfy = K_fy;
            Tfy = T_fy;
            Kfz = K_fz;
            Tfz = T_fz;
        }

        if (walking_tick >= single2Double1 && walking_tick < single2Double1 + 0.05 * wk_Hz)
        {
            Kfx = DyrosMath::cubic(walking_tick, single2Double1, single2Double1 + 0.05 * wk_Hz, 0.0, K_fx, 0, 0);
            Tfx = DyrosMath::cubic(walking_tick, single2Double1, single2Double1 + 0.05 * wk_Hz, 0.0, T_fx, 0, 0);
            Kfy = DyrosMath::cubic(walking_tick, single2Double1, single2Double1 + 0.05 * wk_Hz, 0.0, K_fy, 0, 0);
            Tfy = DyrosMath::cubic(walking_tick, single2Double1, single2Double1 + 0.05 * wk_Hz, 0.0, T_fy, 0, 0);
            Kfz = DyrosMath::cubic(walking_tick, single2Double1, single2Double1 + 0.05 * wk_Hz, 0.0, K_fz, 0, 0);
            Tfz = DyrosMath::cubic(walking_tick, single2Double1, single2Double1 + 0.05 * wk_Hz, 0.0, T_fz, 0, 0);
        }
    }

    F_diff = LF_d - RF_d;
    F_diff_m = Fl_l.segment<3>(0) - Fr_l.segment<3>(0);
    z_ctrl(0) = K_fx / wk_Hz * (F_diff(0) - F_diff_m(0)) + (1 - 1 / (T_fx * wk_Hz)) * z_ctrl(0);
    z_ctrl(1) = K_fy / wk_Hz * (F_diff(1) - F_diff_m(1)) + (1 - 1 / (T_fy * wk_Hz)) * z_ctrl(1);
    z_ctrl(2) = K_fz / wk_Hz * (F_diff(2) - F_diff_m(2)) + (1 - 1 / (T_fz * wk_Hz)) * z_ctrl(2);

    for (int i = 0; i < 3; i++)
    {
        if (z_ctrl(i) > 0.02)
        {
            z_ctrl(i) = 0.02;
        }
        else if (z_ctrl(i) < -0.02)
        {
            z_ctrl(i) = -0.02;
        }
    }
}

void CustomController::mob(RobotData &Robot)
{
    if (walking_tick > 2)
    {
        if (mob_start == false)
        {
            torque_dis_prev.setZero();
            torque_dis.setZero();
            Int_dis.setZero();
            mob_start = true;
        }

        Eigen::Vector12d contactforce;
        Eigen::VectorQd contacttorque;

        if (ft_ok == 0)
        {
            contactforce = WBC::getContactForce(Robot, rd_.torque_desired);
        }
        else
        {
            contactforce.segment(0, 6) = Fl_mu;
            contactforce.segment(6, 6) = Fr_mu;
        }

        if (Robot.ee_[0].contact && Robot.ee_[1].contact)
        {
            contacttorque = (Robot.J_C.transpose() * contactforce).segment<MODEL_DOF>(6);
        }
        else if (Robot.ee_[0].contact)
        {
            contacttorque = (Robot.J_C.transpose() * contactforce.segment(0, 6)).segment<MODEL_DOF>(6);
        }
        else
        {
            contacttorque = (Robot.J_C.transpose() * contactforce.segment(6, 6)).segment<MODEL_DOF>(6);
        }

        Int_dis += ((Cor_1.transpose() * qdot_1).segment<MODEL_DOF>(6) - G_1.segment<MODEL_DOF>(6) - contacttorque + rd_.torque_desired + torque_dis) / 2000.0;

        for (int i = 0; i < MODEL_DOF; i++)
        {
            torque_dis(i) = mobgain[i] * (M_1.block(6, 6, MODEL_DOF, MODEL_DOF) * qdot_1.segment<MODEL_DOF>(6) - Int_dis)(i);
        }

        for (int i = 0; i < MODEL_DOF; i++)
        {
            torque_dis_l(i) = DyrosMath::lowPassFilter(torque_dis(i), torque_dis_prev(i), 1 / 2000.0, 1 / (2.0 * 3.14 * 4.0));
        }

        torque_dis_prev = torque_dis_l;

        torque_est = torque_dis_l + rd_.torque_desired;
        /*
        Eigen::VectorQd torque_m;
        Eigen::VectorVQd nonlinear;
        nonlinear = WBC::CalcNonlinear(Robot);
        torque_m = rd_.A_.block(6,6,MODEL_DOF,MODEL_DOF) * rd_.torque_elmo_ + nonlinear.segment<MODEL_DOF>(6)  + contacttorque;*/
    }
}
