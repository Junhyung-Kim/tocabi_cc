#include "walking.h"

void WalkingController::walkingCompute(RobotData &rd)
{
    /////ModelUpdate//////
    getRobotState(rd);

    if (walking_tick == 0)
    {
        desired_init_q = rd.q_;
    }

    setPelvTrajectory();
    setContactMode();

    RF_trajectory_float.linear() = RF_float_init.linear();
    LF_trajectory_float.linear() = LF_float_init.linear();
    
    RF_trajectory_float.translation()(0) = RFx_trajectory_float(walking_tick);
    RF_trajectory_float.translation()(1) = RFy_trajectory_float(walking_tick);
    RF_trajectory_float.translation()(2) = RFz_trajectory_float(walking_tick);

    LF_trajectory_float.translation()(0) = LFx_trajectory_float(walking_tick);
    LF_trajectory_float.translation()(1) = LFy_trajectory_float(walking_tick);
    LF_trajectory_float.translation()(2) = LFz_trajectory_float(walking_tick);

    inverseKinematics(rd, PELV_trajectory_float, LF_trajectory_float, RF_trajectory_float, desired_leg_q);

    if (dob == 1)
    {
        inverseKinematicsdob(rd);
    }

    cc_mutex.lock();
    for (int i = 0; i < 12; i++)
    {
        rd.q_desired(i) = desired_leg_q(i);
    }
    for (int i = 12; i < MODEL_DOF; i++)
    {
        rd.q_desired(i) = desired_init_q(i);
    }
    cc_mutex.unlock();
    updateNextStepTime(rd);
}

void WalkingController::getRobotState(RobotData &rd)
{
    //////Real Robot Float Frame//////
    COM_float_current.linear() = rd.link_[COM_id].rotm;
    COM_float_current.translation() = rd.link_[COM_id].xpos;
    COMV_support_currentV = rd.link_[COM_id].v;

    RF_float_current.translation() = rd.link_[Right_Foot].xpos;
    RF_float_current.linear() = rd.link_[Right_Foot].rotm;
    LF_float_current.translation() = rd.link_[Left_Foot].xpos;
    LF_float_current.linear() = rd.link_[Left_Foot].rotm;

    PELV_float_current.translation() = rd.link_[Pelvis].xipos;
    PELV_float_current.linear() = rd.link_[Pelvis].rotm;

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

    Agl_leg = Ag_.block(0, 0, 3, 12);
    Agl_armR = Ag_.block(0, 25, 3, 8);
    Agl_armL = Ag_.block(0, 15, 3, 8);
    Agl_waist = Ag_.block(0, 12, 3, 3);
    Ag_leg = Ag_.block(3, 0, 3, 12);
    Ag_armR = Ag_.block(3, 25, 3, 8);
    Ag_armL = Ag_.block(3, 15, 3, 8);
    Ag_waist = Ag_.block(3, 12, 3, 3);
    /*
    yx_vibm(0) = rd.ZMP(0);
    yx_vibm(1) = rd.link_[Pelvis].xipos(0);
    yx_vibm(2) = rd.link_[Pelvis].v(0);

    yy_vibm(0) = rd.ZMP(1);
    yy_vibm(1) = rd.link_[COM_id].xipos(1);
    yy_vibm(2) = rd.link_[COM_id].v(1);
*/
    calcRobotState(rd);
}

void WalkingController::getRobotInitState(RobotData &rd)
{
    if (walking_tick == 0)
    {
        contactMode = 1.0;
        RF_float_init.translation() = rd.link_[Right_Foot].xpos;
        RFx_float_init.translation() = rd.link_[Right_Foot].xipos;
        RF_float_init.linear() = rd.link_[Right_Foot].rotm;
        RFx_float_init.linear() = rd.link_[Right_Foot].rotm;
        LF_float_init.translation() = rd.link_[Left_Foot].xpos;
        LFx_float_init.translation() = rd.link_[Left_Foot].xipos;
        LF_float_init.linear() = rd.link_[Left_Foot].rotm;
        LFx_float_init.linear() = rd.link_[Left_Foot].rotm;

        COM_float_init.translation() = rd.link_[COM_id].xpos;
        COM_float_init.linear() = rd.link_[COM_id].rotm;

        PELV_float_init.translation() = rd.link_[Pelvis].xipos;
        PELV_float_init.linear() = rd.link_[Pelvis].rotm;

        PELV_float_init1.translation() = rd.link_[Pelvis].xpos;
        PELV_float_init1.linear() = rd.link_[Pelvis].rotm;

        HLR_float_init.translation() = rd.link_[4].xpos;
        HLR_float_init.linear() = rd.link_[4].rotm;

        HRR_float_init.translation() = rd.link_[10].xpos;
        HRR_float_init.linear() = rd.link_[10].rotm;

        Eigen::Isometry3d temp;
        temp.linear() = PELV_float_init.linear();
        temp.translation().setZero();
        foot_distance = temp.inverse() * (LF_float_init.translation() - RF_float_init.translation());

        if (foot_step_dir != 1)
        {
            SUF_float_init = RF_float_init;
            SWF_float_init = LF_float_init;
            for (int i = 0; i < 3; i++)
            {
                SUF_float_initV(i) = SUF_float_init.translation()(i);
                SWF_float_initV(i) = SWF_float_init.translation()(i);
            }
            for (int i = 0; i < 3; i++)
            {
                SUF_float_initV(i + 3) = DyrosMath::rot2Euler(SUF_float_init.linear())(i);
                SWF_float_initV(i + 3) = DyrosMath::rot2Euler(SWF_float_init.linear())(i);
            }
        }
        else
        {
            SUF_float_init = LF_float_init;
            SWF_float_init = RF_float_init;
            for (int i = 0; i < 3; i++)
            {
                SUF_float_initV(i) = SUF_float_init.translation()(i);
                SWF_float_initV(i) = SWF_float_init.translation()(i);
            }
            for (int i = 0; i < 3; i++)
            {
                SUF_float_initV(i + 3) = DyrosMath::rot2Euler(SUF_float_init.linear())(i);
                SWF_float_initV(i + 3) = DyrosMath::rot2Euler(SWF_float_init.linear())(i);
            }
        }
        //////Real Robot Support Foot Frame//////
        RF_support_init = DyrosMath::multiplyIsometry3d(DyrosMath::inverseIsometry3d(SUF_float_init), RF_float_init);
        LF_support_init = DyrosMath::multiplyIsometry3d(DyrosMath::inverseIsometry3d(SUF_float_init), LF_float_init);
        PELV_support_init = DyrosMath::inverseIsometry3d(SUF_float_init) * PELV_float_init;
        COM_support_init = DyrosMath::multiplyIsometry3d(PELV_support_init, COM_float_init);

        RF_support_euler_init = DyrosMath::rot2Euler(RF_support_init.linear());
        LF_support_euler_init = DyrosMath::rot2Euler(LF_support_init.linear());
        PELV_support_euler_init = DyrosMath::rot2Euler(PELV_support_init.linear());

        zc = COM_support_init.translation()(2);

        lipm_w = sqrt(GRAVITY / zc);
        total_mass = rd.total_mass_;
    }
}

void WalkingController::footStepGenerator(RobotData &rd)
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

void WalkingController::footStepTotal()
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
    double dlength = step_length_x;
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

void WalkingController::calcRobotState(RobotData &rd)
{
}

void WalkingController::setCpPosition()
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
        b_offset(i) = exp(lipm_w * t_total / wk_Hz);
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

void WalkingController::setFootTrajectory()
{
    if (walking_tick < t_start_real + t_double1)
    {
        LF_trajectory_float.linear() = LF_float_init.linear();
        RF_trajectory_float.linear() = RF_float_init.linear();

        if (current_step_num == 0)
        {
            LF_trajectory_float.translation()(1) = (LF_float_init).translation()(1);
            RF_trajectory_float.translation()(1) = (RF_float_init).translation()(1);

            if (foot_step(current_step_num, 6) == 1)
            {
                LF_trajectory_float.translation()(0) = (LF_float_init).translation()(0);
                RF_trajectory_float.translation()(0) = (RF_float_init).translation()(0);
                LF_trajectory_float.translation()(2) = (LF_float_init).translation()(2);
                RF_trajectory_float.translation()(2) = (RF_float_init).translation()(2);
            }
            else
            {
                RF_trajectory_float.translation()(0) = (RF_float_init).translation()(0);
                LF_trajectory_float.translation()(0) = (LF_float_init).translation()(0);
                LF_trajectory_float.translation()(2) = (LF_float_init).translation()(2);
                RF_trajectory_float.translation()(2) = (RF_float_init).translation()(2);
            }
            /* RF_trajectory_float = PELV_float_init.inverse()* RF_trajectory_float;
            LF_trajectory_float =PELV_float_init.inverse()* LF_trajectory_float;*/
            RF_trajectory_float = RF_trajectory_float;
            LF_trajectory_float = LF_trajectory_float;
        }
        else if (current_step_num == 1)
        {
            if (foot_step(current_step_num, 6) == 1)
            {
                LF_trajectory_float.translation()(0) = foot_step(current_step_num - 1, 0);
                LF_trajectory_float.translation()(1) = foot_step(current_step_num - 1, 1);
            }
            else
            {
                RF_trajectory_float.translation()(0) = foot_step(current_step_num - 1, 0);
                RF_trajectory_float.translation()(1) = foot_step(current_step_num - 1, 1);
            }
        }
        else if (current_step_num == total_step_num - 1)
        {
            if (foot_step(current_step_num, 6) == 1)
            {
                RF_trajectory_float.translation()(0) = foot_step(current_step_num - 2, 0);
                LF_trajectory_float.translation()(0) = foot_step(current_step_num - 1, 0);

                RF_trajectory_float.translation()(1) = foot_step(current_step_num - 2, 1);
                LF_trajectory_float.translation()(1) = foot_step(current_step_num - 1, 1);
            }
            else
            {
                LF_trajectory_float.translation()(0) = foot_step(current_step_num - 2, 0);
                RF_trajectory_float.translation()(0) = foot_step(current_step_num - 1, 0);

                LF_trajectory_float.translation()(1) = foot_step(current_step_num - 2, 1);
                RF_trajectory_float.translation()(1) = foot_step(current_step_num - 1, 1);
            }
        }
        else
        {
            if (foot_step(current_step_num, 6) == 1)
            {
                RF_trajectory_float.translation()(0) = foot_step(current_step_num - 2, 0);
                LF_trajectory_float.translation()(0) = foot_step(current_step_num - 1, 0);

                RF_trajectory_float.translation()(1) = foot_step(current_step_num - 2, 1);
                LF_trajectory_float.translation()(1) = foot_step(current_step_num - 1, 1);
            }
            else
            {
                LF_trajectory_float.translation()(0) = foot_step(current_step_num - 2, 0);
                RF_trajectory_float.translation()(0) = foot_step(current_step_num - 1, 0);

                LF_trajectory_float.translation()(1) = foot_step(current_step_num - 2, 1);
                RF_trajectory_float.translation()(1) = foot_step(current_step_num - 1, 1);
            }
        }
        LFD_trajectory_float.translation().setZero();
        RFD_trajectory_float.translation().setZero();

        contactMode = 1;
    }
    else if (walking_tick >= t_start_real + t_double1 && walking_tick < t_start + t_total - t_double2 - t_rest_last)
    {
        double ankle_temp = 0 * DEG2RAD;

        if (foot_step(current_step_num, 6) == 1)
        {
            if (walking_tick < t_start_real + t_double1 + (t_total - t_rest_init - t_rest_last - t_double1 - t_double2 - t_imp) / 2.0) // the period for lifting the right foot
            {
                RF_trajectory_float.translation()(2) = DyrosMath::QuinticSpline(walking_tick, t_start_real + t_double1 + t_rest_temp, t_start_real + t_double1 + (t_total - t_rest_init - t_rest_last - t_double1 - t_double2 - t_imp) / 2, (RF_float_init).translation()(2), 0.0, 0.0, (RF_float_init).translation()(2) + foot_height, 0.0, 0.0)(0);
                RFD_trajectory_float.translation()(2) = DyrosMath::QuinticSpline(walking_tick, t_start_real + t_double1 + t_rest_temp, t_start_real + t_double1 + (t_total - t_rest_init - t_rest_last - t_double1 - t_double2 - t_imp) / 2, (RF_float_init).translation()(2), 0.0, 0.0, (RF_float_init).translation()(2) + foot_height, 0.0, 0.0)(1) * wk_Hz;

                if (walking_tick > t_start_real + t_double1 + t_rest_temp + 0.015 * wk_Hz)
                {
                    contactMode = 2;
                }
            } // the period for lifting the right foot
            else
            {
                RF_trajectory_float.translation()(2) = DyrosMath::QuinticSpline(walking_tick, t_start_real + t_double1 + (t_total - t_rest_init - t_rest_last - t_double1 - t_double2 - t_imp) / 2.0, t_start + t_total - t_rest_last - t_double2 - t_imp - t_rest_temp, (RF_float_init).translation()(2) + foot_height, 0.0, 0.0, (RF_float_init).translation()(2), 0.0, 0.0)(0);
                RFD_trajectory_float.translation()(2) = DyrosMath::QuinticSpline(walking_tick, t_start_real + t_double1 + (t_total - t_rest_init - t_rest_last - t_double1 - t_double2 - t_imp) / 2.0, t_start + t_total - t_rest_last - t_double2 - t_imp - t_rest_temp, (RF_float_init).translation()(2) + foot_height, 0.0, 0.0, (RF_float_init).translation()(2), 0.0, 0.0)(1) * wk_Hz;

                if (walking_tick < t_start + t_total - t_rest_last - t_double2 - t_imp - t_rest_temp)
                {
                    contactMode = 2;
                }
                else
                {
                    contactMode = 1;
                }
            } // the period for putting the right foot
            if (current_step_num == 0)
            {
                RF_trajectory_float.translation()(0) = DyrosMath::QuinticSpline(walking_tick, t_start_real + t_double1 + 2 * t_rest_temp, t_start + t_total - t_rest_last - t_double2 - t_imp - 2 * t_rest_temp, (RF_float_init).translation()(0), 0.0, 0.0, foot_step(current_step_num, 0), 0.0, 0.0)(0);
                RF_trajectory_float.translation()(1) = DyrosMath::QuinticSpline(walking_tick, t_start_real + t_double1 + 2 * t_rest_temp, t_start + t_total - t_rest_last - t_double2 - t_imp - 2 * t_rest_temp, (RF_float_init).translation()(1), 0.0, 0.0, foot_step(current_step_num, 1), 0.0, 0.0)(0);

                RFD_trajectory_float.translation()(0) = DyrosMath::QuinticSpline(walking_tick, t_start_real + t_double1 + 2 * t_rest_temp, t_start + t_total - t_rest_last - t_double2 - t_imp - 2 * t_rest_temp, (RF_float_init).translation()(0), 0.0, 0.0, foot_step(current_step_num, 0), 0.0, 0.0)(1) * wk_Hz;
                RFD_trajectory_float.translation()(1) = DyrosMath::QuinticSpline(walking_tick, t_start_real + t_double1 + 2 * t_rest_temp, t_start + t_total - t_rest_last - t_double2 - t_imp - 2 * t_rest_temp, (RF_float_init).translation()(1), 0.0, 0.0, foot_step(current_step_num, 1), 0.0, 0.0)(1) * wk_Hz;
            }
            else if (current_step_num == 1)
            {
                RF_trajectory_float.translation()(0) = DyrosMath::QuinticSpline(walking_tick, t_start_real + t_double1 + 2 * t_rest_temp, t_start + t_total - t_rest_last - t_double2 - t_imp - 2 * t_rest_temp, (RF_float_init).translation()(0), 0.0, 0.0, foot_step(current_step_num, 0), 0.0, 0.0)(0);
                RF_trajectory_float.translation()(1) = DyrosMath::QuinticSpline(walking_tick, t_start_real + t_double1 + 2 * t_rest_temp, t_start + t_total - t_rest_last - t_double2 - t_imp - 2 * t_rest_temp, (RF_float_init).translation()(1), 0.0, 0.0, foot_step(current_step_num, 1), 0.0, 0.0)(0);

                RFD_trajectory_float.translation()(0) = DyrosMath::QuinticSpline(walking_tick, t_start_real + t_double1 + 2 * t_rest_temp, t_start + t_total - t_rest_last - t_double2 - t_imp - 2 * t_rest_temp, (RF_float_init).translation()(0), 0.0, 0.0, foot_step(current_step_num, 0), 0.0, 0.0)(1) * wk_Hz;
                RFD_trajectory_float.translation()(1) = DyrosMath::QuinticSpline(walking_tick, t_start_real + t_double1 + 2 * t_rest_temp, t_start + t_total - t_rest_last - t_double2 - t_imp - 2 * t_rest_temp, (RF_float_init).translation()(1), 0.0, 0.0, foot_step(current_step_num, 1), 0.0, 0.0)(1) * wk_Hz;
            }
            else
            {
                RF_trajectory_float.translation()(0) = DyrosMath::QuinticSpline(walking_tick, t_start_real + t_double1 + 2 * t_rest_temp, t_start + t_total - t_rest_last - t_double2 - t_imp - 2 * t_rest_temp, foot_step(current_step_num - 2, 0), 0.0, 0.0, foot_step(current_step_num, 0), 0.0, 0.0)(0);
                RFD_trajectory_float.translation()(0) = DyrosMath::QuinticSpline(walking_tick, t_start_real + t_double1 + 2 * t_rest_temp, t_start + t_total - t_rest_last - t_double2 - t_imp - 2 * t_rest_temp, foot_step(current_step_num - 2, 0), 0.0, 0.0, foot_step(current_step_num, 0), 0.0, 0.0)(1) * wk_Hz;
            }
        }
        else
        {
            if (walking_tick < t_start_real + t_double1 + (t_total - t_rest_init - t_rest_last - t_double1 - t_double2 - t_imp) / 2.0)
            {
                LF_trajectory_float.translation()(2) = DyrosMath::QuinticSpline(walking_tick, t_start_real + t_double1 + t_rest_temp, t_start_real + t_double1 + (t_total - t_rest_init - t_rest_last - t_double1 - t_double2 - t_imp) / 2.0, (LF_float_init).translation()(2), 0.0, 0.0, (LF_float_init).translation()(2) + foot_height, 0.0, 0.0)(0);
                LFD_trajectory_float.translation()(2) = DyrosMath::QuinticSpline(walking_tick, t_start_real + t_double1 + t_rest_temp, t_start_real + t_double1 + (t_total - t_rest_init - t_rest_last - t_double1 - t_double2 - t_imp) / 2.0, (LF_float_init).translation()(2), 0.0, 0.0, (LF_float_init).translation()(2) + foot_height, 0.0, 0.0)(1) * wk_Hz;
                if (walking_tick > t_start_real + t_double1 + t_rest_temp + 0.015 * wk_Hz)
                {
                    contactMode = 3;
                }
            }
            else
            {
                LF_trajectory_float.translation()(2) = DyrosMath::QuinticSpline(walking_tick, t_start_real + t_double1 + (t_total - t_rest_init - t_rest_last - t_double1 - t_double2 - t_imp) / 2.0, t_start + t_total - t_rest_last - t_double2 - t_imp - t_rest_temp, (LF_float_init).translation()(2) + foot_height, 0.0, 0.0, (LF_float_init).translation()(2), 0.0, 0.0)(0);
                LFD_trajectory_float.translation()(2) = DyrosMath::QuinticSpline(walking_tick, t_start_real + t_double1 + (t_total - t_rest_init - t_rest_last - t_double1 - t_double2 - t_imp) / 2.0, t_start + t_total - t_rest_last - t_double2 - t_imp - t_rest_temp, (LF_float_init).translation()(2) + foot_height, 0.0, 0.0, (LF_float_init).translation()(2), 0.0, 0.0)(1) * wk_Hz;
                if (walking_tick < t_start + t_total - t_rest_last - t_double2 - t_imp - t_rest_temp)
                {
                    contactMode = 3;
                }
                else
                {
                    contactMode = 1;
                }
            }
            if (current_step_num == 0)
            {
                LF_trajectory_float.translation()(0) = DyrosMath::QuinticSpline(walking_tick, t_start_real + t_double1 + 2 * t_rest_temp, t_start + t_total - t_rest_last - t_double2 - t_imp - 2 * t_rest_temp, (LF_float_init).translation()(0), 0.0, 0.0, foot_step(current_step_num, 0), 0.0, 0.0)(0);
                LF_trajectory_float.translation()(1) = DyrosMath::QuinticSpline(walking_tick, t_start_real + t_double1 + 2 * t_rest_temp, t_start + t_total - t_rest_last - t_double2 - t_imp - 2 * t_rest_temp, (LF_float_init).translation()(1), 0.0, 0.0, foot_step(current_step_num, 1), 0.0, 0.0)(0);
                LFD_trajectory_float.translation()(0) = DyrosMath::QuinticSpline(walking_tick, t_start_real + t_double1 + 2 * t_rest_temp, t_start + t_total - t_rest_last - t_double2 - t_imp - 2 * t_rest_temp, (LF_float_init).translation()(0), 0.0, 0.0, foot_step(current_step_num, 0), 0.0, 0.0)(1) * wk_Hz;
                LFD_trajectory_float.translation()(1) = DyrosMath::QuinticSpline(walking_tick, t_start_real + t_double1 + 2 * t_rest_temp, t_start + t_total - t_rest_last - t_double2 - t_imp - 2 * t_rest_temp, (LF_float_init).translation()(1), 0.0, 0.0, foot_step(current_step_num, 1), 0.0, 0.0)(1) * wk_Hz;
            }
            else if (current_step_num == 1)
            {
                LF_trajectory_float.translation()(0) = DyrosMath::QuinticSpline(walking_tick, t_start_real + t_double1 + 2 * t_rest_temp, t_start + t_total - t_rest_last - t_double2 - t_imp - 2 * t_rest_temp, (LF_float_init).translation()(0), 0.0, 0.0, foot_step(current_step_num, 0), 0.0, 0.0)(0);
                LF_trajectory_float.translation()(1) = DyrosMath::QuinticSpline(walking_tick, t_start_real + t_double1 + 2 * t_rest_temp, t_start + t_total - t_rest_last - t_double2 - t_imp - 2 * t_rest_temp, (LF_float_init).translation()(1), 0.0, 0.0, foot_step(current_step_num, 1), 0.0, 0.0)(0);
                LFD_trajectory_float.translation()(0) = DyrosMath::QuinticSpline(walking_tick, t_start_real + t_double1 + 2 * t_rest_temp, t_start + t_total - t_rest_last - t_double2 - t_imp - 2 * t_rest_temp, (LF_float_init).translation()(0), 0.0, 0.0, foot_step(current_step_num, 0), 0.0, 0.0)(1) * wk_Hz;
                LFD_trajectory_float.translation()(1) = DyrosMath::QuinticSpline(walking_tick, t_start_real + t_double1 + 2 * t_rest_temp, t_start + t_total - t_rest_last - t_double2 - t_imp - 2 * t_rest_temp, (LF_float_init).translation()(1), 0.0, 0.0, foot_step(current_step_num, 1), 0.0, 0.0)(1) * wk_Hz;
            }
            else
            {
                LF_trajectory_float.translation()(0) = DyrosMath::QuinticSpline(walking_tick, t_start_real + t_double1 + 2 * t_rest_temp, t_start + t_total - t_rest_last - t_double2 - t_imp - 2 * t_rest_temp, foot_step(current_step_num - 2, 0), 0.0, 0.0, foot_step(current_step_num, 0), 0.0, 0.0)(0);
                LFD_trajectory_float.translation()(0) = DyrosMath::QuinticSpline(walking_tick, t_start_real + t_double1 + 2 * t_rest_temp, t_start + t_total - t_rest_last - t_double2 - t_imp - 2 * t_rest_temp, foot_step(current_step_num - 2, 0), 0.0, 0.0, foot_step(current_step_num, 0), 0.0, 0.0)(1) * wk_Hz;
            }
        }
        if (current_step_num == 0)
        {
            RF_trajectory_float.translation()(2) = (RF_float_init).translation()(2);
            LF_trajectory_float.translation()(2) = (LF_float_init).translation()(2);
            RFD_trajectory_float.translation()(2) = 0.0;
            LFD_trajectory_float.translation()(2) = 0.0;

            contactMode = 1;
        }

        if (foot_height == 0.0)
        {
            contactMode = 1;
        }
    }
    else
    {
        if (foot_step(current_step_num, 6) == 1)
        {
            if (current_step_num == 0)
            {
                RF_trajectory_float.translation()(0) = foot_step(current_step_num, 0);
            }
            else if (current_step_num == 1)
            {
                RF_trajectory_float.translation()(0) = foot_step(current_step_num, 0);
            }
            else
            {
                RF_trajectory_float.translation()(0) = foot_step(current_step_num, 0);
            }
            RFD_trajectory_float.translation()(0) = 0.0;
        }
        else if (foot_step(current_step_num, 6) == 0)
        {
            if (current_step_num == 0)
            {
                LF_trajectory_float.translation()(0) = foot_step(current_step_num, 0);
            }
            else if (current_step_num == 1)
            {
                LF_trajectory_float.translation()(0) = foot_step(current_step_num, 0);
            }
            else
            {
                LF_trajectory_float.translation()(0) = foot_step(current_step_num, 0);
            }
            LFD_trajectory_float.translation()(0) = 0.0;
        }
        contactMode = 1;
    }
}

void WalkingController::setContactMode()
{
    if (walking_tick < t_start_real + t_double1)
    {
        contactMode = 1;
    }
    else if (walking_tick >= t_start_real + t_double1 && walking_tick < t_start + t_total - t_double2 - t_rest_last)
    {
        if (foot_step(current_step_num, 6) == 1)
        {
            if (walking_tick < t_start_real + t_double1 + (t_total - t_rest_init - t_rest_last - t_double1 - t_double2 - t_imp) / 2.0) // the period for lifting the right foot
            {
                if (walking_tick > t_start_real + t_double1 + t_rest_temp + 0.015 * wk_Hz)
                {
                    contactMode = 2;
                }
            } // the period for lifting the right foot
            else
            {
                if (walking_tick < t_start + t_total - t_rest_last - t_double2 - t_imp - t_rest_temp)
                {
                    contactMode = 2;
                }
                else
                {
                    contactMode = 1;
                }
            }
        }
        else
        {
            if (walking_tick < t_start_real + t_double1 + (t_total - t_rest_init - t_rest_last - t_double1 - t_double2 - t_imp) / 2.0)
            {
                if (walking_tick > t_start_real + t_double1 + t_rest_temp + 0.015 * wk_Hz)
                {
                    contactMode = 3;
                }
            }
            else
            {
                if (walking_tick < t_start + t_total - t_rest_last - t_double2 - t_imp - t_rest_temp)
                {
                    contactMode = 3;
                }
                else
                {
                    contactMode = 1;
                }
            }
        }
        if (current_step_num == 0)
        {
            contactMode = 1;
        }

        if (foot_height == 0.0)
        {
            contactMode = 1;
        }
    }
    else
    {
        contactMode = 1;
    }
}

void WalkingController::saveFootTrajectory()
{
    LFx_trajectory_float.resize((t_total * (total_step_num + 1) + t_temp - 1));
    RFx_trajectory_float.resize((t_total * (total_step_num + 1) + t_temp - 1));

    LFy_trajectory_float.resize((t_total * (total_step_num + 1) + t_temp - 1));
    RFy_trajectory_float.resize((t_total * (total_step_num + 1) + t_temp - 1));

    LFz_trajectory_float.resize((t_total * (total_step_num + 1) + t_temp - 1));
    RFz_trajectory_float.resize((t_total * (total_step_num + 1) + t_temp - 1));

    LFvx_trajectory_float.resize((t_total * (total_step_num + 1) + t_temp - 1));
    RFvx_trajectory_float.resize((t_total * (total_step_num + 1) + t_temp - 1));

    LFvy_trajectory_float.resize((t_total * (total_step_num + 1) + t_temp - 1));
    RFvy_trajectory_float.resize((t_total * (total_step_num + 1) + t_temp - 1));

    LFvz_trajectory_float.resize((t_total * (total_step_num + 1) + t_temp - 1));
    RFvz_trajectory_float.resize((t_total * (total_step_num + 1) + t_temp - 1));
    debug.resize((t_total * (total_step_num + 1) + t_temp - 1));

    for (int i = 0; i < (t_total * (total_step_num + 1) + t_temp - 1); i++)
    {
        if (i < t_start_real + t_double1)
        {
            LFy_trajectory_float(i) = (LF_float_init).translation()(1);
            RFy_trajectory_float(i) = (RF_float_init).translation()(1);
            LFx_trajectory_float(i) = (LF_float_init).translation()(0);
            RFx_trajectory_float(i) = (RF_float_init).translation()(0);
            LFz_trajectory_float(i) = (LF_float_init).translation()(2);
            RFz_trajectory_float(i) = (RF_float_init).translation()(2);

            LFvx_trajectory_float(i) = 0;
            LFvy_trajectory_float(i) = 0;
            LFvz_trajectory_float(i) = 0;
            RFvx_trajectory_float(i) = 0;
            RFvy_trajectory_float(i) = 0;
            RFvz_trajectory_float(i) = 0;
        }
        else if (i < t_start_real + t_double1 + t_total)
        {
            if (foot_step(1, 6) == 1)
            {
                LFy_trajectory_float(i) = foot_step(0, 1);
                RFy_trajectory_float(i) = (RF_float_init).translation()(1);
                LFx_trajectory_float(i) = foot_step(0, 0);
                RFx_trajectory_float(i) = (RF_float_init).translation()(0);
                LFz_trajectory_float(i) = (LF_float_init).translation()(2);
                RFz_trajectory_float(i) = (RF_float_init).translation()(2);
            }
            else
            {
                LFy_trajectory_float(i) = (LF_float_init).translation()(1);
                RFy_trajectory_float(i) = foot_step(0, 1);
                LFx_trajectory_float(i) = (LF_float_init).translation()(0);
                RFx_trajectory_float(i) = foot_step(0, 0);
                LFz_trajectory_float(i) = (LF_float_init).translation()(2);
                RFz_trajectory_float(i) = (RF_float_init).translation()(2);
            }
            LFvx_trajectory_float(i) = 0;
            LFvy_trajectory_float(i) = 0;
            LFvz_trajectory_float(i) = 0;
            RFvx_trajectory_float(i) = 0;
            RFvy_trajectory_float(i) = 0;
            RFvz_trajectory_float(i) = 0;
        }
        else
        {
            int j = (i - t_temp) / t_total;
            if (j == 1)
            {
                if (i <= t_start_real + t_double1 + t_total * 2)
                {
                    if (foot_step(j, 6) == 1)
                    {
                        LFx_trajectory_float(i) = foot_step(j - 1, 0);
                        LFy_trajectory_float(i) = foot_step(j - 1, 1);
                        LFz_trajectory_float(i) = (LF_float_init).translation()(2);
                        RFy_trajectory_float(i) = (RF_float_init).translation()(1);
                        RFx_trajectory_float(i) = DyrosMath::QuinticSpline(i, t_start_real + t_total + t_double1 + 2 * t_rest_temp, t_start + t_total * 2 - t_rest_last - t_double2 - t_imp - 2 * t_rest_temp, (RF_float_init).translation()(0), 0.0, 0.0, foot_step(j, 0), 0.0, 0.0)(0);
                        RFvx_trajectory_float(i) = DyrosMath::QuinticSpline(i, t_start_real + t_total + t_double1 + 2 * t_rest_temp, t_start + t_total * 2 - t_rest_last - t_double2 - t_imp - 2 * t_rest_temp, (RF_float_init).translation()(0), 0.0, 0.0, foot_step(j, 0), 0.0, 0.0)(1) * wk_Hz;
                        if (i < t_start_real + t_total + t_double1 + (t_total - t_rest_init - t_rest_last - t_double1 - t_double2 - t_imp) / 2.0) // the period for lifting the right foot
                        {
                            RFz_trajectory_float(i) = DyrosMath::QuinticSpline(i, t_start_real + t_total + t_double1 + t_rest_temp, t_start_real + t_total + t_double1 + (t_total - t_rest_init - t_rest_last - t_double1 - t_double2 - t_imp) / 2, (RF_float_init).translation()(2), 0.0, 0.0, (RF_float_init).translation()(2) + foot_height, 0.0, 0.0)(0);
                            RFvz_trajectory_float(i) = DyrosMath::QuinticSpline(i, t_start_real + t_total + t_double1 + t_rest_temp, t_start_real + t_total + t_double1 + (t_total - t_rest_init - t_rest_last - t_double1 - t_double2 - t_imp) / 2, (RF_float_init).translation()(2), 0.0, 0.0, (RF_float_init).translation()(2) + foot_height, 0.0, 0.0)(1) * wk_Hz;
                        }
                        else
                        {
                            RFz_trajectory_float(i) = DyrosMath::QuinticSpline(i, t_start_real + t_total + t_double1 + (t_total - t_rest_init - t_rest_last - t_double1 - t_double2 - t_imp) / 2.0, t_start + t_total + t_total - t_rest_last - t_double2 - t_imp - t_rest_temp, (RF_float_init).translation()(2) + foot_height, 0.0, 0.0, (RF_float_init).translation()(2), 0.0, 0.0)(0);
                            RFvz_trajectory_float(i) = DyrosMath::QuinticSpline(i, t_start_real + t_total + t_double1 + (t_total - t_rest_init - t_rest_last - t_double1 - t_double2 - t_imp) / 2.0, t_start + t_total + t_total - t_rest_last - t_double2 - t_imp - t_rest_temp, (RF_float_init).translation()(2) + foot_height, 0.0, 0.0, (RF_float_init).translation()(2), 0.0, 0.0)(1) * wk_Hz;
                        }

                        LFvx_trajectory_float(i) = 0;
                        LFvy_trajectory_float(i) = 0;
                        LFvz_trajectory_float(i) = 0;
                        RFvy_trajectory_float(i) = 0;
                    }
                    else
                    {
                        RFx_trajectory_float(i) = foot_step(j - 1, 0);
                        RFy_trajectory_float(i) = foot_step(j - 1, 1);
                        RFz_trajectory_float(i) = (RF_float_init).translation()(2);

                        LFy_trajectory_float(i) = (LF_float_init).translation()(1);
                        LFx_trajectory_float(i) = DyrosMath::QuinticSpline(i, t_start_real + t_total + t_double1 + 2 * t_rest_temp, t_start + t_total * 2 - t_rest_last - t_double2 - t_imp - 2 * t_rest_temp, (LF_float_init).translation()(0), 0.0, 0.0, foot_step(j, 0), 0.0, 0.0)(0);
                        LFvx_trajectory_float(i) = DyrosMath::QuinticSpline(i, t_start_real + t_total + t_double1 + 2 * t_rest_temp, t_start + t_total * 2 - t_rest_last - t_double2 - t_imp - 2 * t_rest_temp, (LF_float_init).translation()(0), 0.0, 0.0, foot_step(j, 0), 0.0, 0.0)(1) * wk_Hz;

                        if (i < t_start_real + t_total + t_double1 + (t_total - t_rest_init - t_rest_last - t_double1 - t_double2 - t_imp) / 2.0) // the period for lifting the right foot
                        {
                            LFz_trajectory_float(i) = DyrosMath::QuinticSpline(i, t_start_real + t_total + t_double1 + t_rest_temp, t_start_real + t_total + t_double1 + (t_total - t_rest_init - t_rest_last - t_double1 - t_double2 - t_imp) / 2, (LF_float_init).translation()(2), 0.0, 0.0, (LF_float_init).translation()(2) + foot_height, 0.0, 0.0)(0);
                            LFvz_trajectory_float(i) = DyrosMath::QuinticSpline(i, t_start_real + t_total + t_double1 + t_rest_temp, t_start_real + t_total + t_double1 + (t_total - t_rest_init - t_rest_last - t_double1 - t_double2 - t_imp) / 2, (LF_float_init).translation()(2), 0.0, 0.0, (LF_float_init).translation()(2) + foot_height, 0.0, 0.0)(1) * wk_Hz;
                        }
                        else
                        {
                            LFz_trajectory_float(i) = DyrosMath::QuinticSpline(i, t_start_real + t_total + t_double1 + (t_total - t_rest_init - t_rest_last - t_double1 - t_double2 - t_imp) / 2.0, t_start + t_total + t_total - t_rest_last - t_double2 - t_imp - t_rest_temp, (LF_float_init).translation()(2) + foot_height, 0.0, 0.0, (LF_float_init).translation()(2), 0.0, 0.0)(0);
                            LFvz_trajectory_float(i) = DyrosMath::QuinticSpline(i, t_start_real + t_total + t_double1 + (t_total - t_rest_init - t_rest_last - t_double1 - t_double2 - t_imp) / 2.0, t_start + t_total + t_total - t_rest_last - t_double2 - t_imp - t_rest_temp, (LF_float_init).translation()(2) + foot_height, 0.0, 0.0, (LF_float_init).translation()(2), 0.0, 0.0)(1) * wk_Hz;
                        }

                        RFvx_trajectory_float(i) = 0;
                        RFvy_trajectory_float(i) = 0;
                        RFvz_trajectory_float(i) = 0;
                        LFvy_trajectory_float(i) = 0;
                    }
                }
            }
            else if (j > 1 && j < total_step_num)
            {
                if (i <= t_start + t_double1 + t_total * (j) && i >= t_start + t_total * (j)-1)
                {
                    if (foot_step(j, 6) == 1)
                    {
                        RFx_trajectory_float(i) = foot_step(j - 2, 0);
                        LFx_trajectory_float(i) = foot_step(j - 1, 0);
                        RFy_trajectory_float(i) = foot_step(j - 2, 1);
                        LFy_trajectory_float(i) = foot_step(j - 1, 1);
                        LFz_trajectory_float(i) = (LF_float_init).translation()(2);
                        RFz_trajectory_float(i) = (RF_float_init).translation()(2);
                    }
                    else
                    {
                        LFx_trajectory_float(i) = foot_step(j - 2, 0);
                        RFx_trajectory_float(i) = foot_step(j - 1, 0);
                        LFy_trajectory_float(i) = foot_step(j - 2, 1);
                        RFy_trajectory_float(i) = foot_step(j - 1, 1);
                        RFz_trajectory_float(i) = (RF_float_init).translation()(2);
                        LFz_trajectory_float(i) = (LF_float_init).translation()(2);
                    }

                    LFvx_trajectory_float(i) = 0;
                    LFvy_trajectory_float(i) = 0;
                    LFvz_trajectory_float(i) = 0;
                    RFvx_trajectory_float(i) = 0;
                    RFvy_trajectory_float(i) = 0;
                    RFvz_trajectory_float(i) = 0;
                }
                else if (walking_tick <= t_start_real + t_double1 + t_total * j)
                {
                    if (foot_step(j, 6) == 1)
                    {
                        RFy_trajectory_float(i) = foot_step(j - 2, 1);
                        LFy_trajectory_float(i) = foot_step(j - 1, 1);
                        LFx_trajectory_float(i) = foot_step(j - 1, 0);
                        LFz_trajectory_float(i) = (LF_float_init).translation()(2);
                        RFx_trajectory_float(i) = DyrosMath::QuinticSpline(i, t_start_real + t_total * j + t_double1 + 2 * t_rest_temp, t_start + t_total * (j + 1) - t_rest_last - t_double2 - t_imp - 2 * t_rest_temp, foot_step(j - 2, 0), 0.0, 0.0, foot_step(j, 0), 0.0, 0.0)(0);
                        RFvx_trajectory_float(i) = DyrosMath::QuinticSpline(i, t_start_real + t_total * j + t_double1 + 2 * t_rest_temp, t_start + t_total * (j + 1) - t_rest_last - t_double2 - t_imp - 2 * t_rest_temp, foot_step(j - 2, 0), 0.0, 0.0, foot_step(j, 0), 0.0, 0.0)(1) * wk_Hz;
                        RFvy_trajectory_float(i) = 0;
                        if (i < t_start_real + t_total * j + t_double1 + (t_total - t_rest_init - t_rest_last - t_double1 - t_double2 - t_imp) / 2.0) // the period for lifting the right foot
                        {
                            RFz_trajectory_float(i) = DyrosMath::QuinticSpline(i, t_start_real + t_total * j + t_double1 + t_rest_temp, t_start_real + t_total * j + t_double1 + (t_total - t_rest_init - t_rest_last - t_double1 - t_double2 - t_imp) / 2, (LF_float_init).translation()(2), 0.0, 0.0, (RF_float_init).translation()(2) + foot_height, 0.0, 0.0)(0);
                            RFvz_trajectory_float(i) = DyrosMath::QuinticSpline(i, t_start_real + t_total * j + t_double1 + t_rest_temp, t_start_real + t_total * j + t_double1 + (t_total - t_rest_init - t_rest_last - t_double1 - t_double2 - t_imp) / 2, (LF_float_init).translation()(2), 0.0, 0.0, (RF_float_init).translation()(2) + foot_height, 0.0, 0.0)(1) * wk_Hz;
                        }
                        else
                        {
                            RFz_trajectory_float(i) = DyrosMath::QuinticSpline(i, t_start_real + t_total * j + t_double1 + (t_total - t_rest_init - t_rest_last - t_double1 - t_double2 - t_imp) / 2.0, t_start + t_total * j + t_total - t_rest_last - t_double2 - t_imp - t_rest_temp, (RF_float_init).translation()(2) + foot_height, 0.0, 0.0, (RF_float_init).translation()(2), 0.0, 0.0)(0);
                            RFvz_trajectory_float(i) = DyrosMath::QuinticSpline(i, t_start_real + t_total * j + t_double1 + (t_total - t_rest_init - t_rest_last - t_double1 - t_double2 - t_imp) / 2.0, t_start + t_total * j + t_total - t_rest_last - t_double2 - t_imp - t_rest_temp, (RF_float_init).translation()(2) + foot_height, 0.0, 0.0, (RF_float_init).translation()(2), 0.0, 0.0)(1) * wk_Hz;
                        }
                        LFvx_trajectory_float(i) = 0;
                        LFvy_trajectory_float(i) = 0;
                        LFvz_trajectory_float(i) = 0;
                    }
                    else
                    {
                        LFy_trajectory_float(i) = foot_step(j - 2, 1);
                        RFy_trajectory_float(i) = foot_step(j - 1, 1);
                        RFx_trajectory_float(i) = foot_step(j - 1, 0);
                        RFz_trajectory_float(i) = (RF_float_init).translation()(2);
                        LFx_trajectory_float(i) = DyrosMath::QuinticSpline(i, t_start_real + t_total * j + t_double1 + 2 * t_rest_temp, t_start + t_total * (j + 1) - t_rest_last - t_double2 - t_imp - 2 * t_rest_temp, foot_step(j - 2, 0), 0.0, 0.0, foot_step(j, 0), 0.0, 0.0)(0);
                        LFvx_trajectory_float(i) = DyrosMath::QuinticSpline(i, t_start_real + t_total * j + t_double1 + 2 * t_rest_temp, t_start + t_total * (j + 1) - t_rest_last - t_double2 - t_imp - 2 * t_rest_temp, foot_step(j - 2, 0), 0.0, 0.0, foot_step(j, 0), 0.0, 0.0)(1) * wk_Hz;
                        LFvy_trajectory_float(i) = 0;
                        if (i < t_start_real + t_total * j + t_double1 + (t_total - t_rest_init - t_rest_last - t_double1 - t_double2 - t_imp) / 2.0) // the period for lifting the right foot
                        {
                            LFz_trajectory_float(i) = DyrosMath::QuinticSpline(i, t_start_real + t_total * j + t_double1 + t_rest_temp, t_start_real + t_total * j + t_double1 + (t_total - t_rest_init - t_rest_last - t_double1 - t_double2 - t_imp) / 2, (LF_float_init).translation()(2), 0.0, 0.0, (LF_float_init).translation()(2) + foot_height, 0.0, 0.0)(0);
                            LFvz_trajectory_float(i) = DyrosMath::QuinticSpline(i, t_start_real + t_total * j + t_double1 + t_rest_temp, t_start_real + t_total * j + t_double1 + (t_total - t_rest_init - t_rest_last - t_double1 - t_double2 - t_imp) / 2, (LF_float_init).translation()(2), 0.0, 0.0, (LF_float_init).translation()(2) + foot_height, 0.0, 0.0)(1) * wk_Hz;
                        }
                        else
                        {
                            LFz_trajectory_float(i) = DyrosMath::QuinticSpline(i, t_start_real + t_total * j + t_double1 + (t_total - t_rest_init - t_rest_last - t_double1 - t_double2 - t_imp) / 2.0, t_start + t_total * j + t_total - t_rest_last - t_double2 - t_imp - t_rest_temp, (LF_float_init).translation()(2) + foot_height, 0.0, 0.0, (LF_float_init).translation()(2), 0.0, 0.0)(0);
                            LFvz_trajectory_float(i) = DyrosMath::QuinticSpline(i, t_start_real + t_total * j + t_double1 + (t_total - t_rest_init - t_rest_last - t_double1 - t_double2 - t_imp) / 2.0, t_start + t_total * j + t_total - t_rest_last - t_double2 - t_imp - t_rest_temp, (LF_float_init).translation()(2) + foot_height, 0.0, 0.0, (LF_float_init).translation()(2), 0.0, 0.0)(1) * wk_Hz;
                        }
                        RFvx_trajectory_float(i) = 0;
                        RFvy_trajectory_float(i) = 0;
                        RFvz_trajectory_float(i) = 0;
                    }
                }
            }
            else if (j == total_step_num)
            {
                if (i >= t_start + t_total * (j)-1)
                {
                    if (foot_step(total_step_num - 1, 6) == 1)
                    {
                        RFx_trajectory_float(i) = foot_step(total_step_num - 1, 0);
                        LFx_trajectory_float(i) = foot_step(total_step_num - 2, 0);
                        RFy_trajectory_float(i) = foot_step(total_step_num - 1, 1);
                        LFy_trajectory_float(i) = foot_step(total_step_num - 2, 1);
                    }
                    else
                    {
                        LFx_trajectory_float(i) = foot_step(total_step_num - 1, 0);
                        RFx_trajectory_float(i) = foot_step(total_step_num - 2, 0);
                        LFy_trajectory_float(i) = foot_step(total_step_num - 1, 1);
                        RFy_trajectory_float(i) = foot_step(total_step_num - 2, 1);
                    }
                    LFz_trajectory_float(i) = (LF_float_init).translation()(2);
                    RFz_trajectory_float(i) = (RF_float_init).translation()(2);
                    LFvx_trajectory_float(i) = 0;
                    LFvy_trajectory_float(i) = 0;
                    LFvz_trajectory_float(i) = 0;
                    RFvx_trajectory_float(i) = 0;
                    RFvy_trajectory_float(i) = 0;
                    RFvz_trajectory_float(i) = 0;
                }
            }
        }
    }
}

void WalkingController::cpReferencePatternGeneration()
{
    capturePoint_refx.resize((t_total * (total_step_num + 1) + t_temp - 1));
    capturePoint_refy.resize((t_total * (total_step_num + 1) + t_temp - 1));
    com_refx.resize((t_total * (total_step_num + 1) + t_temp - 1));
    com_refy.resize((t_total * (total_step_num + 1) + t_temp - 1));
    com_refdx.resize((t_total * (total_step_num + 1) + t_temp - 1));
    com_refdy.resize((t_total * (total_step_num + 1) + t_temp - 1));
    zmp_refx.resize((t_total * (total_step_num + 1) + t_temp - 1));
    zmp_refy.resize((t_total * (total_step_num + 1) + t_temp - 1));

    for (int i = 0; i < (t_total * (total_step_num + 1) + t_temp - 1); i++)
    {
        int current_step, capturePointChange;
        double tick;
        if (i < t_temp - 1)
        {
            current_step = i / (t_temp + t_total);
            if (t_temp - t_total <= i)
            {
                tick = (i - (t_temp - t_total - 1)) / wk_Hz;
            }
            else
            {
                tick = i / (wk_Hz);
            }
            capturePointChange = i / (t_temp - 1);
        }
        else
        {
            current_step = (i - t_temp - t_total) / (t_total) + 1;
            capturePointChange = (i - t_temp + 1) / (t_total) + 1;
            tick = i / (wk_Hz)-t_total * (capturePointChange - 1) / wk_Hz - (t_temp - 1) / wk_Hz;
        }
        //ZMP trajectory from CP

        if (!(capturePointChange == total_step_num + 1 && tick > (t_total) / wk_Hz)) //revise
        {
            if (capturePointChange == total_step_num + 2)
            {
                capturePoint_refx(i) = exp(lipm_w * tick) * capturePoint_ox(capturePointChange - 1) + (1 - exp(lipm_w * tick)) * zmp_dx(capturePointChange - 1);
                capturePoint_refy(i) = exp(lipm_w * tick) * capturePoint_oy(capturePointChange - 1) + (1 - exp(lipm_w * tick)) * zmp_dy(capturePointChange - 1);
            }
            else
            {
                capturePoint_refx(i) = exp(lipm_w * tick) * capturePoint_ox(capturePointChange) + (1 - exp(lipm_w * tick)) * zmp_dx(capturePointChange);
                capturePoint_refy(i) = exp(lipm_w * tick) * capturePoint_oy(capturePointChange) + (1 - exp(lipm_w * tick)) * zmp_dy(capturePointChange);
            }
        }
        else
        {
            capturePoint_refx(i) = exp(lipm_w * t_total / wk_Hz) * capturePoint_ox(capturePointChange) + (1 - exp(lipm_w * t_total / wk_Hz)) * zmp_dx(capturePointChange);
            capturePoint_refy(i) = exp(lipm_w * t_total / wk_Hz) * capturePoint_oy(capturePointChange) + (1 - exp(lipm_w * t_total / wk_Hz)) * zmp_dy(capturePointChange);
        }
        if (capturePointChange == 0 && i < t_temp - t_total)
        {
            capturePoint_refx(i) = capturePoint_ox(0);
            capturePoint_refy(i) = capturePoint_oy(0);
        }
        else if (capturePointChange == 0 && t_temp - t_total <= i)
        {
            capturePoint_refx(i) = exp(lipm_w * tick) * capturePoint_ox(capturePointChange) + (1 - exp(lipm_w * tick)) * zmp_dx(capturePointChange);
            capturePoint_refy(i) = exp(lipm_w * tick) * capturePoint_oy(capturePointChange) + (1 - exp(lipm_w * tick)) * zmp_dy(capturePointChange);
        }
        if (i == 0)
        {
            zmp_refx(0) = COM_support_init.translation()(0);
            zmp_refy(0) = COM_float_init.translation()(1);
        }
        else
        {
            zmp_refx(i) = (capturePoint_refx(i - 1)) - (capturePoint_refx(i) - capturePoint_refx(i - 1)) * wk_Hz / (lipm_w);
            zmp_refy(i) = (capturePoint_refy(i - 1)) - (capturePoint_refy(i) - capturePoint_refy(i - 1)) * wk_Hz / (lipm_w);
        }
    }
}

void WalkingController::cptoComTrajectory()
{
    for (int i = 0; i < (t_total * (total_step_num + 1) + t_temp - 1); i++)
    {
        if (i >= t_temp - t_total)
        {
            com_refx(i) = lipm_w / wk_Hz * capturePoint_refx(i) + (1 - lipm_w / wk_Hz) * com_refx(i - 1);
            com_refy(i) = lipm_w / wk_Hz * capturePoint_refy(i) + (1 - lipm_w / wk_Hz) * com_refy(i - 1);
            com_refdx(i) = (com_refx(i) - com_refx(i - 1)) * wk_Hz;
            com_refdy(i) = (com_refy(i) - com_refy(i - 1)) * wk_Hz;
        }
        else
        {
            if (com_control == 0)
            {
                com_refx(i) = (PELV_float_init).translation()(0);
                com_refy(i) = (PELV_float_init).translation()(1);
            }
            else
            {
                com_refx(i) = (COM_float_init).translation()(0);
                com_refy(i) = (COM_float_init).translation()(1);
            }
            com_refdx(i) = 0.0;
            com_refdy(i) = 0.0;
        }
    }
}

void WalkingController::setPelvTrajectory()
{
    double kp = 3.0;

    if (com_control == 0)
    {
        PELV_trajectory_float.translation()(0) = com_refx(walking_tick);
        PELV_trajectory_float.translation()(1) = com_refy(walking_tick);
    }
    else
    {
        //    PELV_trajectory_float.translation()(0) = (PELV_float_current).translation()(0) + kp * (com_refx(walking_tick) - COM_float_current.translation()(0)); //(PELV_float_init.inverse()*COM_float_current).translation()(0));
        //    PELV_trajectory_float.translation()(1) = (PELV_float_current).translation()(1) + kp * (com_refy(walking_tick) - COM_float_current.translation()(1)); //(PELV_float_init.inverse()*COM_float_current).translation()(1));
    }
    PELV_trajectory_float.translation()(0) = com_refx(walking_tick);
    PELV_trajectory_float.translation()(1) = com_refy(walking_tick);

    PELVD_trajectory_float.translation()(0) = com_refdx(walking_tick);
    PELVD_trajectory_float.translation()(1) = com_refdy(walking_tick);
    PELVD_trajectory_float.translation()(2) = 0.0;

    PELV_trajectory_float.translation()(2) = PELV_float_init.translation()(2);
    PELV_trajectory_float.linear() = PELV_float_init.linear();
}

void WalkingController::inverseKinematics(RobotData &Robot, Eigen::Isometry3d PELV_float_transform, Eigen::Isometry3d LF_float_transform, Eigen::Isometry3d RF_float_transform, Eigen::Vector12d &leg_q)
{
    Eigen::Vector3d lp, rp;
    lp = LF_float_transform.linear().transpose() * (PELV_float_transform.translation() - LF_float_transform.translation());
    rp = RF_float_transform.linear().transpose() * (PELV_float_transform.translation() - RF_float_transform.translation());

    Eigen::Matrix3d PELF_rotation, PERF_rotation;
    PELF_rotation = PELV_float_transform.linear().transpose() * LF_float_transform.linear();
    PERF_rotation = PELV_float_transform.linear().transpose() * RF_float_transform.linear();

    Eigen::Vector3d ld, rd;
    ld.setZero();
    rd.setZero();

    ld(0) = HLR_float_init.translation()(0) - PELV_float_init.translation()(0);
    ld(1) = HLR_float_init.translation()(1) - PELV_float_init.translation()(1);
    ld(2) = -(PELV_float_init1.translation()(2) - HLR_float_init.translation()(2)) + (PELV_float_init1.translation()(2) - PELV_float_init.translation()(2));

    rd(0) = HRR_float_init.translation()(0) - PELV_float_init.translation()(0);
    rd(1) = HRR_float_init.translation()(1) - PELV_float_init.translation()(1);
    rd(2) = -(PELV_float_init1.translation()(2) - HRR_float_init.translation()(2)) + (PELV_float_init1.translation()(2) - PELV_float_init.translation()(2));

    ld = LF_float_transform.linear().transpose() * ld;
    rd = RF_float_transform.linear().transpose() * rd;

    Eigen::Vector3d lr, rr;
    lr = lp + ld;
    rr = rp + rd;

    double l_upper = 0.35; //direct length from hip to knee
    double l_lower = 0.35; //direct length from knee to ankle

    double offset_hip_pitch = 0.0 * DEG2RAD;
    double offset_knee_pitch = 0.0 * DEG2RAD;
    double offset_ankle_pitch = 0.0 * DEG2RAD;
    //////////////////////////// LEFT LEG INVERSE KINEMATICS ////////////////////////////

    double lc = sqrt(lr(0) * lr(0) + lr(1) * lr(1) + lr(2) * lr(2));
    leg_q(3) = (-acos((l_upper * l_upper + l_lower * l_lower - lc * lc) / (2 * l_upper * l_lower)) + M_PI); // - offset_knee_pitch //+ alpha_lower

    double l_ankle_pitch = asin((l_upper * sin(M_PI - leg_q(3))) / lc);
    leg_q(4) = -atan2(lr(0), sqrt(lr(1) * lr(1) + lr(2) * lr(2))) - l_ankle_pitch; // - offset_ankle_pitch ;
    leg_q(5) = atan2(lr(1), lr(2));

    Eigen::Matrix3d r_tl2;
    Eigen::Matrix3d r_l2l3;
    Eigen::Matrix3d r_l3l4;
    Eigen::Matrix3d r_l4l5;

    r_tl2.setZero();
    r_l2l3.setZero();
    r_l3l4.setZero();
    r_l4l5.setZero();

    r_l2l3 = DyrosMath::rotateWithY(leg_q(3));
    r_l3l4 = DyrosMath::rotateWithY(leg_q(4));
    r_l4l5 = DyrosMath::rotateWithX(leg_q(5));

    r_tl2 = PELF_rotation * r_l4l5.transpose() * r_l3l4.transpose() * r_l2l3.transpose();

    leg_q(1) = asin(r_tl2(2, 1));

    double c_lq5 = -r_tl2(0, 1) / cos(leg_q(1));
    if (c_lq5 > 1.0)
    {
        c_lq5 = 1.0;
    }
    else if (c_lq5 < -1.0)
    {
        c_lq5 = -1.0;
    }

    leg_q(0) = -asin(c_lq5);
    leg_q(2) = -asin(r_tl2(2, 0) / cos(leg_q(1))) + offset_hip_pitch;
    leg_q(3) = leg_q(3) - offset_knee_pitch;
    leg_q(4) = leg_q(4) - offset_ankle_pitch;

    //////////////////////////// RIGHT LEG INVERSE KINEMATICS ////////////////////////////

    double rc = sqrt(rr(0) * rr(0) + rr(1) * rr(1) + rr(2) * rr(2));
    leg_q(9) = (-acos((l_upper * l_upper + l_lower * l_lower - rc * rc) / (2 * l_upper * l_lower)) + M_PI); // - offset_knee_pitch //+ alpha_lower

    double r_ankle_pitch = asin((l_upper * sin(M_PI - leg_q(9))) / rc);
    leg_q(10) = -atan2(rr(0), sqrt(rr(1) * rr(1) + rr(2) * rr(2))) - r_ankle_pitch;
    leg_q(11) = atan2(rr(1), rr(2));

    Eigen::Matrix3d r_tr2;
    Eigen::Matrix3d r_r2r3;
    Eigen::Matrix3d r_r3r4;
    Eigen::Matrix3d r_r4r5;

    r_tr2.setZero();
    r_r2r3.setZero();
    r_r3r4.setZero();
    r_r4r5.setZero();

    r_r2r3 = DyrosMath::rotateWithY(leg_q(9));
    r_r3r4 = DyrosMath::rotateWithY(leg_q(10));
    r_r4r5 = DyrosMath::rotateWithX(leg_q(11));

    r_tr2 = PERF_rotation * r_r4r5.transpose() * r_r3r4.transpose() * r_r2r3.transpose();
    leg_q(7) = asin(r_tr2(2, 1));
    double c_rq5 = -r_tr2(0, 1) / cos(leg_q(7));

    if (c_rq5 > 1.0)
    {
        c_rq5 = 1.0;
    }
    else if (c_rq5 < -1.0)
    {
        c_rq5 = -1.0;
    }

    leg_q(6) = -asin(c_rq5);
    leg_q(8) = asin(r_tr2(2, 0) / cos(leg_q(7))) - offset_hip_pitch;
    leg_q(9) = -leg_q(9) + offset_knee_pitch;
    leg_q(10) = -leg_q(10) + offset_ankle_pitch;

    leg_q(0) = leg_q(0) * (-1);
    leg_q(6) = leg_q(6) * (-1);
    leg_q(8) = leg_q(8) * (-1);
    leg_q(9) = leg_q(9) * (-1);
    leg_q(10) = leg_q(10) * (-1);
}

void WalkingController::inverseKinematicsdob(RobotData &Robot)
{
    desired_leg_q_temp = desired_leg_q;
    double Kp, Kv;

    for (int i = 0; i < 12; i++)
    {
        dob_hat(i) = desired_leg_q(i) - Robot.q_(i);
    }

    if (walking_tick == 0)
        dob_hat_prev = dob_hat;

    dob_hat = 0.3 * dob_hat + 0.7 * dob_hat_prev;

    double defaultGain = 0.0;
    double compliantGain = 3.0;
    double rejectionGain = -25.0;//-3.5;
    double rejectionGainSim[12] = {-19.0, -19.0, -19.0, -19.0, -19.0, -19.0, -19.0, -19.0, -19.0, -19.0, -19.0, -19.0};
    double rejectionGainReal[12] = {-3.0, -3.0, -3.0, -3.0, -3.0, -3.0, -3.0, -3.0, -3.0, -3.0, -3.0, -3.0};;
    double rejectionGain_[12];
    double compliantTick = 0.05 * wk_Hz;

    memcpy(rejectionGain_, rejectionGainSim, sizeof(rejectionGainSim));
    
    if(current_step_num != 0)
    {
        for (int i = 0; i < 12; i++)
        {
            if (i < 6)
            {
                dobGain = defaultGain;

                if (foot_step(current_step_num, 6) == 0)
                {
                    if (walking_tick < t_start + t_total - t_rest_last - t_double2 - compliantTick)
                    {
                        dobGain = defaultGain;
                    }
                    else if (walking_tick >= t_start + t_total - t_rest_last - t_double2 - compliantTick && walking_tick < t_start + t_total - t_rest_last - t_double2)
                    {
                        dobGain = DyrosMath::QuinticSpline(walking_tick, t_start + t_total - t_rest_last - t_double2 - compliantTick, t_start + t_total - t_rest_last - t_double2, defaultGain, 0.0, 0.0, compliantGain, 0.0, 0.0)(0);
                    }
                    else
                    {
                        dobGain = DyrosMath::QuinticSpline(walking_tick, t_start + t_total - t_rest_last - t_double2, t_start + t_total- t_rest_last, compliantGain, 0.0, 0.0, defaultGain, 0.0, 0.0)(0);
                    }
                }
                else
                {

                    if (walking_tick <t_start_real + t_double1 + t_rest_temp) // the period for lifting the right foot
                    {
                        dobGain = DyrosMath::QuinticSpline(walking_tick, t_start_real + t_rest_temp, t_start_real + t_double1 + t_rest_temp, 0.0, 0.0, 0.0, rejectionGain_[i], 0.0, 0.0)(0);
                    } // the period for lifting the right foot
                    else if(walking_tick <t_start_real + t_double1 + (t_total - t_rest_init - t_rest_last - t_double1 - t_double2 - t_imp))
                    {
                        dobGain = rejectionGain_[i];
                    }
                    else
                    {
                        dobGain = DyrosMath::QuinticSpline(walking_tick, t_start_real + t_double1 + (t_total - t_rest_init - t_rest_last - t_double1 - t_double2 - t_imp) / 2.0, t_start + t_total - t_rest_last - t_double2 - t_imp - t_rest_temp, rejectionGain_[i], 0.0, 0.0, 0.0, 0.0, 0.0)(0);
                    }
                }
                desired_leg_q(i) = desired_leg_q(i) - dobGain * dob_hat(i);
            }
            else
            {
                dobGain = defaultGain;

                if (foot_step(current_step_num, 6) == 1)
                {
                    if (walking_tick < t_start + t_total - t_rest_last - t_double2 - compliantTick)
                    {
                        dobGain = defaultGain;
                    }
                    else if (walking_tick >= t_start + t_total - t_rest_last - t_double2 - compliantTick && walking_tick < t_start + t_total - t_rest_last - t_double2)
                    {
                        dobGain = DyrosMath::QuinticSpline(walking_tick, t_start + t_total - t_rest_last - t_double2 - compliantTick, + t_total - t_rest_last - t_double2, defaultGain, 0.0, 0.0, compliantGain, 0.0, 0.0)(0);
                    }
                    else
                    {
                        dobGain = DyrosMath::QuinticSpline(walking_tick, t_start + t_total - t_rest_last - t_double2, t_start + t_total - t_rest_last, compliantGain, 0.0, 0.0, defaultGain, 0.0, 0.0)(0);
                    }
                }
                else
                {
                    if (walking_tick <t_start_real + t_double1 + t_rest_temp) // the period for lifting the right foot
                    {
                        dobGain = DyrosMath::QuinticSpline(walking_tick, t_start_real + t_rest_temp, t_start_real + t_double1 + t_rest_temp, 0.0, 0.0, 0.0, rejectionGain_[i], 0.0, 0.0)(0);
                    } // the period for lifting the right foot
                    else if(walking_tick <t_start_real + t_double1 + (t_total - t_rest_init - t_rest_last - t_double1 - t_double2 - t_imp))
                    {
                        dobGain = rejectionGain_[i];
                    }
                    else
                    {
                        dobGain = DyrosMath::QuinticSpline(walking_tick, t_start_real + t_double1 + (t_total - t_rest_init - t_rest_last - t_double1 - t_double2 - t_imp) / 2.0, t_start + t_total - t_rest_last - t_double2 - t_imp - t_rest_temp, rejectionGain_[i], 0.0, 0.0, 0.0, 0.0, 0.0)(0);
                    }
                }
                desired_leg_q(i) = desired_leg_q(i) - dobGain * dob_hat(i);
            }
        }
    }
}

void WalkingController::updateNextStepTime(RobotData &rd)
{
    if (walking_tick == t_last)
    {
        if (current_step_num != total_step_num - 1)
        {
            t_start = t_last + 1;
            foot_step_dir = 1.0;
            t_start_real = t_start + t_rest_init;
            t_last = t_start + t_total - 1;

            current_step_num++;
            if (current_step_num == total_step_num)
            {
                current_step_num = total_step_num - 1;
            }
        }
    }

    if (walking_tick == (t_total * (total_step_num + 1) + t_temp - 2))
    {
        rd.tc_.walking_enable = 2.0;
        current_step_num++;
    }

    if (walking_tick >= t_start_real + t_double1 + t_rest_temp - 0.075 * wk_Hz && walking_tick <= t_start_real + t_double1 + t_rest_temp + 0.015 * wk_Hz + 1 && current_step_num != 0)
    {
        phaseChange = true;
        phaseChange1 = false;
        double2Single_pre = t_start_real + t_double1 + t_rest_temp - 0.075 * wk_Hz;
        double2Single = t_start_real + t_double1 + t_rest_temp + 0.015 * wk_Hz + 1;
    }
    else
    {
        phaseChange = false;
    }

    if (walking_tick >= t_start + t_total - t_rest_last - t_double2 - t_imp - t_rest_temp /*+ 0.05 * wk_Hz*/ && walking_tick <= t_start + t_total - t_rest_last - t_double2 - t_imp - t_rest_temp + 0.1 * wk_Hz - 1 && current_step_num != 0 && phaseChange == false)
    {
        phaseChange1 = true;
        phaseChange = false;
        single2Double_pre = t_start + t_total - t_rest_last - t_double2 - t_imp - t_rest_temp + 1 /*+ 0.05 * wk_Hz*/;
        single2Double = t_start + t_total - t_rest_last - t_double2 - t_imp - t_rest_temp + 0.1 * wk_Hz;
    }
    else
    {
        phaseChange1 = false;
        if (walking_tick < t_start_real + t_double1 + t_rest_temp - 0.075 * wk_Hz && walking_tick > t_start_real + t_double1 + t_rest_temp + 0.015 * wk_Hz + 1)
            phaseChange = false;
    }

    walking_tick++;
}

void WalkingController::setWalkingParameter()
{
    desired_foot_step_num = 8;
    walking_tick = 0;
    /* t_rest_init = 0.02 * wk_Hz;
    t_rest_last = 0.02 * wk_Hz;
    t_double1 = 0.020 * wk_Hz;
    t_double2 = 0.020 * wk_Hz;
    t_total = 0.5 * wk_Hz;
    t_temp = 4.0 * wk_Hz;*/
    t_rest_init = 0.1 * wk_Hz;
    t_rest_last = 0.1 * wk_Hz;
    t_double1 = 0.1 * wk_Hz;
    t_double2 = 0.1 * wk_Hz;
    t_total = 1.1 * wk_Hz;
    t_temp = 2.0 * wk_Hz;
    /*t_double1 = 0.35*wk_Hz;
    t_double2 = 0.35*wk_Hz;
    t_rest_init = .15*wk_Hz;
    t_rest_last = .15*wk_Hz;
    t_total= 2.0*wk_Hz;*/
    t_rest_temp = 0.0 * wk_Hz;

    foot_height = 0.03;

    t_imp = 0.0 * wk_Hz;
    t_last = t_total + t_temp;
    t_start = t_temp + 1;

    t_start_real = t_start + t_rest_init;
}

void WalkingController::setInitPose(RobotData &Robot, Eigen::VectorQd &leg_q)
{
    if (walking_init_tick == 0)
    {
        Eigen::VectorQd q_temp;
        q_temp << 0.0, 0.00, -0.595, 1.24, -0.65, 0.00, 0.0, 0.00, -0.595, 1.24, -0.65, 0.00, 0.0, 0.0, 0.0, 0.2, 0.5, 1.5, -1.27, -1, 0, -1, 0, 0, 0, -0.2, -0.5, -1.5, 1.27, 1.0, 0, 1.0, 0;

        //q_temp.setZero();
        //q_target = Robot.q_;
        q_target = q_temp;
        walkingInitialize(Robot);
    }

    for (int i = 0; i < MODEL_DOF; i++)
    {
        leg_q(i) = DyrosMath::QuinticSpline(walking_init_tick, 0.0, 3.0 * wk_Hz, q_init(i), 0.0, 0.0, q_target(i), 0.0, 0.0)(0);
    }
}

void WalkingController::updateInitTime()
{
    walking_init_tick++;
}

void WalkingController::walkingInitialize(RobotData &Robot)
{
    q_init = Robot.q_;
}