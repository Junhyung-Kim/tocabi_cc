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

pinocchio::Model model1;
pinocchio::Data model_data1;

CSharedMemory mpc_start_init, state_init, statemachine, desired_val;

CustomController::CustomController(RobotData &rd) : rd_(rd) //, wbc_(dc.wbc_)
{
    upper_on = true;
    ControlVal_.setZero();
    for (int i = 0; i < 2; i++)
    {
        file[i].open(FILE_NAMES[i].c_str(), std::ios_base::out);
    }
    mpc_cycle = 1;
    mpc_cycle_prev = 1;

    COMX.setZero(3);
    MOMX.setZero(2);
    rd_.mujoco_dist = false;
    nh.setCallbackQueue(&queue_cc_);
    gui_sub_ = nh.subscribe("/chatter", 1, &CustomController::GuiCommandCallback, this);

    
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

    pinocchio::JointModelFreeFlyer root_joint;

    pinocchio::urdf::buildModel("/usr/local/lib/python3.8/dist-packages/robot_properties_tocabi/resources/urdf/tocabi.urdf", root_joint, model);
    ///home/jhk/catkin_ws/src/dyros_tocabi_v2/tocabi_description/robots/dyros_tocabi.urdf
    pinocchio::SE3 M = pinocchio::SE3::Identity();
    M.translation() << 0.03,0.0,-0.1585;
    RFjoint_id = model.getJointId("R_AnkleRoll_Joint");
    LFjoint_id = model.getJointId("L_AnkleRoll_Joint");
    RFframe_id = model.getFrameId("R_Foot_Link");
    LFframe_id = model.getFrameId("L_Foot_Link");
    model.addBodyFrame("LF_contact", LFjoint_id, M, LFframe_id);
    model.addBodyFrame("LF_contact", RFjoint_id, M, LFframe_id);
    LFcframe_id = model.getFrameId("LF_contact");
    RFcframe_id = model.getFrameId("RF_contact");
    pinocchio::Data data(model);
    model_data = data;
    model_data1 = data;
    q_pinocchio.setZero();
    mpc_on = false;
    wlk_on = false;
    walking_tick = 0;
    q_dm.resize(18);
    
    int shm_id_;
    if ((shm_id_ = shmget(key_t(1), sizeof(sizeof(int) * 3), IPC_CREAT | 0666)) == -1)
    {
        std::cout << "shm1 mtx failed " << std::endl;
    }

    if(shmctl(shm_id_, IPC_RMID, NULL) == -1)
    {
        std::cout << "Error Deleting SHAREDMEMORY1"<<std::endl;
    }

    if ((shm_id_ = shmget(key_t(2), sizeof(sizeof(double) * 49), IPC_CREAT | 0666)) == -1)
    {
        std::cout << "shm2 mtx failed " << std::endl;
    }

    if(shmctl(shm_id_, IPC_RMID, NULL) == -1)
    {
        std::cout << "Error Deleting SHAREDMEMORY2"<<std::endl;
    }

    if ((shm_id_ = shmget(key_t(3), sizeof(sizeof(int) * 3), IPC_CREAT | 0666)) == -1)
    {
        std::cout << "shm3 mtx failed " << std::endl;
    }

    if(shmctl(shm_id_, IPC_RMID, NULL) == -1)
    {
        std::cout << "Error Deleting SHAREDMEMORY3"<<std::endl;
    }

    if ((shm_id_ = shmget(key_t(4), sizeof(sizeof(double) * 49), IPC_CREAT | 0666)) == -1)
    {
        std::cout << "shm3 mtx failed " << std::endl;
    }
    if(shmctl(shm_id_, IPC_RMID, NULL) == -1)
    {
        std::cout << "Error Deleting SHAREDMEMORY4"<<std::endl;
    }

    mpc_start_init.setKey(1);
    mpc_start_init.setupSharedMemory(sizeof(int) * 3);
    mpc_start_init.attachSharedMemoryint();
    std::cout << "SHAREDMEMORY FIRST OK"<< std::endl;
    state_init.setKey(2);
    state_init.setupSharedMemory(sizeof(double) * 49);
    state_init.attachSharedMemory();
    std::cout << "SHAREDMEMORY Second OK"<< std::endl;
    statemachine.setKey(3);
    statemachine.setupSharedMemoryRead(sizeof(int) * 3);
    statemachine.attachSharedMemoryint();
    std::cout << "SHAREDMEMORY Third  OK"<< std::endl;
    desired_val.setKey(4);
    desired_val.setupSharedMemoryRead(sizeof(double) * 49);
    desired_val.attachSharedMemory();

    std::cout << "SHAREDMEMORY Fourth OK"<< std::endl;
    mpc_start_init.m_shared_memory_int[0] = 5;
    mpc_start_init.m_shared_memory_int[1] = 3;
    mpc_start_init.m_shared_memory_int[2] = 3;
    std::cout << "Custom Controller Init" << std::endl;
    std::fstream read_file("/home/jhk/walkingdata/beforedata/fdyn/lfoot2_final.txt");
    std::fstream read_file1("/home/jhk/walkingdata/beforedata/fdyn/rfoot2_final.txt");
    std::fstream read_file2("/home/jhk/walkingdata/beforedata/fdyn/zmp2_ssp1_1.txt");
    std::fstream read_file3("/home/jhk/walkingdata/beforedata/ssp2/lfoot1.txt");
    std::fstream read_file4("/home/jhk/walkingdata/beforedata/ssp2/rfoot2.txt");
    std::fstream read_file5("/home/jhk/walkingdata/beforedata/ssp2/zmp3.txt");
    
    std::vector<double> RF_tran, LF_tran, ZMP_bound, RF_tran_ssp2, LF_tran_ssp2, ZMP_bound_ssp2;
    std::string string_test;
    double jointvalue;
    control_input.setZero();

    if (read_file.is_open() && read_file3.is_open())
    {
        while (!read_file.eof())
        {  
            for (int i = 0; i < 3; i++)
            {
                read_file >> string_test;
                //string_test.erase(find(string_test.begin(), string_test.end(), ','));
                jointvalue = atof(string_test.c_str());
                if (abs(jointvalue) >= 0.0)
                {  
                    LF_tran.push_back(jointvalue);
                }
            }
        }

        while (!read_file3.eof())
        {  
            for (int i = 0; i < 3; i++)
            {
                read_file3 >> string_test;
                //string_test.erase(find(string_test.begin(), string_test.end(), ','));
                jointvalue = atof(string_test.c_str());
                if (abs(jointvalue) >= 0.0)
                {  
                    LF_tran_ssp2.push_back(jointvalue);
                }
            }
        }
    }
    else
    {
        std::cout << "LF OPEN FAIL" << std::endl;
    }

    if (read_file1.is_open() && read_file4.is_open())
    {
        while (!read_file1.eof())
        {   
            for (int i = 0; i < 3; i++)
            {
                read_file1 >> string_test;
                //string_test.erase(find(string_test.begin(), string_test.end(), ','));
                
                jointvalue = atof(string_test.c_str());
                if (abs(jointvalue) >= 0.0)
                {  
                    RF_tran.push_back(jointvalue);
                }
            }
        }
        while (!read_file4.eof())
        {  
            for (int i = 0; i < 3; i++)
            {
                read_file4 >> string_test;
                //string_test.erase(find(string_test.begin(), string_test.end(), ','));
                jointvalue = atof(string_test.c_str());
                if (abs(jointvalue) >= 0.0)
                {  
                    RF_tran_ssp2.push_back(jointvalue);
                }
            }
        }
    }
    else
    {
        std::cout << "RF OPEN FAIL" << std::endl;
    }

    if (read_file2.is_open() && read_file5.is_open())
    {
        while (!read_file2.eof())
        {  
            for (int i = 0; i < 4; i++)
            {
                read_file2 >> string_test;
                //string_test.erase(find(string_test.begin(), string_test.end(), ','));
                jointvalue = atof(string_test.c_str());
                if (abs(jointvalue) >= 0.0)
                {  
                    ZMP_bound.push_back(jointvalue);
                }
            }
        }

        while (!read_file5.eof())
        {  
            for (int i = 0; i < 4; i++)
            {
                read_file5 >> string_test;
                //string_test.erase(find(string_test.begin(), string_test.end(), ','));
                jointvalue = atof(string_test.c_str());
                if (abs(jointvalue) >= 0.0)
                {  
                    ZMP_bound_ssp2.push_back(jointvalue);
                }
            }
        }
    }
    else
    {
        std::cout << "ZMP OPEN FAIL" << std::endl;
    }
    
    std::cout << "ZMP" << std::endl;
    std::cout << ZMP_bound_ssp2.size() << std::endl;
    std::cout << "RF_tran_ssp2" << std::endl;
    std::cout << RF_tran_ssp2.size() << std::endl;
    std::cout << "LF_tran_ssp2" << std::endl;
    std::cout << LF_tran_ssp2.size() << std::endl;

    std::cout << "ZMP" << std::endl;
    std::cout << ZMP_bound.size() << std::endl;
    std::cout << "RF_tran" << std::endl;
    std::cout << RF_tran.size() << std::endl;
    std::cout << "LF_tran" << std::endl;
    std::cout << LF_tran.size() << std::endl;
    
    double *RF_tran1 = &RF_tran[0];
    double *LF_tran1 = &LF_tran[0];
    double *ZMP_bound1 = &ZMP_bound[0];
    Eigen::MatrixXd LF_matrix_temp = Eigen::Map<Eigen::MatrixXd>(RF_tran1, 3, 110);
    Eigen::MatrixXd RF_matrix_temp = Eigen::Map<Eigen::MatrixXd>(LF_tran1, 3, 110);
    Eigen::MatrixXd ZMP_bound_temp = Eigen::Map<Eigen::MatrixXd>(ZMP_bound1, 4, 110);
    double *RF_tran1_ssp2 = &RF_tran_ssp2[0];
    double *LF_tran1_ssp2 = &LF_tran_ssp2[0];
    double *ZMP_bound1_ssp2 = &ZMP_bound_ssp2[0]; 
    Eigen::MatrixXd LF_matrix_temp_ssp2 = Eigen::Map<Eigen::MatrixXd>(RF_tran1_ssp2, 3, 133);
    Eigen::MatrixXd RF_matrix_temp_ssp2 = Eigen::Map<Eigen::MatrixXd>(LF_tran1_ssp2, 3, 133);
    Eigen::MatrixXd ZMP_bound_temp_ssp2 = Eigen::Map<Eigen::MatrixXd>(ZMP_bound1_ssp2, 4, 168);
    
    Eigen::MatrixXd RF_matrix1, LF_matrix1;

    RF_matrix1 = RF_matrix_temp.transpose();
    LF_matrix1 = LF_matrix_temp.transpose();

    RF_matrix.resize(109,3);
    LF_matrix.resize(109,3);

    RF_matrix_ssp2 = RF_matrix_temp_ssp2.transpose();
    LF_matrix_ssp2 = LF_matrix_temp_ssp2.transpose();

    for(int i = 0; i < RF_matrix.rows(); i++)
    {
        RF_matrix.row(i) = RF_matrix1.row(i);
        LF_matrix.row(i) = LF_matrix1.row(i);
    }
    ZMP_bound_matrix = ZMP_bound_temp.transpose();
    ZMP_bound_matrix_ssp2 = ZMP_bound_temp_ssp2.transpose();
    z_ctrl_prev.setZero();
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
    auto startTime1 = std::chrono::system_clock::now();
                        
    queue_cc_.callAvailable(ros::WallDuration());
    //struct timeval start_time, final_time;
    //gettimeofday(&start_time, NULL);

    Eigen::Vector2d ZMP_l, ZMP_r, ZMP;
    ZMP_l(0) = -rd_.LF_CF_FT(4)/rd_.LF_CF_FT(2);
    ZMP_l(1) = rd_.LF_CF_FT(3)/rd_.LF_CF_FT(2);
    ZMP_r(0) = -rd_.RF_CF_FT(4)/rd_.RF_CF_FT(2);
    ZMP_r(1) = rd_.RF_CF_FT(3)/rd_.RF_CF_FT(2);

    if(LF_matrix(mpc_cycle+1,2) == 0.0 && RF_matrix(mpc_cycle+1,2) == 0.0)
    { 
        contactMode = 1;
        ZMP(0) = ((ZMP_l(0) + LF_matrix(mpc_cycle+1,0))*rd_.LF_CF_FT(2) + (ZMP_r(0) + RF_matrix(mpc_cycle+1,0))*rd_.RF_CF_FT(2))/(rd_.RF_CF_FT(2) + rd_.LF_CF_FT(2));
        ZMP(1) = ((ZMP_l(1) + LF_matrix(mpc_cycle+1,1))*rd_.LF_CF_FT(2) + (ZMP_r(1) + RF_matrix(mpc_cycle+1,1))*rd_.RF_CF_FT(2))/(rd_.RF_CF_FT(2) + rd_.LF_CF_FT(2));
    }
    else if(LF_matrix(mpc_cycle+1,2) == 0.0)
    {
        contactMode = 2;
        ZMP(0) = ZMP_l(0)+ LF_matrix(mpc_cycle+1,0);
        ZMP(1) = ZMP_l(1)+ LF_matrix(mpc_cycle+1,1);
        ZMP_r(0) = 0.0;
        ZMP_r(1) = 0.0;
    }
    else if(RF_matrix(mpc_cycle+1,2) == 0.0)
    {
        contactMode = 3;
        ZMP(0) = ZMP_r(0)+ RF_matrix(mpc_cycle+1,0);
        ZMP(1) = ZMP_r(1)+ RF_matrix(mpc_cycle+1,1);
        ZMP_l(0) = 0.0;
        ZMP_l(1) = 0.0;
    }
    
    if(walking_tick == 0 && mpc_cycle == 1)
    {
        ZMP_FT_law(0) = ZMP(0);
        ZMP_FT_law(1) = ZMP(1);
    }
    
    cc_mutex.lock();
    for (int i = 0; i < 6; i++)
        q_pinocchio[i] = rd_.q_virtual_[i];

    q_pinocchio[6] = rd_.q_virtual_[39];

    for (int i = 6; i < MODEL_DOF_VIRTUAL; i ++)
        q_pinocchio[i+1] = rd_.q_virtual_[i];
    
    ZMP_FT_law(0) = DyrosMath::lpf(ZMP(0), ZMP_FT_law(0), 2000, 15);
    ZMP_FT_law(1) = DyrosMath::lpf(ZMP(1), ZMP_FT_law(1), 2000, 15);
    ZMP_FT(0) = ZMP(0);
    ZMP_FT(1) = ZMP(1);

    //for(int i = 0; i < 3; i++)
        //q_dot_virtual_11(i) = qd_pinocchio(i);

    /*Eigen::VectorQVQd q_virtual_111;
    q_virtual_111 = q_pinocchio;
    for (int i = 0; i < 19; i ++)
    {
        q_virtual_111(i) = desired_val.m_shared_memory[i];
    }
    for (int i = 0; i < 2; i ++)
    {
        q_virtual_111(i+20) = desired_val.m_shared_memory[i+18];
        //std::cout << "i " << i << std::endl;
    }
  

    for (int i = 0; i < 18; i ++)
    {
        q_dot_virtual_11(i) = desired_val.m_shared_memory[21+i];
        //std::cout << "i " << i << std::endl;
    }
    for (int i = 0; i < 2; i ++)
    {
        q_dot_virtual_11(i+19) = desired_val.m_shared_memory[39+i];
        //std::cout << "i " << i << std::endl;
    }*/

    
    //for (int i = 0; i < MODEL_DOF_VIRTUAL + 1; i ++)
    //    q_dot_virtual_11(i) = desired_val.m_shared_memory[i];
    
    pinocchio::computeCentroidalMomentum(model, model_data1, q_pinocchio, rd_.q_dot_virtual_);
    CMM = pinocchio::computeCentroidalMap(model, model_data1, q_pinocchio);  
    pinocchio::forwardKinematics(model, model_data1, q_pinocchio);
    pinocchio::updateFramePlacements(model,model_data1);
    pinocchio::centerOfMass(model, model_data1, q_pinocchio, rd_.q_dot_virtual_);
    
    //pinocchio::centerOfMass(model, model_data1, q_virtual_111, q_dot_virtual_11);
    cc_mutex.unlock();
    /*
    if(state_init_ == true)
    {
        joint_prev = rd_.q_;
        vjoint_prev = rd_.q_virtual_.segment<3>(0);
        state_init_ = false;
    }

    jointdot = (rd_.q_ - joint_prev)/0.0005;
    vjoint_dot = (rd_.q_virtual_.segment<3>(0) - vjoint_prev)/0.0005;
    joint_prev = rd_.q_;
    vjoint_prev = rd_.q_virtual_.segment<3>(0);
    file[0]<< rd_.q_virtual_(0) << " " << vjoint_dot(0) <<  " " << rd_.q_dot_virtual_(0)<< " " << rd_.q_virtual_(1) << " "<< vjoint_dot(1) <<  " " << rd_.q_dot_virtual_(1) << " " << vjoint_dot(2) <<  " " << rd_.q_dot_virtual_(2)  << " " << jointdot(0) << " " << rd_.q_dot_virtual_(6) << " "<< jointdot(1) << " " << rd_.q_dot_virtual_(7) << " "<< jointdot(2) << " " << rd_.q_dot_virtual_(8) << " "<< jointdot(3) << " " << rd_.q_dot_virtual_(9) << std::endl;
    */
    //gettimeofday(&final_time, NULL);
    //double x_us, y_us, diff;

    //x_us = (double)start_time.tv_sec * 1000000 + (double)start_time.tv_usec;
    //y_us = (double)final_time.tv_sec * 1000000 + (double)final_time.tv_usec;

    //diff = (double)y_us - (double)x_us;
    //std::cout << "latency" << diff<< std::endl;

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

            q_dot_virtual_lpf_ = DyrosMath::lpf(rd_.q_dot_virtual_, q_dot_virtual_lpf_, 2000, 3);
   
            Vector12d fc_redis;
            double fc_ratio = 0.000;
            fc_redis.setZero();

            if(mpc_cycle >= 20 && mpc_cycle <= 48)
            {
                rd_.ee_[0].contact = 0.0;
                rd_.ee_[1].contact = 1.0;
                WBC::SetContact(rd_, rd_.ee_[0].contact, rd_.ee_[1].contact);
                TorqueGrav = WBC::GravityCompensationTorque(rd_);
                //TorqueContact = WBC::ContactForceRedistributionTorqueWalking(rd_, TorqueGrav, fc_ratio, rate, 1);
                rd_.torque_desired_walk = TorqueGrav;// + TorqueContact;
                //rd_.torque_desired_walk = WBC::ContactForceRedistributionTorque(rd_, WBC::GravityCompensationTorque(rd_));
            }
            else if(mpc_cycle == 19 || mpc_cycle == 18)
            {
                rd_.ee_[0].contact = 1.0;
                rd_.ee_[1].contact = 1.0;
                WBC::SetContact(rd_, rd_.ee_[0].contact, rd_.ee_[1].contact);
                rate = DyrosMath::cubic(walking_tick + 40*(mpc_cycle - 18), 0 , 80, 1, 0, 0, 0);
                TorqueGrav = WBC::GravityCompensationTorque(rd_);
                //std::cout << rate << std::endl;
                TorqueContact = WBC::ContactForceRedistributionTorqueWalking(rd_, TorqueGrav, fc_ratio, rate, 0);
                rd_.torque_desired_walk = TorqueContact;
            }
            else if(mpc_cycle == 49 || mpc_cycle == 50)
            {
                rd_.ee_[0].contact = 1.0;
                rd_.ee_[1].contact = 1.0;
                WBC::SetContact(rd_, rd_.ee_[0].contact, rd_.ee_[1].contact);
                rate = DyrosMath::cubic(walking_tick + 40*(mpc_cycle - 49), 0 , 80, 0, 1, 0, 0);
                TorqueGrav = WBC::GravityCompensationTorque(rd_);
                TorqueContact = WBC::ContactForceRedistributionTorqueWalking(rd_, TorqueGrav, fc_ratio, rate, 0);
                rd_.torque_desired_walk = TorqueContact;
            }
            else if(mpc_cycle >= 51 && mpc_cycle <66)
            {
                rd_.ee_[0].contact = 1.0;
                rd_.ee_[1].contact = 1.0;
                WBC::SetContact(rd_, rd_.ee_[0].contact, rd_.ee_[1].contact);
                rate = 1;//DyrosMath::cubic(walking_tick + 40*(mpc_cycle - 49), 0 , 80, 0, 1, 0, 0);
                TorqueGrav = WBC::GravityCompensationTorque(rd_);
                TorqueContact = WBC::ContactForceRedistributionTorqueWalking(rd_, TorqueGrav, fc_ratio, rate, 0);
                rd_.torque_desired_walk =  TorqueContact;
            }
            else if(mpc_cycle == 66 || mpc_cycle == 67)
            {
                rd_.ee_[0].contact = 1.0;
                rd_.ee_[1].contact = 1.0;
                WBC::SetContact(rd_, rd_.ee_[0].contact, rd_.ee_[1].contact);
                rate = DyrosMath::cubic(walking_tick + 40*(mpc_cycle - 66), 0 , 80, 1, 0, 0, 0);
                TorqueGrav = WBC::GravityCompensationTorque(rd_);
                TorqueContact = WBC::ContactForceRedistributionTorqueWalking(rd_, TorqueGrav, fc_ratio, rate, 1);
                rd_.torque_desired_walk = TorqueContact;
            }
            else if(mpc_cycle > 67 && mpc_cycle < 94)
            {
                rd_.ee_[0].contact = 1.0;
                rd_.ee_[1].contact = 0.0;
                WBC::SetContact(rd_, rd_.ee_[0].contact, rd_.ee_[1].contact);
                TorqueGrav = WBC::GravityCompensationTorque(rd_);
                //TorqueContact = WBC::ContactForceRedistributionTorqueWalking(rd_, TorqueGrav, fc_ratio, rate, 1);
                rd_.torque_desired_walk = TorqueGrav;// + TorqueContact;
                //rd_.torque_desired_walk = WBC::ContactForceRedistributionTorque(rd_, WBC::GravityCompensationTorque(rd_));
            }
            else if(mpc_cycle == 94 || mpc_cycle == 95)
            {
                rd_.ee_[0].contact = 1.0;
                rd_.ee_[1].contact = 1.0;
                WBC::SetContact(rd_, rd_.ee_[0].contact, rd_.ee_[1].contact);
                rate = DyrosMath::cubic(walking_tick + 40*(mpc_cycle - 94), 0 , 80, 0, 1, 0, 0);
                TorqueGrav = WBC::GravityCompensationTorque(rd_);
                TorqueContact = WBC::ContactForceRedistributionTorqueWalking(rd_, TorqueGrav, fc_ratio, rate, 1);
                rd_.torque_desired_walk = TorqueContact;
            }
            else
            {
                WBC::SetContact(rd_, 1, 1);
                //rate = 0.0;
                //TorqueGrav = WBC::GravityCompensationTorque(rd_);
                //std::cout << rate << std::endl;
                //TorqueContact = WBC::ContactForceRedistributionTorqueWalking(rd_, TorqueGrav, fc_ratio, rate, 1);
                //rd_.torque_desired_walk = TorqueGrav + TorqueContact;
                //
                rd_.torque_desired_walk = WBC::ContactForceRedistributionTorque(rd_, WBC::GravityCompensationTorque(rd_));
            }

            /*if(!(walking_tick == 40 and mpc_cycle == 50) && mpc_cycle >= 45 && mpc_cycle < 51)
            {
                std::cout << "mpcYc" << mpc_cycle << " " << walking_tick << std::endl;
                std::cout << rd_.torque_desired_walk(10) << " " <<  rd_.torque_desired_walk(9) << std::endl;
            }*/
            //std::cout << "mpc_cycle " << " " << walking_tick << std::endl;
            //std::cout <<  << std::endl;
            /*
            if (walking_tick >= 0 && wlk_on == true && current_step_num != total_step_num)
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
                        TorqueGrav(1) = 1.0 * TorqueGrav(1);
                        TorqueGrav(7) = 1.0 * TorqueGrav(7);
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
                        TorqueGrav(1) = 1.0 * TorqueGrav(1);
                        TorqueGrav(7) = 1.0 * TorqueGrav(7);
                    }

                    rate = DyrosMath::cubic(walking_tick, single2Double_pre, single2Double, 0, 1, 0, 0);
                    if (current_step_num < total_step_num - 1)
                        TorqueContact = WBC::ContactForceRedistributionTorqueWalking(rd_, TorqueGrav, fc_ratio, rate, foot_step_mu(current_step_num, 6));
                    else
                        TorqueContact = WBC::ContactForceRedistributionTorqueWalking(rd_, TorqueGrav, fc_ratio, rate, foot_step_mu(total_step_num - 1, 6));

                    rd_.torque_desired_walk = TorqueGrav;//+ TorqueContact;
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
                
            }*/

            for (int i = 0; i < MODEL_DOF; i++)
            { // 0, 3
                //rd_.torque_desired[i] = rd_.pos_kp_v[i] * 0.8* (rd_.q_desired[i] - rd_.q_[i]) + rd_.pos_kv_v[i]*5 * (qd_pinocchio_[i+6] - rd_.q_dot_[i]) + rd_.torque_desired_walk[i];
                
                if(i == 0 && i == 6)
                {
                    rd_.torque_desired[i] = rd_.pos_kp_v[i] * 30 * (rd_.q_desired[i] - rd_.q_[i]) + rd_.pos_kv_v[i] * 4 * (qd_pinocchio_[i+6] - rd_.q_dot_[i]) + rd_.torque_desired_walk[i];

                }
                else
                {
                    rd_.torque_desired[i] = rd_.pos_kp_v[i] * 0.8 * (rd_.q_desired[i] - rd_.q_[i]) + rd_.pos_kv_v[i] * 2 * (qd_pinocchio_[i+6] - rd_.q_dot_[i]) + rd_.torque_desired_walk[i];

                }
                
            }
            if (mpc_cycle <= controlwalk_time-1) 
                    file[0] <<mpc_cycle << " " <<rd_.torque_desired_walk[2]<< " " << rd_.torque_desired_walk[3]<< " " << rd_.torque_desired_walk[4]<< " " << rd_.torque_desired_walk[8]<< " " << rd_.torque_desired_walk[9]<< " " << rd_.torque_desired_walk[10]<< std::endl;
                
            
            auto endTime1 = std::chrono::system_clock::now();
            
            if(walking_tick == 1)
            {
                auto elapsed2 = std::chrono::duration_cast<std::chrono::microseconds>(endTime1 - startTime1);
                mpc_latency += elapsed2.count();
            }
            else
            {
                mpc_latency = 0.0;
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
        auto startTime = std::chrono::system_clock::now();
        auto startTime1 = std::chrono::system_clock::now();
        auto endTime1 = std::chrono::system_clock::now();
        if (rd_.tc_.walking_enable == 1.0)
        {
            if (wlk_on == false)
            {
                wk_Hz = 2000;
                wk_dt = 1 / wk_Hz;
                controlwalk_time = 98;
               
                //std::cout << " aaa" << std::endl;
                if (walking_tick == 0)
                {
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
                    //std::cout << com_refx(0) << " " << com_refy(0) <<std::endl;
                    cc_mutex.unlock();
                    
                    walking_tick = 0;
                    q_init = rd_.q_;
                    q_desired.resize(15);
                    q_desired.setZero();
                    q_desired.head(15) << 0, 0, -0.55, 1.26, -0.71, 0, 0, 0, -0.55, 1.26, -0.71, 0, 0, 0, 0;//= rd_.q_;
                    //walking_tick = walking_tick + 1;
                }

                if(walking_tick <= 2000)
                {
                    if(walking_tick <= 2000)
                    {
                        for (int i = 0; i < 15; i++)
                        {
                            rd_.q_desired(i) = DyrosMath::cubic(walking_tick, 0, 1000, q_init(i), q_desired(i), 0, 0);
                        }
                    }
                    else
                    {
                        for (int i = 0; i < 15; i++)
                        {
                            rd_.q_desired(i) =  q_desired(i);
                        }
                    }
                    for (int i = 15; i < MODEL_DOF; i++)
                    {
                        rd_.q_desired(i) = q_init(i);
                    }
                    walking_tick = walking_tick + 1;
                }
                else
                {   
                    wlk_on = true;
                    walking_tick = 0;
                    mpc_start_init.m_shared_memory_int[0] = 4;
                    std::cout << "Start" << std::endl;
                }
            }
            
            if(stateestimation == true)
            {   
                for (int i = 0; i < 6; i ++)
                    state_init.m_shared_memory[i] = rd_.q_virtual_[i];

                state_init.m_shared_memory[6] = rd_.q_virtual_[39];
                
                for (int i = 6; i < 18; i ++)
                    state_init.m_shared_memory[i+1] = rd_.q_virtual_[i];

                for (int i = 0; i < 18; i ++)
                    state_init.m_shared_memory[i+21] = rd_.q_dot_virtual_[i];
                
                state_init.m_shared_memory[20] = rd_.q_virtual_[19];
                state_init.m_shared_memory[21] = rd_.q_virtual_[20];

                state_init.m_shared_memory[41] = rd_.link_[COM_id].xpos(0);
                state_init.m_shared_memory[42] = rd_.link_[COM_id].v(0);//rd_.link_[COM_id].xpos(0);
                
                state_init.m_shared_memory[45] = rd_.link_[COM_id].xpos(1);
                state_init.m_shared_memory[46] = rd_.link_[COM_id].v(1);//rd_.link_[COM_id].xpos(0);
                
                
                if(mpc_cycle == 1)
                {
                    state_init.m_shared_memory[43] = rd_.link_[COM_id].xpos(0);
                    state_init.m_shared_memory[47] = rd_.link_[COM_id].xpos(1);
                    state_init.m_shared_memory[44] = 0.0;//model_data.hg.angular[1];//rd_.link_[COM_id].xpos(0);
                    state_init.m_shared_memory[48] = 0.0;//model_data.hg.angular[0];///rd_.link_[COM_id].xpos(0);

                }
                else
                {
                    /*state_init.m_shared_memory[43] = desired_val.m_shared_memory[43];
                    state_init.m_shared_memory[47] = desired_val.m_shared_memory[47];
                    state_init.m_shared_memory[44] = desired_val.m_shared_memory[44];
                    state_init.m_shared_memory[48] = desired_val.m_shared_memory[48];*/
                }
                
                for (int i = 0; i < 49; i++)
                    std::cout << state_init.m_shared_memory[i] << " " << std::endl;

                mpc_start_init.m_shared_memory_int[0] = 1;
                stateestimation = false;
                std::cout << " state esimation" << std::endl;
            }
            
            if(walking_tick != 0)
            {
                for (int i = 0; i < 6; i ++)
                    state_init.m_shared_memory[i] = rd_.q_virtual_[i];

                state_init.m_shared_memory[6] = rd_.q_virtual_[39];
                
                for (int i = 6; i < 18; i ++)
                    state_init.m_shared_memory[i+1] = rd_.q_virtual_[i];
                state_init.m_shared_memory[19] = rd_.q_virtual_[19];
                state_init.m_shared_memory[20] = rd_.q_virtual_[20];

                for (int i = 0; i < 18; i ++)
                    state_init.m_shared_memory[i+21] = rd_.q_dot_virtual_[i];//rd_.q_dot_virtual_[i];
                
                state_init.m_shared_memory[39] =  rd_.q_dot_virtual_[19];//rd_.q_dot_virtual_[19];
                state_init.m_shared_memory[40] =  rd_.q_dot_virtual_[20];//rd_.q_dot_virtual_[20];
                
                state_init.m_shared_memory[41] = rd_.link_[COM_id].xpos(0);
                state_init.m_shared_memory[45] = rd_.link_[COM_id].xpos(1);
                state_init.m_shared_memory[42] = rd_.link_[COM_id].v(0);
                state_init.m_shared_memory[46] = rd_.link_[COM_id].v(1);

                state_init.m_shared_memory[43] = ZMP_FT(0);//desired_val.m_shared_memory[43];
                state_init.m_shared_memory[47] = ZMP_FT(1);//desired_val.m_shared_memory[47];

                state_init.m_shared_memory[44] = model_data1.hg.angular()(1);
                state_init.m_shared_memory[48] = model_data1.hg.angular()(0);
            }
            
            /*if(!(mpc_cycle == ㄹ1 and walking_tick == 0))
            {
                for (int i = 0; i < 6; i ++)
                    state_init.m_shared_memory[i] = desired_val.m_shared_memory[i];//rd_.q_virtual_[i];
                for(int i = 6; i < 49; i ++)
                    state_init.m_shared_memory[i] = desired_val.m_shared_memory[i];
                //std::cout <<" aaaaaa" << std::endl;
            }*/
            
            if(statemachine.m_shared_memory_int[0] == 1 || statemachine.m_shared_memory_int[0] == 2)
            {
                if (walking_tick == 0)
                {
                    qd_pinocchio_.setZero();
                }  
                if(statemachine.m_shared_memory_int[0] == 1 || statemachine.m_shared_memory_int[0] == 3)// || mpc_cycle == 49 || mpc_cycle == 50)
                {
                    mpc_start_init.m_shared_memory_int[0] = 3;
                    if (walking_tick == 0)
                    {  
                        mpc_start_init.m_shared_memory_int[0] = 2;
                        std::cout << "mpc : " << mpc_cycle << " " << walking_tick << " " << RF_matrix(mpc_cycle+1,2)+0.15842<< " " << LF_matrix(mpc_cycle+1,2) + 0.15842<< std::endl;
                        std::cout << rd_.roll <<  " " <<  rd_.pitch <<  std::endl;
                        walking_tick_stop = false;

                        /*std::cout << "desired x" << std::endl;
                        for(int i = 0; i < 49; i++)
                        {
                            std::cout << i << " : " << std::endl;
                            std::cout << desired_val.m_shared_memory[i] << " ";
                        }*/
                        if (mpc_cycle == 1)
                            q_init = rd_.q_;
                        else
                            q_init = rd_.q_;//q_pinocchio_desired.segment<MODEL_DOF>(7);
                        
                        //qd_pinocchio.setZero();
                        

                        if(mpc_cycle <= controlwalk_time)
                        {
                            if(upper_on == true && mpc_cycle <= controlwalk_time)
                            {
                                std::cout << "JOint " << desired_val.m_shared_memory[20] << " " << desired_val.m_shared_memory[21] << std::endl;
                                if(desired_val.m_shared_memory[19] < -0.1300)
                                {  
                                    std::cout << "Roll over" << std::endl;
                                    if(desired_val.m_shared_memory[39] < 0.0)
                                    {
                                        qd_pinocchio(19) = 0.0;
                                        upperd[0] = 0.0;
                                    }
                                    else
                                    {
                                        qd_pinocchio(19) = desired_val.m_shared_memory[39];
                                        upperd[0] = desired_val.m_shared_memory[39];
                                    }
                                }
                                else if(desired_val.m_shared_memory[19] > 0.1300)
                                {   
                                    std::cout << "Roll over" << std::endl;
                                    if(desired_val.m_shared_memory[39] > 0.0)
                                    {
                                        qd_pinocchio(19) = 0.0;
                                        upperd[0] = 0.0;
                                    }
                                    else
                                    {
                                        qd_pinocchio(19) = desired_val.m_shared_memory[39];
                                        upperd[0] = desired_val.m_shared_memory[39];
                                    }
                                }
                                else
                                {   
                                    qd_pinocchio(19) = desired_val.m_shared_memory[39];
                                    upperd[0] = desired_val.m_shared_memory[39];
                                }

                                if(desired_val.m_shared_memory[20] < -0.1300)
                                {
                                    std::cout << "Pitch over" << std::endl;
                                    if(desired_val.m_shared_memory[40] < 0.0)
                                    {
                                        qd_pinocchio(20) = 0.0;
                                        upperd[1] = 0.0;
                                    }
                                    else
                                    {
                                        qd_pinocchio(20) = desired_val.m_shared_memory[40];
                                        upperd[1] = desired_val.m_shared_memory[40];
                                    }
                                }
                                else if(desired_val.m_shared_memory[20] > 0.1300) 
                                {
                                    if(desired_val.m_shared_memory[40] > 0.0)
                                    {
                                        qd_pinocchio(20) = 0.0;
                                        upperd[1] = 0.0;
                                    }
                                    else
                                    {
                                        qd_pinocchio(20) = desired_val.m_shared_memory[40];
                                        upperd[1] = desired_val.m_shared_memory[40];
                                    }  
                                }
                                else
                                {
                                    qd_pinocchio(20) = desired_val.m_shared_memory[40];
                                    upperd[1] = desired_val.m_shared_memory[40];
                                }
                            }
                            

                            comd[0] = desired_val.m_shared_memory[42];
                            comd[1] = desired_val.m_shared_memory[46];
                            comd[2] = 0.0;

                            angm[0] = desired_val.m_shared_memory[48];
                            angm[1] = desired_val.m_shared_memory[44];
                            com_mpc[0] = desired_val.m_shared_memory[41];
                            com_mpc[1] = desired_val.m_shared_memory[45];
                        }
                        else
                        {
                            upperd.setZero();
                            comd.setZero();
                            angm.setZero();
                        }

                        if(mpc_cycle == 0)
                        {
                            rfootd.setZero();
                            lfootd.setZero();
                        }
                        else
                        {   
                            if(mpc_cycle <= 50)
                            {
                                    rfootd[0] = (RF_matrix(mpc_cycle+1,0)-RF_matrix(mpc_cycle,0))/0.02;
                                    rfootd[1] = (RF_matrix(mpc_cycle+1,1)-RF_matrix(mpc_cycle,1))/0.02;
                                    rfootd[2] = (RF_matrix(mpc_cycle+1,2)-RF_matrix(mpc_cycle,2))/0.02;

                                    lfootd[0] = (LF_matrix(mpc_cycle+1,0)-LF_matrix(mpc_cycle,0))/0.02;
                                    lfootd[1] = (LF_matrix(mpc_cycle+1,1)-LF_matrix(mpc_cycle,1))/0.02;
                                    lfootd[2] = (LF_matrix(mpc_cycle+1,2)-LF_matrix(mpc_cycle,2))/0.02;
                            }
                            else
                            {
                                rfootd[0] = (RF_matrix_ssp2(mpc_cycle-48,0)-RF_matrix_ssp2(mpc_cycle-49,0))/0.02;
                                rfootd[1] = (RF_matrix_ssp2(mpc_cycle-48,1)-RF_matrix_ssp2(mpc_cycle-49,1))/0.02;
                                rfootd[2] = (RF_matrix_ssp2(mpc_cycle-48,2)-RF_matrix_ssp2(mpc_cycle-49,2))/0.02;

                                lfootd[0] = (LF_matrix_ssp2(mpc_cycle-48,0)-LF_matrix_ssp2(mpc_cycle-49,0))/0.02;
                                lfootd[1] = (LF_matrix_ssp2(mpc_cycle-48,1)-LF_matrix_ssp2(mpc_cycle-49,1))/0.02;
                                lfootd[2] = (LF_matrix_ssp2(mpc_cycle-48,2)-LF_matrix_ssp2(mpc_cycle-49,2))/0.02;
                            }
                        }
                        if(mpc_cycle > 48)
                        {
                            zmp_mpcx = desired_val.m_shared_memory[43] + 0.0522;
                            zmp_mpcy = desired_val.m_shared_memory[47];
                        }
                        else
                        {
                            zmp_mpcx = desired_val.m_shared_memory[43];
                            zmp_mpcy = desired_val.m_shared_memory[47];
                        }
                    }
                }

                //ZMP_measured = WBC::GetZMPpos_fromFT(rd_);

                if ((walking_tick == 0) && (walking_tick_stop == false))// && mpc_cycle < 50)
                {
                }
                else
                {
                    if(walking_tick_stop == false && mpc_cycle <= controlwalk_time)
                    {
                        zmpControl(rd_);
                        //std::cout << "CONTROL" << std::endl;
                        startTime1 = std::chrono::system_clock::now();
                        
                        comd_(0) = comd(0)+ 3.0 * (comd(0) - rd_.link_[COM_id].v(0));// + 3.0 * (com_mpc(0) - rd_.link_[COM_id].xpos(0));//+ 0.05 * (comd(0) - rd_.link_[COM_id].v(0));
                        comd_(1) = comd(1)+ 3.0 * (comd(1) - rd_.link_[COM_id].v(1));;// + 3.0 * (com_mpc(1) - rd_.link_[COM_id].xpos(1));//+ 0.05 * (comd(1) - rd_.link_[COM_id].v(1));
                        comd_(2) = comd(2);

                        angd_(0) = angm(0);// + 0.5 * (angm(0) - model_data1.hg.angular()(1));//+ 0.05 * (comd(0) - rd_.link_[COM_id].v(0));
                        angd_(1) = angm(1);// + 0.5 * (angm(1) - model_data1.hg.angular()(0));//+ 0.05 * (comd(1) - rd_.link_[COM_id].v(1));
                        
                        //comd_(2) = comd(2) + 0.3 * (comd(2) - rd_.link_[COM_id].v(2));
                        //upperd.setZero();
                        //qd_pinocchio.setZero();

                        //RF_trajectory_float.linear() = RF_trajectory_float.linear() * DyrosMath::rotateWithY(control_input(2)) * DyrosMath::rotateWithX(control_input(3));
                        //LF_trajectory_float.linear() = LF_trajectory_float.linear() * DyrosMath::rotateWithY(control_input(0)) * DyrosMath::rotateWithX(control_input(1));
                        Eigen::Vector3d rfoot_ori, lfoot_ori, rfootd1, lfootd1;
                        rfoot_ori.setZero();
                        lfoot_ori.setZero();
                        rfoot_ori(0) = (control_input(3));
                        rfoot_ori(1) = (control_input(2));
                        lfoot_ori(0) = (control_input(1));
                        lfoot_ori(1) = (control_input(0));

                        rfootd1 = rfootd;
                        lfootd1 = lfootd;
                        if(contactMode == 1)
                        {
                            rfootd1(2) = rfootd(2) - 2.0 * zd_ctrl(2);
                            lfootd1(2) = lfootd(2) + 2.0 * zd_ctrl(2);
                        }
                        
                        momentumControl(rd_, comd_, angd_, rfootd1, lfootd1, upperd, rfoot_ori, lfoot_ori);
                        endTime1 = std::chrono::system_clock::now();
                        auto elapsed2 = std::chrono::duration_cast<std::chrono::microseconds>(endTime1 - startTime1);
                        qd_pinocchio.segment<18>(0) = q_dm;
                        Eigen::VectorQVQd q_pinocchio_temp;
                        q_pinocchio_temp = q_pinocchio;
                        q_pinocchio_temp(20) = q_pinocchio_desired(20);
                        q_pinocchio_temp(21) = q_pinocchio_desired(21);
                        if(walking_tick == 0)
                        {
                            q_pinocchio_desired = q_pinocchio_temp;
                        }
                        
                                                
                        if(mpc_cycle == 1)
                            q_pinocchio_desired = pinocchio::integrate(model, q_pinocchio_temp, qd_pinocchio * 0.0005);
                        else
                            q_pinocchio_desired = pinocchio::integrate(model, q_pinocchio_desired, qd_pinocchio * 0.0005);
                        
                        qd_pinocchio_ = qd_pinocchio;

                    }
                    else
                    {
                        MOMX.setZero();
                        COMX.setZero();
                    }
                    cc_mutex.lock();
                    for (int i = 0; i < MODEL_DOF; i++)
                    {
                        rd_.q_desired(i) = q_pinocchio_desired(i+7) + 0.5 * (q_pinocchio_desired(i+7) - rd_.q_(i));//(q_init(i)*(40-walking_tick) + q_pinocchio_desired(i+7) * walking_tick)/40.0;
                    }
                    cc_mutex.unlock();
                }

                /*if(mpc_cycle == 32)
                {
                    std::cout << "Rd" << walking_tick << std::endl;
                    std::cout << rd_.q_desired.segment<18>(0).transpose() << std::endl;
                    std::cout << rd_.q_.segment<18>(0).transpose() << std::endl;
                }*/
                    

                if(walking_tick == 20)
                {
                    if(mpc_cycle <= controlwalk_time)
                        mpc_start_init.m_shared_memory_int[0] = 1;
                    /*
                    if(mpc_cycle < 50)
                    {
                        std::cout << "LF  " << walking_tick << " " << mpc_cycle << std::endl;
                        std::cout << LF_matrix(mpc_cycle+1,0) <<" " << LF_matrix(mpc_cycle+1,1) << " "<< LF_matrix(mpc_cycle+1,2) + 0.15842 << std::endl;
                        std::cout << "RF" << std::endl;
                        std::cout << RF_matrix(mpc_cycle+1,0) <<" " << RF_matrix(mpc_cycle+1,1) << " "<< RF_matrix(mpc_cycle+1,2) +0.15842 << std::endl;
                    }
                    else
                    {
                        std::cout << "LF  " << walking_tick << " " << mpc_cycle << std::endl;
                        std::cout << LF_matrix_ssp2(mpc_cycle-48,0) + 0.0522 <<" " << LF_matrix_ssp2(mpc_cycle-48,1) << " "<< LF_matrix_ssp2(mpc_cycle-48,2) + 0.15842 << std::endl;
                        std::cout << "RF" << std::endl;
                        std::cout << RF_matrix_ssp2(mpc_cycle-48,0) + 0.0522 <<" " << RF_matrix_ssp2(mpc_cycle-48,1) << " "<< RF_matrix_ssp2(mpc_cycle-48,2) + 0.15842 << std::endl;
                    }*/
                }
                if(walking_tick == 40)
                {
                    walking_tick = 0;
                    walking_tick_stop = true;
                    mpc_cycle = mpc_cycle + 1;
                }

                if (walking_tick_stop == false)
                    walking_tick = walking_tick + 1;

                auto endTime = std::chrono::system_clock::now();
                if(mpc_cycle < controlwalk_time)
                {   
                    Eigen::Vector12d q_temp;
                    if(walking_tick_stop == false)
                    {
                        for(int i = 0; i < 12; i ++)
                            //q_temp(i) = (q_init(i)*(40-walking_tick) + q_pinocchio_desired(i+7) * walking_tick)/40.0;
                            q_temp(i)  = q_pinocchio_desired(i+7);
                    }
                    else
                    {       
                        for(int i = 0; i < 12; i ++)
                            q_temp(i)  = q_pinocchio_desired(i+7);
                    }
                    
                    if(mpc_cycle < controlwalk_time-1)
                    {
                        
                       /* if(mpc_cycle < 50)
                            file[1]  <<  mpc_cycle << " " << RF_matrix(mpc_cycle+1,2)<< " " << LF_matrix(mpc_cycle+1,2) << " " << rd_.link_[Right_Foot].xipos(0) << " "<<  RF_matrix(mpc_cycle+1,1) << " " << rd_.link_[Right_Foot].xipos(1) << " "<<  RF_matrix(mpc_cycle+1,2) << " " << rd_.link_[Right_Foot].xipos(2) -0.07236<< " " << com_mpc[0] << " " << rd_.link_[COM_id].xpos(0) << " " << com_mpc[1] << " " << rd_.link_[COM_id].xpos(1) << std::endl;
                        else
                            file[1]  <<  mpc_cycle << " " << RF_matrix_ssp2(mpc_cycle-48,2) << " " << LF_matrix_ssp2(mpc_cycle-48,2) << " " << rd_.link_[Right_Foot].xipos(0) << " "<<  RF_matrix_ssp2(mpc_cycle-48,1) << " " << rd_.link_[Right_Foot].xipos(1) << " "<<  RF_matrix_ssp2(mpc_cycle-48,2) << " " << rd_.link_[Right_Foot].xipos(2) << " " << com_mpc[0] << " " << rd_.link_[COM_id].xpos(0) << " " << com_mpc[1] << " " << rd_.link_[COM_id].xpos(1) << std::endl;
                        */

                        file[1] << mpc_cycle << " "<<  com_mpc[0] << " " << rd_.link_[COM_id].xpos(0) << " " << com_mpc[1] << " " << rd_.link_[COM_id].xpos(1) << " " << zmp_mpcx << " " << ZMP_FT_law(0) << " " << zmp_mpcy << " " <<ZMP_FT_law(1)<< " " << control_input1(0)<< " " << control_input1(1)<< " " << control_input1(2)<< " " << control_input1(3)<<std::endl;//" "<< q_pinocchio_desired(20) << " " << rd_.q_(13)<< " " << q_pinocchio_desired(21) << " " << rd_.q_(14)<< std::endl;
                    }
                    auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);
                    auto elapsed1 = std::chrono::duration_cast<std::chrono::microseconds>(endTime1 - startTime1);
                    
                    COM_prev1  <<rd_.link_[COM_id].xpos(0), rd_.link_[COM_id].xpos(1), 0.0;
                }
            }

            
            /*
            else if(statemachine.m_shared_memory_int[0] == 1)
            {
                for (int i = 0; i < 6; i ++)
                    state_init.m_shared_memory[i] = rd_.q_virtual_[i];

                state_init.m_shared_memory[6] = rd_.q_virtual_[39];
                
                for (int i = 6; i < 18; i ++)
                    state_init.m_shared_memory[i+1] = rd_.q_virtual_[i];

                for (int i = 0; i < 18; i ++)
                    state_init.m_shared_memory[i+21] = rd_.q_dot_virtual_[i];
                
                state_init.m_shared_memory[20] = rd_.q_virtual_[19];
                state_init.m_shared_memory[21] = rd_.q_virtual_[20];
                state_init.m_shared_memory[41] = rd_.link_[COM_id].xpos(0);
                state_init.m_shared_memory[42] = rd_.link_[COM_id].xpos(1);
                mpc_start_init.m_shared_memory_int[0] = 1;
            }
            */
        }

    }
}

void CustomController::GuiCommandCallback(const std_msgs::StringConstPtr &msg)
{
    if (msg->data == "stateestimation")
    {   
        stateestimation_count = stateestimation_count + 1;
        if (stateestimation_count >= 3 && stateestimation_count % 2 == 1)
        {
            std::cout << "STate" << stateestimation_count << std::endl;
            stateestimation = true;
        }
    }

    if (msg->data == "simvirtualjoint")
    {
        std::cout << "STate" << stateestimation_count << std::endl;
        stateestimation = true;
    }
}

void CustomController::computePlanner()
{
}

void CustomController::copyRobotData(RobotData &rd_l)
{
    std::memcpy(&rd_cc_, &rd_l, sizeof(RobotData));
}

void CustomController::momentumControl(RobotData &Robot, Eigen::Vector3d comd,  Eigen::Vector2d ang_, Eigen::Vector3d rfootd, Eigen::Vector3d lfootd, Eigen::Vector2d upperd, Eigen::Vector3d rfootori, Eigen::Vector3d lfootori)
{
    if(upper_on == false)
    {
        int variable_size, constraint_size;

        variable_size = 18;
        constraint_size = 17;

        MatrixXd J;
        VectorXd X;
        X.setZero(constraint_size);


        J.setZero(constraint_size, variable_size);
        J.block(0,0,3,18) = Robot.link_[COM_id].Jac().block(0,0,3,18);
        J.block(3,0,2,18) = CMM.block(3,0,2,18);
        J.block(5,0,6,18) = Robot.link_[Right_Foot].Jac().block(0,0,6,18);
        J.block(11,0,6,18) = Robot.link_[Left_Foot].Jac().block(0,0,6,18);
    
        X.segment<3>(0) = comd;
        X.segment<2>(3) = ang_;
        X.segment<3>(5) = rfootd;
        X.segment<3>(11) = lfootd;
        Eigen::MatrixXd pinv = J.completeOrthogonalDecomposition().pseudoInverse() ;

        q_dm = pinv * X;
    }
    else
    {
        int variable_size, constraint_size;

        variable_size = 18;
        constraint_size = 17;
  
        MatrixXd J;
        VectorXd X;
        X.setZero(constraint_size);

        //std::cout << "aaaaa" << std::endl;
        //std::cout << CMM.block(3,18,3,3) << std::endl;

        MOMX = CMM.block(3,19,2,2) * upperd;
        COMX = Robot.link_[COM_id].Jac().block(0,19,3,2) * upperd;

        J.setZero(constraint_size, variable_size);
        J.block(0,0,3,18) = Robot.link_[COM_id].Jac().block(0,0,3,18);
        J.block(3,0,2,18) = CMM.block(3,0,2,18);
        J.block(5,0,6,18) = Robot.link_[Right_Foot].Jac().block(0,0,6,18);
        J.block(11,0,6,18) = Robot.link_[Left_Foot].Jac().block(0,0,6,18);
    
        X.segment<3>(0) = comd - COMX;
        X.segment<2>(3) = ang_ - MOMX;
        X.segment<3>(5) = rfootd;
        X.segment<3>(8) = rfootori;
        X.segment<3>(11) = lfootd;
        X.segment<3>(14) = lfootori;
        Eigen::MatrixXd pinv = DyrosMath::pinv_COD(J);
        q_dm = pinv * X;        
    }
}

void CustomController::zmpControl(RobotData &Robot)
{
    Eigen::Vector3d pr, pl;
    if(walking_tick == 0 && mpc_cycle == 0)
    {
        control_input.setZero();
        control_input1.setZero();
        control_input1prev.setZero();
    }
    if (walking_tick >= 1 && mpc_cycle != 0)
    {
        if(walking_tick == 1 && mpc_cycle == 1)
        {
            Fl_prev = rd_.LF_CF_FT;
            Fr_prev = rd_.RF_CF_FT;
        }

        Fl = rd_.LF_CF_FT;
        Fr = rd_.RF_CF_FT;
        
        for (int i = 0; i < 6; i++)
        {
            if(contactMode == 1)
            {
                Fr_l(i) = rd_.RF_CF_FT(i);//DyrosMath::lowPassFilter(Fr(i), Fr_prev(i), 1 / wk_Hz, 1 / (2 * 3.14 * 30));
                Fl_l(i) = rd_.LF_CF_FT(i);//DyrosMath::lowPassFilter(Fl(i), Fl_prev(i), 1 / wk_Hz, 1 / (2 * 3.14 * 30));
            }
            else
            {
                Fr_l(i) = DyrosMath::lowPassFilter(Fr(i), Fr_prev(i), 1 / wk_Hz, 1 / (2 * 3.14 * 30));
                Fl_l(i) = DyrosMath::lowPassFilter(Fl(i), Fl_prev(i), 1 / wk_Hz, 1 / (2 * 3.14 * 30));
            }
        }
        Fr_prev = Fr_l;
        Fl_prev = Fl_l;

        //std::cout << "ZMP" << std::endl;
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
        int zmp_delx = 0.0;
        int zmp_dely = 0.0;

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
    
        for (int i = 0; i < 2; i++)
        {
            if (desired_ankle_torque(i) > 300)
            {
                desired_ankle_torque(i) = 300;
            }

            if (desired_ankle_torque(i) < -300)
            {
                desired_ankle_torque(i) = -300;
            }
        }

        LT(0) = (1 - alpha) * desired_ankle_torque(0);
        RT(0) = (alpha)*desired_ankle_torque(0);

        LT(1) = (1 - alpha) * desired_ankle_torque(1);
        RT(1) = (alpha)*desired_ankle_torque(1);

        dspForceControl(Robot, alpha);

        double arp_l, ark_l, app_l, apk_l, arp_r, ark_r, app_r, apk_r;
        arp_dl = 70;
        ark_dl = 0.03; //95 0.05 double
        app_dl = 65;
        apk_dl = 0.04; // 35 ㅐ0.04

        arp_dr = 70;
        ark_dr = 0.03;
        app_dr = 65;
        apk_dr = 0.04;

        arp_sl = 30;
        ark_sl = 0.05;
        app_sl = 20;
        apk_sl = 0.05;

        arp_sr = 30;
        ark_sr = 0.05;
        app_sr = 20;
        apk_sr = 0.05;//0.06

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

        if(walking_tick == 1 && mpc_cycle == 1)
        {
            LT_prev = LT;
            RT_prev = RT;
        }

        for (int i = 0; i < 2; i++)
        {
            LT_l(i) = DyrosMath::lowPassFilter(LT(i), LT_prev(i), 1 / 2000.0, 1 / (2.0 * 3.14 * 6.0));
            RT_l(i) = DyrosMath::lowPassFilter(RT(i), RT_prev(i), 1 / 2000.0, 1 / (2.0 * 3.14 * 6.0));
        }

        LT_prev = LT_l;
        RT_prev = RT_l;

        if(contactMode == 1)
        {
            control_input1(0) = apk_l / 2000.0 * (LT(1) - Fl_l(4)) + (1 - app_l / 2000.0) * control_input1(0); //pitch
            control_input1(1) = ark_l / 2000.0 * (LT(0) - Fl_l(3)) + (1 - arp_l / 2000.0) * control_input1(1); //roll
            control_input1(2) = apk_r / 2000.0 * (RT(1) - Fr_l(4)) + (1 - app_r / 2000.0) * control_input1(2);
            control_input1(3) = ark_r / 2000.0 * (RT(0) - Fr_l(3)) + (1 - arp_r / 2000.0) * control_input1(3);
        }

        /*control_input(0) = apk_l/ 5.0 * (LT(1) - Fl_l(4)) + (- 1.0/app_l) * control_input(0); //pitch
        control_input(1) = ark_l/ 5.0 * (LT(0) - Fl_l(3)) + ( - 1.0/arp_l) * control_input(1); //roll
        control_input(2) = apk_r/ 5.0 * (RT(1) - Fr_l(4)) + (- 1.0/app_r) * control_input(2);
        control_input(3) = ark_r/ 5.0 * (RT(0) - Fr_l(3)) + (- 1.0/arp_r) * control_input(3);
        */
        //std::cout << control_input(0) << " "<< control_input(2)  << " " << LT(1) << " " << LT(0)<< " " << Fl_l(4)<< " " << desired_ankle_torque(1)<< " " << apk_l<< std::endl; 
        if (contactMode == 2)
        {
            control_input1(0) = apk_l / 2000.0 * (LT(1) - Fl_l(4)) + (1 - app_l / 2000.0) * control_input1(0); //pitch
            control_input1(1) = ark_l / 2000.0 * (LT(0) - Fl_l(3)) + (1 - arp_l / 2000.0) * control_input1(1); //roll
            
            control_input1(2) = (1 - 50 / 2000.0) * control_input1(2);
            control_input1(3) = (1 - 50 / 2000.0) * control_input1(3);
        }
        else if (contactMode == 3)
        {
            control_input1(0) = (1 - 50 / 2000.0) * control_input1(0); //pitch
            control_input1(1) = (1 - 50 / 2000.0) * control_input1(1); //roll

            control_input1(2) = apk_r / 2000.0 * (RT(1) - Fr_l(4)) + (1 - app_r / 2000.0) * control_input1(2);
            control_input1(3) = ark_r / 2000.0 * (RT(0) - Fr_l(3)) + (1 - arp_r / 2000.0) * control_input1(3);
        }

        //posture_input(0) = kc_r / 1000.0 * (-Robot.roll) + (1 - tc_r / 1000.0) * posture_input(0);  //pitch
        //posture_input(1) = kc_p / 1000.0 * (-Robot.pitch) + (1 - tc_p / 1000.0) * posture_input(1); //roll

        for (int i = 0; i < 4; i++)
        {
            if (contactMode == 1)
            {
                if (i == 1 || i == 3)
                {
                    if (control_input1(i) > 0.1500)
                    {
                        control_input1(i) = 0.1500;
                    }

                    if (control_input1(i) < -0.1500)
                    {
                        control_input1(i) = -0.1500;
                    }
                }
                else
                {
                    if (control_input1(i) > 0.1500)
                    {
                        control_input1(i) = 0.1500;
                    }

                    if (control_input1(i) < -0.1500)
                    {
                        control_input1(i) = -0.1500;
                    }
                }
            }
            else
            {
                if (i == 1 || i == 3)
                {
                    if (control_input1(i) > 0.1500)
                    {
                        control_input1(i) = 0.1500;
                    }

                    if (control_input1(i) < -0.1500)
                    {
                        control_input1(i) = -0.1500;
                    }
                }
                else
                {
                    if (control_input1(i) > 0.1500)
                    {
                        control_input1(i) = 0.1500;
                    }

                    if (control_input1(i) < -0.1500)
                    {
                        control_input1(i) = -0.1500;
                    }
                }
            }
            control_input(i) = (control_input1(i) - control_input1prev(i)) * 2000;
            control_input1prev(i) = control_input1(i);
        }
    }
}

void CustomController::dspForceControl(RobotData &Robot, double alpha)
{
    RF_d.setZero();
    LF_d.setZero();
    RF_d(2) = -1 * alpha * Robot.total_mass_ * GRAVITY;
    LF_d(2) = -1 * (1 - alpha) * Robot.total_mass_ * GRAVITY;

    double Kfx, Tfx, Kfy, Tfy, Kfz, Tfz;
    K_fx = 0.0001;
    T_fx = 3;
    K_fy = 0.0001;
    T_fy = 3;
    K_fz = 0.0001;
    T_fz = 3;
/*
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
    */
    Kfx = K_fx;
    Tfx = T_fx;
    Kfy = K_fy;
    Tfy = T_fy;
    Kfz = K_fz;
    Tfz = T_fz;

    F_diff = LF_d - RF_d;
    F_diff_m = Fl_l.segment<3>(0) - Fr_l.segment<3>(0);
    //z_ctrl(0) = K_fx / wk_Hz * (F_diff(0) - F_diff_m(0)) + (1 - 1 / (T_fx * wk_Hz)) * z_ctrl(0);
    //z_ctrl(1) = K_fy / wk_Hz * (F_diff(1) - F_diff_m(1)) + (1 - 1 / (T_fy * wk_Hz)) * z_ctrl(1);
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
        zd_ctrl(i) = (z_ctrl(i) - z_ctrl_prev(i)) * 2000.0;
    }
    

    z_ctrl_prev = z_ctrl;
}