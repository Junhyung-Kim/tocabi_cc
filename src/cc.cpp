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
#include "shm_msgs.h"

using namespace TOCABI;
using namespace pinocchio;
using std::shared_ptr;

pinocchio::Data model_data;
pinocchio::Data model_data1;
pinocchio::Data model_data2;

pinocchio::Model model;

SHMmsgs *mj_shm_;
int shm_msg_id;

CSharedMemory mpc_start_init, state_init, statemachine, desired_val;

CustomController::CustomController(RobotData &rd) : rd_(rd) //, wbc_(dc.wbc_)
{
    upper_on = true;
    time_tick = false;
    time_tick_next = false;
    time_tick_starts = false;
    ControlVal_.setZero();
    for (int i = 0; i < 2; i++)
    {
        file[i].open(FILE_NAMES[i].c_str(), std::ios_base::out);
    }
    mpc_cycle = 0;
    mpc_cycle_prev = 1;

    COMX.setZero(3);
    MOMX.setZero(2);
    rd_.mujoco_dist = false;
    stateestimation = true;

    q_dot_virtual_lpf_.setZero();


    nh.setCallbackQueue(&queue_cc_);
    gui_sub_ = nh.subscribe("/chatter", 1, &CustomController::GuiCommandCallback, this);
   
    qp_result_prev.setZero(MODEL_DOF_VIRTUAL+12);
   
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
    nle.setZero(MODEL_DOF_VIRTUAL);

    nh.getParam("/tocabi_controller/dist", dist);

    mobgain.push_back(mobgain1);
    mobgain.push_back(mobgain2);
    mobgain.push_back(mobgain3);
    mobgain.push_back(mobgain4);
    mobgain.push_back(mobgain5);
    mobgain.push_back(mobgain6);

    RFj1.setZero(6, MODEL_DOF_VIRTUAL);
    LFj1.setZero(6, MODEL_DOF_VIRTUAL);

    init_shm(shm_msg_key, shm_msg_id, &mj_shm_);

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

    pinocchio::Model model1;

    //pinocchio::urdf::buildModel("/home/jhk/tocabi.urdf", root_joint, model1);
    pinocchio::urdf::buildModel("/home/jhk/catkin_ws/src/dyros_tocabi_v2/tocabi_description/robots/dyros_tocabi.urdf", root_joint, model1);
    ///home/jhk/catkin_ws/src/dyros_tocabi_v2/tocabi_description/robots/dyros_tocabi.urdf
    model = model1;
    pinocchio::SE3 M = pinocchio::SE3::Identity();
    M.translation() << 0.03,0.0,-0.1585;
    RFjoint_id = model.getJointId("R_AnkleRoll_Joint");
    LFjoint_id = model.getJointId("L_AnkleRoll_Joint");
    RFframe_id = model.getFrameId("R_Foot_Link");
    LFframe_id = model.getFrameId("L_Foot_Link");
    model.addBodyFrame("LF_contact", LFjoint_id, M, LFframe_id);
    model.addBodyFrame("RF_contact", RFjoint_id, M, RFframe_id);
    LFcframe_id = model.getFrameId("LF_contact");
    RFcframe_id = model.getFrameId("RF_contact");
    RFframe_id = model.getFrameId("R_Foot_Link");
    LFframe_id = model.getFrameId("L_Foot_Link");
    pinocchio::Data data(model);
    model_data = data;
    model_data1 = data;
    model_data2 = data;
    q_pinocchio.setZero();
    mpc_on = false;
    wlk_on = false;
    walking_tick = 0;
    J_RFF.setZero(6,MODEL_DOF_VIRTUAL);
    J_LFF.setZero(6,MODEL_DOF_VIRTUAL);
    q_dm.resize(18);
    qdot_command.resize(18);
    qdd_pinocchio_desired1.setZero();
    qd_pinocchio_prev.setZero();
    G_temp.setZero(12, MODEL_DOF_VIRTUAL);
    g_temp.setZero();

    variable_size1 = MODEL_DOF_VIRTUAL + 12;
    constraint_size1 = 63 + 12;//6 + 12 + 8 + 8 + 1 + MODEL_DOF_VIRTUAL;// + 1;

    qp_torque_control.InitializeProblemSize(variable_size1, constraint_size1);
    qp_torque_control.EnableEqualityCondition(0.001);
    variable_size2 = 18;
    constraint_size2 = 15;
    qp_result1.setZero(variable_size2);
    qp_momentum_control.InitializeProblemSize(variable_size2, constraint_size2);

    J.setZero(constraint_size2, variable_size2);
    X.setZero(constraint_size2);

    qp_momentum_control.EnableEqualityCondition(0.001);
        
    qp_result.setZero(MODEL_DOF_VIRTUAL+12);
    tau_.resize(MODEL_DOF_VIRTUAL);
    tau_.setZero();

    RFj.resize(6, MODEL_DOF_VIRTUAL);
    LFj.resize(6, MODEL_DOF_VIRTUAL);
    RFdj.resize(6, MODEL_DOF_VIRTUAL);
    LFdj.resize(6, MODEL_DOF_VIRTUAL);

    virtual_temp.setZero();
    foot_temp.setZero();

    lfootz = 0.159;
    rfootz = 0.159;

    q_dm.setZero();
    qdot_command.setZero();
   
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
    state_init.setupSharedMemory(sizeof(double) * 50);
    state_init.attachSharedMemory();
    std::cout << "SHAREDMEMORY Second OK"<< std::endl;
    statemachine.setKey(3);
    statemachine.setupSharedMemoryRead(sizeof(int) * 3);
    statemachine.attachSharedMemoryint();
    std::cout << "SHAREDMEMORY Th2ird  OK"<< std::endl;
    desired_val.setKey(4);
    desired_val.setupSharedMemoryRead(sizeof(double) * 49);
    desired_val.attachSharedMemory();

    std::cout << "SHAREDMEMORY Fourth OK"<< std::endl;
    mpc_start_init.m_shared_memory_int[0] = 5;
    mpc_start_init.m_shared_memory_int[1] = 3;
    mpc_start_init.m_shared_memory_int[2] = 3;
    std::cout << "Custom Controller Init" << std::endl;

    initrpy.setZero(7);

    if(as == 0)
    {
        rf = "/home/jhk/catkin_ws/src/tocabi_cc/wpwkfl/lfpos1.txt";
        rf1 = "/home/jhk/catkin_ws/src/tocabi_cc/wpwkfl/rfpos1.txt";
        rf2 = "/home/jhk/catkin_ws/src/tocabi_cc/wpwkfl/zmpbx1.txt";
        rf3 = "/home/jhk/catkin_ws/src/tocabi_cc/wpwkfl/com_ref_lipm.txt";
        rf4 = "/home/jhk/catkin_ws/src/tocabi_cc/wpwkfl/comd_re_lipm.txt";
        rf5 = "/home/jhk/catkin_ws/src/tocabi_cc/wpwkfl/angmom_de_lipm.txt";
        rf6 = "/home/jhk/catkin_ws/src/tocabi_cc/wpwkfl/zmp_des_lipm.txt";
        rf7 = "/home/jhk/catkin_ws/src/tocabi_cc/wpwkfl/zmpby1.txt";
    }
    else if(as == 1)
    {
        rf = "/home/jhk/catkin_ws/src/tocabi_cc/wpwkflup1/lfpos1.txt";
        rf1 = "/home/jhk/catkin_ws/src/tocabi_cc/wpwkflup1/rfpos1.txt";
        rf2 = "/home/jhk/catkin_ws/src/tocabi_cc/wpwkflup1/zmpbx1.txt";
        rf3 = "/home/jhk/catkin_ws/src/tocabi_cc/wpwkflup1/com_ref_lipm.txt";
        rf4 = "/home/jhk/catkin_ws/src/tocabi_cc/wpwkflup1/comd_re_lipm.txt";
        rf5 = "/home/jhk/catkin_ws/src/tocabi_cc/wpwkflup1/angmom_de_lipm.txt";
        rf6 = "/home/jhk/catkin_ws/src/tocabi_cc/wpwkflup1/zmp_des_lipm.txt";
        rf7 = "/home/jhk/catkin_ws/src/tocabi_cc/wpwkflup1/zmpby1.txt";
    }
    else
    {
        rf = "/home/jhk/catkin_ws/src/tocabi_cc/LIPFM_CEN/lfpos.txt";
        rf1 = "/home/jhk/catkin_ws/src/tocabi_cc/LIPFM_CEN/rfpos.txt";
        rf2 = "/home/jhk/catkin_ws/src/tocabi_cc/LIPFM_CEN/zmpb.txt";
        rf3 = "/home/jhk/catkin_ws/src/tocabi_cc/LIPFM_CEN/com_ref.txt";
        rf4 = "/home/jhk/catkin_ws/src/tocabi_cc/LIPFM_CEN/comd_re.txt";
        rf5 = "/home/jhk/catkin_ws/src/tocabi_cc/LIPFM_CEN/angmom_de.txt";
        rf6 = "/home/jhk/catkin_ws/src/tocabi_cc/LIPFM_CEN/zmp_des.txt";
        rf7 = "/home/jhk/catkin_ws/src/tocabi_cc/LIPFM_CEN/zmpb.txt";
    }

    std::fstream read_file(rf);
    std::fstream read_file1(rf1);
    std::fstream read_file2(rf2);
    std::fstream read_file7(rf7);
    
    std::fstream read_file3(rf3);
    std::fstream read_file4(rf4);
    std::fstream read_file5(rf5);
    std::fstream read_file6(rf6);

    InitRPYM.linear().setIdentity();
    InitRPYM.translation().setZero();
    InitRPYM(3,3) = 1.0;

    InitRPYM2.setIdentity();
    
    /*
    std::fstream read_file3("/home/jhk/data/walking/com_ref_lipm.txt");
    std::fstream read_file4("/home/jhk/data/walking/comd_re_lipm.txt");
    std::fstream read_file5("/home/jhk/data/walking/angmom_de_lipm.txt");
    std::fstream read_file6("/home/jhk/data/walking/zmp_des_lipm.txt");
    */
   
    std::vector<double> RF_tran, LF_tran, ZMP_bound, com_ref, comd_ref, angm_ref, zmp_ref;
    std::string string_test;
    double jointvalue;
    virtual_temp1.setZero();
    control_input.setZero();
    lfootd1.setZero();
    rfootd1.setZero();

    if (read_file.is_open())
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
    }
    else
    {
        std::cout << "LF OPEN FAIL" << std::endl;
    }

    if (read_file1.is_open())
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
    }
    else
    {
        std::cout << "RF OPEN FAIL" << std::endl;
    }
    
    if(as == 0 || as == 1)
    {
        if (read_file2.is_open() && read_file7.is_open())
        {
            while (!read_file2.eof() && !read_file7.eof())
            {  
                for (int i = 0; i < 2; i++)
                {
                    read_file2 >> string_test;
                    //string_test.erase(find(string_test.begin(), string_test.end(), ','));
                    jointvalue = atof(string_test.c_str());
                    if (abs(jointvalue) >= 0.0)
                    {  
                        ZMP_bound.push_back(jointvalue);
                    }
                }
                for (int i = 0; i < 2; i++)
                {
                    read_file7 >> string_test;
                    //string_test.erase(find(string_test.begin(), string_test.end(), ','));
                    jointvalue = atof(string_test.c_str());
                    if (abs(jointvalue) >= 0.0)
                    {  
                        ZMP_bound.push_back(jointvalue);
                    }
                }
            }
        }
        else
        {
            std::cout << "ZMP OPEN FAIL" << std::endl;
        }
    }
    else
    {
        if (read_file2.is_open() && read_file7.is_open())
        {
            while (!read_file2.eof() && !read_file7.eof())
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
        }
        else
        {
            std::cout << "ZMP OPEN FAIL" << std::endl;
        }
    }

    if (read_file3.is_open())
    {
        while (!read_file3.eof())
        {  
            for (int i = 0; i < 3; i++)
            {
                read_file3 >> string_test;
                //string_test.erase(find(string_test.begin(), string_test.end(), ','));
                jointvalue = atof(string_test.c_str());
                if (abs(jointvalue) >= 0.0)
                {  
                    com_ref.push_back(jointvalue);
                }
            }
        }
    }
    else
    {
        std::cout << "ZMP1 OPEN FAIL" << std::endl;
    }

    if (read_file4.is_open())
    {
        while (!read_file4.eof())
        {  
            for (int i = 0; i < 3; i++)
            {
                read_file4 >> string_test;
                //string_test.erase(find(string_test.begin(), string_test.end(), ','));
                jointvalue = atof(string_test.c_str());
                if (abs(jointvalue) >= 0.0)
                {  
                    comd_ref.push_back(jointvalue);
                }
            }
        }
    }
    else
    {
        std::cout << "ZMP2 OPEN FAIL" << std::endl;
    }

    if (read_file5.is_open())
    {
        while (!read_file5.eof())
        {  
            for (int i = 0; i < 2; i++)
            {
                read_file5 >> string_test;
                //string_test.erase(find(string_test.begin(), string_test.end(), ','));
                jointvalue = atof(string_test.c_str());
                if (abs(jointvalue) >= 0.0)
                {  
                    angm_ref.push_back(jointvalue);
                }
            }
        }
    }
    else
    {
        std::cout << "ZMP3 OPEN FAIL" << std::endl;
    }

    if (read_file6.is_open())
    {
        while (!read_file6.eof())
        {  
            for (int i = 0; i < 2; i++)
            {
                read_file6 >> string_test;
                //string_test.erase(find(string_test.begin(), string_test.end(), ','));
                jointvalue = atof(string_test.c_str());
                if (abs(jointvalue) >= 0.0)
                {  
                    zmp_ref.push_back(jointvalue);
                }
            }
        }
    }
    else
    {
        std::cout << "ZMP4 OPEN FAIL" << std::endl;
    }

    std::cout << "ZMP" << std::endl;
    std::cout << ZMP_bound.size() << std::endl;
    std::cout << "RF_tran" << std::endl;
    std::cout << RF_tran.size() << std::endl;
    std::cout << "LF_tran" << std::endl;
    std::cout << LF_tran.size() << std::endl;
   
    double *RF_tran1 = &RF_tran[0];
    double *LF_tran1 = &LF_tran[0];
    double *ZMP_bound1 = &ZMP_bound[0];
    double *com_ref1 = &com_ref[0];
    double *comd_ref1 = &comd_ref[0];
    double *angm_ref1 = &angm_ref[0];
    double *zmp_ref1 = &zmp_ref[0];

    Eigen::MatrixXd LF_matrix_temp = Eigen::Map<Eigen::MatrixXd>(LF_tran1, 3, 221);
    Eigen::MatrixXd RF_matrix_temp = Eigen::Map<Eigen::MatrixXd>(RF_tran1, 3, 221);
    Eigen::MatrixXd ZMP_bound_temp = Eigen::Map<Eigen::MatrixXd>(ZMP_bound1, 4, 221);
    Eigen::MatrixXd com_matrix_temp = Eigen::Map<Eigen::MatrixXd>(com_ref1, 3, 200);
    Eigen::MatrixXd comd_matrix_temp = Eigen::Map<Eigen::MatrixXd>(comd_ref1, 3, 199);
    Eigen::MatrixXd angm_ref_temp = Eigen::Map<Eigen::MatrixXd>(angm_ref1, 2, 199);
    Eigen::MatrixXd zmp_temp = Eigen::Map<Eigen::MatrixXd>(zmp_ref1, 2, 200);
    
    Eigen::MatrixXd RF_matrix1, LF_matrix1;

    RF_matrix1 = RF_matrix_temp.transpose();
    LF_matrix1 = LF_matrix_temp.transpose();

    /*RF_matrix.resize(221,3);
    LF_matrix.resize(221,3);
   
    for(int i = 0; i < RF_matrix.rows(); i++)
    {
        RF_matrix.row(i) = RF_matrix1.row(i);
        LF_matrix.row(i) = LF_matrix1.row(i);
    }*/
    RF_matrix = RF_matrix1;
    LF_matrix = LF_matrix1;

    ZMP_bound_matrix = ZMP_bound_temp.transpose();
    com_matrix_matrix = com_matrix_temp.transpose();
    comd_matrix_matrix = comd_matrix_temp.transpose();
    angm_ref_matrix = angm_ref_temp.transpose();
    zmp_matrix = zmp_temp.transpose();

    z_ctrl_prev.setZero();
    qd_pinocchio_desired1_prev.setZero();
    qd_pinocchio_desired1.setZero();
       
    walk_start = false;
    WBC::SetContact(rd_, 1, 1);   
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
    //pinocchio::Data model_data2(model);
    Eigen::Vector2d ZMP_l, ZMP_r, ZMP;
    ZMP_l(0) = -rd_.LF_CF_FT(4)/rd_.LF_CF_FT(2);
    ZMP_l(1) = rd_.LF_CF_FT(3)/rd_.LF_CF_FT(2);
    ZMP_r(0) = -rd_.RF_CF_FT(4)/rd_.RF_CF_FT(2);
    ZMP_r(1) = rd_.RF_CF_FT(3)/rd_.RF_CF_FT(2);
    
    int K_temp = 0;
    double b_;
    
    if(lfootz <= 0.159 && rfootz <= 0.159)
    {
        contactMode = 1;
        setContact_1(rd_, 1, 1, 0, 0);
    }
    else if(lfootz <= 0.159)
    {
        contactMode = 2;
        setContact_1(rd_, 1, 0, 0, 0);
    }
    else if(rfootz <= 0.159)
    {
        contactMode = 3;
        setContact_1(rd_, 0, 1, 0, 0);
    }
   
    //tf2::Matrix3x3 m2(InitRPYM.linear()(0,0),InitRPYM.linear()(0,1),InitRPYM.linear()(0,2),InitRPYM.linear()(1,0),InitRPYM.linear()(1,1),InitRPYM.linear()(1,2),InitRPYM.linear()(2,0),InitRPYM.linear()(2,1),InitRPYM.linear()(2,2));
    //m2.getRotation(q1);
    //tf2::Quaternion q(0.0, 0.0, 0.0, 1.0);
    
    tf2::Quaternion q(rd_.q_virtual_(3), rd_.q_virtual_(4), rd_.q_virtual_(5), rd_.q_virtual_(MODEL_DOF_VIRTUAL));
    tf2::Matrix3x3 m(q);
    tf2::Matrix3x3 m1(InitRPYM.linear()(0,0),InitRPYM.linear()(0,1),InitRPYM.linear()(0,2),InitRPYM.linear()(1,0),InitRPYM.linear()(1,1),InitRPYM.linear()(1,2),InitRPYM.linear()(2,0),InitRPYM.linear()(2,1),InitRPYM.linear()(2,2));
    
    m1 = m1 * m;
    m1.getRotation(q1);
    q1.normalize();

    cc_mutex.lock();
    /*
    for (int i = 0; i < 6; i++)
        q_pinocchio[i] = rd_.q_virtual_[i] + initrpy(i);

    q_pinocchio[6] = rd_.q_virtual_[MODEL_DOF_VIRTUAL] + initrpy(6);
    */

    
    for (int i = 0; i < 3; i++)
        q_pinocchio[i] = rd_.q_virtual_[i] + initrpy(i);

    if(pelv_frame == true)
    {
        q_pinocchio[3] = q1.getX();
        q_pinocchio[4] = q1.getY();
        q_pinocchio[5] = q1.getZ();
        q_pinocchio[6] = q1.getW();
    }
    else
    {
        q_pinocchio[3] = rd_.q_virtual_[3];
        q_pinocchio[4] = rd_.q_virtual_[4];
        q_pinocchio[5] = rd_.q_virtual_[5];
        q_pinocchio[6] = rd_.q_virtual_[MODEL_DOF_VIRTUAL];
    }
  
    /*for (int i = 0; i < 6; i++)
        q_pinocchio[i] = rd_.q_virtual_[i] + initrpy(i);

    q_pinocchio[6] = rd_.q_virtual_[MODEL_DOF_VIRTUAL] + initrpy(6);*/
    
    /*
    for (int i = 0; i < 3; i++)
        q_pinocchio[i] = rd_.q_virtual_[i] + initrpy(i);

    q_pinocchio[3] = q1.getX();
    q_pinocchio[4] = q1.getY();
    q_pinocchio[5] = q1.getZ();
    q_pinocchio[6] = q1.getW();
    */
    for (int i = 6; i < MODEL_DOF_VIRTUAL; i ++)
        q_pinocchio[i+1] = rd_.q_virtual_[i];

    //q_pinocchio = q_pinocchio_desired;
    
    cc_mutex.unlock();

    if(control_start == false)
    {   
        for (int i = 0; i < 3; i++)
            q_pinocchio_desired1[i] = rd_.q_virtual_[i] + initrpy(i);

        if(pelv_frame == true)
        {
            q_pinocchio_desired1[3] = q1.getX();
            q_pinocchio_desired1[4] = q1.getY();
            q_pinocchio_desired1[5] = q1.getZ();
            q_pinocchio_desired1[6] = q1.getW();
        }
        else
        {
            q_pinocchio_desired1[3] = rd_.q_virtual_[3];
            q_pinocchio_desired1[4] = rd_.q_virtual_[4];
            q_pinocchio_desired1[5] = rd_.q_virtual_[5];
            q_pinocchio_desired1[6] = rd_.q_virtual_[MODEL_DOF_VIRTUAL];
        }
        /*
        for (int i = 0; i < 6; i++)
            q_pinocchio_desired1[i] = rd_.q_virtual_[i];// + initrpy(i);

        q_pinocchio_desired1[6] = rd_.q_virtual_[MODEL_DOF_VIRTUAL];// + initrpy(6);
        */

        for (int i = 6; i < MODEL_DOF_VIRTUAL; i ++)
            q_pinocchio_desired1[i+1] = rd_.q_virtual_[i];
    
        q_pinocchio_desired = q_pinocchio_desired1;

        RFz = -1 * rd_.RF_CF_FT(2);
        LFz = -1 * rd_.LF_CF_FT(2);
    }
    if(walk_start == true)
    {
        zmpx = (ZMPx_test);
        zmpy = (ZMPy_test);
    }

    //qddot_virtual은 다시해보기
    //pinocchio::normalize(model, q_pinocchio);
    pinocchio::forwardKinematics(model, model_data2, q_pinocchio);
    pinocchio::centerOfMass(model, model_data2, q_pinocchio, rd_.q_dot_virtual_);
    pinocchio::crba(model, model_data2, q_pinocchio);
    
    pinocchio::nonLinearEffects(model, model_data2, q_pinocchio, rd_.q_dot_virtual_);
    pinocchio::computeJointJacobiansTimeVariation(model, model_data2, q_pinocchio, rd_.q_dot_virtual_);
    
    if(control_start == false)
        zmpx = rd_.link_[COM_id].xpos(0);
    /*if (control_start == false)
    {
        //control_start = true;
    }*/

    
    RFj.setZero();
    LFj.setZero();

    cc_mutex.lock();
    
    pinocchio::updateFramePlacements(model, model_data2);//LOCAL_WORLD_ALIGNED
    pinocchio::computeFrameJacobian(model, model_data2, q_pinocchio, RFcframe_id, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, RFj);
    pinocchio::computeFrameJacobian(model, model_data2, q_pinocchio, LFcframe_id, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, LFj);
    pinocchio::getFrameJacobianTimeVariation(model, model_data2, RFcframe_id, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, RFdj);
    pinocchio::getFrameJacobianTimeVariation(model, model_data2, LFcframe_id, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, LFdj);
    cc_mutex.unlock();
    
    /*if((walk_start == false) || rd_.tc_.mode == 9)
    {
        std::cout << "RFj" << std::endl;
        std::cout << RFj << std::endl;

        std::cout << rd_.link_[Right_Foot].Jac()
    }*/

    /*RFj = InitRPYM2 * RFj;
    LFj = InitRPYM2 * LFj;
    RFdj = InitRPYM2 * RFdj;
    LFdj = InitRPYM2 * LFdj;
    
    RFc_float_current.translation() = DyrosMath::multiplyIsometry3dVector3d(InitRPYM, model_data2.oMf[RFcframe_id].translation());
    LFc_float_current.translation() = DyrosMath::multiplyIsometry3dVector3d(InitRPYM, model_data2.oMf[LFcframe_id].translation());
    RFc_float_current.linear() = InitRPYM.linear() * rd_.link_[Right_Foot].rotm;
    LFc_float_current.linear() = InitRPYM.linear() * rd_.link_[Left_Foot].rotm;
    */
    
    RFc_float_current.translation() = model_data2.oMf[RFcframe_id].translation();
    LFc_float_current.translation() = model_data2.oMf[LFcframe_id].translation();
    RFc_float_current.linear() = rd_.link_[Right_Foot].rotm;
    LFc_float_current.linear() = rd_.link_[Left_Foot].rotm;
    
  
    if(as == 0)
    {
        com_alpha = 0.5;
    }
    else if(as == 1)
    {
        if(mpc_cycle >= 11 && mpc_cycle <= 40)
        {
            if(contactMode = 1)
                com_alpha = 0;
            else
                com_alpha = 1;
        }
        else if(mpc_cycle == 10 || mpc_cycle == 9)
        {
            com_alpha = DyrosMath::cubic(walking_tick + 40*(mpc_cycle - 9), 0 , 80, 0.5, 0.0, 0, 0);
        }
        else if(mpc_cycle == 41 || mpc_cycle == 42)
        {
            com_alpha = DyrosMath::cubic(walking_tick + 40*(mpc_cycle - 41), 0 , 80, 0, 0.5, 0, 0);
        }
        else if(mpc_cycle >= 43 && mpc_cycle <= 58)
        {
            com_alpha = 0.5;
        }
        else if(mpc_cycle == 59 || mpc_cycle == 60)
        {
            com_alpha = DyrosMath::cubic(walking_tick + 40*(mpc_cycle - 59), 0 , 80, 0.5, 1, 0, 0);
        }
        else if(mpc_cycle >= 61 && mpc_cycle <= 90)
        {
            com_alpha = 1;
        }
        else if(mpc_cycle == 91 || mpc_cycle == 92)
        {
            com_alpha = DyrosMath::cubic(walking_tick + 40*(mpc_cycle - 91), 0 , 80, 1, 0.5, 0, 0);
        }
        else if(mpc_cycle >= 93 && mpc_cycle <= 108)
        {
            com_alpha = 0.5;    
        }
        else if(mpc_cycle == 109 || mpc_cycle == 110)
        {
            com_alpha = DyrosMath::cubic(walking_tick + 40*(mpc_cycle - 109), 0 , 80, 0.5, 0.0, 0, 0);
        }
        else if(mpc_cycle >= 111 && mpc_cycle <= 140)
        {
            if(contactMode = 1)
                com_alpha = 0;
            else
                com_alpha = 1;
        }
        else if(mpc_cycle == 141 || mpc_cycle == 142)
        {
            com_alpha = DyrosMath::cubic(walking_tick + 40*(mpc_cycle - 141), 0 , 80, 0.0, 0.5, 0, 0);
        }  
        else if(mpc_cycle >= 143 && mpc_cycle <= 158)
        {
            com_alpha = 0.5; 
        } 
        else if(mpc_cycle == 159 || mpc_cycle == 160)
        {
            com_alpha = DyrosMath::cubic(walking_tick + 40*(mpc_cycle - 159), 0 , 80, 0.5, 1.0, 0, 0);
        }  
        else if(mpc_cycle >= 161 && mpc_cycle <= 190)
        {
            com_alpha = 1.0; 
        } 
        else if(mpc_cycle == 191 || mpc_cycle == 192)
        {
            com_alpha = DyrosMath::cubic(walking_tick + 40*(mpc_cycle - 191), 0 , 80, 1.0, 0.5, 0, 0);
        }  
        else if(mpc_cycle >=193)
        {
            com_alpha = 0.5; 
        }  
        else
        {
            com_alpha = 0.5; 
        }
    }
    else
    {
        if(mpc_cycle >= 20 && mpc_cycle <= 50)
        {
            com_alpha = 1.0;    
        }
        else if(mpc_cycle == 18 || mpc_cycle == 19)
        {
            com_alpha = DyrosMath::cubic(walking_tick + 40*(mpc_cycle - 18), 0 , 80, 0.5, 1, 0, 0);
        }
        else if(mpc_cycle == 51 || mpc_cycle == 52)
        {
            com_alpha = DyrosMath::cubic(walking_tick + 40*(mpc_cycle - 51), 0 , 80, 1, 0.5, 0, 0);
        }
        else if(mpc_cycle >= 50 && mpc_cycle <= 66)
        {
            com_alpha = 0.5;
        }
        else if(mpc_cycle == 68 || mpc_cycle == 67)
        {
            com_alpha = DyrosMath::cubic(walking_tick + 40*(mpc_cycle - 67), 0 , 80, 0.5, 0.0, 0, 0);
        }
        else if(mpc_cycle >= 69 && mpc_cycle <= 97)
        {
            if(contactMode == 1)
                com_alpha = 0.0;
            else
                com_alpha = 1.0;
        }
        else if(mpc_cycle == 98 || mpc_cycle == 99)
        {
            com_alpha = DyrosMath::cubic(walking_tick + 40*(mpc_cycle - 98), 0 , 80, 0.0, 0.5, 0, 0);
        }
        else if(mpc_cycle >= 98 && mpc_cycle <= 116)
        {
            com_alpha = 0.5;    
        }
        else if(mpc_cycle == 117 || mpc_cycle == 118)
        {
            com_alpha = DyrosMath::cubic(walking_tick + 40*(mpc_cycle - 117), 0 , 80, 0.5, 1, 0, 0);
        }
        else if(mpc_cycle >= 119 && mpc_cycle <= 147)
        {
            com_alpha = 1.0; 
        }
        else if(mpc_cycle == 148 || mpc_cycle == 149)
        {
            com_alpha = DyrosMath::cubic(walking_tick + 40*(mpc_cycle - 148), 0 , 80, 1.0, 0.5, 0, 0);
        }  
        else if(mpc_cycle >= 148 && mpc_cycle <= 166)
        {
            com_alpha = 0.5;
        } 
        else if(mpc_cycle == 167 || mpc_cycle == 168)
        {
            com_alpha = DyrosMath::cubic(walking_tick + 40*(mpc_cycle - 167), 0 , 80, 0.5, 0.0, 0, 0);
        }  
        else if(mpc_cycle >= 169 && mpc_cycle <= 197)
        {
            if(contactMode == 1)
                com_alpha = 0.0;
            else
                com_alpha = 1.0; 
        } 
        else if(mpc_cycle == 198 || mpc_cycle == 199)
        {
            com_alpha = DyrosMath::cubic(walking_tick + 40*(mpc_cycle - 198), 0 , 80, 0.0, 0.5, 0, 0);
        }  
        else
        {
            com_alpha = 0.5;
        }
    }    
    
    
    double kkkk = 0;

    if(walk_start1 == true && mpc_cycle < controlwalk_time - 1)
    {
    //    file[1] << mpc_cycle << " " <<contactMode<< " "<< walking_tick << " " <<com_alpha << " " <<rfoot_mpc(0)<< " " <<rfoot_mpc(1)<< " " <<rfoot_mpc(2) << " " <<rfoot_mpc(0)<< " " <<rfoot_mpc(1)<< " " <<rfoot_mpc(2) <<  " " << pelv_ori_c(1) << " " << contactMode << std::endl;
    }
    
    if(rd_.tc_.mode == 8 && walk_start1 == true)
    {
        if(lfootz <= 0.159 && rfootz <= 0.159)
        {
            ZMP(0) = ((ZMP_l(0) + model_data2.oMf[LFcframe_id].translation()(0))*rd_.LF_CF_FT(2) + (ZMP_r(0) + model_data2.oMf[RFcframe_id].translation()(0))*rd_.RF_CF_FT(2))/(rd_.RF_CF_FT(2) + rd_.LF_CF_FT(2));
            ZMP(1) = ((ZMP_l(1) + model_data2.oMf[LFcframe_id].translation()(1))*rd_.LF_CF_FT(2) + (ZMP_r(1) + model_data2.oMf[RFcframe_id].translation()(1))*rd_.RF_CF_FT(2))/(rd_.RF_CF_FT(2) + rd_.LF_CF_FT(2));
        }
        else if(lfootz <= 0.159)
        {
            ZMP(0) = ZMP_l(0)+ model_data2.oMf[LFcframe_id].translation()(0);
            ZMP(1) = ZMP_l(1)+ model_data2.oMf[LFcframe_id].translation()(1);
            ZMP_r(0) = 0.0;
            ZMP_r(1) = 0.0;
        }
        else if(rfootz <= 0.159)
        {
            ZMP(0) = ZMP_r(0)+ model_data2.oMf[RFcframe_id].translation()(0);
            ZMP(1) = ZMP_r(1)+ model_data2.oMf[RFcframe_id].translation()(1);
            ZMP_l(0) = 0.0;
            ZMP_l(1) = 0.0;
        }

        if(zmp_init == false)
        {
            ZMP_FT_law(0) = ZMP(0);
            ZMP_FT_law(1) = ZMP(1);
            qp_result(2) = - nle(2) * 0.5;
            qp_result(8) = - nle(2) * 0.5;
            zmp_init  = true;
        }
    
        cc_mutex.lock();
        ZMP_FT_law(0) = ZMP(0);// - zmp_x_int;//DyrosMath::lpf(ZMP(0), ZMP_FT_law(0), 2000, 10);
        ZMP_FT_law(1) = ZMP(1);// - zmp_y_int;//DyrosMath::lpf(ZMP(1), ZMP_FT_law(1), 2000, 10);
        cc_mutex.unlock();

        

        //for(int i = 0; i < MODEL_DOF_VIRTUAL; i++)
        //    q_dot_virtual_lpf_(i) = DyrosMath::lpf(rd_.q_dot_virtual_(i), q_dot_virtual_lpf_(i), 2000, 10);
        q_dot_virtual_lpf_ = rd_.q_dot_virtual_;

        if(control_start == false || control_time > 0)
        {  
            control_start = true;
            
            if(contactMode == 1)
            {    
                double lx, ly, mu;
                ly = 0.048;
                lx = 0.11;
                mu = 0.8;

                if(LFc_float_current.translation()(0) >  RFc_float_current.translation()(0))
                {
                    if(zmpx > LFc_float_current.translation()(0) + lx)
                        zmpx  = LFc_float_current.translation()(0) + lx;
                    if(zmpx < RFc_float_current.translation()(0) - lx)
                        zmpx  = RFc_float_current.translation()(0) - lx;

                    zmp_bx(0) =  LFc_float_current.translation()(0) + lx;
                    zmp_bx(1) =  RFc_float_current.translation()(0) - lx;
                }
                else if(LFc_float_current.translation()(0) <  RFc_float_current.translation()(0))
                {
                    if(zmpx > RFc_float_current.translation()(0) + lx)
                        zmpx  = RFc_float_current.translation()(0) + lx;
                    if(zmpx < LFc_float_current.translation()(0) - lx)
                        zmpx  = LFc_float_current.translation()(0) - lx;
                    zmp_bx(0) =  RFc_float_current.translation()(0) + lx;
                    zmp_bx(1) =  LFc_float_current.translation()(0) - lx;
                }
                else
                {
                    if(zmpx > RFc_float_current.translation()(0) + lx)
                        zmpx  = RFc_float_current.translation()(0) + lx;
                    if(zmpx < LFc_float_current.translation()(0) - lx)
                        zmpx  = LFc_float_current.translation()(0) - lx;
                    zmp_bx(0) =  LFc_float_current.translation()(0) + lx;
                    zmp_bx(1) =  RFc_float_current.translation()(0) - lx;
                }

                if(zmpy > LFc_float_current.translation()(1) + ly)
                {
                    zmpy = LFc_float_current.translation()(1) + ly;
                }

                if(zmpy < RFc_float_current.translation()(1) - ly)
                {
                    zmpy = RFc_float_current.translation()(1) - ly;
                }
                
                ly = 0.05;
                lx = 0.13;

                MatrixXd J1, H1, A1;
                VectorXd X1, g1, lb1, ub1, lbA1, ubA1;
                X1.setZero(constraint_size1);
                J1.setZero(constraint_size1, variable_size1);
                H1.setZero(variable_size1, variable_size1);
                A1.setZero(constraint_size1, variable_size1);
                g1.setZero(variable_size1);
                lbA1.setZero(constraint_size1);
                ubA1.setZero(constraint_size1);
                lb1.setZero(variable_size1);
                ub1.setZero(variable_size1);
                
                M_ = model_data2.M;
                nle = model_data2.nle;

                H1.setIdentity();
                g1.setZero();
                H1.block(12,12,MODEL_DOF_VIRTUAL,MODEL_DOF_VIRTUAL) = 100 *H1.block(12,12,MODEL_DOF_VIRTUAL,MODEL_DOF_VIRTUAL);
                
                lb1.setConstant(variable_size1, -100000);
                ub1.setConstant(variable_size1, 100000);

                H1(2,2) = 5;//0.1;
                H1(8,8) = 5;//0.1;
                g1(2) = - com_alpha * nle(2) * 5;
                g1(8) = - (1-com_alpha) * nle(2) * 5;
                lb1(2) = 0.0;
                lb1(8) = 0.0;

                lb1(15) = 0.0;
                lb1(16) = 0.0;
                ub1(15) = 0.0;
                ub1(16) = 0.0;
                

                Eigen::MatrixXd H1_temp;
                Eigen::VectorXd g1_temp;
                H1_temp.setZero(variable_size1, variable_size1);
                g1_temp.setZero(variable_size1);
                H1_temp.setIdentity();

                /*if(mpc_cycle < 1)
                {
                    H1_temp = 5 * H1_temp;
                    H1_temp(2,2) = 0.0;
                    H1_temp(8,8) = 0.0;
                    qp_result(2) = 0.0;
                    qp_result(8) = 0.0;
                    //qp_result(2) = com_alpha * nle(2);
                    //qp_result(8) = (1-com_alpha) * nle(2);

                    H1 = H1 + H1_temp;
                    g1_temp = -qp_result * 5.0;

                    g1 = g1 + g1_temp;
                }*/
                
                H1.block(15,15,3,3) = 1000 * H1.block(15,15,3,3);
               
                qdd_pinocchio_desired1_ = qdd_pinocchio_desired1;
        
                lbA1.head(6) = (nle + M_ * qdd_pinocchio_desired1_).head(6);
                ubA1.head(6) = (nle + M_ * qdd_pinocchio_desired1_).head(6);
            
                double weight_resi = 0.0;
               
                g1.tail(MODEL_DOF_VIRTUAL) = g1.tail(MODEL_DOF_VIRTUAL) + weight_resi * G_temp.transpose() * g_temp;
                H1.block(12, 12, MODEL_DOF_VIRTUAL, MODEL_DOF_VIRTUAL) += H1.block(12, 12, MODEL_DOF_VIRTUAL, MODEL_DOF_VIRTUAL) + weight_resi * G_temp.transpose() * G_temp;
            
                A1.block(0,0,6,6) = RFj.transpose().block(0,0,6,6);
                A1.block(0,6,6,6) = LFj.transpose().block(0,0,6,6);
                A1.block(0,12,6, MODEL_DOF_VIRTUAL) = -M_.block(0,0,6,MODEL_DOF_VIRTUAL);
            
                A1.block(6,0,1,6)(0,2) = ly;
                A1.block(6,0,1,6)(0,3) = -1;
                lbA1(6) = 0.0;
                ubA1(6) = 100000.0;
                A1.block(7,0,1,6)(0,2) = ly;
                A1.block(7,0,1,6)(0,3) = 1;
                lbA1(7) = 0.0;
                ubA1(7) = 100000.0;
                A1.block(8,0,1,6)(0,2) = lx;
                A1.block(8,0,1,6)(0,4) = -1;
                lbA1(8) = 0.0;
                ubA1(8) = 100000.0;
                A1.block(9,0,1,6)(0,2) = lx;
                A1.block(9,0,1,6)(0,4) = 1;
                lbA1(9) = 0.0;
                ubA1(9) = 100000.0;
                A1.block(10,6,1,6)(0,2) = ly;
                A1.block(10,6,1,6)(0,3) = -1;
                lbA1(10) = 0.0;
                ubA1(10) = 100000.0;
                A1.block(11,6,1,6)(0,2) = ly;
                A1.block(11,6,1,6)(0,3) = 1;
                lbA1(11) = 0.0;
                ubA1(11) = 100000.0;
                A1.block(12,6,1,6)(0,2) = lx;
                A1.block(12,6,1,6)(0,4) = -1;
                lbA1(12) = 0.0;
                ubA1(12) = 100000.0;
                A1.block(13,6,1,6)(0,2) = lx;
                A1.block(13,6,1,6)(0,4) = 1;
                lbA1(13) = 0.0;
                ubA1(13) = 100000.0;

                A1.block(14,0,1,6)(0,2) = mu;
                A1.block(24,0,1,6)(0,0) = -1;
                lbA1(14) = 0.0;
                ubA1(14) = 100000.0;
                A1.block(15,0,1,6)(0,2) = mu;
                A1.block(15,0,1,6)(0,0) = 1;
                lbA1(15) = 0.0;
                ubA1(15) = 100000.0;
                A1.block(16,0,1,6)(0,2) = mu;
                A1.block(16,0,1,6)(0,1) = -1;
                lbA1(16) = 0.0;
                ubA1(16) = 100000.0;
                A1.block(17,0,1,6)(0,2) = mu;
                A1.block(17,0,1,6)(0,1) = 1;
                lbA1(17) = 0.0;
                ubA1(17) = 100000.0;
                A1.block(18,6,1,6)(0,2) = mu;
                A1.block(18,6,1,6)(0,0) = -1;
                lbA1(18) = 0.0;
                ubA1(18) = 100000.0;
                A1.block(19,6,1,6)(0,2) = mu;
                A1.block(19,6,1,6)(0,0) = 1;
                lbA1(19) = 0.0;
                ubA1(19) = 100000.0;
                A1.block(20,6,1,6)(0,2) = mu;
                A1.block(20,6,1,6)(0,1) = -1;
                lbA1(20) = 0.0;
                ubA1(20) = 100000.0;
                A1.block(21,6,1,6)(0,2) = mu;
                A1.block(21,6,1,6)(0,1) = 1;
                lbA1(21) = 0.0;
                ubA1(21) = 100000.0;
            
                A1.block(22,0,1,6)(0,2) = (RFc_float_current.translation()(1)  - zmpy);
                A1.block(22,0,1,6)(0,3) = 1;
                A1.block(22,6,1,6)(0,2) = (LFc_float_current.translation()(1)  - zmpy);
                A1.block(22,6,1,6)(0,3) = 1;
                lbA1(22) = 0.0;
                ubA1(22) = 0.0;

                A1.block(23,0,1,6)(0,2) = (RFc_float_current.translation()(0)  - zmpx);
                A1.block(23,0,1,6)(0,4) = -1;
                A1.block(23,6,1,6)(0,2) = (LFc_float_current.translation()(0)  - zmpx);
                A1.block(23,6,1,6)(0,4) = -1;
                lbA1(23) = 0.0;
                ubA1(23) = 0.0;
            
                A1.block(24,18,MODEL_DOF_VIRTUAL-6, MODEL_DOF_VIRTUAL-6) = M_.block(6,6,MODEL_DOF_VIRTUAL-6, MODEL_DOF_VIRTUAL-6);
                A1.block(24,0,MODEL_DOF_VIRTUAL-6,6) = -1 * RFj.transpose().block(6,0,MODEL_DOF_VIRTUAL-6,6);
                A1.block(24,6,MODEL_DOF_VIRTUAL-6,6) = -1 * LFj.transpose().block(6,0,MODEL_DOF_VIRTUAL-6,6);
            
                Eigen::VectorVQd Mqddot;
                Mqddot = M_ * qdd_pinocchio_desired1_;

                lbA1(24) = -333-nle(6)-Mqddot(6);
                ubA1(24) = 333-nle(6)-Mqddot(6);
                lbA1(25) = -232-nle(7)-Mqddot(7);
                ubA1(25) = 232-nle(7)-Mqddot(7);
                lbA1(26) = -263-nle(8)-Mqddot(8);
                ubA1(26) = 263-nle(8)-Mqddot(8);
                lbA1(27) = -289-nle(9)-Mqddot(9);
                ubA1(27) = 289-nle(9)-Mqddot(9);
                lbA1(28) = -222-nle(10)-Mqddot(10);
                ubA1(28) = 222-nle(10)-Mqddot(10);
                lbA1(29) = -166-nle(11)-Mqddot(11);
                ubA1(29) = 166-nle(11)-Mqddot(11);

                lbA1(30) = -333-nle(12)-Mqddot(12);
                ubA1(30) = 333-nle(12)-Mqddot(12);
                lbA1(31) = -232-nle(13)-Mqddot(13);
                ubA1(31) = 232-nle(13)-Mqddot(13);
                lbA1(32) = -263-nle(14)-Mqddot(14);
                ubA1(32) = 263-nle(14)-Mqddot(14);
                lbA1(33) = -289-nle(15)-Mqddot(15);
                ubA1(33) = 289-nle(15)-Mqddot(15);
                lbA1(34) = -222-nle(16)-Mqddot(16);
                ubA1(34) = 222-nle(16)-Mqddot(16);
                lbA1(35) = -166-nle(17)-Mqddot(17);
                ubA1(35) = 166-nle(17)-Mqddot(17);

                lbA1(36) = -303-nle(18)-Mqddot(18);
                ubA1(36) = 303-nle(18)-Mqddot(18);
                lbA1(37) = -303-nle(19)-Mqddot(19);
                ubA1(37) = 303-nle(19)-Mqddot(19);
                lbA1(38) = -303-nle(20)-Mqddot(20);
                ubA1(38) = 303-nle(20)-Mqddot(20);

                lbA1(39) = -64-nle(21)-Mqddot(21);
                ubA1(39) = 64-nle(21)-Mqddot(21);
                lbA1(40) = -64-nle(22)-Mqddot(22);
                ubA1(40) = 64-nle(22)-Mqddot(22);
                lbA1(41) = -64-nle(23)-Mqddot(23);
                ubA1(41) = 64-nle(23)-Mqddot(23);
                lbA1(42) = -64-nle(24)-Mqddot(24);
                ubA1(42) = 64-nle(24)-Mqddot(24);
                lbA1(43) = -23-nle(25)-Mqddot(25);
                ubA1(43) = 23-nle(25)-Mqddot(25);
                lbA1(44) = -23-nle(26)-Mqddot(26);
                ubA1(44) = 23-nle(26)-Mqddot(26);
                lbA1(45) = -10-nle(27)-Mqddot(27);
                ubA1(45) = 10-nle(27)-Mqddot(27);
                lbA1(46) = -10-nle(28)-Mqddot(28);
                ubA1(46) = 10-nle(28)-Mqddot(28);

                lbA1(47) = -10-nle(29)-Mqddot(29);
                ubA1(47) = 10-nle(29)-Mqddot(29);
                lbA1(48) = -10-nle(30)-Mqddot(30);
                ubA1(48) = 10-nle(30)-Mqddot(30);

                lbA1(49) = -64-nle(31)-Mqddot(31);
                ubA1(49) = 64-nle(31)-Mqddot(31);
                lbA1(50) = -64-nle(32)-Mqddot(32);
                ubA1(50) = 64-nle(32)-Mqddot(32);
                lbA1(51) = -64-nle(33)-Mqddot(33);
                ubA1(51) = 64-nle(33)-Mqddot(33);
                lbA1(52) = -64-nle(34)-Mqddot(34);
                ubA1(52) = 64-nle(34)-Mqddot(34);
                lbA1(53) = -23-nle(35)-Mqddot(35);
                ubA1(53) = 23-nle(35)-Mqddot(35);
                lbA1(54) = -23-nle(36)-Mqddot(36);
                ubA1(54) = 23-nle(36)-Mqddot(36);
                lbA1(55) = -10-nle(37)-Mqddot(37);
                ubA1(55) = 10-nle(37)-Mqddot(37);
                lbA1(56) = -10-nle(38)-Mqddot(38);
                ubA1(56) = 10-nle(38)-Mqddot(38);

                A1(57, 2) = 1.0;
                lbA1(57) = 0.0;
                ubA1(57) = 1000.0;

                A1(58, 8) = 1.0;
                lbA1(58) = 0.0;
                ubA1(58) = 1000.0;

                A1.block(59,0,1,6)(0,2) = mu;
                A1.block(59,0,1,6)(0,5) = 1;
                lbA1(59) = 0.0;
                ubA1(59) = 100000.0;
                A1.block(60,0,1,6)(0,2) = mu;
                A1.block(60,0,1,6)(0,5) = -1;
                lbA1(60) = 0.0;
                ubA1(60) = 100000.0;

                A1.block(61,6,1,6)(0,2) = mu;
                A1.block(61,6,1,6)(0,5) = 1;
                lbA1(61) = 0.0;
                ubA1(61) = 100000.0;
                A1.block(62,6,1,6)(0,2) = mu;
                A1.block(62,6,1,6)(0,5) = -1;
                lbA1(62) = 0.0;
                ubA1(62) = 100000.0;

                G_temp.setZero();
                g_temp.setZero();
                G_temp.block(0,0,6,MODEL_DOF_VIRTUAL) = RFj;
                G_temp.block(6,0,6,MODEL_DOF_VIRTUAL) = LFj;

                g_temp.tail(12) <<  RFdj * q_dot_virtual_lpf_ + RFj * qdd_pinocchio_desired1_, LFdj * q_dot_virtual_lpf_ + LFj * qdd_pinocchio_desired1_;

                A1.block(63,12,12,MODEL_DOF_VIRTUAL)  = G_temp;

                lbA1(63) = -g_temp(0);  
                lbA1(64) = -g_temp(1);  
                lbA1(65) = -g_temp(2);  
                lbA1(66) = -g_temp(3);  
                lbA1(67) = -g_temp(4);  
                lbA1(68) = -g_temp(5); 
                ubA1(63) = -g_temp(0);  
                ubA1(64) = -g_temp(1);  
                ubA1(65) = -g_temp(2);  
                ubA1(66) = -g_temp(3);  
                ubA1(67) = -g_temp(4);  
                ubA1(68) = -g_temp(5);  

                lbA1(69) = -g_temp(6);  
                lbA1(70) = -g_temp(7);  
                lbA1(71) = -g_temp(8);  
                lbA1(72) = -g_temp(9);  
                lbA1(73) = -g_temp(10);  
                lbA1(74) = -g_temp(11); 
                ubA1(69) = -g_temp(6);  
                ubA1(70) = -g_temp(7);  
                ubA1(71) = -g_temp(8);  
                ubA1(72) = -g_temp(9);  
                ubA1(73) = -g_temp(10);  
                ubA1(74) = -g_temp(11);           

                qp_torque_control.UpdateMinProblem(H1, g1);
                qp_torque_control.UpdateSubjectToAx(A1, lbA1, ubA1);
                qp_torque_control.UpdateSubjectToX(lb1, ub1);
                solved = qp_torque_control.SolveQPoases(100, qp_result);
            
                if (solved == true)
                {
                    /*if(control_time >= 1)
                    {
                        if(abs(qp_result_prev(2)-qp_result(2)) >= 0.0)
                        {  
                            tau_ = ((M_ * (qdd_pinocchio_desired1_ + qp_result.segment<MODEL_DOF_VIRTUAL>(12)) + nle - (RFj.transpose() * qp_result.head(6) + LFj.transpose() * qp_result.segment<6>(6))).transpose());
                            qp_result_prev = qp_result;
                        }
                    }
                    else
                    {
                        tau_ = ((M_ * (qdd_pinocchio_desired1_ + qp_result.segment<MODEL_DOF_VIRTUAL>(12)) + nle - (RFj.transpose() * qp_result.head(6) + LFj.transpose() * qp_result.segment<6>(6))).transpose());
                        qp_result_prev = qp_result;
                    }*/
                    //if(mpc_cycle >= 1)
                    tau_ = ((M_ * (qdd_pinocchio_desired1_ + qp_result.segment<MODEL_DOF_VIRTUAL>(12)) + nle - (RFj.transpose() * qp_result.head(6) + LFj.transpose() * qp_result.segment<6>(6))).transpose());   
                }
                control_time = control_time + 1;
            }
            else if(contactMode == 2)
            {
                double lx, ly, mu;
                ly = 0.048;
                lx = 0.11;
                mu = 0.8;

                if(zmpx > model_data2.oMf[LFcframe_id].translation()(0) + lx)
                    zmpx  = model_data2.oMf[LFcframe_id].translation()(0) + lx;
                if(zmpx < model_data2.oMf[LFcframe_id].translation()(0) - lx)
                    zmpx  = model_data2.oMf[LFcframe_id].translation()(0) - lx;

                if(zmpy > model_data2.oMf[LFcframe_id].translation()(1) + ly)
                    zmpy = model_data2.oMf[LFcframe_id].translation()(1) + ly;
                else if(zmpy < model_data2.oMf[LFcframe_id].translation()(1) - ly)
                    zmpy = model_data2.oMf[LFcframe_id].translation()(1) - ly;


                zmp_bx(0) =  model_data2.oMf[LFcframe_id].translation()(0) + lx;
                zmp_bx(1) =  model_data2.oMf[LFcframe_id].translation()(0) - lx;

                ly = 0.05;
                lx = 0.13;

                MatrixXd J1, H1, A1;
                VectorXd X1, g1, lb1, ub1, lbA1, ubA1;

                X1.setZero(constraint_size1);
                J1.setZero(constraint_size1, variable_size1);
                H1.setZero(variable_size1, variable_size1);
                A1.setZero(constraint_size1, variable_size1);
                g1.setZero(variable_size1);
                lbA1.setZero(constraint_size1);
                ubA1.setZero(constraint_size1);
                lb1.setZero(variable_size1);
                ub1.setZero(variable_size1);
            
                M_ = model_data2.M;//rd_.A_;
                nle = model_data2.nle;

                H1.setIdentity();
                g1.setZero();
                H1.block(12,12,MODEL_DOF_VIRTUAL,MODEL_DOF_VIRTUAL) = 1000 *H1.block(12,12,MODEL_DOF_VIRTUAL,MODEL_DOF_VIRTUAL);
                lb1.setConstant(variable_size1, -100000);
                ub1.setConstant(variable_size1, 100000);
            
            
                H1(8,8) = 5;
                g1(8) = - com_alpha * nle(2) * 5;
                lb1(2) = 0.0;
                lb1(8) = 0.0;
                lb1(15) = 0.0;
                lb1(16) = 0.0;
                ub1(15) = 0.0;
                ub1(16) = 0.0;

                H1.block(15,15,3,3) = 100 * H1.block(15,15,3,3);
            
                qdd_pinocchio_desired1_ = qdd_pinocchio_desired1;
            
                double weight_resi = 0.0;
                
                g1.tail(MODEL_DOF_VIRTUAL) = g1.tail(MODEL_DOF_VIRTUAL) + weight_resi * G_temp.transpose() * g_temp;
                H1.block(12, 12, MODEL_DOF_VIRTUAL, MODEL_DOF_VIRTUAL) += H1.block(12, 12, MODEL_DOF_VIRTUAL, MODEL_DOF_VIRTUAL) + weight_resi * G_temp.transpose() * G_temp;
        
                lbA1.head(6) = (nle + M_ * qdd_pinocchio_desired1_).head(6);
                ubA1.head(6) = (nle + M_ * qdd_pinocchio_desired1_).head(6);

                A1.block(0,6,6,6) = LFj.transpose().block(0,0,6,6);
                A1.block(0,12,6, MODEL_DOF_VIRTUAL) = -M_.block(0,0,6,MODEL_DOF_VIRTUAL);
            
                A1.block(6,6,1,6)(0,2) = ly;
                A1.block(6,6,1,6)(0,3) = -1;
                lbA1(6) = 0.0;
                ubA1(6) = 100000.0;
                A1.block(7,6,1,6)(0,2) = ly;
                A1.block(7,6,1,6)(0,3) = 1;
                lbA1(7) = 0.0;
                ubA1(7) = 100000.0;
                A1.block(8,6,1,6)(0,2) = lx;
                A1.block(8,6,1,6)(0,4) = -1;
                lbA1(8) = 0.0;
                ubA1(8) = 100000.0;
                A1.block(9,6,1,6)(0,2) = lx;
                A1.block(9,6,1,6)(0,4) = 1;
                lbA1(9) = 0.0;
                ubA1(9) = 100000.0;

                A1.block(14,6,1,6)(0,2) = mu;
                A1.block(24,6,1,6)(0,0) = -1;
                lbA1(14) = 0.0;
                ubA1(14) = 100000.0;
                A1.block(15,6,1,6)(0,2) = mu;
                A1.block(15,6,1,6)(0,0) = 1;
                lbA1(15) = 0.0;
                ubA1(15) = 100000.0;
                A1.block(16,6,1,6)(0,2) = mu;
                A1.block(16,6,1,6)(0,1) = -1;
                lbA1(16) = 0.0;
                ubA1(16) = 100000.0;
                A1.block(17,6,1,6)(0,2) = mu;
                A1.block(17,6,1,6)(0,1) = 1;
                lbA1(17) = 0.0;
                ubA1(17) = 100000.0;
        
                A1.block(22,6,1,6)(0,2) = (model_data2.oMf[LFcframe_id].translation()(1)  - zmpy);
                A1.block(22,6,1,6)(0,3) = 1;

                lbA1(22) = 0.0;
                ubA1(22) = 0.0;
            
                A1.block(23,6,1,6)(0,2) = (model_data2.oMf[LFcframe_id].translation()(0)  - zmpx);
                A1.block(23,6,1,6)(0,4) = -1;
    
                lbA1(23) = 0.0;
                ubA1(23) = 0.0;
            
                A1.block(24,18,MODEL_DOF_VIRTUAL-6, MODEL_DOF_VIRTUAL-6) = M_.block(6,6,MODEL_DOF_VIRTUAL-6, MODEL_DOF_VIRTUAL-6);
                A1.block(24,6,MODEL_DOF_VIRTUAL-6,6) = -1 * LFj.transpose().block(6,0,MODEL_DOF_VIRTUAL-6,6);
        
                Eigen::VectorVQd Mqddot;
                Mqddot = M_ * qdd_pinocchio_desired1_;

                lbA1(24) = -333-nle(6)-Mqddot(6);
                ubA1(24) = 333-nle(6)-Mqddot(6);
                lbA1(25) = -232-nle(7)-Mqddot(7);
                ubA1(25) = 232-nle(7)-Mqddot(7);
                lbA1(26) = -263-nle(8)-Mqddot(8);
                ubA1(26) = 263-nle(8)-Mqddot(8);
                lbA1(27) = -289-nle(9)-Mqddot(9);
                ubA1(27) = 289-nle(9)-Mqddot(9);
                lbA1(28) = -222-nle(10)-Mqddot(10);
                ubA1(28) = 222-nle(10)-Mqddot(10);
                lbA1(29) = -166-nle(11)-Mqddot(11);
                ubA1(29) = 166-nle(11)-Mqddot(11);

                lbA1(30) = -333-nle(12)-Mqddot(12);
                ubA1(30) = 333-nle(12)-Mqddot(12);
                lbA1(31) = -232-nle(13)-Mqddot(13);
                ubA1(31) = 232-nle(13)-Mqddot(13);
                lbA1(32) = -263-nle(14)-Mqddot(14);
                ubA1(32) = 263-nle(14)-Mqddot(14);
                lbA1(33) = -289-nle(15)-Mqddot(15);
                ubA1(33) = 289-nle(15)-Mqddot(15);
                lbA1(34) = -222-nle(16)-Mqddot(16);
                ubA1(34) = 222-nle(16)-Mqddot(16);
                lbA1(35) = -166-nle(17)-Mqddot(17);
                ubA1(35) = 166-nle(17)-Mqddot(17);

                lbA1(36) = -303-nle(18)-Mqddot(18);
                ubA1(36) = 303-nle(18)-Mqddot(18);
                lbA1(37) = -303-nle(19)-Mqddot(19);
                ubA1(37) = 303-nle(19)-Mqddot(19);
                lbA1(38) = -303-nle(20)-Mqddot(20);
                ubA1(38) = 303-nle(20)-Mqddot(20);

                lbA1(39) = -64-nle(21)-Mqddot(21);
                ubA1(39) = 64-nle(21)-Mqddot(21);
                lbA1(40) = -64-nle(22)-Mqddot(22);
                ubA1(40) = 64-nle(22)-Mqddot(22);
                lbA1(41) = -64-nle(23)-Mqddot(23);
                ubA1(41) = 64-nle(23)-Mqddot(23);
                lbA1(42) = -64-nle(24)-Mqddot(24);
                ubA1(42) = 64-nle(24)-Mqddot(24);
                lbA1(43) = -23-nle(25)-Mqddot(25);
                ubA1(43) = 23-nle(25)-Mqddot(25);
                lbA1(44) = -23-nle(26)-Mqddot(26);
                ubA1(44) = 23-nle(26)-Mqddot(26);
                lbA1(45) = -10-nle(27)-Mqddot(27);
                ubA1(45) = 10-nle(27)-Mqddot(27);
                lbA1(46) = -10-nle(28)-Mqddot(28);
                ubA1(46) = 10-nle(28)-Mqddot(28);

                lbA1(47) = -10-nle(29)-Mqddot(29);
                ubA1(47) = 10-nle(29)-Mqddot(29);
                lbA1(48) = -10-nle(30)-Mqddot(30);
                ubA1(48) = 10-nle(30)-Mqddot(30);

                lbA1(49) = -64-nle(31)-Mqddot(31);
                ubA1(49) = 64-nle(31)-Mqddot(31);
                lbA1(50) = -64-nle(32)-Mqddot(32);
                ubA1(50) = 64-nle(32)-Mqddot(32);
                lbA1(51) = -64-nle(33)-Mqddot(33);
                ubA1(51) = 64-nle(33)-Mqddot(33);
                lbA1(52) = -64-nle(34)-Mqddot(34);
                ubA1(52) = 64-nle(34)-Mqddot(34);
                lbA1(53) = -23-nle(35)-Mqddot(35);
                ubA1(53) = 23-nle(35)-Mqddot(35);
                lbA1(54) = -23-nle(36)-Mqddot(36);
                ubA1(54) = 23-nle(36)-Mqddot(36);
                lbA1(55) = -10-nle(37)-Mqddot(37);
                ubA1(55) = 10-nle(37)-Mqddot(37);
                lbA1(56) = -10-nle(38)-Mqddot(38);
                ubA1(56) = 10-nle(38)-Mqddot(38);
        
                A1(57, 8) = 1.0;
                lbA1(57) = 800.0;
                ubA1(57) = 1000.0;

                A1.block(58,6,1,6)(0,2) = mu;
                A1.block(58,6,1,6)(0,5) = 1;
                lbA1(58) = 0.0;
                ubA1(58) = 100000.0;
                A1.block(59,6,1,6)(0,2) = mu;
                A1.block(59,6,1,6)(0,5) = -1;
                lbA1(59) = 0.0;
                ubA1(59) = 100000.0;

                G_temp.setZero();
                g_temp.setZero();
                G_temp.block(0,0,6,MODEL_DOF_VIRTUAL) = LFj;
                g_temp.head(6) <<  LFdj * q_dot_virtual_lpf_ + LFj * qdd_pinocchio_desired1_;
                
                A1.block(60,12,6,MODEL_DOF_VIRTUAL)  = G_temp;

                lbA1(60) = -g_temp(0);  
                lbA1(61) = -g_temp(1);  
                lbA1(62) = -g_temp(2);  
                lbA1(63) = -g_temp(3);  
                lbA1(64) = -g_temp(4);  
                lbA1(65) = -g_temp(5); 
                ubA1(60) = -g_temp(0);  
                ubA1(61) = -g_temp(1);  
                ubA1(62) = -g_temp(2);  
                ubA1(63) = -g_temp(3);  
                ubA1(64) = -g_temp(4);  
                ubA1(65) = -g_temp(5);
                
                qp_torque_control.UpdateMinProblem(H1, g1);
                qp_torque_control.UpdateSubjectToAx(A1, lbA1, ubA1);
                qp_torque_control.UpdateSubjectToX(lb1, ub1);
                solved = qp_torque_control.SolveQPoases(100, qp_result);
                if (solved == true)
                {
                    tau_ = ((M_ * (qdd_pinocchio_desired1_  + qp_result.segment<MODEL_DOF_VIRTUAL>(12)) + nle - (RFj.transpose() * qp_result.head(6) + LFj.transpose() * qp_result.segment<6>(6))).transpose());
                }
                control_time = control_time + 1;
            }
            else if(contactMode == 3)//|| mpc_cycle == 49)
            {
                double lx, ly, mu;
                ly = 0.048;
                lx = 0.11;
                mu = 0.8;

                if(zmpx > model_data2.oMf[RFcframe_id].translation()(0) + lx)
                    zmpx  = model_data2.oMf[RFcframe_id].translation()(0) + lx;
                if(zmpx < model_data2.oMf[RFcframe_id].translation()(0) - lx)
                    zmpx  = model_data2.oMf[RFcframe_id].translation()(0) - lx;

                if(zmpy < model_data2.oMf[RFcframe_id].translation()(1) - ly)
                    zmpy = model_data2.oMf[RFcframe_id].translation()(1) - ly;
                else if(zmpy > model_data2.oMf[RFcframe_id].translation()(1) + ly)
                    zmpy = model_data2.oMf[RFcframe_id].translation()(1) + ly;

                zmp_bx(0) =  model_data2.oMf[RFcframe_id].translation()(0) + lx;
                zmp_bx(1) =  model_data2.oMf[RFcframe_id].translation()(0) - lx;

                ly = 0.05;
                lx = 0.13;
                
                MatrixXd J1, H1, A1;
                VectorXd X1, g1, lb1, ub1, lbA1, ubA1;
                
                X1.setZero(constraint_size1);
                J1.setZero(constraint_size1, variable_size1);
                H1.setZero(variable_size1, variable_size1);
                A1.setZero(constraint_size1, variable_size1);
                g1.setZero(variable_size1);
                lbA1.setZero(constraint_size1);
                ubA1.setZero(constraint_size1);
                lb1.setZero(variable_size1);
                ub1.setZero(variable_size1);
            
                M_ = model_data2.M;//= rd_.A_;
                nle = model_data2.nle;

                H1.setIdentity();
                g1.setZero();
                H1.block(12,12,MODEL_DOF_VIRTUAL,MODEL_DOF_VIRTUAL) = 10000 *H1.block(12,12,MODEL_DOF_VIRTUAL,MODEL_DOF_VIRTUAL);
                lb1.setConstant(variable_size1, -100000);
                ub1.setConstant(variable_size1, 100000);

                
                H1(2,2) = 5;
                g1(2) = - com_alpha * nle(2) * 5;
                lb1(2) = 0.0;
                lb1(8) = 0.0;
                lb1(15) = 0.0;
                lb1(16) = 0.0;
                ub1(15) = 0.0;
                ub1(16) = 0.0;

                H1.block(15,15,3,3) = 100 * H1.block(15,15,3,3);
            
                qdd_pinocchio_desired1_ = qdd_pinocchio_desired1;
            
                double weight_resi = 0.0;
                G_temp.setZero();
                g_temp.setZero();
                G_temp.block(0,0,6,MODEL_DOF_VIRTUAL) = RFj;
                g_temp.head(6) <<  RFdj * q_dot_virtual_lpf_ + RFj * qdd_pinocchio_desired1_;
                g1.tail(MODEL_DOF_VIRTUAL) = g1.tail(MODEL_DOF_VIRTUAL) + weight_resi * G_temp.transpose() * g_temp;
                H1.block(12, 12, MODEL_DOF_VIRTUAL, MODEL_DOF_VIRTUAL) += H1.block(12, 12, MODEL_DOF_VIRTUAL, MODEL_DOF_VIRTUAL) + weight_resi * G_temp.transpose() * G_temp;
        
                lbA1.head(6) = (nle + M_ * qdd_pinocchio_desired1_).head(6);
                ubA1.head(6) = (nle + M_ * qdd_pinocchio_desired1_).head(6);

                A1.block(0,0,6,6) = RFj.transpose().block(0,0,6,6);
                A1.block(0,12,6, MODEL_DOF_VIRTUAL) = -M_.block(0,0,6,MODEL_DOF_VIRTUAL);

                A1.block(6,0,1,6)(0,2) = ly;
                A1.block(6,0,1,6)(0,3) = -1;
                lbA1(6) = 0.0;
                ubA1(6) = 100000.0;
                A1.block(7,0,1,6)(0,2) = ly;
                A1.block(7,0,1,6)(0,3) = 1;
                lbA1(7) = 0.0;
                ubA1(7) = 100000.0;
                A1.block(8,0,1,6)(0,2) = lx;
                A1.block(8,0,1,6)(0,4) = -1;
                lbA1(8) = 0.0;
                ubA1(8) = 100000.0;
                A1.block(9,0,1,6)(0,2) = lx;
                A1.block(9,0,1,6)(0,4) = 1;
                lbA1(9) = 0.0;
                ubA1(9) = 100000.0;

                A1.block(14,0,1,6)(0,2) = mu;
                A1.block(24,0,1,6)(0,0) = -1;
                lbA1(14) = 0.0;
                ubA1(14) = 100000.0;
                A1.block(15,0,1,6)(0,2) = mu;
                A1.block(15,0,1,6)(0,0) = 1;
                lbA1(15) = 0.0;
                ubA1(15) = 100000.0;
                A1.block(16,0,1,6)(0,2) = mu;
                A1.block(16,0,1,6)(0,1) = -1;
                lbA1(16) = 0.0;
                ubA1(16) = 100000.0;
                A1.block(17,0,1,6)(0,2) = mu;
                A1.block(17,0,1,6)(0,1) = 1;
                lbA1(17) = 0.0;
                ubA1(17) = 100000.0;
        
                A1.block(22,0,1,6)(0,2) = (model_data2.oMf[RFcframe_id].translation()(1) - zmpy);
                A1.block(22,0,1,6)(0,3) = 1;

                lbA1(22) = 0.0;
                ubA1(22) = 0.0;
            
                A1.block(23,0,1,6)(0,2) = (model_data2.oMf[RFcframe_id].translation()(0) - zmpx);
                A1.block(23,0,1,6)(0,4) = -1;
    
                lbA1(23) = 0.0;
                ubA1(23) = 0.0;
            
                A1.block(24,18,MODEL_DOF_VIRTUAL-6, MODEL_DOF_VIRTUAL-6) = M_.block(6,6,MODEL_DOF_VIRTUAL-6, MODEL_DOF_VIRTUAL-6);
                A1.block(24,0,MODEL_DOF_VIRTUAL-6,6) = -1 * RFj.transpose().block(6,0,MODEL_DOF_VIRTUAL-6,6);
        
                Eigen::VectorVQd Mqddot;
                Mqddot = M_ * qdd_pinocchio_desired1_;

                lbA1(24) = -333-nle(6)-Mqddot(6);
                ubA1(24) = 333-nle(6)-Mqddot(6);
                lbA1(25) = -232-nle(7)-Mqddot(7);
                ubA1(25) = 232-nle(7)-Mqddot(7);
                lbA1(26) = -263-nle(8)-Mqddot(8);
                ubA1(26) = 263-nle(8)-Mqddot(8);
                lbA1(27) = -289-nle(9)-Mqddot(9);
                ubA1(27) = 289-nle(9)-Mqddot(9);
                lbA1(28) = -222-nle(10)-Mqddot(10);
                ubA1(28) = 222-nle(10)-Mqddot(10);
                lbA1(29) = -166-nle(11)-Mqddot(11);
                ubA1(29) = 166-nle(11)-Mqddot(11);

                lbA1(30) = -333-nle(12)-Mqddot(12);
                ubA1(30) = 333-nle(12)-Mqddot(12);
                lbA1(31) = -232-nle(13)-Mqddot(13);
                ubA1(31) = 232-nle(13)-Mqddot(13);
                lbA1(32) = -263-nle(14)-Mqddot(14);
                ubA1(32) = 263-nle(14)-Mqddot(14);
                lbA1(33) = -289-nle(15)-Mqddot(15);
                ubA1(33) = 289-nle(15)-Mqddot(15);
                lbA1(34) = -222-nle(16)-Mqddot(16);
                ubA1(34) = 222-nle(16)-Mqddot(16);
                lbA1(35) = -166-nle(17)-Mqddot(17);
                ubA1(35) = 166-nle(17)-Mqddot(17);

                lbA1(36) = -303-nle(18)-Mqddot(18);
                ubA1(36) = 303-nle(18)-Mqddot(18);
                lbA1(37) = -303-nle(19)-Mqddot(19);
                ubA1(37) = 303-nle(19)-Mqddot(19);
                lbA1(38) = -303-nle(20)-Mqddot(20);
                ubA1(38) = 303-nle(20)-Mqddot(20);

                lbA1(39) = -64-nle(21)-Mqddot(21);
                ubA1(39) = 64-nle(21)-Mqddot(21);
                lbA1(40) = -64-nle(22)-Mqddot(22);
                ubA1(40) = 64-nle(22)-Mqddot(22);
                lbA1(41) = -64-nle(23)-Mqddot(23);
                ubA1(41) = 64-nle(23)-Mqddot(23);
                lbA1(42) = -64-nle(24)-Mqddot(24);
                ubA1(42) = 64-nle(24)-Mqddot(24);
                lbA1(43) = -23-nle(25)-Mqddot(25);
                ubA1(43) = 23-nle(25)-Mqddot(25);
                lbA1(44) = -23-nle(26)-Mqddot(26);
                ubA1(44) = 23-nle(26)-Mqddot(26);
                lbA1(45) = -10-nle(27)-Mqddot(27);
                ubA1(45) = 10-nle(27)-Mqddot(27);
                lbA1(46) = -10-nle(28)-Mqddot(28);
                ubA1(46) = 10-nle(28)-Mqddot(28);

                lbA1(47) = -10-nle(29)-Mqddot(29);
                ubA1(47) = 10-nle(29)-Mqddot(29);
                lbA1(48) = -10-nle(30)-Mqddot(30);
                ubA1(48) = 10-nle(30)-Mqddot(30);

                lbA1(49) = -64-nle(31)-Mqddot(31);
                ubA1(49) = 64-nle(31)-Mqddot(31);
                lbA1(50) = -64-nle(32)-Mqddot(32);
                ubA1(50) = 64-nle(32)-Mqddot(32);
                lbA1(51) = -64-nle(33)-Mqddot(33);
                ubA1(51) = 64-nle(33)-Mqddot(33);
                lbA1(52) = -64-nle(34)-Mqddot(34);
                ubA1(52) = 64-nle(34)-Mqddot(34);
                lbA1(53) = -23-nle(35)-Mqddot(35);
                ubA1(53) = 23-nle(35)-Mqddot(35);
                lbA1(54) = -23-nle(36)-Mqddot(36);
                ubA1(54) = 23-nle(36)-Mqddot(36);
                lbA1(55) = -10-nle(37)-Mqddot(37);
                ubA1(55) = 10-nle(37)-Mqddot(37);
                lbA1(56) = -10-nle(38)-Mqddot(38);
                ubA1(56) = 10-nle(38)-Mqddot(38);
        
                A1(57, 2) = 1.0;
                lbA1(57) = 800.0;
                ubA1(57) = 1000.0;

                A1.block(58,0,1,6)(0,2) = mu;
                A1.block(58,0,1,6)(0,5) = 1;
                lbA1(58) = 0.0;
                ubA1(58) = 100000.0;
                A1.block(59,0,1,6)(0,2) = mu;
                A1.block(59,0,1,6)(0,5) = -1;
                lbA1(59) = 0.0;
                ubA1(59) = 100000.0;

                A1.block(60,12,6,MODEL_DOF_VIRTUAL)  = G_temp;

                lbA1(60) = -g_temp(0);  
                lbA1(61) = -g_temp(1);  
                lbA1(62) = -g_temp(2);  
                lbA1(63) = -g_temp(3);  
                lbA1(64) = -g_temp(4);  
                lbA1(65) = -g_temp(5); 
                ubA1(60) = -g_temp(0);  
                ubA1(61) = -g_temp(1);  
                ubA1(62) = -g_temp(2);  
                ubA1(63) = -g_temp(3);  
                ubA1(64) = -g_temp(4);  
                ubA1(65) = -g_temp(5); 

                qp_torque_control.UpdateMinProblem(H1, g1);
                qp_torque_control.UpdateSubjectToAx(A1, lbA1, ubA1);
                qp_torque_control.UpdateSubjectToX(lb1, ub1);
                solved = qp_torque_control.SolveQPoases(100, qp_result);
            
                if (solved == true)
                {
                    /*if(control_time >= 1)
                    {
                       
                        tau_ = ((M_ * (qdd_pinocchio_desired1_ + qp_result.segment<MODEL_DOF_VIRTUAL>(12)) + nle - (RFj.transpose() * qp_result.head(6) + LFj.transpose() * qp_result.segment<6>(6))).transpose());
                        qp_result_prev = qp_result;
                        
                    }
                    else
                    {
                        qp_result_prev = qp_result;
                    }*/
                    tau_ = ((M_ * (qdd_pinocchio_desired1_ + qp_result.segment<MODEL_DOF_VIRTUAL>(12)) + nle - (RFj.transpose() * qp_result.head(6) + LFj.transpose() * qp_result.segment<6>(6))).transpose());
                        
                }
                control_time = control_time + 1;
            }
        }
        if(walk_start1 == true)
        {
            if(mpc_cycle < controlwalk_time)// && mpc_cycle <= 83)
            {    
                for (int i = 0; i < MODEL_DOF_VIRTUAL-6; i++)
                {  
                    rd_.torque_desired[i] = tau_[i+6] + 1.0 * (rd_.pos_kp_v[i] * (q_pinocchio_desired1[i+7] - rd_.q_[i]) + rd_.pos_kv_v[i] * (qd_pinocchio_desired1[i+6]- rd_.q_dot_[i])); //qd_pinocchio_desired1_prev[i+6]
                }
                double_temp = 3;
            }
            else
            {  
                for (int i = 0; i < MODEL_DOF_VIRTUAL-6; i++)
                {  
                    rd_.torque_desired[i] = 1.0 * (rd_.pos_kp_v[i] * (q_pinocchio_desired1[i+7] - rd_.q_[i]) + rd_.pos_kv_v[i] * (- rd_.q_dot_[i]));
                }
                double_temp = 4;
            }
        }
        else
        {
            WBC::SetContact(rd_, 1, 1);
            rd_.torque_desired_walk = WBC::ContactForceRedistributionTorque(rd_, WBC::GravityCompensationTorque(rd_));
            tau_.segment<MODEL_DOF_VIRTUAL-6>(6) = rd_.torque_desired_walk;
            //if (walking_tick == 0)
            //    std::cout << "tau" << " " << tau_.transpose() << std::endl;
            if(first_control == false)
            {
                rd_.q_desired = rd_.q_;
            }
            for (int i = 0; i < MODEL_DOF_VIRTUAL-6; i++)
            {  
                rd_.torque_desired[i] = 1.0 * (rd_.pos_kp_v[i] * (rd_.q_desired(i) - rd_.q_[i]) + rd_.pos_kv_v[i] * (- rd_.q_dot_[i])) + tau_[i+6];
            }
            double_temp = 2;
        }

    //file[0] <<mpc_cycle << " " << walking_tick << " "<<solved<< " "<< zmp_bx(0) << " " << zmp_bx(1) << " "<< zmpx << " " << zmpy << " "  << ZMP_FT_law(0) << "  " << ZMP_FT_law(1)<< " "  <<  comt_[0] << " " << comt_[1]<< " "<<rd_.link_[COM_id].xpos(0)<< " "<<rd_.link_[COM_id].xpos(1)<<  " " << comdt_(0)<<  " " << comdt_(1)<< " " <<rd_.link_[COM_id].v(0)<< " "<<rd_.link_[COM_id].v(1)<<  " "  << q_pinocchio_desired1[2+7] << " " << rd_.q_[2] << " " << q_pinocchio_desired1[3+7] << " " << rd_.q_[3] << " " << q_pinocchio_desired1[4+7] << " " << rd_.q_[4] << std::endl;
    //file[0]<<mpc_cycle << " " << walking_tick << " "<<com_z_init<< " " << rd_.link_[COM_id].xpos(2)<< " " <<foot_x_int<< " " << foot_y_int << " " << rd_.link_[Left_Foot].xipos(0) << " " << rd_.link_[Left_Foot].xipos(1) << " " << rd_.link_[Left_Foot].xipos(2)<< " " << lfoot_mpc(0) << " " << lfoot_mpc(1)<< " " << lfoot_mpc(2) + foot_temp(1) - 0.159<< " " << rd_.link_[Right_Foot].xipos(0) << " " << rd_.link_[Right_Foot].xipos(1) << " " << rd_.link_[Right_Foot].xipos(2)<< " " << rfoot_mpc(0) << " " << rfoot_mpc(1)<< " " << rfoot_mpc(2) + foot_temp(1) - 0.159<< " " << rfootz << " " <<lfootz <<  " " << angd_(0)  <<  " " << angd_(1)<<  " " << model_data2.hg.angular()[0]  <<  " " << model_data2.hg.angular()[1]<< " "  << rd_.q_desired(5)<< " " << rd_.torque_desired[5] << " " << tau_[5]<<std::endl;
    /*file[1] <<mpc_cycle << " " << walking_tick << " " << q_pinocchio_desired1[0+7] << " " << rd_.q_[0] << " " << q_pinocchio_desired1[1+7] << " " << rd_.q_[1] << " " << q_pinocchio_desired1[2+7] << " " << rd_.q_[2] << " " << q_pinocchio_desired1[3+7] << " " << rd_.q_[3] << " " << q_pinocchio_desired1[4+7] << " " << rd_.q_[4] << " " << q_pinocchio_desired1[5+7] << " " << rd_.q_[5] << " " << rd_.torque_desired[0] << " " << tau_[0+6] + 1.0 * (rd_.pos_kp_v[0] * (q_pinocchio_desired1[0+7] - rd_.q_[0]) + rd_.pos_kv_v[0] * (qd_pinocchio_desired1[0+6]- rd_.q_dot_[0]))<< " " << tau_[1+6] + 1.0 * (rd_.pos_kp_v[1] * (q_pinocchio_desired1[1+7] - rd_.q_[1]) + rd_.pos_kv_v[1] * (qd_pinocchio_desired1[1+6]- rd_.q_dot_[1])) << " " << tau_[2+6] + 1.0 * (rd_.pos_kp_v[2] * (q_pinocchio_desired1[2+7] - rd_.q_[2]) + rd_.pos_kv_v[2] * (qd_pinocchio_desired1[2+6]- rd_.q_dot_[2]))
    << " " << tau_[3+6] + 1.0 * (rd_.pos_kp_v[3] * (q_pinocchio_desired1[3+7] - rd_.q_[3]) + rd_.pos_kv_v[3] * (qd_pinocchio_desired1[3+6]- rd_.q_dot_[3]))
    << " " << tau_[4+6] + 1.0 * (rd_.pos_kp_v[4] * (q_pinocchio_desired1[4+7] - rd_.q_[4]) + rd_.pos_kv_v[4] * (qd_pinocchio_desired1[4+6]- rd_.q_dot_[4]))
    << " " << rd_.q_desired(9)<< " " << rd_.torque_desired[5] << std::endl;*/
    }
    else
    {
        WBC::SetContact(rd_, 1, 1);
        rd_.torque_desired_walk = WBC::ContactForceRedistributionTorque(rd_, WBC::GravityCompensationTorque(rd_));
        tau_.segment<MODEL_DOF_VIRTUAL-6>(6) = rd_.torque_desired_walk;
        if(first_control == false)
        {
            rd_.q_desired = rd_.q_;
        }
        for (int i = 0; i < MODEL_DOF_VIRTUAL-6; i++)
        {  
            rd_.torque_desired[i] = 1.0 * (rd_.pos_kp_v[i] * (rd_.q_desired(i) - rd_.q_[i]) + rd_.pos_kv_v[i] * (- rd_.q_dot_[i])) + tau_[i+6];
        }

        double_temp = 1;
    }        

    if(walk_start == true && mpc_cycle < controlwalk_time && mpc_cycle >= 1)
    {
        //file[0] <<double_temp  << " " << mpc_cycle << " " << walking_tick << " " << solved << " " << "0 ";//<<lfoot_mpc(2) -0.159 + foot_temp(1) << " " << rd_.link_[Left_Foot].xipos(2) << " " << rd_.link_[Left_Foot].xpos(0)<< " " << rd_.link_[Right_Foot].xpos(0) << " "<<zmpy << " " << zmpx<< " " << ZMP_FT_law(1) << " " <<ZMP_FT_law(0) << " " << com_alpha<< " " << com_mpc[0] << " " <<com_mpc[1] << " " << rd_.link_[COM_id].xpos(0)<< " " << rd_.link_[COM_id].xpos(1);
        file[0] << mpc_cycle << " " << walking_tick << " " <<contactMode << " "<<(RFc_float_current.translation()(1)  - zmpy) << " " << (LFc_float_current.translation()(1)  - zmpy) << " " << comt_[0] << " "<< COM_float_current.translation()(0)<< " "<< comt_[1] << " " << COM_float_current.translation()(1)<<" " <<model_data2.oMf[RFcframe_id].translation()(0) << " " << model_data2.oMf[RFcframe_id].translation()(1)  << " " << zmpx << " " << ZMP_FT_law(0) << " " << zmpy << " " << ZMP_FT_law(1) << " " << comdt_(1) << " " << rfoot_mpc(2)<<" " << lfoot_mpc(2)<<  " " << 0.159 + RF_float_current.translation()(2) - foot_temp(1)<<  " " << 0.159 + LF_float_current.translation()(2) - foot_temp(1) <<  " " << rd_.torque_desired[4] <<  " "  <<  " " << rd_.torque_desired[5] <<  " " << pelv_ori_c(0) << " " <<pelv_ori_c(1)<< std::endl;
        
        //file[0] << mpc_cycle << " " << comdt_(0) << " " <<COMv_float_current.translation()(0)<< " " << comdt_(1) << " " <<COMv_float_current.translation()(1) << " " << comt_(0) << " " <<COM_float_current.translation()(0) << " " << comt_(1) << " " <<COM_float_current.translation()(1) << " " << q_pinocchio_desired(0) << " " << q_pinocchio_desired(1) << std::endl;

        //q_pinocchio_desired1(0) << " " << rd_.q_virtual_(0)<< " " << q_pinocchio_desired1(1) << " " << rd_.q_virtual_(1);
        
        //qp_result(3) << " " << qp_result(9)  << " " <<-1* rd_.LF_CF_FT(3) << " " <<-1* rd_.RF_CF_FT(3)<< " " << rd_.LF_CF_FT(3) << " " << rd_.RF_CF_FT(3) << " " << zmpx << " " << zmpy << " " << ZMP_FT_law(0)<< " " << ZMP_FT_law(1)  << " "<< pelv_ori_c(0) << " " << pelv_ori_c(1);
        // " "<< zmp_x_int << " " << zmp_y_int<< " " << (model_data2.oMf[RFcframe_id].translation()(1) - zmpy) << " " << (model_data2.oMf[LFcframe_id].translation()(1) - zmpy) << " "  << model_data2.oMf[RFcframe_id].translation()(1) << " " << model_data2.oMf[LFcframe_id].translation()(1) << " " << LF_matrix(mpc_cycle,1) << " " << RF_matrix(mpc_cycle,1) << " "<< com_alpha * nle(2)<< " " << (1-com_alpha) * nle(2) << " " << zmpx  << " " << ZMPx_test << " "<< zmpy<< " " << ZMPy_test << " "<< ZMP_FT_law(0) << " " << ZMP_FT_law(1) << " "  << ZMP_r(1) << " " <<ZMP_l(1)<< " " << rd_.RF_CF_FT(2) << " " << rd_.LF_CF_FT(2) << " "<< com_mpc[0] << " " <<com_mpc[1] << " " << rd_.link_[COM_id].xpos(0)<< " " << rd_.link_[COM_id].xpos(1) << " " << COM_float_current.translation()(0) << " " << COM_float_current.translation()(1) << " " <<  rd_.link_[Left_Foot].xpos(1)<< " " << rd_.link_[Right_Foot].xpos(1) << " " << RF_float_current.translation()(1)<< " " << LF_float_current.translation()(1)<< " " << RFc_float_current.translation()(1)<< " " << LFc_float_current.translation()(1) <<  " " << com_x_int << " " << com_y_int << " ";
        //file[1] << q_pinocchio_desired1(9) << " " << rd_.q_(2) << " "<< q_pinocchio_desired1(10) << " " << rd_.q_(3) << " "<< q_pinocchio_desired1(11) << " " << rd_.q_(4) << " "<< q_pinocchio_desired1(15) << " " << rd_.q_(8) << " "<< q_pinocchio_desired1(16) << " " << rd_.q_(9) << " "<< q_pinocchio_desired1(17) << " "<< rd_.q_(10) ;
        //file[1] << virtual_temp(0) << " " << virtual_temp1(0) << " " << zmp_mpcx << " "<< virtual_temp(1) << " " << virtual_temp1(1) << " " << desired_val.m_shared_memory[43] <<  " " << rd_.q_(13) << " " << rd_.q_(14) << " " << desired_val.m_shared_memory[19] << " " << desired_val.m_shared_memory[20] << " " << pelv_ori_c(0) <<  "  " << pelv_ori_c(1) << std::endl;// << rfoot_ori_c(0) << " " << rfoot_ori_c(1) << " " << rfoot_ori_c(2) << " " << lfoot_ori_c(0) << " " << lfoot_ori_c(1) << " " << lfoot_ori_c(2);
        
        //for (int i = 0; i < 6; i++) //tau_[i+6] << " " << rd_.torque_desired[i] << " "
        //    file[0] << qdd_pinocchio_desired1_(i) << " ";//qp_result[i+12] << " ";
        //    file[0]  << q_pinocchio_desired1[i] << " " << rd_.q_virtual_(i) << " ";
        
        
        // << (RFj.transpose() * qp_result.head(6) + LFj.transpose() * qp_result.segment<6>(6))[i] << " "; // -tau_[i+6] + rd_.torque_desired[i] << " " << rd_.torque_desired[i] << " " <<tau_[i+6] << " ";
        //file[0] << std::endl;
    }
    
    
    /*file[0] <<mpc_cycle << " " <<walking_tick << " "<< tau_[5+6] << " "  ;
    for(int i = 0; i < MODEL_DOF_VIRTUAL+1; i++)
        file[0] << q_pinocchio_desired1[i] << " " ;
    for(int i = 0; i <MODEL_DOF_VIRTUAL; i++ )
        file[0] << qdd_pinocchio_desired1_[i] << " ";
    for(int i = 0; i <MODEL_DOF_VIRTUAL; i++ )
        file[0] << rd_.q_dot_virtual_[i] << " ";
    file[0] << std::endl;*/
}

void CustomController::computeFast()
{ 
    if (time_tick == false)
    {
        startTime = std::chrono::system_clock::now();
        time_tick = true;
    }     

    if (std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count() >= 500)
    {
        time_tick_next = true;
        //std::cout << "dd " << mpc_cycle << " " << walking_tick << " " << std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count()  << std::endl; 
        startTime = std::chrono::system_clock::now();
    }

    mpc_cycle_int = (mpc_cycle-50) / 50;
    mpc_cycle_int1 = (mpc_cycle - 50) % 50;

    rd_.tc_.walking_enable = 1.0;
    if(time_tick_next == true)
    {     
        time_tick_next = false;

        cc_mutex2.lock();
        //PELV_float_current.linear() = InitRPYM.linear() * rd_.link_[Pelvis].rotm;
        
        //RF_float_current.translation() = rd_.link_[Right_Foot].xpos;//DyrosMath::multiplyIsometry3dVector3d(InitRPYM, rd_.link_[Right_Foot].xpos);
        //LF_float_current.translation() = rd_.link_[Left_Foot].xpos;//DyrosMath::multiplyIsometry3dVector3d(InitRPYM, rd_.link_[Left_Foot].xpos);
        
        //RF_float_current.linear() = InitRPYM.linear() * rd_.link_[Right_Foot].rotm;
        //LF_float_current.linear() = InitRPYM.linear() * rd_.link_[Left_Foot].rotm;
        
        //COM_float_current.translation() = DyrosMath::multiplyIsometry3dVector3d(InitRPYM, rd_.link_[COM_id].xpos);
        //COMv_float_current.translation() = DyrosMath::multiplyIsometry3dVector3d(InitRPYM, rd_.link_[COM_id].v);

        //J_RFF = rd_.link_[Right_Foot].Jac();//.block(0,0,6,18);
        //J_LFF = rd_.link_[Left_Foot].Jac();//.block(0,0,6,18);


        PELV_float_current.linear() = InitRPYM.linear() * rd_.link_[Pelvis].rotm;
        
        RF_float_current.translation() = DyrosMath::multiplyIsometry3dVector3d(InitRPYM, rd_.link_[Right_Foot].xpos);
        LF_float_current.translation() = DyrosMath::multiplyIsometry3dVector3d(InitRPYM, rd_.link_[Left_Foot].xpos);
        
        RF_float_current.linear() = InitRPYM.linear() * rd_.link_[Right_Foot].rotm;
        LF_float_current.linear() = InitRPYM.linear() * rd_.link_[Left_Foot].rotm;
        
        COM_float_current.translation() = DyrosMath::multiplyIsometry3dVector3d(InitRPYM, rd_.link_[COM_id].xpos);
        COMv_float_current.translation() = DyrosMath::multiplyIsometry3dVector3d(InitRPYM, rd_.link_[COM_id].v);

        J_RFF = InitRPYM2 * rd_.link_[Right_Foot].Jac();//.block(0,0,6,18);
        J_LFF = InitRPYM2 * rd_.link_[Left_Foot].Jac();//.block(0,0,6,18);

        /*
        std::cout << " LF_float_current.translation() "  << std::endl;
        std::cout << LF_float_current.translation().transpose() << std::endl;
        std::cout << " RF_float_current.translation() "  << std::endl;
        std::cout << RF_float_current.translation().transpose() << std::endl;
        std::cout << " PELV_float_current.translation() "  << std::endl;
        std::cout << rd_.link_[Pelvis].xpos(0) << " " << rd_.link_[Pelvis].xpos(1) << " " << std::endl;
        std::cout << rd_.q_virtual_(0) << " "<< rd_.q_virtual_(1) << std::endl;
        */
        
        //RF_float_current.translation() = rd_.link_[Right_Foot].xipos;
        //LF_float_current.translation() = rd_.link_[Left_Foot].xipos;
        //RF_float_current.linear() = rd_.link_[Right_Foot].rotm;
        //LF_float_current.linear() = rd_.link_[Left_Foot].rotm;

        
        //COM_float_current.translation() = rd_.link_[COM_id].xpos;//DyrosMath::multiplyIsometry3dVector3d(InitRPYM, rd_.link_[COM_id].xpos);
        //COMv_float_current.translation() = rd_.link_[COM_id].v;//DyrosMath::multiplyIsometry3dVector3d(InitRPYM, rd_.link_[COM_id].v);

        //J_RFF = InitRPYM2 * rd_.link_[Right_Foot].Jac();//.block(0,0,6,18);
        //J_LFF = InitRPYM2 * rd_.link_[Left_Foot].Jac();//.block(0,0,6,18);
        
        cc_mutex2.unlock();

        if (rd_.tc_.mode == 6)
        {   
            if (wlk_on == false)
            {
                wk_Hz = 2000;
                wk_dt = 1 / wk_Hz;
                controlwalk_time = 190;//217;//360;

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

                    /*setWalkingParameter();
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
                    cc_mutex.unlock();*/
                
                    walking_tick = 0;

                    q_init = rd_.q_;
                    q_desired.resize(33);
                    q_desired.setZero();
                    q_desired.head(33) << 0, 0, -0.55, 1.26, -0.71, 0, 0, 0, -0.55, 1.26,
                    -0.71, 0, 0, 0, 
                    0, 0.2, 0.6, 1.5, -1.47, -1,
                    0, -1, 0, 0, 0, -0.2, -0.6, -1.5, 1.47, 1, 
                    0, 1, 0;
                }
                
                if(walking_tick <= 6000)
                {
                    if(walking_tick <= 6000)//6000)
                    {
                        for (int i = 0; i < 33; i++)
                        {
                            rd_.q_desired(i) = DyrosMath::cubic(walking_tick, 0, 5500, q_init(i), q_desired(i), 0, 0);
                        }
                    }
                    else
                    {
                        for (int i = 0; i < 33; i++)
                        {
                            rd_.q_desired(i) =  rd_.q_desired(i);//q_desired(i);
                        }
                    }
                    for (int i = 33; i < MODEL_DOF; i++)
                    {
                        rd_.q_desired(i) = rd_.q_desired(i);//q_desired(i);
                    }

                    walking_tick = walking_tick + 1;
                    first_control = true;
                    /*std::cout << "q" << std::endl;
                    std::cout << rd_.q_desired.transpose() << std::endl;
                    std::cout << "q virtu" << std::endl;
                    std::cout << rd_.q_virtual_.transpose() << std::endl;*/
                }
                else
                {  
                    walking_tick_stop = true;
                    walking_tick_stop1 = true;
                    walking_tick = 0;
                    
                    std::cout << "Start" << std::endl;
                    walk_start = true;
                    wlk_on = true;
                    stateestimation = true;
                }
            }
        }

        if(rd_.tc_.mode == 9 && print_cout == false)
        {
            if(pelv_frame == true)
                InitRPYM.linear()= rd_.link_[Pelvis].rotm;
            else
                InitRPYM.linear().setIdentity();

            InitRPYM.translation()(0) = rd_.link_[Pelvis].xpos(0);
            InitRPYM.translation()(1) = rd_.link_[Pelvis].xpos(1);
            InitRPYM.translation()(2) = rd_.link_[Pelvis].xpos(2);
            InitRPYM = DyrosMath::inverseIsometry3d(InitRPYM);

            initrpy(0) =  -1 * rd_.link_[Pelvis].xpos(0);
            initrpy(1) =  -1 * rd_.link_[Pelvis].xpos(1);
            initrpy(2) =  -1 * rd_.link_[Pelvis].xpos(2);

            InitRPYM2.block(0,0,3,3) = InitRPYM.linear();
            InitRPYM2.block(3,3,3,3) = InitRPYM.linear();
            
            if(print_cout == false)
            {
                std::cout << "rd_ " << std::endl;
                //tf2::Quaternion q(rd_.q_virtual(3), rd_.q_virtual(4), rd_.q_virtual(5), rd_.q_virtual_(MODEL_DOF_VIRTUAL));
                //tf2::Quaternion q(7.91422e-03, -2.88212e-03,  1.94139e-04, 9.99965e-01);
                
                std::cout << " " <<rd_.link_[Pelvis].rotm << std::endl;; 
                std::cout << rd_.q_virtual_(0) << " " << rd_.q_virtual_(1)<< " " <<rd_.q_virtual_(3) << " " << rd_.q_virtual_(4)<< " " << rd_.q_virtual_(5)<< " " << rd_.q_virtual_(MODEL_DOF_VIRTUAL) << std::endl;

                sleep(1);
                std::cout << "pelv" << std::endl;
                std::cout << rd_.link_[Pelvis].xpos.transpose() << std::endl;
                std::cout << rd_.link_[Pelvis].xipos.transpose() << std::endl;
                std::cout << InitRPYM(3,3) << std::endl;
                std::cout << "Foot" << std::endl;

                std::cout << "cc" << std::endl;
                std::cout << COM_float_current.translation()(0) << " " << COM_float_current.translation()(1) << std::endl;
                std::cout << rd_.link_[COM_id].xpos(0) << " " << rd_.link_[COM_id].xpos(1) << std::endl;
                std::cout << InitRPYM.translation().transpose() << std::endl;
                std::cout << rd_.q_virtual_(0) << " "<< rd_.q_virtual_(1) << " "<< rd_.q_virtual_(2) << " "<< rd_.q_virtual_(3) << " "<< rd_.q_virtual_(4) << " "<< rd_.q_virtual_(5) << " "<< rd_.q_virtual_(MODEL_DOF_VIRTUAL) << std::endl;
                print_cout = true;
            }
        }

        // 0.03, 0.0378015
        if(rd_.tc_.mode == 8)
        {
            if(mpc_cycle < controlwalk_time && wlk_on == true && stateestimation == true)
            {
                if (walking_tick == 0)
                {
                    qd_pinocchio_.setZero();
                }  
                if(walking_tick == 1)
                {
                    if(q_desired_bool == false)
                        q_pinocchio_desired.head(19) = q_pinocchio.head(19);
                }
                if((walking_tick_stop == true && walking_tick == 0))
                {  
                    if (walking_tick == 0)
                    {   
                        if(mpc_cycle == 0)
                        {
                            foot_x_int = RF_matrix(mpc_cycle,0) - RF_float_current.translation()(0);
                            foot_y_int = RF_matrix(mpc_cycle,1) - RF_float_current.translation()(1);
                            com_x_int = com_matrix_matrix(mpc_cycle,0) - COM_float_current.translation()(0);
                            com_y_int = com_matrix_matrix(mpc_cycle,1) - COM_float_current.translation()(1);
                            com_z_init = COM_float_current.translation()(2);

                            zmp_x_int = (RF_matrix(mpc_cycle,0) - RF_float_current.translation()(0));//com_x_int;
                            zmp_y_int = -(RF_float_current.translation()(1)+LF_float_current.translation()(1))/2;//com_y_int;
                            
                            if(contactMode == 1)
                            {
                                foot_temp(0) = RF_float_current.translation()(2);
                                foot_temp(1) = LF_float_current.translation()(2);
                            }
                        }
                        walking_tick_stop = false;

                        if(mpc_cycle == 0)
                        {
                            angm_prev.setZero();
                            comprev.setZero();
                        }
                        else 
                        {
                            angm_prev = angm;
                            comprev = comd;
                        }
                        comd[0] = comd_matrix_matrix(mpc_cycle,0);
                        comd[1] = comd_matrix_matrix(mpc_cycle,1);
                        comd[2] = 0.0;

                        angm[0] = angm_ref_matrix(mpc_cycle,0);
                        angm[1] = angm_ref_matrix(mpc_cycle,1);
                        if(mpc_cycle == 0)
                        {
                            ZMPx_prev = zmp_matrix(mpc_cycle,0) - zmp_x_int;
                            ZMPy_prev = zmp_matrix(mpc_cycle,1) - zmp_y_int;
                        }
                        else
                        {
                            ZMPx_prev = zmp_mpcx;
                            ZMPy_prev = zmp_mpcy;
                        }  
                        
                        zmp_mpcx = zmp_matrix(mpc_cycle,0) - zmp_x_int;
                        zmp_mpcy = zmp_matrix(mpc_cycle,1) - zmp_y_int;

                        if(mpc_cycle == 0)
                        {
                            com_mpc_prev[0] = com_matrix_matrix(mpc_cycle,0) - com_x_int;
                            com_mpc_prev[1] = com_matrix_matrix(mpc_cycle,1) - com_y_int;
                        }
                        else
                        {
                            com_mpc_prev = com_mpc;
                        }  
                            
                        com_mpc[0] = com_matrix_matrix(mpc_cycle,0) - com_x_int;
                        com_mpc[1] = com_matrix_matrix(mpc_cycle,1) - com_y_int;

                        
                        if(mpc_cycle == 0)
                        {
                            rfootd.setZero();
                            lfootd.setZero();
                        }
                        else
                        {  
                            std::cout << "mpc " << mpc_cycle << " " << RF_matrix(mpc_cycle,0) << " " << RF_matrix(mpc_cycle-1,0)<< " " <<LF_matrix(mpc_cycle,0) << " " << LF_matrix(mpc_cycle-1,0) << std::endl;
                            rfootd[0] = (RF_matrix(mpc_cycle,0)-RF_matrix(mpc_cycle-1,0))/0.02;
                            rfootd[1] = (RF_matrix(mpc_cycle,1)-RF_matrix(mpc_cycle-1,1))/0.02;
                            rfootd[2] = (RF_matrix(mpc_cycle,2)-RF_matrix(mpc_cycle-1,2))/0.02;

                            lfootd[0] = (LF_matrix(mpc_cycle,0)-LF_matrix(mpc_cycle-1,0))/0.02;
                            lfootd[1] = (LF_matrix(mpc_cycle,1)-LF_matrix(mpc_cycle-1,1))/0.02;
                            lfootd[2] = (LF_matrix(mpc_cycle,2)-LF_matrix(mpc_cycle-1,2))/0.02;
                        }
                    }
                }

                if(mpc_cycle < controlwalk_time)
                {   
                    comdt_(0) = (comd(0) * walking_tick + comprev(0) * (40 -walking_tick))/40;
                    comdt_(1) = (comd(1) * walking_tick + comprev(1) * (40 -walking_tick))/40;

                    comt_(0) = (com_mpc(0) * walking_tick + com_mpc_prev(0) * (40 -walking_tick))/40;
                    comt_(1) = (com_mpc(1) * walking_tick + com_mpc_prev(1) * (40 -walking_tick))/40;

                    if(q_desired_bool == false)
                    {
                        comd_(0) = comdt_(0)+ 30.0 * (comdt_(0) - COMv_float_current.translation()(0)) + 300.0 * (comt_[0] - COM_float_current.translation()(0));
                        comd_(1) = comdt_(1)+ 10.0 * (comdt_(1) - COMv_float_current.translation()(1)) + 100.0 * (comt_[1] - COM_float_current.translation()(1));
                        comd_(2) = 0.0 + 10.0 * (com_z_init - COM_float_current.translation()(2));
                    }
                    else
                    {
                        //comd_(0) = comdt_(0);
                        //comd_(1) = comdt_(1);
                        comd_(0) = comdt_(0)+ 0.0 * (comdt_(0) - COMv_float_current.translation()(0)) + 30.0 * (comt_[0] - COM_float_current.translation()(0));
                        comd_(1) = comdt_(1)+ 3.0 * (comdt_(1) - COMv_float_current.translation()(1)) + 30.0 * (comt_[1] - COM_float_current.translation()(1));
                        comd_(2) = 0.0 + 10.0 * (com_z_init - COM_float_current.translation()(2));
                    }

                    angd_(0) = (angm(0) * walking_tick + angm_prev(0) * (40 -walking_tick))/40;
                    angd_(1) = (angm(1) * walking_tick + angm_prev(1) * (40 -walking_tick))/40;
                    
                    rfoot_ori.setZero();
                    lfoot_ori.setZero();
                    rfoot_mpc.setZero();
                    lfoot_mpc.setZero();

                    rfoot_ori_c = DyrosMath::rot2Euler(RF_float_current.linear());
                    lfoot_ori_c = DyrosMath::rot2Euler(LF_float_current.linear());
                    pelv_ori_c = DyrosMath::rot2Euler(PELV_float_current.linear());

                    rfootd1 = rfootd;
                    lfootd1 = lfootd;
                    double gain_xz, gain_ori;

                    if(q_desired_bool == false)
                    {
                        gain_xz = 60.0; //10
                        gain_ori = 40.00; // 40
                    }
                    else
                    {
                        gain_xz = 0.0;
                        gain_ori = 0.0;
                    }
                    
                    if(mpc_cycle  == 0)
                    {
                        rfoot_mpc(0) = RF_matrix(mpc_cycle,0) - foot_x_int;
                        rfoot_mpc(1) = RF_matrix(mpc_cycle,1) - foot_y_int;
                        rfoot_mpc(2) = RF_matrix(mpc_cycle,2);
                        lfoot_mpc(0) = LF_matrix(mpc_cycle,0) - foot_x_int;
                        lfoot_mpc(1) = LF_matrix(mpc_cycle,1) - foot_y_int;
                        lfoot_mpc(2) = LF_matrix(mpc_cycle,2);
                    
                    }
                    else
                    {
                        rfoot_mpc(0) = (RF_matrix(mpc_cycle,0) * walking_tick + RF_matrix(mpc_cycle-1,0) *(40-walking_tick))/40 - foot_x_int;
                        rfoot_mpc(1) = (RF_matrix(mpc_cycle,1) * walking_tick + RF_matrix(mpc_cycle-1,1) *(40-walking_tick))/40 - foot_y_int;
                        rfoot_mpc(2) = (RF_matrix(mpc_cycle,2) * walking_tick + RF_matrix(mpc_cycle-1,2) *(40-walking_tick))/40;
                        lfoot_mpc(0) = (LF_matrix(mpc_cycle,0) * walking_tick + LF_matrix(mpc_cycle-1,0) *(40-walking_tick))/40 - foot_x_int;
                        lfoot_mpc(1) = (LF_matrix(mpc_cycle,1) * walking_tick + LF_matrix(mpc_cycle-1,1) *(40-walking_tick))/40 - foot_y_int;
                        lfoot_mpc(2) = (LF_matrix(mpc_cycle,2) * walking_tick + LF_matrix(mpc_cycle-1,2) *(40-walking_tick))/40;
                    }

                    lfootz = lfoot_mpc(2);
                    rfootz = rfoot_mpc(2);

                    rfootd1(1) = 0.0;
                    lfootd1(1) = 0.0;
                    
                    if(rfoot_mpc(2)>0.159)
                    {
                        rfoot_ori(0) = gain_ori * (-rfoot_ori_c(0));
                        rfoot_ori(1) = gain_ori * (-rfoot_ori_c(1));
                    
                        lfoot_ori(0) = 2.5*gain_ori * (pelv_ori_c(0));
                        lfoot_ori(1) = 2.5*gain_ori * (pelv_ori_c(1));

                        rfootd1(2) = rfootd1(2)+ gain_xz * (rfoot_mpc(2) -0.159 - RF_float_current.translation()(2) + foot_temp(0));
                        rfootd1(1) = rfootd1(1) + 10.00 * (-0.1025 - RF_float_current.translation()(1) - virtual_temp1(1));
                        rfootd1(0) = rfootd1(0)+ gain_xz * (rfoot_mpc(0) - RF_float_current.translation()(0));
                    }
                    else if(lfoot_mpc(2)>0.159)
                    {
                        lfoot_ori(0) = gain_ori * (-lfoot_ori_c(0));
                        lfoot_ori(1) = gain_ori * (-lfoot_ori_c(1));

                        rfoot_ori(0) = 2.5*gain_ori * (pelv_ori_c(0));
                        rfoot_ori(1) = 2.5*gain_ori * (pelv_ori_c(1));
                        lfootd1(2) = lfootd1(2)+ gain_xz * (lfoot_mpc(2) -0.159 - LF_float_current.translation()(2) + foot_temp(1));
                        lfootd1(1) = lfootd1(1) + 10.00 * (0.1025 - LF_float_current.translation()(1) - virtual_temp1(1));
                        lfootd1(0) = lfootd1(0)+ gain_xz * (lfoot_mpc(0) - LF_float_current.translation()(0));
                    }  
                    else
                    { 
                        lfoot_ori(0) = 0.5*gain_ori * (pelv_ori_c(0));
                        lfoot_ori(1) = 0.5*gain_ori * (pelv_ori_c(1));
                        rfoot_ori(0) = 0.5*gain_ori * (pelv_ori_c(0));
                        rfoot_ori(1) = 0.5*gain_ori * (pelv_ori_c(1));

                        rfoot_mpc(2) = 0.159;
                        lfoot_mpc(2) = 0.159;
                    }

                    if(q_desired_bool == true)
                    {
                        rfootd1(1) = 0.0;
                        lfootd1(1) = 0.0;
                    }

                    //lfoot_ori.setZero();
                    //rfoot_ori.setZero();

                    momentumControl(rd_, comd_, angd_, rfootd1, lfootd1, upperd, rfoot_ori, lfoot_ori);
                    qd_pinocchio.setZero();                   
                    qd_pinocchio.segment<18>(0) = q_dm;
                    
                    qdd_pinocchio_desired1_raw = ((qd_pinocchio - qd_pinocchio_prev)/0.02);
                    
                    
                    for(int i = 0; i <MODEL_DOF_VIRTUAL; i++)
                    {
                        if(abs(qdd_pinocchio_desired1(i) - qdd_pinocchio_desired1_raw(i))>0.8)
                        {
                            break;
                        }
                        if(i == MODEL_DOF_VIRTUAL - 1)
                        {
                            cc_mutex2.lock();
                            qdd_pinocchio_desired1 = qdd_pinocchio_desired1_raw;
                            cc_mutex2.unlock();
                        }
                    }
                    qd_pinocchio_prev = qd_pinocchio;

                    ZMPx_test = (zmp_mpcx * walking_tick + ZMPx_prev *(40-walking_tick))/40;
                    ZMPy_test = (zmp_mpcy * walking_tick + ZMPy_prev *(40-walking_tick))/40;
                    
                    if(mpc_cycle == 0)
                        q_pinocchio_desired = pinocchio::integrate(model, q_pinocchio_desired, qd_pinocchio * 0.0005);
                    else
                        q_pinocchio_desired = pinocchio::integrate(model, q_pinocchio_desired, qd_pinocchio * 0.0005);
                    
                    q_pinocchio_desired1 = q_pinocchio_desired;
                    qd_pinocchio_desired1 = qd_pinocchio;
                    walk_start1 = true;
                }  
                if(walking_tick == 40)
                {
                    walking_tick = 0;
                    walking_tick_stop = true;
                    mpc_cycle = mpc_cycle + 1;
                }

                if (walking_tick_stop == false)
                    walking_tick = walking_tick + 1;
            }
        }
    }
    endTime = std::chrono::system_clock::now();
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
        /*int variable_size, constraint_size;

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

        q_dm = pinv * X;*/
    }
    else
    {  
        MatrixXd J1, H1, A1;
        VectorXd X1, g1, lb1, ub1, lbA1, ubA1;
        X1.setZero(constraint_size2);
        J1.setZero(constraint_size2, variable_size2);
        H1.setZero(variable_size2, variable_size2);
        A1.setZero(constraint_size2, variable_size2);
        g1.setZero(variable_size2);
        lbA1.setZero(constraint_size2);
        ubA1.setZero(constraint_size2);
        lb1.setZero(variable_size2);
        ub1.setZero(variable_size2);
           
        lb1.setConstant(variable_size2, -100000);
        ub1.setConstant(variable_size2, 100000);
    
        H1(3,3) = 100.0;
        H1(4,4) = 100.0;
        H1(5,5) = 100.0;
        //std::cout << "Jac" << std::endl;
        //std::cout << Robot.link_[COM_id].Jac().block(0,0,3,3) << std::endl;
        J.block(0,0,3,18) = InitRPYM.linear() * Robot.link_[COM_id].Jac().block(0,0,3,18);
        J.block(3,0,6,18) = J_RFF.block(0,0,6,18);
        J.block(9,0,6,18) = J_LFF.block(0,0,6,18);
        
        X.segment<3>(0) = comd;
        X.segment<3>(3) = rfootd;
        X.segment<3>(6) = rfootori;
        X.segment<3>(9) = lfootd;
        X.segment<3>(12) = lfootori;
       
        H1 = H1;
        g1 = g1;
    
        A1 = J;
        lbA1 = X;
        ubA1 = X;

        qp_momentum_control.UpdateMinProblem(H1, g1);
        qp_momentum_control.UpdateSubjectToAx(A1, lbA1, ubA1);
        qp_momentum_control.UpdateSubjectToX(lb1, ub1);
       
        qp_solved = qp_momentum_control.SolveQPoases(100, qp_result1);
        
        if(qp_solved == true)
            q_dm = qp_result1;
        else
            q_dm.setZero();
    }
}

void CustomController::zmpControl(RobotData &Robot)
{
    std::cout << "ZMP";
}

void CustomController::setContact_1(RobotData &Robot, bool left_foot, bool right_foot, bool left_hand=0, bool right_hand=0)
{
    Robot.ee_[0].contact = left_foot;
    Robot.ee_[1].contact = right_foot;
    Robot.ee_[2].contact = left_hand;
    Robot.ee_[3].contact = right_hand;

    Robot.contact_index = 0;
    if (left_foot)
    {
        Robot.contact_part[Robot.contact_index] = TOCABI::Left_Foot;
        Robot.ee_idx[Robot.contact_index] = 0;
        Robot.contact_index++;
    }
    if (right_foot)
    {
        Robot.contact_part[Robot.contact_index] = TOCABI::Right_Foot;
        Robot.ee_idx[Robot.contact_index] = 1;
        Robot.contact_index++;
    }
    if (left_hand)
    {
        Robot.contact_part[Robot.contact_index] = TOCABI::Left_Hand;
        Robot.ee_idx[Robot.contact_index] = 2;
        Robot.contact_index++;
    }
    if (right_hand)
    {
        Robot.contact_part[Robot.contact_index] = TOCABI::Right_Hand;
        Robot.ee_idx[Robot.contact_index] = 3;
        Robot.contact_index++;
    }

    Robot.J_C.setZero(Robot.contact_index * 6, MODEL_DOF_VIRTUAL);
}