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
#include <fstream>
using namespace TOCABI;

pinocchio::Model model;
pinocchio::Model model_state;
pinocchio::Data model_data;
pinocchio::Data model_data_cen;
pinocchio::Data model_data_test;
pinocchio::Data model_data_state;

SHMmsgs *mj_shm_;
int shm_msg_id;

CSharedMemory mpc_start_init, state_init, statemachine, desired_val, desired_cp;

CustomController::CustomController(RobotData &rd) : rd_(rd)
{  
    nh_avatar_.setCallbackQueue(&queue_avatar_);
   
    bool urdfmode = false;
    std::string urdf_path, desc_package_path;
    ros::param::get("/tocabi_controller/urdf_path", desc_package_path);

    RigidBodyDynamics::Addons::URDFReadFromFile(desc_package_path.c_str(), &model_d_, true, false);
    RigidBodyDynamics::Addons::URDFReadFromFile(desc_package_path.c_str(), &model_c_, true, false);
    RigidBodyDynamics::Addons::URDFReadFromFile(desc_package_path.c_str(), &model_C_, true, false);
   
    pinocchio::JointModelFreeFlyer root_joint;
    pinocchio::Model model1;
    pinocchio::urdf::buildModel("/home/dyros/catkin_ws/src/dyros_tocabi_v2/tocabi_description/robots/dyros_tocabi.urdf", root_joint, model1);
    model = model1;
    model_state = model1;
    model = model1;
    pinocchio::SE3 M = pinocchio::SE3::Identity();
    M.translation() << 0.03, 0.0,-0.1585;
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

    RFjoint_id = model_state.getJointId("R_AnkleRoll_Joint");
    LFjoint_id = model_state.getJointId("L_AnkleRoll_Joint");
    RFframe_id = model_state.getFrameId("R_Foot_Link");
    LFframe_id = model_state.getFrameId("L_Foot_Link");
    model_state.addBodyFrame("LF_contact", LFjoint_id, M, LFframe_id);
    model_state.addBodyFrame("RF_contact", RFjoint_id, M, RFframe_id);
    LFcframe_id = model_state.getFrameId("LF_contact");
    RFcframe_id = model_state.getFrameId("RF_contact");
    RFframe_id = model_state.getFrameId("R_Foot_Link");
    LFframe_id = model_state.getFrameId("L_Foot_Link");
    pinocchio::Data data(model);
    model_data = data;
    model_data_state = data;
    model_data_cen = data;
    model_data_test = data;

    q_virtual_state.setZero();
    q_virtual_state(0) = 0.0;
    q_virtual_state(1) = 0.0;
    q_virtual_state(2) = 0.0;
    q_virtual_state(3) = 0.0;
    q_virtual_state(4) = 0.0;
    q_virtual_state(5) = 0.0;
    q_virtual_state(6) = 1.0;

    init_shm(shm_msg_key, shm_msg_id, &mj_shm_);

    //Disturbance
    //mj_shm_->dis_check = false;
    walking_tick_stop = true;

    for (int i = 0; i < FILE_CNT1; i++)
    {
        file[i].open(FILE_NAMES1[i]);
    }

    setGains();
    first_loop_larm_ = true;
    first_loop_rarm_ = true;
    first_loop_upperbody_ = true;
    first_loop_hqpik_ = true;
    first_loop_hqpik2_ = true;
    first_loop_qp_retargeting_ = true;
    first_loop_camhqp_ = true;

    contactMode = 1; //2 LF

    q_desireddot.setZero(18);
    COMX.setZero(3);
    MOMX.setZero(2);

    std::vector<double> RF_tran, LF_tran, ZMP_bound, com_ref, comd_ref, angm_ref, zmp_ref;
    std::string string_test;
    double jointvalue;
    /*
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

    rfoot_ori1.setZero();
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
    std::cout << "SHAREDMEMORY Third  OK"<< std::endl;
    desired_val.setKey(4);
    desired_val.setupSharedMemoryRead(sizeof(double) * 49);
    desired_val.attachSharedMemory();
    std::cout << "SHAREDMEMORY Fourth  OK"<< std::endl;
    desired_cp.setKey(5);
    desired_cp.setupSharedMemoryRead(sizeof(double) * 49);
    desired_cp.attachSharedMemory();
    */
    mpc_cycle = 0;
    walking_tick = 0;

    variable_size1 = MODEL_DOF_VIRTUAL + 12 + 2;
    constraint_size1 = 63 + 12;//6 + 12 + 8 + 8 + 1 + MODEL_DOF_VIRTUAL;// + 1;

    qp_torque_control.InitializeProblemSize(variable_size1, constraint_size1);
    qp_torque_control.EnableEqualityCondition(0.001);
   

    H1_temp.setZero(variable_size1, variable_size1);
    g1_temp.setZero(variable_size1);

    X1.setZero(constraint_size1);
    J1.setZero(constraint_size1, variable_size1);
    H1.setZero(variable_size1, variable_size1);
    A1.setZero(constraint_size1, variable_size1);
    g1.setZero(variable_size1);
    lbA1.setZero(constraint_size1);
    ubA1.setZero(constraint_size1);
    lb1.setZero(variable_size1);
    ub1.setZero(variable_size1);

    G_temp.setZero(12, MODEL_DOF_VIRTUAL);

    com_alpha = 0.5;
    qp_result.setZero(variable_size1);

    RFj.setZero(6, MODEL_DOF_VIRTUAL);
    LFj.setZero(6, MODEL_DOF_VIRTUAL);
    RFdj.setZero(6, MODEL_DOF_VIRTUAL);
    LFdj.setZero(6, MODEL_DOF_VIRTUAL);

    qd_des_prev.setZero();
    qd_des_virtual_prev.setZero();
    qdd_des_virtual.setZero();
    qd_des_virtual_prev.setZero();
    qd_des_lpf.setZero();
    qdd_des_lpf.setZero();
    qdd_des_virtual_.setZero();
    qdd_des_virtual_fast.setZero();

    nonlineareff.resize(MODEL_DOF_VIRTUAL,1);
    tau_.setZero(MODEL_DOF_VIRTUAL);

    if(as == 0)
    {
        variable_size2 = 18;
        constraint_size2 = 15;
    }
    else
    {
        variable_size2 = 18;
        constraint_size2 = 17;
    }
    qp_result1.setZero(variable_size2);
    qp_momentum_control.InitializeProblemSize(variable_size2, constraint_size2);
    qp_momentum_control.EnableEqualityCondition(0.001);
    q_dm.setZero(variable_size2);

    X2.setZero(constraint_size2);
    J2.setZero(constraint_size2, variable_size2);
    H2.setZero(variable_size2, variable_size2);
    A2.setZero(constraint_size2, variable_size2);
    g2.setZero(variable_size2);
    lbA2.setZero(constraint_size2);
    ubA2.setZero(constraint_size2);
    lb2.setZero(variable_size2);
    ub2.setZero(variable_size2);
       
    lb2.setConstant(variable_size2, -100000);
    ub2.setConstant(variable_size2, 100000);

    Hg_slow_.setZero(6, MODEL_DOF_VIRTUAL);
    Hg.setZero(6, MODEL_DOF_VIRTUAL);
    Hg_.setZero(6, MODEL_DOF_VIRTUAL);

    q_desireddot.setZero(18);
    com_prev.resize(50);
    a_temp = 50;
    q_upper.setZero();

    state_init_.setZero(50);
    state_init_slow.setZero(50);
    state_init_mu.setZero(50);
    desired_val_.setZero(49);
    desired_val_slow.setZero(49);
    desired_val_mu.setZero(49);

    mpc_start_init_ = 4;
   
    socket_send = socket(PF_INET, SOCK_STREAM, 0);
    socket_receive = socket(PF_INET, SOCK_STREAM, 0); //SOCK_STREAM

    if (socket_send < 0) {
        std::cout << "socket_send creation error" << std::endl;
    }
    else{
        std::cout << "socet send OK" << std::endl;
    }

    if (socket_receive < 0) {
        std::cout << "socket_receive creation error" << std::endl;
    }
    else{
        std::cout << "socet receive OK" << std::endl;
    }


    bzero(&serveraddr, sizeof(serveraddr));
    bzero(&serveraddr1, sizeof(serveraddr1));
    serveraddr.sin_family=PF_INET;
    serveraddr.sin_port=htons(8076);
    serveraddr1.sin_family=PF_INET;
    serveraddr1.sin_port=htons(7073);

   

    if(inet_pton(PF_INET, "10.112.1.10", &serveraddr1.sin_addr)<=0)
    {
        std::cout << "Invalid address/ Address not supported" << std::endl;
    }
    else
    {
        std::cout << "Ip add2 OK" << std::endl;
    }

    upper_on = true;

    thread proc1(&CustomController::proc_recv, this);
    thread proc2(&CustomController::proc_recv1, this);

    proc1.detach();
    proc2.detach();
}

void CustomController::setGains()
{
    ////real
    kp_compos_.setZero();
    kd_compos_.setZero();

    kp_compos_(0, 0) = 0.2;
    kp_compos_(1, 1) = 0.2;
    kp_compos_(2, 2) = 0.2;

    kd_compos_(0, 0) = 0.00;
    kd_compos_(1, 1) = 0.00;
    kd_compos_(2, 2) = 0.00;

    //////////COM LIMIT/////
    //min
    com_pos_limit_(0) = -0.5;
    com_pos_limit_(1) = -0.5;
    com_pos_limit_(2) = 0.5;
    //max
    com_pos_limit_(3) = 0.5;
    com_pos_limit_(4) = 0.5;
    com_pos_limit_(5) = 0.9;

    //min
    com_vel_limit_(0) = -0.5;
    com_vel_limit_(1) = -0.5;
    com_vel_limit_(2) = -0.2;
    //max
    com_vel_limit_(3) = +0.5;
    com_vel_limit_(4) = +0.5;
    com_vel_limit_(5) = +0.2;

    //min
    com_acc_limit_(0) = -5;
    com_acc_limit_(1) = -5;
    com_acc_limit_(2) = -2;
    //max
    com_acc_limit_(3) = 5;
    com_acc_limit_(4) = 5;
    com_acc_limit_(5) = 2;

    ////////////////////////////////////////////////////

    /////////Torque Limit///////////
    torque_task_min_(0) = -300;
    torque_task_min_(1) = -300;
    torque_task_min_(2) = -300;
    torque_task_min_(3) = -300;
    torque_task_min_(4) = -300;
    torque_task_min_(5) = -300;

    torque_task_min_(6) = -300;
    torque_task_min_(7) = -300;
    torque_task_min_(8) = -300;
    torque_task_min_(9) = -300;
    torque_task_min_(10) = -300;
    torque_task_min_(11) = -300;

    torque_task_min_(12) = -300;
    torque_task_min_(13) = -300;
    torque_task_min_(14) = -300;

    torque_task_min_(15) = -300;
    torque_task_min_(16) = -300;
    torque_task_min_(17) = -300;
    torque_task_min_(18) = -300;
    torque_task_min_(19) = -300;
    torque_task_min_(20) = -300;
    torque_task_min_(21) = -100;
    torque_task_min_(22) = -100;

    torque_task_min_(23) = -100;
    torque_task_min_(24) = -100;

    torque_task_min_(25) = -300;
    torque_task_min_(26) = -300;
    torque_task_min_(27) = -300;
    torque_task_min_(28) = -300;
    torque_task_min_(29) = -300;
    torque_task_min_(30) = -300;
    torque_task_min_(31) = -100;
    torque_task_min_(32) = -100;

    torque_task_max_(0) = 300;
    torque_task_max_(1) = 300;
    torque_task_max_(2) = 300;
    torque_task_max_(3) = 300;
    torque_task_max_(4) = 300;
    torque_task_max_(5) = 300;

    torque_task_max_(6) = 300;
    torque_task_max_(7) = 300;
    torque_task_max_(8) = 300;
    torque_task_max_(9) = 300;
    torque_task_max_(10) = 300;
    torque_task_max_(11) = 300;

    torque_task_max_(12) = 300;
    torque_task_max_(13) = 300;
    torque_task_max_(14) = 300;

    torque_task_max_(15) = 100;
    torque_task_max_(16) = 300;
    torque_task_max_(17) = 300;
    torque_task_max_(18) = 300;
    torque_task_max_(19) = 300;
    torque_task_max_(20) = 300;
    torque_task_max_(21) = 100;
    torque_task_max_(22) = 100;

    torque_task_max_(23) = 100;
    torque_task_max_(24) = 100;

    torque_task_max_(25) = 100;
    torque_task_max_(26) = 300;
    torque_task_max_(27) = 300;
    torque_task_max_(28) = 300;
    torque_task_max_(29) = 300;
    torque_task_max_(30) = 300;
    torque_task_max_(31) = 100;
    torque_task_max_(32) = 100;
    ////////////////////////////////

    //////////Joint PD Gain/////////
    ///For Simulation
    // for (int i = 0; i < MODEL_DOF; i++)
    // {
    // 	kp_joint_(i) = 100; 		//(tune)
    // 	kv_joint_(i) = 20;		//(tune)
    // }

    // //Waist Joint Gains
    // for (int i = 0; i < 3; i++)
    // {
    // 	kp_joint_(12 + i) = 900;
    // 	kv_joint_(12 + i) = 60;
    // }
    // kp_joint_(12) = 2500;
    // kp_joint_(13) = 900;
    // kp_joint_(14) = 900;

    // kv_joint_(12) = 100;
    // kv_joint_(13) = 60;
    // kv_joint_(14) = 60;

    // kp_joint_(20) = 64;	//forearm
    // kp_joint_(21) = 64;	//wrist1
    // kp_joint_(22) = 64;	//wrist2
    // kv_joint_(20) = 10;
    // kv_joint_(21) = 10;
    // kv_joint_(22) = 10;

    // kp_joint_(30) = 64;
    // kp_joint_(31) = 64;
    // kp_joint_(32) = 64;
    // kv_joint_(30) = 10;
    // kv_joint_(31) = 10;
    // kv_joint_(32) = 10;

    // kp_joint_(23) = 49;	//head
    // kp_joint_(24) = 49;
    // kv_joint_(23) = 14;	//head
    // kv_joint_(24) = 14;

    // //stiff	//(tune)
    // kp_stiff_joint_(0) = 3600; //R hip yaw joint gain
    // kv_stiff_joint_(0) = 120;
    // kp_stiff_joint_(1) = 4900; //L hip roll joint gain
    // kv_stiff_joint_(1) = 140;
    // kp_stiff_joint_(2) = 4900; //L hip pitch joint gain
    // kv_stiff_joint_(2) = 140;

    // kp_stiff_joint_(3) = 1600; //L knee joint gain
    // kv_stiff_joint_(3) = 80;

    // kp_stiff_joint_(4) = 400; //L ankle pitch joint gain
    // kv_stiff_joint_(4) = 40;
    // kp_stiff_joint_(5) = 400; //L ankle roll joint gain
    // kv_stiff_joint_(5) = 40;

    // kp_stiff_joint_(6) = 3600; //R hip yaw joint gain
    // kv_stiff_joint_(6) = 120;
    // kp_stiff_joint_(7) = 4900; //R hip roll joint gain
    // kv_stiff_joint_(7) = 140;
    // kp_stiff_joint_(8) = 4900; //R hip pitch joint gain
    // kv_stiff_joint_(8) = 140;

    // kp_stiff_joint_(9) = 1600; //R knee joint gain
    // kv_stiff_joint_(9) = 80;

    // kp_stiff_joint_(10) = 400; //R ankle pitch joint gain
    // kv_stiff_joint_(10) = 40;
    // kp_stiff_joint_(11) = 400; //R ankle roll joint gain
    // kv_stiff_joint_(11) = 40;

    // //soft	//(tune)
    // kp_soft_joint_(0) = 3600; //L hip yaw joint gain
    // kv_soft_joint_(0) = 120;
    // kp_soft_joint_(1) = 400; //L hip roll joint gain
    // kv_soft_joint_(1) = 40;
    // kp_soft_joint_(2) = 400; //L hip pitch joint gain
    // kv_soft_joint_(2) = 40;

    // kp_soft_joint_(3) = 100; //L knee joint gain
    // kv_soft_joint_(3) = 20;

    // kp_soft_joint_(4) = 25; //L ankle pitch joint gain
    // kv_soft_joint_(4) = 10;
    // kp_soft_joint_(5) = 25; //L ankle roll joint gain
    // kv_soft_joint_(5) = 10;

    // kp_soft_joint_(6) = 3600; //R hip yaw joint gain
    // kv_soft_joint_(6) = 120;
    // kp_soft_joint_(7) = 400; //R hip roll joint gain
    // kv_soft_joint_(7) = 40;
    // kp_soft_joint_(8) = 400; //R hip pitch joint gain
    // kv_soft_joint_(8) = 40;

    // kp_soft_joint_(9) = 100; //R knee joint gain
    // kv_soft_joint_(9) = 20;

    // kp_soft_joint_(10) = 25; //R ankle pitch joint gain
    // kv_soft_joint_(10) = 10;
    // kp_soft_joint_(11) = 25; //R ankle roll joint gain
    // kv_soft_joint_(11) = 10;

    // for (int i = 0; i < 12; i++) //Leg
    // {
    // 	kp_joint_(i) = kp_stiff_joint_(i);
    // 	kv_joint_(i) = kv_stiff_joint_(i);
    // }
    /////////////////

    ///For Real Robot
    kp_stiff_joint_(0) = 2000; //right leg
    kp_stiff_joint_(1) = 5000;
    kp_stiff_joint_(2) = 4000;
    kp_stiff_joint_(3) = 3700;
    kp_stiff_joint_(4) = 5000;
    kp_stiff_joint_(5) = 5000;
    kp_stiff_joint_(6) = 2000; //left leg
    kp_stiff_joint_(7) = 5000;
    kp_stiff_joint_(8) = 4000;
    kp_stiff_joint_(9) = 3700;
    kp_stiff_joint_(10) = 5000;
    kp_stiff_joint_(11) = 5000;
    kp_stiff_joint_(12) = 6000; //waist
    kp_stiff_joint_(13) = 10000;
    kp_stiff_joint_(14) = 10000;
    kp_stiff_joint_(15) = 2000;//400; //left arm
    kp_stiff_joint_(16) = 3000;//800;
    kp_stiff_joint_(17) = 2000;//400;
    kp_stiff_joint_(18) = 2000;//400;
    kp_stiff_joint_(19) = 125;
    kp_stiff_joint_(20) = 125;
    kp_stiff_joint_(21) = 25;
    kp_stiff_joint_(22) = 25;
    kp_stiff_joint_(23) = 50; //head
    kp_stiff_joint_(24) = 50;
    kp_stiff_joint_(25) = 2000;//400; //right arm
    kp_stiff_joint_(26) = 3000;//800;
    kp_stiff_joint_(27) = 2000;//400;
    kp_stiff_joint_(28) = 2000;//400;
    kp_stiff_joint_(29) = 125;
    kp_stiff_joint_(30) = 125;
    kp_stiff_joint_(31) = 25;
    kp_stiff_joint_(32) = 25;

    kv_stiff_joint_(0) = 15; //right leg
    kv_stiff_joint_(1) = 50;
    kv_stiff_joint_(2) = 20;
    kv_stiff_joint_(3) = 25;
    kv_stiff_joint_(4) = 30;
    kv_stiff_joint_(5) = 30;
    kv_stiff_joint_(6) = 15; //left leg
    kv_stiff_joint_(7) = 50;
    kv_stiff_joint_(8) = 20;
    kv_stiff_joint_(9) = 25;
    kv_stiff_joint_(10) = 30;
    kv_stiff_joint_(11) = 30;
    kv_stiff_joint_(12) = 200; //waist
    kv_stiff_joint_(13) = 100;
    kv_stiff_joint_(14) = 100;
    kv_stiff_joint_(15) = 20;//7; //left arm
    kv_stiff_joint_(16) = 20;//5;
    kv_stiff_joint_(17) = 20;//2.5;
    kv_stiff_joint_(18) = 20;//2.5;
    kv_stiff_joint_(19) = 2.5;
    kv_stiff_joint_(20) = 2;
    kv_stiff_joint_(21) = 2;
    kv_stiff_joint_(22) = 2;
    kv_stiff_joint_(23) = 2; //head
    kv_stiff_joint_(24) = 2;
    kv_stiff_joint_(25) = 20;//7; //right arm
    kv_stiff_joint_(26) = 20;//5;
    kv_stiff_joint_(27) = 20;//2.5;
    kv_stiff_joint_(28) = 20;//2.5;
    kv_stiff_joint_(29) = 2.5;
    kv_stiff_joint_(30) = 2;
    kv_stiff_joint_(31) = 2;
    kv_stiff_joint_(32) = 2;

    kp_soft_joint_(0) = 2000; //right leg
    kp_soft_joint_(1) = 5000;
    kp_soft_joint_(2) = 4000;
    kp_soft_joint_(3) = 3700;
    kp_soft_joint_(4) = 5000;
    kp_soft_joint_(5) = 5000;
    kp_soft_joint_(6) = 2000; //left leg
    kp_soft_joint_(7) = 5000;
    kp_soft_joint_(8) = 4000;
    kp_soft_joint_(9) = 3700;
    kp_soft_joint_(10) = 5000;
    kp_soft_joint_(11) = 5000;
    kp_soft_joint_(12) = 6000; //waist
    kp_soft_joint_(13) = 10000;
    kp_soft_joint_(14) = 10000;
    kp_soft_joint_(15) = 200; //left arm
    kp_soft_joint_(16) = 80;
    kp_soft_joint_(17) = 60;
    kp_soft_joint_(18) = 60;
    kp_soft_joint_(19) = 60;
    kp_soft_joint_(20) = 60;
    kp_soft_joint_(21) = 20;
    kp_soft_joint_(22) = 20;
    kp_soft_joint_(23) = 50; //head
    kp_soft_joint_(24) = 50;
    kp_soft_joint_(25) = 200; //right arm
    kp_soft_joint_(26) = 80;
    kp_soft_joint_(27) = 60;
    kp_soft_joint_(28) = 60;
    kp_soft_joint_(29) = 60;
    kp_soft_joint_(30) = 60;
    kp_soft_joint_(31) = 20;
    kp_soft_joint_(32) = 20;

    kv_soft_joint_(0) = 15; //right leg
    kv_soft_joint_(1) = 50;
    kv_soft_joint_(2) = 20;
    kv_soft_joint_(3) = 25;
    kv_soft_joint_(4) = 30;
    kv_soft_joint_(5) = 30;
    kv_soft_joint_(6) = 15; //left leg
    kv_soft_joint_(7) = 50;
    kv_soft_joint_(8) = 20;
    kv_soft_joint_(9) = 25;
    kv_soft_joint_(10) = 30;
    kv_soft_joint_(11) = 30;
    kv_soft_joint_(12) = 200; //waist
    kv_soft_joint_(13) = 100;
    kv_soft_joint_(14) = 100;
    kv_soft_joint_(15) = 14; //left arm
    kv_soft_joint_(16) = 10;
    kv_soft_joint_(17) = 5;
    kv_soft_joint_(18) = 5;
    kv_soft_joint_(19) = 2.5;
    kv_soft_joint_(20) = 2;
    kv_soft_joint_(21) = 2;
    kv_soft_joint_(22) = 2;
    kv_soft_joint_(23) = 2; //head
    kv_soft_joint_(24) = 2;
    kv_soft_joint_(25) = 14; //right arm
    kv_soft_joint_(26) = 10;
    kv_soft_joint_(27) = 5;
    kv_soft_joint_(28) = 5;
    kv_soft_joint_(29) = 2.5;
    kv_soft_joint_(30) = 2;
    kv_soft_joint_(31) = 2;
    kv_soft_joint_(32) = 2;
    // for (int i = 0; i < MODEL_DOF; i++)
    // {
    //     kp_soft_joint_(i) = kp_stiff_joint_(i) / 4;
    //     kp_soft_joint_(i) = kv_stiff_joint_(i) / 2;
    // }

    for (int i = 0; i < MODEL_DOF; i++)
    {
        kp_joint_(i) = kp_stiff_joint_(i);
        kv_joint_(i) = kv_stiff_joint_(i);
    }
    ///////////////

    ///////////////////////////////

    //arm controller
    joint_limit_l_.resize(33);
    joint_limit_h_.resize(33);
    joint_vel_limit_l_.resize(33);
    joint_vel_limit_h_.resize(33);

    //LEG
    for (int i = 0; i < 12; i++)
    {
        joint_limit_l_(i) = -180 * DEG2RAD;
        joint_limit_h_(i) = 180 * DEG2RAD;
    }

    //WAIST
    joint_limit_l_(12) = -30 * DEG2RAD;
    joint_limit_h_(12) = 30 * DEG2RAD;
    joint_limit_l_(13) = -15 * DEG2RAD;
    joint_limit_h_(13) = 30 * DEG2RAD;
    joint_limit_l_(14) = -15 * DEG2RAD;
    joint_limit_h_(14) = 15 * DEG2RAD;
    //LEFT ARM
    joint_limit_l_(15) = -30 * DEG2RAD;
    joint_limit_h_(15) = 30 * DEG2RAD;
    joint_limit_l_(16) = -160 * DEG2RAD;
    joint_limit_h_(16) = 70 * DEG2RAD;
    joint_limit_l_(17) = -95 * DEG2RAD;
    joint_limit_h_(17) = 95 * DEG2RAD;
    joint_limit_l_(18) = -180 * DEG2RAD;
    joint_limit_h_(18) = 180 * DEG2RAD;
    joint_limit_l_(19) = -150 * DEG2RAD;
    joint_limit_h_(19) = -10 * DEG2RAD;
    joint_limit_l_(20) = -180 * DEG2RAD;
    joint_limit_h_(20) = 180 * DEG2RAD;
    joint_limit_l_(21) = -70 * DEG2RAD;
    joint_limit_h_(21) = 70 * DEG2RAD;
    joint_limit_l_(22) = -60 * DEG2RAD;
    joint_limit_h_(22) = 60 * DEG2RAD;
    //HEAD
    joint_limit_l_(23) = -80 * DEG2RAD;
    joint_limit_h_(23) = 80 * DEG2RAD;
    joint_limit_l_(24) = -40 * DEG2RAD;
    joint_limit_h_(24) = 30 * DEG2RAD;
    //RIGHT ARM
    joint_limit_l_(25) = -30 * DEG2RAD;
    joint_limit_h_(25) = 30 * DEG2RAD;
    joint_limit_l_(26) = -70 * DEG2RAD;
    joint_limit_h_(26) = 160 * DEG2RAD;
    joint_limit_l_(27) = -95 * DEG2RAD;
    joint_limit_h_(27) = 95 * DEG2RAD;
    joint_limit_l_(28) = -180 * DEG2RAD;
    joint_limit_h_(28) = 180 * DEG2RAD;
    joint_limit_l_(29) = 10 * DEG2RAD;
    joint_limit_h_(29) = 150 * DEG2RAD;
    joint_limit_l_(30) = -180 * DEG2RAD;
    joint_limit_h_(30) = 180 * DEG2RAD;
    joint_limit_l_(31) = -70 * DEG2RAD;
    joint_limit_h_(31) = 70 * DEG2RAD;
    joint_limit_l_(32) = -60 * DEG2RAD;
    joint_limit_h_(32) = 60 * DEG2RAD;

    //LEG
    for (int i = 0; i < 12; i++)
    {
        joint_vel_limit_l_(i) = -2 * M_PI;
        joint_vel_limit_h_(i) = 2 * M_PI;
    }

    //UPPERBODY
    for (int i = 12; i < 33; i++)
    {
        joint_vel_limit_l_(i) = -M_PI * 3;
        joint_vel_limit_h_(i) = M_PI * 3;
    }

    //1st arm joint vel limit
    joint_vel_limit_l_(15) = -M_PI / 3;
    joint_vel_limit_h_(15) = M_PI / 3;

    joint_vel_limit_l_(25) = -M_PI / 3;
    joint_vel_limit_h_(25) = M_PI / 3;

    // Head joint vel limit
    joint_vel_limit_l_(23) = -2 * M_PI;
    joint_vel_limit_h_(23) = 2 * M_PI;
    joint_vel_limit_l_(24) = -2 * M_PI;
    joint_vel_limit_h_(24) = 2 * M_PI;

    // forearm joint vel limit
    joint_vel_limit_l_(20) = -2 * M_PI;
    joint_vel_limit_h_(20) = 2 * M_PI;
    joint_vel_limit_l_(30) = -2 * M_PI;
    joint_vel_limit_h_(30) = 2 * M_PI;
}

Eigen::VectorQd CustomController::getControl()
{
    return rd_.torque_desired;
}

void CustomController::computeSlow()
{
    queue_avatar_.callAvailable(ros::WallDuration());
   
    if (rd_.tc_.mode == 6)
    {
        if (initial_flag == 0)
        {
            Joint_gain_set_MJ();
            walking_enable_ = true;
            // Initial pose
            ref_q_ = rd_.q_;
            for (int i = 0; i < 12; i++)
            {
                Initial_ref_q_(i) = ref_q_(i);
            }

            // Saving for initial upper body pose
            // edited by MJ (Initial upper body trajectory generation for CAM control /220110)
            CAM_upper_init_q_.setZero();
            Initial_ref_upper_q_.setZero();
            for (int i = 12; i < MODEL_DOF; i++)
            {
                Initial_ref_upper_q_(i) = ref_q_(i);
            }
            /*CAM_upper_init_q_(16) = +10.0 * DEG2RAD;
            CAM_upper_init_q_(26) = -10.0 * DEG2RAD;
            CAM_upper_init_q_(17) = +70.0 * DEG2RAD; // Rolling dist +70.0 deg
            CAM_upper_init_q_(27) = -70.0 * DEG2RAD; // Rolling dist -70.0 deg
            CAM_upper_init_q_(19) = -90.0 * DEG2RAD; //-90.0
            CAM_upper_init_q_(29) = +90.0 * DEG2RAD; //+90.0*/

            CAM_upper_init_q_.segment<18>(15) << 0.2, 0.6, 1.5, -1.47, -1,
                    0, -1, 0, 0, 0, -0.2, -0.6, -1.5, 1.47, 1,
                    0, 1, 0;
           
            q_prev_MJ_ = rd_.q_;
            walking_tick_mj = 0;
            walking_end_flag = 0;
            parameterSetting();
            cout << "computeslow mode = 10 is initialized" << endl;
            cout << "time: "<<rd_.control_time_ << endl; //dg add

            WBC::SetContact(rd_, 1, 1);
            if(torque_control == true)  
                Gravity_MJ_ = WBC::ContactForceRedistributionTorqueWalking(rd_, WBC::GravityCompensationTorque(rd_), 0.9, 1, 0);
            else
                Gravity_MJ_ = WBC::ContactForceRedistributionTorqueWalking(rd_, WBC::GravityCompensationTorque(rd_));
            //Gravity_MJ_.setZero();
            atb_grav_update_ = false;
            initial_flag = 1;
        }

        if (atb_desired_update_ == false)
        {
            atb_grav_update_ = true;
            Gravity_MJ_fast_ = Gravity_MJ_;
            atb_grav_update_ = false;
        }  

        // edited by MJ (Initial upper body trajectory generation for CAM control /220110)
        if(initial_tick_mj <= 2.0 * hz_)
        {
            Eigen::Vector12d q_leg;
            q_leg.head(12) <<  0, 0, -0.55, 1.26, -0.71, 0, 0, 0, -0.55, 1.26, -0.71, 0;//0, 0, -0.24, 0.6, -0.36, 0, 0, 0, -0.24, 0.6, -0.36, 0;//
            for (int i = 0; i < 12; i++)
                ref_q_(i) = DyrosMath::cubic(initial_tick_mj, 0, 2.0 * hz_, Initial_ref_q_(i), q_leg(i), 0.0, 0.0);

            for(int i = 15; i < 33; i ++)
                ref_q_(i) = DyrosMath::cubic(initial_tick_mj, 0, 2.0 * hz_, Initial_ref_upper_q_(i), CAM_upper_init_q_(i), 0.0, 0.0);
           
            initial_tick_mj ++;        
        }
       
        for (int i = 0; i < MODEL_DOF; i++)
        {
            rd_.torque_desired(i) = Kp(i) * (ref_q_(i) - rd_.q_(i)) - Kd(i) * rd_.q_dot_(i) + 1.0 * Gravity_MJ_fast_(i);
        }        
    }
    else if (rd_.tc_.mode == 7)
    {
        if(walking_tick_mj < t_temp_ - t_mpc_init_ + t_total_ * 2 + 1)
            mpc_cycle = -1;
        else if(walking_tick_mj == t_temp_ - t_mpc_init_ + t_total_ * 2 + 1)
            mpc_cycle = 0;

        mpc_cycle_int = (mpc_cycle - 50) / 50;
        mpc_cycle_int1 = (mpc_cycle - 50) % 50;

        if (time_tick == false)
        {
            startTime = std::chrono::system_clock::now();
            time_tick = true;      
        }      

        while(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now() - startTime).count() < 500)
        {
            torque_upper_.setZero();
            torque_lower_.setZero();
            if(torque_control == false)
            {
                for (int i = 0; i < 12; i++)
                {
                    torque_lower_(i) = (Kp(i) * (ref_q_(i) - rd_.q_(i)) + Kd(i) * (q_dm(i+6) * 2000 - rd_.q_dot_(i))) + Gravity_MJ_fast_(i) + Tau_CP(i);
                }
            }
            else
            {
                for (int i = 0; i < 12; i++)
                {
                    torque_lower_(i) = (Kp(i) * (ref_q_(i) - rd_.q_(i)) + Kd(i) * (q_dm(i+6) * 2000 - rd_.q_dot_(i))) + Gravity_MJ_fast_(i) + Tau_CP(i);
                }
            }

            if(walking_end_flag == 1)
            {
                for (int i = 0; i < 12; i++)
                {
                    torque_lower_(i) = Kp(i) * (ref_q_(i) - rd_.q_(i)) - Kd(i) * rd_.q_dot_(i) + 1.0 * Gravity_MJ_fast_(i);
                }
            }
       
            for (int i = 0; i < 12; i++)
            {
                torque_lower_(i) = (Kp(i) * (ref_q_(i) - rd_.q_(i)) + Kd(i) * (q_dm(i+6) * 2000 - rd_.q_dot_(i))) + Gravity_MJ_fast_(i) + Tau_CP(i);
            }

            torque_upper_.setZero();
            for (int i = 12; i < MODEL_DOF; i++)
            {
                torque_upper_(i) = (kp_joint_(i) * (desired_q_fast_(i) - rd_.q_(i)) + kv_joint_(i) * (desired_q_dot_fast_(i) - rd_.q_dot_(i)) + 1.0 * Gravity_MJ_fast_(i));
            }

            rd_.torque_desired = torque_lower_ + torque_upper_;
        }
        int K_ = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now() - startTime).count();
       
           
        rd_.torque_desired = torque_lower_ + torque_upper_;
        startTime = std::chrono::system_clock::now();
        time_tick_next = true;

        if(walking_enable_ == true && time_tick_next == true)
        {
            if (walking_tick_mj == 0)
            {
                parameterSetting();
                initial_flag = 0;

                atb_grav_update_ = false;
                atb_desired_q_update_ = false;
                atb_walking_traj_update_ = false;
                torque_upper_fast_.setZero();
                torque_upper_fast_.segment(12, MODEL_DOF - 12) = rd_.torque_desired.segment(12, MODEL_DOF - 12);
                torque_upper_.setZero();
                torque_upper_.segment(12, MODEL_DOF - 12) = rd_.torque_desired.segment(12, MODEL_DOF - 12);

                cout << "parameter setting OK" << endl;
                cout << "mode = 11" << endl;
            }
                   
            updateInitialState();
            getRobotState();

            controlwalk_time = 2;
            if (mpc_cycle < controlwalk_time-1)
            {
                for (int i = 0; i < 3; i++)
                    q_pinocchio1[i] = rd_.q_virtual_[i];
           
                q_pinocchio1[3] = rd_.q_virtual_[3];
                q_pinocchio1[4] = rd_.q_virtual_[4];
                q_pinocchio1[5] = rd_.q_virtual_[5];
                q_pinocchio1[6] = rd_.q_virtual_[MODEL_DOF_VIRTUAL];
               
                for (int i = 6; i < MODEL_DOF_VIRTUAL; i++)
                    q_pinocchio1[i+1] = rd_.q_virtual_[i];
               
                pinocchio::normalize(model, q_pinocchio1);
                pinocchio::computeCentroidalMomentum(model, model_data_cen, q_pinocchio1, rd_.q_dot_virtual_);
                Hg_slow_ = pinocchio::computeCentroidalMap(model, model_data_cen, q_pinocchio1);
               
                if(time_tick_next == true)
                {
                    if(as == 0 || (as == 1 && mpc_cycle < 0))
                    {
                        floatToSupportFootstep();
                        getZmpTrajectory();
                        getComTrajectory();
                        ZMP_X_REF_1 = ZMP_X_REF;
                        ZMP_Y_REF_1 = ZMP_Y_REF;
                        CentroidalMomentCalculator();
                        getFootTrajectory();

                        if(walking_tick_mj ==  t_temp_ - t_mpc_init_ + t_total_ * 2 - 40 + 15)
                            getMPCTrajectoryInit();
                    }
                    else
                    {  
                        floatToSupportFootstep();
                        getMPCTrajectory();
                        if(mpc_cycle < 0)
                        {
                            getZmpTrajectory();
                            getComTrajectory();
                            CentroidalMomentCalculator();
                            ZMP_X_REF_1 = ZMP_X_REF;
                            ZMP_Y_REF_1 = ZMP_Y_REF;
                        }
                        else
                        {
                            if(mpc_cycle == 0)
                            {
                                getZmpTrajectory();
                                getComTrajectory();
                                CentroidalMomentCalculator();
                                ZMP_X_REF_1 = ZMP_X_REF;
                                ZMP_Y_REF_1 = ZMP_Y_REF;
                            }
                            if(mpc_cycle > 1 || (mpc_cycle == 1 && walking_tick >= 1))
                            {
                                //getZmpTrajectory();
                                //getComTrajectory();
                               
                                SC_err_compen(com_support_current_(0), com_support_current_(1));
                                cp_measured_(0) = com_support_cp_(0) + com_float_current_dot_LPF(0) / wn;
                                cp_measured_(1) = com_support_current_(1) + com_float_current_dot_LPF(1) / wn;
                                ZMP_X_REF_1 = ZMP_X_REF;
                                ZMP_Y_REF_1 = ZMP_Y_REF;
                                ZMP_X_REF = ZMP_gl(0);
                                ZMP_Y_REF = ZMP_gl(1);

                                double lx, ly, mu;
                                ly = 0.048;
                                lx = 0.11;

                                if(contactMode == 1)
                                {
                                    if(lfoot_sx >  rfoot_sx)
                                    {
                                        if(ZMP_X_REF > lfoot_sx + lx)
                                            ZMP_X_REF  = lfoot_sx + lx;
                                        else if(ZMP_X_REF < rfoot_sx - lx)
                                            ZMP_X_REF  = rfoot_sx - lx;
                                    }
                                    else if(lfoot_sx <  rfoot_sx)
                                    {
                                        if(ZMP_X_REF > rfoot_sx + lx)
                                            ZMP_X_REF  = rfoot_sx + lx;
                                        else if(ZMP_X_REF < lfoot_sx - lx)
                                            ZMP_X_REF  = lfoot_sx - lx;
                                    }
                                    else
                                    {
                                        if(ZMP_X_REF > rfoot_sx + lx)
                                            ZMP_X_REF  = rfoot_sx + lx;
                                        else if(ZMP_X_REF < lfoot_sx - lx)
                                            ZMP_X_REF  = lfoot_sx - lx;
                                    }

                                    if(ZMP_Y_REF > lfoot_sy + ly)
                                        ZMP_Y_REF = lfoot_sy + ly;
                                   
                                    if(ZMP_Y_REF < rfoot_sy - ly)
                                        ZMP_Y_REF = rfoot_sy - ly;
                                }
                                else if(contactMode == 2)
                                {
                                    if(ZMP_X_REF > lfoot_sx + lx)
                                        ZMP_X_REF  = lfoot_sx + lx;
                                    else if(ZMP_X_REF < lfoot_sx - lx)
                                        ZMP_X_REF  = lfoot_sx - lx;
                                   
                                    if(ZMP_Y_REF > lfoot_sy + ly)
                                        ZMP_Y_REF = lfoot_sy + ly;
                                    else if(ZMP_Y_REF < lfoot_sy - ly)
                                        ZMP_Y_REF = lfoot_sy - ly;
                                }
                                else
                                {
                                    if(ZMP_X_REF > rfoot_sx + lx)
                                        ZMP_X_REF  = rfoot_sx + lx;
                                    else if(ZMP_X_REF < rfoot_sx - lx)
                                        ZMP_X_REF  = rfoot_sx - lx;
                                   
                                    if(ZMP_Y_REF < rfoot_sy - ly)
                                        ZMP_Y_REF = rfoot_sy - ly;
                                    else if(ZMP_Y_REF > rfoot_sy + ly)
                                        ZMP_Y_REF = rfoot_sy + ly;
                                }
                               
                                cp_desired_(0) = com_mpc[0] + comd_s[0]/wn;
                                cp_desired_(1) = com_mpc[1] + comd_s[1]/wn;
                                com_dot_desired_(0) = comd_s[0];
                                com_dot_desired_(1) = comd_s[1];
                                com_desired_(0) = com_mpc[0];
                                com_desired_(1) = com_mpc[1];
                                del_zmp(0) = 0.7 * (cp_measured_(0) - cp_desired_(0));//1.6, 1.6
                                del_zmp(1) = 0.7 * (cp_measured_(1) - cp_desired_(1));
                            }
                        }
                        getFootTrajectory();
                    }
                }


                //Disturbance
                /*
                if(mpc_cycle >= 174  && mpc_cycle <= 178)//&& (walking_tick >= 1  && walking_tick <= 20))
                    mj_shm_->dis_check = true;
                else if(mpc_cycle == 179)
                    mj_shm_->dis_check = false;

                if(mpc_cycle >= 374  && mpc_cycle <= 378)//&& (walking_tick >= 1  && walking_tick <= 20))
                    mj_shm_->dis_check = true;
                else if(mpc_cycle == 379)
                    mj_shm_->dis_check = false;
               
                if(mpc_cycle >= 203  && mpc_cycle <= 207)//&& (walking_tick >= 1  && walking_tick <= 20))
                    mj_shm_->dis_check = true;
                else if(mpc_cycle == 208)
                    mj_shm_->dis_check = false;
               
                if(mpc_cycle >= 403 && mpc_cycle <= 407)//&& (walking_tick >= 1  && walking_tick <= 20))
                    mj_shm_->dis_check = true;
                else if(mpc_cycle == 408)
                    mj_shm_->dis_check = false;
                */
                //y-direction
                /*
                if(mpc_cycle >= 192  && mpc_cycle <= 196)//&& (walking_tick >= 1  && walking_tick <= 20))
                    mj_shm_->dis_check = true;
                else if(mpc_cycle == 197)
                    mj_shm_->dis_check = false;
               
                if(mpc_cycle >= 392 && mpc_cycle <= 396)//&& (walking_tick >= 1  && walking_tick <= 20))
                    mj_shm_->dis_check = true;
                else if(mpc_cycle ==397)
                    mj_shm_->dis_check = false;
                */

                if(atb_phase_update_ == false)
                {
                    atb_phase_update_ = true;
                    supportFoot_ = foot_step_(current_step_num_, 6);
                    zmpx_ = ZMP_X_REF;
                    zmpy_ = ZMP_Y_REF;
                    contactMode_ = contactMode;
                    com_alpha_ = com_alpha;
                    atb_phase_update_ = false;
                }
                getPelvTrajectory();
                supportToFloatPattern();
               
                if(walking_tick_mj == 0)
                {
                    com_d.setZero();
                    ang_d.setZero();
                    rfoot_d.setZero();
                    lfoot_d.setZero();
                    rfoot_ori.setZero();
                    lfoot_ori.setZero();
                    lfoot_trajectory_float_pre = lfoot_trajectory_float_;
                    rfoot_trajectory_float_pre = rfoot_trajectory_float_;
                    pelv_trajectory_float_pre = pelv_trajectory_float_;

                    computeIkControl_MJ(pelv_trajectory_float_, lfoot_trajectory_float_, rfoot_trajectory_float_, q_des_);
                    desired_q_not_compensated_.head(12) = q_des_;

                    if(momentumControlMode == false)
                        com_desired_(2) = com_support_current_(2);
                }
                else
                {
                    if(step_change == false)
                    {
                        lfootd1 = PELV_YaW6D.block(0,0,3,3) * lfootd1;
                        rfootd1 = PELV_YaW6D.block(0,0,3,3) * rfootd1;

                        lfootd1 = sup_YaW6D.block(0,0,3,3) * lfootd1;
                        rfootd1 = sup_YaW6D.block(0,0,3,3) * rfootd1;

                        com_d = com_dot_desired_/2000.0;
                        rfoot_d = rfootd1;
                        lfoot_d = lfootd1;

                        lfoot_ori = -DyrosMath::getPhi(lfoot_trajectory_float_pre.linear(), lfoot_trajectory_float_.linear()) * 0.5/10;
                        rfoot_ori = -DyrosMath::getPhi(rfoot_trajectory_float_pre.linear(), rfoot_trajectory_float_.linear()) * 0.5/10;

                        com_d1 = com_d;
                        rfoot_d1 = rfoot_d;
                        lfoot_d1 = lfoot_d;
                    }
                    else
                    {
                        com_d = com_d1;
                        rfoot_d = rfoot_d1;
                        lfoot_d = lfoot_d1;
                        lfoot_ori = -DyrosMath::getPhi(lfoot_trajectory_float_pre.linear(), lfoot_trajectory_float_.linear()) * 0.5/10;
                        rfoot_ori = -DyrosMath::getPhi(rfoot_trajectory_float_pre.linear(), rfoot_trajectory_float_.linear()) * 0.5/10;
                    }

                    /*lfoot_ori(0) = -F_T_L_x_input_dot/2000;
                    rfoot_ori(0) = -F_T_R_x_input_dot/2000;
                    lfoot_ori(1) = F_T_L_y_input_dot/2000;
                    rfoot_ori(1) = F_T_R_y_input_dot/2000;*/

                    pelv_vtran = (pelv_trajectory_float_.translation() - pelv_trajectory_float_pre.translation()) * 2000;
                    rf_vtran = (rfoot_trajectory_float_.translation() - rfoot_trajectory_float_pre.translation()) * 2000;
                    lf_vtran = (lfoot_trajectory_float_.translation() - lfoot_trajectory_float_pre.translation()) * 2000;


                    lfoot_trajectory_float_pre = lfoot_trajectory_float_;
                    rfoot_trajectory_float_pre = rfoot_trajectory_float_;
                    pelv_trajectory_float_pre = pelv_trajectory_float_;
                }

                if(momentumControlMode == false)
                {
                    computeIkControl_MJ(pelv_trajectory_float_, lfoot_trajectory_float_, rfoot_trajectory_float_, q_des_);
                    pelv_trajectory_support_.linear().setIdentity();
                    lfoot_trajectory_support_.linear().setIdentity();
                    rfoot_trajectory_support_.linear().setIdentity();
                    pelv_trajectory_float_ = DyrosMath::inverseIsometry3d(pelv_trajectory_support_) * pelv_trajectory_support_;
                    lfoot_trajectory_float_ = DyrosMath::inverseIsometry3d(pelv_trajectory_support_) * lfoot_trajectory_support_;
                    rfoot_trajectory_float_ = DyrosMath::inverseIsometry3d(pelv_trajectory_support_) * rfoot_trajectory_support_;
                    computeIkControl_MJ(pelv_trajectory_float_, lfoot_trajectory_float_, rfoot_trajectory_float_, q_des_1);
                }
                else
                {
                    if(time_tick_next == true)
                    {
                        if(walking_tick_stop == false || mpc_cycle <= 0)
                        {
                            ang_d_temp = ang_d/2000;
                            rfoot_d2.setZero();
                            lfoot_d2.setZero();
                            /*if(rfoot_trajectory_support_.translation()(2) > 0 && contactMode != 1)
                            {
                                rfoot_d2(2) = rfoot_d(2) + 0.0 * (rfoot_trajectory_support_.translation()(2) - rfoot_support_current_.translation()(2)) + F_F_input * 0.005/10;    
                                rfoot_d2(0) = rfoot_d(0) + 0.0 * (rfoot_trajectory_support_.translation()(0) - rfoot_support_current_.translation()(0));
                                rfoot_d2(1) = rfoot_d(1) + 0.0 * (rfoot_trajectory_support_.translation()(1) - rfoot_support_current_.translation()(1));
                            }
                            else if(lfoot_trajectory_support_.translation()(2) > 0 && contactMode != 1)
                            {
                                lfoot_d2(2) = lfoot_d(2) + 0.0 * (lfoot_trajectory_support_.translation()(2) - lfoot_support_current_.translation()(2)) - F_F_input * 0.005/10;
                                lfoot_d2(0) = lfoot_d(0) + 0.0 * (lfoot_trajectory_support_.translation()(0) - lfoot_support_current_.translation()(0));
                                lfoot_d2(1) = lfoot_d(1) + 0.0 * (lfoot_trajectory_support_.translation()(1) - lfoot_support_current_.translation()(1));
                            }
                            else
                            {
                                rfoot_d2(2) = rfoot_d(2) + 0.00 * (rfoot_trajectory_support_.translation()(2) - rfoot_support_current_.translation()(2)) + F_F_input * 0.005/10;    
                                lfoot_d2(2) = lfoot_d(2) + 0.00 * (lfoot_trajectory_support_.translation()(2) - lfoot_support_current_.translation()(2)) - F_F_input * 0.005/10;
                            }*/
                           
                            Eigen::Vector3d rpy_foot;
                            rpy_foot = DyrosMath::rot2Euler(rfoot_trajectory_float_.linear());
                            rfoot_ori1(0) = rfoot_ori(0) + 0.004 * (rpy_foot(0) - rfoot_rpy_current_(0));
                            rfoot_ori1(1) = rfoot_ori(1) + 0.004 * (rpy_foot(1) - rfoot_rpy_current_(1));
                           
                            rpy_foot = DyrosMath::rot2Euler(lfoot_trajectory_float_.linear());
                            lfoot_ori1(0) = lfoot_ori(0) + 0.004 * (rpy_foot(0) - lfoot_rpy_current_(0));
                            lfoot_ori1(1) = lfoot_ori(1) + 0. * (rpy_foot(1) - lfoot_rpy_current_(1));

                            momentumControl(rd_, com_d, ang_d_temp, rfoot_d, lfoot_d, upperd, rfoot_ori1, lfoot_ori1);
                           
                            q_dm_test.setZero();
                            q_dm_test.head(18) = q_dm.head(18) * 2000;
                            q_dm_test(19) = upperd(0);
                            q_dm_test(20) = upperd(1);
                           
                            pinocchio::computeCentroidalMomentum(model, model_data_test, q_pinocchio1, q_dm_test);
                            pinocchio::centerOfMass(model, model_data_test, q_pinocchio1);
                            pinocchio::updateFramePlacements(model,model_data_test);
                           
                            if(walking_tick == 0 && mpc_cycle > 0)
                            {
                                desired_q_not_compensated_1.head(12) = rd_.q_.head(12);
                            }
                            else if(mpc_cycle <= 0)
                            {
                                desired_q_not_compensated_1.head(12) = desired_q_not_compensated_.head(12);
                            }
                            else
                            {
                                desired_q_not_compensated_1.head(12) = q_des_.head(12);
                            }
                           
                            desired_q_not_compensated_.head(12) = desired_q_not_compensated_.head(12) + q_dm.segment<12>(6);
                           
                            if(rfoot_trajectory_support_.translation()(2) > 0 && contactMode != 1)
                            {
                                rfoot_d2(2) = rfoot_d(2) + 0.006 * (rfoot_trajectory_support_.translation()(2) - rfoot_support_current_.translation()(2)) + F_F_input * 0.005/10;    
                                rfoot_d2(0) = rfoot_d(0) + 0.003 * (rfoot_trajectory_support_.translation()(0) - rfoot_support_current_.translation()(0));
                                rfoot_d2(1) = rfoot_d(1) + 0.003 * (rfoot_trajectory_support_.translation()(1) - rfoot_support_current_.translation()(1));
                               
                                lfoot_d2(2) = lfoot_d(2) + 0.000 * (lfoot_trajectory_support_.translation()(2) - lfoot_support_current_.translation()(2)) - F_F_input * 0.005/10;
                            }
                            else if(lfoot_trajectory_support_.translation()(2) > 0 && contactMode != 1)
                            {
                                lfoot_d2(2) = lfoot_d(2) + 0.006 * (lfoot_trajectory_support_.translation()(2) - lfoot_support_current_.translation()(2)) - F_F_input * 0.005/10;
                                lfoot_d2(0) = lfoot_d(0) + 0.003 * (lfoot_trajectory_support_.translation()(0) - lfoot_support_current_.translation()(0));
                                lfoot_d2(1) = lfoot_d(1) + 0.003 * (lfoot_trajectory_support_.translation()(1) - lfoot_support_current_.translation()(1));
                               
                                rfoot_d2(2) = rfoot_d(2) + 0.00 * (rfoot_trajectory_support_.translation()(2) - rfoot_support_current_.translation()(2)) + F_F_input * 0.005/10;    
                            }
                            else
                            {
                                rfoot_d2(2) = rfoot_d(2) + 0.00 * (rfoot_trajectory_support_.translation()(2) - rfoot_support_current_.translation()(2)) + F_F_input * 0.005/10;    
                                lfoot_d2(2) = lfoot_d(2) + 0.00 * (lfoot_trajectory_support_.translation()(2) - lfoot_support_current_.translation()(2)) - F_F_input * 0.005/10;
                            }

                            com_d2(0) = com_d(0) + 0.00 * (com_desired_(0) - com_support_current_(0));
                            com_d2(1) = com_d(1) + 0.00 * (com_desired_(1) - com_support_current_(1));
                            com_d2(2) = 0.0 + 0.003 * (com_desired_(2) - com_support_current_(2));
                           
                            //ang_d_temp(0) = ang_d_temp(0) + 10 * (ang_d_temp(0) - model_data_cen.hg.angular()(0)/2000);
                            //ang_d_temp(1) = ang_d_temp(1) + 10 * (ang_d_temp(1) - model_data_cen.hg.angular()(1)/2000);
                           

                            momentumControl(rd_, com_d2, ang_d_temp, rfoot_d2, lfoot_d2, upperd, rfoot_ori1, lfoot_ori1);

                            if(mpc_cycle < 0)
                                q_des_.head(12) = desired_q_not_compensated_1.head(12) + q_dm.segment<12>(6);
                            else
                                q_des_.head(12) = desired_q_not_compensated_1.head(12) + q_dm.segment<12>(6);

                            q_des_.head(12) = desired_q_not_compensated_.head(12);
                        }
                    }
                }
                Compliant_control(q_des_);

                for (int i = 0; i < 12; i++)
                {
                    ref_q_(i) = DOB_IK_output_(i);
                }

                if (atb_grav_update_ == false)
                {
                    atb_grav_update_ = true;
                    Gravity_MJ_fast_ = Gravity_MJ_;
                    atb_grav_update_ = false;
                }
               
                if(walking_tick_mj == 1)
                {
                    for(int i = 0; i < 12; i++)
                        q_virtual_state(i+7) = q_des_1(i);
                   
                    qdd_des_lpf.setZero();
                    qdd_des_virtual_lpf.setZero();
                    qdd_des_virtual_ori_lpf.setZero();

                    pinocchio::forwardKinematics(model_state, model_data_state, q_virtual_state);
                    pinocchio::updateFramePlacement(model_state, model_data_state, RFcframe_id);
                    pinocchio::updateFramePlacement(model_state, model_data_state, LFcframe_id);
                   
                    RFc_vector_prev = model_data_state.oMf[RFcframe_id].translation();
                    LFc_vector_prev = model_data_state.oMf[LFcframe_id].translation();
                   
                    for(int i = 0; i < 12; i ++)
                        q_des_prev(i) = q_des_1(i);
                }
                if(walking_tick_mj >= 1)
                {
                    if(momentumControlMode == true)
                    {
                        qdd_des_virtual = (q_dm_test.head(3) - qd_des_virtual_prev.head(3))*2000;
                        qdd_des_virtual_ori = (q_dm_test.segment<3>(3) - qd_des_ori_prev) * 2000;
                        qdd_des_.head(12) = (q_dm_test.segment<12>(6) - qd_des_prev.head(12))*2000;
                        qdd_des_upper = (upperd - upperd_prev) * 2000;

                        if(step_change == false)
                        {
                            qdd_des_lpf =  1 / (1 + 0.1 * M_PI * 6.0 * del_t) * qdd_des_lpf + (0.1 * M_PI * 6.0 * del_t) / (1 + 0.1 * M_PI * 6.0 * del_t) * qdd_des_;
                            qdd_des_virtual_lpf =  1 / (1 + 0.1 * M_PI * 6.0 * del_t) * qdd_des_virtual_lpf + (0.1 * M_PI * 6.0 * del_t) / (1 + 0.1 * M_PI * 6.0 * del_t) * qdd_des_virtual;
                            qdd_des_virtual_ori_lpf = 1 / (1 + 0.1 * M_PI * 6.0 * del_t) * qdd_des_virtual_ori_lpf + (0.1 * M_PI * 6.0 * del_t) / (1 + 0.1 * M_PI * 6.0 * del_t) * qdd_des_virtual_ori;
                            qdd_des_upper_lpf = 1 / (1 + 0.1 * M_PI * 6.0 * del_t) * qdd_des_upper_lpf + (0.1 * M_PI * 6.0 * del_t) / (1 + 0.1 * M_PI * 6.0 * del_t) * qdd_des_upper;


                            qdd_des_virtual_.head(3) =  qdd_des_virtual_lpf.head(3);
                            qdd_des_virtual_.segment<3>(3) = qdd_des_virtual_ori_lpf.head(3);
                            qdd_des_virtual_.segment<12>(6) =  qdd_des_lpf.head(12);
                            qdd_des_virtual_.segment<2>(19) = qdd_des_upper_lpf;
                        }
                       
                        qd_des_virtual_prev.head(3) = q_dm_test.head(3);
                        qd_des_prev.head(12) = q_dm_test.segment<12>(6);
                        qd_des_ori_prev.head(3) = q_dm_test.segment<3>(3);
                        upperd_prev = upperd;
                    }
                    else
                    {
                        for(int i = 0; i < 12; i++)
                            q_virtual_state(i+7) = q_des_1(i);

                        pinocchio::forwardKinematics(model_state, model_data_state, q_virtual_state);
                        pinocchio::updateFramePlacement(model_state, model_data_state, RFcframe_id);
                        pinocchio::updateFramePlacement(model_state, model_data_state, LFcframe_id);
                   
                        RFc_vector = model_data_state.oMf[RFcframe_id].translation();
                        LFc_vector = model_data_state.oMf[LFcframe_id].translation();

                        if(foot_step_(current_step_num_, 6) == 0)
                        {
                            qd_des_virtual.head(3) = -(RFc_vector - RFc_vector_prev).head(3) * 2000;
                        }
                        else
                        {
                            qd_des_virtual.head(3) = -(LFc_vector - LFc_vector_prev).head(3) * 2000;
                        }

                        //qd_des_virtual.head(3) = (-(RFc_vector - RFc_vector_prev).head(3)-(LFc_vector - LFc_vector_prev).head(3))/2 * 2000;

                        qd_des_ = (q_des_1 - q_des_prev)*2000;

                        qd_des_lpf =  1 / (1 + 0.1 * M_PI * 6.0 * del_t) * qd_des_lpf + (0.1 * M_PI * 6.0 * del_t) / (1 + 0.1 * M_PI * 6.0 * del_t) * qd_des_;
                   
                        qd_des_virtual_lpf =  1 / (1 + 0.1 * M_PI * 6.0 * del_t) * qd_des_virtual_lpf + (0.1 * M_PI * 6.0 * del_t) / (1 + 0.1 * M_PI * 6.0 * del_t) * qd_des_virtual;
               

                        if(walking_tick_mj >= 2)
                        {
                            qdd_des_virtual = (qd_des_virtual_lpf-qd_des_virtual_prev) * 2000;
                            qdd_des_ = (qd_des_lpf - qd_des_prev) * 2000;
                        }


                        qdd_des_lpf =  1 / (1 + 0.1 * M_PI * 6.0 * del_t) * qdd_des_lpf + (0.1 * M_PI * 6.0 * del_t) / (1 + 0.1 * M_PI * 6.0 * del_t) * qdd_des_;
                        qdd_des_virtual_lpf =  1 / (1 + 0.1 * M_PI * 6.0 * del_t) * qdd_des_virtual_lpf + (0.1 * M_PI * 6.0 * del_t) / (1 + 0.1 * M_PI * 6.0 * del_t) * qdd_des_virtual;

                        qdd_des_virtual_.head(3) =  qdd_des_virtual_lpf;
                        qdd_des_virtual_.segment<12>(6) =  qdd_des_lpf;  
                    }

                    if (atb_qddot_update_ == false)
                    {
                        atb_qddot_update_ = true;
                        qdd_des_virtual_fast = qdd_des_virtual_;
                        atb_qddot_update_ = false;
                    }
   
                    if(momentumControlMode == false)
                    {
                        RFc_vector_prev = RFc_vector;
                        LFc_vector_prev = LFc_vector;
                       
                        qd_des_virtual_prev = qd_des_virtual_lpf;

                        for(int i = 0; i < 12; i ++)
                            q_des_prev(i) = q_des_1(i);
                        for(int i = 0; i < 12; i ++)
                            qd_des_prev(i) = qd_des_lpf(i);
                    }
                }
               
                if(mpc_cycle < controlwalk_time-1)
                {
                   
                    file[1] <<walking_tick_mj << " "<<mpc_cycle <<" "<<buffer[0] << " " <<mpc_start_init_ <<" " <<mpc_start_init_bool<< " " <<mpc_start_init_bool1<<" " <<mpc_start_init_bool2 <<" "<<mpc_start_init_bool3 <<" " <<mpc_start_init_bool3 << " "<<virtual_temp(0) << " " <<virtual_temp(1)<< " " <<virtual_temp1(0) << " " <<virtual_temp1(1) << " " <<contactMode << " " << com_mpcx << " " << com_mpcy << " "<<desired_val_slow[41] << " " << desired_val_slow[45] << " ";
                   
                    file[1] << "12  " << zmpy_d << " " << zmpx_d << " "  << zmpy_ << " " << zmp_measured_mj_(1)  << " "<<zmpx_ << " " <<  zmp_measured_mj_(0) << " " << (1-com_alpha_fast) * rd_.link_[COM_id].mass * GRAVITY * 1.0 << " " << (com_alpha_fast) * rd_.link_[COM_id].mass * GRAVITY * 1.0 << " ";
                   

                    file[1] << "123  ";
                    file[1] << com_desired_(0) << " " << com_support_current_(0) << " "<< com_desired_(1) << " " << com_support_current_(1) << " " << com_mpc1[1] << " "<< com_float_current_(1) << " " << com_float_current_(2) << " ";

                   

                    file[1] << "56 ";

                    for(int i = 0; i < 12; i++)
                    {
                        file[1] << qp_result(i) << " ";
                    }
                   
                    file[1] << K_ << std::endl;
                    /*file[1] << "213 " << qp_solved << " " << model_data_cen.hg.angular()(0) << " " << model_data_cen.hg.angular()(1)<< " "<< ang_d(0) << " " << ang_d(1);
                    file[1] << " 55 "  << ZMP_Y_REF << " " << zmp_measured_mj_(1) << " " <<ZMPy_test << " "<<state_init_[47]<< " " << desired_val_slow[47] << " " ;//<< std::endl;//<<cp_desired_(1) << " " << cp_desired_(0) << std::endl;
                    file[1] << " 66 "  << ZMP_X_REF << " " << zmp_measured_mj_(0) << " " <<ZMPx_test << " "<<state_init_[43]<< " " << desired_val_slow[43] << " " ;//<< std::endl;//<<cp_desired_(1) << " " << cp_desired_(0) << std::endl;
                   
                    file[1] << " 77 " << com_vel_current_(1) << " " <<desired_val_slow[46]<<  " " << com_float_current_(1) << " "<< desired_val_slow[45]-virtual_temp1(1)<< " "<<com_mpc1[1] << " " << com_desired_(1) << " " << com_support_current_(1) << " " << virtual_temp1(0) << " " << virtual_temp1(1)<<std::endl;
                    file[0] << rfoot_trajectory_support_.translation()(0) << " " << rfoot_trajectory_support_.translation()(1) << " " <<rfoot_trajectory_support_.translation()(2) << " " << rfoot_support_current_.translation()(0) << " " << rfoot_support_current_.translation()(1) << " " <<rfoot_support_current_.translation()(2) << " " << rd_.link_[Left_Foot].xipos(2) << " " << rd_.link_[Right_Foot].xipos(2) << std::endl;
                    */
                    /*
                    file[1] << mpc_cycle << " " << contactMode << " " << virtual_temp(0) << " " << virtual_temp(1) << " "  << virtual_temp1(0) << " " << virtual_temp1(1) << " " << rd_.link_[Left_Foot].xipos(2) << " " << rd_.link_[Right_Foot].xipos(2) << " "<< rd_.link_[Left_Foot].xipos(0) << " " << rd_.link_[Right_Foot].xipos(0) << " "<< rd_.link_[Left_Foot].xipos(1) << " " << rd_.link_[Right_Foot].xipos(1) << " " << rfoot_ori(0)<< " " << rfoot_ori(1)<< " " << rfoot_ori1(0)<< " " << rfoot_ori1(1);
                   
                    file[1] << " 6 ";
                   
                    file[1] << ZMP_Y_REF << " " << zmp_measured_mj_(1) << " " <<ZMPy_test<< " "<< ZMP_X_REF << " " << zmp_measured_mj_(0) << " " << rfoot_rpy_current_(0)  << " " << rfoot_rpy_current_(1) << " " << F_T_R_x_input <<" "<< F_T_R_y_input<< " "<< lfoot_rpy_current_(0)  << " " << lfoot_rpy_current_(1) << " " << F_T_L_x_input <<" "<< F_T_L_y_input << std::endl;
                    */
                }
                lfoot_trajectory_float_pre = lfoot_trajectory_float_;
                rfoot_trajectory_float_pre = rfoot_trajectory_float_;
                pelv_trajectory_float_pre = pelv_trajectory_float_;
               

                if(hipcompen == true)
                    hip_compensator();
               
                if(torque_control == false)
                {
                    CP_compen_MJ();
                    CP_compen_MJ_FT();
                }
                else
                {
                    CP_compen_MJ();
                    CP_compen_MJ_FT();
                }

                torque_lower_.setZero();

                if(torque_control == false)
                {
                    for (int i = 0; i < 12; i++)
                    {
                        torque_lower_(i) = (Kp(i) * (ref_q_(i) - rd_.q_(i)) + Kd(i) * (q_dm(i+6) * 2000 - rd_.q_dot_(i))) + Gravity_MJ_fast_(i) + Tau_CP(i);
                    }
                }
                else
                {
                    for (int i = 0; i < 12; i++)
                    {
                        torque_lower_(i) = (Kp(i) * (ref_q_(i) - rd_.q_(i)) + Kd(i) * (q_dm(i+6) * 2000 - rd_.q_dot_(i))) + Gravity_MJ_fast_(i) + Tau_CP(i);
                    }
                }

                if(time_tick_next == true)
                {
                    updateNextStepTime();
                    //time_tick_next = false;
                }

                if(walking_tick_mj == t_start_)
                    step_change = true;
                else
                    step_change = false;

                q_prev_MJ_ = rd_.q_;
            }
            else
            {
                walking_enable_ = false;
            }
        }
        else
        {
            if (walking_end_flag == 0)
            {
                cout << "walking finish" << endl;
                walking_end_flag = 1;
                initial_flag = 0;
            }

            if (atb_grav_update_ == false)
            {
                atb_grav_update_ = true;
                Gravity_MJ_fast_ = Gravity_MJ_;
                atb_grav_update_ = false;
            }

            torque_lower_.setZero();
            for (int i = 0; i < 12; i++)
            {
                torque_lower_(i) = Kp(i) * (ref_q_(i) - rd_.q_(i)) - Kd(i) * rd_.q_dot_(i) + 1.0 * Gravity_MJ_fast_(i);
            }
        }
        /////////////////////////////////////////////////////////////////////////////////////////
        if (atb_desired_q_update_ == false)
        {
            atb_desired_q_update_ = true;
            desired_q_fast_ = desired_q_slow_;
            desired_q_dot_fast_ = desired_q_dot_slow_;
            atb_desired_q_update_ = false;
        }

        if(momentumControlMode == true)
        {
            desired_q_fast_(13) = q_upper(0);
            desired_q_fast_(14) = q_upper(1);
            desired_q_dot_fast_(13) = upperd(0);
            desired_q_dot_fast_(14) = upperd(1);
        }

        torque_upper_.setZero();
        for (int i = 12; i < MODEL_DOF; i++)
        {
            torque_upper_(i) = (kp_joint_(i) * (desired_q_fast_(i) - rd_.q_(i)) + kv_joint_(i) * (desired_q_dot_fast_(i) - rd_.q_dot_(i)) + 1.0 * Gravity_MJ_fast_(i));
        }

        rd_.torque_desired = torque_lower_ + torque_upper_;
    }
}

void CustomController::computeFast()
{
    if (rd_.tc_.mode == 6)
    {
        if (initial_flag == 1)
        {
            WBC::SetContact(rd_, 1, 1);

            if (atb_grav_update_ == false)
            {
                VectorQd Gravity_MJ_local = WBC::ContactForceRedistributionTorqueWalking(rd_, WBC::GravityCompensationTorque(rd_), 0.9, 1, 0);

                atb_grav_update_ = true;
                Gravity_MJ_ = Gravity_MJ_local;
                atb_grav_update_ = false;
                //cout<<"comutefast tc.mode =10 is initialized"<<endl;
            }
            //initial_flag = 2;
        }
    }
    else if (rd_.tc_.mode == 7)
    {
        ////////////////////////////////////////////////////////////////////////////
        /////////////////// Biped Walking Controller made by MJ ////////////////////
        ////////////////////////////////////////////////////////////////////////////
        if (walking_enable_ == true)
        {
            if (current_step_num_ < total_step_num_)
            {
                getRobotData();
                if(torque_control == false)
                    GravityCalculate_MJ();
                else
                {
                    if (atb_qddot_update_ == false)
                    {
                        atb_qddot_update_ = true;
                        qdd_pinocchio_desired1_ = qdd_des_virtual_fast;
                        atb_qddot_update_ = false;
                    }
                    if(torquecontrol_Ok == false)
                        GravityCalculate_MJ();
                    else
                    {
                        setContact_custom();
                       
                        X1.setZero();
                        J1.setZero();
                        H1.setZero();
                        A1.setZero();
                        g1.setZero();
                        lbA1.setZero();
                        ubA1.setZero();
                        lb1.setZero();
                        ub1.setZero();

                        RigidBodyDynamics::NonlinearEffects(model_c_, rd_.q_virtual_, rd_.q_dot_virtual_, nonlineareff);

                        if(true)//torquecontrol_first == true || (torquecontrol_first == false && walking_tick_jk >= 300))
                        {
                            if(contactMode_fast == 1)
                            {    
                                double lx, ly, mu;
                                ly = 0.048;
                                lx = 0.11;
                                mu = 0.8;

                                if(lfoot_sx >  rfoot_sx)
                                {
                                    if(zmpx_fast > lfoot_sx + lx)
                                        zmpx_d  = lfoot_sx + lx;
                                    else if(zmpx_fast < rfoot_sx - lx)
                                        zmpx_d  = rfoot_sx - lx;
                                    else
                                        zmpx_d = zmpx_fast;

                                    zmp_bx(0) =  lfoot_sy + ly;
                                    zmp_bx(1) =  rfoot_sy - ly;
                                }
                                else if(lfoot_sx <  rfoot_sx)
                                {
                                    if(zmpx_fast > rfoot_sx + lx)
                                        zmpx_d  = rfoot_sx + lx;
                                    else if(zmpx_fast < lfoot_sx - lx)
                                        zmpx_d  = lfoot_sx - lx;
                                    else
                                        zmpx_d = zmpx_fast;
                                    zmp_bx(0) =  lfoot_sy + ly;
                                    zmp_bx(1) =  rfoot_sy - ly;
                                }
                                else
                                {
                                    if(zmpx_fast > rfoot_sx + lx)
                                        zmpx_d  = rfoot_sx + lx;
                                    else if(zmpx_fast < lfoot_sx - lx)
                                        zmpx_d  = lfoot_sx - lx;
                                    else
                                        zmpx_d = zmpx_fast;
                                    zmp_bx(0) =  lfoot_sy + ly;
                                    zmp_bx(1) =  rfoot_sy - ly;
                                }

                                if(zmpy_fast > lfoot_sy + ly)
                                    zmpy_d = lfoot_sy + ly;
                                else
                                    zmpy_d = zmpy_fast;

                                if(zmpy_fast < rfoot_sy - ly)
                                    zmpy_d = rfoot_sy - ly;
                                else
                                    zmpy_d = zmpy_fast;
                       
                                ly = 0.05;
                                lx = 0.13;
                       
                                M_ = rd_.A_;
                                nle = nonlineareff;

                                H1.setIdentity();
                       
                                g1.setZero();
                                H1.block(12,12,MODEL_DOF_VIRTUAL,MODEL_DOF_VIRTUAL) = 1000000.0 * H1.block(12,12,MODEL_DOF_VIRTUAL,MODEL_DOF_VIRTUAL);
                           
                                lb1.setConstant(variable_size1, -100000);
                                ub1.setConstant(variable_size1, 100000);

                                //wpwkfl
                                H1(2,2) = 1000.0;//0.1;
                                H1(8,8) = 1000.0;//0.1;
                                g1(2) = - (1-com_alpha_fast) * rd_.link_[COM_id].mass * GRAVITY * 1000.0;
                                g1(8) = - (com_alpha_fast) * rd_.link_[COM_id].mass * GRAVITY * 1000.0;
                                lb1(2) = 0.0;
                                lb1(8) = 0.0;

                                lb1(15) = 0.0;
                                lb1(16) = 0.0;
                                ub1(15) = 0.0;
                                ub1(16) = 0.0;

                                lb1(17) = 0.0;
                                ub1(17) = 0.0;
                           
                                H1_temp.setIdentity();

                                H1.block(12,12,6,6) = 1 * H1.block(12,12,6,6);
                               
                                if(torquecontrol_first)
                                {
                                    H1_temp = 1 * H1_temp;
                                    g1(2) = - (1-com_alpha_fast) * rd_.link_[COM_id].mass * GRAVITY * 10.0;
                                    g1(8) = - (com_alpha_fast) * rd_.link_[COM_id].mass * GRAVITY * 10.0;
                                    H1 = H1 + H1_temp;
                                    qp_result(2) = (1-com_alpha_fast) * rd_.link_[COM_id].mass * GRAVITY * 1.0;
                                    qp_result(8) = (com_alpha_fast) * rd_.link_[COM_id].mass * GRAVITY * 1.0;;
                                    g1_temp = -qp_result * 1.0;
                                    g1 = g1 + g1_temp;
                                    torquecontrol_first = false;
                                }
                                else
                                {
                                    H1_temp.setZero();
                                    double weight_s = 0.01;
                                    //Experimental gain 0.03
                                   
                                    //H1_temp(2,2) = 1.0;
                                    //H1_temp(8,8) = 1.0;

                                    H1_temp.block(0,0,12,12).setIdentity();
                                    H1_temp(2,2) = 0.0;
                                    H1_temp(8,8) = 0.0;

                                    H1_temp.block(12+MODEL_DOF_VIRTUAL,12+MODEL_DOF_VIRTUAL,2,2).setIdentity();
                                    H1_temp = 60.0 * H1_temp;
                                    H1_temp.block(12+MODEL_DOF_VIRTUAL,12+MODEL_DOF_VIRTUAL,2,2) = weight_s * H1_temp.block(12+MODEL_DOF_VIRTUAL,12+MODEL_DOF_VIRTUAL,2,2);
                                    H1_temp.block(12+MODEL_DOF_VIRTUAL,12+MODEL_DOF_VIRTUAL,2,2) = 60 * H1_temp.block(12+MODEL_DOF_VIRTUAL,12+MODEL_DOF_VIRTUAL,2,2);
                           
                                    H1 = H1 + H1_temp;
                                    g1_temp.setZero();
                                    g1_temp.head(12) = -qp_result.head(12) * 60.0;
                                    g1_temp(2) = 0.0;
                                    g1_temp(8) = 0.0;
                                    //g1_temp(2) = -qp_result(2) * 60.0;
                                    //g1_temp(8) = -qp_result(8) * 60.0;
                                    g1_temp.tail(2) = -qp_result.tail(2) * 60.0 * weight_s;
                                    g1 = g1 + g1_temp;
                                }

                                lbA1.head(6) = (nle + M_ * qdd_pinocchio_desired1_).head(6);
                                ubA1.head(6) = (nle + M_ * qdd_pinocchio_desired1_).head(6);
                           
                                double weight_resi = 0.0;
                                G_temp.setZero();
                                g_temp.setZero();
                                G_temp.block(0,0,6,MODEL_DOF_VIRTUAL) = RFj;
                                G_temp.block(6,0,6,MODEL_DOF_VIRTUAL) = LFj;

                                g_temp.tail(12) <<  RFdj * rd_.q_dot_virtual_ + RFj * qdd_pinocchio_desired1_, LFdj * rd_.q_dot_virtual_ + LFj * qdd_pinocchio_desired1_;
                                g1.tail(MODEL_DOF_VIRTUAL) = g1.tail(MODEL_DOF_VIRTUAL) + weight_resi * G_temp.transpose() * g_temp;
                                H1.block(12, 12, MODEL_DOF_VIRTUAL, MODEL_DOF_VIRTUAL) = H1.block(12, 12, MODEL_DOF_VIRTUAL, MODEL_DOF_VIRTUAL) + weight_resi * G_temp.transpose() * G_temp;
                   
                                //H1(12+MODEL_DOF_VIRTUAL,12+MODEL_DOF_VIRTUAL) = 100.0;//1.0;//0.0000001;

                                //H1(13+MODEL_DOF_VIRTUAL,13+MODEL_DOF_VIRTUAL) = 100.0;//1.0;//0.00000001;

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
                   
                                A1.block(22,0,1,6)(0,2) = (rfoot_sy  - zmpy_d);
                                A1.block(22,0,1,6)(0,3) = 1;
                                A1.block(22,6,1,6)(0,2) = (lfoot_sy  - zmpy_d);
                                A1.block(22,6,1,6)(0,3) = 1;
                                lbA1(22) = 0.0;
                                ubA1(22) = 0.0;

                                A1(22,MODEL_DOF_VIRTUAL+12) = 1.0;

                                A1.block(23,0,1,6)(0,2) = (rfoot_sx  - zmpx_d);
                                A1.block(23,0,1,6)(0,4) = -1;
                                A1.block(23,6,1,6)(0,2) = (lfoot_sx  - zmpx_d);
                                A1.block(23,6,1,6)(0,4) = -1;
                                lbA1(23) = 0.0;
                                ubA1(23) = 0.0;

                                A1(23,MODEL_DOF_VIRTUAL+13) = 1.0;

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
                                lbA1(57) = 0.1;
                                ubA1(57) = 1000.0;

                                A1(58, 8) = 1.0;
                                lbA1(58) = 0.1;
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

                                qp_torque_control.UpdateMinProblem(H1, g1);
                                qp_torque_control.UpdateSubjectToAx(A1, lbA1, ubA1);
                                qp_torque_control.UpdateSubjectToX(lb1, ub1);
                                solved = qp_torque_control.SolveQPoases(100, qp_result);
                           
                                if (solved == true)
                                {
                                    tau_ = ((M_ * (qdd_pinocchio_desired1_ + qp_result.segment<MODEL_DOF_VIRTUAL>(12))+  nle - (RFj.transpose() * qp_result.head(6) + LFj.transpose() * qp_result.segment<6>(6))).transpose());  
                                }
                                control_time = control_time + 1;
                            }
                            else if(contactMode_fast == 2)
                            {
                                double lx, ly, mu;
                                ly = 0.048;
                                lx = 0.11;
                                mu = 0.8;

                                if(zmpx_fast > lfoot_sx + lx)
                                    zmpx_d  = lfoot_sx + lx;
                                else if(zmpx_fast < lfoot_sx - lx)
                                    zmpx_d  = lfoot_sx - lx;
                                else
                                    zmpx_d = zmpx_fast;

                                if(zmpy_fast > lfoot_sy + ly)
                                    zmpy_d = lfoot_sy + ly;
                                else if(zmpy_fast < lfoot_sy - ly)
                                    zmpy_d = lfoot_sy - ly;
                                else
                                    zmpy_d = zmpy_fast;

                                zmp_bx(0) =  lfoot_sy + ly;
                                zmp_bx(1) =  lfoot_sy - ly;

                                ly = 0.05;
                                lx = 0.13;
                   
                                M_ = rd_.A_;//rd_.A_;
                                nle = nonlineareff;;

                                H1.setIdentity();

                                g1.setZero();
                                H1.block(12,12,MODEL_DOF_VIRTUAL,MODEL_DOF_VIRTUAL) = 1000000*H1.block(12,12,MODEL_DOF_VIRTUAL,MODEL_DOF_VIRTUAL);
                                lb1.setConstant(variable_size1, -100000);
                                ub1.setConstant(variable_size1, 100000);
                   
                                H1(8,8) = 5;
                                g1(8) = - (com_alpha_fast) * rd_.link_[COM_id].mass * GRAVITY * 5.0;
                                //g1(8) = - com_alpha_fast * rd_.link_[COM_id].mass * GRAVITY * 5;
                                lb1(2) = 0.0;
                                lb1(8) = 0.0;
                                lb1(15) = 0.0;
                                lb1(16) = 0.0;
                                ub1(15) = 0.0;
                                ub1(16) = 0.0;

                                lb1(17) = 0.0;
                                ub1(17) = 0.0;


                                H1.block(12,12,6,6) = 1 * H1.block(12,12,6,6);//10.0 * H1.block(12,12,6,6);revise
                                //H1.block(12,12,MODEL_DOF_VIRTUAL-6,MODEL_DOF_VIRTUAL-6) = 10 * H1.block(18,18,MODEL_DOF_VIRTUAL-6,MODEL_DOF_VIRTUAL-6);

                                H1_temp.setZero();
                                double weight_s = 10000.0;
                                H1_temp(2,2) = 1.0;
                                H1_temp(8,8) = 1.0;
                                H1_temp.block(12+MODEL_DOF_VIRTUAL,12+MODEL_DOF_VIRTUAL,2,2).setIdentity();
                                H1_temp = 0.03 * H1_temp;
                                H1_temp.block(12+MODEL_DOF_VIRTUAL,12+MODEL_DOF_VIRTUAL,2,2) = weight_s * H1_temp.block(12+MODEL_DOF_VIRTUAL,12+MODEL_DOF_VIRTUAL,2,2);
                                H1_temp.block(12+MODEL_DOF_VIRTUAL,12+MODEL_DOF_VIRTUAL,2,2) = 10 * H1_temp.block(12+MODEL_DOF_VIRTUAL,12+MODEL_DOF_VIRTUAL,2,2);
                                H1 = H1 + H1_temp;
                                g1_temp.setZero();
                                g1_temp(2) = -qp_result(2) * 0.03;
                                g1_temp(8) = -qp_result(8) * 0.03;
                                g1_temp.tail(2) = -qp_result.tail(2) * 0.03 * weight_s;
                                g1 = g1 + g1_temp;
                   
                                double weight_resi = 0.0;
                                G_temp.setZero();
                                g_temp.setZero();
                                G_temp.block(0,0,6,MODEL_DOF_VIRTUAL) = LFj;
                                g_temp.head(6) <<  LFdj * rd_.q_dot_virtual_ + LFj * qdd_pinocchio_desired1_;
       
                                g1.tail(MODEL_DOF_VIRTUAL) = g1.tail(MODEL_DOF_VIRTUAL) + weight_resi * G_temp.transpose() * g_temp;
                                H1.block(12, 12, MODEL_DOF_VIRTUAL, MODEL_DOF_VIRTUAL) = H1.block(12, 12, MODEL_DOF_VIRTUAL, MODEL_DOF_VIRTUAL) + weight_resi * G_temp.transpose() * G_temp;
               
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
               
                                A1.block(22,6,1,6)(0,2) = (lfoot_sy  - zmpy_d);
                                A1.block(22,6,1,6)(0,3) = 1;

                                lbA1(22) = 0.0;
                                ubA1(22) = 0.0;

                                A1(22,MODEL_DOF_VIRTUAL+12) = 1.0;
                   
                                A1.block(23,6,1,6)(0,2) = (lfoot_sx  - zmpx_d);
                                A1.block(23,6,1,6)(0,4) = -1;
           
                                lbA1(23) = 0.0;
                                ubA1(23) = 0.0;

                                A1(23,MODEL_DOF_VIRTUAL+13) = 1.0;
                   
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
                           
                                qp_torque_control.UpdateMinProblem(H1, g1);
                                qp_torque_control.UpdateSubjectToAx(A1, lbA1, ubA1);
                                qp_torque_control.UpdateSubjectToX(lb1, ub1);
                                solved = qp_torque_control.SolveQPoases(100, qp_result);
                                if (solved == true)
                                {
                                    tau_ = ((M_ * (qdd_pinocchio_desired1_ + qp_result.segment<MODEL_DOF_VIRTUAL>(12))+  nle - (LFj.transpose() * qp_result.segment<6>(6))).transpose());
                                }
                                control_time = control_time + 1;
                            }
                            else if(contactMode_fast == 3)//|| mpc_cycle == 49)
                            {
                                double lx, ly, mu;
                                ly = 0.048;
                                lx = 0.11;
                                mu = 0.8;

                                if(zmpx_fast > rfoot_sx + lx)
                                    zmpx_d  = rfoot_sx + lx;
                                else if(zmpx_fast < rfoot_sx - lx)
                                    zmpx_d  = rfoot_sx - lx;
                                else
                                    zmpx_d = zmpx_fast;

                                if(zmpy_fast < rfoot_sy - ly)
                                    zmpy_d = rfoot_sy - ly;
                                else if(zmpy_fast > rfoot_sy + ly)
                                    zmpy_d = rfoot_sy + ly;
                                else
                                    zmpy_d = zmpy_fast;

                                zmp_bx(0) =  rfoot_sy + ly;
                                zmp_bx(1) =  rfoot_sy - ly;

                                ly = 0.05;
                                lx = 0.13;
                   
                                M_ = rd_.A_;//= rd_.A_;
                                nle = nonlineareff;

                                H1.setIdentity();

                                g1.setZero();
                                H1.block(12,12,MODEL_DOF_VIRTUAL,MODEL_DOF_VIRTUAL) = 1000000*H1.block(12,12,MODEL_DOF_VIRTUAL,MODEL_DOF_VIRTUAL);
                                lb1.setConstant(variable_size1, -100000);
                                ub1.setConstant(variable_size1, 100000);

                                //H1(12+MODEL_DOF_VIRTUAL,12+MODEL_DOF_VIRTUAL) = 100.0;//0.001;
                           
                                //H1(13+MODEL_DOF_VIRTUAL,13+MODEL_DOF_VIRTUAL) = 100.0;//0.0001;

                                H1(2,2) = 5;
                                g1(2) = - (1-com_alpha_fast) * rd_.link_[COM_id].mass * GRAVITY * 5.0;
                           
                                //g1(2) = - com_alpha_fast * rd_.link_[COM_id].mass * GRAVITY * 5;
                                lb1(2) = 0.0;
                                lb1(8) = 0.0;
                                lb1(15) = 0.0;
                                lb1(16) = 0.0;
                                ub1(15) = 0.0;
                                ub1(16) = 0.0;
                                lb1(17) = 0.0;
                                ub1(17) = 0.0;

                                H1.block(12,12,6,6) = 1 * H1.block(12,12,6,6);//revise
                           
                                /*H1_temp.setZero();
                                H1_temp.block(0,0,12,12).setIdentity();
                                H1_temp = 0.0003 * H1_temp;
                                H1 = H1 + H1_temp;
                                g1_temp.setZero();
                                g1_temp.head(12) = -qp_result.head(12) * 0.0003;
                                g1 = g1 + g1_temp;*/
                                H1_temp.setZero();
                                double weight_s = 10000.0;
                                H1_temp(2,2) = 1.0;
                                H1_temp(8,8) = 1.0;
                                H1_temp.block(12+MODEL_DOF_VIRTUAL,12+MODEL_DOF_VIRTUAL,2,2).setIdentity();
                                H1_temp = 0.03 * H1_temp;
                                H1_temp.block(12+MODEL_DOF_VIRTUAL,12+MODEL_DOF_VIRTUAL,2,2) = weight_s * H1_temp.block(12+MODEL_DOF_VIRTUAL,12+MODEL_DOF_VIRTUAL,2,2);
                                H1_temp.block(12+MODEL_DOF_VIRTUAL,12+MODEL_DOF_VIRTUAL,2,2) = 10 * H1_temp.block(12+MODEL_DOF_VIRTUAL,12+MODEL_DOF_VIRTUAL,2,2);                          
                                H1 = H1 + H1_temp;
                                g1_temp.setZero();
                                g1_temp(2) = -qp_result(2) * 0.03;
                                g1_temp(8) = -qp_result(8) * 0.03;
                                g1_temp.tail(2) = -qp_result.tail(2) * 0.03 * weight_s;
                                g1 = g1 + g1_temp;
                   
                                double weight_resi = 0.0;
                                G_temp.setZero();
                                g_temp.setZero();
                                G_temp.block(0,0,6,MODEL_DOF_VIRTUAL) = RFj;
                                g_temp.head(6) <<  RFdj * rd_.q_dot_virtual_ + RFj * qdd_pinocchio_desired1_;
                           
                                g1.tail(MODEL_DOF_VIRTUAL) = g1.tail(MODEL_DOF_VIRTUAL) + weight_resi * G_temp.transpose() * g_temp;
                                H1.block(12, 12, MODEL_DOF_VIRTUAL, MODEL_DOF_VIRTUAL) = H1.block(12, 12, MODEL_DOF_VIRTUAL, MODEL_DOF_VIRTUAL) + weight_resi * G_temp.transpose() * G_temp;
               
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
                           
                                A1.block(22,0,1,6)(0,2) = (rfoot_sy - zmpy_d);
                                A1.block(22,0,1,6)(0,3) = 1;

                                lbA1(22) = 0.0;
                                ubA1(22) = 0.0;

                                A1(22,MODEL_DOF_VIRTUAL+12) = 1.0;
                   
                                A1.block(23,0,1,6)(0,2) = (rfoot_sx - zmpx_d);
                                A1.block(23,0,1,6)(0,4) = -1;
           
                                lbA1(23) = 0.0;
                                ubA1(23) = 0.0;

                                A1(23,MODEL_DOF_VIRTUAL+13) = 1.0;
                           
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
                           
                                qp_torque_control.UpdateMinProblem(H1, g1);
                                qp_torque_control.UpdateSubjectToAx(A1, lbA1, ubA1);
                                qp_torque_control.UpdateSubjectToX(lb1, ub1);
                                solved = qp_torque_control.SolveQPoases(100, qp_result);
                                if (solved == true)
                                {
                                    tau_ = ((M_ * (qdd_pinocchio_desired1_ + qp_result.segment<MODEL_DOF_VIRTUAL>(12))+  nle - (RFj.transpose() * qp_result.head(6))).transpose());
                                }
                                control_time = control_time + 1;
                            }
                        }

                        /*if(torquecontrol_first == false && walking_tick_jk == 0)
                        {
                            tau_1 = tau_;
                        }*/
                       
                        if (atb_grav_update_ == false)
                        {
                            atb_grav_update_ = true;
                            if(torquecontrol_first == false && walking_tick_jk < 200)
                            {
                                Gravity_MJ_ =  1 / (1 + 2 * M_PI * 4.0 * del_t) * Gravity_MJ_ + (2 * M_PI * 4.0 * del_t) / (1 + 2 * M_PI * 4.0 * del_t) * tau_.segment<MODEL_DOF_VIRTUAL-6>(6);
                                walking_tick_jk = walking_tick_jk + 1;
                            }
                            else
                                Gravity_MJ_ = tau_.segment<MODEL_DOF_VIRTUAL-6>(6);
                            atb_grav_update_ = false;
                        }
                    }
                }  
            }
        }
        else
        {
            WBC::SetContact(rd_, 1, 1);
            int support_foot;
            if (foot_step_(current_step_num_, 6) == 1)
            {
                support_foot = 1;
            }
            else
            {
                support_foot = 0;
            }
           
            //del_ang_momentum_slow_ = 0.8*del_ang_momentum_slow_;    // Decrease to zero exponencially

            if (atb_grav_update_ == false)
            {
                VectorQd Gravity_MJ_local = WBC::ContactForceRedistributionTorqueWalking(rd_, WBC::GravityCompensationTorque(rd_), 0.9, 1, support_foot);
                           
                atb_grav_update_ = true;
                Gravity_MJ_ =  1 / (1 + 2 * M_PI * 4.0 * del_t) * Gravity_MJ_ + (2 * M_PI * 4.0 * del_t) / (1 + 2 * M_PI * 4.0 * del_t) * Gravity_MJ_local;//tau_.segment<MODEL_DOF_VIRTUAL-6>(6);
                //Gravity_MJ_ = Gravity_MJ_local;
                atb_grav_update_ = false;
            }
        }
        /////////////////////////////////////////////////////////////////////////////////////////

        if (rd_.tc_init == true)
        {
            initWalkingParameter();
            rd_.tc_init = false;
        }

        //data process//
        //walkingStateManager(); //avatar
        getProcessedRobotData();

        //motion planing and control//
        motionGenerator();
        //STEP3: Compute q_dot for CAM control
        //computeCAMcontrol_HQP();

        for (int i = 12; i < MODEL_DOF; i++)
        {
            desired_q_(i) = motion_q_(i);
            desired_q_dot_(i) = motion_q_dot_(i);
            // desired_q_dot_(i) = 0;
        }

        //STEP4: send desired q to the fast thread
        if (atb_desired_q_update_ == false)
        {
            atb_desired_q_update_ = true;
            desired_q_slow_ = desired_q_;
            desired_q_dot_slow_ = desired_q_dot_;
            atb_desired_q_update_ = false;
        }

        savePreData();

        // printOutTextFile();
    }
}

void CustomController::initWalkingParameter()
{
    walking_mode_on_ = true;
    program_ready_duration_ = 0;
    walking_control_transition_duration_ = 0.1;
    upper_body_mode_ = 3;
    stop_vel_threshold_ = 0.20;
    walking_duration_cmd_ = 1.3;
    dsp_duration_ = 0.6;
    dsp_ratio_ = dsp_duration_ / walking_duration_cmd_;
    turning_duration_ = (walking_duration_cmd_ - dsp_duration_) * 0.8;
    walking_phase_ = 0;
    turning_phase_ = 0;
    walking_speed_ = 0.00;
    // walking_speed_ = 0.05/1.3; //5cm walking speed
    // walking_speed_ = 0.10/1.3; //5cm walking speed
    walking_speed_side_ = 0.0;
    // knee_target_angle_ = 18*DEG2RAD;
    knee_target_angle_ = 0.6; //4.5degree
    com_target_height_ = 0.71;

    swingfoot_highest_time_ = (1 - dsp_ratio_) / 2 + dsp_ratio_;
    ankle2footcenter_offset_ = 0.02;
    yaw_angular_vel_ = 0; //   rad/s
    swing_foot_height_ = 0.05;
    switching_phase_duration_ = 0.05;
    foot_contact_ = -1;
    foot_contact_pre_ = foot_contact_;
    step_width_ = 0.22; //for preview control
    alpha_x_ = 0.01;
    alpha_y_ = 0.18;
    alpha_x_command_ = alpha_x_;
    alpha_y_command_ = alpha_y_;

    start_walking_trigger_ = false;
    first_step_trigger_ = false;
    foot_swing_trigger_ = false;
    stop_walking_trigger_ = true;
    falling_detection_flag_ = false;

    upperbody_mode_recieved_ = true;

    preview_horizon_ = 1.6; //seconds
    preview_hz_ = 2000;
    zmp_size_ = preview_horizon_ * preview_hz_;
    ref_zmp_.setZero(zmp_size_, 2);
    zmp_y_offset_ = -0.04; //outward from com

    walking_duration_start_delay_ = preview_horizon_;
    max_stop_walking_num_ = int(preview_horizon_ / walking_duration_cmd_) + 1;
    stop_walking_counter_ = 0;

    jac_rhand_.setZero(6, MODEL_DOF_VIRTUAL);
    jac_lhand_.setZero(6, MODEL_DOF_VIRTUAL);
    jac_rfoot_.setZero(6, MODEL_DOF_VIRTUAL);
    jac_lfoot_.setZero(6, MODEL_DOF_VIRTUAL);

    com_pos_error_.setZero();
    com_vel_error_.setZero();
    //set init pre data
    com_pos_desired_pre_ = rd_.link_[COM_id].xpos;
    com_vel_desired_pre_.setZero();
    com_acc_desired_pre_.setZero();

    com_vel_cutoff_freq_ = 1;

    pre_time_ = rd_.control_time_ - 0.001;
    pre_desired_q_ = rd_.q_;
    last_desired_q_ = rd_.q_;
    pre_desired_q_dot_.setZero();

    init_q_ = rd_.q_;
    zero_q_ = init_q_;
    desired_q_ = init_q_;
    desired_q_dot_.setZero();
    desired_q_ddot_.setZero();
    torque_task_.setZero();
    torque_task_pre_.setZero();

    A_mat_pre_ = rd_.A_;

    motion_q_pre_ = init_q_;
    motion_q_dot_pre_.setZero();

    contact_force_lfoot_.setZero();
    contact_force_rfoot_.setZero();
    contact_force_lfoot_local_.setZero();
    contact_force_rfoot_local_.setZero();

    zmp_local_lfoot_.setZero();
    zmp_local_rfoot_.setZero();
    zmp_measured_.setZero();
    zmp_dot_measured_.setZero();

    f_star_l_.setZero();
    f_star_r_.setZero();
    f_star_l_pre_.setZero();
    f_star_r_pre_.setZero();

    swingfoot_f_star_l_.setZero();
    swingfoot_f_star_r_.setZero();
    swingfoot_f_star_l_pre_.setZero();
    swingfoot_f_star_r_pre_.setZero();

    f_lfoot_damping_.setZero();
    f_rfoot_damping_.setZero();
    f_lfoot_damping_pre_.setZero();
    f_rfoot_damping_pre_.setZero();

    foot_lift_count_ = 0;
    foot_landing_count_ = 0;

    lhand_control_point_offset_.setZero();
    rhand_control_point_offset_.setZero();
    lhand_control_point_offset_(2) = -0.13;
    rhand_control_point_offset_(2) = -0.13;

    robot_shoulder_width_ = 0.6;

    robot_upperarm_max_l_ = 0.3376 * 1.0;
    robot_lowerarm_max_l_ = 0.31967530867;
    // robot_arm_max_l_ = 0.98*sqrt(robot_upperarm_max_l_*robot_upperarm_max_l_ + robot_lowerarm_max_l_*robot_lowerarm_max_l_ + 2*robot_upperarm_max_l_*robot_lowerarm_max_l_*cos( -joint_limit_h_(19)) );
    robot_arm_max_l_ = (robot_upperarm_max_l_ + robot_lowerarm_max_l_) * 0.999 + lhand_control_point_offset_.norm();

    hmd_check_pose_calibration_[0] = false;
    hmd_check_pose_calibration_[1] = false;
    hmd_check_pose_calibration_[2] = false;
    hmd_check_pose_calibration_[3] = false;
    hmd_check_pose_calibration_[4] = false;
    still_pose_cali_flag_ = false;
    t_pose_cali_flag_ = false;
    forward_pose_cali_flag_ = false;
    read_cali_log_flag_ = false;

    hmd_larm_max_l_ = 0.45;
    hmd_rarm_max_l_ = 0.45;
    hmd_shoulder_width_ = 0.5;

    hmd_pelv_pose_.setIdentity();
    hmd_lshoulder_pose_.setIdentity();
    hmd_lhand_pose_.setIdentity();
    hmd_rshoulder_pose_.setIdentity();
    hmd_rupperarm_pose_.setIdentity();
    hmd_rhand_pose_.setIdentity();
    hmd_chest_pose_.setIdentity();

    hmd_pelv_pose_raw_.setIdentity();
    hmd_lshoulder_pose_raw_.setIdentity();
    hmd_lhand_pose_raw_.setIdentity();
    hmd_rshoulder_pose_raw_.setIdentity();
    hmd_rupperarm_pose_raw_.setIdentity();
    hmd_rhand_pose_raw_.setIdentity();
    hmd_chest_pose_raw_.setIdentity();

    hmd_pelv_pose_raw_last_.setIdentity();
    hmd_lshoulder_pose_raw_last_.setIdentity();
    hmd_lhand_pose_raw_last_.setIdentity();
    hmd_rshoulder_pose_raw_last_.setIdentity();
    hmd_rupperarm_pose_raw_last_.setIdentity();
    hmd_rhand_pose_raw_last_.setIdentity();
    hmd_chest_pose_raw_last_.setIdentity();

    hmd_head_pose_pre_.setIdentity();
    hmd_lshoulder_pose_pre_.setIdentity();
    hmd_lupperarm_pose_pre_.setIdentity();
    hmd_lhand_pose_pre_.setIdentity();
    hmd_rshoulder_pose_pre_.setIdentity();
    hmd_rupperarm_pose_pre_.setIdentity();
    hmd_rhand_pose_pre_.setIdentity();
    hmd_chest_pose_pre_.setIdentity();
    hmd_pelv_pose_pre_.setIdentity();

    hmd_pelv_pose_init_.setIdentity();
    tracker_status_changed_time_ = current_time_;
    hmd_tracker_status_ = false;
    hmd_tracker_status_raw_ = false;
    hmd_tracker_status_pre_ = false;

    // hmd_tracker_status_ = true;
    // hmd_tracker_status_raw_ = true;
    // hmd_tracker_status_pre_ = true;

    hmd_head_abrupt_motion_count_ = 0;
    hmd_lupperarm_abrupt_motion_count_ = 0;
    hmd_lhand_abrupt_motion_count_ = 0;
    hmd_rupperarm_abrupt_motion_count_ = 0;
    hmd_rhand_abrupt_motion_count_ = 0;
    hmd_chest_abrupt_motion_count_ = 0;
    hmd_pelv_abrupt_motion_count_ = 0;

    last_solved_hierarchy_num_ = hierarchy_num_hqpik_ - 1;
}

void CustomController::getRobotData()
{
    current_time_ = rd_.control_time_;

    if (current_time_ != pre_time_)
    {
        dt_ = current_time_ - pre_time_;
    }

    current_q_ = rd_.q_;
    current_q_dot_ = rd_.q_dot_;
    current_q_ddot_ = rd_.q_ddot_virtual_.segment(6, MODEL_DOF);
    pelv_pos_current_ = rd_.link_[Pelvis].xpos;
    pelv_vel_current_.segment(0, 3) = rd_.link_[Pelvis].v;
    pelv_vel_current_.segment(3, 3) = rd_.link_[Pelvis].w;

    pelv_rot_current_ = rd_.link_[Pelvis].rotm;
    pelv_rpy_current_ = DyrosMath::rot2Euler(pelv_rot_current_); //ZYX multiply
    pelv_rot_current_yaw_aline_ = DyrosMath::rotateWithZ(pelv_rpy_current_(2));
   
    pelv_transform_current_from_global_.translation().setZero();
    pelv_transform_current_from_global_.linear() = pelv_rot_current_yaw_aline_;
    pelv_yaw_rot_current_from_global_ = pelv_transform_current_from_global_.linear();
    pelv_angvel_current_ = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Pelvis].w;

    com_pos_current_ = pelv_yaw_rot_current_from_global_.transpose() * (rd_.link_[COM_id].xpos - pelv_pos_current_);
    com_vel_current_ = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[COM_id].v;
    com_mass_ = rd_.link_[COM_id].mass;

    for (int i = 0; i < 3; i++)
        q_pinocchio[i] = rd_.q_virtual_[i];
 
    q_pinocchio[3] = rd_.q_virtual_[3];
    q_pinocchio[4] = rd_.q_virtual_[4];
    q_pinocchio[5] = rd_.q_virtual_[5];
    q_pinocchio[6] = rd_.q_virtual_[MODEL_DOF_VIRTUAL];
       
    for (int i = 6; i < MODEL_DOF_VIRTUAL; i ++)
        q_pinocchio[i+1] = rd_.q_virtual_[i];

    pinocchio::normalize(model, q_pinocchio);
    pinocchio::computeJointJacobiansTimeVariation(model, model_data, q_pinocchio, rd_.q_dot_virtual_);
    pinocchio::computeFrameJacobian(model, model_data, q_pinocchio, RFcframe_id, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, RFj);
    pinocchio::computeFrameJacobian(model, model_data, q_pinocchio, LFcframe_id, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, LFj);
    pinocchio::getFrameJacobianTimeVariation(model, model_data, RFcframe_id, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, RFdj);
    pinocchio::getFrameJacobianTimeVariation(model, model_data, LFcframe_id, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, LFdj);
   
    lfoot_transform_current_from_global_.translation() = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_transform_current_from_global_), rd_.link_[Left_Foot].xpos);
    lfoot_transform_current_from_global_.linear() = DyrosMath::inverseIsometry3d(pelv_transform_current_from_global_) * rd_.link_[Left_Foot].rotm;
    rfoot_transform_current_from_global_.translation() = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_transform_current_from_global_), rd_.link_[Right_Foot].xpos);
    rfoot_transform_current_from_global_.linear() = DyrosMath::inverseIsometry3d(pelv_transform_current_from_global_) *  rd_.link_[Right_Foot].rotm;

    lfootc_transform_current_from_global_.translation() = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_transform_current_from_global_), model_data.oMf[LFcframe_id].translation());
    lfootc_transform_current_from_global_.linear() = DyrosMath::inverseIsometry3d(pelv_transform_current_from_global_) * model_data.oMf[LFcframe_id].rotation();
    rfootc_transform_current_from_global_.translation() = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_transform_current_from_global_), model_data.oMf[RFcframe_id].translation());
    rfootc_transform_current_from_global_.linear() = DyrosMath::inverseIsometry3d(pelv_transform_current_from_global_) *  model_data.oMf[RFcframe_id].rotation();


    Eigen::Isometry3d supportfoot_from_global;
    Eigen::Matrix6d rot_sup, rot_float;
    rot_sup.setZero();
    rot_float.setZero();

    if(atb_phase_update_ == false)
    {
        atb_phase_update_ = true;
        supportFoot_fast = supportFoot_;
        zmpx_fast = zmpx_;
        zmpy_fast = zmpy_;
        contactMode_fast = contactMode_;
        com_alpha_fast = com_alpha_;
        atb_phase_update_ = false;
    }
   
    if(supportFoot_fast == 0)
    {
        supportfoot_from_global = rfoot_transform_current_from_global_;
    }
    else
    {
        supportfoot_from_global = lfoot_transform_current_from_global_;
    }

    Eigen::Isometry3d supportfoot_from_global_current_yaw_only;
    supportfoot_from_global_current_yaw_only.translation() = supportfoot_from_global.translation();
    Eigen::Vector3d support_foot_from_global_rpy;
    support_foot_from_global_rpy = DyrosMath::rot2Euler(supportfoot_from_global.linear());
    supportfoot_from_global_current_yaw_only.linear() = DyrosMath::rotateWithZ(support_foot_from_global_rpy(2));
   
    rot_sup.block(0,0,3,3) = supportfoot_from_global_current_yaw_only.linear().transpose();
    rot_sup.block(3,3,3,3) = rot_sup.block(0,0,3,3);
    rot_float.block(0,0,3,3) = pelv_transform_current_from_global_.linear().transpose();
    rot_float.block(3,3,3,3) = rot_float.block(0,0,3,3);

    lfoot_transform_current_from_support_ = DyrosMath::inverseIsometry3d(supportfoot_from_global_current_yaw_only) * lfoot_transform_current_from_global_;
    rfoot_transform_current_from_support_ = DyrosMath::inverseIsometry3d(supportfoot_from_global_current_yaw_only) * rfoot_transform_current_from_global_;
   
    lfootc_transform_current_from_support_ = DyrosMath::inverseIsometry3d(supportfoot_from_global_current_yaw_only) * lfootc_transform_current_from_global_;
    rfootc_transform_current_from_support_ = DyrosMath::inverseIsometry3d(supportfoot_from_global_current_yaw_only) * rfootc_transform_current_from_global_;
   

    lfoot_sx = lfootc_transform_current_from_support_.translation()(0);
    lfoot_sy  = lfootc_transform_current_from_support_.translation()(1);
    rfoot_sx = rfootc_transform_current_from_support_.translation()(0);
    rfoot_sy  = rfootc_transform_current_from_support_.translation()(1);

    lfoot_sx_float = lfoot_transform_current_from_global_.translation()(0);
    lfoot_sy_float  = lfoot_transform_current_from_global_.translation()(1);
    rfoot_sx_float = rfoot_transform_current_from_global_.translation()(0);
    rfoot_sy_float  = rfoot_transform_current_from_global_.translation()(1);

    RFj = rot_sup * rot_float * RFj;
    LFj = rot_sup * rot_float * LFj;
    RFdj = rot_sup * rot_float * RFdj;
    LFdj = rot_sup * rot_float * LFdj;
}

void CustomController::walkingStateManager()
{
    if (walking_phase_ < 1)
    {
        if (walking_speed_ == 0)
        {
            //first step start
            if (foot_swing_trigger_ == false)
            {

                start_walking_trigger_ = false;
                stop_walking_trigger_ = true;

                if (stop_walking_trigger_ == true)
                {
                    // if (balanceTrigger(com_pos_current_.segment<2>(0), com_vel_current_.segment<2>(0))) //gonna be fall
                    // // if(false)
                    // {
                    // 	foot_swing_trigger_ = true;
                    // 	first_step_trigger_ = true;
                    // 	start_walking_trigger_ = false;
                    // 	stop_walking_trigger_ = false;
                    // 	start_time_ = current_time_;

                    // 	if(com_vel_current_(1) >=0)
                    // 	{
                    // 		foot_contact_ = -1;
                    // 	}
                    // 	else
                    // 	{
                    // 		foot_contact_ = 1;
                    // 	}

                    // 	// foot_contact_ = -foot_contact_;  //support foot change
                    // 	// std::cout << " ################################ Balancing Control ON! ################################" << std::endl;
                    // }
                    // else
                    // {
                    foot_swing_trigger_ = false;
                    first_step_trigger_ = false;
                    start_time_ = current_time_;
                    // std::cout << " ################################ STOP WALKING ON! ################################" << std::endl;
                    // }
                }
            }
        }
        else
        {
            stop_walking_trigger_ = false;

            if (foot_swing_trigger_ == false)
            {

                start_walking_trigger_ = true;

                if (current_time_ >= start_time_ + walking_duration_start_delay_) // swing foot starts to move
                {
                    foot_swing_trigger_ = true;
                    first_step_trigger_ = true;
                    start_walking_trigger_ = false;
                    start_time_ = current_time_;

                    std::cout << " ################################ First Step Triggered! ################################" << std::endl;
                }
            }
            else
            {

                if (foot_contact_ == 1)
                {

                    // if (-r_ft_(2) > rd_.link_[COM_id].mass * GRAVITY / 2)
                    // {
                    // 	foot_landing_count_ += 1;
                    // }
                    // else
                    // {
                    // 	foot_landing_count_ = 0;
                    // }

                    // if ((walking_phase_ > 0.5) && (foot_landing_count_ > 100) )
                    // {

                    // 	// std::cout << " ################ Early Step Change Occured! ("<<walking_phase_<<", "<< -r_ft_(2) <<") #########################" << std::endl;
                    // 	walking_phase_ = 1;
                    // 	// start_time_ = current_time_ - walking_duration_; //make walking_phase_ 1
                    // }
                }
                else if (foot_contact_ == -1)
                {

                    // if (-l_ft_(2) > rd_.link_[COM_id].mass * GRAVITY / 2)
                    // {
                    // 	foot_landing_count_ += 1;
                    // }
                    // else
                    // {
                    // 	foot_landing_count_ = 0;
                    // }

                    // if ((walking_phase_ > 0.5) && (foot_landing_count_ > 100))
                    // {
                    // 	// std::cout << " ################ Early Step Change Occured! ("<<walking_phase_<<", "<< -l_ft_(2) <<") #########################" << std::endl;
                    // 	// start_time_ = current_time_ - walking_duration_; //make walking_phase_ 1
                    // 	walking_phase_ = 1;
                    // }
                }
            }
        }
    }

    if (walking_phase_ == 1)
    {
        if (walking_speed_ == 0)
        {
            // if (balanceTrigger(com_pos_current_.segment<2>(0), com_vel_current_.segment<2>(0))) //gonna be fall

            stop_walking_counter_++;
            if (stop_walking_counter_ < max_stop_walking_num_)
            {
                foot_swing_trigger_ = true;
                foot_contact_ = -foot_contact_; //support foot change

                if (first_step_trigger_ == true)
                {
                    first_step_trigger_ = false;
                }
                std::cout << " ################################ Robot Is Stopping! ################################" << std::endl;
                std::cout << " ################################" << max_stop_walking_num_ - stop_walking_counter_ << "steps are left################################" << std::endl;
            }
            else
            {
                foot_swing_trigger_ = false;
                stop_walking_trigger_ = true; //robot stop
                first_step_trigger_ = false;
                start_walking_trigger_ = false;

                // foot_contact_ = -foot_contact_;
                stance_start_time_ = current_time_;

                stop_walking_counter_ = 0;
                std::cout << " ################################ Robot Stops Walking! ################################" << std::endl;
            }
        }
        else
        {
            foot_swing_trigger_ = true;
            stop_walking_trigger_ = false;
            first_step_trigger_ = false;
            start_walking_trigger_ = false;

            foot_contact_ = -foot_contact_;
            std::cout << " ################################ Support Foot Changed! ################################" << std::endl;
        }
        start_time_ = current_time_;
    }

    if (start_walking_trigger_ == true)
    {
        walking_duration_ = walking_duration_cmd_ + walking_duration_start_delay_;
    }
    else
    {
        walking_duration_ = walking_duration_cmd_;
        walking_duration_ = DyrosMath::minmax_cut(walking_duration_, 0.2, 1.5);
    }

    // turning_duration_ = walking_duration_*0.8;
    turning_duration_ = DyrosMath::minmax_cut(turning_duration_, 0.2, 1.5);

    walking_phase_ = (current_time_ - start_time_) / walking_duration_;
    walking_phase_ = DyrosMath::minmax_cut(walking_phase_, 0.0, 1.0);
    turning_phase_ = (current_time_ - start_time_ - (dsp_duration_)) / turning_duration_;
    turning_phase_ = DyrosMath::minmax_cut(turning_phase_, 0.0, 1.0);
    // walking_duration_ = walking_duration_cmd_  - 1.0*(abs(com_pos_error_(1)) + abs(com_vel_error_(1))*0.3) - 1.0*(abs(com_pos_error_(0)) + abs(com_vel_error_(0))*0.3);

    // if( true)
    // {
    // 	cout<<"walking_phase: "<<walking_phase_<<endl;
    // 	cout<<"turning phase: "<<turning_phase_<<endl;
    // }
}

bool CustomController::balanceTrigger(Eigen::Vector2d com_pos_2d, Eigen::Vector2d com_vel_2d)
{
    bool trigger = false;
    Vector2d capture_point_2d;
    Vector2d middle_point_of_foot_2d;
    double omega;
    omega = sqrt(GRAVITY / (com_pos_current_(2) - support_foot_transform_current_.translation()(2)));
    capture_point_2d = com_pos_2d + com_vel_2d / omega;
    middle_point_of_foot_2d = middle_of_both_foot_.segment(0, 2);

    // if(capture_point_2d.norm() > stop_vel_threshold_)
    // {
    //     trigger = true;
    //     cout<<"balance swing foot control activated"<<endl;
    // }
    if ((capture_point_2d(0) > middle_point_of_foot_2d(0) + 0.10) || (capture_point_2d(0) < middle_point_of_foot_2d(0) - 0.05))
    {
        trigger = true;
        // std::cout << "Catpure point in X axis is over the safety boundary! balance swing foot control activated" << std::endl;
    }

    if ((capture_point_2d(1) > lfoot_transform_current_from_global_.translation()(1) - 0.02) || (capture_point_2d(1) < rfoot_transform_current_from_global_.translation()(1) + 0.02))
    {
        trigger = true;
        // std::cout << "Catpure point in Y axis is over the safety boundary! balance swing foot control activated" << std::endl;
    }

    // if( com_vel_2d.norm() > stop_vel_threshold_)
    // {
    //     trigger = true;
    //     cout<<"com vel is over the limit ("<< com_vel_2d.norm()<<")"<<endl;
    // }

    if (abs(com_vel_2d(0)) > 0.2 || abs(com_vel_2d(1)) > 0.15)
    {
        trigger = true;
        // std::cout << "com vel is over the limit (" << com_vel_2d(0) << "," << com_vel_2d(1) << ")" << std::endl;
    }

    if (abs(lfoot_transform_current_from_global_.translation()(0) - rfoot_transform_current_from_global_.translation()(0)) > 0.03 || abs(lfoot_transform_current_from_global_.translation()(1) - rfoot_transform_current_from_global_.translation()(1)) > 0.25 || abs(lfoot_transform_current_from_global_.translation()(1) - rfoot_transform_current_from_global_.translation()(1)) < 0.18)
    {
        trigger = true;
        // std::cout << "Foot is not aligned" << std::endl;
    }

    // if (current_q_(3) > 0.2 || current_q_(9) > 0.2)
    // {
    // 	trigger = true;
    // 	cout << "Knee is bent" << endl;
    // }

    return trigger;
}

int CustomController::checkZMPinWhichFoot(Eigen::Vector2d zmp_measured)
{
    int flag;
    Eigen::Vector2d diff_zmp_lfoot;
    Eigen::Vector2d diff_zmp_rfoot;
    Eigen::Vector2d foot_size;
    double safe_region_ratio = 0.9;

    diff_zmp_lfoot(0) = abs(zmp_measured(0) - lfoot_transform_current_from_global_.translation()(0));
    diff_zmp_lfoot(1) = abs(zmp_measured(1) - lfoot_transform_current_from_global_.translation()(1));

    diff_zmp_rfoot(0) = abs(zmp_measured(0) - rfoot_transform_current_from_global_.translation()(0));
    diff_zmp_rfoot(1) = abs(zmp_measured(1) - rfoot_transform_current_from_global_.translation()(1));

    foot_size(0) = 0.15;
    foot_size(1) = 0.085;
    if ((diff_zmp_lfoot(0) < safe_region_ratio * foot_size(0)) && (diff_zmp_lfoot(1) < safe_region_ratio * foot_size(1)))
    {
        flag = 1; //zmp is in the left foot
    }
    else if ((diff_zmp_rfoot(0) < safe_region_ratio * foot_size(0)) && (diff_zmp_rfoot(1) < safe_region_ratio * foot_size(1)))
    {
        flag = -1; //zmp is in the right foot
    }
    else
    {
        flag = 0;
    }

    return flag;
}

void CustomController::getProcessedRobotData()
{
    if (foot_contact_ == 1) // left support foot
    {
        swing_foot_transform_current_ = rfoot_transform_current_from_global_;
        support_foot_transform_current_ = lfoot_transform_current_from_global_;
        // swing_foot_transform_current_ = rfoot_transform_pre_desired_from_;
        // support_foot_transform_current_ = lfoot_transform_pre_desired_from_;
        swing_foot_vel_current_ = rfoot_vel_current_from_global_;
        // support_foot_vel_current_.setZero();
        // support_foot_vel_current_.segment(3, 3) = pelv_angvel_current_ - lfoot_to_com_jac_from_global_.block(3, 6, 3, MODEL_DOF)*current_q_dot_;

        // com_vel_est1_ = lfoot_to_com_jac_from_global_.block(0, 6, 3, MODEL_DOF)*current_q_dot_ ;
        // com_vel_est2_ = lfoot_to_com_jac_from_global_.block(0, 6, 3, MODEL_DOF)*current_q_dot_ + DyrosMath::skm(lfoot_transform_current_from_global_.translation())*(support_foot_vel_current_.segment(3, 3));
    }
    else if (foot_contact_ == -1) //right support foot
    {
        swing_foot_transform_current_ = lfoot_transform_current_from_global_;
        support_foot_transform_current_ = rfoot_transform_current_from_global_;
        // swing_foot_transform_current_ = lfoot_transform_pre_desired_from_;
        // support_foot_transform_current_ = rfoot_transform_pre_desired_from_;
        swing_foot_vel_current_ = lfoot_vel_current_from_global_;
        // support_foot_vel_current_.setZero();
        // support_foot_vel_current_.segment(3, 3) = pelv_angvel_current_ - rfoot_to_com_jac_from_global_.block(3, 6, 3, MODEL_DOF)*current_q_dot_;

        // com_vel_est1_ = rfoot_to_com_jac_from_global_.block(0, 6, 3, MODEL_DOF)*current_q_dot_ ;
        // com_vel_est2_ = rfoot_to_com_jac_from_global_.block(0, 6, 3, MODEL_DOF)*current_q_dot_ + DyrosMath::skm(rfoot_transform_current_from_global_.translation())*(support_foot_vel_current_.segment(3, 3));
    }
    else if (foot_swing_trigger_ == false)
    {
    }

    //////////////////////////////Variables in Support Foot Frame////////////////////////
    ///////Support Foot Frame's origin is attatched to the Support Foot Frame origin////////////////////////////////
    ///////z axis is aligned with gravity force and upward//////////////////////////////////////////
    ////// x axis is poining out from center of foot to the toe direction//////////////
    Vector3d swing_foot_rpy = DyrosMath::rot2Euler(support_foot_transform_current_.linear());
    Isometry3d support_foot_transform_yaw_align = support_foot_transform_current_;
    // support_foot_transform_yaw_align.linear() = DyrosMath::rotateWithZ(swing_foot_rpy(2));	//global orientation in roll and pitch

    support_foot_transform_current_from_support_ = support_foot_transform_yaw_align.inverse() * support_foot_transform_yaw_align;
    swing_foot_transform_current_from_support_ = support_foot_transform_yaw_align.inverse() * swing_foot_transform_current_;
    lfoot_transform_current_from_support_ = support_foot_transform_yaw_align.inverse() * lfoot_transform_current_from_global_;
    rfoot_transform_current_from_support_ = support_foot_transform_yaw_align.inverse() * rfoot_transform_current_from_global_;
    pelv_transform_current_from_support_ = support_foot_transform_yaw_align.inverse() * pelv_transform_current_from_global_;

    middle_of_both_foot_ = (lfoot_transform_current_from_support_.translation() + rfoot_transform_current_from_support_.translation()) / 2;

    com_pos_current_from_support_ = DyrosMath::multiplyIsometry3dVector3d(support_foot_transform_yaw_align.inverse(), com_pos_current_);
    com_vel_current_from_support_ = support_foot_transform_yaw_align.linear().transpose() * com_vel_current_;
    com_acc_current_from_support_ = support_foot_transform_yaw_align.linear().transpose() * com_acc_current_;

    if (foot_contact_ != foot_contact_pre_)
    {
        com_pos_pre_from_support_ = DyrosMath::multiplyIsometry3dVector3d(swing_foot_transform_current_from_support_, com_pos_pre_from_support_);
        com_pos_ppre_from_support_ = DyrosMath::multiplyIsometry3dVector3d(swing_foot_transform_current_from_support_, com_pos_ppre_from_support_);
        com_vel_pre_lpf_from_support_ = swing_foot_transform_current_from_support_.linear() * com_vel_pre_lpf_from_support_;
        com_vel_ppre_lpf_from_support_ = swing_foot_transform_current_from_support_.linear() * com_vel_ppre_lpf_from_support_;
        com_vel_pre_from_support_ = swing_foot_transform_current_from_support_.linear() * com_vel_pre_from_support_;
        com_vel_ppre_from_support_ = swing_foot_transform_current_from_support_.linear() * com_vel_ppre_from_support_;
        com_acc_pre_from_support_ = swing_foot_transform_current_from_support_.linear() * com_acc_pre_from_support_;
        com_acc_ppre_from_support_ = swing_foot_transform_current_from_support_.linear() * com_acc_ppre_from_support_;

        com_pos_desired_from_support_ = DyrosMath::multiplyIsometry3dVector3d(swing_foot_transform_current_from_support_, com_pos_desired_from_support_);
        com_vel_desired_from_support_ = swing_foot_transform_current_from_support_.linear() * com_vel_desired_from_support_;
        com_acc_desired_from_support_ = swing_foot_transform_current_from_support_.linear() * com_acc_desired_from_support_;
        com_jerk_desired_from_support_ = swing_foot_transform_current_from_support_.linear() * com_jerk_desired_from_support_;

        com_pos_pre_desired_from_support_ = DyrosMath::multiplyIsometry3dVector3d(swing_foot_transform_current_from_support_, com_pos_pre_desired_from_support_);
        com_vel_pre_desired_from_support_ = swing_foot_transform_current_from_support_.linear() * com_vel_pre_desired_from_support_;
        com_acc_pre_desired_from_support_ = swing_foot_transform_current_from_support_.linear() * com_acc_pre_desired_from_support_;
        com_jerk_pre_desired_from_support_ = swing_foot_transform_current_from_support_.linear() * com_jerk_pre_desired_from_support_;

        lfoot_transform_desired_last_ = swing_foot_transform_current_from_support_ * lfoot_transform_desired_;
        rfoot_transform_desired_last_ = swing_foot_transform_current_from_support_ * rfoot_transform_desired_;
        pelv_transform_desired_last_ = swing_foot_transform_current_from_support_ * pelv_transform_desired_;

        cout << "_______________Support Foot is Changed!!!_______________" << endl;
    }
    /////////////////////////////////////////////////////////////////////////////////////

    com_vel_current_lpf_from_support_ = DyrosMath::secondOrderLowPassFilter<3>(
        com_vel_current_from_support_, com_vel_pre_from_support_, com_vel_ppre_from_support_, com_vel_pre_lpf_from_support_, com_vel_ppre_lpf_from_support_,
        com_vel_cutoff_freq_, 1 / sqrt(2), 1 / dt_);
    // com_vel_current_lpf_from_support_ = DyrosMath::lpf<3>(com_vel_current_from_support_, com_vel_pre_lpf_from_support_, 1 / dt_, com_vel_cutoff_freq_);

    zc_ = com_pos_current_from_support_(2);
    wn_ = sqrt(GRAVITY / zc_);
    cp_current_from_suppport_ = com_pos_current_from_support_ + com_vel_current_lpf_from_support_ / wn_;

    // zmp_measured_local_ = WBC::GetZMPpos_fromFT(rd_, true);

    swing_foot_pos_error_from_support_ = swing_foot_pos_trajectory_from_support_ - swing_foot_transform_current_from_support_.translation();
    // middle_of_both_foot_ = (lfoot_transform_current_from_global_.translation() + rfoot_transform_current_from_global_.translation()) / 2;
    // middle_of_both_foot_ = (lfoot_transform_pre_desired_from_.translation() + rfoot_transform_pre_desired_from_.translation())/2;

    if (walking_mode_on_) //command on
    {
        stance_start_time_ = current_time_;
        start_time_ = current_time_;
        program_start_time_ = current_time_;

        init_q_ = current_q_;
        last_desired_q_ = current_q_;

        com_pos_pre_from_support_ = com_pos_current_from_support_;
        com_pos_ppre_from_support_ = com_pos_current_from_support_;
        com_vel_pre_from_support_.setZero();
        com_vel_ppre_from_support_.setZero();
        com_acc_pre_from_support_.setZero();
        com_acc_ppre_from_support_.setZero();

        com_vel_pre_lpf_from_support_.setZero();
        com_vel_ppre_lpf_from_support_.setZero();

        com_pos_desired_preview_ = com_pos_current_;
        com_vel_desired_preview_.setZero();
        com_acc_desired_preview_.setZero();

        com_vel_desired_preview_pre_ = com_pos_current_;
        com_vel_desired_preview_pre_.setZero();
        com_vel_desired_preview_pre_.setZero();

        com_pos_init_from_support_ = com_pos_current_from_support_;

        com_pos_desired_ = com_pos_current_;
        com_vel_desired_.setZero();
        com_acc_desired_.setZero();

        com_pos_desired_last_ = com_pos_current_;
        com_vel_desired_last_.setZero();
        com_acc_desired_last_.setZero();

        com_pos_desired_from_support_ = com_pos_current_from_support_;
        com_vel_desired_from_support_.setZero();
        com_acc_desired_from_support_.setZero();
        com_jerk_desired_from_support_.setZero();

        com_pos_pre_desired_from_support_ = com_pos_current_from_support_;
        com_vel_pre_desired_from_support_.setZero();
        com_acc_pre_desired_from_support_.setZero();
        com_jerk_pre_desired_from_support_.setZero();

        xs_.setZero();
        ys_.setZero();

        xi_ = com_pos_current_from_support_(0);
        yi_ = com_pos_current_from_support_(1);

        xd_.setZero();
        yd_.setZero();
        xd_(0) = com_pos_current_from_support_(0);
        yd_(0) = com_pos_current_from_support_(1);

        xd_b.setZero();
        yd_b.setZero();
        xd_b(0) = com_pos_current_from_support_(0);
        yd_b(0) = com_pos_current_from_support_(1);

        walking_mode_on_ = false;

        swing_foot_pos_error_from_support_.setZero();

        pelv_transform_init_from_support_ = pelv_transform_current_from_support_;
        pelv_transform_start_from_support_ = pelv_transform_current_from_support_;
        lfoot_transform_start_from_support_ = lfoot_transform_current_from_support_;
        rfoot_transform_start_from_support_ = rfoot_transform_current_from_support_;

        lfoot_transform_desired_ = lfoot_transform_current_from_support_;
        rfoot_transform_desired_ = rfoot_transform_current_from_support_;
        pelv_transform_desired_ = pelv_transform_current_from_support_;

        lfoot_transform_desired_last_ = lfoot_transform_current_from_support_;
        rfoot_transform_desired_last_ = rfoot_transform_current_from_support_;
        pelv_transform_desired_last_ = pelv_transform_current_from_support_;

        pelv_transform_start_from_global_.translation() = pelv_pos_current_;
        pelv_transform_start_from_global_.linear() = pelv_rot_current_yaw_aline_;
        lfoot_transform_start_from_global_ = lfoot_transform_current_from_global_;
        rfoot_transform_start_from_global_ = rfoot_transform_current_from_global_;

        lfoot_transform_init_from_global_ = lfoot_transform_current_from_global_;
        rfoot_transform_init_from_global_ = rfoot_transform_current_from_global_;

        lhand_transform_init_from_global_ = lhand_transform_current_from_global_;
        rhand_transform_init_from_global_ = rhand_transform_current_from_global_;

        lelbow_transform_init_from_global_ = lelbow_transform_current_from_global_;
        relbow_transform_init_from_global_ = relbow_transform_current_from_global_;

        lupperarm_transform_init_from_global_ = lupperarm_transform_current_from_global_;
        rupperarm_transform_init_from_global_ = rupperarm_transform_current_from_global_;

        lshoulder_transform_init_from_global_ = lshoulder_transform_current_from_global_;
        rshoulder_transform_init_from_global_ = rshoulder_transform_current_from_global_;

        lacromion_transform_init_from_global_ = lacromion_transform_current_from_global_;
        racromion_transform_init_from_global_ = racromion_transform_current_from_global_;

        larmbase_transform_init_from_global_ = larmbase_transform_current_from_global_;
        rarmbase_transform_init_from_global_ = rarmbase_transform_current_from_global_;

        head_transform_init_from_global_ = head_transform_current_from_global_;
        upperbody_transform_init_from_global_ = upperbody_transform_current_from_global_;

        lhand_vel_error_.setZero();
        rhand_vel_error_.setZero();
        lelbow_vel_error_.setZero();
        relbow_vel_error_.setZero();
        lacromion_vel_error_.setZero();
        racromion_vel_error_.setZero();
    }

    bool robot_goes_into_stance_phase = (current_time_ == stance_start_time_);
    bool robot_start_walking = ((start_walking_trigger_ == true) && (current_time_ == start_time_));
    bool robot_start_swing = ((foot_swing_trigger_ == true) && (current_time_ == start_time_));

    if (robot_goes_into_stance_phase || robot_start_walking || robot_start_swing)
    {
        com_pos_init_ = com_pos_current_;
        com_vel_init_ = com_vel_current_;
        com_acc_init_ = com_acc_current_;

        com_pos_init_from_support_ = com_pos_current_from_support_;

        pelv_pos_init_ = pelv_pos_current_;
        pelv_vel_init_ = pelv_vel_current_;
        pelv_rot_init_ = pelv_rot_current_;
        pelv_rpy_init_ = pelv_rpy_current_;
        pelv_rot_init_yaw_aline_ = pelv_rot_current_yaw_aline_;
        pelv_transform_init_from_global_ = pelv_transform_current_from_global_;

        lfoot_transform_init_from_global_ = lfoot_transform_current_from_global_;
        rfoot_transform_init_from_global_ = rfoot_transform_current_from_global_;
        // lfoot_transform_init_from_global_ = lfoot_transform_pre_desired_from_;
        // rfoot_transform_init_from_global_ = rfoot_transform_pre_desired_from_;
        if (foot_contact_ == 1) // left support foot
        {
            swing_foot_transform_init_ = rfoot_transform_current_from_global_;
            support_foot_transform_init_ = lfoot_transform_current_from_global_;
            // swing_foot_transform_init_ = rfoot_transform_pre_desired_from_;
            // support_foot_transform_init_ = lfoot_transform_pre_desired_from_;
            swing_foot_vel_init_ = rfoot_vel_current_from_global_;
        }
        else if (foot_contact_ == -1) //right support foot
        {
            swing_foot_transform_init_ = lfoot_transform_current_from_global_;
            support_foot_transform_init_ = rfoot_transform_current_from_global_;
            // swing_foot_transform_init_ = lfoot_transform_pre_desired_from_;
            // support_foot_transform_init_ = rfoot_transform_pre_desired_from_;
            swing_foot_vel_init_ = lfoot_vel_current_from_global_;
        }
        swing_foot_rpy_init_ = DyrosMath::rot2Euler(swing_foot_transform_init_.linear());
        support_foot_rpy_init_ = DyrosMath::rot2Euler(support_foot_transform_init_.linear());

        // init_q_ = current_q_;
        last_desired_q_ = desired_q_;
        foot_lift_count_ = 0;

        com_pos_desired_last_ = com_pos_desired_;
        com_vel_desired_last_ = com_vel_desired_;
        com_acc_desired_last_ = com_acc_desired_;

        middle_of_both_foot_init_ = middle_of_both_foot_;

        swingfoot_f_star_l_pre_.setZero();
        swingfoot_f_star_r_pre_.setZero();

        swing_foot_transform_init_from_support_ = swing_foot_transform_current_from_support_;
        swing_foot_rpy_init_from_support_ = DyrosMath::rot2Euler(swing_foot_transform_init_from_support_.linear());
        support_foot_transform_init_from_support_ = support_foot_transform_current_from_support_;
        support_foot_rpy_init_from_support_ = DyrosMath::rot2Euler(support_foot_transform_init_from_support_.linear());

        lfoot_transform_init_from_support_ = lfoot_transform_current_from_support_;
        rfoot_transform_init_from_support_ = rfoot_transform_current_from_support_;
        pelv_transform_init_from_support_ = pelv_transform_current_from_support_;
        pelv_rpy_init_from_support_ = DyrosMath::rot2Euler(pelv_transform_init_from_support_.linear());
        swing_foot_pos_error_from_support_.setZero();
    }

    if (current_time_ == program_start_time_)
    {
        support_foot_transform_pre_ = support_foot_transform_current_;
        swing_foot_transform_pre_ = swing_foot_transform_current_;

        com_pos_desired_preview_pre_ = com_pos_current_from_support_;
        com_vel_desired_preview_pre_.setZero();
        com_acc_desired_preview_pre_.setZero();

        //preview gain update
        previewParam_MJ(1 / preview_hz_, zmp_size_, zc_, K_act_, Gi_, Gd_, Gx_, A_, B_, C_, D_, A_bar_, B_bar_);

        last_preview_param_update_time_ = current_time_;
        preview_update_time_ = current_time_;

        for (int i = 0; i < zmp_size_; i++)
        {
            ref_zmp_(i, 0) = com_pos_init_from_support_(0);
            ref_zmp_(i, 1) = com_pos_init_from_support_(1);
        }
    }

    swingfoot_force_control_converter_ = DyrosMath::cubic(walking_phase_, 0.8, 0.9, 0, 1, 0, 0);
    // swingfoot_force_control_converter_ = 0;
}

void CustomController::motionGenerator()
{
    motion_q_dot_.setZero();
    motion_q_.setZero();
    pd_control_mask_.setZero();

    ///////////////////////LEG/////////////////////////
    //////LEFT LEG///////0 0 0.02 0.15 -0.17 0
    motion_q_(0) = 0;
    motion_q_(1) = 0;
    motion_q_(2) = -0.55;
    // motion_q_(3)   = DyrosMath::cubic(walking_phase_, 0.7, 1, knee_target_angle_, 2*knee_target_angle_, 0, 0); //0.1
    motion_q_(3) = 1.26;
    motion_q_(4) = -0.71;
    motion_q_(5) = 0.0;
    pd_control_mask_(0) = 1;
    pd_control_mask_(1) = 1;
    pd_control_mask_(2) = 1;
    pd_control_mask_(3) = 1;
    pd_control_mask_(4) = 1;
    pd_control_mask_(5) = 1;
    //////////////////////
    /////RIFHT LEG////////0 0 0.02 0.15 -0.17 0
    motion_q_(6) = 0;
    motion_q_(7) = 0;
    motion_q_(8) = -0.55;
    // motion_q_(9)   = DyrosMath::cubic(walking_phase_, 0.7, 1, knee_target_angle_, 2*knee_target_angle_, 0, 0); //0.1
    motion_q_(9) = 1.26;
    motion_q_(10) = -0.71;
    motion_q_(11) = 0.0;
    pd_control_mask_(6) = 1;
    pd_control_mask_(7) = 1;
    pd_control_mask_(8) = 1;
    pd_control_mask_(9) = 1;
    pd_control_mask_(10) = 1;
    pd_control_mask_(11) = 1;
    //////////////////////

    if (upper_body_mode_ == 3) // Freezing
    {
        if (upperbody_mode_recieved_ == true)
        {
            cout << "Upperbody Mode is Changed to #3" << endl;
            cout << "----------Robot is Freezed---------" << endl;

            upperbody_mode_recieved_ = false;
            upperbody_mode_q_init_ = motion_q_pre_;
        }

        for (int i = 12; i < MODEL_DOF; i++)
        {
            motion_q_(i) = upperbody_mode_q_init_(i);
            pd_control_mask_(i) = 1;
        }
    }
}

void CustomController::savePreData()
{
    pre_time_ = current_time_;
    pre_q_ = rd_.q_;
    pre_desired_q_ = desired_q_;
    pre_desired_q_dot_ = desired_q_dot_;
    motion_q_pre_ = motion_q_;
    motion_q_dot_pre_ = motion_q_dot_;

    zmp_measured_ppre_ = zmp_measured_pre_;
    zmp_measured_pre_ = zmp_measured_;
    com_pos_desired_pre_ = com_pos_desired_;
    com_vel_desired_pre_ = com_vel_desired_;
    com_acc_desired_pre_ = com_acc_desired_;

    com_pos_ppre_from_support_ = com_pos_pre_from_support_;
    com_vel_ppre_from_support_ = com_vel_pre_from_support_;
    com_acc_ppre_from_support_ = com_acc_pre_from_support_;

    com_pos_pre_from_support_ = com_pos_current_from_support_;
    com_vel_pre_from_support_ = com_vel_current_from_support_;
    com_acc_pre_from_support_ = com_acc_current_from_support_;

    com_vel_ppre_lpf_from_support_ = com_vel_pre_lpf_from_support_;
    com_vel_pre_lpf_from_support_ = com_vel_current_lpf_from_support_;

    if (current_time_ == preview_update_time_)
    {
        com_pos_pre_desired_from_support_ = com_pos_desired_from_support_;
        com_vel_pre_desired_from_support_ = com_vel_desired_from_support_;
        com_acc_pre_desired_from_support_ = com_acc_desired_from_support_;
        com_jerk_pre_desired_from_support_ = com_jerk_desired_from_support_;
    }

    torque_task_pre_ = torque_task_;
    torque_grav_pre_ = torque_grav_;

    foot_contact_pre_ = foot_contact_;

    zmp_desired_pre_ = zmp_desired_from_global_;
    zmp_local_lfoot_pre_ = zmp_local_lfoot_;
    zmp_local_rfoot_pre_ = zmp_local_rfoot_;

    swing_foot_transform_pre_ = swing_foot_transform_current_;
    support_foot_transform_pre_ = support_foot_transform_current_;
    swing_foot_transform_pre_from_support_ = swing_foot_transform_current_from_support_;
    support_foot_transform_pre_from_support_ = support_foot_transform_current_from_support_;

    com_pos_desired_preview_pre_ = com_pos_desired_preview_;
    com_vel_desired_preview_pre_ = com_vel_desired_preview_;
    com_acc_desired_preview_pre_ = com_acc_desired_preview_;
}

/////////////////////////////////////PREVIEW CONTROL RELATED FUNCTION////////////////////////////////////
void CustomController::getComTrajectory_Preview()
{
    // on-line preview
    // xs_(0) = com_pos_current_(0); ys_(0) = com_pos_current_(1);
    // xs_(1) = com_vel_current_(0); ys_(1) = com_vel_current_(1);
    // xs_(2) = com_acc_current_(0); ys_(2) = com_acc_current_(1);
    // xs_(0) = support_foot_transform_current_.translation()(0) + com_pos_desired_pre_(0) - support_foot_transform_pre_.translation()(0);
    // ys_(0) = support_foot_transform_current_.translation()(1) + com_pos_desired_pre_(1) - support_foot_transform_pre_.translation()(1);
    // xs_(0) = com_pos_current_from_support_(0);
    // ys_(0) = com_pos_current_from_support_(1);

    //off-line preview
    xs_(0) = com_pos_pre_desired_from_support_(0);
    ys_(0) = com_pos_pre_desired_from_support_(1);
    xs_(1) = com_vel_pre_desired_from_support_(0);
    ys_(1) = com_vel_pre_desired_from_support_(1);
    xs_(2) = com_acc_pre_desired_from_support_(0);
    ys_(2) = com_acc_pre_desired_from_support_(1);

    modifiedPreviewControl_MJ();

    // off-line preview
    // xs_(0) = xd_(0); ys_(0) = yd_(0);
    // // On,off-line preview
    // xs_(1) = xd_(1); ys_(1) = yd_(1); // 여기서부터
    // xs_(2) = xd_(2); ys_(2) = yd_(2);

    if (current_time_ == preview_update_time_)
    {
        com_pos_desired_from_support_.setZero();
        com_vel_desired_from_support_.setZero();
        com_acc_desired_from_support_.setZero();
        com_jerk_desired_from_support_.setZero();

        com_pos_desired_from_support_(0) = xd_(0);
        com_pos_desired_from_support_(1) = yd_(0);
        com_vel_desired_from_support_(0) = xd_(1);
        com_vel_desired_from_support_(1) = yd_(1);
        com_acc_desired_from_support_(0) = xd_(2);
        com_acc_desired_from_support_(1) = yd_(2);
        com_jerk_desired_from_support_(0) = UX_;
        com_jerk_desired_from_support_(1) = UY_;

        // std::cout<<"xd_(0) :"<<xd_(0) <<std::endl;
        // std::cout<<"yd_(0) :"<<yd_(0) <<std::endl;
    }

    for (int i = 0; i < 3; i++)
    {
        com_pos_desired_from_support_(i) = DyrosMath::minmax_cut(com_pos_desired_from_support_(i), com_pos_limit_(i), com_pos_limit_(i + 3));
        com_vel_desired_from_support_(i) = DyrosMath::minmax_cut(com_vel_desired_from_support_(i), com_vel_limit_(i), com_vel_limit_(i + 3));
        com_acc_desired_from_support_(i) = DyrosMath::minmax_cut(com_acc_desired_from_support_(i), com_acc_limit_(i), com_acc_limit_(i + 3));
    }

    if ((com_pos_desired_from_support_(0) == com_pos_limit_(0)) || (com_pos_desired_from_support_(0) == com_pos_limit_(3)))
    {
        cout << "COM POS X IS OVER THE LIMIT" << endl;
    }
    if ((com_pos_desired_from_support_(1) == com_pos_limit_(1)) || (com_pos_desired_from_support_(1) == com_pos_limit_(4)))
    {
        cout << "COM POS Y IS OVER THE LIMIT" << endl;
    }
    if ((com_vel_desired_from_support_(0) == com_vel_limit_(0)) || (com_vel_desired_from_support_(0) == com_vel_limit_(3)))
    {
        cout << "COM VEL X IS OVER THE LIMIT" << endl;
    }
    if ((com_vel_desired_from_support_(1) == com_vel_limit_(1)) || (com_vel_desired_from_support_(1) == com_vel_limit_(4)))
    {
        cout << "COM VEL Y IS OVER THE LIMIT" << endl;
    }
    if ((com_acc_desired_from_support_(0) == com_acc_limit_(0)) || (com_acc_desired_from_support_(0) == com_acc_limit_(3)))
    {
        cout << "COM ACC X IS OVER THE LIMIT" << endl;
    }
    if ((com_acc_desired_from_support_(1) == com_acc_limit_(1)) || (com_acc_desired_from_support_(1) == com_acc_limit_(4)))
    {
        cout << "COM ACC Y IS OVER THE LIMIT" << endl;
    }

    if (int(current_time_ * 10000) % 1000 == 0)
    {

        // std::cout<<"current_time_ :"<<current_time_ <<std::endl;
        // std::cout<<"preview_update_time_ :"<<preview_update_time_ <<std::endl;

        // std::cout<<"com_pos_desired_preview_(1) :"<<com_pos_desired_preview_(1) <<std::endl;
        // std::cout<<"com_vel_desired_preview_(1) :"<<com_vel_desired_preview_(1) <<std::endl;
        // std::cout<<"com_acc_desired_preview_(1) :"<<com_acc_desired_preview_(1) <<std::endl;
        // std::cout<<"com_target_height_ :"<<com_target_height_ <<std::endl;
    }
}

void CustomController::modifiedPreviewControl_MJ()
{
    /////reference: http://www.tandfonline.com/doi/pdf/10.1080/0020718508961156?needAccess=true/////////////
    // if( (current_time_-last_preview_param_update_time_) >= 0.1 ) //10hz update
    if (false)
    {
        previewParam_MJ(1.0 / preview_hz_, zmp_size_, zc_, K_act_, Gi_, Gd_, Gx_, A_, B_, C_, D_, A_bar_, B_bar_);
        last_preview_param_update_time_ = current_time_;
        //previewParam_MJ_CPM(1.0/hz_, 16*hz_/10, K_ ,com_support_init_, Gi_, Gd_, Gx_, A_, B_, C_, D_, A_bar_, B_bar_);
    }

    // if( (current_time_-preview_update_time_) >= 1/preview_hz_ ) // preview_hz_ update
    if (true)
    {
        xd_b = xd_; //save previous com desired trajectory
        yd_b = yd_;
        preview_MJ(1.0 / preview_hz_, zmp_size_, xi_, yi_, xs_, ys_, UX_, UY_, Gi_, Gd_, Gx_, A_, B_, C_, xd_, yd_);
        preview_update_time_ = current_time_;
    }
}

void CustomController::previewParam_MJ(double dt, int NL, double zc, Eigen::Matrix4d &K, Eigen::MatrixXd &Gi, Eigen::VectorXd &Gd, Eigen::MatrixXd &Gx,
                                       Eigen::MatrixXd &A, Eigen::VectorXd &B, Eigen::MatrixXd &C, Eigen::MatrixXd &D, Eigen::MatrixXd &A_bar, Eigen::VectorXd &B_bar)
{
    A.resize(3, 3);
    A(0, 0) = 1.0;
    A(0, 1) = dt;
    A(0, 2) = dt * dt / 2;
    A(1, 0) = 0;
    A(1, 1) = 1.0;
    A(1, 2) = dt;
    A(2, 0) = 0;
    A(2, 1) = 0;
    A(2, 2) = 1;

    B.resize(3, 1);
    B(0, 0) = dt * dt * dt / 6;
    B(1, 0) = dt * dt / 2;
    B(2, 0) = dt;

    C.resize(1, 3);
    C(0, 0) = 1;
    C(0, 1) = 0;
    // C(0,2) = -zc/GRAVITY;
    C(0, 2) = -0.71 / 9.81; //mj gain

    B_bar.resize(4);
    B_bar.segment(0, 1) = C * B;
    B_bar.segment(1, 3) = B;

    Eigen::Matrix1x4d B_bar_tran;
    B_bar_tran = B_bar.transpose();

    Eigen::MatrixXd I_bar;
    Eigen::MatrixXd F_bar;
    A_bar.setZero(4, 4);
    I_bar.setZero(4, 1);
    F_bar.setZero(4, 3);

    F_bar.block<1, 3>(0, 0) = C * A;
    F_bar.block<3, 3>(1, 0) = A;

    I_bar.setZero();
    I_bar(0, 0) = 1.0;

    A_bar.block<4, 1>(0, 0) = I_bar;
    A_bar.block<4, 3>(0, 1) = F_bar;

    Eigen::MatrixXd Qe;
    Qe.resize(1, 1);
    Qe(0, 0) = 1.0;

    Eigen::MatrixXd R;
    R.resize(1, 1);
    R(0, 0) = 1e-6;

    Eigen::MatrixXd Qx;
    Qx.setZero(3, 3);

    Eigen::MatrixXd Q_bar;
    Q_bar.setZero(4, 4);
    Q_bar(0, 0) = Qe(0, 0);

    // K=discreteRiccatiEquationPrev(A_bar, B_bar, R, Q_bar);
    // mj gain
    K(0, 0) = 1083.572780788710;
    K(0, 1) = 586523.188429418020;
    K(0, 2) = 157943.283121116518;
    K(0, 3) = 41.206077691894;
    K(1, 0) = 586523.188429418020;
    K(1, 1) = 319653984.254277825356;
    K(1, 2) = 86082274.531361579895;
    K(1, 3) = 23397.754069026785;
    K(2, 0) = 157943.283121116518;
    K(2, 1) = 86082274.531361579895;
    K(2, 2) = 23181823.112113621086;
    K(2, 3) = 6304.466397614751;
    K(3, 0) = 41.206077691894;
    K(3, 1) = 23397.754069026785;
    K(3, 2) = 6304.466397614751;
    K(3, 3) = 2.659250532188;

    Eigen::MatrixXd Temp_mat;
    Eigen::MatrixXd Temp_mat_inv;
    Eigen::MatrixXd Ac_bar;
    Temp_mat.setZero(1, 1);
    Temp_mat_inv.setZero(1, 1);
    Ac_bar.setZero(4, 4);

    Temp_mat = R + B_bar_tran * K * B_bar;
    Temp_mat_inv = Temp_mat.inverse();

    Ac_bar = A_bar - B_bar * Temp_mat_inv * B_bar_tran * K * A_bar;

    Eigen::MatrixXd Ac_bar_tran(4, 4);
    Ac_bar_tran = Ac_bar.transpose();

    Gi.resize(1, 1);
    Gx.resize(1, 3);
    // Gi = Temp_mat_inv * B_bar_tran * K * I_bar ;
    // Gx = Temp_mat_inv * B_bar_tran * K * F_bar ;
    //mj gain
    Gi(0, 0) = 872.3477;
    Gx(0, 0) = 945252.1760702;
    Gx(0, 1) = 256298.6905049;
    Gx(0, 2) = 542.0544196;

    Eigen::MatrixXd X_bar;
    Eigen::Vector4d X_bar_col;
    X_bar.setZero(4, NL);
    X_bar_col.setZero();
    X_bar_col = -Ac_bar_tran * K * I_bar;

    for (int i = 0; i < NL; i++)
    {
        X_bar.block<4, 1>(0, i) = X_bar_col;
        X_bar_col = Ac_bar_tran * X_bar_col;
    }

    Gd.resize(NL);
    Eigen::VectorXd Gd_col(1);
    Gd_col(0) = -Gi(0, 0);

    for (int i = 0; i < NL; i++)
    {
        Gd.segment(i, 1) = Gd_col;
        Gd_col = Temp_mat_inv * B_bar_tran * X_bar.col(i);
    }
}

void CustomController::preview_MJ(double dt, int NL, double x_i, double y_i, Eigen::Vector3d xs, Eigen::Vector3d ys, double &UX, double &UY,
                                  Eigen::MatrixXd Gi, Eigen::VectorXd Gd, Eigen::MatrixXd Gx, Eigen::MatrixXd A, Eigen::VectorXd B, Eigen::MatrixXd C, Eigen::Vector3d &XD, Eigen::Vector3d &YD)
{

    Eigen::VectorXd px_ref, py_ref;
    px_ref.resize(zmp_size_);
    py_ref.resize(zmp_size_);

    for (int i = 0; i < zmp_size_; i++)
    {
        px_ref(i) = ref_zmp_(i, 0);
        py_ref(i) = ref_zmp_(i, 1);
    }

    // Eigen::Matrix1x3d C;
    C(0, 0) = 1;
    C(0, 1) = 0;
    // C(0,2) = -zc_/GRAVITY;
    C(0, 2) = -0.71 / 9.81; //mj gain

    Eigen::VectorXd px, py;
    px.resize(1);
    py.resize(1);

    if (current_time_ == program_start_time_)
    {
        preview_x_b.setZero();
        preview_y_b.setZero();
        preview_x.setZero();
        preview_y.setZero();

        preview_x_b(0) = x_i; // 이때 before 값이 없으면 첫 tick에서 deltaC x의 에러가 갑작스럽게 생겨서 발산
        preview_y_b(0) = y_i;
        preview_x(0) = x_i;
        preview_y(0) = y_i;
    }
    else
    {
        // preview_x(0) = com_pos_pre_desired_from_support_(0);
        // preview_y(0) = com_pos_pre_desired_from_support_(1);
        // preview_x(1) = com_vel_pre_desired_from_support_(0);
        // preview_y(1) = com_vel_pre_desired_from_support_(1);
        // preview_x(2) = com_acc_pre_desired_from_support_(0);
        // preview_y(2) = com_acc_pre_desired_from_support_(1);
        preview_x = xs;
        preview_y = ys;

        preview_x_b(0) = com_pos_pre_desired_from_support_(0) - com_vel_pre_desired_from_support_(0) / preview_hz_;
        preview_y_b(0) = com_pos_pre_desired_from_support_(1) - com_vel_pre_desired_from_support_(1) / preview_hz_;
        preview_x_b(1) = com_vel_pre_desired_from_support_(0) - com_acc_pre_desired_from_support_(0) / preview_hz_;
        preview_y_b(1) = com_vel_pre_desired_from_support_(1) - com_acc_pre_desired_from_support_(1) / preview_hz_;
        preview_x_b(2) = com_acc_pre_desired_from_support_(0) - com_jerk_pre_desired_from_support_(0) / preview_hz_;
        preview_y_b(2) = com_acc_pre_desired_from_support_(1) - com_jerk_pre_desired_from_support_(1) / preview_hz_;

        // preview_x_b(0) = preview_x(0) - preview_x(1)/preview_hz_;
        // preview_y_b(0) = preview_y(0) - preview_y(1)/preview_hz_;
        // preview_x_b(1) = preview_x(1) - preview_x(2)/preview_hz_;
        // preview_y_b(1) = preview_y(1) - preview_y(2)/preview_hz_;
        // preview_x_b(2) = preview_x(2) - UX/preview_hz_;
        // preview_y_b(2) = preview_y(2) - UY/preview_hz_;

        // preview_x_b = preview_x;
        // preview_y_b = preview_y;

        // preview_x_b(0) = preview_x(0) - preview_x(1)/preview_hz_;
        // preview_y_b(0) = preview_y(0) - preview_y(1)/preview_hz_;
    }

    px = C * preview_x;
    py = C * preview_y;
    // px(0) = zmp_measured_(0);
    // py(0) = zmp_measured_(1);
    zmp_current_by_com_from_support_(0) = px(0);
    zmp_current_by_com_from_support_(1) = py(0);

    double sum_Gd_px_ref = 0, sum_Gd_py_ref = 0;

    for (int i = 0; i < NL - 1; i++) // Preview Step 개수만큼 더함.
    {
        sum_Gd_px_ref = sum_Gd_px_ref + Gd(i) * (px_ref(i + 1) - px_ref(i));
        sum_Gd_py_ref = sum_Gd_py_ref + Gd(i) * (py_ref(i + 1) - py_ref(i));
    }
    Eigen::MatrixXd del_ux(1, 1);
    Eigen::MatrixXd del_uy(1, 1);
    del_ux.setZero();
    del_uy.setZero();

    Eigen::VectorXd GX_X(1);
    GX_X = Gx * (preview_x - preview_x_b);
    Eigen::VectorXd GX_Y(1);
    GX_Y = Gx * (preview_y - preview_y_b);

    del_ux(0, 0) = -(px(0) - px_ref(0)) * Gi(0, 0) - GX_X(0) - sum_Gd_px_ref;
    del_uy(0, 0) = -(py(0) - py_ref(0)) * Gi(0, 0) - GX_Y(0) - sum_Gd_py_ref;

    UX = UX + del_ux(0, 0);
    UY = UY + del_uy(0, 0);

    XD = A * preview_x + B * UX;
    YD = A * preview_y + B * UY;
}

Eigen::MatrixXd CustomController::discreteRiccatiEquationPrev(Eigen::MatrixXd a, Eigen::MatrixXd b, Eigen::MatrixXd r, Eigen::MatrixXd q)
{
    int n = a.rows(); //number of rows
    int m = b.cols(); //number of columns

    Eigen::MatrixXd z11(n, n), z12(n, n), z21(n, n), z22(n, n);

    z11 = a.inverse();
    z12 = a.inverse() * b * r.inverse() * b.transpose();
    z21 = q * a.inverse();
    z22 = a.transpose() + q * a.inverse() * b * r.inverse() * b.transpose();

    Eigen::MatrixXd z;
    z.setZero(2 * n, 2 * n);
    z.topLeftCorner(n, n) = z11;
    z.topRightCorner(n, n) = z12;
    z.bottomLeftCorner(n, n) = z21;
    z.bottomRightCorner(n, n) = z22;

    std::vector<Eigen::VectorXd> eigVec_real(2 * n);
    std::vector<Eigen::VectorXd> eigVec_img(2 * n);

    for (int i = 0; i < 8; i++)
    {
        eigVec_real[i].setZero(2 * n);
        eigVec_img[i].setZero(2 * n);
    }

    Eigen::VectorXd deigVal_real(2 * n);
    Eigen::VectorXd deigVal_img(2 * n);
    deigVal_real.setZero();
    deigVal_img.setZero();
    Eigen::MatrixXd deigVec_real(2 * n, 2 * n);
    Eigen::MatrixXd deigVec_img(2 * n, 2 * n);
    deigVec_real.setZero();
    deigVec_img.setZero();

    deigVal_real = z.eigenvalues().real();
    deigVal_img = z.eigenvalues().imag();
    Eigen::EigenSolver<Eigen::MatrixXd> ev(z);

    for (int i = 0; i < 2 * n; i++)
    {
        for (int j = 0; j < 2 * n; j++)
        {
            deigVec_real(j, i) = ev.eigenvectors().col(i)(j).real();
            deigVec_img(j, i) = ev.eigenvectors().col(i)(j).imag();
        }
    }

    //Order the eigenvectors
    //move e-vectors correspnding to e-value outside the unite circle to the left

    Eigen::MatrixXd tempZ_real(2 * n, n), tempZ_img(2 * n, n);
    tempZ_real.setZero();
    tempZ_img.setZero();
    int c = 0;

    for (int i = 0; i < 2 * n; i++)
    {
        if ((deigVal_real(i) * deigVal_real(i) + deigVal_img(i) * deigVal_img(i)) > 1.0) //outside the unit cycle
        {
            for (int j = 0; j < 2 * n; j++)
            {
                tempZ_real(j, c) = deigVec_real(j, i);
                tempZ_img(j, c) = deigVec_img(j, i);
            }
            c++;
        }
    }

    Eigen::MatrixXcd tempZ_comp(2 * n, n);
    for (int i = 0; i < 2 * n; i++)
    {
        for (int j = 0; j < n; j++)
        {
            tempZ_comp.real()(i, j) = tempZ_real(i, j);
            tempZ_comp.imag()(i, j) = tempZ_img(i, j);
        }
    }
    Eigen::MatrixXcd U11(n, n), U21(n, n), X(n, n);
    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < n; j++)
        {
            U11(i, j) = tempZ_comp(i, j);
            U21(i, j) = tempZ_comp(i + n, j);
        }
    }
    X = U21 * (U11.inverse());

    Eigen::MatrixXd X_sol(n, n);
    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < n; j++)
        {
            X_sol(i, j) = X.real()(i, j);
        }
    }

    return X_sol;
}

void CustomController::printOutTextFile()
{
    // if (int( (current_time_ - program_start_time_) * 2000) % 1 == 0) // 2000 hz
    // if ((current_time_ - program_start_time_) >= 0.0)
    // {
    // file[0] << current_time_ - program_start_time_ << "\t" << walking_phase_ << "\t" << foot_contact_ << "\t"
    //         << foot_swing_trigger_ << "\t" << first_step_trigger_ << "\t" << start_walking_trigger_ << "\t"
    //         << stop_walking_trigger_ << "\t" << stance_start_time_ << "\t" << walking_duration_ << "\t"
    //         << turning_duration_ << "\t" << turning_phase_ << "\t" << knee_target_angle_ << endl;

    // file[1] << current_time_ - program_start_time_ << "\t" << walking_phase_ << "\t" << foot_contact_ << "\t"                                   //1
    //         << com_pos_current_from_support_(0) << "\t" << com_pos_current_from_support_(1) << "\t" << com_pos_current_from_support_(2) << "\t" //4
    //         << com_vel_current_from_support_(0) << "\t" << com_vel_current_from_support_(1) << "\t" << com_vel_current_from_support_(2) << "\t" //7
    //         << com_acc_current_from_support_(0) << "\t" << com_acc_current_from_support_(1) << "\t" << com_acc_current_from_support_(2) << "\t" //10
    //         << com_pos_desired_from_support_(0) << "\t" << com_pos_desired_from_support_(1) << "\t" << com_pos_desired_from_support_(2) << "\t" //13
    //         << com_vel_desired_from_support_(0) << "\t" << com_vel_desired_from_support_(1) << "\t" << com_vel_desired_from_support_(2) << "\t" //16
    //         << com_acc_desired_from_support_(0) << "\t" << com_acc_desired_from_support_(1) << "\t" << com_acc_desired_from_support_(2) << endl;

    // file[2]<<ref_zmp_(0,0)<<"\t"<<ref_zmp_(0,1)<<"\t"<< zmp_current_by_com_from_support_(0)<<"\t"<< zmp_current_by_com_from_support_(1)<<endl;

    // for (int i = 0; i < zmp_size_; i++)
    // {
    //     if (i % 10 == 0)
    //         file[2] << ref_zmp_(i, 0) << "\t";
    // }
    // for (int i = 0; i < zmp_size_; i++)
    // {
    //     if (i % 10 == 0)
    //         file[2] << ref_zmp_(i, 1) << "\t";
    // }
    // file[2] << zmp_current_by_com_from_support_(0) << "\t" << zmp_current_by_com_from_support_(1) << endl;

    // file[3] << current_time_ - program_start_time_ << "\t" << walking_phase_ << "\t" << foot_contact_ << "\t"                                                                                                     //1
    //         << lfoot_transform_current_from_support_.translation()(0) << "\t" << lfoot_transform_current_from_support_.translation()(1) << "\t" << lfoot_transform_current_from_support_.translation()(2) << "\t" //4
    //         << rfoot_transform_current_from_support_.translation()(0) << "\t" << rfoot_transform_current_from_support_.translation()(1) << "\t" << rfoot_transform_current_from_support_.translation()(2) << "\t" //7
    //         << swing_foot_pos_trajectory_from_support_(0) << "\t" << swing_foot_pos_trajectory_from_support_(1) << "\t" << swing_foot_pos_trajectory_from_support_(2) << endl;                                    //28

    // file[4]<<current_time_ - program_start_time_<<"\t"<<walking_phase_<<"\t"<<foot_contact_<<"\t"
    // <<rd_.torque_desired (0)<<"\t"<<rd_.torque_desired (1)<<"\t"<<rd_.torque_desired (2)<<"\t"<<rd_.torque_desired (3)<<"\t"<<rd_.torque_desired (4)<<"\t"<<rd_.torque_desired (5)<<"\t"<<rd_.torque_desired (6)<<"\t"<<rd_.torque_desired (7)<<"\t"<<rd_.torque_desired (8)<<"\t"<<rd_.torque_desired (9)<<"\t"<<rd_.torque_desired (10)<<"\t"<<rd_.torque_desired (11)<<"\t"<<rd_.torque_desired (12)<<"\t"<<rd_.torque_desired (13)<<"\t"<<rd_.torque_desired (14)<<"\t"<<rd_.torque_desired (15)<<"\t"<<rd_.torque_desired (16)<<"\t"<<rd_.torque_desired (17)<<"\t"<<rd_.torque_desired (18)<<"\t"<<rd_.torque_desired (19)<<"\t"<<rd_.torque_desired (20)<<"\t"<<rd_.torque_desired (21)<<"\t"<<rd_.torque_desired (22)<<"\t"<<rd_.torque_desired (23)<<"\t"<<rd_.torque_desired (24)<<"\t"<<rd_.torque_desired (25)<<"\t"<<rd_.torque_desired (26)<<"\t"<<rd_.torque_desired (27)<<"\t"<<rd_.torque_desired (28)<<"\t"<<rd_.torque_desired (29)<<"\t"<<rd_.torque_desired (30)<<"\t"<<rd_.torque_desired (31)<<"\t"<<rd_.torque_desired (32)<<endl;
    // file[4] << torque_grav_(0) << "\t" << torque_grav_(1) << "\t" << torque_grav_(2) << "\t" << torque_grav_(3) << "\t" << torque_grav_(4) << "\t" << torque_grav_(5) << "\t" << torque_grav_(6) << "\t" << torque_grav_(7) << "\t" << torque_grav_(8) << "\t" << torque_grav_(9) << "\t" << torque_grav_(10) << "\t" << torque_grav_(11) << endl;

    // file[5] << current_time_ - upperbody_command_time_ << "\t" << walking_phase_ << "\t" << foot_contact_ << "\t"
    //         << desired_q_(0) << "\t" << desired_q_(1) << "\t" << desired_q_(2) << "\t" << desired_q_(3) << "\t" << desired_q_(4) << "\t" << desired_q_(5) << "\t" << desired_q_(6) << "\t" << desired_q_(7) << "\t" << desired_q_(8) << "\t" << desired_q_(9) << "\t" << desired_q_(10) << "\t" << desired_q_(11) << "\t" << desired_q_(12) << "\t" << desired_q_(13) << "\t" << desired_q_(14) << "\t" << desired_q_(15) << "\t" << desired_q_(16) << "\t" << desired_q_(17) << "\t" << desired_q_(18) << "\t" << desired_q_(19) << "\t" << desired_q_(20) << "\t" << desired_q_(21) << "\t" << desired_q_(22) << "\t" << desired_q_(23) << "\t" << desired_q_(24) << "\t" << desired_q_(25) << "\t" << desired_q_(26) << "\t" << desired_q_(27) << "\t" << desired_q_(28) << "\t" << desired_q_(29) << "\t" << desired_q_(30) << "\t" << desired_q_(31) << "\t" << desired_q_(32) << "\t"
    //         << current_q_(0) << "\t" << current_q_(1) << "\t" << current_q_(2) << "\t" << current_q_(3) << "\t" << current_q_(4) << "\t" << current_q_(5) << "\t" << current_q_(6) << "\t" << current_q_(7) << "\t" << current_q_(8) << "\t" << current_q_(9) << "\t" << current_q_(10) << "\t" << current_q_(11) << "\t" << current_q_(12) << "\t" << current_q_(13) << "\t" << current_q_(14) << "\t" << current_q_(15) << "\t" << current_q_(16) << "\t" << current_q_(17) << "\t" << current_q_(18) << "\t" << current_q_(19) << "\t" << current_q_(20) << "\t" << current_q_(21) << "\t" << current_q_(22) << "\t" << current_q_(23) << "\t" << current_q_(24) << "\t" << current_q_(25) << "\t" << current_q_(26) << "\t" << current_q_(27) << "\t" << current_q_(28) << "\t" << current_q_(29) << "\t" << current_q_(30) << "\t" << current_q_(31) << "\t" << current_q_(32) << "\t"
    //         << desired_q_dot_(0) << "\t" << desired_q_dot_(1) << "\t" << desired_q_dot_(2) << "\t" << desired_q_dot_(3) << "\t" << desired_q_dot_(4) << "\t" << desired_q_dot_(5) << "\t" << desired_q_dot_(6) << "\t" << desired_q_dot_(7) << "\t" << desired_q_dot_(8) << "\t" << desired_q_dot_(9) << "\t" << desired_q_dot_(10) << "\t" << desired_q_dot_(11) << "\t" << desired_q_dot_(12) << "\t" << desired_q_dot_(13) << "\t" << desired_q_dot_(14) << "\t" << desired_q_dot_(15) << "\t" << desired_q_dot_(16) << "\t" << desired_q_dot_(17) << "\t" << desired_q_dot_(18) << "\t" << desired_q_dot_(19) << "\t" << desired_q_dot_(20) << "\t" << desired_q_dot_(21) << "\t" << desired_q_dot_(22) << "\t" << desired_q_dot_(23) << "\t" << desired_q_dot_(24) << "\t" << desired_q_dot_(25) << "\t" << desired_q_dot_(26) << "\t" << desired_q_dot_(27) << "\t" << desired_q_dot_(28) << "\t" << desired_q_dot_(29) << "\t" << desired_q_dot_(30) << "\t" << desired_q_dot_(31) << "\t" << desired_q_dot_(32) << "\t"
    //         << current_q_dot_(0) << "\t" << current_q_dot_(1) << "\t" << current_q_dot_(2) << "\t" << current_q_dot_(3) << "\t" << current_q_dot_(4) << "\t" << current_q_dot_(5) << "\t" << current_q_dot_(6) << "\t" << current_q_dot_(7) << "\t" << current_q_dot_(8) << "\t" << current_q_dot_(9) << "\t" << current_q_dot_(10) << "\t" << current_q_dot_(11) << "\t" << current_q_dot_(12) << "\t" << current_q_dot_(13) << "\t" << current_q_dot_(14) << "\t" << current_q_dot_(15) << "\t" << current_q_dot_(16) << "\t" << current_q_dot_(17) << "\t" << current_q_dot_(18) << "\t" << current_q_dot_(19) << "\t" << current_q_dot_(20) << "\t" << current_q_dot_(21) << "\t" << current_q_dot_(22) << "\t" << current_q_dot_(23) << "\t" << current_q_dot_(24) << "\t" << current_q_dot_(25) << "\t" << current_q_dot_(26) << "\t" << current_q_dot_(27) << "\t" << current_q_dot_(28) << "\t" << current_q_dot_(29) << "\t" << current_q_dot_(30) << "\t" << current_q_dot_(31) << "\t" << current_q_dot_(32) << endl;

    // file[5] << rd_.control_time_ << "t";
    // for(int i = 0; i<40; i++)
    // {
    //     file[5] << rd_.q_virtual_(i) << "t";
    // }
    // for(int i = 0; i<39; i++)
    // {
    //     file[5] << rd_.q_dot_virtual_(i) << "t" ;
    // }
    // for(int i = 0; i<33; i++)
    // {
    //     file[5] << rd_.torque_desired(i) << "t" ;
    // }
    // file[5] << endl;
    // for(int i = 0; i<5; i++)
    // {
    //     file[5] << rd_.q_ddot_virtual_(i) << "t" ;
    // }
    // file[5] << rd_.q_ddot_virtual_(5) << endl;

    // file[6]
    // <<current_time_ - upperbody_command_time_ << lhand_transform_current_from_global_.translation()(0)<<"\t"<<lhand_transform_current_from_global_.translation()(1)<<"\t"<<lhand_transform_current_from_global_.translation()(2)<<"\t"<<rhand_transform_current_from_global_.translation()(0)<<"\t"<<rhand_transform_current_from_global_.translation()(1)<<"\t"<<rhand_transform_current_from_global_.translation()(2)<<"\t"
    // <<lhand_rpy_current_from_global_(0)<<"\t"<<lhand_rpy_current_from_global_(1)<<"\t"<<lhand_rpy_current_from_global_(2)<<"\t"<<rhand_rpy_current_from_global_(0)<<"\t"<<rhand_rpy_current_from_global_(1)<<"\t"<<rhand_rpy_current_from_global_(2)<<"\t"
    // <<lhand_transform_pre_desired_from_.translation()(0)<<"\t"<<lhand_transform_pre_desired_from_.translation()(1)<<"\t"<<lhand_transform_pre_desired_from_.translation()(2)<<"\t"<<rhand_transform_pre_desired_from_.translation()(0)<<"\t"<<rhand_transform_pre_desired_from_.translation()(1)<<"\t"<<rhand_transform_pre_desired_from_.translation()(2)<<endl;

    // file[7]
    // <<lelbow_transform_current_from_global_.translation()(0)<<"\t"<<lelbow_transform_current_from_global_.translation()(1)<<"\t"<<lelbow_transform_current_from_global_.translation()(2)<<"\t"<<relbow_transform_current_from_global_.translation()(0)<<"\t"<<relbow_transform_current_from_global_.translation()(1)<<"\t"<<relbow_transform_current_from_global_.translation()(2)<<"\t"
    // <<lelbow_rpy_current_from_global_(0)<<"\t"<<lelbow_rpy_current_from_global_(1)<<"\t"<<lelbow_rpy_current_from_global_(2)<<"\t"<<relbow_rpy_current_from_global_(0)<<"\t"<<relbow_rpy_current_from_global_(1)<<"\t"<<relbow_rpy_current_from_global_(2)<<"\t"
    // <<lelbow_vel_current_from_global_(0)<<"\t"<<lelbow_vel_current_from_global_(1)<<"\t"<<lelbow_vel_current_from_global_(2)<<"\t"<<relbow_vel_current_from_global_(0)<<"\t"<<relbow_vel_current_from_global_(1)<<"\t"<<relbow_vel_current_from_global_(2)<<endl;

    // file[8]
    // <<lshoulder_transform_current_from_global_.translation()(0)<<"\t"<<lshoulder_transform_current_from_global_.translation()(1)<<"\t"<<lshoulder_transform_current_from_global_.translation()(2)<<"\t"<<rshoulder_transform_current_from_global_.translation()(0)<<"\t"<<rshoulder_transform_current_from_global_.translation()(1)<<"\t"<<rshoulder_transform_current_from_global_.translation()(2)<<"\t"
    // <<lshoulder_rpy_current_from_global_(0)<<"\t"<<lshoulder_rpy_current_from_global_(1)<<"\t"<<lshoulder_rpy_current_from_global_(2)<<"\t"<<rshoulder_rpy_current_from_global_(0)<<"\t"<<rshoulder_rpy_current_from_global_(1)<<"\t"<<rshoulder_rpy_current_from_global_(2)<<"\t"
    // <<lshoulder_vel_current_from_global_(0)<<"\t"<<lshoulder_vel_current_from_global_(1)<<"\t"<<lshoulder_vel_current_from_global_(2)<<"\t"<<rshoulder_vel_current_from_global_(0)<<"\t"<<rshoulder_vel_current_from_global_(1)<<"\t"<<rshoulder_vel_current_from_global_(2)<<endl;

    // file[9]
    // <<lacromion_transform_current_from_global_.translation()(0)<<"\t"<<lacromion_transform_current_from_global_.translation()(1)<<"\t"<<lacromion_transform_current_from_global_.translation()(2)<<"\t"<<racromion_transform_current_from_global_.translation()(0)<<"\t"<<racromion_transform_current_from_global_.translation()(1)<<"\t"<<racromion_transform_current_from_global_.translation()(2)<<"\t"
    // <<lacromion_rpy_current_from_global_(0)<<"\t"<<lacromion_rpy_current_from_global_(1)<<"\t"<<lacromion_rpy_current_from_global_(2)<<"\t"<<racromion_rpy_current_from_global_(0)<<"\t"<<racromion_rpy_current_from_global_(1)<<"\t"<<racromion_rpy_current_from_global_(2)<<"\t"
    // <<lacromion_vel_current_from_global_(0)<<"\t"<<lacromion_vel_current_from_global_(1)<<"\t"<<lacromion_vel_current_from_global_(2)<<"\t"<<racromion_vel_current_from_global_(0)<<"\t"<<racromion_vel_current_from_global_(1)<<"\t"<<racromion_vel_current_from_global_(2)<<endl;

    // file[11]
    //     << hmd_lhand_pose_.translation()(0) << "\t" << hmd_lhand_pose_.translation()(1) << "\t" << hmd_lhand_pose_.translation()(2) << "\t" << hmd_rhand_pose_.translation()(0) << "\t" << hmd_rhand_pose_.translation()(1) << "\t" << hmd_rhand_pose_.translation()(2) << "\t"
    //     << master_lhand_pose_raw_.translation()(0) << "\t" << master_lhand_pose_raw_.translation()(1) << "\t" << master_lhand_pose_raw_.translation()(2) << "\t" << master_rhand_pose_raw_.translation()(0) << "\t" << master_rhand_pose_raw_.translation()(1) << "\t" << master_rhand_pose_raw_.translation()(2) << endl;
    // << master_lhand_pose_.translation()(0) << "\t" << master_lhand_pose_.translation()(1) << "\t" << master_lhand_pose_.translation()(2) << "\t" << master_rhand_pose_.translation()(0) << "\t" << master_rhand_pose_.translation()(1) << "\t" << master_rhand_pose_.translation()(2) << endl;
    // << master_lhand_rqy_(0) << "\t" << master_lhand_rqy_(1) << "\t" << master_lhand_rqy_(2) << "\t" << master_rhand_rqy_(0) << "\t" << master_rhand_rqy_(1) << "\t" << master_rhand_rqy_(2) << "\t"
    // << hmd_lupperarm_pose_.translation()(0) << "\t" << hmd_lupperarm_pose_.translation()(1) << "\t" << hmd_lupperarm_pose_.translation()(2) << "\t" << hmd_rupperarm_pose_.translation()(0) << "\t" << hmd_rupperarm_pose_.translation()(1) << "\t" << hmd_rupperarm_pose_.translation()(2) << "\t"
    // << master_lelbow_pose_raw_.translation()(0) << "\t" << master_lelbow_pose_raw_.translation()(1) << "\t" << master_lelbow_pose_raw_.translation()(2) << "\t" << master_relbow_pose_raw_.translation()(0) << "\t" << master_relbow_pose_raw_.translation()(1) << "\t" << master_relbow_pose_raw_.translation()(2) << "\t"
    // << master_lelbow_pose_.translation()(0) << "\t" << master_lelbow_pose_.translation()(1) << "\t" << master_lelbow_pose_.translation()(2) << "\t" << master_relbow_pose_.translation()(0) << "\t" << master_relbow_pose_.translation()(1) << "\t" << master_relbow_pose_.translation()(2) << "\t"
    // << master_lelbow_rqy_(0) << "\t" << master_lelbow_rqy_(1) << "\t" << master_lelbow_rqy_(2) << "\t" << master_relbow_rqy_(0) << "\t" << master_relbow_rqy_(1) << "\t" << master_relbow_rqy_(2) << "\t"
    // << hmd_lshoulder_pose_.translation()(0) << "\t" << hmd_lshoulder_pose_.translation()(1) << "\t" << hmd_lshoulder_pose_.translation()(2) << "\t" << hmd_rshoulder_pose_.translation()(0) << "\t" << hmd_rshoulder_pose_.translation()(1) << "\t" << hmd_rshoulder_pose_.translation()(2) << "\t"
    // << master_lshoulder_pose_raw_.translation()(0) << "\t" << master_lshoulder_pose_raw_.translation()(1) << "\t" << master_lshoulder_pose_raw_.translation()(2) << "\t" << master_rshoulder_pose_raw_.translation()(0) << "\t" << master_rshoulder_pose_raw_.translation()(1) << "\t" << master_rshoulder_pose_raw_.translation()(2) << "\t"
    // << master_lshoulder_pose_.translation()(0) << "\t" << master_lshoulder_pose_.translation()(1) << "\t" << master_lshoulder_pose_.translation()(2) << "\t" << master_rshoulder_pose_.translation()(0) << "\t" << master_rshoulder_pose_.translation()(1) << "\t" << master_rshoulder_pose_.translation()(2) << "\t"
    // << master_lshoulder_rqy_(0) << "\t" << master_lshoulder_rqy_(1) << "\t" << master_lshoulder_rqy_(2) << "\t" << master_rshoulder_rqy_(0) << "\t" << master_rshoulder_rqy_(1) << "\t" << master_rshoulder_rqy_(2) << "\t"
    // << hmd_head_pose_.translation()(0) << "\t" << hmd_head_pose_.translation()(1) << "\t" << hmd_head_pose_.translation()(2) << "\t"
    // << master_head_pose_raw_.translation()(0) << "\t" << master_head_pose_raw_.translation()(1) << "\t" << master_head_pose_raw_.translation()(2) << "\t"
    // << master_head_pose_.translation()(0) << "\t" << master_head_pose_.translation()(1) << "\t" << master_head_pose_.translation()(2) << "\t"
    // << master_head_rqy_(0) << "\t" << master_head_rqy_(1) << "\t" << master_head_rqy_(2) << "\t"
    // << hmd_pelv_pose_.translation()(0) << "\t" << hmd_pelv_pose_.translation()(1) << "\t" << hmd_pelv_pose_.translation()(2) << endl;

    // file[12]
    //     << lhand_pos_error_(0) << "\t" << lhand_pos_error_(1) << "\t" << lhand_pos_error_(2) << "\t" << rhand_pos_error_(0) << "\t" << rhand_pos_error_(1) << "\t" << rhand_pos_error_(2) << "\t"
    //     << lhand_ori_error_(0) << "\t" << lhand_ori_error_(1) << "\t" << lhand_ori_error_(2) << "\t" << rhand_ori_error_(0) << "\t" << rhand_ori_error_(1) << "\t" << rhand_ori_error_(2) << "\t"
    //     << lelbow_ori_error_(0) << "\t" << lelbow_ori_error_(1) << "\t" << lelbow_ori_error_(2) << "\t" << relbow_ori_error_(0) << "\t" << relbow_ori_error_(1) << "\t" << relbow_ori_error_(2) << "\t"
    //     << lshoulder_ori_error_(0) << "\t" << lshoulder_ori_error_(1) << "\t" << lshoulder_ori_error_(2) << "\t" << rshoulder_ori_error_(0) << "\t" << rshoulder_ori_error_(1) << "\t" << rshoulder_ori_error_(2) << "\t"
    //     << lhand_vel_error_(0) << "\t" << lhand_vel_error_(1) << "\t" << lhand_vel_error_(2) << "\t" << lhand_vel_error_(3) << "\t" << lhand_vel_error_(4) << "\t" << lhand_vel_error_(5) << "\t"
    //     << rhand_vel_error_(0) << "\t" << rhand_vel_error_(1) << "\t" << rhand_vel_error_(2) << "\t" << rhand_vel_error_(3) << "\t" << rhand_vel_error_(4) << "\t" << rhand_vel_error_(5) << "\t"
    //     << lelbow_vel_error_(0) << "\t" << lelbow_vel_error_(1) << "\t" << lelbow_vel_error_(2) << "\t" << relbow_vel_error_(0) << "\t" << relbow_vel_error_(1) << "\t" << relbow_vel_error_(2) << "\t"
    //     << lacromion_vel_error_(0) << "\t" << lacromion_vel_error_(1) << "\t" << lacromion_vel_error_(2) << "\t" << racromion_vel_error_(0) << "\t" << racromion_vel_error_(1) << "\t" << racromion_vel_error_(2) << endl;

    // file[13]
    // <<hmd_head_vel_(0)<<"\t"<<hmd_head_vel_(1)<<"\t"<<hmd_head_vel_(2)<<"\t"<<hmd_head_vel_(3)<<"\t"<<hmd_head_vel_(4)<<"\t"<<hmd_head_vel_(5)<<"\t"
    // <<hmd_lshoulder_vel_(0)<<"\t"<<hmd_lshoulder_vel_(1)<<"\t"<<hmd_lshoulder_vel_(2)<<"\t"<<hmd_lshoulder_vel_(3)<<"\t"<<hmd_lshoulder_vel_(4)<<"\t"<<hmd_lshoulder_vel_(5)<<"\t"
    // <<hmd_lupperarm_vel_(0)<<"\t"<<hmd_lupperarm_vel_(1)<<"\t"<<hmd_lupperarm_vel_(2)<<"\t"<<hmd_lupperarm_vel_(3)<<"\t"<<hmd_lupperarm_vel_(4)<<"\t"<<hmd_lupperarm_vel_(5)<<"\t"
    // <<hmd_lhand_vel_(0)<<"\t"<<hmd_lhand_vel_(1)<<"\t"<<hmd_lhand_vel_(2)<<"\t"<<hmd_lhand_vel_(3)<<"\t"<<hmd_lhand_vel_(4)<<"\t"<<hmd_lhand_vel_(5)<<"\t"
    // <<hmd_rshoulder_vel_(0)<<"\t"<<hmd_rshoulder_vel_(1)<<"\t"<<hmd_rshoulder_vel_(2)<<"\t"<<hmd_rshoulder_vel_(3)<<"\t"<<hmd_rshoulder_vel_(4)<<"\t"<<hmd_rshoulder_vel_(5)<<"\t"
    // <<hmd_rupperarm_vel_(0)<<"\t"<<hmd_rupperarm_vel_(1)<<"\t"<<hmd_rupperarm_vel_(2)<<"\t"<<hmd_rupperarm_vel_(3)<<"\t"<<hmd_rupperarm_vel_(4)<<"\t"<<hmd_rupperarm_vel_(5)<<"\t"
    // <<hmd_rhand_vel_(0)<<"\t"<<hmd_rhand_vel_(1)<<"\t"<<hmd_rhand_vel_(2)<<"\t"<<hmd_rhand_vel_(3)<<"\t"<<hmd_rhand_vel_(4)<<"\t"<<hmd_rhand_vel_(5)<<"\t"
    // <<hmd_chest_vel_(0)<<"\t"<<hmd_chest_vel_(1)<<"\t"<<hmd_chest_vel_(2)<<"\t"<<hmd_chest_vel_(3)<<"\t"<<hmd_chest_vel_(4)<<"\t"<<hmd_chest_vel_(5)<<"\t"
    // <<hmd_pelv_vel_(0)<<"\t"<<hmd_pelv_vel_(1)<<"\t"<<hmd_pelv_vel_(2)<<"\t"<<hmd_pelv_vel_(3)<<"\t"<<hmd_pelv_vel_(4)<<"\t"<<hmd_pelv_vel_(5)<<endl;

    // }
}

//////////////////////////////MJ's Functions////////////////////
void CustomController::PedalCommandCallback(const tocabi_msgs::WalkingCommandConstPtr &msg)
{
    if (joy_input_enable_ == true)
    {
        joystick_input(0) = DyrosMath::minmax_cut(2 * (msg->step_length_x), 0.0, 2.0) - 1.0; //FW
        joystick_input(2) = DyrosMath::minmax_cut(2 * (msg->theta) - DyrosMath::sign(msg->theta), -0.5 + 0.5 * DyrosMath::sign(msg->theta), 0.5 + 0.5 * DyrosMath::sign(msg->theta));
        // joystick_input(2) = msg->theta;
        joystick_input(3) = DyrosMath::minmax_cut(2 * (msg->z), 0.0, 2.0) - 1.0; //BW
        joystick_input(1) = (joystick_input(0) + 1) / 2 + abs(joystick_input(2)) + (joystick_input(3) + 1) / 2;
    }
    else
    {
        joystick_input(1) = -1.0;
    }

    if (joystick_input(1) > 0.0001)
    {
        walking_enable_ = true;
        walking_end_flag = 1;
        // cout<<"walking triggered!!"<<endl;
    }
}

void CustomController::updateInitialState()
{
    if (walking_tick_mj == 0)
    {
        //calculateFootStepTotal();

        pelv_rpy_current_mj_.setZero();
        pelv_rpy_current_mj_ = DyrosMath::rot2Euler(rd_.link_[Pelvis].rotm); //ZYX multiply

        pelv_yaw_rot_current_from_global_mj_ = DyrosMath::rotateWithZ(pelv_rpy_current_mj_(2));

        pelv_float_init_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_) * rd_.link_[Pelvis].rotm;

        pelv_float_init_.translation() = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_), rd_.link_[Pelvis].xpos);
        //pelv_float_init_.translation()(0) += 0.11;

        pelv_float_init_.translation()(0) = 0;
        pelv_float_init_.translation()(1) = 0;

        lfoot_float_init_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_) * rd_.link_[Left_Foot].rotm;
        lfoot_float_init_.translation() = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_), rd_.link_[Left_Foot].xpos); // 지면에서 Ankle frame 위치

        //lfoot_float_init_.translation()(0) = 0;
        //lfoot_float_init_.translation()(1) = 0.1225;

        rfoot_float_init_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_) * rd_.link_[Right_Foot].rotm;
        rfoot_float_init_.translation() = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_), rd_.link_[Right_Foot].xpos); // 지면에서 Ankle frame

        //rfoot_float_init_.translation()(0) = 0;
        //rfoot_float_init_.translation()(1) = -0.1225;

        com_float_init_ = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_), rd_.link_[COM_id].xpos); // 지면에서 CoM 위치

        if (aa == 0)
        {
            //lfoot_float_init_.translation()(1) = 0.1025;
            //rfoot_float_init_.translation()(1) = -0.1025;
            aa = 1;
        }
        cout << "First " << pelv_float_init_.translation()(0) << "," << lfoot_float_init_.translation()(0) << "," << rfoot_float_init_.translation()(0) << "," << pelv_rpy_current_mj_(2) * 180 / 3.141592 << endl;

        Eigen::Isometry3d ref_frame;

        calculateFootStepTotal_MJ();

        if (foot_step_(0, 6) == 0) //right foot support
        {
            ref_frame = rfoot_float_init_;
        }
        else if (foot_step_(0, 6) == 1)
        {
            ref_frame = lfoot_float_init_;
        }

        Eigen::Isometry3d ref_frame_yaw_only;
        ref_frame_yaw_only.translation() = ref_frame.translation();
        Eigen::Vector3d ref_frame_rpy;
        ref_frame_rpy = DyrosMath::rot2Euler(ref_frame.linear());
        ref_frame_yaw_only.linear() = DyrosMath::rotateWithZ(ref_frame_rpy(2));

        pelv_support_init_ = DyrosMath::inverseIsometry3d(ref_frame_yaw_only) * pelv_float_init_;
        com_support_init_ = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(ref_frame_yaw_only), com_float_init_);
        pelv_support_euler_init_ = DyrosMath::rot2Euler(pelv_support_init_.linear());

        lfoot_support_init_ = DyrosMath::multiplyIsometry3d(DyrosMath::inverseIsometry3d(ref_frame_yaw_only), lfoot_float_init_);
        rfoot_support_init_ = DyrosMath::multiplyIsometry3d(DyrosMath::inverseIsometry3d(ref_frame_yaw_only), rfoot_float_init_);
        rfoot_support_euler_init_ = DyrosMath::rot2Euler(rfoot_support_init_.linear());
        lfoot_support_euler_init_ = DyrosMath::rot2Euler(lfoot_support_init_.linear());

        supportfoot_float_init_.setZero();
        swingfoot_float_init_.setZero();

        if (foot_step_(0, 6) == 1) //left suppport foot
        {
            for (int i = 0; i < 2; i++)
                supportfoot_float_init_(i) = lfoot_float_init_.translation()(i);
            for (int i = 0; i < 3; i++)
                supportfoot_float_init_(i + 3) = DyrosMath::rot2Euler(lfoot_float_init_.linear())(i);

            for (int i = 0; i < 2; i++)
                swingfoot_float_init_(i) = rfoot_float_init_.translation()(i);
            for (int i = 0; i < 3; i++)
                swingfoot_float_init_(i + 3) = DyrosMath::rot2Euler(rfoot_float_init_.linear())(i);

            supportfoot_float_init_(0) = 0.0;
            swingfoot_float_init_(0) = 0.0;
        }
        else
        {
            for (int i = 0; i < 2; i++)
                supportfoot_float_init_(i) = rfoot_float_init_.translation()(i);
            for (int i = 0; i < 3; i++)
                supportfoot_float_init_(i + 3) = DyrosMath::rot2Euler(rfoot_float_init_.linear())(i);

            for (int i = 0; i < 2; i++)
                swingfoot_float_init_(i) = lfoot_float_init_.translation()(i);
            for (int i = 0; i < 3; i++)
                swingfoot_float_init_(i + 3) = DyrosMath::rot2Euler(lfoot_float_init_.linear())(i);

            supportfoot_float_init_(0) = 0.0;
            swingfoot_float_init_(0) = 0.0;
        }

        pelv_support_start_ = pelv_support_init_;
        // cout<<"pelv_support_start_.translation()(2): "<<pelv_support_start_.translation()(2);
        total_step_num_ = foot_step_.col(1).size();

        xi_mj_ = com_support_init_(0); // preview parameter
        yi_mj_ = com_support_init_(1);
        zc_mj_ = com_support_init_(2);

        std::cout << "pelv_yaw_rot_current_from_global_mj_ " << std::endl;
        std::cout << pelv_yaw_rot_current_from_global_mj_.linear() << std::endl;
        std::cout << pelv_yaw_rot_current_from_global_mj_.translation() << std::endl;
    }
    else if (current_step_num_ != 0 && walking_tick_mj == t_start_) // step change
    {
        std::cout << "INIT" << walking_tick_mj << std::endl;
        pelv_rpy_current_mj_.setZero();
        pelv_rpy_current_mj_ = DyrosMath::rot2Euler(rd_.link_[Pelvis].rotm); //ZYX multiply

        pelv_yaw_rot_current_from_global_mj_ = DyrosMath::rotateWithZ(pelv_rpy_current_mj_(2));

        rfoot_rpy_current_.setZero();
        lfoot_rpy_current_.setZero();
        rfoot_rpy_current_ = DyrosMath::rot2Euler(rd_.link_[Right_Foot].rotm);
        lfoot_rpy_current_ = DyrosMath::rot2Euler(rd_.link_[Left_Foot].rotm);

        rfoot_roll_rot_ = DyrosMath::rotateWithX(rfoot_rpy_current_(0));
        lfoot_roll_rot_ = DyrosMath::rotateWithX(lfoot_rpy_current_(0));
        rfoot_pitch_rot_ = DyrosMath::rotateWithY(rfoot_rpy_current_(1));
        lfoot_pitch_rot_ = DyrosMath::rotateWithY(lfoot_rpy_current_(1));

        pelv_float_init_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_) * rd_.link_[Pelvis].rotm;

        pelv_float_init_.translation() = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_), rd_.link_[Pelvis].xpos);

        lfoot_float_init_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_) * rd_.link_[Left_Foot].rotm;
        // lfoot_float_init_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_) * DyrosMath::inverseIsometry3d(lfoot_pitch_rot_) * DyrosMath::inverseIsometry3d(lfoot_roll_rot_) * rd_.link_[Left_Foot].rotm;
        lfoot_float_init_.translation() = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_), rd_.link_[Left_Foot].xpos); // 지면에서 Ankle frame 위치

        rfoot_float_init_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_) * rd_.link_[Right_Foot].rotm;
        // rfoot_float_init_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_) * DyrosMath::inverseIsometry3d(rfoot_pitch_rot_) * DyrosMath::inverseIsometry3d(rfoot_roll_rot_) * rd_.link_[Right_Foot].rotm;
        rfoot_float_init_.translation() = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_), rd_.link_[Right_Foot].xpos); // 지면에서 Ankle frame

        com_float_init_ = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_), rd_.link_[COM_id].xpos); // 지면에서 CoM 위치

        Eigen::Isometry3d ref_frame;

        if (foot_step_(current_step_num_, 6) == 0) //right foot support
        {
            ref_frame = rfoot_float_init_;
        }
        else if (foot_step_(current_step_num_, 6) == 1)
        {
            ref_frame = lfoot_float_init_;
        }
        /////dg edit
        Eigen::Isometry3d ref_frame_yaw_only;
        ref_frame_yaw_only.translation() = ref_frame.translation();
        Eigen::Vector3d ref_frame_rpy;
        ref_frame_rpy = DyrosMath::rot2Euler(ref_frame.linear());
        ref_frame_yaw_only.linear() = DyrosMath::rotateWithZ(ref_frame_rpy(2));

        pelv_support_init_ = DyrosMath::inverseIsometry3d(ref_frame_yaw_only) * pelv_float_init_;
        com_support_init_ = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(ref_frame_yaw_only), com_float_init_);
        pelv_support_euler_init_ = DyrosMath::rot2Euler(pelv_support_init_.linear());

        lfoot_support_init_ = DyrosMath::multiplyIsometry3d(DyrosMath::inverseIsometry3d(ref_frame_yaw_only), lfoot_float_init_);
        rfoot_support_init_ = DyrosMath::multiplyIsometry3d(DyrosMath::inverseIsometry3d(ref_frame_yaw_only), rfoot_float_init_);
        rfoot_support_euler_init_ = DyrosMath::rot2Euler(rfoot_support_init_.linear());
        lfoot_support_euler_init_ = DyrosMath::rot2Euler(lfoot_support_init_.linear());
    }
}

void CustomController::getRobotState()
{
    pelv_rpy_current_mj_.setZero();
    pelv_rpy_current_mj_ = DyrosMath::rot2Euler(rd_.link_[Pelvis].rotm); //ZYX multiply

    R_angle = pelv_rpy_current_mj_(0);
    P_angle = pelv_rpy_current_mj_(1);
    pelv_yaw_rot_current_from_global_mj_ = DyrosMath::rotateWithZ(pelv_rpy_current_mj_(2));

    rfoot_rpy_current_.setZero();
    lfoot_rpy_current_.setZero();
    rfoot_rpy_current_ = DyrosMath::rot2Euler(rd_.link_[Right_Foot].rotm);
    lfoot_rpy_current_ = DyrosMath::rot2Euler(rd_.link_[Left_Foot].rotm);

    rfoot_roll_rot_ = DyrosMath::rotateWithX(rfoot_rpy_current_(0));
    lfoot_roll_rot_ = DyrosMath::rotateWithX(lfoot_rpy_current_(0));
    rfoot_pitch_rot_ = DyrosMath::rotateWithY(rfoot_rpy_current_(1));
    lfoot_pitch_rot_ = DyrosMath::rotateWithY(lfoot_rpy_current_(1));

    pelv_float_current_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_) * rd_.link_[Pelvis].rotm;
    pelv_float_current_.translation() = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_), rd_.link_[Pelvis].xpos);

    lfoot_float_current_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_) * rd_.link_[Left_Foot].rotm;
    lfoot_float_current_.translation() = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_), rd_.link_[Left_Foot].xpos); // 지면에서 Ankle frame 위치

    rfoot_float_current_.linear() = DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_) * rd_.link_[Right_Foot].rotm;
    rfoot_float_current_.translation() = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_), rd_.link_[Right_Foot].xpos); // 지면에서 Ankle frame
   
   
    PELV_YaW6D.setZero();
    PELV_YaW6D.block(0,0,3,3) = pelv_yaw_rot_current_from_global_mj_.linear().transpose();
    PELV_YaW6D.block(3,3,3,3) = PELV_YaW6D.block(0,0,3,3);
    J_LFF = PELV_YaW6D * (rd_.link_[Left_Foot].jac.cast<double>());
    J_RFF = PELV_YaW6D * (rd_.link_[Right_Foot].jac.cast<double>());
    comj = PELV_YaW6D * (rd_.link_[COM_id].jac.cast<double>());

    if(contactMode == 1)
    {
        zero_center(0) = (lfoot_float_current_.translation()(0) + rfoot_float_current_.translation()(0))/2;
        zero_center(1) = (lfoot_float_current_.translation()(1) + rfoot_float_current_.translation()(1))/2;
        zero_center(2) = (lfoot_float_current_.translation()(2) + rfoot_float_current_.translation()(2))/2;
    }
    else if(contactMode == 2)
    {
        zero_center(0) = (lfoot_float_current_.translation()(0)) - 0.08;//Reivse
        zero_center(1) = (lfoot_float_current_.translation()(1) - 0.1025);
        zero_center(2) = (lfoot_float_current_.translation()(2));  
    }
    else
    {
        zero_center(0) = (rfoot_float_current_.translation()(0)) - 0.08;//Reivse
        zero_center(1) = (rfoot_float_current_.translation()(1) - (-0.1025));
        zero_center(2) = (rfoot_float_current_.translation()(2));  
    }

    com_float_current_ = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_), rd_.link_[COM_id].xpos); // 지면에서 CoM 위치
    com_float_current_dot = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(pelv_yaw_rot_current_from_global_mj_), rd_.link_[COM_id].v);

    if (walking_tick_mj == 0)
    {
        com_float_current_dot_LPF = com_float_current_dot;
        com_float_current_dot_prev = com_float_current_dot;
    }

    com_float_current_dot_prev = com_float_current_dot;
    com_float_current_dot_LPF = 1 / (1 + 2 * M_PI * 6.0 * del_t) * com_float_current_dot_LPF + (2 * M_PI * 8.0 * del_t) / (1 + 2 * M_PI * 6.0 * del_t) * com_float_current_dot;

    if (walking_tick_mj == 0)
    {
        com_float_current_LPF = com_float_current_;
    }

    com_float_current_LPF = 1 / (1 + 2 * M_PI * 8.0 * del_t) * com_float_current_LPF + (2 * M_PI * 8.0 * del_t) / (1 + 2 * M_PI * 8.0 * del_t) * com_float_current_;

    double support_foot_flag = foot_step_(current_step_num_, 6);

    if (support_foot_flag == 0)
    {
        supportfoot_float_current_ = rfoot_float_current_;
    }
    else if (support_foot_flag == 1)
    {
        supportfoot_float_current_ = lfoot_float_current_;
    }

    ///////////dg edit
    supportfoot_float_current_yaw_only.translation() = supportfoot_float_current_.translation();
    Eigen::Vector3d support_foot_current_rpy;
    support_foot_current_rpy = DyrosMath::rot2Euler(supportfoot_float_current_.linear());
    supportfoot_float_current_yaw_only.linear() = DyrosMath::rotateWithZ(support_foot_current_rpy(2));
   
    pelv_support_current_ = DyrosMath::inverseIsometry3d(supportfoot_float_current_yaw_only) * pelv_float_current_;
    lfoot_support_current_ = DyrosMath::inverseIsometry3d(supportfoot_float_current_yaw_only) * lfoot_float_current_;
    rfoot_support_current_ = DyrosMath::inverseIsometry3d(supportfoot_float_current_yaw_only) * rfoot_float_current_;
   
    /*lfoot_sx = lfoot_support_current_.translation()(0) + 0.03;
    lfoot_sy  = lfoot_support_current_.translation()(1);
    rfoot_sx = rfoot_support_current_.translation()(0) + 0.03;
    rfoot_sy  = rfoot_support_current_.translation()(1);*/

    //cout << "L : " << lfoot_float_current_.linear() << "\n" <<  "R : " << rfoot_float_current_.linear() << endl;
    com_support_current_ = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(supportfoot_float_current_yaw_only), com_float_current_);
    com_support_current_dot_ = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(supportfoot_float_current_yaw_only), com_float_current_dot);
    com_support_current_dot_LPF = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(supportfoot_float_current_yaw_only), com_float_current_dot_LPF);

    sup_YaW6D.setZero();
    sup_YaW6D.block(0,0,3,3) = supportfoot_float_current_yaw_only.linear().transpose();
    sup_YaW6D.block(3,3,3,3) = PELV_YaW6D.block(0,0,3,3);
   
    J_LFF = sup_YaW6D * J_LFF;
    J_RFF = sup_YaW6D * J_RFF;
    comj = sup_YaW6D * comj;

    if(walking_tick_mj == 0)
    {
        pelv_support_current_init = pelv_support_current_;
        lfoot_support_current_init = lfoot_support_current_;
        rfoot_support_current_init = rfoot_support_current_;
        com_support_current_init.translation() = com_support_current_;
    }

    // l_ft : generated force by robot
    l_ft_ = rd_.LF_FT;
    r_ft_ = rd_.RF_FT;

    if (walking_tick_mj == 0)
    {
        l_ft_LPF = l_ft_;
        r_ft_LPF = r_ft_;
    }

    l_ft_LPF = 1 / (1 + 2 * M_PI * 6.0 * del_t) * l_ft_LPF + (2 * M_PI * 6.0 * del_t) / (1 + 2 * M_PI * 6.0 * del_t) * l_ft_;
    r_ft_LPF = 1 / (1 + 2 * M_PI * 6.0 * del_t) * r_ft_LPF + (2 * M_PI * 6.0 * del_t) / (1 + 2 * M_PI * 6.0 * del_t) * r_ft_;


    left_zmp(0) = -l_ft_LPF(4) / l_ft_LPF(2) + lfoot_support_current_.translation()(0);
    //left_zmp(0) = l_ft_LPF(4) / l_ft_LPF(2) + lfoot_support_current_.translation()(0);
    left_zmp(1) = l_ft_LPF(3) / l_ft_LPF(2) + lfoot_support_current_.translation()(1);

    right_zmp(0) = -r_ft_LPF(4) / r_ft_LPF(2) + rfoot_support_current_.translation()(0);
    //right_zmp(0) = r_ft_LPF(4) / r_ft_LPF(2) + rfoot_support_current_.translation()(0);
   
    right_zmp(1) = r_ft_LPF(3) / r_ft_LPF(2) + rfoot_support_current_.translation()(1);

    if(contactMode == 1)
    {
        zmp_measured_mj_(0) = (left_zmp(0) * l_ft_LPF(2) + right_zmp(0) * r_ft_LPF(2)) / (l_ft_LPF(2) + r_ft_LPF(2));
        zmp_measured_mj_(1) = (left_zmp(1) * l_ft_LPF(2) + right_zmp(1) * r_ft_LPF(2)) / (l_ft_LPF(2) + r_ft_LPF(2)); // ZMP Y
    }
    else if(contactMode == 2)
    {
        zmp_measured_mj_(0) = left_zmp(0);
        zmp_measured_mj_(1) = left_zmp(1);
    }
    else
    {
        zmp_measured_mj_(0) = right_zmp(0);
        zmp_measured_mj_(1) = right_zmp(1);
    }

    Eigen::Vector3d zmp_temp;
    zmp_temp(0) = zmp_measured_mj_(0);
    zmp_temp(1) = zmp_measured_mj_(1);
    zmp_temp(2) = 0.0;

    ZMP_float = DyrosMath::multiplyIsometry3dVector3d(supportfoot_float_current_yaw_only, zmp_temp);

    wn = sqrt(GRAVITY / zc_mj_);

    if (walking_tick_mj == 0)
    {
        zmp_measured_LPF_.setZero();
    }

    if(walking_tick_mj == 3)
    {
        torquecontrol_Ok = true;
    }

    zmp_measured_LPF_ = (2 * M_PI * 8.0 * del_t) / (1 + 2 * M_PI * 8.0 * del_t) * zmp_measured_mj_ + 1 / (1 + 2 * M_PI * 8.0 * del_t) * zmp_measured_LPF_;
}

void CustomController::calculateFootStepTotal()
{
    double initial_rot = 0.0;
    double final_rot = 0.0;
    double initial_drot = 0.0;
    double final_drot = 0.0;

    initial_rot = atan2(target_y_, target_x_);

    if (initial_rot > 0.0)
        initial_drot = 5 * DEG2RAD;
    else
        initial_drot = -5 * DEG2RAD;

    unsigned int initial_total_step_number = initial_rot / initial_drot;
    double initial_residual_angle = initial_rot - initial_total_step_number * initial_drot;

    final_rot = target_theta_ - initial_rot;
    if (final_rot > 0.0)
        final_drot = 5 * DEG2RAD;
    else
        final_drot = -5 * DEG2RAD;

    unsigned int final_total_step_number = final_rot / final_drot;
    double final_residual_angle = final_rot - final_total_step_number * final_drot;
    double length_to_target = sqrt(target_x_ * target_x_ + target_y_ * target_y_);
    double dlength = step_length_x_;
    unsigned int middle_total_step_number = length_to_target / dlength;
    double middle_residual_length = length_to_target - middle_total_step_number * dlength;

    if (length_to_target == 0)
    {
        middle_total_step_number = 30; //
        dlength = 0;
    }

    unsigned int number_of_foot_step;

    int del_size;

    del_size = 1;
    number_of_foot_step = initial_total_step_number * del_size + middle_total_step_number * del_size + final_total_step_number * del_size;

    if (initial_total_step_number != 0 || abs(initial_residual_angle) >= 0.0001)
    {
        if (initial_total_step_number % 2 == 0)
            number_of_foot_step = number_of_foot_step + 2 * del_size;
        else
        {
            if (abs(initial_residual_angle) >= 0.0001)
                number_of_foot_step = number_of_foot_step + 3 * del_size;
            else
                number_of_foot_step = number_of_foot_step + del_size;
        }
    }

    if (middle_total_step_number != 0 || abs(middle_residual_length) >= 0.0001)
    {
        if (middle_total_step_number % 2 == 0)
            number_of_foot_step = number_of_foot_step + 2 * del_size;
        else
        {
            if (abs(middle_residual_length) >= 0.0001)
                number_of_foot_step = number_of_foot_step + 3 * del_size;
            else
                number_of_foot_step = number_of_foot_step + del_size;
        }
    }

    if (final_total_step_number != 0 || abs(final_residual_angle) >= 0.0001)
    {
        if (abs(final_residual_angle) >= 0.0001)
            number_of_foot_step = number_of_foot_step + 2 * del_size;
        else
            number_of_foot_step = number_of_foot_step + del_size;
    }

    foot_step_.resize(number_of_foot_step, 7);
    foot_step_.setZero();
    foot_step_support_frame_.resize(number_of_foot_step, 7);
    foot_step_support_frame_.setZero();

    int index = 0;
    int temp, temp2, temp3, is_right;

    if (is_right_foot_swing_ == true)
        is_right = 1;
    else
        is_right = -1;

    temp = -is_right;
    temp2 = -is_right;
    temp3 = -is_right;

    if (initial_total_step_number != 0 || abs(initial_residual_angle) >= 0.0001) // 첫번째 회전
    {
        for (int i = 0; i < initial_total_step_number; i++)
        {
            temp *= -1;
            foot_step_(index, 0) = temp * 0.1025 * sin((i + 1) * initial_drot);
            foot_step_(index, 1) = -temp * 0.1025 * cos((i + 1) * initial_drot);
            foot_step_(index, 5) = (i + 1) * initial_drot;
            foot_step_(index, 6) = 0.5 + 0.5 * temp;
            index++;
        }

        if (temp == is_right)
        {
            if (abs(initial_residual_angle) >= 0.0001)
            {
                temp *= -1;

                foot_step_(index, 0) = temp * 0.1025 * sin((initial_total_step_number)*initial_drot + initial_residual_angle);
                foot_step_(index, 1) = -temp * 0.1025 * cos((initial_total_step_number)*initial_drot + initial_residual_angle);
                foot_step_(index, 5) = (initial_total_step_number)*initial_drot + initial_residual_angle;
                foot_step_(index, 6) = 0.5 + 0.5 * temp;
                index++;

                temp *= -1;

                foot_step_(index, 0) = temp * 0.1025 * sin((initial_total_step_number)*initial_drot + initial_residual_angle);
                foot_step_(index, 1) = -temp * 0.1025 * cos((initial_total_step_number)*initial_drot + initial_residual_angle);
                foot_step_(index, 5) = (initial_total_step_number)*initial_drot + initial_residual_angle;
                foot_step_(index, 6) = 0.5 + 0.5 * temp;
                index++;

                temp *= -1;

                foot_step_(index, 0) = temp * 0.1025 * sin((initial_total_step_number)*initial_drot + initial_residual_angle);
                foot_step_(index, 1) = -temp * 0.1025 * cos((initial_total_step_number)*initial_drot + initial_residual_angle);
                foot_step_(index, 5) = (initial_total_step_number)*initial_drot + initial_residual_angle;
                foot_step_(index, 6) = 0.5 + 0.5 * temp;
                index++;
            }
            else
            {
                temp *= -1;

                foot_step_(index, 0) = temp * 0.1025 * sin((initial_total_step_number)*initial_drot + initial_residual_angle);
                foot_step_(index, 1) = -temp * 0.1025 * cos((initial_total_step_number)*initial_drot + initial_residual_angle);
                foot_step_(index, 5) = (initial_total_step_number)*initial_drot + initial_residual_angle;
                foot_step_(index, 6) = 0.5 + 0.5 * temp;
                index++;
            }
        }
        else if (temp == -is_right)
        {
            temp *= -1;

            foot_step_(index, 0) = temp * 0.1025 * sin((initial_total_step_number)*initial_drot + initial_residual_angle);
            foot_step_(index, 1) = -temp * 0.1025 * cos((initial_total_step_number)*initial_drot + initial_residual_angle);
            foot_step_(index, 5) = (initial_total_step_number)*initial_drot + initial_residual_angle;
            foot_step_(index, 6) = 0.5 + 0.5 * temp;
            index++;

            temp *= -1;

            foot_step_(index, 0) = temp * 0.1025 * sin((initial_total_step_number)*initial_drot + initial_residual_angle);
            foot_step_(index, 1) = -temp * 0.1025 * cos((initial_total_step_number)*initial_drot + initial_residual_angle);
            foot_step_(index, 5) = (initial_total_step_number)*initial_drot + initial_residual_angle;
            foot_step_(index, 6) = 0.5 + 0.5 * temp;
            index++;
        }
    }

    if (middle_total_step_number != 0 || abs(middle_residual_length) >= 0.0001)
    {
        for (int i = 0; i < middle_total_step_number; i++)
        {
            temp2 *= -1;

            foot_step_(index, 0) = cos(initial_rot) * (dlength * (i + 1)) + temp2 * sin(initial_rot) * (0.1025);
            foot_step_(index, 1) = sin(initial_rot) * (dlength * (i + 1)) - temp2 * cos(initial_rot) * (0.1025);
            foot_step_(index, 5) = initial_rot;
            foot_step_(index, 6) = 0.5 + 0.5 * temp2;
            index++;
        }

        if (temp2 == is_right)
        {
            if (abs(middle_residual_length) >= 0.0001)
            {
                temp2 *= -1;

                foot_step_(index, 0) = cos(initial_rot) * (dlength * (middle_total_step_number) + middle_residual_length) + temp2 * sin(initial_rot) * (0.1025);
                foot_step_(index, 1) = sin(initial_rot) * (dlength * (middle_total_step_number) + middle_residual_length) - temp2 * cos(initial_rot) * (0.1025);
                foot_step_(index, 5) = initial_rot;
                foot_step_(index, 6) = 0.5 + 0.5 * temp2;

                index++;

                temp2 *= -1;

                foot_step_(index, 0) = cos(initial_rot) * (dlength * (middle_total_step_number) + middle_residual_length) + temp2 * sin(initial_rot) * (0.1025);
                foot_step_(index, 1) = sin(initial_rot) * (dlength * (middle_total_step_number) + middle_residual_length) - temp2 * cos(initial_rot) * (0.1025);
                foot_step_(index, 5) = initial_rot;
                foot_step_(index, 6) = 0.5 + 0.5 * temp2;
                index++;

                temp2 *= -1;

                foot_step_(index, 0) = cos(initial_rot) * (dlength * (middle_total_step_number) + middle_residual_length) + temp2 * sin(initial_rot) * (0.1025);
                foot_step_(index, 1) = sin(initial_rot) * (dlength * (middle_total_step_number) + middle_residual_length) - temp2 * cos(initial_rot) * (0.1025);
                foot_step_(index, 5) = initial_rot;
                foot_step_(index, 6) = 0.5 + 0.5 * temp2;
                index++;
            }
            else
            {
                temp2 *= -1;

                foot_step_(index, 0) = cos(initial_rot) * (dlength * (middle_total_step_number) + middle_residual_length) + temp2 * sin(initial_rot) * (0.1025);
                foot_step_(index, 1) = sin(initial_rot) * (dlength * (middle_total_step_number) + middle_residual_length) - temp2 * cos(initial_rot) * (0.1025);
                foot_step_(index, 5) = initial_rot;
                foot_step_(index, 6) = 0.5 + 0.5 * temp2;
                index++;
            }
        }
        else if (temp2 == -is_right)
        {
            temp2 *= -1;

            foot_step_(index, 0) = cos(initial_rot) * (dlength * (middle_total_step_number) + middle_residual_length) + temp2 * sin(initial_rot) * (0.1025);
            foot_step_(index, 1) = sin(initial_rot) * (dlength * (middle_total_step_number) + middle_residual_length) - temp2 * cos(initial_rot) * (0.1025);
            foot_step_(index, 5) = initial_rot;
            foot_step_(index, 6) = 0.5 + 0.5 * temp2;
            index++;

            temp2 *= -1;

            foot_step_(index, 0) = cos(initial_rot) * (dlength * (middle_total_step_number) + middle_residual_length) + temp2 * sin(initial_rot) * (0.1025);
            foot_step_(index, 1) = sin(initial_rot) * (dlength * (middle_total_step_number) + middle_residual_length) - temp2 * cos(initial_rot) * (0.1025);
            foot_step_(index, 5) = initial_rot;
            foot_step_(index, 6) = 0.5 + 0.5 * temp2;
            index++;
        }
    }

    double final_position_x = cos(initial_rot) * (dlength * (middle_total_step_number) + middle_residual_length);
    double final_position_y = sin(initial_rot) * (dlength * (middle_total_step_number) + middle_residual_length);

    if (final_total_step_number != 0 || abs(final_residual_angle) >= 0.0001)
    {
        for (int i = 0; i < final_total_step_number; i++)
        {
            temp3 *= -1;

            foot_step_(index, 0) = final_position_x + temp3 * 0.1025 * sin((i + 1) * final_drot + initial_rot);
            foot_step_(index, 1) = final_position_y - temp3 * 0.1025 * cos((i + 1) * final_drot + initial_rot);
            foot_step_(index, 5) = (i + 1) * final_drot + initial_rot;
            foot_step_(index, 6) = 0.5 + 0.5 * temp3;
            index++;
        }

        if (abs(final_residual_angle) >= 0.0001)
        {
            temp3 *= -1;

            foot_step_(index, 0) = final_position_x + temp3 * 0.1025 * sin(target_theta_);
            foot_step_(index, 1) = final_position_y - temp3 * 0.1025 * cos(target_theta_);
            foot_step_(index, 5) = target_theta_;
            foot_step_(index, 6) = 0.5 + 0.5 * temp3;
            index++;

            temp3 *= -1;

            foot_step_(index, 0) = final_position_x + temp3 * 0.1025 * sin(target_theta_);
            foot_step_(index, 1) = final_position_y - temp3 * 0.1025 * cos(target_theta_);
            foot_step_(index, 5) = target_theta_;
            foot_step_(index, 6) = 0.5 + 0.5 * temp3;
            index++;
        }
        else
        {
            temp3 *= -1;

            foot_step_(index, 0) = final_position_x + temp3 * 0.1025 * sin(target_theta_);
            foot_step_(index, 1) = final_position_y - temp3 * 0.1025 * cos(target_theta_);
            foot_step_(index, 5) = target_theta_;
            foot_step_(index, 6) = 0.5 + 0.5 * temp3;
            index++;
        }
    }
}

void CustomController::calculateFootStepTotal_MJ()
{
    double initial_rot = 0.0;
    double final_rot = 0.0;
    double initial_drot = 0.0;
    double final_drot = 0.0;

    initial_rot = atan2(target_y_, target_x_);

    if (initial_rot > 0.0)
        initial_drot = 20 * DEG2RAD;
    else
        initial_drot = -20 * DEG2RAD;

    unsigned int initial_total_step_number = initial_rot / initial_drot;
    double initial_residual_angle = initial_rot - initial_total_step_number * initial_drot;

    final_rot = target_theta_ - initial_rot;
    if (final_rot > 0.0)
        final_drot = 20 * DEG2RAD;
    else
        final_drot = -20 * DEG2RAD;

    unsigned int final_total_step_number = final_rot / final_drot;
    double final_residual_angle = final_rot - final_total_step_number * final_drot;
    double length_to_target = sqrt(target_x_ * target_x_ + target_y_ * target_y_);
    double dlength = step_length_x_;
    unsigned int middle_total_step_number = length_to_target / dlength;
    double middle_residual_length = length_to_target - middle_total_step_number * dlength;

    double step_width_init;
    double step_width;

    step_width_init = 0.00; //0.01
    step_width = 0.00; //0.02

    if (length_to_target == 0.0)
    {
        middle_total_step_number = 12; //total foot step number
        dlength = 0;
    }

    unsigned int number_of_foot_step;

    int del_size;

    del_size = 1;
    number_of_foot_step = 2 + initial_total_step_number * del_size + middle_total_step_number * del_size + final_total_step_number * del_size;

    if (initial_total_step_number != 0 || abs(initial_residual_angle) >= 0.0001)
    {
        if (initial_total_step_number % 2 == 0)
            number_of_foot_step = number_of_foot_step + 2 * del_size;
        else
        {
            if (abs(initial_residual_angle) >= 0.0001)
                number_of_foot_step = number_of_foot_step + 3 * del_size;
            else
                number_of_foot_step = number_of_foot_step + del_size;
        }
    }

    if (middle_total_step_number != 0 || abs(middle_residual_length) >= 0.0001)
    {
        if (middle_total_step_number % 2 == 0)
            number_of_foot_step = number_of_foot_step + 2 * del_size;
        else
        {
            if (abs(middle_residual_length) >= 0.0001)
                number_of_foot_step = number_of_foot_step + 3 * del_size;
            else
                number_of_foot_step = number_of_foot_step + del_size;
        }
    }

    if (final_total_step_number != 0 || abs(final_residual_angle) >= 0.0001)
    {
        if (abs(final_residual_angle) >= 0.0001)
            number_of_foot_step = number_of_foot_step + 2 * del_size;
        else
            number_of_foot_step = number_of_foot_step + del_size;
    }

    foot_step_.resize(number_of_foot_step, 7);
    foot_step_.setZero();
    foot_step_support_frame_.resize(number_of_foot_step, 7);
    foot_step_support_frame_.setZero();

    int index = 0;
    int temp, temp2, temp3, is_right;

    if (is_right_foot_swing_ == true)
        is_right = 1;
    else
        is_right = -1;

    temp = -is_right;
    temp2 = -is_right;
    temp3 = -is_right;

    int temp0;
    temp0 = -is_right;

    double initial_dir = 0.0;
    if(walking_tick_mj == 0)
        foot_legnth = (lfoot_float_init_.translation()(1) - rfoot_float_init_.translation()(1))/2;

    if (aa == 0)
    {
        for (int i = 0; i < 2; i++)
        {
            temp0 *= -1;

            if (i == 0)
            {
                foot_step_(index, 0) = cos(initial_dir) * (0.0) + temp0 * sin(initial_dir) * (foot_legnth + step_width_init * (i + 1));
                foot_step_(index, 1) = sin(initial_dir) * (0.0) - temp0 * cos(initial_dir) * (foot_legnth + step_width_init * (i + 1));
            }
            else if (i == 1)
            {
                foot_step_(index, 0) = cos(initial_dir) * (0.0) + temp0 * sin(initial_dir) * (foot_legnth + step_width_init * (i + 1));
                foot_step_(index, 1) = sin(initial_dir) * (0.0) - temp0 * cos(initial_dir) * (foot_legnth + step_width_init * (i + 1));
            }

            foot_step_(index, 5) = initial_dir;
            foot_step_(index, 6) = 0.5 + 0.5 * temp0;
            index++;
        }
    }
    else if (aa == 1)
    {
        for (int i = 0; i < 2; i++)
        {
            temp0 *= -1;

            foot_step_(index, 0) = cos(initial_dir) * (0.0) + temp0 * sin(initial_dir) * (foot_legnth + step_width);
            foot_step_(index, 1) = sin(initial_dir) * (0.0) - temp0 * cos(initial_dir) * (foot_legnth + step_width);
            foot_step_(index, 5) = initial_dir;
            foot_step_(index, 6) = 0.5 + 0.5 * temp0;
            index++;
        }
    }

    if (initial_total_step_number != 0 || abs(initial_residual_angle) >= 0.0001) // 첫번째 회전
    {
        for (int i = 0; i < initial_total_step_number; i++)
        {
            temp *= -1;
            foot_step_(index, 0) = temp * (foot_legnth + step_width) * sin((i + 1) * initial_drot);
            foot_step_(index, 1) = -temp * (foot_legnth + step_width) * cos((i + 1) * initial_drot);
            foot_step_(index, 5) = (i + 1) * initial_drot;
            foot_step_(index, 6) = 0.5 + 0.5 * temp;
            index++;
        }

        if (temp == is_right)
        {
            if (abs(initial_residual_angle) >= 0.0001)
            {
                temp *= -1;

                foot_step_(index, 0) = temp * (foot_legnth + step_width) * sin((initial_total_step_number)*initial_drot + initial_residual_angle);
                foot_step_(index, 1) = -temp * (foot_legnth + step_width) * cos((initial_total_step_number)*initial_drot + initial_residual_angle);
                foot_step_(index, 5) = (initial_total_step_number)*initial_drot + initial_residual_angle;
                foot_step_(index, 6) = 0.5 + 0.5 * temp;
                index++;

                temp *= -1;

                foot_step_(index, 0) = temp * (foot_legnth + step_width) * sin((initial_total_step_number)*initial_drot + initial_residual_angle);
                foot_step_(index, 1) = -temp * (foot_legnth + step_width) * cos((initial_total_step_number)*initial_drot + initial_residual_angle);
                foot_step_(index, 5) = (initial_total_step_number)*initial_drot + initial_residual_angle;
                foot_step_(index, 6) = 0.5 + 0.5 * temp;
                index++;

                temp *= -1;

                foot_step_(index, 0) = temp * (foot_legnth + step_width) * sin((initial_total_step_number)*initial_drot + initial_residual_angle);
                foot_step_(index, 1) = -temp * (foot_legnth + step_width) * cos((initial_total_step_number)*initial_drot + initial_residual_angle);
                foot_step_(index, 5) = (initial_total_step_number)*initial_drot + initial_residual_angle;
                foot_step_(index, 6) = 0.5 + 0.5 * temp;
                index++;
            }
            else
            {
                temp *= -1;

                foot_step_(index, 0) = temp * (foot_legnth + step_width) * sin((initial_total_step_number)*initial_drot + initial_residual_angle);
                foot_step_(index, 1) = -temp * (foot_legnth + step_width) * cos((initial_total_step_number)*initial_drot + initial_residual_angle);
                foot_step_(index, 5) = (initial_total_step_number)*initial_drot + initial_residual_angle;
                foot_step_(index, 6) = 0.5 + 0.5 * temp;
                index++;
            }
        }
        else if (temp == -is_right)
        {
            temp *= -1;

            foot_step_(index, 0) = temp * (foot_legnth + step_width) * sin((initial_total_step_number)*initial_drot + initial_residual_angle);
            foot_step_(index, 1) = -temp * (foot_legnth + step_width) * cos((initial_total_step_number)*initial_drot + initial_residual_angle);
            foot_step_(index, 5) = (initial_total_step_number)*initial_drot + initial_residual_angle;
            foot_step_(index, 6) = 0.5 + 0.5 * temp;
            index++;

            temp *= -1;

            foot_step_(index, 0) = temp * (foot_legnth + step_width) * sin((initial_total_step_number)*initial_drot + initial_residual_angle);
            foot_step_(index, 1) = -temp * (foot_legnth + step_width) * cos((initial_total_step_number)*initial_drot + initial_residual_angle);
            foot_step_(index, 5) = (initial_total_step_number)*initial_drot + initial_residual_angle;
            foot_step_(index, 6) = 0.5 + 0.5 * temp;
            index++;
        }
    }

    if (middle_total_step_number != 0 || abs(middle_residual_length) >= 0.0001) // 직진, 제자리 보행
    {

        for (int i = 0; i < middle_total_step_number; i++)
        {
            temp2 *= -1;

            foot_step_(index, 0) = cos(initial_rot) * (dlength * (i + 1)) + temp2 * sin(initial_rot) * (foot_legnth + step_width);
            foot_step_(index, 1) = sin(initial_rot) * (dlength * (i + 1)) - temp2 * cos(initial_rot) * (foot_legnth + step_width);
            foot_step_(index, 5) = initial_rot;
            foot_step_(index, 6) = 0.5 + 0.5 * temp2;
            index++;
        }

        if (temp2 == is_right)
        {
            if (abs(middle_residual_length) >= 0.0001)
            {
                temp2 *= -1;

                foot_step_(index, 0) = cos(initial_rot) * (dlength * (middle_total_step_number) + middle_residual_length) + temp2 * sin(initial_rot) * (foot_legnth + step_width);
                foot_step_(index, 1) = sin(initial_rot) * (dlength * (middle_total_step_number) + middle_residual_length) - temp2 * cos(initial_rot) * (foot_legnth + step_width);
                foot_step_(index, 5) = initial_rot;
                foot_step_(index, 6) = 0.5 + 0.5 * temp2;

                index++;

                temp2 *= -1;

                foot_step_(index, 0) = cos(initial_rot) * (dlength * (middle_total_step_number) + middle_residual_length) + temp2 * sin(initial_rot) * (foot_legnth + step_width);
                foot_step_(index, 1) = sin(initial_rot) * (dlength * (middle_total_step_number) + middle_residual_length) - temp2 * cos(initial_rot) * (foot_legnth + step_width);
                foot_step_(index, 5) = initial_rot;
                foot_step_(index, 6) = 0.5 + 0.5 * temp2;
                index++;

                temp2 *= -1;

                foot_step_(index, 0) = cos(initial_rot) * (dlength * (middle_total_step_number) + middle_residual_length) + temp2 * sin(initial_rot) * (foot_legnth + step_width);
                foot_step_(index, 1) = sin(initial_rot) * (dlength * (middle_total_step_number) + middle_residual_length) - temp2 * cos(initial_rot) * (foot_legnth + step_width);
                foot_step_(index, 5) = initial_rot;
                foot_step_(index, 6) = 0.5 + 0.5 * temp2;
                index++;
            }
            else
            {
                temp2 *= -1;

                foot_step_(index, 0) = cos(initial_rot) * (dlength * (middle_total_step_number) + middle_residual_length) + temp2 * sin(initial_rot) * (foot_legnth + step_width);
                foot_step_(index, 1) = sin(initial_rot) * (dlength * (middle_total_step_number) + middle_residual_length) - temp2 * cos(initial_rot) * (foot_legnth + step_width);
                foot_step_(index, 5) = initial_rot;
                foot_step_(index, 6) = 0.5 + 0.5 * temp2;
                index++;
            }
        }
        else if (temp2 == -is_right)
        {
            temp2 *= -1;

            foot_step_(index, 0) = cos(initial_rot) * (dlength * (middle_total_step_number) + middle_residual_length) + temp2 * sin(initial_rot) * (foot_legnth + step_width);
            foot_step_(index, 1) = sin(initial_rot) * (dlength * (middle_total_step_number) + middle_residual_length) - temp2 * cos(initial_rot) * (foot_legnth + step_width);
            foot_step_(index, 5) = initial_rot;
            foot_step_(index, 6) = 0.5 + 0.5 * temp2;
            index++;

            temp2 *= -1;

            foot_step_(index, 0) = cos(initial_rot) * (dlength * (middle_total_step_number) + middle_residual_length) + temp2 * sin(initial_rot) * (foot_legnth + step_width);
            foot_step_(index, 1) = sin(initial_rot) * (dlength * (middle_total_step_number) + middle_residual_length) - temp2 * cos(initial_rot) * (foot_legnth + step_width);
            foot_step_(index, 5) = initial_rot;
            foot_step_(index, 6) = 0.5 + 0.5 * temp2;
            index++;
        }
    }

    double final_position_x = cos(initial_rot) * (dlength * (middle_total_step_number) + middle_residual_length);
    double final_position_y = sin(initial_rot) * (dlength * (middle_total_step_number) + middle_residual_length);

    if (final_total_step_number != 0 || abs(final_residual_angle) >= 0.0001)
    {
        for (int i = 0; i < final_total_step_number; i++)
        {
            temp3 *= -1;

            foot_step_(index, 0) = final_position_x + temp3 * (foot_legnth + step_width) * sin((i + 1) * final_drot + initial_rot);
            foot_step_(index, 1) = final_position_y - temp3 * (foot_legnth + step_width) * cos((i + 1) * final_drot + initial_rot);
            foot_step_(index, 5) = (i + 1) * final_drot + initial_rot;
            foot_step_(index, 6) = 0.5 + 0.5 * temp3;
            index++;
        }

        if (abs(final_residual_angle) >= 0.0001)
        {
            temp3 *= -1;

            foot_step_(index, 0) = final_position_x + temp3 * (foot_legnth + step_width) * sin(target_theta_);
            foot_step_(index, 1) = final_position_y - temp3 * (foot_legnth + step_width) * cos(target_theta_);
            foot_step_(index, 5) = target_theta_;
            foot_step_(index, 6) = 0.5 + 0.5 * temp3;
            index++;

            temp3 *= -1;

            foot_step_(index, 0) = final_position_x + temp3 * (foot_legnth + step_width) * sin(target_theta_);
            foot_step_(index, 1) = final_position_y - temp3 * (foot_legnth + step_width) * cos(target_theta_);
            foot_step_(index, 5) = target_theta_;
            foot_step_(index, 6) = 0.5 + 0.5 * temp3;
            index++;
        }
        else
        {
            temp3 *= -1;

            foot_step_(index, 0) = final_position_x + temp3 * (foot_legnth + step_width) * sin(target_theta_);
            foot_step_(index, 1) = final_position_y - temp3 * (foot_legnth + step_width) * cos(target_theta_);
            foot_step_(index, 5) = target_theta_;
            foot_step_(index, 6) = 0.5 + 0.5 * temp3;
            index++;
        }
    }
}

void CustomController::floatToSupportFootstep()
{
    Eigen::Isometry3d reference;
    if (current_step_num_ == 0)
    {
        if (foot_step_(0, 6) == 0)
        {
            reference.translation() = rfoot_float_init_.translation();
            reference.translation()(2) = 0.0;
            reference.linear() = DyrosMath::rotateWithZ(DyrosMath::rot2Euler(rfoot_float_init_.linear())(2));
            reference.translation()(0) = 0.0;
        }
        else
        {
            reference.translation() = lfoot_float_init_.translation();
            reference.translation()(2) = 0.0;
            reference.linear() = DyrosMath::rotateWithZ(DyrosMath::rot2Euler(lfoot_float_init_.linear())(2));
            reference.translation()(0) = 0.0;
        }
    }
    else
    {
        reference.linear() = DyrosMath::rotateWithZ(foot_step_(current_step_num_ - 1, 5));
        for (int i = 0; i < 3; i++)
        {
            reference.translation()(i) = foot_step_(current_step_num_ - 1, i);
        }
    }

    Eigen::Vector3d temp_local_position;
    Eigen::Vector3d temp_global_position;
   
    for (int i = 0; i < total_step_num_; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            temp_global_position(j) = foot_step_(i, j);
        }

        temp_local_position = reference.linear().transpose() * (temp_global_position - reference.translation());
       
        for (int j = 0; j < 3; j++)
        {
            foot_step_support_frame_(i, j) = temp_local_position(j);
        }

        foot_step_support_frame_(i, 3) = foot_step_(i, 3);
        foot_step_support_frame_(i, 4) = foot_step_(i, 4);
        if (current_step_num_ == 0)
        {
            foot_step_support_frame_(i, 5) = foot_step_(i, 5) - supportfoot_float_init_(5);
        }
        else
        {
            foot_step_support_frame_(i, 5) = foot_step_(i, 5) - foot_step_(current_step_num_ - 1, 5);
        }
    }

    for (int j = 0; j < 3; j++)
        temp_global_position(j) = swingfoot_float_init_(j); // swingfoot_float_init_은 Pelvis에서 본 Swing 발의 Position, orientation.

    temp_local_position = reference.linear().transpose() * (temp_global_position - reference.translation());

    for (int j = 0; j < 3; j++)
        swingfoot_support_init_(j) = temp_local_position(j);

    swingfoot_support_init_(3) = swingfoot_float_init_(3);
    swingfoot_support_init_(4) = swingfoot_float_init_(4);

    if (current_step_num_ == 0)
        swingfoot_support_init_(5) = swingfoot_float_init_(5) - supportfoot_float_init_(5);
    else
        swingfoot_support_init_(5) = swingfoot_float_init_(5) - foot_step_(current_step_num_ - 1, 5);

    for (int j = 0; j < 3; j++)
        temp_global_position(j) = supportfoot_float_init_(j);

    temp_local_position = reference.linear().transpose() * (temp_global_position - reference.translation());

    for (int j = 0; j < 3; j++)
        supportfoot_support_init_(j) = temp_local_position(j);

    supportfoot_support_init_(3) = supportfoot_float_init_(3);
    supportfoot_support_init_(4) = supportfoot_float_init_(4);

    if (current_step_num_ == 0)
        supportfoot_support_init_(5) = 0;
    else
        supportfoot_support_init_(5) = supportfoot_float_init_(5) - foot_step_(current_step_num_ - 1, 5);
}

void CustomController::Joint_gain_set_MJ()
{
    //simulation gains
    Kp(0) = 1800.0;
    Kd(0) = 70.0; // Left Hip yaw
    Kp(1) = 2100.0;
    Kd(1) = 90.0; // Left Hip roll
    Kp(2) = 2100.0;
    Kd(2) = 90.0; // Left Hip pitch
    Kp(3) = 2100.0;
    Kd(3) = 90.0; // Left Knee pitch
    Kp(4) = 2100.0;
    Kd(4) = 90.0; // Left Ankle pitch
    Kp(5) = 2100.0;
    Kd(5) = 90.0; // Left Ankle roll

    Kp(6) = 1800.0;
    Kd(6) = 70.0; // Right Hip yaw
    Kp(7) = 2100.0;
    Kd(7) = 90.0; // Right Hip roll
    Kp(8) = 2100.0;
    Kd(8) = 90.0; // Right Hip pitch
    Kp(9) = 2100.0;
    Kd(9) = 90.0; // Right Knee pitch
    Kp(10) = 2100.0;
    Kd(10) = 90.0; // Right Ankle pitch
    Kp(11) = 2100.0;
    Kd(11) = 90.0; // Right Ankle roll

    Kp(12) = 2200.0;
    Kd(12) = 90.0; // Waist yaw
    Kp(13) = 2200.0;
    Kd(13) = 90.0; // Waist pitch
    Kp(14) = 2200.0;
    Kd(14) = 90.0; // Waist roll

    Kp(15) = 400.0;
    Kd(15) = 10.0;
    Kp(16) = 800.0;
    Kd(16) = 10.0;
    Kp(17) = 400.0;
    Kd(17) = 10.0;
    Kp(18) = 400.0;
    Kd(18) = 10.0;
    Kp(19) = 250.0;
    Kd(19) = 2.5;
    Kp(20) = 250.0;
    Kd(20) = 2.0;
    Kp(21) = 50.0;
    Kd(21) = 2.0; // Left Wrist
    Kp(22) = 50.0;
    Kd(22) = 2.0; // Left Wrist

    Kp(23) = 50.0;
    Kd(23) = 2.0; // Neck
    Kp(24) = 50.0;
    Kd(24) = 2.0; // Neck

    Kp(25) = 400.0;
    Kd(25) = 10.0;
    Kp(26) = 800.0;
    Kd(26) = 10.0;
    Kp(27) = 400.0;
    Kd(27) = 10.0;
    Kp(28) = 400.0;
    Kd(28) = 10.0;
    Kp(29) = 250.0;
    Kd(29) = 2.5;
    Kp(30) = 250.0;
    Kd(30) = 2.0;
    Kp(31) = 50.0;
    Kd(31) = 2.0; // Right Wrist
    Kp(32) = 50.0;
    Kd(32) = 2.0; // Right Wrist

    // Kp(0) = 2000.0;
    // Kd(0) = 20.0; // Left Hip yaw
    // Kp(1) = 5000.0;
    // Kd(1) = 55.0; // Left Hip roll //55
    // Kp(2) = 4000.0;
    // Kd(2) = 45.0; // Left Hip pitch
    // Kp(3) = 3700.0;
    // Kd(3) = 40.0; // Left Knee pitch
    // Kp(4) = 4000.0; // 5000
    // Kd(4) = 65.0; // Left Ankle pitch /5000 / 30  //55
    // Kp(5) = 4000.0; // 5000
    // Kd(5) = 65.0; // Left Ankle roll /5000 / 30 //55

    // Kp(6) = 2000.0;
    // Kd(6) = 20.0; // Right Hip yaw
    // Kp(7) = 5000.0;
    // Kd(7) = 55.0; // Right Hip roll  //55
    // Kp(8) = 4000.0;
    // Kd(8) = 45.0; // Right Hip pitch
    // Kp(9) = 3700.0;
    // Kd(9) = 40.0; // Right Knee pitch
    // Kp(10) = 4000.0; // 5000
    // Kd(10) = 65.0; // Right Ankle pitch //55
    // Kp(11) = 4000.0; // 5000
    // Kd(11) = 65.0; // Right Ankle roll //55

    // Kp(12) = 6000.0;
    // Kd(12) = 200.0; // Waist yaw
    // Kp(13) = 10000.0;
    // Kd(13) = 100.0; // Waist pitch
    // Kp(14) = 10000.0;
    // Kd(14) = 100.0; // Waist roll

    // Kp(15) = 400.0;
    // Kd(15) = 10.0;
    // Kp(16) = 800.0;
    // Kd(16) = 10.0;
    // Kp(17) = 400.0;
    // Kd(17) = 10.0;
    // Kp(18) = 400.0;
    // Kd(18) = 10.0;
    // Kp(19) = 250.0;
    // Kd(19) = 2.5;
    // Kp(20) = 250.0;
    // Kd(20) = 2.0;
    // Kp(21) = 50.0;
    // Kd(21) = 2.0; // Left Wrist
    // Kp(22) = 50.0;
    // Kd(22) = 2.0; // Left Wrist

    // Kp(23) = 50.0;
    // Kd(23) = 2.0; // Neck
    // Kp(24) = 50.0;
    // Kd(24) = 2.0; // Neck

    // Kp(25) = 400.0;
    // Kd(25) = 10.0;
    // Kp(26) = 800.0;
    // Kd(26) = 10.0;
    // Kp(27) = 400.0;
    // Kd(27) = 10.0;
    // Kp(28) = 400.0;
    // Kd(28) = 10.0;
    // Kp(29) = 250.0;
    // Kd(29) = 2.5;
    // Kp(30) = 250.0;
    // Kd(30) = 2.0;
    // Kp(31) = 50.0;
    // Kd(31) = 2.0; // Right Wrist
    // Kp(32) = 50.0;
    // Kd(32) = 2.0; // Right Wrist
}

void CustomController::addZmpOffset()
{
    double lfoot_zmp_offset_, rfoot_zmp_offset_;

    // lfoot_zmp_offset_ = -0.025; // 0.9 초
    // rfoot_zmp_offset_ = 0.025;

    // lfoot_zmp_offset_ = -0.02; // 1.1 초
    // rfoot_zmp_offset_ = 0.02;

    lfoot_zmp_offset_ = -0.02; // simul 1.1 s
    rfoot_zmp_offset_ = 0.02;

    foot_step_support_frame_offset_ = foot_step_support_frame_;

    supportfoot_support_init_offset_ = supportfoot_support_init_;

    if (foot_step_(0, 6) == 0) //right support foot
    {
        supportfoot_support_init_offset_(1) = supportfoot_support_init_(1) + rfoot_zmp_offset_;
        //swingfoot_support_init_offset_(1) = swingfoot_support_init_(1) + lfoot_zmp_offset_;
    }
    else
    {
        supportfoot_support_init_offset_(1) = supportfoot_support_init_(1) + lfoot_zmp_offset_;
        //swingfoot_support_init_offset_(1) = swingfoot_support_init_(1) + rfoot_zmp_offset_;
    }

    for (int i = 0; i < total_step_num_; i++)
    {
        if (foot_step_(i, 6) == 0) //right support, left swing
        {
            foot_step_support_frame_offset_(i, 1) += lfoot_zmp_offset_;
        }
        else
        {
            foot_step_support_frame_offset_(i, 1) += rfoot_zmp_offset_;
        }
    }
}

void CustomController::getZmpTrajectory()
{
    unsigned int planning_step_number = 3;
    unsigned int norm_size = 0;

    if (current_step_num_ >= total_step_num_ - planning_step_number)
        norm_size = (t_last_ - t_start_ + 1) * (total_step_num_ - current_step_num_) + 3.0 * hz_;
    else
        norm_size = (t_last_ - t_start_ + 1) * (planning_step_number);
    if (current_step_num_ == 0)
        norm_size = norm_size + t_temp_ + 1;
    addZmpOffset();
    zmpGenerator(norm_size, planning_step_number);
}

void CustomController::zmpGenerator(const unsigned int norm_size, const unsigned planning_step_num)
{
    ref_zmp_mj_.resize(norm_size, 2);
    Eigen::VectorXd temp_px;
    Eigen::VectorXd temp_py;

    unsigned int index = 0;
    // 매 tick 마다 zmp가 3발 앞까지 계산 된다.

    if (current_step_num_ == 0) // Walking을 수행 할 때, 정지 상태 일때 3초 동안 Ref X ZMP를 0으로 보냄. Y ZMP는 제자리 유지.
    {
        for (int i = 0; i <= t_temp_; i++) //600 tick
        {
            if (i < 1.0 * hz_)
            {
                ref_zmp_mj_(i, 0) = com_support_init_(0);
                ref_zmp_mj_(i, 1) = com_support_init_(1);
            }
            else if (i < 2.0 * hz_)
            {
                double del_x = i - 1.0 * hz_;
                ref_zmp_mj_(i, 0) = com_support_init_(0) - del_x * com_support_init_(0) / (1.0 * hz_);
                ref_zmp_mj_(i, 1) = com_support_init_(1);
            }
            else
            {
                ref_zmp_mj_(i, 0) = 0.0;
                ref_zmp_mj_(i, 1) = com_support_init_(1);
            }
            index++;
        }
    }
    /////////////////////////////////////////////////////////////////////
    if (current_step_num_ >= total_step_num_ - planning_step_num)
    {
        for (unsigned int i = current_step_num_; i < total_step_num_; i++)
        {
            onestepZmp(i, temp_px, temp_py);

            for (unsigned int j = 0; j < t_total_; j++)
            {
                ref_zmp_mj_(index + j, 0) = temp_px(j);
                ref_zmp_mj_(index + j, 1) = temp_py(j);
            }
            index = index + t_total_;
        }

        for (unsigned int j = 0; j < 3.0 * hz_; j++)
        {
            ref_zmp_mj_(index + j, 0) = ref_zmp_mj_(index - 1, 0);
            ref_zmp_mj_(index + j, 1) = ref_zmp_mj_(index - 1, 1);
        }
        index = index + 3.0 * hz_;
    }
    else // 보행 중 사용 하는 Ref ZMP
    {
        for (unsigned int i = current_step_num_; i < current_step_num_ + planning_step_num; i++)
        {
            onestepZmp(i, temp_px, temp_py);
            for (unsigned int j = 0; j < t_total_; j++) // 1 step 보행은 1.2초, 240 tick
            {
                ref_zmp_mj_(index + j, 0) = temp_px(j);
                ref_zmp_mj_(index + j, 1) = temp_py(j);
            }
            index = index + t_total_; // 참조 zmp가 이만큼 쌓였다.
                                      // 결국 실제 로봇 1Hz마다 720개의 ref_zmp를 생성함. 3.6초
        }
    }
}

void CustomController::onestepZmp(unsigned int current_step_number, Eigen::VectorXd &temp_px, Eigen::VectorXd &temp_py)
{
    temp_px.resize(t_total_); // 함수가 실행 될 때 마다, 240 tick의 참조 ZMP를 담는다. Realtime = 1.2초
    temp_py.resize(t_total_);
    temp_px.setZero();
    temp_py.setZero();

    double Kx = 0, Ky = 0, A = 0, B = 0, wn = 0, Kx2 = 0, Ky2 = 0;
    if (current_step_number == 0)
    {
        Kx = 0;
        Ky = supportfoot_support_init_offset_(1) - com_support_init_(1);
        Kx2 = foot_step_support_frame_offset_(current_step_number, 0) / 2 - supportfoot_support_init_offset_(0);
        Ky2 = foot_step_support_frame_offset_(current_step_number, 1) / 2 - supportfoot_support_init_offset_(1);

        for (int i = 0; i < t_total_; i++)
        {
            if (i >= 0 && i < t_rest_init_ + t_double1_) //0.05 ~ 0.15초 , 10 ~ 30 tick
            {
                temp_px(i) = Kx / (t_double1_ + t_rest_init_) * (i + 1);
                temp_py(i) = com_support_init_(1) + Ky / (t_double1_ + t_rest_init_) * (i + 1);
            }
            else if (i >= t_rest_init_ + t_double1_ && i < t_total_ - t_rest_last_ - t_double2_) //0.15 ~ 1.05초 , 30 ~ 210 tick
            {
                temp_px(i) = 0;
                temp_py(i) = supportfoot_support_init_offset_(1);
            }
            else if (i >= t_total_ - t_rest_last_ - t_double2_ && i < t_total_) //1.05 ~ 1.15초 , 210 ~ 230 tick
            {
                temp_px(i) = 0 + Kx2 / (t_rest_last_ + t_double2_) * (i + 1 - (t_total_ - t_rest_last_ - t_double2_));
                temp_py(i) = supportfoot_support_init_offset_(1) + Ky2 / (t_rest_last_ + t_double2_) * (i + 1 - (t_total_ - t_rest_last_ - t_double2_));
            }
        }
    }
    else if (current_step_number == 1)
    {
        Kx = foot_step_support_frame_offset_(current_step_number - 1, 0) - (foot_step_support_frame_offset_(current_step_number - 1, 0) + supportfoot_support_init_(0)) / 2;
        Ky = foot_step_support_frame_offset_(current_step_number - 1, 1) - (foot_step_support_frame_offset_(current_step_number - 1, 1) + supportfoot_support_init_(1)) / 2;
        Kx2 = (foot_step_support_frame_offset_(current_step_number, 0) + foot_step_support_frame_offset_(current_step_number - 1, 0)) / 2 - foot_step_support_frame_offset_(current_step_number - 1, 0);
        Ky2 = (foot_step_support_frame_offset_(current_step_number, 1) + foot_step_support_frame_offset_(current_step_number - 1, 1)) / 2 - foot_step_support_frame_offset_(current_step_number - 1, 1);

        for (int i = 0; i < t_total_; i++)
        {
            if (i < t_rest_init_ + t_double1_) //0.05 ~ 0.15초 , 10 ~ 30 tick
            {
                temp_px(i) = (foot_step_support_frame_offset_(current_step_number - 1, 0) + supportfoot_support_init_(0)) / 2 + Kx / (t_rest_init_ + t_double1_) * (i + 1);
                temp_py(i) = (foot_step_support_frame_offset_(current_step_number - 1, 1) + supportfoot_support_init_(1)) / 2 + Ky / (t_rest_init_ + t_double1_) * (i + 1);
            }
            else if (i >= t_rest_init_ + t_double1_ && i < t_total_ - t_rest_last_ - t_double2_) //0.15 ~ 1.05초 , 30 ~ 210 tick
            {
                temp_px(i) = foot_step_support_frame_offset_(current_step_number - 1, 0);
                temp_py(i) = foot_step_support_frame_offset_(current_step_number - 1, 1);
            }
            else if (i >= t_total_ - t_rest_last_ - t_double2_ && i < t_total_) //1.05 ~ 1.2초 , 210 ~ 240 tick
            {
                temp_px(i) = foot_step_support_frame_offset_(current_step_number - 1, 0) + Kx2 / (t_double2_ + t_rest_last_) * (i + 1 - (t_total_ - t_rest_last_ - t_double2_));
                temp_py(i) = foot_step_support_frame_offset_(current_step_number - 1, 1) + Ky2 / (t_double2_ + t_rest_last_) * (i + 1 - (t_total_ - t_rest_last_ - t_double2_));
            }
        }
    }
    else
    {
        Kx = foot_step_support_frame_offset_(current_step_number - 1, 0) - ((foot_step_support_frame_offset_(current_step_number - 2, 0) + foot_step_support_frame_offset_(current_step_number - 1, 0)) / 2);
        Ky = foot_step_support_frame_offset_(current_step_number - 1, 1) - ((foot_step_support_frame_offset_(current_step_number - 2, 1) + foot_step_support_frame_offset_(current_step_number - 1, 1)) / 2);
        Kx2 = (foot_step_support_frame_offset_(current_step_number, 0) + foot_step_support_frame_offset_(current_step_number - 1, 0)) / 2 - foot_step_support_frame_offset_(current_step_number - 1, 0);
        Ky2 = (foot_step_support_frame_offset_(current_step_number, 1) + foot_step_support_frame_offset_(current_step_number - 1, 1)) / 2 - foot_step_support_frame_offset_(current_step_number - 1, 1);

        for (int i = 0; i < t_total_; i++)
        {
            if (i < t_rest_init_ + t_double1_) //0 ~ 0.15초 , 0 ~ 30 tick
            {
                temp_px(i) = (foot_step_support_frame_offset_(current_step_number - 2, 0) + foot_step_support_frame_offset_(current_step_number - 1, 0)) / 2 + Kx / (t_rest_init_ + t_double1_) * (i + 1);
                temp_py(i) = (foot_step_support_frame_offset_(current_step_number - 2, 1) + foot_step_support_frame_offset_(current_step_number - 1, 1)) / 2 + Ky / (t_rest_init_ + t_double1_) * (i + 1);
            }
            else if (i >= t_rest_init_ + t_double1_ && i < t_total_ - t_rest_last_ - t_double2_) //0.15 ~ 1.05초 , 30 ~ 210 tick
            {
                temp_px(i) = foot_step_support_frame_offset_(current_step_number - 1, 0);
                temp_py(i) = foot_step_support_frame_offset_(current_step_number - 1, 1);
            }
            else if (i >= t_total_ - t_rest_last_ - t_double2_ && i < t_total_) //1.05 ~ 1.2초 , 210 ~ 240 tick
            {
                temp_px(i) = foot_step_support_frame_offset_(current_step_number - 1, 0) + Kx2 / (t_double2_ + t_rest_last_) * (i + 1 - (t_total_ - t_rest_last_ - t_double2_));
                temp_py(i) = foot_step_support_frame_offset_(current_step_number - 1, 1) + Ky2 / (t_double2_ + t_rest_last_) * (i + 1 - (t_total_ - t_rest_last_ - t_double2_));
            }
        }
    }
}

void CustomController::getFootTrajectory()
{
    Eigen::Vector6d target_swing_foot;

    for (int i = 0; i < 6; i++)
    {
        target_swing_foot(i) = foot_step_support_frame_(current_step_num_, i);
    }

    if(current_step_num_ <= 1)
    {
        if (walking_tick_mj < t_start_ + t_rest_init_1 + t_double1_)
        {
            double t_rest_temp = 0.00 * hz_;
            contactMode = 1;
            if (foot_step_(current_step_num_, 6) == 1) // 왼발 지지
            {
                lfoot_trajectory_support_.translation().setZero();
                lfoot_trajectory_euler_support_.setZero();

                rfoot_trajectory_support_.translation() = rfoot_support_init_.translation();
                rfoot_trajectory_support_.translation()(2) = 0;
                rfoot_trajectory_euler_support_ = rfoot_support_euler_init_;

                if(walking_tick_mj > t_start_ + t_rest_init_1 + t_rest_temp)
                {
                    //com_alpha = DyrosMath::QuinticSpline(walking_tick_mj, t_start_ + t_rest_init_ + t_rest_temp, t_start_ + t_rest_init_ + t_double1_ + t_rest_temp, 0.5, 0.0, 0.0, 0.0);
                }
            }
            else if (foot_step_(current_step_num_, 6) == 0) // 오른발 지지
            {
                rfoot_trajectory_support_.translation().setZero();
                rfoot_trajectory_euler_support_.setZero();

                lfoot_trajectory_support_.translation() = lfoot_support_init_.translation();
                lfoot_trajectory_support_.translation()(2) = 0;
                lfoot_trajectory_euler_support_ = lfoot_support_euler_init_;

                if(walking_tick_mj > t_start_ + t_rest_init_1  + t_rest_temp)
                {
                    //com_alpha = DyrosMath::QuinticSpline(walking_tick_mj, t_start_ + t_rest_init_ + t_rest_temp, t_start_ + t_rest_init_ + t_double1_ + t_rest_temp, 0.5, 1.0, 0.0, 0.0);
                }
            }
   
            lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(F_T_L_y_input) * DyrosMath::rotateWithX(-F_T_L_x_input);
            rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(F_T_R_y_input) * DyrosMath::rotateWithX(-F_T_R_x_input);
            rfootd1.setZero();
            lfootd1.setZero();
        }
        else if (walking_tick_mj >= t_start_ + t_rest_init_1 + t_double1_ && walking_tick_mj < t_start_ + t_total_ - t_double2_ - t_rest_last_1)
        {
            double t_rest_temp = 0.00 * hz_;
            if (foot_step_(current_step_num_, 6) == 1)
            {
                lfoot_trajectory_support_.translation() = lfoot_support_init_.translation();
                lfoot_trajectory_euler_support_.setZero();

                lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(F_T_L_y_input) * DyrosMath::rotateWithX(-F_T_L_x_input);
                lfootd1.setZero();
                if (walking_tick_mj < t_start_ + t_rest_init_1 + t_double1_ + (t_total_ - t_rest_init_1 - t_rest_last_1 - t_double1_ - t_double2_) / 2.0)
                {
                    contactMode = 2;
                    rfoot_trajectory_support_.translation()(2) = DyrosMath::QuinticSpline(walking_tick_mj, t_start_ + t_rest_init_1 + t_double1_ + t_rest_temp, t_start_real_ + t_double1_ + (t_total_ - t_rest_init_1 - t_rest_last_1 - t_double1_ - t_double2_) / 2.0, 0.0, 0.0, 0.0, foot_height_, 0.0, 0.0)(0);
                    rfootd1(2) = DyrosMath::QuinticSpline(walking_tick_mj, t_start_ + t_rest_init_1 + t_double1_ + t_rest_temp, t_start_real_ + t_double1_ + (t_total_ - t_rest_init_1 - t_rest_last_1 - t_double1_ - t_double2_) / 2.0, 0.0, 0.0, 0.0, foot_height_, 0.0, 0.0)(1);
                }
                else
                {
                    contactMode = 2;
                    rfoot_trajectory_support_.translation()(2) = DyrosMath::QuinticSpline(walking_tick_mj, t_start_real_ + t_double1_ + (t_total_ - t_rest_init_1 - t_rest_last_1 - t_double1_ - t_double2_) / 2.0, t_start_ + t_total_ - t_rest_last_1 - t_double2_, foot_height_, 0.0, 0.0, target_swing_foot(2), 0.0, 0.0)(0);
                    rfootd1(2) = DyrosMath::QuinticSpline(walking_tick_mj, t_start_real_ + t_double1_ + (t_total_ - t_rest_init_1 - t_rest_last_1 - t_double1_ - t_double2_) / 2.0, t_start_ + t_total_ - t_rest_last_1 - t_double2_, foot_height_, 0.0, 0.0, target_swing_foot(2), 0.0, 0.0)(1);
                }

                for (int i = 0; i < 2; i++)
                {
                    rfoot_trajectory_support_.translation()(i) = DyrosMath::QuinticSpline(walking_tick_mj, t_start_real_ + t_double1_ + t_rest_temp, t_start_ + t_total_ - t_rest_last_1 - t_double2_, rfoot_support_init_.translation()(i), 0.0, 0.0, target_swing_foot(i), 0.0, 0.0)(0);
                    rfootd1(i) = DyrosMath::QuinticSpline(walking_tick_mj, t_start_real_ + t_double1_ + t_rest_temp, t_start_ + t_total_ - t_rest_last_1 - t_double2_, rfoot_support_init_.translation()(i), 0.0, 0.0, target_swing_foot(i), 0.0, 0.0)(1);
                }

                rfoot_trajectory_euler_support_(0) = 0;
                rfoot_trajectory_euler_support_(1) = 0;
                rfoot_trajectory_euler_support_(2) = DyrosMath::QuinticSpline(walking_tick_mj, t_start_ + t_rest_init_1 + t_double1_, t_start_ + t_total_ - t_rest_last_1 - t_double2_, rfoot_support_euler_init_(2), 0.0, 0.0, target_swing_foot(5), 0.0, 0.0)(0);
                rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(F_T_R_y_input) * DyrosMath::rotateWithX(-F_T_R_x_input);
            }
            else if (foot_step_(current_step_num_, 6) == 0)
            {
                rfoot_trajectory_support_.translation() = rfoot_support_init_.translation();
                rfoot_trajectory_euler_support_.setZero();

                rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(F_T_R_y_input) * DyrosMath::rotateWithX(-F_T_R_x_input);
                rfootd1.setZero();
                if (walking_tick_mj < t_start_ + t_rest_init_1 + t_double1_ + (t_total_ - t_rest_init_1 - t_rest_last_1 - t_double1_ - t_double2_) / 2.0)
                {
                    contactMode = 3;
                    lfoot_trajectory_support_.translation()(2) = DyrosMath::QuinticSpline(walking_tick_mj, t_start_real_ + t_double1_ + t_rest_temp, t_start_real_ + t_double1_ + (t_total_ - t_rest_init_1 - t_rest_last_1 - t_double1_ - t_double2_) / 2.0, 0, 0.0, 0.0, foot_height_, 0.0, 0.0)(0);
                    lfootd1(2) = DyrosMath::QuinticSpline(walking_tick_mj, t_start_real_ + t_double1_ + t_rest_temp, t_start_real_ + t_double1_ + (t_total_ - t_rest_init_1 - t_rest_last_1 - t_double1_ - t_double2_) / 2.0, 0, 0.0, 0.0, foot_height_, 0.0, 0.0)(1);
                }
                else
                {
                    contactMode = 3;
                    lfoot_trajectory_support_.translation()(2) = DyrosMath::QuinticSpline(walking_tick_mj, t_start_real_ + t_double1_ + (t_total_ - t_rest_init_1 - t_rest_last_1 - t_double1_ - t_double2_) / 2.0, t_start_ + t_total_ - t_rest_last_1 - t_double2_, foot_height_, 0.0, 0.0, target_swing_foot(2), 0.0, 0.0)(0);
                    lfootd1(2) = DyrosMath::QuinticSpline(walking_tick_mj, t_start_real_ + t_double1_ + (t_total_ - t_rest_init_1 - t_rest_last_1 - t_double1_ - t_double2_) / 2.0, t_start_ + t_total_ - t_rest_last_1 - t_double2_, foot_height_, 0.0, 0.0, target_swing_foot(2), 0.0, 0.0)(1);
                }

                for (int i = 0; i < 2; i++)
                {
                    lfoot_trajectory_support_.translation()(i) = DyrosMath::QuinticSpline(walking_tick_mj, t_start_real_ + t_double1_ + t_rest_temp, t_start_ + t_total_ - t_rest_last_1 - t_double2_, lfoot_support_init_.translation()(i), 0.0, 0.0, target_swing_foot(i), 0.0, 0.0)(0);
                    lfootd1(i) = DyrosMath::QuinticSpline(walking_tick_mj, t_start_real_ + t_double1_ + t_rest_temp, t_start_ + t_total_ - t_rest_last_1 - t_double2_, lfoot_support_init_.translation()(i), 0.0, 0.0, target_swing_foot(i), 0.0, 0.0)(1);
                }

                lfoot_trajectory_euler_support_(0) = 0;
                lfoot_trajectory_euler_support_(1) = 0;
                lfoot_trajectory_euler_support_(2) = DyrosMath::QuinticSpline(walking_tick_mj, t_start_real_ + t_double1_, t_start_ + t_total_ - t_rest_last_1 - t_double2_, lfoot_support_euler_init_(2), 0.0, 0.0, target_swing_foot(5), 0.0, 0.0)(0);
                lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(F_T_L_y_input) * DyrosMath::rotateWithX(-F_T_L_x_input);
            }
        }
        else
        {
            contactMode = 1;
            lfootd1.setZero();
            rfootd1.setZero();
            if (foot_step_(current_step_num_, 6) == 1)
            {
                lfoot_trajectory_euler_support_.setZero();
                lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(F_T_L_y_input) * DyrosMath::rotateWithX(-F_T_L_x_input);

                for (int i = 0; i < 3; i++)
                {
                    rfoot_trajectory_support_.translation()(i) = target_swing_foot(i);
                    rfoot_trajectory_euler_support_(i) = target_swing_foot(i + 3);
                }

                rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(F_T_R_y_input) * DyrosMath::rotateWithX(-F_T_R_x_input);
            }
            else if (foot_step_(current_step_num_, 6) == 0)
            {
                rfoot_trajectory_euler_support_.setZero();
                rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(F_T_R_y_input) * DyrosMath::rotateWithX(-F_T_R_x_input);

                for (int i = 0; i < 3; i++)
                {
                    lfoot_trajectory_support_.translation()(i) = target_swing_foot(i);
                    lfoot_trajectory_euler_support_(i) = target_swing_foot(i + 3);
                }
                lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(F_T_L_y_input) * DyrosMath::rotateWithX(-F_T_L_x_input);
            }
        }
    }
    else if(current_step_num_ == 2)
    {
        if (walking_tick_mj < t_start_ + t_rest_init_2 + t_double1_)
        {
            lfootd1.setZero();
            rfootd1.setZero();
           
            double t_rest_temp = 0.00 * hz_;
            contactMode = 1;
            if (foot_step_(current_step_num_, 6) == 1) // 왼발 지지
            {
                lfoot_trajectory_support_.translation().setZero();
                lfoot_trajectory_euler_support_.setZero();

                rfoot_trajectory_support_.translation() = rfoot_support_init_.translation();
                rfoot_trajectory_support_.translation()(2) = 0;
                rfoot_trajectory_euler_support_ = rfoot_support_euler_init_;

                if(walking_tick_mj > t_start_ + t_rest_init_2 + t_rest_temp)
                {
                    //com_alpha = DyrosMath::QuinticSpline(walking_tick_mj, t_start_ + t_rest_init_ + t_rest_temp, t_start_ + t_rest_init_ + t_double1_ + t_rest_temp, 0.5, 0.0, 0.0, 0.0);
                }
            }
            else if (foot_step_(current_step_num_, 6) == 0) // 오른발 지지
            {
                rfoot_trajectory_support_.translation().setZero();
                rfoot_trajectory_euler_support_.setZero();

                lfoot_trajectory_support_.translation() = lfoot_support_init_.translation();
                lfoot_trajectory_support_.translation()(2) = 0;
                lfoot_trajectory_euler_support_ = lfoot_support_euler_init_;

                if(walking_tick_mj > t_start_ + t_rest_init_2  + t_rest_temp)
                {
                    //com_alpha = DyrosMath::QuinticSpline(walking_tick_mj, t_start_ + t_rest_init_ + t_rest_temp, t_start_ + t_rest_init_ + t_double1_ + t_rest_temp, 0.5, 1.0, 0.0, 0.0);
                }
            }
   
            lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(F_T_L_y_input) * DyrosMath::rotateWithX(-F_T_L_x_input);
            rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(F_T_R_y_input) * DyrosMath::rotateWithX(-F_T_R_x_input);
        }
        else if (walking_tick_mj >= t_start_ + t_rest_init_2 + t_double1_ && walking_tick_mj < t_start_ + t_total_ - t_double2_ - t_rest_last_2)
        {
            double t_rest_temp = 0.00 * hz_;
            if (foot_step_(current_step_num_, 6) == 1)
            {
                lfoot_trajectory_support_.translation() = lfoot_support_init_.translation();
                lfoot_trajectory_euler_support_.setZero();

                lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(F_T_L_y_input) * DyrosMath::rotateWithX(-F_T_L_x_input);
                lfootd1.setZero();
           
                if (walking_tick_mj < t_start_ + t_rest_init_2 + t_double1_ + (t_total_ - t_rest_init_2 - t_rest_last_2 - t_double1_ - t_double2_) / 2.0)
                {
                    contactMode = 2;
                    rfoot_trajectory_support_.translation()(2) = DyrosMath::QuinticSpline(walking_tick_mj, t_start_ + t_rest_init_2 + t_double1_ + t_rest_temp, t_start_real_ + t_double1_ + (t_total_ - t_rest_init_2 - t_rest_last_2 - t_double1_ - t_double2_) / 2.0, 0, 0.0, 0.0, foot_height_, 0.0, 0.0)(0);
                    rfootd1(2)  = DyrosMath::QuinticSpline(walking_tick_mj, t_start_ + t_rest_init_2 + t_double1_ + t_rest_temp, t_start_real_ + t_double1_ + (t_total_ - t_rest_init_2 - t_rest_last_2 - t_double1_ - t_double2_) / 2.0, 0, 0.0, 0.0, foot_height_, 0.0, 0.0)(1);
                }
                else
                {
                    contactMode = 2;
                    rfoot_trajectory_support_.translation()(2) = DyrosMath::QuinticSpline(walking_tick_mj, t_start_real_ + t_double1_ + (t_total_ - t_rest_init_2 - t_rest_last_2 - t_double1_ - t_double2_) / 2.0, t_start_ + t_total_ - t_rest_last_2 - t_double2_, foot_height_, 0.0, 0.0, target_swing_foot(2), 0.0, 0.0)(0);
                    rfootd1(2) = DyrosMath::QuinticSpline(walking_tick_mj, t_start_real_ + t_double1_ + (t_total_ - t_rest_init_2 - t_rest_last_2 - t_double1_ - t_double2_) / 2.0, t_start_ + t_total_ - t_rest_last_2 - t_double2_, foot_height_, 0.0, 0.0, target_swing_foot(2), 0.0, 0.0)(1);
                }

                for (int i = 0; i < 2; i++)
                {
                    rfoot_trajectory_support_.translation()(i) = DyrosMath::QuinticSpline(walking_tick_mj, t_start_real_ + t_double1_ + t_rest_temp, t_start_ + t_total_ - t_rest_last_2 - t_double2_, rfoot_support_init_.translation()(i), 0.0, 0.0, target_swing_foot(i), 0.0, 0.0)(0);
                    rfootd1(i) =  DyrosMath::QuinticSpline(walking_tick_mj, t_start_real_ + t_double1_ + t_rest_temp, t_start_ + t_total_ - t_rest_last_2 - t_double2_, rfoot_support_init_.translation()(i), 0.0, 0.0, target_swing_foot(i), 0.0, 0.0)(1);
                }

                rfoot_trajectory_euler_support_(0) = 0;
                rfoot_trajectory_euler_support_(1) = 0;
                rfoot_trajectory_euler_support_(2) = DyrosMath::QuinticSpline(walking_tick_mj, t_start_ + t_rest_init_2 + t_double1_, t_start_ + t_total_ - t_rest_last_2 - t_double2_, rfoot_support_euler_init_(2), 0.0, 0.0, target_swing_foot(5), 0.0, 0.0)(0);
                rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(F_T_R_y_input) * DyrosMath::rotateWithX(-F_T_R_x_input);
            }
            else if (foot_step_(current_step_num_, 6) == 0)
            {
                rfoot_trajectory_support_.translation() = rfoot_support_init_.translation();
                rfoot_trajectory_euler_support_.setZero();

                rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(F_T_R_y_input) * DyrosMath::rotateWithX(-F_T_R_x_input);
                rfootd1.setZero();

                if (walking_tick_mj < t_start_ + t_rest_init_2 + t_double1_ + (t_total_ - t_rest_init_2 - t_rest_last_2 - t_double1_ - t_double2_) / 2.0)
                {
                    contactMode = 3;
                    lfoot_trajectory_support_.translation()(2) = DyrosMath::QuinticSpline(walking_tick_mj, t_start_real_ + t_double1_ + t_rest_temp, t_start_real_ + t_double1_ + (t_total_ - t_rest_init_2 - t_rest_last_2 - t_double1_ - t_double2_) / 2.0, 0, 0.0, 0.0, foot_height_, 0.0, 0.0)(0);
                    lfootd1(2) = DyrosMath::QuinticSpline(walking_tick_mj, t_start_real_ + t_double1_ + t_rest_temp, t_start_real_ + t_double1_ + (t_total_ - t_rest_init_2 - t_rest_last_2 - t_double1_ - t_double2_) / 2.0, 0, 0.0, 0.0, foot_height_, 0.0, 0.0)(1);
                }
                else
                {
                    contactMode = 3;
                    lfoot_trajectory_support_.translation()(2) = DyrosMath::QuinticSpline(walking_tick_mj, t_start_real_ + t_double1_ + (t_total_ - t_rest_init_2 - t_rest_last_2 - t_double1_ - t_double2_) / 2.0, t_start_ + t_total_ - t_rest_last_2 - t_double2_, foot_height_, 0.0, 0.0, target_swing_foot(2), 0.0, 0.0)(0);
                    lfootd1(2) = DyrosMath::QuinticSpline(walking_tick_mj, t_start_real_ + t_double1_ + (t_total_ - t_rest_init_2 - t_rest_last_2 - t_double1_ - t_double2_) / 2.0, t_start_ + t_total_ - t_rest_last_2 - t_double2_, foot_height_, 0.0, 0.0, target_swing_foot(2), 0.0, 0.0)(1);
                }

                for (int i = 0; i < 2; i++)
                {
                    lfoot_trajectory_support_.translation()(i) = DyrosMath::QuinticSpline(walking_tick_mj, t_start_real_ + t_double1_ + t_rest_temp, t_start_ + t_total_ - t_rest_last_2 - t_double2_, lfoot_support_init_.translation()(i), 0.0, 0.0, target_swing_foot(i), 0.0, 0.0)(0);
                    lfootd1(i) = DyrosMath::QuinticSpline(walking_tick_mj, t_start_real_ + t_double1_ + t_rest_temp, t_start_ + t_total_ - t_rest_last_2 - t_double2_, lfoot_support_init_.translation()(i), 0.0, 0.0, target_swing_foot(i), 0.0, 0.0)(1);
                }

                lfoot_trajectory_euler_support_(0) = 0;
                lfoot_trajectory_euler_support_(1) = 0;
                lfoot_trajectory_euler_support_(2) = DyrosMath::QuinticSpline(walking_tick_mj, t_start_real_ + t_double1_, t_start_ + t_total_ - t_rest_last_2 - t_double2_, lfoot_support_euler_init_(2), 0.0, 0.0, target_swing_foot(5), 0.0, 0.0)(0);
                lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(F_T_L_y_input) * DyrosMath::rotateWithX(-F_T_L_x_input);
            }
        }
        else
        {
            contactMode = 1;
            lfootd1.setZero();
            rfootd1.setZero();
           
            if (foot_step_(current_step_num_, 6) == 1)
            {
                lfoot_trajectory_euler_support_.setZero();
                lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(F_T_L_y_input) * DyrosMath::rotateWithX(-F_T_L_x_input);

                for (int i = 0; i < 3; i++)
                {
                    rfoot_trajectory_support_.translation()(i) = target_swing_foot(i);
                    rfoot_trajectory_euler_support_(i) = target_swing_foot(i + 3);
                }

                rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(F_T_R_y_input) * DyrosMath::rotateWithX(-F_T_R_x_input);
            }
            else if (foot_step_(current_step_num_, 6) == 0)
            {
                rfoot_trajectory_euler_support_.setZero();
                rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(F_T_R_y_input) * DyrosMath::rotateWithX(-F_T_R_x_input);

                for (int i = 0; i < 3; i++)
                {
                    lfoot_trajectory_support_.translation()(i) = target_swing_foot(i);
                    lfoot_trajectory_euler_support_(i) = target_swing_foot(i + 3);
                }
                lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(F_T_L_y_input) * DyrosMath::rotateWithX(-F_T_L_x_input);
            }
        }
    }
    else
    {
        if (walking_tick_mj < t_start_ + t_rest_init_ + t_double1_)
        {
            double t_rest_temp = 0.00 * hz_;
            contactMode = 1;
            //com_alpha = 0.5;
            lfootd1.setZero();
            rfootd1.setZero();
           
            if (foot_step_(current_step_num_, 6) == 1) // 왼발 지지
            {
                a_temp = 0.0;
                lfoot_trajectory_support_.translation().setZero();
                lfoot_trajectory_euler_support_.setZero();

                rfoot_trajectory_support_.translation() = rfoot_support_init_.translation();
                rfoot_trajectory_support_.translation()(2) = 0;
                rfoot_trajectory_euler_support_ = rfoot_support_euler_init_;

                if(walking_tick_mj > t_start_ + t_rest_init_ + t_rest_temp)
                {
                    //com_alpha = DyrosMath::QuinticSpline(walking_tick_mj, t_start_ + t_rest_init_ + t_rest_temp, t_start_ + t_rest_init_ + t_double1_ + t_rest_temp, 0.5, 0.0, 0.0, 0.0);
                }

                if(walking_tick_mj == 14000 || walking_tick_mj == 14001 || walking_tick_mj == 14002)
                {
                    std::cout << "aa " << walking_tick_mj << " " <<rfoot_trajectory_support_.translation()(0) << " " <<lfoot_trajectory_support_.translation()(0) << " " << lfoot_support_init_.translation()(0) << " " << lfoot_support_init_.translation()(1) << " "<<lfoot_support_init_.translation()(2)<< " " << rfoot_support_init_.translation()(0) << " " << rfoot_support_init_.translation()(1) << " "<<rfoot_support_init_.translation()(2)<< std::endl;
                }
            }
            else if (foot_step_(current_step_num_, 6) == 0) // 오른발 지지
            {
                a_temp = 1.0;
                rfoot_trajectory_support_.translation().setZero();
                rfoot_trajectory_euler_support_.setZero();

                lfoot_trajectory_support_.translation() = lfoot_support_init_.translation();
                lfoot_trajectory_support_.translation()(2) = 0;
                lfoot_trajectory_euler_support_ = lfoot_support_euler_init_;

                if(walking_tick_mj > t_start_ + t_rest_init_  + t_rest_temp)
                {
                    //com_alpha = DyrosMath::QuinticSpline(walking_tick_mj, t_start_ + t_rest_init_ + t_rest_temp, t_start_ + t_rest_init_ + t_double1_ + t_rest_temp, 0.5, 1.0, 0.0, 0.0);
                }
            }
   
            lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(F_T_L_y_input) * DyrosMath::rotateWithX(-F_T_L_x_input);
            rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(F_T_R_y_input) * DyrosMath::rotateWithX(-F_T_R_x_input);
        }
        else if (walking_tick_mj >= t_start_ + t_rest_init_ + t_double1_ && walking_tick_mj < t_start_ + t_total_ - t_double2_ - t_rest_last_)
        {
            double t_rest_temp = 0.00 * hz_;
            if (foot_step_(current_step_num_, 6) == 1)
            {
                a_temp = 2.0;
                //com_alpha = 1.0;
                lfootd1.setZero();
                lfoot_trajectory_support_.translation() = lfoot_support_init_.translation();
                lfoot_trajectory_euler_support_.setZero();

                lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(F_T_L_y_input) * DyrosMath::rotateWithX(-F_T_L_x_input);
           
                if (walking_tick_mj < t_start_ + t_rest_init_ + t_double1_ + (t_total_ - t_rest_init_ - t_rest_last_ - t_double1_ - t_double2_) / 2.0)
                {
                    contactMode = 2;
                    rfoot_trajectory_support_.translation()(2) = DyrosMath::QuinticSpline(walking_tick_mj, t_start_ + t_rest_init_ + t_double1_ + t_rest_temp, t_start_real_ + t_double1_ + (t_total_ - t_rest_init_ - t_rest_last_ - t_double1_ - t_double2_) / 2.0, 0, 0.0, 0.0, foot_height_, 0.0, 0.0)(0);
                    rfootd1(2) = DyrosMath::QuinticSpline(walking_tick_mj, t_start_ + t_rest_init_ + t_double1_ + t_rest_temp, t_start_real_ + t_double1_ + (t_total_ - t_rest_init_ - t_rest_last_ - t_double1_ - t_double2_) / 2.0, 0, 0.0, 0.0, foot_height_, 0.0, 0.0)(1);
                }
                else
                {
                    contactMode = 2;
                    rfoot_trajectory_support_.translation()(2) = DyrosMath::QuinticSpline(walking_tick_mj, t_start_real_ + t_double1_ + (t_total_ - t_rest_init_ - t_rest_last_ - t_double1_ - t_double2_) / 2.0, t_start_ + t_total_ - t_rest_last_ - t_double2_, foot_height_, 0.0, 0.0, target_swing_foot(2), 0.0, 0.0)(0);
                    rfootd1(2) = DyrosMath::QuinticSpline(walking_tick_mj, t_start_real_ + t_double1_ + (t_total_ - t_rest_init_ - t_rest_last_ - t_double1_ - t_double2_) / 2.0, t_start_ + t_total_ - t_rest_last_ - t_double2_, foot_height_, 0.0, 0.0, target_swing_foot(2), 0.0, 0.0)(1);
                }

                for (int i = 0; i < 2; i++)
                {
                    rfoot_trajectory_support_.translation()(i) = DyrosMath::QuinticSpline(walking_tick_mj, t_start_real_ + t_double1_ + t_rest_temp, t_start_ + t_total_ - t_rest_last_ - t_double2_, rfoot_support_init_.translation()(i), 0.0, 0.0, target_swing_foot(i), 0.0, 0.0)(0);
                    rfootd1(i) = DyrosMath::QuinticSpline(walking_tick_mj, t_start_real_ + t_double1_ + t_rest_temp, t_start_ + t_total_ - t_rest_last_ - t_double2_, rfoot_support_init_.translation()(i), 0.0, 0.0, target_swing_foot(i), 0.0, 0.0)(1);
                }

                rfoot_trajectory_euler_support_(0) = 0;
                rfoot_trajectory_euler_support_(1) = 0;
                rfoot_trajectory_euler_support_(2) = DyrosMath::QuinticSpline(walking_tick_mj, t_start_ + t_rest_init_ + t_double1_, t_start_ + t_total_ - t_rest_last_ - t_double2_, rfoot_support_euler_init_(2), 0.0, 0.0, target_swing_foot(5), 0.0, 0.0)(0);
                rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(F_T_R_y_input) * DyrosMath::rotateWithX(-F_T_R_x_input);
            }
            else if (foot_step_(current_step_num_, 6) == 0)
            {
                a_temp = 3.0;
                //com_alpha = 1.0;
                rfoot_trajectory_support_.translation() = rfoot_support_init_.translation();
                rfoot_trajectory_euler_support_.setZero();

                rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(F_T_R_y_input) * DyrosMath::rotateWithX(-F_T_R_x_input);
                rfootd1.setZero();

                if (walking_tick_mj < t_start_ + t_rest_init_ + t_double1_ + (t_total_ - t_rest_init_ - t_rest_last_ - t_double1_ - t_double2_) / 2.0)
                {
                    contactMode = 3;
                    lfoot_trajectory_support_.translation()(2) = DyrosMath::QuinticSpline(walking_tick_mj, t_start_real_ + t_double1_ + t_rest_temp, t_start_real_ + t_double1_ + (t_total_ - t_rest_init_ - t_rest_last_ - t_double1_ - t_double2_) / 2.0, 0, 0.0, 0.0, foot_height_, 0.0, 0.0)(0);
                    lfootd1(2) = DyrosMath::QuinticSpline(walking_tick_mj, t_start_real_ + t_double1_ + t_rest_temp, t_start_real_ + t_double1_ + (t_total_ - t_rest_init_ - t_rest_last_ - t_double1_ - t_double2_) / 2.0, 0, 0.0, 0.0, foot_height_, 0.0, 0.0)(1);
                }
                else
                {
                    contactMode = 3;
                    lfoot_trajectory_support_.translation()(2) = DyrosMath::QuinticSpline(walking_tick_mj, t_start_real_ + t_double1_ + (t_total_ - t_rest_init_ - t_rest_last_ - t_double1_ - t_double2_) / 2.0, t_start_ + t_total_ - t_rest_last_ - t_double2_, foot_height_, 0.0, 0.0, target_swing_foot(2), 0.0, 0.0)(0);
                    lfootd1(2) = DyrosMath::QuinticSpline(walking_tick_mj, t_start_real_ + t_double1_ + (t_total_ - t_rest_init_ - t_rest_last_ - t_double1_ - t_double2_) / 2.0, t_start_ + t_total_ - t_rest_last_ - t_double2_, foot_height_, 0.0, 0.0, target_swing_foot(2), 0.0, 0.0)(1);
                }

                for (int i = 0; i < 2; i++)
                {
                    lfoot_trajectory_support_.translation()(i) = DyrosMath::QuinticSpline(walking_tick_mj, t_start_real_ + t_double1_ + t_rest_temp, t_start_ + t_total_ - t_rest_last_ - t_double2_, lfoot_support_init_.translation()(i), 0.0, 0.0, target_swing_foot(i), 0.0, 0.0)(0);
                    lfootd1(i) =  DyrosMath::QuinticSpline(walking_tick_mj, t_start_real_ + t_double1_ + t_rest_temp, t_start_ + t_total_ - t_rest_last_ - t_double2_, lfoot_support_init_.translation()(i), 0.0, 0.0, target_swing_foot(i), 0.0, 0.0)(1);
                }

                lfoot_trajectory_euler_support_(0) = 0;
                lfoot_trajectory_euler_support_(1) = 0;
                lfoot_trajectory_euler_support_(2) = DyrosMath::QuinticSpline(walking_tick_mj, t_start_real_ + t_double1_, t_start_ + t_total_ - t_rest_last_ - t_double2_, lfoot_support_euler_init_(2), 0.0, 0.0, target_swing_foot(5), 0.0, 0.0)(0);
                lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(F_T_L_y_input) * DyrosMath::rotateWithX(-F_T_L_x_input);
                //lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(lfoot_trajectory_euler_support_(1)) * DyrosMath::rotateWithX(lfoot_trajectory_euler_support_(0));
            }
        }
        else
        {
            contactMode = 1;
            lfootd1.setZero();
            rfootd1.setZero();
           
            if (foot_step_(current_step_num_, 6) == 1)
            {
                a_temp = 4.0;
                //com_alpha = DyrosMath::QuinticSpline(walking_tick_mj, t_start_ + t_total_ - t_double2_ - t_rest_last_, t_start_ + t_total_ - t_rest_last_, 0.0, 0.5, 0.0, 0.0);
                lfoot_trajectory_euler_support_.setZero();
                //lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(lfoot_trajectory_euler_support_(1)) * DyrosMath::rotateWithX(lfoot_trajectory_euler_support_(0));
                lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(F_T_L_y_input) * DyrosMath::rotateWithX(-F_T_L_x_input);

                for (int i = 0; i < 3; i++)
                {
                    rfoot_trajectory_support_.translation()(i) = target_swing_foot(i);
                    rfoot_trajectory_euler_support_(i) = target_swing_foot(i + 3);
                }

                rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(F_T_R_y_input) * DyrosMath::rotateWithX(-F_T_R_x_input);
                //rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(rfoot_trajectory_euler_support_(1)) * DyrosMath::rotateWithX(rfoot_trajectory_euler_support_(0));
            }
            else if (foot_step_(current_step_num_, 6) == 0)
            {
                a_temp = 5.0;
                //com_alpha = DyrosMath::QuinticSpline(walking_tick_mj, t_start_ + t_total_ - t_double2_ - t_rest_last_, t_start_ + t_total_ - t_rest_last_, 1.0, 0.5, 0.0, 0.0);
                rfoot_trajectory_euler_support_.setZero();
                rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(F_T_R_y_input) * DyrosMath::rotateWithX(-F_T_R_x_input);

                //rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(rfoot_trajectory_euler_support_(1)) * DyrosMath::rotateWithX(rfoot_trajectory_euler_support_(0));

                for (int i = 0; i < 3; i++)
                {
                    lfoot_trajectory_support_.translation()(i) = target_swing_foot(i);
                    lfoot_trajectory_euler_support_(i) = target_swing_foot(i + 3);
                }
                lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(F_T_L_y_input) * DyrosMath::rotateWithX(-F_T_L_x_input);

                //lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2)) * DyrosMath::rotateWithY(lfoot_trajectory_euler_support_(1)) * DyrosMath::rotateWithX(lfoot_trajectory_euler_support_(0));
            }
        }

    }
}


void CustomController::preview_Parameter(double dt, int NL, Eigen::MatrixXd &Gi, Eigen::VectorXd &Gd, Eigen::MatrixXd &Gx, Eigen::MatrixXd &A, Eigen::VectorXd &B, Eigen::MatrixXd &C)
{
    A.resize(3, 3);
    A(0, 0) = 1.0;
    A(0, 1) = dt;
    A(0, 2) = dt * dt * 0.5;
    A(1, 0) = 0;
    A(1, 1) = 1.0;
    A(1, 2) = dt;
    A(2, 0) = 0;
    A(2, 1) = 0;
    A(2, 2) = 1;

    B.resize(3);
    B(0) = dt * dt * dt / 6;
    B(1) = dt * dt / 2;
    B(2) = dt;

    C.resize(1, 3);
    C(0, 0) = 1;
    C(0, 1) = 0;
    C(0, 2) = -0.71 / 9.81;

    Eigen::MatrixXd A_bar;
    Eigen::VectorXd B_bar;

    B_bar.resize(4);
    B_bar.segment(0, 1) = C * B;
    B_bar.segment(1, 3) = B;

    Eigen::Matrix1x4d B_bar_tran;
    B_bar_tran = B_bar.transpose();

    Eigen::MatrixXd I_bar;
    Eigen::MatrixXd F_bar;
    A_bar.resize(4, 4);
    I_bar.resize(4, 1);
    F_bar.resize(4, 3);
    F_bar.setZero();

    F_bar.block<1, 3>(0, 0) = C * A;
    F_bar.block<3, 3>(1, 0) = A;

    I_bar.setZero();
    I_bar(0, 0) = 1.0;

    A_bar.block<4, 1>(0, 0) = I_bar;
    A_bar.block<4, 3>(0, 1) = F_bar;

    Eigen::MatrixXd Qe;
    Qe.resize(1, 1);
    Qe(0, 0) = 1.0;

    Eigen::MatrixXd R;
    R.resize(1, 1);
    R(0, 0) = 0.000001;

    Eigen::MatrixXd Qx;
    Qx.resize(3, 3);
    Qx.setZero();

    Eigen::MatrixXd Q_bar;
    Q_bar.resize(3, 3);
    Q_bar.setZero();
    Q_bar(0, 0) = Qe(0, 0);

    Eigen::Matrix4d K;

    K(0, 0) = 1083.572780788710;
    K(0, 1) = 586523.188429418020;
    K(0, 2) = 157943.283121116518;
    K(0, 3) = 41.206077691894;
    K(1, 0) = 586523.188429418020;
    K(1, 1) = 319653984.254277825356;
    K(1, 2) = 86082274.531361579895;
    K(1, 3) = 23397.754069026785;
    K(2, 0) = 157943.283121116518;
    K(2, 1) = 86082274.531361579895;
    K(2, 2) = 23181823.112113621086;
    K(2, 3) = 6304.466397614751;
    K(3, 0) = 41.206077691894;
    K(3, 1) = 23397.754069026785;
    K(3, 2) = 6304.466397614751;
    K(3, 3) = 2.659250532188;

    Eigen::MatrixXd Temp_mat;
    Eigen::MatrixXd Temp_mat_inv;
    Eigen::MatrixXd Ac_bar;
    Temp_mat.resize(1, 1);
    Temp_mat.setZero();
    Temp_mat_inv.resize(1, 1);
    Temp_mat_inv.setZero();
    Ac_bar.setZero();
    Ac_bar.resize(4, 4);

    Temp_mat = R + B_bar_tran * K * B_bar;
    Temp_mat_inv = Temp_mat.inverse();

    Ac_bar = A_bar - B_bar * Temp_mat_inv * B_bar_tran * K * A_bar;

    Eigen::MatrixXd Ac_bar_tran(4, 4);
    Ac_bar_tran = Ac_bar.transpose();

    Gi.resize(1, 1);
    Gx.resize(1, 3);
    Gi(0, 0) = 872.3477; //Temp_mat_inv * B_bar_tran * K * I_bar ;
    //Gx = Temp_mat_inv * B_bar_tran * K * F_bar ;
    Gx(0, 0) = 945252.1760702;
    Gx(0, 1) = 256298.6905049;
    Gx(0, 2) = 542.0544196;
    Eigen::MatrixXd X_bar;
    Eigen::Vector4d X_bar_col;
    X_bar.resize(4, NL);
    X_bar.setZero();
    X_bar_col.setZero();
    X_bar_col = -Ac_bar_tran * K * I_bar;

    for (int i = 0; i < NL; i++)
    {
        X_bar.block<4, 1>(0, i) = X_bar_col;
        X_bar_col = Ac_bar_tran * X_bar_col;
    }

    Gd.resize(NL);
    Eigen::VectorXd Gd_col(1);
    Gd_col(0) = -Gi(0, 0);

    for (int i = 0; i < NL; i++)
    {
        Gd.segment(i, 1) = Gd_col;
        Gd_col = Temp_mat_inv * B_bar_tran * X_bar.col(i);
    }
}

void CustomController::previewcontroller(double dt, int NL, int tick, double x_i, double y_i, Eigen::Vector3d xs, Eigen::Vector3d ys, double &UX, double &UY,
                                         Eigen::MatrixXd Gi, Eigen::VectorXd Gd, Eigen::MatrixXd Gx, Eigen::MatrixXd A, Eigen::VectorXd B, Eigen::MatrixXd C, Eigen::Vector3d &XD, Eigen::Vector3d &YD)
{
    int zmp_size;
    zmp_size = ref_zmp_mj_.col(1).size();
    Eigen::VectorXd px_ref, py_ref;
    px_ref.resize(zmp_size);
    py_ref.resize(zmp_size);

    for (int i = 0; i < zmp_size; i++)
    {
        px_ref(i) = ref_zmp_mj_(i, 0);
        py_ref(i) = ref_zmp_mj_(i, 1);
    }

    ZMP_X_REF = px_ref(tick);
    ZMP_Y_REF = py_ref(tick);

    Eigen::VectorXd px, py;
    px.resize(1);
    py.resize(1);

    if (tick == 0 && current_step_num_ == 0)
    {
        preview_x_b_mj.setZero();
        preview_y_b_mj.setZero();
        preview_x_mj.setZero();
        preview_y_mj.setZero();
        preview_x_b_mj(0) = x_i;
        preview_y_b_mj(0) = y_i;
        preview_x_mj(0) = x_i;
        preview_y_mj(0) = y_i;
        UX = 0;
        UY = 0;
        cout << "preview X state : " << preview_x_mj(0) << "," << preview_x_mj(1) << "," << preview_x_mj(2) << endl;
        cout << "preview Y state : " << preview_y_mj(0) << "," << preview_y_mj(1) << "," << preview_y_mj(2) << endl;
    }
    else
    {
        preview_x_mj = xs;
        preview_y_mj = ys;

        preview_x_b_mj(0) = preview_x_mj(0) - preview_x_mj(1) * 0.0005;
        preview_y_b_mj(0) = preview_y_mj(0) - preview_y_mj(1) * 0.0005;
        preview_x_b_mj(1) = preview_x_mj(1) - preview_x_mj(2) * 0.0005;
        preview_y_b_mj(1) = preview_y_mj(1) - preview_y_mj(2) * 0.0005;
        preview_x_b_mj(2) = preview_x_mj(2) - UX * 0.0005;
        preview_y_b_mj(2) = preview_y_mj(2) - UY * 0.0005;
    }
    px = C * preview_x_mj;
    py = C * preview_y_mj;

    double sum_Gd_px_ref = 0, sum_Gd_py_ref = 0;

    for (int i = 0; i < NL; i++)
    {
        sum_Gd_px_ref = sum_Gd_px_ref + Gd(i) * (px_ref(tick + 1 + i) - px_ref(tick + i));
        sum_Gd_py_ref = sum_Gd_py_ref + Gd(i) * (py_ref(tick + 1 + i) - py_ref(tick + i));
    }

    Eigen::MatrixXd del_ux(1, 1);
    Eigen::MatrixXd del_uy(1, 1);
    del_ux.setZero();
    del_uy.setZero();

    Eigen::VectorXd GX_X(1);
    GX_X = Gx * (preview_x_mj - preview_x_b_mj);
    Eigen::VectorXd GX_Y(1);
    GX_Y = Gx * (preview_y_mj - preview_y_b_mj);

    if (walking_tick_mj == 0)
    {
        del_zmp.setZero();
        cout << "del_zmp : " << del_zmp(0) << "," << del_zmp(1) << endl;
    }

    del_ux(0, 0) = -(px(0) - px_ref(tick)) * Gi(0, 0) - GX_X(0) - sum_Gd_px_ref;
    del_uy(0, 0) = -(py(0) - py_ref(tick)) * Gi(0, 0) - GX_Y(0) - sum_Gd_py_ref;

    UX = UX + del_ux(0, 0);
    UY = UY + del_uy(0, 0);

    XD = A * preview_x_mj + B * UX;
    YD = A * preview_y_mj + B * UY;
    //SC_err_compen(XD(0), YD(0));
    if (walking_tick_mj == 0)
    {
        zmp_err_(0) = 0;
        zmp_err_(1) = 0;
    }
    else
    {
        zmp_err_(0) = zmp_err_(0) + (px_ref(tick) - zmp_measured_LPF_(0)) * 0.0005;
        zmp_err_(1) = zmp_err_(1) + (py_ref(tick) - zmp_measured_LPF_(1)) * 0.0005;
    }

    cp_desired_(0) = XD(0) + XD(1) / wn;
    cp_desired_(1) = YD(0) + YD(1) / wn;

    SC_err_compen(com_support_current_(0), com_support_current_(1));

    cp_measured_(0) = com_support_cp_(0) + com_float_current_dot_LPF(0) / wn;
    cp_measured_(1) = com_support_current_(1) + com_float_current_dot_LPF(1) / wn;

    // cout  << px_ref(tick) << "," << py_ref(tick) << "," << XD(0) << "," << YD(0) << endl;
    //MJ_graph << px_ref(tick) << "," << py_ref(tick) << "," << XD(0) << "," << YD(0) << endl;
    //MJ_graph << zmp_measured_mj_(0) << "," << zmp_measured_mj_(1) << endl;
}

void CustomController::SC_err_compen(double x_des, double y_des)
{
    if (walking_tick_mj == 0)
    {
        SC_com.setZero();
    }
    if (walking_tick_mj == t_start_ + t_total_ - 1 && current_step_num_ != total_step_num_ - 1) // step change 1 tick 이전
    {
        sc_err_before.setZero();
        sc_err_before(0) = com_support_current_(0) - foot_step_support_frame_(current_step_num_, 0); // 1.3으로 할꺼면 마지막에 더해야됨. SC_com을 이 함수보다 나중에 더하기 때문에
                                                                                                     // sc_err_before(1) = y_des - com_support_current_(1);
    }

    if (current_step_num_ != 0 && walking_tick_mj == t_start_) // step change
    {
        sc_err_after.setZero();
        sc_err_after(0) = com_support_current_(0);
        // sc_err_after(1) = y_des - com_support_current_(1);
        sc_err = sc_err_after - sc_err_before;
    }

    if (current_step_num_ != 0)
    {
        SC_com(0) = DyrosMath::cubic(walking_tick_mj, t_start_, t_start_ + 0.05 * hz_, sc_err(0), 0, 0.0, 0.0);
        SC_com(1) = DyrosMath::cubic(walking_tick_mj, t_start_, t_start_ + 0.05 * hz_, sc_err(1), 0, 0.0, 0.0);
    }

    if (current_step_num_ != total_step_num_ - 1)
    {
        if (current_step_num_ != 0 && walking_tick_mj >= t_start_ && walking_tick_mj < t_start_ + t_total_)
        {
            com_support_cp_(0) = com_support_current_(0) - SC_com(0);
        }
        else
        {
            com_support_cp_(0) = com_support_current_(0);
        }
    }
    else if (current_step_num_ == total_step_num_ - 1)
    {
        if (walking_tick_mj >= t_start_ && walking_tick_mj < t_start_ + 2 * t_total_)
        {
            com_support_cp_(0) = com_support_current_(0) - SC_com(0);
        }
    }
}

void CustomController::getPelvTrajectory()
{
    double pelv_transition_time = 2.0;
    double pelv_height_offset_ = 0.0;
    if (walking_enable_ == true)
    {
        //pelv_height_offset_ = DyrosMath::cubic(walking_tick_mj, 0, pelv_transition_time * hz_, 0.0, 0.05, 0.0, 0.0);
    }

    double z_rot = foot_step_support_frame_(current_step_num_, 5);
   
    pelv_trajectory_support_.linear() = pelv_support_current_init.linear();
    pelv_trajectory_support_.translation()(0) = pelv_support_current_.translation()(0) + 0.7 * (com_desired_(0) - 0.0 * damping_x - com_support_current_(0));
    pelv_trajectory_support_.translation()(1) = pelv_support_current_.translation()(1) + 0.9 * (com_desired_(1) - 0.0 * damping_y - com_support_current_(1));
    pelv_trajectory_support_.translation()(2) = com_desired_(2) - 0 * pelv_height_offset_;

    comd1(0) = 0.7 * (com_desired_(0) - 0.0 * damping_x - com_support_current_(0));
    comd1(1) = 0.9 * (com_desired_(1) - 0.0 * damping_y - com_support_current_(1));
    comd1(2) = 0.0;

    // MJ_graph << com_desired_(0) << "," << com_support_current_(0) << "," << com_desired_(1) << "," << com_support_current_(1) << endl;
    Eigen::Vector3d Trunk_trajectory_euler;
    Trunk_trajectory_euler.setZero();
   
    if (walking_tick_mj < t_start_ + t_rest_init_ + t_double1_)
    {
        Trunk_trajectory_euler(2) = pelv_support_euler_init_(2);
    }
    else if (walking_tick_mj >= t_start_ + t_rest_init_ + t_double1_ && walking_tick_mj < t_start_ + t_total_ - t_double2_ - t_rest_last_)
    {
        Trunk_trajectory_euler(2) = DyrosMath::cubic(walking_tick_mj, t_start_real_ + t_double1_, t_start_ + t_total_ - t_double2_ - t_rest_last_, pelv_support_euler_init_(2), z_rot / 2.0, 0.0, 0.0);
    }
    else
    {
        Trunk_trajectory_euler(2) = z_rot / 2.0;
    }

    // P_angle_i = P_angle_i + (0 - P_angle)*del_t;
    // Trunk_trajectory_euler(1) = 0.05*(0.0 - P_angle) + 1.5*P_angle_i;

    if(torque_control == false)
    {
        if (aa == 0 && walking_tick_mj == 0 && (walking_enable_ == true))
        {
            P_angle_input = 0;
            R_angle_input = 0;
        }

        P_angle_input_dot = 1.5 * (0.0 - P_angle) - 0.01 * P_angle_input;
        R_angle_input_dot = 2.0 * (0.0 - R_angle) - 0.01 * R_angle_input;

        P_angle_input = P_angle_input + P_angle_input_dot * del_t;
        R_angle_input = R_angle_input + R_angle_input_dot * del_t;

        if (R_angle_input > 3 * DEG2RAD) //1.5 degree
        {
            R_angle_input = 3 * DEG2RAD;
        }
        else if (R_angle_input < -3 * DEG2RAD)
        {
            R_angle_input = -3 * DEG2RAD;
        }

        if (P_angle_input > 5 * DEG2RAD) //5 degree
        {
            P_angle_input = 5 * DEG2RAD;
            // cout << "a" << endl;
        }
        else if (P_angle_input < -5 * DEG2RAD)
        {
            P_angle_input = -5 * DEG2RAD;
            // cout << "b" << endl;
        }
        //Trunk_trajectory_euler(0) = R_angle_input;
        Trunk_trajectory_euler(1) = P_angle_input;
    }
    else
    {
        if (aa == 0 && walking_tick_mj == 0 && (walking_enable_ == true))
        {
            P_angle_input = 0;
            R_angle_input = 0;
        }

        P_angle_input_dot = (1.5 * (0.0 - P_angle) - 0.01 * P_angle_input);
        R_angle_input_dot =  0.2 * (1.0 * (0.0 - R_angle) - 0.2 * R_angle_input);

        P_angle_input = P_angle_input + P_angle_input_dot * del_t;
        R_angle_input = R_angle_input + R_angle_input_dot * del_t;

        if (R_angle_input > 3 * DEG2RAD) //1.5 degree
        {
            R_angle_input = 3 * DEG2RAD;
        }
        else if (R_angle_input < -3 * DEG2RAD)
        {
            R_angle_input = -3 * DEG2RAD;
        }

        if (P_angle_input > 5 * DEG2RAD) //5 degree
        {
            P_angle_input = 5 * DEG2RAD;
            // cout << "a" << endl;
        }
        else if (P_angle_input < -5 * DEG2RAD)
        {
            P_angle_input = -5 * DEG2RAD;
            // cout << "b" << endl;
        }

        Trunk_trajectory_euler(1) = P_angle_input;
    }
   
    pelv_trajectory_support_.linear() = DyrosMath::rotateWithZ(Trunk_trajectory_euler(2)) * DyrosMath::rotateWithY(Trunk_trajectory_euler(1)) * DyrosMath::rotateWithX(Trunk_trajectory_euler(0));
}

void CustomController::supportToFloatPattern()
{
    pelv_trajectory_float_ = DyrosMath::inverseIsometry3d(pelv_trajectory_support_) * pelv_trajectory_support_;
    lfoot_trajectory_float_ = DyrosMath::inverseIsometry3d(pelv_trajectory_support_) * lfoot_trajectory_support_;
    rfoot_trajectory_float_ = DyrosMath::inverseIsometry3d(pelv_trajectory_support_) * rfoot_trajectory_support_;
   
    rfoot_trajectory_float_.translation()(2) = rfoot_trajectory_float_.translation()(2) + F_F_input * 0.5;
    lfoot_trajectory_float_.translation()(2) = lfoot_trajectory_float_.translation()(2) - F_F_input * 0.5;
}

void CustomController::getComTrajectory()
{
    if (walking_tick_mj == 0)
    {
        Gi_mj_.setZero();
        Gx_mj_.setZero();
        Gd_mj_.setZero();
        preview_Parameter(1.0 / hz_, 16 * hz_ / 10, Gi_mj_, Gd_mj_, Gx_mj_, A_mj_, B_mj_, C_mj_);
        xs_mj_(0) = xi_mj_;
        xs_mj_(1) = 0;
        xs_mj_(2) = 0;
        ys_mj_(0) = yi_mj_;
        ys_mj_(1) = 0;
        xs_mj_(2) = 0;
        UX_mj_ = 0;
        UY_mj_ = 0;
        xd_mj_ = xs_mj_;
    }

    if (current_step_num_ == 0)
    {
        zmp_start_time_mj_ = 0.0;
    }
    else
    {
        zmp_start_time_mj_ = t_start_;
    }

    previewcontroller(0.0005, 3200, walking_tick_mj - zmp_start_time_mj_, xi_mj_, yi_mj_, xs_mj_, ys_mj_, UX_mj_, UY_mj_, Gi_mj_, Gd_mj_, Gx_mj_, A_mj_, B_mj_, C_mj_, xd_mj_, yd_mj_);
   
    xs_mj_ = xd_mj_;
    ys_mj_ = yd_mj_;

    com_desired_(0) = xd_mj_(0);
    com_desired_(1) = yd_mj_(0);
    com_desired_(2) = pelv_support_current_init.translation()(2);

    com_dot_desired_(0) = xd_mj_(1);
    com_dot_desired_(1) = yd_mj_(1);
    com_dot_desired_(2) = 0.0;
   
    if (walking_tick_mj == t_start_ + t_total_ - 1 && current_step_num_ != total_step_num_ - 1)
    {
        Eigen::Vector3d com_pos_prev;
        Eigen::Vector3d com_pos;
        Eigen::Vector3d com_vel_prev;
        Eigen::Vector3d com_vel;
        Eigen::Vector3d com_acc_prev;
        Eigen::Vector3d com_acc;
        Eigen::Matrix3d temp_rot;
        Eigen::Vector3d temp_pos;

        temp_rot = DyrosMath::rotateWithZ(-foot_step_support_frame_(current_step_num_, 5));
        for (int i = 0; i < 3; i++)
            temp_pos(i) = foot_step_support_frame_(current_step_num_, i);

        com_pos_prev(0) = xs_mj_(0);
        com_pos_prev(1) = ys_mj_(0);
        com_pos = temp_rot * (com_pos_prev - temp_pos);

        com_vel_prev(0) = xs_mj_(1);
        com_vel_prev(1) = ys_mj_(1);
        com_vel_prev(2) = 0.0;
        com_vel = temp_rot * com_vel_prev;

        com_acc_prev(0) = xs_mj_(2);
        com_acc_prev(1) = ys_mj_(2);
        com_acc_prev(2) = 0.0;
        com_acc = temp_rot * com_acc_prev;

        xs_mj_(0) = com_pos(0);
        ys_mj_(0) = com_pos(1);
        xs_mj_(1) = com_vel(0);
        ys_mj_(1) = com_vel(1);
        xs_mj_(2) = com_acc(0);
        ys_mj_(2) = com_acc(1);
    }
}

void CustomController::computeIkControl_MJ(Eigen::Isometry3d float_trunk_transform, Eigen::Isometry3d float_lleg_transform, Eigen::Isometry3d float_rleg_transform, Eigen::Vector12d &q_des)
{
    Eigen::Vector3d R_r, R_D, L_r, L_D;

    L_D << 0.11, +0.1025, -0.1025;
    R_D << 0.11, -0.1025, -0.1025;

    L_r = float_lleg_transform.rotation().transpose() * (float_trunk_transform.translation() + float_trunk_transform.rotation() * L_D - float_lleg_transform.translation());
    R_r = float_rleg_transform.rotation().transpose() * (float_trunk_transform.translation() + float_trunk_transform.rotation() * R_D - float_rleg_transform.translation());

    double R_C = 0, L_C = 0, L_upper = 0.351, L_lower = 0.351, R_alpha = 0, L_alpha = 0;

    L_C = sqrt(pow(L_r(0), 2) + pow(L_r(1), 2) + pow(L_r(2), 2));
    R_C = sqrt(pow(R_r(0), 2) + pow(R_r(1), 2) + pow(R_r(2), 2));

    q_des(3) = (-acos((pow(L_upper, 2) + pow(L_lower, 2) - pow(L_C, 2)) / (2 * L_upper * L_lower)) + M_PI);
    q_des(9) = (-acos((pow(L_upper, 2) + pow(L_lower, 2) - pow(R_C, 2)) / (2 * L_upper * L_lower)) + M_PI);
    L_alpha = asin(L_upper / L_C * sin(M_PI - q_des(3)));
    R_alpha = asin(L_upper / R_C * sin(M_PI - q_des(9)));

    q_des(4) = -atan2(L_r(0), sqrt(pow(L_r(1), 2) + pow(L_r(2), 2))) - L_alpha;
    q_des(10) = -atan2(R_r(0), sqrt(pow(R_r(1), 2) + pow(R_r(2), 2))) - R_alpha;

    Eigen::Matrix3d R_Knee_Ankle_Y_rot_mat, L_Knee_Ankle_Y_rot_mat;
    Eigen::Matrix3d R_Ankle_X_rot_mat, L_Ankle_X_rot_mat;
    Eigen::Matrix3d R_Hip_rot_mat, L_Hip_rot_mat;

    L_Knee_Ankle_Y_rot_mat = DyrosMath::rotateWithY(-q_des(3) - q_des(4));
    L_Ankle_X_rot_mat = DyrosMath::rotateWithX(-q_des(5));
    R_Knee_Ankle_Y_rot_mat = DyrosMath::rotateWithY(-q_des(9) - q_des(10));
    R_Ankle_X_rot_mat = DyrosMath::rotateWithX(-q_des(11));

    L_Hip_rot_mat.setZero();
    R_Hip_rot_mat.setZero();

    L_Hip_rot_mat = float_trunk_transform.rotation().transpose() * float_lleg_transform.rotation() * L_Ankle_X_rot_mat * L_Knee_Ankle_Y_rot_mat;
    R_Hip_rot_mat = float_trunk_transform.rotation().transpose() * float_rleg_transform.rotation() * R_Ankle_X_rot_mat * R_Knee_Ankle_Y_rot_mat;

    q_des(0) = atan2(-L_Hip_rot_mat(0, 1), L_Hip_rot_mat(1, 1));                                                       // Hip yaw
    q_des(1) = atan2(L_Hip_rot_mat(2, 1), -L_Hip_rot_mat(0, 1) * sin(q_des(0)) + L_Hip_rot_mat(1, 1) * cos(q_des(0))); // Hip roll
    q_des(2) = atan2(-L_Hip_rot_mat(2, 0), L_Hip_rot_mat(2, 2));                                                       // Hip pitch
    q_des(3) = q_des(3);                                                                                               // Knee pitch
    q_des(4) = q_des(4);                                                                                               // Ankle pitch
    q_des(5) = atan2(L_r(1), L_r(2));                                                                                  // Ankle roll

    q_des(6) = atan2(-R_Hip_rot_mat(0, 1), R_Hip_rot_mat(1, 1));
    q_des(7) = atan2(R_Hip_rot_mat(2, 1), -R_Hip_rot_mat(0, 1) * sin(q_des(6)) + R_Hip_rot_mat(1, 1) * cos(q_des(6)));
    q_des(8) = atan2(-R_Hip_rot_mat(2, 0), R_Hip_rot_mat(2, 2));
    q_des(9) = q_des(9);
    q_des(10) = q_des(10);
    q_des(11) = atan2(R_r(1), R_r(2));

    if (walking_tick_mj == 0)
    {
        sc_joint_err.setZero();
    }

    if (walking_tick_mj == t_start_ + t_total_ - 1 && current_step_num_ != total_step_num_ - 1) // step change 1 tick 이전
    {                                                                                           //5.3, 0
        sc_joint_before.setZero();
        sc_joint_before = q_des;
    }
    if (current_step_num_ != 0 && walking_tick_mj == t_start_) // step change
    {                                                          //5.3005, 1
        sc_joint_after.setZero();
        sc_joint_after = q_des;

        sc_joint_err = sc_joint_after - sc_joint_before;
    }
    if (current_step_num_ != 0)
    {
        for (int i = 0; i < 12; i++)
        {
            SC_joint(i) = DyrosMath::cubic(walking_tick_mj, t_start_, t_start_ + 0.005 * hz_, sc_joint_err(i), 0.0, 0.0, 0.0);
        }

        if (walking_tick_mj >= t_start_ && walking_tick_mj < t_start_ + 0.005 * hz_)
        {
            q_des = q_des - SC_joint;
        }
    }
}

void CustomController::GravityCalculate_MJ()
{
    double contact_gain = 0.0;
    double eta = 0.9;
    VectorQd grav_;
    if(torque_control == false)
    {
        if (walking_tick_mj < t_start_ + t_rest_init_)
        {
            WBC::SetContact(rd_, 1, 1);
            Gravity_DSP_ = WBC::GravityCompensationTorque(rd_);
            Gravity_SSP_.setZero();
            contact_gain = 1.0;
            if (foot_step_(current_step_num_, 6) == 1) // 왼발 지지
            {
                Gravity_DSP_ = WBC::ContactForceRedistributionTorqueWalking(rd_, Gravity_DSP_, eta, contact_gain, 1);
            }
            else if (foot_step_(current_step_num_, 6) == 0) // 오른발 지지
            {
                Gravity_DSP_ = WBC::ContactForceRedistributionTorqueWalking(rd_, Gravity_DSP_, eta, contact_gain, 0);
            }
        }
        else if (walking_tick_mj >= t_start_ + t_rest_init_ && walking_tick_mj < t_start_ + t_rest_init_ + t_double1_) // 0.03 s
        {
            contact_gain = DyrosMath::cubic(walking_tick_mj, t_start_ + t_rest_init_, t_start_ + t_rest_init_ + t_double1_, 1.0, 0.0, 0.0, 0.0);

            WBC::SetContact(rd_, 1, 1);
            Gravity_DSP_ = WBC::GravityCompensationTorque(rd_);
            Gravity_SSP_.setZero();
            if (foot_step_(current_step_num_, 6) == 1) // 왼발 지지
            {
                Gravity_DSP_ = WBC::ContactForceRedistributionTorqueWalking(rd_, Gravity_DSP_, eta, contact_gain, 1);
            }
            else if (foot_step_(current_step_num_, 6) == 0) // 오른발 지지
            {
                Gravity_DSP_ = WBC::ContactForceRedistributionTorqueWalking(rd_, Gravity_DSP_, eta, contact_gain, 0);
            }
        }

        else if (walking_tick_mj >= t_start_ + t_rest_init_ + t_double1_ && walking_tick_mj < t_start_ + t_total_ - t_rest_last_ - t_double2_) // SSP
        {
            if (foot_step_(current_step_num_, 6) == 1) // 왼발 지지
            {
                WBC::SetContact(rd_, 1, 0);
                Gravity_SSP_ = WBC::GravityCompensationTorque(rd_);
                Gravity_SSP_(1) = 1.0 * Gravity_SSP_(1);
                Gravity_SSP_(5) = 1.0 * Gravity_SSP_(5);
            }
            else if (foot_step_(current_step_num_, 6) == 0) // 오른발 지지
            {
                WBC::SetContact(rd_, 0, 1);
                Gravity_SSP_ = WBC::GravityCompensationTorque(rd_);
                Gravity_SSP_(7) = 1.0 * Gravity_SSP_(7);
                Gravity_SSP_(11) = 1.0 * Gravity_SSP_(11);
            }
            Gravity_DSP_.setZero();
            contact_torque_MJ.setZero();
        }

        else if (walking_tick_mj >= t_start_ + t_total_ - t_rest_last_ - t_double2_ && walking_tick_mj < t_start_ + t_total_ - t_rest_last_)
        {
            contact_gain = DyrosMath::cubic(walking_tick_mj, t_start_ + t_total_ - t_rest_last_ - t_double2_, t_start_ + t_total_ - t_rest_last_, 0.0, 1.0, 0.0, 0.0);
            Gravity_SSP_.setZero();
            if (foot_step_(current_step_num_, 6) == 1) // 왼발 지지
            {
                WBC::SetContact(rd_, 1, 1);
                Gravity_DSP_ = WBC::GravityCompensationTorque(rd_);
                Gravity_DSP_ = WBC::ContactForceRedistributionTorqueWalking(rd_, Gravity_DSP_, eta, contact_gain, 1);
            }
            else if (foot_step_(current_step_num_, 6) == 0) // 오른발 지지
            {
                WBC::SetContact(rd_, 1, 1);
                Gravity_DSP_ = WBC::GravityCompensationTorque(rd_);
                Gravity_DSP_ = WBC::ContactForceRedistributionTorqueWalking(rd_, Gravity_DSP_, eta, contact_gain, 0);
            }
        }
        else if (walking_tick_mj >= t_start_ + t_total_ - t_rest_last_ && walking_tick_mj < t_start_ + t_total_)
        {
            contact_gain = 1.0;

            WBC::SetContact(rd_, 1, 1);
            Gravity_DSP_ = WBC::GravityCompensationTorque(rd_);

            Gravity_SSP_.setZero();
            if (foot_step_(current_step_num_, 6) == 1) // 왼발 지지
            {
                Gravity_DSP_ = WBC::ContactForceRedistributionTorqueWalking(rd_, Gravity_DSP_, eta, contact_gain, 1);
            }
            else if (foot_step_(current_step_num_, 6) == 0) // 오른발 지지
            {
                Gravity_DSP_ = WBC::ContactForceRedistributionTorqueWalking(rd_, Gravity_DSP_, eta, contact_gain, 0);
            }
        }
    }
    else
    {
        WBC::SetContact(rd_, 1, 1);
        Gravity_DSP_ = WBC::ContactForceRedistributionTorqueWalking(rd_, WBC::GravityCompensationTorque(rd_));
        Gravity_SSP_.setZero();
    }
    if (atb_grav_update_ == false)
    {
        atb_grav_update_ = true;
        Gravity_MJ_ = Gravity_DSP_ + Gravity_SSP_; // + contact_torque_MJ;
        atb_grav_update_ = false;
    }

    //return grav_;
}

void CustomController::setContact_custom()
{
    /*if(current_step_num_ <= 1)
    {
        if (walking_tick_mj < t_start_ + t_rest_init_1)
        {
            WBC::SetContact(rd_, 1, 1);
            a_temp1 = 0.5;
        }
        else if (walking_tick_mj >= t_start_ + t_rest_init_1 && walking_tick_mj < t_start_ + t_rest_init_1 + t_double1_) // 0.03 s
        {
            WBC::SetContact(rd_, 1, 1);
            a_temp1 = 0.5;
        }

        else if (walking_tick_mj >= t_start_ + t_rest_init_1 + t_double1_ && walking_tick_mj < t_start_ + t_total_ - t_rest_last_1 - t_double2_) // SSP
        {
            if (foot_step_(current_step_num_, 6) == 1) // 왼발 지지
            {
                WBC::SetContact(rd_, 1, 0);
                a_temp1 = 0.3;
            }
            else if (foot_step_(current_step_num_, 6) == 0) // 오른발 지지
            {
                WBC::SetContact(rd_, 0, 1);
                a_temp1 = 0.2;
            }
        }

        else if (walking_tick_mj >= t_start_ + t_total_ - t_rest_last_1 - t_double2_ && walking_tick_mj < t_start_ + t_total_ - t_rest_last_1)
        {
            if (foot_step_(current_step_num_, 6) == 1) // 왼발 지지
            {
                WBC::SetContact(rd_, 1, 1);
                a_temp1 = 0.5;
            }
            else if (foot_step_(current_step_num_, 6) == 0) // 오른발 지지
            {
                WBC::SetContact(rd_, 1, 1);
                a_temp1 = 0.5;
            }
        }
        else if (walking_tick_mj >= t_start_ + t_total_ - t_rest_last_1 && walking_tick_mj < t_start_ + t_total_)
        {
            WBC::SetContact(rd_, 1, 1);  
            a_temp1 = 0.5;
        }
    }
    else if(current_step_num_ == 2)
    {
        if (walking_tick_mj < t_start_ + t_rest_init_2)
        {
            WBC::SetContact(rd_, 1, 1);
            a_temp1 = 0.5;
        }
        else if (walking_tick_mj >= t_start_ + t_rest_init_2 && walking_tick_mj < t_start_ + t_rest_init_2 + t_double1_) // 0.03 s
        {
            WBC::SetContact(rd_, 1, 1);
            a_temp1 = 0.5;
        }

        else if (walking_tick_mj >= t_start_ + t_rest_init_2 + t_double1_ && walking_tick_mj < t_start_ + t_total_ - t_rest_last_2 - t_double2_) // SSP
        {
            if (foot_step_(current_step_num_, 6) == 1) // 왼발 지지
            {
                WBC::SetContact(rd_, 1, 0);
                a_temp1 = 0.3;
            }
            else if (foot_step_(current_step_num_, 6) == 0) // 오른발 지지
            {
                WBC::SetContact(rd_, 0, 1);
                a_temp1 = 0.2;
            }
        }

        else if (walking_tick_mj >= t_start_ + t_total_ - t_rest_last_2 - t_double2_ && walking_tick_mj < t_start_ + t_total_ - t_rest_last_2)
        {
            if (foot_step_(current_step_num_, 6) == 1) // 왼발 지지
            {
                WBC::SetContact(rd_, 1, 1);
                a_temp1 = 0.5;
            }
            else if (foot_step_(current_step_num_, 6) == 0) // 오른발 지지
            {
                WBC::SetContact(rd_, 1, 1);
                a_temp1 = 0.5;
            }
        }
        else if (walking_tick_mj >= t_start_ + t_total_ - t_rest_last_2 && walking_tick_mj < t_start_ + t_total_)
        {
            WBC::SetContact(rd_, 1, 1);  
            a_temp1 = 0.5;
        }
    }
    else
    {
        if (walking_tick_mj < t_start_ + t_rest_init_)
        {
            WBC::SetContact(rd_, 1, 1);
            a_temp1 = 0.5;
        }
        else if (walking_tick_mj >= t_start_ + t_rest_init_ && walking_tick_mj < t_start_ + t_rest_init_ + t_double1_) // 0.03 s
        {
            WBC::SetContact(rd_, 1, 1);
            a_temp1 = 0.5;
        }

        else if (walking_tick_mj >= t_start_ + t_rest_init_ + t_double1_ && walking_tick_mj < t_start_ + t_total_ - t_rest_last_ - t_double2_) // SSP
        {
            if (foot_step_(current_step_num_, 6) == 1) // 왼발 지지
            {
                WBC::SetContact(rd_, 1, 0);
                a_temp1 = 0.3;
            }
            else if (foot_step_(current_step_num_, 6) == 0) // 오른발 지지
            {
                WBC::SetContact(rd_, 0, 1);
                a_temp1 = 0.2;
            }
        }

        else if (walking_tick_mj >= t_start_ + t_total_ - t_rest_last_ - t_double2_ && walking_tick_mj < t_start_ + t_total_ - t_rest_last_)
        {
            if (foot_step_(current_step_num_, 6) == 1) // 왼발 지지
            {
                WBC::SetContact(rd_, 1, 1);
                a_temp1 = 0.5;
            }
            else if (foot_step_(current_step_num_, 6) == 0) // 오른발 지지
            {
                WBC::SetContact(rd_, 1, 1);
                a_temp1 = 0.5;
            }
        }
        else if (walking_tick_mj >= t_start_ + t_total_ - t_rest_last_ && walking_tick_mj < t_start_ + t_total_)
        {
            WBC::SetContact(rd_, 1, 1);  
            a_temp1 = 0.5;
        }
    }*/

    if(contactMode_fast == 1)
    {
        setContact_1(rd_, 1, 1, 0, 0);
        step_test = 0.1;
    }
    else if(contactMode_fast == 2)
    {
        setContact_1(rd_, 1, 0, 0, 0);
        step_test = 0.2;
    }
    else
    {
        setContact_1(rd_, 0, 1, 0, 0);
        step_test = 0.3;
    }
}

void CustomController::parameterSetting()
{
    target_x_ = 0.5;
    target_y_ = 0.0;
    target_z_ = 0.0;
    com_height_ = 0.71;
    target_theta_ = 0.0;
    step_length_x_ = 0.1;
    step_length_y_ = 0.0;

    is_right_foot_swing_ = 0;

    if(as == 1)
    {
        t_rest_init_ = 0.196 * hz_; // slack 18
        t_rest_last_ = 0.206 * hz_; // slack 22
        t_rest_init_1 = 0.196 * hz_;
        t_rest_last_1 = 0.206 * hz_;
        t_rest_init_2 = 0.196 * hz_;
        t_rest_last_2 = 0.206 * hz_;

        /*t_rest_init_ = 0.19 * hz_; // slack 18
        t_rest_last_ = 0.21 * hz_; // slack 22
        t_rest_init_1 = 0.20 * hz_;
        t_rest_last_1 = 0.20 * hz_;
        t_rest_init_2 = 0.19 * hz_;
        t_rest_last_2 = 0.21 * hz_;*/

        t_double1_ = 0.00 * hz_;
        t_double2_ = 0.00 * hz_;
        t_total_ = 1.0 * hz_;
        foot_height_ = 0.03;
    }
    else if(as == 0)
    {
        t_rest_init_ = 0.2 * hz_; // slack
        t_rest_last_ = 0.2 * hz_;
        t_rest_init_1 = t_rest_init_;
        t_rest_last_1 = t_rest_last_;
        t_rest_init_2 = t_rest_init_;
        t_rest_last_2 = t_rest_last_;
        t_double1_ = 0.03 * hz_;
        t_double2_ = 0.03 * hz_;
        t_total_ = 1.1 * hz_;
        foot_height_ = 0.05;
    }

    t_mpc_init_ = t_rest_init_;

    t_temp_ = 4.0 * hz_;
    t_last_ = t_total_ + t_temp_;
    t_start_ = t_temp_ + 1;
    t_start_real_ = t_start_ + t_rest_init_1;

    current_step_num_ = 0;
    pelv_height_offset_ = 0.0;
}

void CustomController::updateNextStepTime()
{
    if(as == 1)
    {
        if (walking_tick_mj == t_last_)
        {
            if (current_step_num_ != total_step_num_ - 1)
            {
                t_start_ = t_last_ + 1;
                t_start_real_ = t_start_ + t_rest_init_;
                t_last_ = t_start_ + t_total_ - 1;
                current_step_num_++;
            }
        }
        if(mpc_cycle >= 0)
        {
            if(walking_tick_stop == false)// || (walking_tick == 0 && walking_tick_stop == true))
            {  
                walking_tick_mj = walking_tick_mj + 1;

                if(walking_tick == 40)
                {
                    walking_tick = 0;
                    mpc_cycle = mpc_cycle + 1;
                }

                if(walking_tick == 0)
                    walking_tick_stop = true;
            }
        }
        else
            walking_tick_mj++;
       
    }
    else
    {
        if (walking_tick_mj == t_last_)
        {
            if (current_step_num_ != total_step_num_ - 1)
            {
                t_start_ = t_last_ + 1;
                t_start_real_ = t_start_ + t_rest_init_;
                t_last_ = t_start_ + t_total_ - 1;
                current_step_num_++;
            }
        }
        walking_tick_mj++;
    }
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

void CustomController::CLIPM_ZMP_compen_MJ(double XZMP_ref, double YZMP_ref)
{
    double Kp_x_ssp, Kv_x_ssp;
    double Kp_y_ssp, Kv_y_ssp;
    Kp_x_ssp = 30;
    Kv_x_ssp = 2;
    Kp_y_ssp = 40;
    Kv_y_ssp = 2;
    double del_t = 0.0005;

    if (walking_tick_mj == 0)
    {
        A_x_ssp.resize(2, 2);
        B_x_ssp.resize(2, 1);
        Ad_x_ssp.resize(2, 2);
        Bd_x_ssp.resize(2, 1);
        C_x_ssp.resize(1, 2);
        D_x_ssp.resize(1, 1);

        A_y_ssp.resize(2, 2);
        B_y_ssp.resize(2, 1);
        Ad_y_ssp.resize(2, 2);
        Bd_y_ssp.resize(2, 1);
        C_y_ssp.resize(1, 2);
        D_y_ssp.resize(1, 1);

        ff_gain_x_ssp.resize(1, 1);
        ff_gain_y_ssp.resize(1, 1);

        K_x_ssp.resize(1, 2);
        K_y_ssp.resize(1, 2);

        X_x_ssp.setZero();
        Y_x_ssp.resize(1, 1);

        X_y_ssp.setZero();
        Y_y_ssp.resize(1, 1);

        K_x_ssp(0, 0) = 0.0083;
        K_x_ssp(0, 1) = 0.19;
        // Control pole : -5 , damping : 0.7 (실제 로봇) // Control pole : -7 , damping : 0.9 (시뮬레이션)
        K_y_ssp(0, 0) = -0.375;
        K_y_ssp(0, 1) = 0.125;

        // Define the state space equation
        A_x_ssp(0, 0) = 0;
        A_x_ssp(0, 1) = 1;
        A_x_ssp(1, 0) = -Kp_x_ssp;
        A_x_ssp(1, 1) = -Kv_x_ssp;

        B_x_ssp(0, 0) = 0;
        B_x_ssp(1, 0) = Kp_x_ssp;

        Ad_x_ssp(0, 0) = 1 - 0.5 * Kp_x_ssp * del_t * del_t;
        Ad_x_ssp(0, 1) = del_t - 0.5 * Kv_x_ssp * del_t * del_t;
        Ad_x_ssp(1, 0) = -Kp_x_ssp * del_t;
        Ad_x_ssp(1, 1) = 1 - Kv_x_ssp * del_t;

        Bd_x_ssp(0, 0) = 0.5 * Kp_x_ssp * del_t * del_t;
        Bd_x_ssp(1, 0) = Kp_x_ssp * del_t;

        C_x_ssp(0, 0) = 1 + zc_mj_ / GRAVITY * Kp_x_ssp;
        C_x_ssp(0, 1) = zc_mj_ / GRAVITY * Kv_x_ssp;

        D_x_ssp(0, 0) = -zc_mj_ / GRAVITY * Kp_x_ssp;

        ff_gain_x_ssp = (-(C_x_ssp - D_x_ssp * K_x_ssp) * ((A_x_ssp - B_x_ssp * K_x_ssp).inverse()) * B_x_ssp + D_x_ssp).inverse();

        A_y_ssp(0, 0) = 0;
        A_y_ssp(0, 1) = 1;
        A_y_ssp(1, 0) = -Kp_y_ssp;
        A_y_ssp(1, 1) = -Kv_y_ssp;

        B_y_ssp(0, 0) = 0;
        B_y_ssp(1, 0) = Kp_y_ssp;

        Ad_y_ssp(0, 0) = 1 - 0.5 * Kp_y_ssp * del_t * del_t;
        Ad_y_ssp(0, 1) = del_t - 0.5 * Kv_y_ssp * del_t * del_t;
        Ad_y_ssp(1, 0) = -Kp_y_ssp * del_t;
        Ad_y_ssp(1, 1) = 1 - Kv_y_ssp * del_t;

        Bd_y_ssp(0, 0) = 0.5 * Kp_y_ssp * del_t * del_t;
        Bd_y_ssp(1, 0) = Kp_y_ssp * del_t;

        C_y_ssp(0, 0) = 1 + zc_mj_ / GRAVITY * Kp_y_ssp;
        C_y_ssp(0, 1) = zc_mj_ / GRAVITY * Kv_y_ssp;

        D_y_ssp(0, 0) = -zc_mj_ / GRAVITY * Kp_y_ssp;

        ff_gain_y_ssp = (-(C_y_ssp - D_y_ssp * K_y_ssp) * ((A_y_ssp - B_y_ssp * K_y_ssp).inverse()) * B_y_ssp + D_y_ssp).inverse();
    }

    //X_x_ssp(0) = com_float_current_(0);
    X_x_ssp(0) = com_support_current_(0);

    if (foot_step_(current_step_num_, 6) == 1) // 왼발 지지
    {
        X_y_ssp(0) = com_support_current_(1) - rfoot_support_current_.translation()(1) * 0.5;
    }
    else if (foot_step_(current_step_num_, 6) == 0)
    {
        X_y_ssp(0) = com_support_current_(1) - lfoot_support_current_.translation()(1) * 0.5;
    }

    U_ZMP_x_ssp = -(K_x_ssp(0, 0) * X_x_ssp(0) + K_x_ssp(0, 1) * preview_x_mj(1)) + XZMP_ref * ff_gain_x_ssp(0, 0);
    U_ZMP_y_ssp = -(K_y_ssp(0, 0) * X_y_ssp(0) + K_y_ssp(0, 1) * preview_y_mj(1)) + YZMP_ref * ff_gain_y_ssp(0, 0);

    U_ZMP_x_ssp_LPF = 1 / (1 + 2 * M_PI * 3.0 * del_t) * U_ZMP_x_ssp_LPF + (2 * M_PI * 3.0 * del_t) / (1 + 2 * M_PI * 3.0 * del_t) * U_ZMP_x_ssp;
    U_ZMP_y_ssp_LPF = 1 / (1 + 2 * M_PI * 6.0 * del_t) * U_ZMP_y_ssp_LPF + (2 * M_PI * 6.0 * del_t) / (1 + 2 * M_PI * 6.0 * del_t) * U_ZMP_y_ssp;
    if (walking_tick_mj == 0)
    {
        U_ZMP_x_ssp_LPF = U_ZMP_x_ssp;
        U_ZMP_y_ssp_LPF = U_ZMP_y_ssp;
    }

    damping_x = U_ZMP_x_ssp_LPF;
    damping_y = U_ZMP_y_ssp_LPF;

    if (damping_x > 0.02)
    {
        damping_x = 0.02;
    }
    else if (damping_x < -0.02)
    {
        damping_x = -0.02;
    }

    if (damping_y > 0.03) // 로봇 0.03, 시뮬 0.02
    {
        damping_y = 0.03;
    }
    else if (damping_y < -0.03)
    {
        damping_y = -0.03;
    }
}

void CustomController::hip_compensator()
{  //0.2 0.2 0.8 0.8
    /*double left_hip_roll = -0.8 * DEG2RAD, right_hip_roll = -0.8 * DEG2RAD, left_hip_roll_first = -0.9 * DEG2RAD, right_hip_roll_first = -0.9 * DEG2RAD, //실험, 제자리 0.6, 0.4
        left_hip_pitch = 0.4 * DEG2RAD, right_hip_pitch = 0.4 * DEG2RAD, left_hip_pitch_first = 0.75 * DEG2RAD, right_hip_pitch_first = 0.75 * DEG2RAD,    // 실험 , 제자리 0.75deg
        left_ank_pitch = 0.0 * DEG2RAD, right_ank_pitch = 0.0 * DEG2RAD, left_ank_pitch_first = 0.0 * DEG2RAD, right_ank_pitch_first = 0.0 * DEG2RAD, temp_time = 0.05 * hz_;
      */    
    double left_hip_roll = -0.4 * DEG2RAD, right_hip_roll = -0.7 * DEG2RAD, left_hip_roll_first = -0.4 * DEG2RAD, right_hip_roll_first = -0.7 * DEG2RAD, //실험, 제자리 0.6, 0.4
        left_hip_pitch = 0.4 * DEG2RAD, right_hip_pitch = 0.4 * DEG2RAD, left_hip_pitch_first = 0.4 * DEG2RAD, right_hip_pitch_first = 0.4 * DEG2RAD,    // 실험 , 제자리 0.75deg
        left_ank_pitch = 0.0 * DEG2RAD, right_ank_pitch = 0.0 * DEG2RAD, left_ank_pitch_first = 0.0 * DEG2RAD, right_ank_pitch_first = 0.0 * DEG2RAD,
           left_hip_roll_temp = 0.0, right_hip_roll_temp = 0.0, left_hip_pitch_temp = 0.0, right_hip_pitch_temp = 0.0, left_ank_pitch_temp = 0.0, right_ank_pitch_temp = 0.0, temp_time = 0.05 * hz_;

    if (current_step_num_ <= 1)
    {
        if (foot_step_(current_step_num_, 6) == 1) //left support foot
        {
            if (walking_tick_mj < t_start_ + t_total_ - t_rest_last_1 - t_double2_ - temp_time)
            {
                left_hip_roll_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_rest_init_1 + t_double1_, t_start_ + t_rest_init_1 + t_double1_ + temp_time, 0.0, left_hip_roll_first, 0.0, 0.0);
                left_hip_pitch_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_rest_init_1 + t_double1_, t_start_ + t_rest_init_1 + t_double1_ + temp_time, 0.0, left_hip_pitch_first, 0.0, 0.0);
                left_ank_pitch_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_rest_init_1 + t_double1_, t_start_ + t_rest_init_1 + t_double1_ + temp_time, 0.0, left_ank_pitch_first, 0.0, 0.0);
            }
            else if (walking_tick_mj >= t_start_ + t_total_ - t_rest_last_1 - t_double2_ - temp_time)
            {
                left_hip_roll_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_total_ - t_rest_last_1 - t_double2_ - temp_time, t_start_ + t_total_ - t_rest_last_1, left_hip_roll_first, 0.0, 0.0, 0.0);
                left_hip_pitch_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_total_ - t_rest_last_1 - t_double2_ - temp_time, t_start_ + t_total_ - t_rest_last_1, left_hip_pitch_first, 0.0, 0.0, 0.0);
                left_ank_pitch_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_total_ - t_rest_last_1 - t_double2_ - temp_time, t_start_ + t_total_ - t_rest_last_1, left_ank_pitch_first, 0.0, 0.0, 0.0);
            }
            else
            {
                left_hip_roll_temp = 0.0;
                left_hip_pitch_temp = 0.0;
                left_ank_pitch_temp = 0.0;
            }
        }
        else if (foot_step_(current_step_num_, 6) == 0) // right support foot
        {
            if (walking_tick_mj < t_start_ + t_total_ - t_rest_last_1 - t_double2_ - temp_time)
            {
                right_hip_roll_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_rest_init_1 + t_double1_, t_start_ + t_rest_init_1 + t_double1_ + temp_time, 0.0, right_hip_roll_first, 0.0, 0.0);
                right_hip_pitch_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_rest_init_1 + t_double1_, t_start_ + t_rest_init_1 + t_double1_ + temp_time, 0.0, right_hip_pitch_first, 0.0, 0.0);
                right_ank_pitch_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_rest_init_1 + t_double1_, t_start_ + t_rest_init_1 + t_double1_ + temp_time, 0.0, right_ank_pitch_first, 0.0, 0.0);
            }
            else if (walking_tick_mj >= t_start_ + t_total_ - t_rest_last_1 - t_double2_ - temp_time)
            {
                right_hip_roll_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_total_ - t_rest_last_1 - t_double2_ - temp_time, t_start_ + t_total_ - t_rest_last_1, right_hip_roll_first, 0.0, 0.0, 0.0);
                right_hip_pitch_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_total_ - t_rest_last_1 - t_double2_ - temp_time, t_start_ + t_total_ - t_rest_last_1, right_hip_pitch_first, 0.0, 0.0, 0.0);
                right_ank_pitch_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_total_ - t_rest_last_1 - t_double2_ - temp_time, t_start_ + t_total_ - t_rest_last_1, right_ank_pitch_first, 0.0, 0.0, 0.0);
            }
            else
            {
                right_hip_roll_temp = 0.0;
                right_hip_pitch_temp = 0.0;
                right_ank_pitch_temp = 0.0;
            }
        }
    }
    else if(current_step_num_ == 2)
    {
        if (foot_step_(current_step_num_, 6) == 1) //left support foot
        {
            if (walking_tick_mj < t_start_ + t_total_ - t_rest_last_2 - t_double2_ - temp_time)
            {
                left_hip_roll_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_rest_init_2 + t_double1_, t_start_ + t_rest_init_2 + t_double1_ + temp_time, 0.0, left_hip_roll_first, 0.0, 0.0);
                left_hip_pitch_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_rest_init_2 + t_double1_, t_start_ + t_rest_init_2 + t_double1_ + temp_time, 0.0, left_hip_pitch_first, 0.0, 0.0);
                left_ank_pitch_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_rest_init_2 + t_double1_, t_start_ + t_rest_init_2 + t_double1_ + temp_time, 0.0, left_ank_pitch_first, 0.0, 0.0);
            }
            else if (walking_tick_mj >= t_start_ + t_total_ - t_rest_last_2 - t_double2_ - temp_time)
            {
                left_hip_roll_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_total_ - t_rest_last_2 - t_double2_ - temp_time, t_start_ + t_total_ - t_rest_last_2, left_hip_roll_first, 0.0, 0.0, 0.0);
                left_hip_pitch_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_total_ - t_rest_last_2 - t_double2_ - temp_time, t_start_ + t_total_ - t_rest_last_2, left_hip_pitch_first, 0.0, 0.0, 0.0);
                left_ank_pitch_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_total_ - t_rest_last_2 - t_double2_ - temp_time, t_start_ + t_total_ - t_rest_last_2, left_ank_pitch_first, 0.0, 0.0, 0.0);
            }
            else
            {
                left_hip_roll_temp = 0.0;
                left_hip_pitch_temp = 0.0;
                left_ank_pitch_temp = 0.0;
            }
        }
        else if (foot_step_(current_step_num_, 6) == 0) // right support foot
        {
            if (walking_tick_mj < t_start_ + t_total_ - t_rest_last_2 - t_double2_ - temp_time)
            {
                right_hip_roll_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_rest_init_2 + t_double1_, t_start_ + t_rest_init_2 + t_double1_ + temp_time, 0.0, right_hip_roll_first, 0.0, 0.0);
                right_hip_pitch_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_rest_init_2 + t_double1_, t_start_ + t_rest_init_2 + t_double1_ + temp_time, 0.0, right_hip_pitch_first, 0.0, 0.0);
                right_ank_pitch_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_rest_init_2 + t_double1_, t_start_ + t_rest_init_2 + t_double1_ + temp_time, 0.0, right_ank_pitch_first, 0.0, 0.0);
            }
            else if (walking_tick_mj >= t_start_ + t_total_ - t_rest_last_2 - t_double2_ - temp_time)
            {
                right_hip_roll_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_total_ - t_rest_last_2 - t_double2_ - temp_time, t_start_ + t_total_ - t_rest_last_2, right_hip_roll_first, 0.0, 0.0, 0.0);
                right_hip_pitch_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_total_ - t_rest_last_2 - t_double2_ - temp_time, t_start_ + t_total_ - t_rest_last_2, right_hip_pitch_first, 0.0, 0.0, 0.0);
                right_ank_pitch_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_total_ - t_rest_last_2 - t_double2_ - temp_time, t_start_ + t_total_ - t_rest_last_2, right_ank_pitch_first, 0.0, 0.0, 0.0);
            }
            else
            {
                right_hip_roll_temp = 0.0;
                right_hip_pitch_temp = 0.0;
                right_ank_pitch_temp = 0.0;
            }
        }
    }
    else
    {
        if (foot_step_(current_step_num_, 6) == 1) //left support foot
        {
            if (walking_tick_mj < t_start_ + t_total_ - t_rest_last_ - t_double2_ - temp_time)
            {
                left_hip_roll_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_rest_init_ + t_double1_, t_start_ + t_rest_init_ + t_double1_ + temp_time, 0, left_hip_roll, 0.0, 0.0);
                left_hip_pitch_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_rest_init_ + t_double1_, t_start_ + t_rest_init_ + t_double1_ + temp_time, 0, left_hip_pitch, 0.0, 0.0);
                left_ank_pitch_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_rest_init_ + t_double1_, t_start_ + t_rest_init_ + t_double1_ + temp_time, 0, left_ank_pitch, 0.0, 0.0);
            }
            else if (walking_tick_mj >= t_start_ + t_total_ - t_rest_last_ - t_double2_ - temp_time)
            {
                left_hip_roll_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_total_ - t_rest_last_ - t_double2_ - temp_time, t_start_ + t_total_ - t_rest_last_, left_hip_roll, 0.0, 0.0, 0.0);
                left_hip_pitch_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_total_ - t_rest_last_ - t_double2_ - temp_time, t_start_ + t_total_ - t_rest_last_, left_hip_pitch, 0.0, 0.0, 0.0);
                left_ank_pitch_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_total_ - t_rest_last_ - t_double2_ - temp_time, t_start_ + t_total_ - t_rest_last_, left_ank_pitch, 0.0, 0.0, 0.0);
            }
            else
            {
                left_hip_roll_temp = 0;
                left_hip_pitch_temp = 0;
                left_ank_pitch_temp = 0;
            }
        }
        else if (foot_step_(current_step_num_, 6) == 0) // right support foot
        {
            if (walking_tick_mj < t_start_ + t_total_ - t_rest_last_ - t_double2_ - temp_time)
            {
                right_hip_roll_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_rest_init_ + t_double1_, t_start_ + t_rest_init_ + t_double1_ + temp_time, 0, right_hip_roll, 0.0, 0.0);
                right_hip_pitch_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_rest_init_ + t_double1_, t_start_ + t_rest_init_ + t_double1_ + temp_time, 0, right_hip_pitch, 0.0, 0.0);
                right_ank_pitch_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_rest_init_ + t_double1_, t_start_ + t_rest_init_ + t_double1_ + temp_time, 0, right_ank_pitch, 0.0, 0.0);
            }
            else if (walking_tick_mj >= t_start_ + t_total_ - t_rest_last_ - t_double2_ - temp_time)
            {
                right_hip_roll_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_total_ - t_rest_last_ - t_double2_ - temp_time, t_start_ + t_total_ - t_rest_last_, right_hip_roll, 0.0, 0.0, 0.0);
                right_hip_pitch_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_total_ - t_rest_last_ - t_double2_ - temp_time, t_start_ + t_total_ - t_rest_last_, right_hip_pitch, 0.0, 0.0, 0.0);
                right_ank_pitch_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_total_ - t_rest_last_ - t_double2_ - temp_time, t_start_ + t_total_ - t_rest_last_, right_ank_pitch, 0.0, 0.0, 0.0);
            }
            else
            {
                right_hip_roll_temp = 0;
                right_hip_pitch_temp = 0;
                right_ank_pitch_temp = 0.0;
            }
        }
    }

    ref_q_(1) = ref_q_(1) - left_hip_roll_temp;
    ref_q_(7) = ref_q_(7) + right_hip_roll_temp;
    ref_q_(2) = ref_q_(2) - left_hip_pitch_temp;
    ref_q_(8) = ref_q_(8) - right_hip_pitch_temp;
    ref_q_(4) = ref_q_(4) - left_ank_pitch_temp;
    ref_q_(10) = ref_q_(10) - right_ank_pitch_temp;
}

void CustomController::Compliant_control(Eigen::Vector12d desired_leg_q)
{
    Eigen::Vector12d current_u;
    double del_t = 0.0, Kp = 0.0;
    del_t = 1 / hz_;
    Kp = 100.0; // 실험

    if (walking_tick_mj == 0)
    {
        for (int i = 0; i < 12; i++)
        {
            DOB_IK_output_b_(i) = rd_.q_(i);
            DOB_IK_output_(i) = rd_.q_(i);
            current_u(i) = rd_.q_(i);
        }
    }

    if (walking_tick_mj > 0)
    {
        for (int i = 0; i < 12; i++)
        {
            current_u(i) = (rd_.q_(i) - (1 - Kp * del_t) * q_prev_MJ_(i)) / (Kp * del_t);
        }
    }

    Eigen::Vector12d d_hat;
    d_hat = current_u - DOB_IK_output_b_;

    if (walking_tick_mj == 0)
        d_hat_b = d_hat;

    d_hat = (2 * M_PI * 5.0 * del_t) / (1 + 2 * M_PI * 5.0 * del_t) * d_hat + 1 / (1 + 2 * M_PI * 5.0 * del_t) * d_hat_b;

    double default_gain = 0.0;
    double compliant_gain = 0.0;
    double compliant_tick = 0.1 * hz_;
    double gain_temp = 0.0;
    for (int i = 0; i < 12; i++)
    {
        if (i < 6)
        {
            gain_temp = default_gain;

            if (foot_step_(current_step_num_, 6) == 0)
            {
                if (walking_tick_mj < t_start_ + t_total_ - t_rest_last_ - t_double2_ - compliant_tick)
                {
                    gain_temp = default_gain;
                }
                else if (walking_tick_mj >= t_start_ + t_total_ - t_rest_last_ - t_double2_ - compliant_tick && walking_tick_mj < t_start_ + t_total_ - t_rest_last_ - t_double2_ + compliant_tick)
                {
                    gain_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_total_ - t_rest_last_ - t_double2_ - compliant_tick, t_start_ + t_total_ - t_rest_last_ - t_double2_ + compliant_tick, default_gain, compliant_gain, 0.0, 0.0);
                }
                else
                {
                    gain_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_total_ - t_rest_last_ - t_double2_ + compliant_tick, t_start_ + t_total_, compliant_gain, default_gain, 0.0, 0.0);
                }
            }
            else
            {
                gain_temp = default_gain;
            }

            DOB_IK_output_(i) = desired_leg_q(i) + gain_temp * d_hat(i);
        }
        else
        {
            gain_temp = default_gain;

            if (foot_step_(current_step_num_, 6) == 1) // 왼발 지지 상태
            {
                if (walking_tick_mj < t_start_ + t_total_ - t_rest_last_ - t_double2_ - compliant_tick)
                {
                    gain_temp = default_gain;
                }
                else if (walking_tick_mj >= t_start_ + t_total_ - t_rest_last_ - t_double2_ - compliant_tick && walking_tick_mj < t_start_ + t_total_ - t_rest_last_ - t_double2_ + compliant_tick)
                {
                    gain_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_total_ - t_rest_last_ - t_double2_ - compliant_tick, t_start_ + t_total_ - t_rest_last_ - t_double2_ + compliant_tick, default_gain, compliant_gain, 0.0, 0.0);
                }
                else
                {
                    gain_temp = DyrosMath::cubic(walking_tick_mj, t_start_ + t_total_ - t_rest_last_ - t_double2_ + compliant_tick, t_start_ + t_total_, compliant_gain, default_gain, 0.0, 0.0);
                }
            }
            else // 오른발 지지 상태
            {
                gain_temp = default_gain;
            }

            DOB_IK_output_(i) = desired_leg_q(i) + gain_temp * d_hat(i);
        }
    }

    d_hat_b = d_hat;
    DOB_IK_output_b_ = DOB_IK_output_;
}

void CustomController::CP_compen_MJ()
{
    double F_R = 0, F_L = 0;

    // Tau_R.setZero(); Tau_L.setZero();

    Tau_CP.setZero();

    alpha = (com_float_current_(1) - rfoot_float_current_.translation()(1)) / (lfoot_float_current_.translation()(1) - rfoot_float_current_.translation()(1));

    if (alpha > 1)
    {
        alpha = 1;
    }
    else if (alpha < 0)
    {
        alpha = 0;
    }
   
    F_R = (1 - alpha) * rd_.link_[COM_id].mass * GRAVITY;
    F_L = alpha * rd_.link_[COM_id].mass * GRAVITY;
   
    Tau_CP(4) = F_L * del_zmp(0);  // L pitch
    Tau_CP(10) = F_R * del_zmp(0); // R pitch

    Tau_CP(5) = -F_L * del_zmp(1);  // L roll
    Tau_CP(11) = -F_R * del_zmp(1); // R roll
}

void CustomController::CP_compen_MJ_FT()
{ // 기존 알고리즘에서 바꾼거 : 0. previewcontroller에서 ZMP_Y_REF 변수 추가 1. zmp offset 2. getrobotstate에서 LPF 3. supportToFloatPattern 함수 4. Tau_CP -> 0  5. getfoottrajectory에서 발의 Euler angle
    double F_R = 0, F_L = 0;
    double Tau_all_y = 0, Tau_R_y = 0, Tau_L_y = 0;
    double Tau_all_x = 0, Tau_R_x = 0, Tau_L_x = 0;
    double zmp_offset = 0;
    double alpha_new = 0;

    zmp_offset = 0.02; // zmp_offset 함수 참고
   
    // Preview를 이용한 COM 생성시 ZMP offset을 x cm 안쪽으로 넣었지만, alpha 계산은 x cm 넣으면 안되기 때문에 조정해주는 코드
    // 어떻게 보면 COM, CP 궤적은 ZMP offset이 반영되었고, CP 제어기는 반영안시킨게 안맞는거 같기도함
    if(mpc_cycle < 0)
    {
        if (walking_tick_mj > t_temp_)
        {
            if (walking_tick_mj < t_start_ + t_rest_init_ + t_double1_)
            {
                if (foot_step_(current_step_num_, 6) == 1)
                {
                    ZMP_Y_REF_alpha = ZMP_Y_REF_1 + zmp_offset * (walking_tick_mj - (t_start_ + t_rest_init_ + t_double1_) + t_rest_init_ + t_double1_) / (t_rest_init_ + t_double1_);
                }
                else
                {
                    ZMP_Y_REF_alpha = ZMP_Y_REF_1 - zmp_offset * (walking_tick_mj - (t_start_ + t_rest_init_ + t_double1_) + t_rest_init_ + t_double1_) / (t_rest_init_ + t_double1_);
                }
            }
            else if (walking_tick_mj >= t_start_ + t_rest_init_ + t_double1_ && walking_tick_mj < t_start_ + t_total_ - t_double2_ - t_rest_last_)
            {
                if (foot_step_(current_step_num_, 6) == 1)
                {
                    ZMP_Y_REF_alpha = ZMP_Y_REF_1 + zmp_offset;
                }
                else
                {
                    ZMP_Y_REF_alpha = ZMP_Y_REF_1 - zmp_offset;
                }
            }
            else if (walking_tick_mj >= t_start_ + t_total_ - t_double2_ - t_rest_last_ && walking_tick_mj < t_start_ + t_total_)
            {
                if (foot_step_(current_step_num_, 6) == 1)
                {
                    ZMP_Y_REF_alpha = ZMP_Y_REF_1 + zmp_offset - zmp_offset * (walking_tick_mj - (t_start_ + t_total_ - t_rest_last_ - t_double2_)) / (t_rest_last_ + t_double2_);
                }
                else
                {
                    ZMP_Y_REF_alpha = ZMP_Y_REF_1 - zmp_offset + zmp_offset * (walking_tick_mj - (t_start_ + t_total_ - t_rest_last_ - t_double2_)) / (t_rest_last_ + t_double2_);
                }
            }
            else
            {
                ZMP_Y_REF_alpha = ZMP_Y_REF_1;
            }
        }
        else
        {
            ZMP_Y_REF_alpha = ZMP_Y_REF_1;
        }
        double A = 0, B = 0, d = 0, X1 = 0, Y1 = 0, e_2 = 0, L = 0, l = 0;
        A = (lfoot_support_current_.translation()(0) - rfoot_support_current_.translation()(0));
        B = -(lfoot_support_current_.translation()(1) - rfoot_support_current_.translation()(1));
        X1 = ZMP_Y_REF_alpha + 0 * del_zmp(1) - rfoot_support_current_.translation()(1);
        Y1 = ZMP_X_REF_1 + 0 * del_zmp(0) - rfoot_support_current_.translation()(0);
        L = sqrt(A * A + B * B);
        d = abs(A * X1 + B * Y1) / L;
        e_2 = X1 * X1 + Y1 * Y1;
        l = sqrt(e_2 - d * d);
        alpha_new = l / L;
        alpha = (ZMP_Y_REF_alpha + 0 * del_zmp(1) - rfoot_support_current_.translation()(1)) / (lfoot_support_current_.translation()(1) - rfoot_support_current_.translation()(1));
        //cout << alpha << "," << ZMP_Y_REF << "," << rfoot_support_current_.translation()(1) << "," << lfoot_support_current_.translation()(1) - rfoot_support_current_.translation()(1) << endl;
        // 로봇에서 구현할때 alpha가 0~1로 나오는지 확인, ZMP offset 0으로 해야됨.
        if (alpha > 1)
        {
            alpha = 1;
        } // 왼발 지지때 alpha = 1
        else if (alpha < 0)
        {
            alpha = 0;
        }
        if (alpha_new > 1)
        {
            alpha_new = 1;
        } // 왼발 지지때 alpha = 1
        else if (alpha_new < 0)
        {
            alpha_new = 0;
        }
    }
    else
    {
        comGainTrajectory();
    }
   
    F_R = -(1 - alpha) * rd_.link_[COM_id].mass * GRAVITY;
    F_L = -alpha * rd_.link_[COM_id].mass * GRAVITY; // alpha가 0~1이 아니면 desired force가 로봇 무게보다 계속 작게나와서 지면 반발력을 줄이기위해 다리길이를 줄임.
   
    com_alpha = alpha;
   
    if (walking_tick_mj == 0)
    {
        F_F_input = 0.0;
        F_T_L_x_input = 0.0;
        F_T_R_x_input = 0.0;
        F_T_L_y_input = 0.0;
        F_T_R_y_input = 0.0;
    }
    //////////// Force
    if(mpc_cycle >= 0)
    {
        F_F_input_dot = 0.0002 * ((l_ft_(2) - r_ft_(2)) - (F_L - F_R)) - 3.0 * F_F_input; // F_F_input이 크면 다리를 원래대로 빨리줄인다. 이정도 게인 적당한듯0.001/0.00001 // SSP, DSP 게인값 바꿔야?
        F_F_input = F_F_input + F_F_input_dot * del_t;
    }
    else
    {//0.0004
        F_F_input_dot = 0.0002 * ((l_ft_(2) - r_ft_(2)) - (F_L - F_R)) - 5.0 * F_F_input; // F_F_input이 크면 다리를 원래대로 빨리줄인다. 이정도 게인 적당한듯0.001/0.00001 // SSP, DSP 게인값 바꿔야?
        F_F_input = F_F_input + F_F_input_dot * del_t;
    }
    if (F_F_input >= 0.02)
    {
        F_F_input = 0.02;
    }
    else if (F_F_input <= -0.02)
    {
        F_F_input = -0.02;
    }

    //F_L = -1 * F_L;
    //F_R = -1 * F_R;
   
    //////////// Torque
    // X,Y 축을 X,Y 방향으로 헷갈렸었고, 위치 명령을 발목 IK각도에 바로 넣었었음.

    if(mpc_cycle <= 0)
        ZMP_Y_REF = ZMP_Y_REF_alpha;
   
    Tau_all_x = -((rfoot_support_current_.translation()(1) - (ZMP_Y_REF + del_zmp(1))) * F_R + (lfoot_support_current_.translation()(1) - (ZMP_Y_REF + del_zmp(1))) * F_L);
    Tau_all_y = -((rfoot_support_current_.translation()(0) - (ZMP_X_REF + del_zmp(0))) * F_R + (lfoot_support_current_.translation()(0) - (ZMP_X_REF + del_zmp(0))) * F_L);

    //Tau_all_y = -((rfoot_support_current_.translation()(0) - (ZMP_X_REF + del_zmp(0))) * F_R + (lfoot_support_current_.translation()(0) - (ZMP_X_REF + del_zmp(0))) * F_L);
   
   
    if (Tau_all_x > 100)
    {
        Tau_all_x = 100;
    }
    else if (Tau_all_x < -100)
    {
        Tau_all_x = -100;
    }

    if (Tau_all_y > 100)
    {
        Tau_all_y = 100;
    }
    else if (Tau_all_y < -100)
    {
        Tau_all_y = -100;
    }

    Tau_R_x = (1-alpha) * Tau_all_x;
    Tau_L_x = (alpha)*Tau_all_x;

    Tau_R_y = -(1-alpha) * Tau_all_y;
    Tau_L_y = -alpha * Tau_all_y;
   

    double Kr_roll = 0.0, Kl_roll = 0.0;
    double Kr_pitch = 0.0, Kl_pitch = 0.0;

    if (walking_tick_mj < t_start_ + t_rest_init_ + t_double1_)
    {
        Kr_roll = 30.0;
        Kl_roll = 30.0;
        Kr_pitch = 30.0;
        Kl_pitch = 30.0;
    }
    else if (walking_tick_mj >= t_start_ + t_rest_init_ + t_double1_ && walking_tick_mj < t_start_ + t_total_ - t_double2_ - t_rest_last_)
    {
        if (alpha == 1) // 왼발 지지
        {
            Kl_roll = 30.0;
            Kr_roll = 30.0;
            Kl_pitch = 30.0;
            Kr_pitch = 30.0;
        }
        if (alpha == 0) // 오른발 지지
        {
            Kl_roll = 30.0;
            Kr_roll = 30.0;
            Kl_pitch = 30.0;
            Kr_pitch = 30.0;
        }
    }
    else
    {
        Kr_roll = 30.0;
        Kl_roll = 30.0;
        Kr_pitch = 30.0;
        Kl_pitch = 30.0;
    }

    //Roll 방향 -0.3,50 -> High performance , -0.1, 50 평지 보행 적당
    if(torque_control == false)
    {
        F_T_L_x_input_dot = -0.07 * (Tau_L_x - l_ft_LPF(3)) - Kl_roll * F_T_L_x_input;
        F_T_L_x_input = F_T_L_x_input + F_T_L_x_input_dot * del_t;
        //F_T_L_x_input = 0;
        F_T_R_x_input_dot = -0.07 * (Tau_R_x - r_ft_LPF(3)) - Kr_roll * F_T_R_x_input;
        F_T_R_x_input = F_T_R_x_input + F_T_R_x_input_dot * del_t;
        //F_T_R_x_input = 0;

        //Pitch 방향
        F_T_L_y_input_dot = 0.07 * (Tau_L_y - l_ft_LPF(4)) - Kl_pitch * F_T_L_y_input;
        F_T_L_y_input = F_T_L_y_input + F_T_L_y_input_dot * del_t;
        //F_T_L_y_input = 0;
        F_T_R_y_input_dot = 0.07 * (Tau_R_y - r_ft_LPF(4)) - Kr_pitch * F_T_R_y_input;
        F_T_R_y_input = F_T_R_y_input + F_T_R_y_input_dot * del_t;
    }
    else
    {   //0.1
        if(mpc_cycle > 0)
        {
            F_T_L_x_input_dot = -0.1 * (Tau_L_x - l_ft_LPF(3)) - Kl_roll * F_T_L_x_input;
            F_T_L_x_input = F_T_L_x_input + F_T_L_x_input_dot * del_t;
            //F_T_L_x_input = 0;
            F_T_R_x_input_dot = -0.1 * (Tau_R_x - r_ft_LPF(3)) - Kr_roll * F_T_R_x_input;
            F_T_R_x_input = F_T_R_x_input + F_T_R_x_input_dot * del_t;
            //F_T_R_x_input = 0;

            //Pitch 방향
            F_T_L_y_input_dot = 0.1 * (Tau_L_y - l_ft_LPF(4)) - Kl_pitch * F_T_L_y_input;
            F_T_L_y_input = F_T_L_y_input + F_T_L_y_input_dot * del_t;
            //F_T_L_y_input = 0;
            F_T_R_y_input_dot = 0.1 * (Tau_R_y - r_ft_LPF(4)) - Kr_pitch * F_T_R_y_input;
            F_T_R_y_input = F_T_R_y_input + F_T_R_y_input_dot * del_t;
        }
        else
        {
            F_T_L_x_input_dot = -0.1 * (Tau_L_x - l_ft_LPF(3)) - Kl_roll * F_T_L_x_input;
            F_T_L_x_input = F_T_L_x_input + F_T_L_x_input_dot * del_t;
            //F_T_L_x_input = 0;
            F_T_R_x_input_dot = -0.1 * (Tau_R_x - r_ft_LPF(3)) - Kr_roll * F_T_R_x_input;
            F_T_R_x_input = F_T_R_x_input + F_T_R_x_input_dot * del_t;
            //F_T_R_x_input = 0;

            //Pitch 방향
            F_T_L_y_input_dot = 0.07 * (Tau_L_y - l_ft_LPF(4)) - Kl_pitch * F_T_L_y_input;
            F_T_L_y_input = F_T_L_y_input + F_T_L_y_input_dot * del_t;
            //F_T_L_y_input = 0;
            F_T_R_y_input_dot = 0.07 * (Tau_R_y - r_ft_LPF(4)) - Kr_pitch * F_T_R_y_input;
            F_T_R_y_input = F_T_R_y_input + F_T_R_y_input_dot * del_t;
        }
    }

    if (F_T_L_x_input >= 0.2) // 8.5 deg limit
    {
        F_T_L_x_input = 0.2;
    }
    else if (F_T_L_x_input < -0.2)
    {
        F_T_L_x_input = -0.2;
    }

    if (F_T_R_x_input >= 0.2) // 8.5 deg limit
    {
        F_T_R_x_input = 0.2;
    }
    else if (F_T_R_x_input < -0.2)
    {
        F_T_R_x_input = -0.2;
    }

    if (F_T_L_y_input >= 0.2) // 8.5 deg limit
    {
        F_T_L_y_input = 0.2;
    }
    else if (F_T_L_y_input < -0.2)
    {
        F_T_L_y_input = -0.2;
    }

    if (F_T_R_y_input >= 0.2) // 8.5 deg limit
    {
        F_T_R_y_input = 0.2;
    }
    else if (F_T_R_y_input < -0.2)
    {
        F_T_R_y_input = -0.2;
    }  
}

void CustomController::computePlanner()
{
}

void CustomController::copyRobotData(RobotData &rd_l)
{
    std::memcpy(&rd_cc_, &rd_l, sizeof(RobotData));
}

void CustomController::CentroidalMomentCalculator()
{
    if(mpc_cycle > 0)
    {
        del_zmp(0) = 1.5 * (cp_measured_(0) - cp_desired_(0));//1.6, 1.6
        del_zmp(1) = 1.5 * (cp_measured_(1) - cp_desired_(1));
    }
    else
    {
        del_zmp(0) = 2.2 * (cp_measured_(0) - cp_desired_(0));//1.6, 1.6
        del_zmp(1) = 2.2 * (cp_measured_(1) - cp_desired_(1));
    }
   
    CLIPM_ZMP_compen_MJ(del_zmp(0), del_zmp(1)); // 원래 previewcontroller 자리에 있던 함수.
}


void CustomController::momentumControl(RobotData &Robot, Eigen::Vector3d comd,  Eigen::Vector2d ang_, Eigen::Vector3d rfootd, Eigen::Vector3d lfootd, Eigen::Vector2d upperd, Eigen::Vector3d rfootori, Eigen::Vector3d lfootori)
{
    if(as == 0)
    {
        H2(3,3) = 100.0;
        H2(4,4) = 100.0;
        H2(5,5) = 100.0;
   
        J2.block(0,0,3,18) = comj.block(0,0,3,18);//InitRPYM7.inverse() *
        J2.block(3,0,6,18) = J_RFF.block(0,0,6,18);//J_RFF RFj comj
        J2.block(9,0,6,18) = J_LFF.block(0,0,6,18);//J_LFF LFj
   
        X2.segment<3>(0) = comd;
        X2.segment<3>(3) = rfootd;
        X2.segment<3>(6) = rfootori;
        X2.segment<3>(9) = lfootd;
        X2.segment<3>(12) = lfootori;

        A2 = J2;
        lbA2 = X2;
        ubA2 = X2;

        qp_momentum_control.UpdateMinProblem(H2, g2);
        qp_momentum_control.UpdateSubjectToAx(A2, lbA2, ubA2);
        qp_momentum_control.UpdateSubjectToX(lb2, ub2);
   
        qp_solved = qp_momentum_control.SolveQPoases(100, qp_result1);
   
        if(qp_solved == true)
            q_dm = qp_result1;
        else
            q_dm.setZero();
    }
    else
    {
        if(mpc_cycle > 1 || (mpc_cycle == 1 && walking_tick >= 1))
        {
            temp_mom = 1;
            H2(3,3) = 5.0;
            H2(4,4) = 5.0;
            H2(5,5) = 1.0;
   
            MOMX = Hg_slow_.block(3,19,2,2) * upperd/2000;
            COMX = comj.block(0,19,3,2) * upperd/2000;


            J2.block(0,0,3,18) = comj.block(0,0,3,18);//InitRPYM7.inverse() *
            J2.block(3,0,6,18) = J_RFF.block(0,0,6,18);//J_RFF RFj comj
            J2.block(9,0,6,18) = J_LFF.block(0,0,6,18);//J_LFF LFj
            J2.block(15,0,2,18) = Hg_slow_.block(3,0,2,18);//InitRPYM7.inverse() *
           
            X2.segment<3>(0) = comd - COMX;
            X2.segment<3>(3) = rfootd;
            X2.segment<3>(6) = rfootori;
            X2.segment<3>(9) = lfootd;
            X2.segment<3>(12) = lfootori;
           
            X2(15) = ang_(0) - MOMX(0);
            X2(16) = ang_(1) - MOMX(1);

            A2 = J2;
            lbA2 = X2;
            ubA2 = X2;

            lbA2(15) = lbA2(15)-0.1;
            ubA2(15) = ubA2(15)+0.1;
            lbA2(16) = lbA2(16)-0.1;
            ubA2(16) = ubA2(16)+0.1;

            qp_momentum_control.UpdateMinProblem(H2, g2);
            qp_momentum_control.UpdateSubjectToAx(A2, lbA2, ubA2);
            qp_momentum_control.UpdateSubjectToX(lb2, ub2);
   
            qp_solved = qp_momentum_control.SolveQPoases(100, qp_result1);
   
            if(qp_solved == true)
                q_dm = qp_result1;
            else
                q_dm.setZero();
            /*
            H2(3,3) = 100.0;
            H2(4,4) = 100.0;
            H2(5,5) = 100.0;
   
            J2.block(0,0,3,18) = comj.block(0,0,3,18);//InitRPYM7.inverse() *
            J2.block(3,0,6,18) = J_RFF.block(0,0,6,18);//J_RFF RFj comj
            J2.block(9,0,6,18) = J_LFF.block(0,0,6,18);//J_LFF LFj
   
            X2.segment<3>(0) = comd;
            X2.segment<3>(3) = rfootd;
            X2.segment<3>(6) = rfootori;
            X2.segment<3>(9) = lfootd;
            X2.segment<3>(12) = lfootori;

            A2 = J2;
            lbA2 = X2;
            ubA2 = X2;
   
            qp_momentum_control.UpdateMinProblem(H2, g2);
            qp_momentum_control.UpdateSubjectToAx(A2, lbA2, ubA2);
            qp_momentum_control.UpdateSubjectToX(lb2, ub2);
   
            qp_solved = qp_momentum_control.SolveQPoases(100, qp_result1);
   
            if(qp_solved == true)
                q_dm = qp_result1;
            else
                q_dm.setZero();
            */
        }
        else
        {
            temp_mom = 2;
            H2(3,3) = 100.0;
            H2(4,4) = 100.0;
            H2(5,5) = 100.0;
   
            J2.block(0,0,3,18) = comj.block(0,0,3,18);//InitRPYM7.inverse() *
            J2.block(3,0,6,18) = J_RFF.block(0,0,6,18);//J_RFF RFj comj
            J2.block(9,0,6,18) = J_LFF.block(0,0,6,18);//J_LFF LFj
   
            X2.segment<3>(0) = comd;
            X2.segment<3>(3) = rfootd;
            X2.segment<3>(6) = rfootori;
            X2.segment<3>(9) = lfootd;
            X2.segment<3>(12) = lfootori;

            A2 = J2;
            lbA2 = X2;
            ubA2 = X2;

            qp_momentum_control.UpdateMinProblem(H2, g2);
            qp_momentum_control.UpdateSubjectToAx(A2, lbA2, ubA2);
            qp_momentum_control.UpdateSubjectToX(lb2, ub2);
   
            qp_solved = qp_momentum_control.SolveQPoases(100, qp_result1);
   
            if(qp_solved == true)
                q_dm = qp_result1;
            else
                q_dm.setZero();
        }
    }
}

void CustomController::getMPCTrajectory()
{
    if(contactMode == 1)
    {
        virtual_temp(2) = -((rd_.link_[Right_Foot].xipos(2)  + rd_.link_[Left_Foot].xipos(2))/2 - 0.0724);
        if(mpc_cycle <= 69)
        {
            if(mpc_cycle >= 49 && (walking_tick == 0 || walking_tick == 1))
            {
                virtual_temp(0) = -((rfoot_float_current_.translation()(0) + lfoot_float_current_.translation()(0))/2 - 0.064964);
                virtual_temp(1) = -((rfoot_float_current_.translation()(1) + lfoot_float_current_.translation()(1))/2);
            }
        }
        else if(mpc_cycle >= 99 && mpc_cycle <= 149)//temp
        {
            if((walking_tick == 0 || walking_tick == 1))
            {
                if(mpc_cycle <= 148)
                {
                    virtual_temp(0) = -((rfoot_float_current_.translation()(0) + lfoot_float_current_.translation()(0))/2 - 0.064964);
                }
                else
                {
                    virtual_temp(0) = -((rfoot_float_current_.translation()(0) + lfoot_float_current_.translation()(0))/2 - 0.064964);
                }
            }
            if(mpc_cycle == 100)
            {
                virtual_temp(1) = -((rfoot_float_current_.translation()(1) + lfoot_float_current_.translation()(1))/2);
            }
        }
        else
        {
            if(mpc_cycle_int % 2 == 0)
            {
                if((walking_tick == 0 || walking_tick == 1))
                {
                    virtual_temp(0) = -((rfoot_float_current_.translation()(0) + lfoot_float_current_.translation()(0))/2 - 0.064964);
                }
            }
            else
            {
                if((walking_tick == 0 || walking_tick == 1))
                {
                    virtual_temp(0) = -((rfoot_float_current_.translation()(0) + lfoot_float_current_.translation()(0))/2 - 0.064964);
                }
            }
        }
    }    
    else if(contactMode == 2)
    {
        virtual_temp(2) = -(rd_.link_[Left_Foot].xipos(2) - 0.0724);
        if(walking_tick == 0)
        {
            virtual_temp(0) =  -(lfoot_float_current_.translation()(0) - 1.151300000000000090e-01);
            virtual_temp(1) =  -(lfoot_float_current_.translation()(1) - 0.1025);
        }
        if((mpc_cycle == 98 || mpc_cycle == 99) && (walking_tick == 0))
        {
            virtual_temp(0) = -((rfoot_float_current_.translation()(0) + lfoot_float_current_.translation()(0))/2 - 0.064964);
            virtual_temp(1) = -((rfoot_float_current_.translation()(1) + lfoot_float_current_.translation()(1))/2);
        }
        else if((mpc_cycle_int1 == 48 || mpc_cycle_int1 == 49) && (walking_tick == 0))
        {
            virtual_temp(0) = -((rfoot_float_current_.translation()(0) + lfoot_float_current_.translation()(0))/2 - 0.064964);
            virtual_temp(1) = -((rfoot_float_current_.translation()(1) + lfoot_float_current_.translation()(1))/2);
        }
    }
    else
    {  
        virtual_temp(2) = -(rd_.link_[Right_Foot].xipos(2) - 0.0724);
        if(walking_tick == 0)
        {
            if(mpc_cycle < 100)
            {
                virtual_temp(0) = -(rfoot_float_current_.translation()(0) - 6.479871019999999704e-02);
            }
            else
            {
                virtual_temp(0) = -(rfoot_float_current_.translation()(0) - 1.151300000000000090e-01);  
            }
            virtual_temp(1) = -(rfoot_float_current_.translation()(1) + 0.1025);
        }

        if((mpc_cycle == 48 || mpc_cycle == 49) && (walking_tick == 0))
        {
            virtual_temp(0) = -((rfoot_float_current_.translation()(0) + lfoot_float_current_.translation()(0))/2 - 0.064964);
            virtual_temp(1) = -((rfoot_float_current_.translation()(1) + lfoot_float_current_.translation()(1))/2);
        }
        else if((mpc_cycle_int1 == 48 || mpc_cycle_int1 == 49) && (walking_tick == 0))
        {
            virtual_temp(0) = -((rfoot_float_current_.translation()(0) + lfoot_float_current_.translation()(0))/2 - 0.064964);
            virtual_temp(1) = -((rfoot_float_current_.translation()(1) + lfoot_float_current_.translation()(1))/2);
        }
    }

    if(walking_tick >= 0)
    {  
        state_init_[0] = rd_.q_virtual_[0] +virtual_temp(0);
        state_init_[1] = rd_.q_virtual_[1] +virtual_temp(1);
        state_init_[2] = rd_.q_virtual_[2] +virtual_temp(2);

        for (int i = 3; i < 6; i ++)
            state_init_[i] = rd_.q_virtual_[i];
   
        state_init_[6] = rd_.q_virtual_[39];
   
        for (int i = 6; i < 18; i ++)
            state_init_[i+1] = rd_.q_virtual_[i];
        //state_init_[20] = rd_.q_virtual_[19];
        //state_init_[21] = rd_.q_virtual_[20];
        state_init_[19] = rd_.q_virtual_[19];
        state_init_[20] = rd_.q_virtual_[20];

        for (int i = 0; i < 18; i ++)
            state_init_[i+21] = rd_.q_dot_virtual_[i];
   
        state_init_[39] =  rd_.q_dot_virtual_[19];
        state_init_[40] =  rd_.q_dot_virtual_[20];
   
        state_init_[41] = rd_.link_[COM_id].xpos(0) + virtual_temp(0);//com_float_current_(0)+virtual_temp(0);//+virtual_temp_sup(0);
        state_init_[45] = rd_.link_[COM_id].xpos(1) + virtual_temp(1);//com_float_current_(1)+virtual_temp(1);//+virtual_temp_sup(1);
        state_init_[42] = rd_.link_[COM_id].v(0);//com_float_current_dot(0);
        state_init_[46] = rd_.link_[COM_id].v(1);//com_float_current_dot(1);
   
        state_init_[43] = ZMP_float(0) + virtual_temp(0);//+virtual_temp(0);//zmp_measured_mj_(0);//+virtual_temp_sup(0);
        state_init_[47] = ZMP_float(1) + virtual_temp(1);//+virtual_temp(1);//zmp_measured_mj_(1);//+virtual_temp_sup(1);
       
        state_init_[44] = model_data_cen.hg.angular()(1);
        state_init_[48] = model_data_cen.hg.angular()(0);//model_data1.hg.angular()[0];

        state_init_[49] = rd_.link_[COM_id].xpos(2) + virtual_temp(2);//com_support_current_(2);//model_data1.hg.angular()[0];
       
        if(atb_state_update_ == false)
        {
            atb_state_update_ = true;
            state_init_slow = state_init_;
            atb_state_update_ = false;
        }
    }
   
    if(statemachine_ == 2 && walking_tick > 3)
        mpc_start_init_ = 3;

    if(statemachine_ == 1 || statemachine_ == 2 && mpc_cycle < controlwalk_time)
    {
        if(walking_tick == 0)
        {
            qd_pinocchio_.setZero();
        }  
        if((statemachine_ == 1 && walking_tick_stop == true && walking_tick == 0 && walking_tick_stop == true))
        {  
            mpc_start_init_ = 2;
            if (walking_tick == 0)
            {
                q_pinocchio_desired.head(19) = q_pinocchio.head(19);
                q_pinocchio_desired1.head(19) = q_pinocchio.head(19);
           
                walking_tick_stop = false;
           
                if (contactMode == 1 && mpc_cycle <= 1)
                {
                    foot_temp(1) = rfoot_float_current_.translation()(2);
                    foot_temp(1) = lfoot_float_current_.translation()(2);
                }

                if(upper_on == true)
                {
                    std::cout << "mpc_cycle " << " : " << mpc_cycle << " " << walking_tick <<  " " << rfoot_mpc(2) << " " << lfoot_mpc(2) << " " << contactMode << std::endl;

                    qd_pinocchio.setZero();
                   
                    thread2_lock.lock();
                    desired_val_slow = desired_val_mu;
                    thread2_lock.unlock();

                    if(desired_val_slow[19] < -0.4000 || rd_.q_(13) < -0.4000)
                    {  
                        std::cout << "Pitch over" << std::endl;
                        if(desired_val_slow[39] < 0.0)
                        {
                            qd_pinocchio(19) = 0.0;
                            upperd[0] = 0.0;
                        }
                        else
                        {
                            qd_pinocchio(19) = desired_val_slow[39];
                            upperd[0] = desired_val_slow[39];
                        }
                    }
                    else if(desired_val_slow[19] > 0.4000 || rd_.q_(13) > 0.4000)
                    {  
                        std::cout << "Pitch over" << std::endl;
                        if(desired_val_slow[39] > 0.0)
                        {
                            qd_pinocchio(19) = 0.0;
                            upperd[0] = 0.0;
                            //qd_pinocchio(19) = desired_val_slow[39];
                            //upperd[0] = desired_val_slow[39];
                        }
                        else
                        {
                            qd_pinocchio(19) = desired_val_slow[39];
                            upperd[0] = desired_val_slow[39];
                        }
                    }
                    else
                    {  
                        qd_pinocchio(19) = desired_val_slow[39];
                        upperd[0] = desired_val_slow[39];
                    }

                    if(desired_val_slow[20] < -0.4000 || rd_.q_(14) < -0.4000)
                    {
                        std::cout << "Roll over" << std::endl;
                        if(desired_val_slow[40] < 0.0)
                        {
                            qd_pinocchio(20) = 0.0;
                            upperd[1] = 0.0;
                        }
                        else
                        {
                            qd_pinocchio(20) = desired_val_slow[40];
                            upperd[1] = desired_val_slow[40];
                        }
                    }
                    else if(desired_val_slow[20] > 0.4000 || rd_.q_(14) > 0.4000)
                    {
                        std::cout << "Roll over" << std::endl;
                        if(desired_val_slow[40] > 0.0)
                        {
                            qd_pinocchio(20) = 0.0;
                            upperd[1] = 0.0;
                            //qd_pinocchio(20) = desired_val_slow[40];
                            //upperd[1] = desired_val_slow[40];
                        }
                        else
                        {
                            qd_pinocchio(20) = desired_val_slow[40];
                            upperd[1] = desired_val_slow[40];
                        }  
                    }
                    else
                    {
                        qd_pinocchio(20) = desired_val_slow[40];
                        upperd[1] = desired_val_slow[40];
                    }
                }
               
                std::cout << "JOint " <<mpc_cycle << " "<< rd_.q_(13) << " " << rd_.q_(14) << "  " << qd_pinocchio(19) << " " << qd_pinocchio(20) << " " << desired_val_slow[39] << " " << desired_val_slow[40]<< " " << q_pinocchio_desired(20) << " " << q_pinocchio_desired(21) <<std::endl;//DyrosMath::rot2Euler(rd_.link_[Pelvis].rotm)(0) << " " << DyrosMath::rot2Euler(rd_.link_[Pelvis].rotm)(1) << " " << q_pinocchio_desired(20) << " " << q_pinocchio_desired(21) << std::endl;
                   
               
                if(mpc_cycle == 0)
                {
                    comprev.setZero();
                }
                else
                {
                    comprev = comd;
                }

                comd[0] = desired_val_slow[42] + 0.0;
                comd[1] = desired_val_slow[46] + 0.0;
                comd[2] = 0.0;

                //angm[0] = desired_val_slow[48];
                //angm[1] = desired_val_slow[44];

                zmp_temp1(0) = ZMP_X_REF;
                zmp_temp1(1) = ZMP_Y_REF;
                zmp_temp1(2) = 0.0;

                zmp_temp1 = DyrosMath::multiplyIsometry3dVector3d(supportfoot_float_current_yaw_only, zmp_temp1);


                if(mpc_cycle == 0)
                {
                    ZMPx_prev = zmp_temp1(0)-virtual_temp1(0);//ZMP_float(0);
                    ZMPy_prev = zmp_temp1(1)-virtual_temp1(1);//ZMP_float(1);

                    angm_prev(0) = model_data_cen.hg.angular()(0);
                    angm_prev(1) = model_data_cen.hg.angular()(1);
                    com_mpcy_prev = com_float_current_(1);
                    com_mpcx_prev = com_float_current_(0);
                }
                else
                {
                    if(mpc_cycle == 1)
                    {
                        angm_prev(0) = model_data_cen.hg.angular()(0);
                        angm_prev(1) = model_data_cen.hg.angular()(1);
                    }
                   
                    ZMPx_prev = zmp_mpcx;
                    ZMPy_prev = zmp_mpcy;
                    angm_prev = angm;
                    //angm_prev(0) = model_data_cen.hg.angular()(0);
                    //angm_prev(1) = model_data_cen.hg.angular()(1);
                   
                    com_mpcy_prev = com_mpc1[1];
                    com_mpcx_prev = com_mpc1[0];
                }

                zmp_mpcx = desired_val_slow[43]-virtual_temp1(0);
                zmp_mpcy = desired_val_slow[47]-virtual_temp1(1);

                double lx, ly, mu;
                ly = 0.048;
                lx = 0.11;

                if(contactMode == 1)
                {
                    if(lfoot_sx_float >  rfoot_sx_float)
                    {
                        if(zmp_mpcx /* + virtual_temp1(0)*/ > lfoot_sx_float + lx)
                            zmp_mpcx  = lfoot_sx_float + lx /* - virtual_temp1(0)*/;
                        else if(zmp_mpcx /* + virtual_temp1(0)*/ < rfoot_sx_float - lx)
                            zmp_mpcx  = rfoot_sx_float - lx /* - virtual_temp1(0)*/;
                    }
                    else if(lfoot_sx_float <  rfoot_sx_float)
                    {
                        if(zmp_mpcx /* + virtual_temp1(0)*/  > rfoot_sx_float + lx)
                            zmp_mpcx  = rfoot_sx_float + lx /* - virtual_temp1(0)*/;
                        else if(zmp_mpcx /* + virtual_temp1(0)*/  < lfoot_sx_float - lx)
                            zmp_mpcx  = lfoot_sx_float - lx /* - virtual_temp1(0)*/;
                    }
                    else
                    {
                        if(zmp_mpcx /* + virtual_temp1(0)*/  > rfoot_sx_float + lx)
                            zmp_mpcx  = rfoot_sx_float + lx /* - virtual_temp1(0)*/;
                        else if(zmp_mpcx /* + virtual_temp1(0)*/  < lfoot_sx_float - lx)
                            zmp_mpcx  = lfoot_sx_float - lx /* - virtual_temp1(0)*/;
                    }

                    if(zmp_mpcy /* + virtual_temp1(1)*/ > lfoot_sy_float + ly)
                        zmp_mpcy = lfoot_sy_float + ly /*- virtual_temp1(1)*/;
                   
                    if(zmp_mpcy /* + virtual_temp1(1)*/ < rfoot_sy_float - ly)
                        zmp_mpcy = rfoot_sy_float - ly /*- virtual_temp1(1)*/;
                }
                else if(contactMode == 2)
                {
                    if(zmp_mpcx /* + virtual_temp1(0)*/  > lfoot_sx_float + lx)
                        zmp_mpcx  = lfoot_sx_float + lx /* - virtual_temp1(0)*/ ;
                    else if(zmp_mpcx /* + virtual_temp1(0)*/  < lfoot_sx_float - lx)
                        zmp_mpcx  = lfoot_sx_float - lx /* - virtual_temp1(0)*/ ;
                   
                    if(zmp_mpcy /* + virtual_temp1(1)*/ > lfoot_sy_float + ly)
                        zmp_mpcy = lfoot_sy_float + ly /*- virtual_temp1(1)*/;
                    else if(zmp_mpcy /* + virtual_temp1(1)*/ < lfoot_sy_float - ly)
                        zmp_mpcy = lfoot_sy_float - ly /*- virtual_temp1(1)*/;
                }
                else
                {
                    if(zmp_mpcx /* + virtual_temp1(0)*/  > rfoot_sx_float + lx)
                        zmp_mpcx  = rfoot_sx_float + lx /* - virtual_temp1(0)*/ ;
                    else if(zmp_mpcx /* + virtual_temp1(0)*/ < rfoot_sx_float - lx)
                        zmp_mpcx  = rfoot_sx_float - lx /* - virtual_temp1(0)*/;
                   
                    if(zmp_mpcy /* + virtual_temp1(1)*/ < rfoot_sy_float - ly)
                        zmp_mpcy = rfoot_sy_float - ly /*- virtual_temp1(1)*/;
                    else if(zmp_mpcy /* + virtual_temp1(1)*/ > rfoot_sy_float + ly)
                        zmp_mpcy = rfoot_sy_float + ly /*- virtual_temp1(1)*/;
                }

                com_mpcx = desired_val_slow[41]-virtual_temp1(0);
                com_mpcy = desired_val_slow[45]-virtual_temp1(1);  
                angm(0) = desired_val_slow[48] + 0.0;
                angm(1) = desired_val_slow[44] + 0.0;

                for(int i = 0; i < 18; i++)
                {
                    q_desireddot(i) = desired_val_slow[i+21];
                }
               
                if(comd[0] > 0.5)
                    comd[0] = 0.5;
                else if(comd[0] < - 0.5)
                    comd[0] = -0.5;
                if(comd[1] > 0.5)
                    comd[1] = 0.5;
                else if(comd[1] < - 0.5)
                    comd[1] = -0.5;

                if(angm[0] > 9.0)
                    angm[0] = 9.0;
                else if(angm[0] < - 9.0)
                    angm[0] = -9.0;
                if(angm[1] > 9.0)
                    angm[1] = 9.0;
                else if(angm[1] < - 9.0)
                    angm[1] = -9.0;
            }
        }
    }    

    if(walking_tick_stop == false)
    {
        if(walking_tick == 0)
        {
            std::cout << "CP "<< mpc_cycle << " " << com_float_current_(1) + com_vel_current_(1)/wn << std::endl;
            if(mpc_cycle ==0)
            {
                comd_init[0] = com_dot_desired_(0);//com_float_current_dot_LPF(0);
                comd_init[1] = com_dot_desired_(1);//com_float_current_dot_LPF(1);  
            }
            else
            {
                comd_init[0] = comd_s[0];//com_float_current_dot_LPF(0);
                comd_init[1] = comd_s[1];//com_float_current_dot_LPF(1);  
            }

            comd_s[0] = (comd_init[0] *(40-(walking_tick+1)) + comd[0]*(walking_tick+1))/40;
            comd_s[1] = (comd_init[1] *(40-(walking_tick+1)) + comd[1]*(walking_tick+1))/40;
            com_mpc1[2] = com_float_current_(2);

            com_mpc1[1] = (com_mpcy_prev *(40-(walking_tick+1)) + com_mpcy*(walking_tick+1))/40;
            com_mpc1[0] = (com_mpcx_prev *(40-(walking_tick+1)) + com_mpcx*(walking_tick+1))/40;
       
            ZMPx_test = (zmp_mpcx * (walking_tick+1) + ZMPx_prev *(40-walking_tick-1))/40;
            ZMPy_test = (zmp_mpcy * (walking_tick+1) + ZMPy_prev *(40-walking_tick-1))/40;  
            ang_d = (angm * (walking_tick+1) + angm_prev *(40-walking_tick-1))/40;
            q_upper = q_upper + upperd * 0.02/40;
        }
        else
        {
            //comd_s[0] = comd[0];
            //comd_s[1] = comd[1];
            comd_s[0] = (comd_init[0] *(40-(walking_tick+1)) + comd[0]*(walking_tick+1))/40;
            comd_s[1] = (comd_init[1] *(40-(walking_tick+1)) + comd[1]*(walking_tick+1))/40;
           
            com_mpc1[0] = com_mpc1[0] + comd[0] * 0.02/40;
            com_mpc1[1] = (com_mpcy_prev *(40-(walking_tick+1)) + com_mpcy*(walking_tick+1))/40;
            com_mpc1[0] = (com_mpcx_prev *(40-(walking_tick+1)) + com_mpcx*(walking_tick+1))/40;
       
           
            ZMPx_test = (zmp_mpcx * (walking_tick+1) + ZMPx_prev *(40-walking_tick-1))/40;
            ZMPy_test = (zmp_mpcy * (walking_tick+1) + ZMPy_prev *(40-walking_tick-1))/40;                
            ang_d = (angm * (walking_tick+1) + angm_prev *(40-walking_tick-1))/40;
            q_upper = q_upper + upperd * 0.02/40;
        }

        ZMP_gl(0) = ZMPx_test;
        ZMP_gl(1) = ZMPy_test;
        ZMP_gl(2) = 0.0;
        com_mpc = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(supportfoot_float_current_yaw_only), com_mpc1);
        ZMP_gl = DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(supportfoot_float_current_yaw_only), ZMP_gl);
    }        

    if(mpc_cycle == 0)
    {
        rfootd.setZero();
        lfootd.setZero();
    }

    if(mpc_cycle <= 49 && mpc_cycle >= 2)
    {
        rfoot_mpc(0) = (RF_matrix(mpc_cycle,0) * walking_tick + RF_matrix(mpc_cycle-1,0) *(40-walking_tick))/40;
        rfoot_mpc(1) = (RF_matrix(mpc_cycle,1) * walking_tick + RF_matrix(mpc_cycle-1,1) *(40-walking_tick))/40;
        rfoot_mpc(2) = (RF_matrix(mpc_cycle,2) * walking_tick + RF_matrix(mpc_cycle-1,2) *(40-walking_tick))/40;
        lfoot_mpc(0) = (LF_matrix(mpc_cycle,0) * walking_tick + LF_matrix(mpc_cycle-1,0) *(40-walking_tick))/40;
        lfoot_mpc(1) = (LF_matrix(mpc_cycle,1) * walking_tick + LF_matrix(mpc_cycle-1,1) *(40-walking_tick))/40;
        lfoot_mpc(2) = (LF_matrix(mpc_cycle,2) * walking_tick + LF_matrix(mpc_cycle-1,2) *(40-walking_tick))/40;
       
        lfootz = lfoot_mpc(2);
        rfootz = rfoot_mpc(2);
    }
    else if(mpc_cycle > 49 && mpc_cycle < 100)
    {
        rfoot_mpc(0) = (RF_matrix_ssp2(mpc_cycle-49,0) * walking_tick + RF_matrix_ssp2(mpc_cycle-50,0) *(40-walking_tick))/40;
        rfoot_mpc(1) = (RF_matrix_ssp2(mpc_cycle-49,1) * walking_tick + RF_matrix_ssp2(mpc_cycle-50,1) *(40-walking_tick))/40;
        rfoot_mpc(2) = (RF_matrix_ssp2(mpc_cycle-49,2) * walking_tick + RF_matrix_ssp2(mpc_cycle-50,2) *(40-walking_tick))/40;
        lfoot_mpc(0) = (LF_matrix_ssp2(mpc_cycle-49,0) * walking_tick + LF_matrix_ssp2(mpc_cycle-50,0) *(40-walking_tick))/40;
        lfoot_mpc(1) = (LF_matrix_ssp2(mpc_cycle-49,1) * walking_tick + LF_matrix_ssp2(mpc_cycle-50,1) *(40-walking_tick))/40;
        lfoot_mpc(2) = (LF_matrix_ssp2(mpc_cycle-49,2) * walking_tick + LF_matrix_ssp2(mpc_cycle-50,2) *(40-walking_tick))/40;
       
        lfootz = lfoot_mpc(2);
        rfootz = rfoot_mpc(2);
    }
    else if(mpc_cycle >= 100 && mpc_cycle < 150)
    {
        rfoot_mpc(0) = (RF_matrix_ssp1(mpc_cycle-99,0) * walking_tick + RF_matrix_ssp1(mpc_cycle-100,0) *(40-walking_tick))/40;
        rfoot_mpc(1) = (RF_matrix_ssp1(mpc_cycle-99,1) * walking_tick + RF_matrix_ssp1(mpc_cycle-100,1) *(40-walking_tick))/40;
        rfoot_mpc(2) = (RF_matrix_ssp1(mpc_cycle-99,2) * walking_tick + RF_matrix_ssp1(mpc_cycle-100,2) *(40-walking_tick))/40;
        lfoot_mpc(0) = (LF_matrix_ssp1(mpc_cycle-99,0) * walking_tick + LF_matrix_ssp1(mpc_cycle-100,0) *(40-walking_tick))/40;
        lfoot_mpc(1) = (LF_matrix_ssp1(mpc_cycle-99,1) * walking_tick + LF_matrix_ssp1(mpc_cycle-100,1) *(40-walking_tick))/40;
        lfoot_mpc(2) = (LF_matrix_ssp1(mpc_cycle-99,2) * walking_tick + LF_matrix_ssp1(mpc_cycle-100,2) *(40-walking_tick))/40;
       
        lfootz = lfoot_mpc(2);
        rfootz = rfoot_mpc(2);
       
    }
    else if(mpc_cycle >= 150)
    {
        if(mpc_cycle_int % 2 == 0)
        {
            rfoot_mpc(0) = (RF_matrix_ssp2(mpc_cycle_int1 + 1,0) * walking_tick + RF_matrix_ssp2(mpc_cycle_int1,0) *(40-walking_tick))/40;
            rfoot_mpc(1) = (RF_matrix_ssp2(mpc_cycle_int1 + 1,1) * walking_tick + RF_matrix_ssp2(mpc_cycle_int1,1) *(40-walking_tick))/40;
            rfoot_mpc(2) = (RF_matrix_ssp2(mpc_cycle_int1 + 1,2) * walking_tick + RF_matrix_ssp2(mpc_cycle_int1,2) *(40-walking_tick))/40;
            lfoot_mpc(0) = (LF_matrix_ssp2(mpc_cycle_int1 + 1,0) * walking_tick + LF_matrix_ssp2(mpc_cycle_int1,0) *(40-walking_tick))/40;
            lfoot_mpc(1) = (LF_matrix_ssp2(mpc_cycle_int1 + 1,1) * walking_tick + LF_matrix_ssp2(mpc_cycle_int1,1) *(40-walking_tick))/40;
            lfoot_mpc(2) = (LF_matrix_ssp2(mpc_cycle_int1 + 1,2) * walking_tick + LF_matrix_ssp2(mpc_cycle_int1,2) *(40-walking_tick))/40;
           
            lfootz = lfoot_mpc(2);
            rfootz = rfoot_mpc(2);
           
       
        }
        else
        {
            rfoot_mpc(0) = (RF_matrix_ssp1(mpc_cycle_int1 + 1,0) * walking_tick + RF_matrix_ssp1(mpc_cycle_int1,0) *(40-walking_tick))/40;
            rfoot_mpc(1) = (RF_matrix_ssp1(mpc_cycle_int1 + 1,1) * walking_tick + RF_matrix_ssp1(mpc_cycle_int1,1) *(40-walking_tick))/40;
            rfoot_mpc(2) = (RF_matrix_ssp1(mpc_cycle_int1 + 1,2) * walking_tick + RF_matrix_ssp1(mpc_cycle_int1,2) *(40-walking_tick))/40;
            lfoot_mpc(0) = (LF_matrix_ssp1(mpc_cycle_int1 + 1,0) * walking_tick + LF_matrix_ssp1(mpc_cycle_int1,0) *(40-walking_tick))/40;
            lfoot_mpc(1) = (LF_matrix_ssp1(mpc_cycle_int1 + 1,1) * walking_tick + LF_matrix_ssp1(mpc_cycle_int1,1) *(40-walking_tick))/40;
            lfoot_mpc(2) = (LF_matrix_ssp1(mpc_cycle_int1 + 1,2) * walking_tick + LF_matrix_ssp1(mpc_cycle_int1,2) *(40-walking_tick))/40;
           
            lfootz = lfoot_mpc(2);
            rfootz = rfoot_mpc(2);
        }
    }

    if(walking_tick_stop == false)
    {
        if(contactMode == 1)
        {
            if(mpc_cycle == 50 && walking_tick == 0)
            {
                virtual_temp1(0) = virtual_temp(0);
                virtual_temp1(1) = virtual_temp(1);
            }
            if(mpc_cycle == 0 && walking_tick == 0)
            {
                virtual_temp1(0) = virtual_temp(0);
                virtual_temp1(1) = virtual_temp(1);
            }

            if(mpc_cycle == 100 && (walking_tick == 0))
            {
                virtual_temp1(0) = virtual_temp(0);
                virtual_temp1(1) = virtual_temp(1);
            }
            else if(mpc_cycle_int1 == 0 && (walking_tick == 0))
            {
                virtual_temp1(0) = virtual_temp(0);
                virtual_temp1(1) = virtual_temp(1);
            }

            if(walking_tick == 0 || walking_tick == 1)
            {
                virtual_temp1(0) = virtual_temp(0);
            }
        }
        else if(contactMode == 2)
        {
            /*if(mpc_cycle == 20 && walking_tick == 39)
            {
                virtual_temp1(0) = virtual_temp(0);
                virtual_temp1(1) = virtual_temp(1);
            }
            else if(mpc_cycle == 49 && walking_tick == 0)
            {
                virtual_temp1(0) = virtual_temp(0);
                virtual_temp1(1) = virtual_temp(1);
            }
            else if(mpc_cycle_int1 == 19 && walking_tick == 39)//20)
            {
                virtual_temp1(0) = virtual_temp(0);
                virtual_temp1(1) = virtual_temp(1);
            }

            if(mpc_cycle == 98 && (walking_tick == 39))
            {
                virtual_temp1(0) = virtual_temp(0);
                virtual_temp1(1) = virtual_temp(1);
            }
            else if(mpc_cycle_int1 == 48 && (walking_tick == 39))
            {
                virtual_temp1(0) = virtual_temp(0);
                virtual_temp1(1) = virtual_temp(1);
            }*/
            if(walking_tick == 39)
            {
                virtual_temp1(0) = virtual_temp(0);
                virtual_temp1(1) = virtual_temp(1);
            }
        }
        else
        {
            /*if(mpc_cycle == 120 && walking_tick == 39)
            {
                virtual_temp1(0) = virtual_temp(0);
                virtual_temp1(1) = virtual_temp(1);
            }
            else if(mpc_cycle_int1 == 19 && walking_tick == 39)//20)
            {
                virtual_temp1(0) = virtual_temp(0);
                virtual_temp1(1) = virtual_temp(1);
            }

            if(walking_tick == 0)
            {
                virtual_temp1(0) = virtual_temp(0);
                virtual_temp1(1) = virtual_temp(1);
            }

            if(mpc_cycle_int1 == 48 && walking_tick == 39)//20)
            {
                virtual_temp1(0) = virtual_temp(0);
                virtual_temp1(1) = virtual_temp(1);
            }
            else if(mpc_cycle == 48 && walking_tick == 39)
            {
                virtual_temp1(0) = virtual_temp(0);
                virtual_temp1(1) = virtual_temp(1);
            }*/
            if(walking_tick == 39)
            {
                virtual_temp1(0) = virtual_temp(0);
                virtual_temp1(1) = virtual_temp(1);
            }
        }
        walking_tick = walking_tick + 1;
    }
    else
    {
        if(mpc_cycle == 50 && walking_tick == 0)
        {
            virtual_temp1(0) = virtual_temp(0);
            virtual_temp1(1) = virtual_temp(1);
        }
        if(mpc_cycle == 0 && walking_tick == 0)
        {
            virtual_temp1(0) = virtual_temp(0);
            virtual_temp1(1) = virtual_temp(1);
        }

        if(mpc_cycle == 100 && (walking_tick == 0))
        {
            virtual_temp1(0) = virtual_temp(0);
            virtual_temp1(1) = virtual_temp(1);
        }
        else if(mpc_cycle_int1 == 0 && (walking_tick == 0))
        {
            virtual_temp1(0) = virtual_temp(0);
            virtual_temp1(1) = virtual_temp(1);
        }
    }

    if(walking_tick == 1)
    {
        if(mpc_cycle <= controlwalk_time)
            mpc_start_init_ = 1;
    }
}

void CustomController::getMPCTrajectoryInit()
{
    mpc_start_init_ = 4;

    virtual_temp(0) = -((rfoot_float_current_.translation()(0) + lfoot_float_current_.translation()(0))/2 - 6.479871019999999704e-02);
    virtual_temp(1) = -((rfoot_float_current_.translation()(1) + lfoot_float_current_.translation()(1))/2);
    virtual_temp(2) = -((rd_.link_[Right_Foot].xipos(2)+rd_.link_[Left_Foot].xipos(2))/2 - 0.0724);
    if(walking_tick >= 0)
    {  
        state_init_[0] = rd_.q_virtual_[0]+virtual_temp(0);
        state_init_[1] = rd_.q_virtual_[1]+virtual_temp(1);
        state_init_[2] = rd_.q_virtual_[2]+virtual_temp(2);

        for (int i = 3; i < 6; i ++)
            state_init_[i] = rd_.q_virtual_[i];
   
        state_init_[6] = rd_.q_virtual_[39];
   
        for (int i = 6; i < 18; i ++)
            state_init_[i+1] = rd_.q_virtual_[i];
        state_init_[19] = rd_.q_virtual_[19];
        state_init_[20] = rd_.q_virtual_[20];

        for (int i = 0; i < 18; i ++)
            state_init_[i+21] = rd_.q_dot_virtual_[i];
   
        state_init_[39] =  rd_.q_dot_virtual_[19];
        state_init_[40] =  rd_.q_dot_virtual_[20];
   
        state_init_[41] = com_float_current_(0)+virtual_temp(0);
        state_init_[45] = com_float_current_(1)+virtual_temp(1);
        state_init_[42] = com_float_current_dot(0);
        state_init_[46] = com_float_current_dot(1);
   
        state_init_[43] = ZMP_float(0)+virtual_temp(0);//zmp_measured_mj_(0);
        state_init_[47] = ZMP_float(1)+virtual_temp(1);//zmp_measured_mj_(1);
       
        state_init_[44] = model_data_cen.hg.angular()(1);
        state_init_[48] = model_data_cen.hg.angular()(0);//model_data1.hg.angular()[0];

        state_init_[49] = rd_.link_[COM_id].xpos(2) + virtual_temp(2);//com_support_current_(2);//model_data1.hg.angular()[0];
   
        //state_init_[49] = 1.0;//com_support_current_(2) + 0.1585;//model_data1.hg.angular()[0];
        std::cout << "com " <<com_support_current_(2);
        //com_float_current_(0)+virtual_temp(0) << " " << com_float_current_dot_LPF(0) << " "<<com_float_current_(1)+virtual_temp(1) << " " << com_float_current_dot_LPF(1) << " " << wn << " " << com_float_current_(0)+virtual_temp(0) + com_float_current_dot_LPF(0)/wn << " " <<com_float_current_(1) +virtual_temp(1)+ com_float_current_dot_LPF(1)/wn << std::endl;
   
        if(atb_state_update_ == false)
        {
            atb_state_update_ = true;
            state_init_slow = state_init_;
            atb_state_update_ = false;
        }
    }

    mpc_start_init_ = 1;
    std::cout << " CP " << std::endl;
    std::cout << (com_float_current_ + com_float_current_dot/wn).transpose()<< std::endl;
}

void CustomController::proc_recv(){

    while (bind(socket_receive, (struct sockaddr *)&serveraddr1, sizeof(serveraddr1)) < 0) {
        //std::cout << "Connection2 .... " <<  strerror(errno) << std::endl;
    }

    std::cout <<"connect2 OK" << std::endl;

    if(listen(socket_receive, 3) < 0){
        std::cout << "listen err" << std::endl;
    }
    else
    {
        std::cout <<"listen2 ok" << std::endl;
    }

    socklen_t addrlen = sizeof(serveraddr1);
   
    while ((new_socket = accept(socket_receive, (struct sockaddr*)&serveraddr1, &addrlen)) < 0) {
    }

    std::cout <<"accept1 OK" << std::endl;
   
    while(true)
    {
        int valread = recv(new_socket, buffer1, sizeof(buffer1), MSG_WAITALL);
        if(valread == sizeof(buffer1))
        {
            std::copy(&buffer1[1], &buffer1[1] + 49, &desired_val_[0]);
            statemachine_ = buffer1[0];
            thread2_lock.lock();
            desired_val_mu = desired_val_;
            thread2_lock.unlock();
        }
        std::this_thread::sleep_for(std::chrono::microseconds(3));
    }
}

void CustomController::proc_recv1(){

    if(inet_pton(PF_INET, "10.112.1.20", &serveraddr.sin_addr)<=0)
    {
        std::cout << "Invalid address/ Address not supported" << std::endl;
    }
    else
    {
        std::cout << "Ip add1 OK" << std::endl;
    }

    std::cout << "connection1..."<< std::endl;
    while (connect(socket_send, (struct sockaddr*)&serveraddr, sizeof(serveraddr)) < 0) {
    }
    std::cout <<"connect1 OK" << std::endl;

    while(true)
    {
        if(atb_state_update_ == false)
        {
            atb_state_update_ = true;
            state_init_mu= state_init_slow;
            atb_state_update_ = false;
        }

        std::copy(&state_init_mu[0], &state_init_mu[0] + 50, &buffer[1]);
        send(socket_send,buffer,sizeof(buffer),0);

        if (mpc_start_init_ == 1 && mpc_start_init_bool == false)
        {  
            std::cout <<"sned" << std::endl;
            buffer[0] = 1;
            send(socket_send,buffer,sizeof(buffer),0);
            mpc_start_init_bool = true;
            mpc_start_init_bool1 = false;
            mpc_start_init_bool2 = false;
        }
        else if(mpc_start_init_ == 2 && mpc_start_init_bool1 == false)
        {
            buffer[0] = 2;
            send(socket_send,buffer,sizeof(buffer),0);
            mpc_start_init_bool1 = true;
            mpc_start_init_bool = false;
        }
        else if(mpc_start_init_ == 3 && mpc_start_init_bool2 == false)
        {
            buffer[0] = 3;
            send(socket_send,buffer,sizeof(buffer),0);
            mpc_start_init_bool2 = true;
            mpc_start_init_bool = false;
        }
        else if(mpc_start_init_ == 4 && mpc_start_init_bool3 == false)
        {
            buffer[0] = 4;
            mpc_start_init_bool3 = true;
            send(socket_send,buffer,sizeof(buffer),0);
        }
        std::this_thread::sleep_for(std::chrono::microseconds(300));
    }
}

void CustomController::comGainTrajectory()
{
    if (walking_tick_mj < t_start_ + t_rest_init_ + t_double1_)
    {
        if(foot_step_(current_step_num_,6) == 1)
        {
            alpha = (walking_tick_mj - (t_start_ + t_rest_init_ + t_double1_))/(t_rest_last_*2) + 1.0;
        }
        else
        {  
            alpha = -(walking_tick_mj - (t_start_ + t_rest_init_ + t_double1_))/(t_rest_last_*2) + 0.0;
        }
    }
    else if (walking_tick_mj >= t_start_ + t_rest_init_ + t_double1_ && walking_tick_mj < t_start_ + t_total_ - t_double2_ - t_rest_last_)
    {
        if(foot_step_(current_step_num_,6) == 1)
        {
            alpha = 1.0;
        }
        else
        {
            alpha = 0.0;
        }
    }
    else
    {
        if(foot_step_(current_step_num_,6) == 1)
        {
            alpha = -(walking_tick_mj - (t_start_ + t_total_ - t_double2_ - t_rest_last_))/(t_rest_last_*2) + 1.0;
        }
        else
        {  
            alpha = (walking_tick_mj - (t_start_ + t_total_ - t_double2_ - t_rest_last_))/(t_rest_last_*2) + 0.0;
        }
    }
}