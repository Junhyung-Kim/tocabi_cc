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
//pinocchio::Data model_data2;

pinocchio::Model model;

SHMmsgs *mj_shm_;
int shm_msg_id;

void CustomController::proc_recv(){

    while(true)
    {
        int valread = recv(new_socket, buffer1, sizeof(buffer1), MSG_WAITALL);
        if(valread == sizeof(buffer1))
        {
            statemachine = buffer1[0];
            thread1_lock.lock();
            std::copy(&buffer1[1], &buffer1[1] + 49, &desired_val[0]);
            thread1_lock.unlock();

            //std::cout << "G " << buffer1[0]<<std::endl;
        }
    }
}

void CustomController::proc_recv1(){

    while(true)
    {
        thread2_lock.lock();
        std::copy(&state_init_mu[0], &state_init_mu[0] + 50,&buffer[1]);
        thread2_lock.unlock();
        //buffer[0] = mpc_start_init;
        if (mpc_start_init == 1 && mpc_start_init_bool == false)
        {   //MPC cal
            buffer[0] = 1;
            send(socket_send,buffer,sizeof(buffer),0);
            //std::cout << "KK " << mpc_start_init << " " << buffer[0] <<std::endl;
            
            mpc_start_init_bool = true;
            mpc_start_init_bool1 = false;
            mpc_start_init_bool2 = false;
        }
        else if(mpc_start_init == 2 && mpc_start_init_bool1 == false)
        {
            buffer[0] = 2;
            send(socket_send,buffer,sizeof(buffer),0);
            //std::cout << "FFF" << std::endl;
            mpc_start_init_bool1 = true;
            mpc_start_init_bool = false; 
        }
        else if(mpc_start_init == 3 && mpc_start_init_bool2 == false)
        {
            buffer[0] = 3;
            send(socket_send,buffer,sizeof(buffer),0);
            //std::cout << "FFF5" << std::endl;
            mpc_start_init_bool2 = true; 
            mpc_start_init_bool = false; 
        }
        else if(mpc_start_init == 4 && mpc_start_init_bool3 == false)
        {
            buffer[0] = 4;
            //std::cout << "FFF6" << std::endl;
            mpc_start_init_bool3 = true;
            send(socket_send,buffer,sizeof(buffer),0);
        }
    }
}

/*
thread2_lock.lock();
        std::copy(&state_init_mu[0], &state_init_mu[0] + 50,&buffer[1]);
        thread2_lock.unlock();
        
        if (mpc_start_init == 1 && mpc_start_init_bool == false)
        {   //MPC cal
            buffer[0] = 1;
            send(socket_send,buffer,sizeof(buffer),0);
            std::cout << "KK " << mpc_start_init << " " << buffer[0] <<std::endl;
            
            mpc_start_init_bool = true;
            mpc_start_init_bool1 = false;
            mpc_start_init_bool2 = false;
        }
        else if(mpc_start_init == 2 && mpc_start_init_bool1 == false)
        {
            buffer[0] = 2;
            send(socket_send,buffer,sizeof(buffer),0);
            std::cout << "FFF" << std::endl;
            mpc_start_init_bool1 = true;
            mpc_start_init_bool = false; 
        }
        else if(mpc_start_init == 3 && mpc_start_init_bool2 == false)
        {
            buffer[0] = 3;
            send(socket_send,buffer,sizeof(buffer),0);
            std::cout << "FFF5" << std::endl;
            mpc_start_init_bool2 = true; 
            mpc_start_init_bool = false; 
        }
        else if(mpc_start_init == 4 && mpc_start_init_bool == false)
        {
            buffer[0] = 4;
            std::cout << "FFF6" << std::endl;
            send(socket_send,buffer,sizeof(buffer),0);
        }
*/

CustomController::CustomController(RobotData &rd) : rd_(rd) //, wbc_(dc.wbc_)
{
    socket_send = socket(PF_INET, SOCK_STREAM, 0);
    socket_receive = socket(PF_INET, SOCK_STREAM, 0);
    sockaddr_in serveraddr;
    sockaddr_in serveraddr1;
    bzero(&serveraddr, sizeof(serveraddr)); 
    serveraddr.sin_family=PF_INET;
    serveraddr.sin_port=htons(8080);
    serveraddr1.sin_family=PF_INET;
    serveraddr1.sin_port=htons(8081);

    state_init.setZero(50);
    desired_val.setZero(49);
    state_init_mu.setZero(50);
    desired_val_mu.setZero(49);

    time_tick = false;
    time_tick_next = true;
    time_tick_starts = false;

    rfoot_mpc.setZero();
    lfoot_mpc.setZero();

    //state_init_bool = false;
    //desired_init_bool = false;

    if(inet_pton(PF_INET, "127.0.0.1", &serveraddr.sin_addr)<=0) 
    {
        std::cerr << "Invalid address/ Address not supported" << std::endl;
    }

    if(inet_pton(PF_INET, "127.0.0.1", &serveraddr1.sin_addr)<=0) 
    {
        std::cerr << "Invalid address/ Address not supported" << std::endl;
    }
    
    while (bind(socket_receive, (struct sockaddr *)&serveraddr1, sizeof(serveraddr1)) < 0) {
        std::cerr << "Connection2 ...." << std::endl;
    }

    if(listen(socket_receive, 3) < 0){
        std::cout << "listen err" << std::endl;
    }

    socklen_t addrlen = sizeof(serveraddr1);    
    while (connect(socket_send, (sockaddr*)&serveraddr, sizeof(serveraddr)) < 0) {
        std::cerr << "Connection1 ...." << std::endl;
        //break;
    }

    if ((new_socket = accept(socket_receive, (struct sockaddr*)&serveraddr1, &addrlen))
        < 0) {
            std::cout << "accept err" << std::endl;
    }
    sockaddr_in clientaddr;

    upper_on = true;
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

    q_desireddot.resize(18);
    q_desireddot.setZero();

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

    nh.getParam("/tocabi_controller/dist", dist);

    mobgain.push_back(mobgain1);
    mobgain.push_back(mobgain2);
    mobgain.push_back(mobgain3);
    mobgain.push_back(mobgain4);
    mobgain.push_back(mobgain5);
    mobgain.push_back(mobgain6);


    init_shm(shm_msg_key, shm_msg_id, &mj_shm_);

    //mj_shm_->dis_check = true;

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

    pinocchio::urdf::buildModel("/usr/local/lib/python3.8/dist-packages/robot_properties_tocabi/resources/urdf/tocabi.urdf", root_joint, model1);
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
    pinocchio::Data data(model);
    model_data = data;
    model_data1 = data;
    //model_data2 = data;
    q_pinocchio.setZero();
    mpc_on = false;
    wlk_on = false;
    walking_tick = 0;
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
    constraint_size2 = 17;//15;
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

    lfootz = 0;
    rfootz = 0;

    q_dm.setZero();
    qdot_command.setZero();
   
    std::cout << "Custom Controller Init" << std::endl;
    std::fstream read_file("/home/jhk/walkingdata/beforedata/fdyn/lfoot2_final.txt");
    std::fstream read_file1("/home/jhk/walkingdata/beforedata/fdyn/rfoot2_final.txt");
    std::fstream read_file2("/home/jhk/walkingdata/beforedata/fdyn/zmp2_ssp1_1.txt");
    std::fstream read_file3("/home/jhk/walkingdata/beforedata/ssp2/lfoot1.txt");
    std::fstream read_file4("/home/jhk/walkingdata/beforedata/ssp2/rfoot2.txt");
    std::fstream read_file5("/home/jhk/walkingdata/beforedata/ssp2/zmp3.txt");
    std::fstream read_file6("/home/jhk/walkingdata/beforedata/ssp1/lfoot2.txt");
    std::fstream read_file7("/home/jhk/walkingdata/beforedata/ssp1/rfoot2.txt");
    std::fstream read_file8("/home/jhk/walkingdata/beforedata/ssp1/zmp3.txt");
   
    std::vector<double> RF_tran, LF_tran, ZMP_bound, RF_tran_ssp2, LF_tran_ssp2, ZMP_bound_ssp2, RF_tran_ssp1, LF_tran_ssp1, ZMP_bound_ssp1;
    std::string string_test;
    double jointvalue;
    virtual_temp1.setZero();
    control_input.setZero();
    lfootd1.setZero();
    rfootd1.setZero();

    if (read_file.is_open() && read_file3.is_open() && read_file6.is_open())
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

        while (!read_file6.eof())
        {  
            for (int i = 0; i < 3; i++)
            {
                read_file6 >> string_test;
                //string_test.erase(find(string_test.begin(), string_test.end(), ','));
                jointvalue = atof(string_test.c_str());
                if (abs(jointvalue) >= 0.0)
                {  
                    LF_tran_ssp1.push_back(jointvalue);
                }
            }
        }
    }
    else
    {
        std::cout << "LF OPEN FAIL" << std::endl;
    }

    if (read_file1.is_open() && read_file4.is_open() && read_file7.is_open())
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

        while (!read_file7.eof())
        {  
            for (int i = 0; i < 3; i++)
            {
                read_file7 >> string_test;
                //string_test.erase(find(string_test.begin(), string_test.end(), ','));
                jointvalue = atof(string_test.c_str());
                if (abs(jointvalue) >= 0.0)
                {  
                    RF_tran_ssp1.push_back(jointvalue);
                }
            }
        }
    }
    else
    {
        std::cout << "RF OPEN FAIL" << std::endl;
    }

    if (read_file2.is_open() && read_file5.is_open() && read_file8.is_open())
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

        while (!read_file8.eof())
        {  
            for (int i = 0; i < 4; i++)
            {
                read_file8 >> string_test;
                //string_test.erase(find(string_test.begin(), string_test.end(), ','));
                jointvalue = atof(string_test.c_str());
                if (abs(jointvalue) >= 0.0)
                {  
                    ZMP_bound_ssp1.push_back(jointvalue);
                }
            }
        }
    }
    else
    {
        std::cout << "ZMP OPEN FAIL" << std::endl;
    }

    std::cout << "ZMP" << std::endl;
    std::cout << ZMP_bound_ssp1.size() << std::endl;
    std::cout << "RF_tran_ssp1" << std::endl;
    std::cout << RF_tran_ssp1.size() << std::endl;
    std::cout << "LF_tran_ssp1" << std::endl;
    std::cout << LF_tran_ssp1.size() << std::endl;

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
   
    double *RF_tran1_ssp1 = &RF_tran_ssp1[0];
    double *LF_tran1_ssp1 = &LF_tran_ssp1[0];
    double *ZMP_bound1_ssp1 = &ZMP_bound_ssp1[0];
    Eigen::MatrixXd LF_matrix_temp_ssp1 = Eigen::Map<Eigen::MatrixXd>(RF_tran1_ssp1, 3, 133);
    Eigen::MatrixXd RF_matrix_temp_ssp1 = Eigen::Map<Eigen::MatrixXd>(LF_tran1_ssp1, 3, 140);
    Eigen::MatrixXd ZMP_bound_temp_ssp1 = Eigen::Map<Eigen::MatrixXd>(ZMP_bound1_ssp1, 4, 168);
   
    Eigen::MatrixXd RF_matrix1, LF_matrix1;

    RF_matrix1 = RF_matrix_temp.transpose();
    LF_matrix1 = LF_matrix_temp.transpose();

    RF_matrix.resize(109,3);
    LF_matrix.resize(109,3);

    RF_matrix_ssp2 = RF_matrix_temp_ssp2.transpose();
    LF_matrix_ssp2 = LF_matrix_temp_ssp2.transpose();

    RF_matrix_ssp1 = RF_matrix_temp_ssp1.transpose();
    LF_matrix_ssp1 = LF_matrix_temp_ssp1.transpose();
   
    for(int i = 0; i < RF_matrix.rows(); i++)
    {
        RF_matrix.row(i) = RF_matrix1.row(i);
        LF_matrix.row(i) = LF_matrix1.row(i);
    }
    ZMP_bound_matrix = ZMP_bound_temp.transpose();
    ZMP_bound_matrix_ssp2 = ZMP_bound_temp_ssp2.transpose();
    z_ctrl_prev.setZero();
    qd_pinocchio_desired1_prev.setZero();
    qd_pinocchio_desired1.setZero();
       
    walk_start = false;
    thread proc1(&CustomController::proc_recv, this);

    thread proc2(&CustomController::proc_recv1, this);

    proc2.detach();
    proc1.detach();
    //std::cout << "GGG3" << std::endl;
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
    
    pinocchio::Data model_data2(model);
    Eigen::Vector2d ZMP_l, ZMP_r, ZMP;
    ZMP_l(0) = -rd_.LF_CF_FT(4)/rd_.LF_CF_FT(2);
    ZMP_l(1) = rd_.LF_CF_FT(3)/rd_.LF_CF_FT(2);
    ZMP_r(0) = -rd_.RF_CF_FT(4)/rd_.RF_CF_FT(2);
    ZMP_r(1) = rd_.RF_CF_FT(3)/rd_.RF_CF_FT(2);

    int K_temp = 0;
    double b_;
    
    if(mpc_cycle <= 69)
    {
        if(lfootz <= 0.0010 && rfootz <= 0.0010)
        {
            if(mpc_cycle == 47 || mpc_cycle == 48|| mpc_cycle == 49)
            {
                if(mpc_cycle == 49)
                {
                    contactMode = 1;
                }
                else
                {
                    contactMode = 3;
                }
            }
            else
            {
                contactMode = 1;
            }
        }
        else if(lfootz <= 0.0010)
        {
            contactMode = 2;
        }
        else if(rfootz <= 0.0010)
        {
            contactMode = 3;
        }
    }
    else if(mpc_cycle <= 99)
    {
        if(lfootz<= 0.0010 && rfootz <= 0.0010)
        {
            if(mpc_cycle == 95 || mpc_cycle == 96 || mpc_cycle == 97|| mpc_cycle == 98)// || mpc_cycle == 98 || `|| mpc_cycle == 100)
            {
                contactMode = 2;
            }
            else
            {
                contactMode = 1;
            }
        }
        else if(lfootz <= 0.0010)
        {
            contactMode = 2;
            }
        else if(rfootz <= 0.0010)
        {
            contactMode = 3;
        }
    }
    else if(mpc_cycle <= 149)
    {
        if(lfootz <= 0.0010 && rfootz <= 0.0010)
        {
            if(mpc_cycle == 145 || mpc_cycle == 146 || mpc_cycle == 147|| mpc_cycle == 148)
            {
                contactMode = 3;
            }
            else
            {
                contactMode = 1;
            }
        }
        else if(lfootz <= 0.0010)
        {
            contactMode = 2;
        }
        else if(rfootz <= 0.0010)
        {
            contactMode = 3;
        }
    }
    else
    {
        if(mpc_cycle_int % 2 == 0)
        {
            if(lfootz <= 0.0010 && rfootz <= 0.0010)
            {
                if(mpc_cycle == 50 * (mpc_cycle_int + 2) - 5 || mpc_cycle == 50 * (mpc_cycle_int + 2) - 4 || mpc_cycle == 50 * (mpc_cycle_int + 2) - 3|| mpc_cycle == 50 * (mpc_cycle_int + 2) - 2)// || mpc_cycle == 98 || mpc_cycle == 99|| mpc_cycle == 100)
                {
                    contactMode = 2;
                }
                else
                {
                    contactMode = 1;
                }
            }
            else if(lfootz <= 0.0010)
            { 
                contactMode = 2;
            }
            else if(rfootz <= 0.0010)
            {
                contactMode = 3;
            }
        }
        else
        {
            if(lfootz <= 0.0010 && rfootz <= 0.0010)
            {
                if(mpc_cycle == 50 * (mpc_cycle_int + 2) - 5 || mpc_cycle == 50 * (mpc_cycle_int + 2) - 4 || mpc_cycle == 50 * (mpc_cycle_int + 2) - 3|| mpc_cycle == 50 * (mpc_cycle_int + 2) - 2)
                {
                    contactMode = 3;
                }
                else
                {
                    contactMode = 1;
                }
            }
            else if(lfootz <= 0.0010)
            {
                contactMode = 2;
            }
            else if(rfootz <= 0.0010)
            {
                contactMode = 3;
            }
        }
    }
   
    if(walking_tick == 0 && mpc_cycle == 0)
    {
        com_z_init = rd_.link_[COM_id].xpos(2);
    }
   
    cc_mutex.lock();
    for (int i = 0; i < 6; i++)
        q_pinocchio[i] = rd_.q_virtual_[i];

    q_pinocchio[6] = rd_.q_virtual_[39];

    for (int i = 6; i < MODEL_DOF_VIRTUAL; i ++)
        q_pinocchio[i+1] = rd_.q_virtual_[i];

    pinocchio::normalize(model, q_pinocchio);
    Eigen::MatrixXd RFj1, LFj1;
    RFj1.resize(6, MODEL_DOF_VIRTUAL);
    LFj1.resize(6, MODEL_DOF_VIRTUAL);
    CMM = pinocchio::computeCentroidalMap(model, model_data2, q_pinocchio);
    pinocchio::computeCentroidalMomentum(model, model_data1, q_pinocchio, rd_.q_dot_virtual_);  
    cc_mutex.unlock();

    if(control_start == false)
    {    
        for (int i = 0; i < 6; i++)
            q_pinocchio_desired1[i] = rd_.q_virtual_[i];

        q_pinocchio_desired1[6] = rd_.q_virtual_[39];

        for (int i = 6; i < MODEL_DOF_VIRTUAL; i ++)
            q_pinocchio_desired1[i+1] = rd_.q_virtual_[i];

        q_pinocchio_desired = q_pinocchio_desired1;
       
        RFz = -1 * rd_.RF_CF_FT(2);
        LFz = -1 * rd_.LF_CF_FT(2);
     
        zmpx = rd_.link_[COM_id].xpos(0);
    }

    if(walk_start == true)
    {
        zmpx = ZMPx_test;
        zmpy = ZMPy_test;
    }

    //qddot_virtual은 다시해보기
    pinocchio::forwardKinematics(model, model_data2, q_pinocchio);
    pinocchio::centerOfMass(model, model_data2, q_pinocchio, rd_.q_dot_virtual_);
    pinocchio::crba(model, model_data2, q_pinocchio);
    pinocchio::nonLinearEffects(model, model_data2, q_pinocchio, rd_.q_dot_virtual_);
    pinocchio::computeCentroidalMomentum(model, model_data2, q_pinocchio, rd_.q_dot_virtual_);
    pinocchio::computeJointJacobiansTimeVariation(model, model_data2, q_pinocchio, rd_.q_dot_virtual_);
    
    RFj.setZero();
    LFj.setZero();

    cc_mutex.lock();
    pinocchio::computeFrameJacobian(model, model_data2, q_pinocchio, RFcframe_id, RFj);
    pinocchio::computeFrameJacobian(model, model_data2, q_pinocchio, LFcframe_id, LFj);
   
    pinocchio::getFrameJacobianTimeVariation(model, model_data2, RFcframe_id, WORLD, RFdj);
    pinocchio::getFrameJacobianTimeVariation(model, model_data2, LFcframe_id, WORLD, LFdj);
    cc_mutex.unlock();


    RFj.block(0, 18, 6, MODEL_DOF_VIRTUAL -18).setZero();
    LFj.block(0, 18, 6, MODEL_DOF_VIRTUAL -18).setZero();
    if(mpc_cycle > 20 && mpc_cycle <= 49)
    {
        com_alpha = 1.0;    
    }
    else if(mpc_cycle == 19 || mpc_cycle == 20)
    {
        com_alpha = DyrosMath::cubic(walking_tick + 40*(mpc_cycle - 19), 0 , 80, 0.5, 1, 0, 0);
    }
    else if(mpc_cycle == 50 || mpc_cycle == 51)
    {
        com_alpha = DyrosMath::cubic(walking_tick + 40*(mpc_cycle - 50), 0 , 80, 1, 0.5, 0, 0);
    }
    else if(mpc_cycle > 51 && mpc_cycle <= 67)
    {
        com_alpha = 0.5;
    }
    else if(mpc_cycle == 68 || mpc_cycle == 69)
    {
        com_alpha = DyrosMath::cubic(walking_tick + 40*(mpc_cycle - 68), 0 , 80, 0.5, 1, 0, 0);
    }
    else if(mpc_cycle >= 70 && mpc_cycle <= 98)
    {
        com_alpha = 1;
    }
    else if(mpc_cycle == 99 || mpc_cycle == 100)
    {
        com_alpha = DyrosMath::cubic(walking_tick + 40*(mpc_cycle - 99), 0 , 80, 1, 0.5, 0, 0);
    }
    else if(mpc_cycle >= 101 && mpc_cycle <= 117)
    {
        com_alpha = 0.5;    
    }
    else if(mpc_cycle == 118 || mpc_cycle == 119)
    {
        com_alpha = DyrosMath::cubic(walking_tick + 40*(mpc_cycle - 118), 0 , 80, 0.5, 1, 0, 0);
    }
    else if(mpc_cycle >= 119 && mpc_cycle <= 148)
    {
        com_alpha = 1.0; 
    }
    else if(mpc_cycle <= 150 && mpc_cycle > 148)
    {
        com_alpha = DyrosMath::cubic(walking_tick + 40*(mpc_cycle - 149), 0 , 80, 1.0, 0.5, 0, 0);
    }   
    else if(mpc_cycle > 150)
    {
        if(mpc_cycle_int % 2 == 0)
        {
            if(mpc_cycle == (mpc_cycle_int + 1) * 50 + 18 || mpc_cycle == (mpc_cycle_int + 1) * 50 + 19)
            {
                com_alpha = DyrosMath::cubic(walking_tick + 40*(mpc_cycle - ((mpc_cycle_int + 1) * 50 + 18)), 0 , 80, 0.5, 1, 0, 0);
            }
            else if(mpc_cycle == (mpc_cycle_int + 2) * 50 - 1)
            {
                com_alpha = DyrosMath::cubic(walking_tick + 40*(mpc_cycle - ((mpc_cycle_int + 2) * 50 - 1)), 0 , 80, 1, 0.5, 0, 0);
            }
            else if(mpc_cycle == (mpc_cycle_int + 1) * 50)
            {
                com_alpha = DyrosMath::cubic(walking_tick + 40*(mpc_cycle - ((mpc_cycle_int + 1) * 50- 1)), 0 , 80, 1, 0.5, 0, 0);   
            }
        }
        else
        {
            if(mpc_cycle == (mpc_cycle_int + 1) * 50 + 18 || mpc_cycle == (mpc_cycle_int + 1) * 50 + 19)
            {
                com_alpha = DyrosMath::cubic(walking_tick + 40*(mpc_cycle - ((mpc_cycle_int + 1) * 50 + 18)), 0 , 80, 0.5,  1, 0, 0);
            }
            else if(mpc_cycle == (mpc_cycle_int + 2) * 50 - 1 || mpc_cycle == (mpc_cycle_int + 2) * 50)
            {
                com_alpha = DyrosMath::cubic(walking_tick + 40*(mpc_cycle - ((mpc_cycle_int + 2) * 50 - 1)), 0 , 80, 1, 0.5, 0, 0);
            }
            else if(mpc_cycle == (mpc_cycle_int + 1) * 50)
            {
                com_alpha = DyrosMath::cubic(walking_tick + 40*(mpc_cycle - ((mpc_cycle_int + 1) * 50 - 1) ), 0 , 80, 1, 0.5, 0, 0);   
            }
        }
    }
    else
    {
        WBC::SetContact(rd_, 1, 1);
        com_alpha = 0.5;
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
    */
    /*
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
    
    
    double kkkk = 0;
    cc_mutex.lock();    
    if(contactMode == 1)
    {
        if(mpc_cycle_int1 == 49 && (walking_tick == 0 || walking_tick == 1))
            state_virtual = true;

        if(mpc_cycle <= 69)
        {
            if(mpc_cycle >= 49 && (walking_tick == 0 || walking_tick == 1))
            {
                virtual_temp_x = -((model_data2.oMf[RFcframe_id].translation()(0) + model_data2.oMf[LFcframe_id].translation()(0))/2 - 0.094964);
                virtual_temp_y = -((model_data2.oMf[RFcframe_id].translation()(1) + model_data2.oMf[LFcframe_id].translation()(1))/2);
            }
            if(mpc_cycle == 48 && (walking_tick >= 0))
            {
                virtual_temp_x = -((model_data2.oMf[RFcframe_id].translation()(0) + model_data2.oMf[LFcframe_id].translation()(0))/2 - 0.094964);
                virtual_temp_y = -((model_data2.oMf[RFcframe_id].translation()(1) + model_data2.oMf[LFcframe_id].translation()(1))/2);
            }
            else if(LF_matrix(mpc_cycle, 2) <= 0.0000000001 && (walking_tick == 0 || walking_tick == 1))
            {
                virtual_temp_x = -((model_data2.oMf[RFcframe_id].translation()(0) + model_data2.oMf[LFcframe_id].translation()(0))/2 - 9.479871019999999704e-02);
                virtual_temp_y = -((model_data2.oMf[RFcframe_id].translation()(1) + model_data2.oMf[LFcframe_id].translation()(1))/2);
            }
            else if(LF_matrix(mpc_cycle, 2) >= 0.0000000001 && (walking_tick == 0 || walking_tick == 1))
            {
                virtual_temp_y = -(model_data2.oMf[RFcframe_id].translation()(1) + 0.1025);
            }
        }
        else if(mpc_cycle >= 99 && mpc_cycle <= 149)//temp
        {
            if(((walking_tick == 0 && state_virtual == true)|| walking_tick == 1))
            {
                state_virtual = false;
                if(mpc_cycle <= 148)
                    virtual_temp_x = -((model_data2.oMf[RFcframe_id].translation()(0) + model_data2.oMf[LFcframe_id].translation()(0))/2 - 0.094964);
                else
                    virtual_temp_x = -((model_data2.oMf[RFcframe_id].translation()(0) + model_data2.oMf[LFcframe_id].translation()(0))/2 - 0.094964);
            }
            else
            {
                state_virtual = true;
            }
            if(mpc_cycle == 100)
                virtual_temp_y = -((model_data2.oMf[RFcframe_id].translation()(1) + model_data2.oMf[LFcframe_id].translation()(1))/2);
        }
        else
        {
            if(mpc_cycle_int % 2 == 0)
            {
                if(((walking_tick == 0 && state_virtual == true) || walking_tick == 1))
                {
                    state_virtual = false;
                    virtual_temp_x = -((model_data2.oMf[RFcframe_id].translation()(0) + model_data2.oMf[LFcframe_id].translation()(0))/2 - 0.094964);
                }
                else
                {
                    state_virtual = true;
                }
                
            }
            else
            {
                if(((walking_tick == 0 && state_virtual == true) || walking_tick == 1))
                {
                    state_virtual = false;
                    virtual_temp_x = -((model_data2.oMf[RFcframe_id].translation()(0) + model_data2.oMf[LFcframe_id].translation()(0))/2 - 0.094964);
                }

                else
                {
                    state_virtual = true;
                }
            }
        }

        if(mpc_cycle == 49 && (walking_tick == 40 || walking_tick == 39))//49)
        {
            Debug_check = 1;
            virtual_temp(0) = virtual_temp_x;
            virtual_temp1_x = virtual_temp(0);//-(model_data2.oMf[RFcframe_id].translation()(0) - 9.479871019999999704e-02);
        }
        else if(mpc_cycle_int1 == 49 && (walking_tick == 40 || walking_tick == 39))
        {
            Debug_check  =-1;
            virtual_temp(0) = virtual_temp_x;
            virtual_temp1_x = virtual_temp(0);//-(model_data2.oMf[RFcframe_id].translation()(0) - 9.479871019999999704e-02);
        }
        else
            Debug_check = 0;
        if(mpc_cycle == 49)
        {
            virtual_temp2(0) = virtual_temp_x;
            virtual_temp2_x = virtual_temp2(0);
        }
            
        else if(mpc_cycle_int1 == 0)
        {
            virtual_temp2(0) = virtual_temp_x;
            virtual_temp2_x = virtual_temp2(0);
        }
    }    
    else if(contactMode == 2)
    {
        if(walking_tick == 0)
        {
            if(state_virtual == true)
            {
                virtual_temp_x =  -(model_data2.oMf[LFcframe_id].translation()(0) - 1.451300000000000090e-01);
                virtual_temp_y =  -(model_data2.oMf[LFcframe_id].translation()(1) - 0.1025);
                state_virtual = false;
            }
            if(mpc_cycle_int1 == 20)
            {
                virtual_temp2(0) = virtual_temp_x;
                virtual_temp2_x = virtual_temp2(0);
            }
        }
        else
            state_virtual = true;
        if(mpc_cycle_int1 == 20 && walking_tick == 10)//20)
        {
            virtual_temp2(0) = virtual_temp_x;
            virtual_temp1_x = virtual_temp2(0);
        }   
    }
    else
    {  
        if(walking_tick == 0)
        {
            if(state_virtual == true)
            {
                if(mpc_cycle < 100)
                    virtual_temp_x = -(model_data2.oMf[RFcframe_id].translation()(0) - 9.479871019999999704e-02);
                else
                    virtual_temp_x = -(model_data2.oMf[RFcframe_id].translation()(0) - 1.451300000000000090e-01);    
                virtual_temp_y = -(model_data2.oMf[RFcframe_id].translation()(1) + 0.1025);
                state_virtual = false;
            }

            if(mpc_cycle_int1 == 20)
            {
                virtual_temp2(0) = virtual_temp_x;
                virtual_temp2_x = virtual_temp2(0);
            }
        }
        else
            state_virtual = true;
        if(mpc_cycle_int1 == 20 && walking_tick == 10)//20)
        {
            virtual_temp2(0) = virtual_temp_x;
            virtual_temp1_x = virtual_temp2(0);
        }
    }
    cc_mutex.unlock();


    if(mpc_cycle <= 69)
    {
        if(lfootz <= 0.0010 && rfootz <= 0.0010)
        {
            if(mpc_cycle == 47 || mpc_cycle == 48|| mpc_cycle == 49)
            {
                ZMP(0) = ZMP_r(0)+ RF_matrix(mpc_cycle,0);
                ZMP(1) = ZMP_r(1)+ RF_matrix(mpc_cycle,1);
                ZMP_l(0) = 0.0;
                ZMP_l(1) = 0.0;
            }
            else
            {
                ZMP(0) = ((ZMP_l(0) + LF_matrix(mpc_cycle,0))*rd_.LF_CF_FT(2) + (ZMP_r(0) + RF_matrix(mpc_cycle,0))*rd_.RF_CF_FT(2))/(rd_.RF_CF_FT(2) + rd_.LF_CF_FT(2));
                ZMP(1) = ((ZMP_l(1) + LF_matrix(mpc_cycle,1))*rd_.LF_CF_FT(2) + (ZMP_r(1) + RF_matrix(mpc_cycle,1))*rd_.RF_CF_FT(2))/(rd_.RF_CF_FT(2) + rd_.LF_CF_FT(2));
            }
        }
        else if(lfootz <= 0.0010)
        {
            ZMP(0) = ZMP_l(0)+ LF_matrix(mpc_cycle,0);
            ZMP(1) = ZMP_l(1)+ LF_matrix(mpc_cycle,1);
            ZMP_r(0) = 0.0;
            ZMP_r(1) = 0.0;
        }
        else if(rfootz <= 0.0010)
        {
            ZMP(0) = ZMP_r(0)+ RF_matrix(mpc_cycle,0);
            ZMP(1) = ZMP_r(1)+ RF_matrix(mpc_cycle,1);
            ZMP_l(0) = 0.0;
            ZMP_l(1) = 0.0;
        }
    }
    else if(mpc_cycle <= 99)
    {
        if(lfootz<= 0.0010 && rfootz <= 0.0010)
        {
            if(mpc_cycle == 95 || mpc_cycle == 96 || mpc_cycle == 97|| mpc_cycle == 98)// || mpc_cycle == 98 || mpc_cycle == 99|| mpc_cycle == 100)
            {
                ZMP(0) = ZMP_l(0)+ LF_matrix(mpc_cycle,0);
                ZMP(1) = ZMP_l(1)+ LF_matrix(mpc_cycle,1);
                ZMP_r(0) = 0.0;
                ZMP_r(1) = 0.0;
            }
            else
            {
                ZMP(0) = ((ZMP_l(0) + LF_matrix_ssp2(mpc_cycle-49,0) - virtual_temp2_x)*rd_.LF_CF_FT(2) + (ZMP_r(0) + RF_matrix_ssp2(mpc_cycle-49,0) - virtual_temp2_x)*rd_.RF_CF_FT(2))/(rd_.RF_CF_FT(2) + rd_.LF_CF_FT(2));
                ZMP(1) = ((ZMP_l(1) + LF_matrix_ssp2(mpc_cycle-49,1))*rd_.LF_CF_FT(2) + (ZMP_r(1) + RF_matrix_ssp2(mpc_cycle-49,1))*rd_.RF_CF_FT(2))/(rd_.RF_CF_FT(2) + rd_.LF_CF_FT(2));
            }
        }
        else if(lfootz <= 0.0010)
        {
            ZMP(0) = ZMP_l(0)+ LF_matrix_ssp2(mpc_cycle-49,0) - virtual_temp2_x;
            ZMP(1) = ZMP_l(1)+ LF_matrix_ssp2(mpc_cycle-49,1);
            ZMP_r(0) = 0.0;
            ZMP_r(1) = 0.0;
        }
        else if(rfootz <= 0.0010)
        {
            ZMP(0) = ZMP_r(0)+ RF_matrix_ssp2(mpc_cycle-49,0) - virtual_temp2_x;
            ZMP(1) = ZMP_r(1)+ RF_matrix_ssp2(mpc_cycle-49,1);
            ZMP_l(0) = 0.0;
            ZMP_l(1) = 0.0;
        }
    }
    else if(mpc_cycle <= 149)
    {
        if(lfootz <= 0.0010 && rfootz <= 0.0010)
        {
            if(mpc_cycle == 145 || mpc_cycle == 146 || mpc_cycle == 147|| mpc_cycle == 148)
            {
                ZMP(0) = ZMP_r(0)+ RF_matrix_ssp1(mpc_cycle-99,0) - virtual_temp2_x;
                ZMP(1) = ZMP_r(1)+ RF_matrix_ssp1(mpc_cycle-99,1);
                ZMP_l(0) = 0.0;
                ZMP_l(1) = 0.0;
            }
            else
            {
                ZMP(0) = ((ZMP_l(0) + LF_matrix_ssp1(mpc_cycle-99,0) - virtual_temp2_x)*rd_.LF_CF_FT(2) + (ZMP_r(0) + RF_matrix_ssp1(mpc_cycle-99,0) - virtual_temp2_x)*rd_.RF_CF_FT(2))/(rd_.RF_CF_FT(2) + rd_.LF_CF_FT(2));
                ZMP(1) = ((ZMP_l(1) + LF_matrix_ssp1(mpc_cycle-99,1))*rd_.LF_CF_FT(2) + (ZMP_r(1) + RF_matrix_ssp1(mpc_cycle-99,1))*rd_.RF_CF_FT(2))/(rd_.RF_CF_FT(2) + rd_.LF_CF_FT(2));
            }
        }
        else if(lfootz <= 0.0010)
        {
            ZMP(0) = ZMP_l(0)+ LF_matrix_ssp1(mpc_cycle-99,0) - virtual_temp2_x;
            ZMP(1) = ZMP_l(1)+ LF_matrix_ssp1(mpc_cycle-99,1);
            ZMP_r(0) = 0.0;
            ZMP_r(1) = 0.0;
        }
        else if(rfootz <= 0.0010)
        {
            ZMP(0) = ZMP_r(0)+ RF_matrix_ssp1(mpc_cycle-99,0) - virtual_temp2_x;
            ZMP(1) = ZMP_r(1)+ RF_matrix_ssp1(mpc_cycle-99,1);
            ZMP_l(0) = 0.0;
            ZMP_l(1) = 0.0;
        }
    }
    else
    {
        if(mpc_cycle_int % 2 == 0)
        {
            if(lfootz <= 0.0010 && rfootz <= 0.0010)
            {
                if(mpc_cycle == 50 * (mpc_cycle_int + 2) - 5 || mpc_cycle == 50 * (mpc_cycle_int + 2) - 4 || mpc_cycle == 50 * (mpc_cycle_int + 2) - 3|| mpc_cycle == 50 * (mpc_cycle_int + 2) - 2)// || mpc_cycle == 98 || mpc_cycle == 99|| mpc_cycle == 100)
                {
                    ZMP(0) = ZMP_l(0)+ LF_matrix_ssp2(mpc_cycle_int1,0)- virtual_temp2_x;
                    ZMP(1) = ZMP_l(1)+ LF_matrix_ssp2(mpc_cycle_int1,1);
                    ZMP_r(0) = 0.0;
                    ZMP_r(1) = 0.0;
                }
                else
                {
                    ZMP(0) = ((ZMP_l(0) + LF_matrix_ssp2(mpc_cycle_int1,0) - virtual_temp2_x)*rd_.LF_CF_FT(2) + (ZMP_r(0) + RF_matrix_ssp2(mpc_cycle_int1,0) - virtual_temp2_x)*rd_.RF_CF_FT(2))/(rd_.RF_CF_FT(2) + rd_.LF_CF_FT(2));
                    ZMP(1) = ((ZMP_l(1) + LF_matrix_ssp2(mpc_cycle_int1,1))*rd_.LF_CF_FT(2) + (ZMP_r(1) + RF_matrix_ssp2(mpc_cycle_int1,1))*rd_.RF_CF_FT(2))/(rd_.RF_CF_FT(2) + rd_.LF_CF_FT(2));
                }
            }
            else if(lfootz <= 0.0010)
            { 
                ZMP(0) = ZMP_l(0)+ LF_matrix_ssp2(mpc_cycle_int1,0) - virtual_temp2_x;
                ZMP(1) = ZMP_l(1)+ LF_matrix_ssp2(mpc_cycle_int1,1);
                ZMP_r(0) = 0.0;
                ZMP_r(1) = 0.0;
            }
            else if(rfootz <= 0.0010)
            {
                ZMP(0) = ZMP_r(0)+ RF_matrix_ssp2(mpc_cycle_int1,0) - virtual_temp2_x;
                ZMP(1) = ZMP_r(1)+ RF_matrix_ssp2(mpc_cycle_int1,1);
                ZMP_l(0) = 0.0;
                ZMP_l(1) = 0.0;
            }
        }
        else
        {
            if(lfootz <= 0.0010 && rfootz <= 0.0010)
            {
                if(mpc_cycle == 50 * (mpc_cycle_int + 2) - 5 || mpc_cycle == 50 * (mpc_cycle_int + 2) - 4 || mpc_cycle == 50 * (mpc_cycle_int + 2) - 3|| mpc_cycle == 50 * (mpc_cycle_int + 2) - 2)
                {
                    ZMP(0) = ZMP_r(0)+ RF_matrix_ssp1(mpc_cycle_int1,0) - virtual_temp2_x;
                    ZMP(1) = ZMP_r(1)+ RF_matrix_ssp1(mpc_cycle_int1,1);
                    ZMP_l(0) = 0.0;
                    ZMP_l(1) = 0.0;
                }
                else
                {
                    ZMP(0) = ((ZMP_l(0) + LF_matrix_ssp1(mpc_cycle_int1,0) - virtual_temp2_x)*rd_.LF_CF_FT(2) + (ZMP_r(0) + RF_matrix_ssp1(mpc_cycle_int1,0) - virtual_temp2_x)*rd_.RF_CF_FT(2))/(rd_.RF_CF_FT(2) + rd_.LF_CF_FT(2));
                    ZMP(1) = ((ZMP_l(1) + LF_matrix_ssp1(mpc_cycle_int1,1))*rd_.LF_CF_FT(2) + (ZMP_r(1) + RF_matrix_ssp1(mpc_cycle_int1,1))*rd_.RF_CF_FT(2))/(rd_.RF_CF_FT(2) + rd_.LF_CF_FT(2));
                }
            }
            else if(lfootz <= 0.0010)
            {
                ZMP(0) = ZMP_l(0)+ LF_matrix_ssp1(mpc_cycle_int1,0) - virtual_temp2_x;
                ZMP(1) = ZMP_l(1)+ LF_matrix_ssp1(mpc_cycle_int1,1);
                ZMP_r(0) = 0.0;
                ZMP_r(1) = 0.0;
            }
            else if(rfootz <= 0.0010)
            {
                ZMP(0) = ZMP_r(0)+ RF_matrix_ssp1(mpc_cycle_int1,0) - virtual_temp2_x;
                ZMP(1) = ZMP_r(1)+ RF_matrix_ssp1(mpc_cycle_int1,1);
                ZMP_l(0) = 0.0;
                ZMP_l(1) = 0.0;
            }
        }
    }

    if(walking_tick == 0 && mpc_cycle == 0)
    {
        ZMP_FT_law(0) = ZMP(0);
        ZMP_FT_law(1) = ZMP(1);
    }
   
    cc_mutex.lock();
    ZMP_FT_law(0) = DyrosMath::lpf(ZMP(0), ZMP_FT_law(0), 2000, 10);
    ZMP_FT_law(1) = DyrosMath::lpf(ZMP(1), ZMP_FT_law(1), 2000, 10);
    //ZMP_FT(0) = ZMP(0);
    //ZMP_FT(1) = ZMP(1);
    cc_mutex.unlock();

   
    if(control_start == false || control_time > 0)// && walking_tick_stop == tru) //추후 삭제
    {  
        control_start = true;
        
        if(contactMode == 1)
            {
                double lx, ly, mu;
                ly = 0.048;
                lx = 0.11;
                mu = 0.8;

                if(model_data2.oMf[LFcframe_id].translation()(0) >  model_data2.oMf[RFcframe_id].translation()(0))
                {
                    if(zmpx > model_data2.oMf[LFcframe_id].translation()(0) + lx)
                        zmpx  = model_data2.oMf[LFcframe_id].translation()(0) + lx;
                    if(zmpx < model_data2.oMf[RFcframe_id].translation()(0) - lx)
                        zmpx  = model_data2.oMf[RFcframe_id].translation()(0) - lx;

                    zmp_bx(0) =  model_data2.oMf[LFcframe_id].translation()(0) + lx;
                    zmp_bx(1) =  model_data2.oMf[RFcframe_id].translation()(0) - lx;
                }
                else if(model_data2.oMf[LFcframe_id].translation()(0) <  model_data2.oMf[RFcframe_id].translation()(0))
                {
                    if(zmpx > model_data2.oMf[RFcframe_id].translation()(0) + lx)
                        zmpx  = model_data2.oMf[RFcframe_id].translation()(0) + lx;
                    if(zmpx < model_data2.oMf[LFcframe_id].translation()(0) - lx)
                        zmpx  = model_data2.oMf[LFcframe_id].translation()(0) - lx;
                    zmp_bx(0) =  model_data2.oMf[RFcframe_id].translation()(0) + lx;
                    zmp_bx(1) =  model_data2.oMf[LFcframe_id].translation()(0) - lx;
                }
                else
                {
                    if(zmpx > model_data2.oMf[RFcframe_id].translation()(0) + lx)
                        zmpx  = model_data2.oMf[RFcframe_id].translation()(0) + lx;
                    if(zmpx < model_data2.oMf[LFcframe_id].translation()(0) - lx)
                        zmpx  = model_data2.oMf[LFcframe_id].translation()(0) - lx;
                    zmp_bx(0) =  model_data2.oMf[LFcframe_id].translation()(0) + lx;
                    zmp_bx(1) =  model_data2.oMf[RFcframe_id].translation()(0) - lx;
                }

                if(zmpy > model_data2.oMf[LFcframe_id].translation()(1) + ly)
                {
                    zmpy = model_data2.oMf[LFcframe_id].translation()(1) + ly;
                }

                if(zmpy < model_data2.oMf[RFcframe_id].translation()(1) - ly)
                {
                    zmpy = model_data2.oMf[RFcframe_id].translation()(1) - ly;
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
                
                M_ = model_data2.M;
                nle = model_data2.nle;

                H1.setIdentity();
                H1.block(12,12,MODEL_DOF_VIRTUAL,MODEL_DOF_VIRTUAL) = 100 *H1.block(12,12,MODEL_DOF_VIRTUAL,MODEL_DOF_VIRTUAL);
                
                lb1.setConstant(variable_size1, -100000);
                ub1.setConstant(variable_size1, 100000);

                H1(2,2) = 1.0;
                H1(8,8) = 1.0;
                g1(2) = - com_alpha * nle(2) * 1.0;
                g1(8) = - (1-com_alpha) * nle(2) * 1.0;
                lb1(2) = 0.0;
                lb1(8) = 0.0;

                lb1(15) = 0.0;
                lb1(16) = 0.0;
                ub1(15) = 0.0;
                ub1(16) = 0.0;
            
                H1.block(15,15,3,3) = 100 * H1.block(15,15,3,3);

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
            
                A1.block(22,0,1,6)(0,2) = (model_data2.oMf[RFcframe_id].translation()(1)  - zmpy);
                A1.block(22,0,1,6)(0,3) = 1;
                A1.block(22,6,1,6)(0,2) = (model_data2.oMf[LFcframe_id].translation()(1)  - zmpy);
                A1.block(22,6,1,6)(0,3) = 1;
                lbA1(22) = 0.0;
                ubA1(22) = 0.0;

                A1.block(23,0,1,6)(0,2) = (model_data2.oMf[RFcframe_id].translation()(0)  - zmpx);
                A1.block(23,0,1,6)(0,4) = -1;
                A1.block(23,6,1,6)(0,2) = (model_data2.oMf[LFcframe_id].translation()(0)  - zmpx);
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

                g_temp.tail(12) <<  RFdj * rd_.q_dot_virtual_ + RFj * qdd_pinocchio_desired1_, LFdj * rd_.q_dot_virtual_ + LFj * qdd_pinocchio_desired1_;

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
            
                M_ = model_data2.M;
                nle = model_data2.nle;

                H1.setIdentity();
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
                g_temp.head(6) <<  LFdj * rd_.q_dot_virtual_ + LFj * qdd_pinocchio_desired1_;
                
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
            
                M_ = model_data2.M;
                nle = model_data2.nle;

                H1.setIdentity();
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
                g_temp.head(6) <<  RFdj * rd_.q_dot_virtual_ + RFj * qdd_pinocchio_desired1_;
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
        /*
        if(contactMode == 1)// && mpc_cycle != 49)
        {
            double lx, ly, mu;
            ly = 0.048;
            lx = 0.11;
            mu = 0.8;

            if(model_data2.oMf[LFcframe_id].translation()(0) >  model_data2.oMf[RFcframe_id].translation()(0))
            {
                if(zmpx > model_data2.oMf[LFcframe_id].translation()(0) + lx)
                    zmpx  = model_data2.oMf[LFcframe_id].translation()(0) + lx;
                if(zmpx < model_data2.oMf[RFcframe_id].translation()(0) - lx)
                    zmpx  = model_data2.oMf[RFcframe_id].translation()(0) - lx;

                zmp_bx(0) =  model_data2.oMf[LFcframe_id].translation()(0) + lx;
                zmp_bx(1) =  model_data2.oMf[RFcframe_id].translation()(0) - lx;
            }
            else if(model_data2.oMf[LFcframe_id].translation()(0) <  model_data2.oMf[RFcframe_id].translation()(0))
            {
                if(zmpx > model_data2.oMf[RFcframe_id].translation()(0) + lx)
                    zmpx  = model_data2.oMf[RFcframe_id].translation()(0) + lx;
                if(zmpx < model_data2.oMf[LFcframe_id].translation()(0) - lx)
                    zmpx  = model_data2.oMf[LFcframe_id].translation()(0) - lx;
                zmp_bx(0) =  model_data2.oMf[RFcframe_id].translation()(0) + lx;
                zmp_bx(1) =  model_data2.oMf[LFcframe_id].translation()(0) - lx;
            }
            else
            {
                if(zmpx > model_data2.oMf[RFcframe_id].translation()(0) + lx)
                    zmpx  = model_data2.oMf[RFcframe_id].translation()(0) + lx;
                if(zmpx < model_data2.oMf[LFcframe_id].translation()(0) - lx)
                    zmpx  = model_data2.oMf[LFcframe_id].translation()(0) - lx;
                zmp_bx(0) =  model_data2.oMf[LFcframe_id].translation()(0) + lx;
                zmp_bx(1) =  model_data2.oMf[RFcframe_id].translation()(0) - lx;
            }

            if(zmpy > model_data2.oMf[LFcframe_id].translation()(1) + ly)
            {
                zmpy = model_data2.oMf[LFcframe_id].translation()(1) + ly;
            }

            if(zmpy < model_data2.oMf[RFcframe_id].translation()(1) - ly)
            {
                zmpy = model_data2.oMf[RFcframe_id].translation()(1) - ly;
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
            
            M_ = model_data2.M;
            nle = model_data2.nle;

            H1.setIdentity();
            H1.block(12,12,MODEL_DOF_VIRTUAL,MODEL_DOF_VIRTUAL) = 100 *H1.block(12,12,MODEL_DOF_VIRTUAL,MODEL_DOF_VIRTUAL);
            lb1.setConstant(variable_size1, -100000);
            ub1.setConstant(variable_size1, 100000);

            H1(2,2) = 1.0;
            H1(8,8) = 1.0;
            g1(2) = - com_alpha * nle(2) * 1.0;
            g1(8) = - (1-com_alpha) * nle(2) * 1.0;
            lb1(2) = 0.0;
            lb1(8) = 0.0;
           
            H1.block(12,12,6,6) = 2 * H1.block(12,12,6,6);

            qdd_pinocchio_desired1_ = qdd_pinocchio_desired1;
       
            lbA1.head(6) = (nle + M_ * qdd_pinocchio_desired1_).head(6);
            ubA1.head(6) = (nle + M_ * qdd_pinocchio_desired1_).head(6);
           
            double weight_resi = 0.0, weight_resi1 = 2.0;;
            G_temp.setZero();
            g_temp.setZero();
            G_temp.block(0,0,6,MODEL_DOF_VIRTUAL) = RFj;
            G_temp.block(6,0,6,MODEL_DOF_VIRTUAL) = LFj;

            g_temp.tail(12) <<  RFdj * rd_.q_dot_virtual_ + RFj * qdd_pinocchio_desired1_, LFdj * rd_.q_dot_virtual_ + LFj * qdd_pinocchio_desired1_;
           
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
           
            A1.block(22,0,1,6)(0,2) = (model_data2.oMf[RFcframe_id].translation()(1)  - zmpy);
            A1.block(22,0,1,6)(0,3) = 1;
            A1.block(22,6,1,6)(0,2) = (model_data2.oMf[LFcframe_id].translation()(1)  - zmpy);
            A1.block(22,6,1,6)(0,3) = 1;
            lbA1(22) = 0.0;
            ubA1(22) = 0.0;

            A1.block(23,0,1,6)(0,2) = (model_data2.oMf[RFcframe_id].translation()(0)  - zmpx);
            A1.block(23,0,1,6)(0,4) = -1;
            A1.block(23,6,1,6)(0,2) = (model_data2.oMf[LFcframe_id].translation()(0)  - zmpx);
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

            qp_torque_control.UpdateMinProblem(H1, g1);
            qp_torque_control.UpdateSubjectToAx(A1, lbA1, ubA1);
            qp_torque_control.UpdateSubjectToX(lb1, ub1);
            solved = qp_torque_control.SolveQPoases(100, qp_result);
           
            if (solved == true)
            {
                if(control_time >= 1)
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
                }
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
           
            M_ = model_data2.M;
            nle = model_data2.nle;

            H1.setIdentity();
            H1.block(12,12,MODEL_DOF_VIRTUAL,MODEL_DOF_VIRTUAL) = 1000 *H1.block(12,12,MODEL_DOF_VIRTUAL,MODEL_DOF_VIRTUAL);
            lb1.setConstant(variable_size1, -100000);
            ub1.setConstant(variable_size1, 100000);
           
           
            H1(8,8) = 5;
            g1(8) = - com_alpha * nle(2) * 5;
            lb1(2) = 0.0;
            lb1(8) = 0.0;

            H1.block(12,12,6,6) = 100 * H1.block(12,12,6,6);
           
            qdd_pinocchio_desired1_ = qdd_pinocchio_desired1;
           
            double weight_resi = 100.0;
            G_temp.setZero();
            g_temp.setZero();
            G_temp.block(0,0,6,MODEL_DOF_VIRTUAL) = LFj;
            g_temp.head(6) <<  LFdj * rd_.q_dot_virtual_ + LFj * qdd_pinocchio_desired1_;
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
   
            qp_torque_control.UpdateMinProblem(H1, g1);
            qp_torque_control.UpdateSubjectToAx(A1, lbA1, ubA1);
            qp_torque_control.UpdateSubjectToX(lb1, ub1);
            solved = qp_torque_control.SolveQPoases(100, qp_result);
           
            if (solved == true)
            {
                tau_ = ((M_ * (qdd_pinocchio_desired1_  + qp_result.segment<MODEL_DOF_VIRTUAL>(12)) + nle - (RFj.transpose() * qp_result.head(6) + LFj.transpose() * qp_result.segment<6>(6))).transpose());

            }
                //tau_ = ((M_ * (qdd_pinocchio_desired1_ + qp_result.segment<MODEL_DOF_VIRTUAL>(12)) + nle - (RFj.transpose() * qp_result.head(6) + LFj.transpose() * qp_result.segment<6>(6))).transpose());
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
           
            M_ = model_data2.M;
            nle = model_data2.nle;

            H1.setIdentity();
            H1.block(12,12,MODEL_DOF_VIRTUAL,MODEL_DOF_VIRTUAL) = 10000 *H1.block(12,12,MODEL_DOF_VIRTUAL,MODEL_DOF_VIRTUAL);
            lb1.setConstant(variable_size1, -100000);
            ub1.setConstant(variable_size1, 100000);

            
            H1(2,2) = 5;
            g1(2) = - com_alpha * nle(2) * 5;
            lb1(2) = 0.0;
            lb1(8) = 0.0;

            H1.block(12,12,6,6) = 100 * H1.block(12,12,6,6);
           
            
            qdd_pinocchio_desired1_ = qdd_pinocchio_desired1;
           
            double weight_resi = 100.0;
            G_temp.setZero();
            g_temp.setZero();
            G_temp.block(0,0,6,MODEL_DOF_VIRTUAL) = RFj;
            g_temp.head(6) <<  RFdj * rd_.q_dot_virtual_ + RFj * qdd_pinocchio_desired1_;
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

            qp_torque_control.UpdateMinProblem(H1, g1);
            qp_torque_control.UpdateSubjectToAx(A1, lbA1, ubA1);
            qp_torque_control.UpdateSubjectToX(lb1, ub1);
            solved = qp_torque_control.SolveQPoases(100, qp_result);
           
            if (solved == true)
            {
                    tau_ = ((M_ * (qdd_pinocchio_desired1_ + qp_result.segment<MODEL_DOF_VIRTUAL>(12)) + nle - (RFj.transpose() * qp_result.head(6) + LFj.transpose() * qp_result.segment<6>(6))).transpose());
                    qp_result_prev = qp_result;
                //}
            }
            control_time = control_time + 1;
        }*/
    }
    

    if(mpc_cycle >= 2 && mpc_cycle < controlwalk_time && statemachine != 3)// &&walking_tick_stop == false )
    {
        file[1] << mpc_cycle << " " << walking_tick << " " << desired_val_mu[0]  << " " << desired_val_mu[1] << " " << desired_val_mu[2]<< " " << desired_val_mu[43];
        //file[1] << virtual_temp_x << " " << virtual_temp1_x << " " << zmp_mpcx << " "<< virtual_temp_y << " " << virtual_temp1_y << " " << desired_val_mu[43] <<  " " << rd_.q_(13) << " " << rd_.q_(14) << " " << desired_val_mu[19] << " " << desired_val_mu[20] << " " << pelv_ori_c(0) <<  "  " << pelv_ori_c(1) << std::endl;// << rfoot_ori_c(0) << " " << rfoot_ori_c(1) << " " << rfoot_ori_c(2) << " " << lfoot_ori_c(0) << " " << lfoot_ori_c(1) << " " << lfoot_ori_c(2);
        file[1] << std::endl;
        file[0] <<contactMode << " " << Debug_check << " " << mpc_cycle << " " <<walking_tick << " "<< qp_solved<< " "<<solved<< " "<< mpc_cycle_int1 << " " << mpc_cycle_int << " "<< lfoot_mpc(0)<< " " << model_data2.oMf[LFcframe_id].translation()(0)+ virtual_temp1_x << " "<< rfoot_mpc(0)<< " " <<model_data2.oMf[RFcframe_id].translation()(0) + virtual_temp1_x<< " "<<state_virtual << " " << virtual_temp_x<< " " << virtual_temp1_x<< " " << virtual_temp2_x<< " "<< com_mpc[0] << " " << com_mpc[1] << " "<< rd_.link_[COM_id].xpos(0)<< " " << rd_.link_[COM_id].xpos(1)<< " " << angd_(0) << " " << angd_(1) << " "  << model_data1.hg.angular()[0]  << " " << model_data1.hg.angular()[1] << " " <<H_temp_22 << " " << pelv_ori_c(1) << " " << zmpx << " " << zmpy << " " << ZMP_FT_law(0)<< " " << ZMP_FT_law(1)<< " " <<zmp_bx(0) << " " << zmp_bx(1) <<  " " << pelv_ori_c(1) <<std::endl;//<< " " << model_data2.oMf[RFcframe_id].translation()(0)<< " " << model_data2.oMf[LFcframe_id].translation()(0)   <<  " " << model_data2.oMf[RFcframe_id].translation()(2)<< " " << model_data2.oMf[LFcframe_id].translation()(2)   <<  " " << rfoot_mpc(2) << " " << lfoot_mpc(2) << " " << rd_.link_[COM_id].xpos(2)<< " " << com_z_init << " " << rfoot_ori_c(0)<< " " << rfoot_ori_c(1)<< " " << lfoot_ori_c(0)<< " " << lfoot_ori_c(1)<<std::endl;///<< mpc_cycle << " " << contactMode << " "  << rd_.q_(13) << " " << rd_.q_(14)<< " "<<pelv_ori_c(0) << " " << pelv_ori_c(1) << " " << ang_de(0) << " " << ang_de(2)<< " " << ang_de(4)<< " " <<desired_val[25] << " " << q_dm(4) <<" "<< model_data1.hg.angular()[0] << " " << model_data1.hg.angular()[1] << " "<< angd_(0) << " " << angd_(1) << " "<< ZMP_FT_law(0) << " "  << zmpx << std::endl;//" " << ZMP_FT_law(0) << " "  << zmpx<< " " << ZMP_FT_law(1) << " "  << zmpy<< " " << rd_.link_[COM_id].xpos(2)<<" " << com_z_init << " " << comd(2) << " " << model_data1.hg.angular()[0] << " " << model_data1.hg.angular()[1] << " "<< angd_(0) << " " << angd_(1) << " "<< rfoot_mpc(0)<< " " <<model_data2.oMf[RFcframe_id].translation()(0) + virtual_temp1_x<< " "<<  lfoot_mpc(0) << " " << model_data2.oMf[LFcframe_id].translation()(0) + virtual_temp1_x<<  std::endl;//<<  lfoot_mpc(2) << " " << model_data2.oMf[LFcframe_id].translation()(2) << " "<<  rfoot_mpc(2) << " " << model_data2.oMf[RFcframe_id].translation()(2)<< std::endl;//<< angd_(0) << " " << angd_(1) << std:::endl;//std::endl;//file[0] << mpc_cycle <<  " " <<walking_tick << " "<< contactMode << " "<< solved<< " " << qp_solved<< " " << virtual_temp2_x << " "<< ZMP_FT_law(0) << " " << ZMP(0)<<  " " << zmpx<< " " << zmp_bx(0) << " " << zmp_bx(1) << " "  << ZMP_FT_law(1) << " "<< zmpy << " " << rd_.link_[COM_id].xpos(1)<< " " << model_data1.hg.angular()[0] << " " << model_data1.hg.angular()[1] << " "<< angd_(0) << " " << angd_(1) << " " << comd(0) << " " << comd(1) << " " << rd_.link_[COM_id].v(0)<< " " << rd_.link_[COM_id].v(1)  << " " << rd_.link_[COM_id].xpos(0)<< " " << rd_.link_[COM_id].xpos(1) << " " << com_mpc[0]  << " " << com_mpc[1] <<  " " << rd_.link_[COM_id].xpos(2) << " " << mj_shm_->dis_check<< " " << rd_.q_(13) << " " << rd_.q_(14)<< " " << q_pinocchio_desired(20) << " " << q_pinocchio_desired(21)<<" " <<qp_result.segment<MODEL_DOF_VIRTUAL>(12)(19)<< " " << qp_result.segment<MODEL_DOF_VIRTUAL>(12)(20) <<std::endl;//rfoot_mpc(0)<< " " <<model_data2.oMf[RFcframe_id].translation()(0) + virtual_temp1_x <<" " <<lfoot_mpc(0)<< " " <<model_data2.oMf[LFcframe_id].translation()(0) + virtual_temp1_x << " " << rd_.link_[COM_id].xpos(2)<<  " "<<com_z_init  <<  std::endl;   
    }
    
    if(mpc_cycle <= controlwalk_time - 1)// && mpc_cycle <= 83)
    {    
        for (int i = 0; i < MODEL_DOF_VIRTUAL-6; i++)
        {  
            rd_.torque_desired[i] = tau_[i+6] + 1.0 * (rd_.pos_kp_v[i] * (q_pinocchio_desired1[i+7] - rd_.q_[i]) + rd_.pos_kv_v[i] * (qd_pinocchio_desired1[i+6]- rd_.q_dot_[i])); //qd_pinocchio_desired1_prev[i+6]
        }
    }
    else
    {  
        for (int i = 0; i < MODEL_DOF_VIRTUAL-6; i++)
        {  
            rd_.torque_desired[i] = 1.0 * (rd_.pos_kp_v[i] * (q_pinocchio_desired1[i+7] - rd_.q_[i]) + rd_.pos_kv_v[i] * (- rd_.q_dot_[i]));
        }
    }

    if (rd_.tc_.mode == 10)
    {
        if (rd_.tc_init)
        {
            //Initialize settings for Task Control!

            rd_.tc_init = false;
            std::cout << "cc mode 11" << std::endl;
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
   
            auto endTime1 = std::chrono::system_clock::now();
           
            if(walking_tick == 1)
            {
                //auto elapsed2 = std::chrono::duration_cast<std::chrono::microseconds>(endTime1 - startTime1);
                //mpc_latency += elapsed2.count();
            }
            else
            {
                mpc_latency = 0.0;
            }      

        }
        else if (rd_.tc_.walking_enable == 3.0 || rd_.tc_.walking_enable == 2.0)
        {
            //WBC::SetContact(rd_, 1, 1);
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
        //auto startTime = std::chrono::system_clock::now();
        //auto startTime1 = std::chrono::system_clock::now();
        //auto endTime1 = std::chrono::system_clock::now();
        mpc_cycle_int = (mpc_cycle -50) / 50;
        mpc_cycle_int1 = (mpc_cycle - 50) % 50;

        if (time_tick == false)
        {
            startTime = std::chrono::system_clock::now();
            time_tick = true;
            
        }              

        if (std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count() >= 500)
        {
            time_tick_next = true;
           // std::cout << "dd " << mpc_cycle << " " << walking_tick << " " << std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count()  << std::endl; 
            startTime = std::chrono::system_clock::now();
            
        }
        else
        {
            //std::cout << "dd22 " << mpc_cycle << " " << walking_tick << " " << std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count()  << std::endl; 
            //std::cout << "NOO" << std::endl;
        }

        if(time_tick_next == true)
        {
            time_tick_next = false;
            if (rd_.tc_.walking_enable == 1.0)
            {           
                if (wlk_on == false)
                {
                    wk_Hz = 2000;
                    wk_dt = 1 / wk_Hz;
                    controlwalk_time = 370;//660;//217;//360;

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
                        mpc_start_init = 4;
                        std::cout << "Start" << std::endl;
                    }
                }
            
                if(stateestimation == true)
                {  
                    for (int i = 0; i < 6; i ++)
                        state_init[i] = rd_.q_virtual_[i];

                    state_init[6] = rd_.q_virtual_[39];
                
                    for (int i = 6; i < 18; i ++)
                        state_init[i+1] = rd_.q_virtual_[i];

                    for (int i = 0; i < 18; i ++)
                        state_init[i+21] = rd_.q_dot_virtual_[i];
                
                    state_init[20] = rd_.q_virtual_[19];
                    state_init[21] = rd_.q_virtual_[20];

                    state_init[41] = rd_.link_[COM_id].xpos(0);
                    state_init[42] = rd_.link_[COM_id].v(0);//rd_.link_[COM_id].xpos(0);
                
                    state_init[45] = rd_.link_[COM_id].xpos(1);
                    state_init[46] = rd_.link_[COM_id].v(1);//rd_.link_[COM_id].xpos(0);
                
                
                    if(mpc_cycle == 0)
                    {
                        state_init[43] = rd_.link_[COM_id].xpos(0);
                        state_init[47] = rd_.link_[COM_id].xpos(1);
                        state_init[44] = 0.0;//model_data.hg.angular[1];//rd_.link_[COM_id].xpos(0);
                        state_init[48] = 0.0;//model_data.hg.angular[0];///rd_.link_[COM_id].xpos(0);
                    }
                    
                    stateestimation = false;
                    walk_start = true;
                    thread2_lock.lock();
                    state_init_mu = state_init;
                    thread2_lock.unlock();
                    mpc_start_init = 1;
                    std::cout << " state esimation" << std::endl;
                }
            
                if(walking_tick >= 0)
                {   
                    state_init[0] = rd_.q_virtual_[0]+virtual_temp_x;
                    state_init[1] = rd_.q_virtual_[1]+virtual_temp_y;
            
                    for (int i = 2; i < 6; i ++)
                        state_init[i] = rd_.q_virtual_[i];
                
                    state_init[6] = rd_.q_virtual_[39];
                
                    for (int i = 6; i < 18; i ++)
                        state_init[i+1] = rd_.q_virtual_[i];
                    state_init[19] = rd_.q_virtual_[19];
                    state_init[20] = rd_.q_virtual_[20];

                    for (int i = 0; i < 18; i ++)
                        state_init[i+21] = rd_.q_dot_virtual_[i];
                
                    state_init[39] =  rd_.q_dot_virtual_[19];
                    state_init[40] =  rd_.q_dot_virtual_[20];
                
                    state_init[41] = rd_.link_[COM_id].xpos(0)+virtual_temp_x;
                    state_init[45] = rd_.link_[COM_id].xpos(1)+virtual_temp_y;
                    state_init[42] = rd_.link_[COM_id].v(0);
                    state_init[46] = rd_.link_[COM_id].v(1);
                
                    state_init[43] = ZMP_FT_law(0)+virtual_temp_x;
                    state_init[47] = ZMP_FT_law(1)+virtual_temp_y;
                    
                    state_init[44] = model_data1.hg.angular()[1];
                    state_init[48] = model_data1.hg.angular()[0];

                    state_init[49] = rd_.link_[COM_id].xpos(2);//model_data1.hg.angular()[0];

                    thread2_lock.lock();
                    state_init_mu = state_init;
                    thread2_lock.unlock();   
                }

                if(statemachine == 1 || statemachine == 2 && mpc_cycle < controlwalk_time)
                {
                    if (walking_tick == 0)
                    {
                        qd_pinocchio_.setZero();
                    }  
                    if((statemachine == 1 && walking_tick_stop == true && walking_tick == 0))
                    {  
                        mpc_start_init = 2;
                        if (walking_tick == 0)
                        {
                            q_pinocchio_desired.head(19) = q_pinocchio.head(19);
                            q_pinocchio_desired1.head(19) = q_pinocchio.head(19);
                        
                            walking_tick_stop = false;
                        
                            if (contactMode == 1 && mpc_cycle <= 1)
                            {
                                foot_temp_y = rd_.link_[Right_Foot].xipos(2);
                                foot_temp_y = rd_.link_[Left_Foot].xipos(2);
                            }

                            if(upper_on == true)
                            {
                                std::cout << "mpc_cycle " << " : " << mpc_cycle << " " << walking_tick <<  " " << rfoot_mpc(2) << " " << lfoot_mpc(2) << " " << contactMode << std::endl;
                                qd_pinocchio.setZero();
                                thread1_lock.lock();
                                desired_val_mu = desired_val;
                                thread1_lock.unlock();
                                if(desired_val_mu[19] < -0.4000 || rd_.q_(13) < -0.4000)
                                {  
                                    std::cout << "Pitch over" << std::endl;
                                    if(desired_val_mu[39] < 0.0)
                                    {
                                        qd_pinocchio(19) = 0.0;
                                        upperd[0] = 0.0;
                                    }
                                    else
                                    {
                                        qd_pinocchio(19) = desired_val_mu[39];
                                        upperd[0] = desired_val_mu[39];
                                    }
                                }
                                else if(desired_val_mu[19] > 0.4000 || rd_.q_(13) > 0.4000)
                                {  
                                    std::cout << "Pitch over" << std::endl;
                                    if(desired_val_mu[39] > 0.0)
                                    {
                                        qd_pinocchio(19) = 0.0;
                                        upperd[0] = 0.0;
                                    }
                                    else
                                    {
                                        qd_pinocchio(19) = desired_val_mu[39];
                                        upperd[0] = desired_val_mu[39];
                                    }
                                }
                                else
                                {  
                                    qd_pinocchio(19) = desired_val_mu[39];
                                    upperd[0] = desired_val_mu[39];
                                }

                                if(desired_val_mu[20] < -0.4000 || rd_.q_(14) < -0.4000)
                                {
                                    std::cout << "Roll over" << std::endl;
                                    if(desired_val_mu[40] < 0.0)
                                    {
                                        qd_pinocchio(20) = 0.0;
                                        upperd[1] = 0.0;
                                    }
                                    else
                                    {
                                        qd_pinocchio(20) = desired_val_mu[40];
                                        upperd[1] = desired_val_mu[40];
                                    }
                                }
                                else if(desired_val_mu[20] > 0.4000 || rd_.q_(14) > 0.4000)
                                {
                                    std::cout << "Roll over" << std::endl;
                                    if(desired_val_mu[40] > 0.0)
                                    {
                                        qd_pinocchio(20) = 0.0;
                                        upperd[1] = 0.0;
                                    }
                                    else
                                    {
                                        qd_pinocchio(20) = desired_val_mu[40];
                                        upperd[1] = desired_val_mu[40];
                                    }  
                                }
                                else
                                {
                                    qd_pinocchio(20) = desired_val_mu[40];
                                    upperd[1] = desired_val_mu[40];
                                }
                            }
                            
                            //std::cout << "JOint " << rd_.q_(13) << " " << rd_.q_(14) << "  " << qd_pinocchio(19) << " " << qd_pinocchio(20) << " " << desired_val_mu[39] << " " << desired_val_mu[40]<< " " << q_pinocchio_desired(20) << " " << q_pinocchio_desired(21) <<std::endl;//DyrosMath::rot2Euler(rd_.link_[Pelvis].rotm)(0) << " " << DyrosMath::rot2Euler(rd_.link_[Pelvis].rotm)(1) << " " << q_pinocchio_desired(20) << " " << q_pinocchio_desired(21) << std::endl;
                                
                            
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

                            comd[0] = desired_val_mu[42];
                            comd[1] = desired_val_mu[46];
                            comd[2] = 0.0;

                            angm[0] = desired_val_mu[48];
                            angm[1] = desired_val_mu[44];

                            ZMPx_prev = zmp_mpcx;
                            ZMPy_prev = zmp_mpcy;
                        
                            zmp_mpcx = desired_val_mu[43] - virtual_temp1_x;
                            zmp_mpcy = desired_val_mu[47] - virtual_temp1_y;  

                            for(int i = 0; i < 18; i++)
                            {
                                q_desireddot(i) = desired_val_mu[i+21];
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

                            com_mpc[0] = rd_.link_[COM_id].xpos(0) + comd[0] * 0.02;//desired_val[41] - virtual_temp_x;
                            com_mpc[1] = rd_.link_[COM_id].xpos(1) + comd[1] * 0.02;//desired_val[45] - virtual_temp_y;
                        
                            
                            if(mpc_cycle == 0)
                            {
                                rfootd.setZero();
                                lfootd.setZero();
                            }
                            else
                            {  
                                if(mpc_cycle <= 49)
                                {
                                    rfootd[0] = (RF_matrix(mpc_cycle,0)-RF_matrix(mpc_cycle-1,0))/0.02;
                                    rfootd[1] = (RF_matrix(mpc_cycle,1)-RF_matrix(mpc_cycle-1,1))/0.02;
                                    rfootd[2] = (RF_matrix(mpc_cycle,2)-RF_matrix(mpc_cycle-1,2))/0.02;

                                    lfootd[0] = (LF_matrix(mpc_cycle,0)-LF_matrix(mpc_cycle-1,0))/0.02;
                                    lfootd[1] = (LF_matrix(mpc_cycle,1)-LF_matrix(mpc_cycle-1,1))/0.02;
                                    lfootd[2] = (LF_matrix(mpc_cycle,2)-LF_matrix(mpc_cycle-1,2))/0.02;
                                }
                                else if(mpc_cycle <= 99)
                                {
                                    rfootd[0] = (RF_matrix_ssp2(mpc_cycle-49,0)-RF_matrix_ssp2(mpc_cycle-50,0))/0.02;
                                    rfootd[1] = (RF_matrix_ssp2(mpc_cycle-49,1)-RF_matrix_ssp2(mpc_cycle-50,1))/0.02;
                                    rfootd[2] = (RF_matrix_ssp2(mpc_cycle-49,2)-RF_matrix_ssp2(mpc_cycle-50,2))/0.02;

                                    lfootd[0] = (LF_matrix_ssp2(mpc_cycle-49,0)-LF_matrix_ssp2(mpc_cycle-50,0))/0.02;
                                    lfootd[1] = (LF_matrix_ssp2(mpc_cycle-49,1)-LF_matrix_ssp2(mpc_cycle-50,1))/0.02;
                                    lfootd[2] = (LF_matrix_ssp2(mpc_cycle-49,2)-LF_matrix_ssp2(mpc_cycle-50,2))/0.02;
                                }
                                else if(mpc_cycle <= 149)
                                {
                                    rfootd[0] = (RF_matrix_ssp1(mpc_cycle-99,0)-RF_matrix_ssp1(mpc_cycle-100,0))/0.02;
                                    rfootd[1] = (RF_matrix_ssp1(mpc_cycle-99,1)-RF_matrix_ssp1(mpc_cycle-100,1))/0.02;
                                    rfootd[2] = (RF_matrix_ssp1(mpc_cycle-99,2)-RF_matrix_ssp1(mpc_cycle-100,2))/0.02;

                                    lfootd[0] = (LF_matrix_ssp1(mpc_cycle-99,0)-LF_matrix_ssp1(mpc_cycle-100,0))/0.02;
                                    lfootd[1] = (LF_matrix_ssp1(mpc_cycle-99,1)-LF_matrix_ssp1(mpc_cycle-100,1))/0.02;
                                    lfootd[2] = (LF_matrix_ssp1(mpc_cycle-99,2)-LF_matrix_ssp1(mpc_cycle-100,2))/0.02;
                                }
                                else if(mpc_cycle >= 150)
                                {
                                    if(mpc_cycle_int % 2 == 0)
                                    {
                                        rfootd[0] = (RF_matrix_ssp2(mpc_cycle_int1 + 1,0)-RF_matrix_ssp2(mpc_cycle_int1,0))/0.02;
                                        rfootd[1] = (RF_matrix_ssp2(mpc_cycle_int1 + 1,1)-RF_matrix_ssp2(mpc_cycle_int1,1))/0.02;
                                        rfootd[2] = (RF_matrix_ssp2(mpc_cycle_int1 + 1,2)-RF_matrix_ssp2(mpc_cycle_int1,2))/0.02;

                                        lfootd[0] = (LF_matrix_ssp2(mpc_cycle_int1 + 1,0)-LF_matrix_ssp2(mpc_cycle_int1,0))/0.02;
                                        lfootd[1] = (LF_matrix_ssp2(mpc_cycle_int1 + 1,1)-LF_matrix_ssp2(mpc_cycle_int1,1))/0.02;
                                        lfootd[2] = (LF_matrix_ssp2(mpc_cycle_int1 + 1,2)-LF_matrix_ssp2(mpc_cycle_int1,2))/0.02;
                                    }
                                    else
                                    {
                                        rfootd[0] = (RF_matrix_ssp1(mpc_cycle_int1 + 1,0)-RF_matrix_ssp1(mpc_cycle_int1,0))/0.02;
                                        rfootd[1] = (RF_matrix_ssp1(mpc_cycle_int1 + 1,1)-RF_matrix_ssp1(mpc_cycle_int1,1))/0.02;
                                        rfootd[2] = (RF_matrix_ssp1(mpc_cycle_int1 + 1,2)-RF_matrix_ssp1(mpc_cycle_int1,2))/0.02;

                                        lfootd[0] = (LF_matrix_ssp1(mpc_cycle_int1 + 1,0)-LF_matrix_ssp1(mpc_cycle_int1,0))/0.02;
                                        lfootd[1] = (LF_matrix_ssp1(mpc_cycle_int1 + 1,1)-LF_matrix_ssp1(mpc_cycle_int1,1))/0.02;
                                        lfootd[2] = (LF_matrix_ssp1(mpc_cycle_int1 + 1,2)-LF_matrix_ssp1(mpc_cycle_int1,2))/0.02;
                                    }
                                }
                            }
                            
                        }
                    }

                    if(statemachine == 2 && walking_tick > 3)
                        mpc_start_init = 3;

                    KK_temp = 0.0;
                    if ((walking_tick == 0) && (walking_tick_stop == true))
                    {
                        KK_temp = 0.5;
                        //qd_pinocchio_desired1.setZero();
                        //qdd_pinocchio_desired1.setZero();
                        
                    }
                    else
                    {
                        if(walking_tick_stop == false && mpc_cycle < controlwalk_time)
                        {
                            KK_temp = 1.0;
                            
                            comdt_(0) = (comd(0) * walking_tick + comprev(0) * (40 -walking_tick))/40;
                            comdt_(1) = (comd(1) * walking_tick + comprev(1) * (40 -walking_tick))/40;

                            comd_(0) = comdt_(0)+ 0.0 * (comdt_(0) - rd_.link_[COM_id].v(0)) + 0.0 * (com_mpc[0] - rd_.link_[COM_id].xpos(0));
                            comd_(1) = comdt_(1)+ 0.0 * (comdt_(1) - rd_.link_[COM_id].v(1)) + 0.0 * (com_mpc[1] - rd_.link_[COM_id].xpos(1));
                            comd_(2) = comd(2) + 50.0 * (com_z_init - rd_.link_[COM_id].xpos(2));

                            angd_(0) = (angm(0) * walking_tick + angm_prev(0) * (40 -walking_tick))/40;
                            angd_(1) = (angm(1) * walking_tick + angm_prev(1) * (40 -walking_tick))/40;

                            rfoot_ori.setZero();
                            lfoot_ori.setZero();

                            rfoot_ori_c = DyrosMath::rot2Euler(rd_.link_[Right_Foot].rotm);
                            lfoot_ori_c = DyrosMath::rot2Euler(rd_.link_[Left_Foot].rotm);
                            pelv_ori_c = DyrosMath::rot2Euler(rd_.link_[Pelvis].rotm);

                            rfootd1 = rfootd;
                            lfootd1 = lfootd;
                            double gain_xz = 10.0;
                            double gain_ori = 5.00;
                        
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

                                rfootd1(1) = 0.0;
                                lfootd1(1) = 0.0;
                                if(rfoot_mpc(2)>0.0)
                                {
                                    rfoot_ori(0) = gain_ori * (-rfoot_ori_c(0));
                                    rfoot_ori(1) = gain_ori * (-rfoot_ori_c(1));
                                    rfoot_ori(2) = 1.0 * (-rfoot_ori_c(2));
                                    rfootd1(2) = rfootd1(2)+ gain_xz * (rfoot_mpc(2) - rd_.link_[Right_Foot].xipos(2) + foot_temp_y);
                                    rfootd1(1) = rfootd1(1) + 10.00 * (-0.1025 - rd_.link_[Right_Foot].xipos(1)- virtual_temp1_y);
                                    rfootd1(0) = rfootd1(0)+ gain_xz * (rfoot_mpc(0) - rd_.link_[Right_Foot].xipos(0) - 0.0378);
                                }
                                else if(lfoot_mpc(2)>0.0)
                                {
                                    lfoot_ori(0) = gain_ori * (-lfoot_ori_c(0));
                                    lfoot_ori(1) = gain_ori * (-lfoot_ori_c(1));
                                    lfoot_ori(2) = 1.0 * (-lfoot_ori_c(2));
                                    lfootd1(2) = lfootd1(2)+ gain_xz * (lfoot_mpc(2) - rd_.link_[Left_Foot].xipos(2) + foot_temp_y);
                                    lfootd1(1) = lfootd1(1) + 10.00 * (0.1025 - rd_.link_[Left_Foot].xipos(1)- virtual_temp1_y);
                                    lfootd1(0) = lfootd1(0)+ gain_xz * (lfoot_mpc(0) - rd_.link_[Left_Foot].xipos(0) - 0.0378);
                                }  
                                else
                                {
                                    // rfootd1(2) = rfootd1(2)+ gain_xz/2 * (rfoot_mpc(2) - rd_.link_[Right_Foot].xipos(2) + foot_temp_y);
                                // lfootd1(2) = lfootd1(2)+ gain_xz/2 * (lfoot_mpc(2) - rd_.link_[Left_Foot].xipos(2) + foot_temp_y);
                                
                                    rfoot_mpc(2) = 0.0;
                                    lfoot_mpc(2) = 0.0;
                                }
                            
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
                                
                                rfootd1(1) = 0.0;
                                lfootd1(1) = 0.0;

                                if(rfoot_mpc(2)>0.0)
                                {
                                    rfoot_ori(0) = gain_ori * (-rfoot_ori_c(0));
                                    rfoot_ori(1) = gain_ori * (-rfoot_ori_c(1));
                                    rfoot_ori(2) = 1.0 * (-rfoot_ori_c(2));
                                    rfootd1(2) = rfootd1(2)+ gain_xz * (rfoot_mpc(2) - rd_.link_[Right_Foot].xipos(2) + foot_temp_y);
                                    rfootd1(1) = rfootd1(1)+ 10.00 * (-0.1025 - rd_.link_[Right_Foot].xipos(1)- virtual_temp1_y);
                                    rfootd1(0) = rfootd1(0)+ gain_xz * (rfoot_mpc(0) - virtual_temp1_x- rd_.link_[Right_Foot].xipos(0) - 0.0378);
                                }
                                else if(lfoot_mpc(2)>0.0)
                                {
                                    lfoot_ori(0) = gain_ori * (-lfoot_ori_c(0));
                                    lfoot_ori(1) = gain_ori * (-lfoot_ori_c(1));
                                    lfoot_ori(2) = 1.0 * (-lfoot_ori_c(2));
                                    lfootd1(2) = lfootd1(2)+ gain_xz * (lfoot_mpc(2) - rd_.link_[Left_Foot].xipos(2) + foot_temp_y);
                                    lfootd1(1) = lfootd1(1) + 10.00 * (0.1025 - rd_.link_[Left_Foot].xipos(1) - virtual_temp1_y);
                                    lfootd1(0) = lfootd1(0)+ gain_xz * (lfoot_mpc(0) - virtual_temp1_x - rd_.link_[Left_Foot].xipos(0) - 0.0378);
                                }
                                else
                                {
                                // rfootd1(2) = rfootd1(2)+ gain_xz/2 * (rfoot_mpc(2) - rd_.link_[Right_Foot].xipos(2) + foot_temp_y);
                                // lfootd1(2) = lfootd1(2)+ gain_xz/2 * (lfoot_mpc(2) - rd_.link_[Left_Foot].xipos(2) + foot_temp_y);
                                
                                    rfoot_mpc(2) = 0.0;
                                    lfoot_mpc(2) = 0.0;
                                }
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
                                
                                rfootd1(1) = 0.0;
                                lfootd1(1) = 0.0;
                                if(rfoot_mpc(2)>0.0)
                                {
                                    rfoot_ori(0) = gain_ori * (-rfoot_ori_c(0));
                                    rfoot_ori(1) = gain_ori * (-rfoot_ori_c(1));
                                    rfoot_ori(2) = 1.0 * (-rfoot_ori_c(2));
                                    rfootd1(2) = rfootd1(2)+ gain_xz * (rfoot_mpc(2) - rd_.link_[Right_Foot].xipos(2) + foot_temp_y);
                                    rfootd1(1) = rfootd1(1)+ 10.00 * (-0.1025 - rd_.link_[Right_Foot].xipos(1)- virtual_temp1_y);
                                    rfootd1(0) = rfootd1(0)+ gain_xz * (rfoot_mpc(0) - virtual_temp1_x- rd_.link_[Right_Foot].xipos(0) - 0.0378);
                                }
                                else if(lfoot_mpc(2)>0.0)
                                {
                                    lfoot_ori(0) = gain_ori * (-lfoot_ori_c(0));
                                    lfoot_ori(1) = gain_ori * (-lfoot_ori_c(1));
                                    lfoot_ori(2) = 1.0 * (-lfoot_ori_c(2));
                                    lfootd1(2) = lfootd1(2)+ gain_xz * (lfoot_mpc(2) - rd_.link_[Left_Foot].xipos(2) + foot_temp_y);
                                    lfootd1(1) = lfootd1(1) + 10.00 * (0.1025 - rd_.link_[Left_Foot].xipos(1)- virtual_temp1_y);
                                    lfootd1(0) = lfootd1(0)+ gain_xz * (lfoot_mpc(0) - virtual_temp1_x - rd_.link_[Left_Foot].xipos(0) - 0.0378);
                                }
                                else
                                {
                                // rfootd1(2) = rfootd1(2)+ gain_xz/2 * (rfoot_mpc(2) - rd_.link_[Right_Foot].xipos(2) + foot_temp_y);
                                // lfootd1(2) = lfootd1(2)+ gain_xz/2 * (lfoot_mpc(2) - rd_.link_[Left_Foot].xipos(2) + foot_temp_y);
                                
                                    rfoot_mpc(2) = 0.0;
                                    lfoot_mpc(2) = 0.0;
                                }
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
                                    
                                    rfootd1(1) = 0.0;
                                    lfootd1(1) = 0.0;
                                    if(rfoot_mpc(2)>0.0)
                                    {
                                        rfoot_ori(0) = 3.0 * (-rfoot_ori_c(0));
                                        rfoot_ori(1) = 3.0 * (-rfoot_ori_c(1));
                                        rfoot_ori(2) = 1.0 * (-rfoot_ori_c(2));
                                        rfootd1(2) = rfootd1(2)+ gain_xz * (rfoot_mpc(2) - rd_.link_[Right_Foot].xipos(2) + foot_temp_y);
                                        rfootd1(1) = rfootd1(1)+ 10.00 * (-0.1025 - rd_.link_[Right_Foot].xipos(1)- virtual_temp1_y);
                                        rfootd1(0) = rfootd1(0)+ gain_xz * (rfoot_mpc(0) - virtual_temp1_x- rd_.link_[Right_Foot].xipos(0) - 0.0378);
                                    }
                                    else if(lfoot_mpc(2)>0.0)
                                    {
                                        lfoot_ori(0) = 3.0 * (-lfoot_ori_c(0));
                                        lfoot_ori(1) = 3.0 * (-lfoot_ori_c(1));
                                        lfoot_ori(2) = 1.0 * (-lfoot_ori_c(2));
                                        lfootd1(2) = lfootd1(2)+ gain_xz * (lfoot_mpc(2) - rd_.link_[Left_Foot].xipos(2) + foot_temp_y);
                                        lfootd1(1) = lfootd1(1) + 10.00 * (0.1025 - rd_.link_[Left_Foot].xipos(1) - virtual_temp1_y);
                                        lfootd1(0) = lfootd1(0)+ gain_xz * (lfoot_mpc(0) - virtual_temp1_x - rd_.link_[Left_Foot].xipos(0) - 0.0378);
                                    }
                                    else
                                    {
                                        rfootd1(2) = rfootd1(2)+ gain_xz/2 * (rfoot_mpc(2) - rd_.link_[Right_Foot].xipos(2) + foot_temp_y);
                                        lfootd1(2) = lfootd1(2)+ gain_xz/2 * (lfoot_mpc(2) - rd_.link_[Left_Foot].xipos(2) + foot_temp_y);
                                
                                        rfoot_mpc(2) = 0.0;
                                        lfoot_mpc(2) = 0.0;
                                    }
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

                                    rfootd1(1) = 0.0;
                                    lfootd1(1) = 0.0;
                                    if(rfoot_mpc(2)>0.0)
                                    {
                                        rfoot_ori(0) = 3.0 * (-rfoot_ori_c(0));
                                        rfoot_ori(1) = 3.0 * (-rfoot_ori_c(1));
                                        rfoot_ori(2) = 1.0 * (-rfoot_ori_c(2));
                                        rfootd1(2) = rfootd1(2)+ gain_xz * (rfoot_mpc(2) - rd_.link_[Right_Foot].xipos(2) + foot_temp_y);
                                        rfootd1(1) = rfootd1(1)+ 10.00 * (-0.1025 - rd_.link_[Right_Foot].xipos(1)- virtual_temp1_y);
                                        rfootd1(0) = rfootd1(0)+ gain_xz * (rfoot_mpc(0) - virtual_temp1_x- rd_.link_[Right_Foot].xipos(0) - 0.0378);
                        
                                    }
                                    else if(lfoot_mpc(2)>0.0)
                                    {
                                        lfoot_ori(0) = 3.0 * (-lfoot_ori_c(0));
                                        lfoot_ori(1) = 3.0 * (-lfoot_ori_c(1));
                                        lfoot_ori(2) = 1.0 * (-lfoot_ori_c(2));
                                        lfootd1(2) = lfootd1(2)+ gain_xz * (lfoot_mpc(2) - rd_.link_[Left_Foot].xipos(2) + foot_temp_y);
                                        lfootd1(1) = lfootd1(1) + 10.00 * (0.1025 - rd_.link_[Left_Foot].xipos(1)- virtual_temp1_y);
                                        lfootd1(0) = lfootd1(0)+ gain_xz * (lfoot_mpc(0) - virtual_temp1_x - rd_.link_[Left_Foot].xipos(0) - 0.0378);
                                    }
                                    else
                                    {
                                        rfootd1(2) = rfootd1(2)+ gain_xz/2 * (rfoot_mpc(2) - rd_.link_[Right_Foot].xipos(2) + foot_temp_y);
                                        lfootd1(2) = lfootd1(2)+ gain_xz/2 * (lfoot_mpc(2) - rd_.link_[Left_Foot].xipos(2) + foot_temp_y);
                                
                                        rfoot_mpc(2) = 0.0;
                                        lfoot_mpc(2) = 0.0;
                                    }
                                }
                            }


                            momentumControl(rd_, comd_, angd_, rfootd1, lfootd1, upperd, rfoot_ori, lfoot_ori);
                            qd_pinocchio.segment<18>(0) = q_dm;
                        
                            qdd_pinocchio_desired1 = ((qd_pinocchio - qd_pinocchio_prev)/0.02);
                            qd_pinocchio_prev = qd_pinocchio;


                            ZMPx_test = (zmp_mpcx * walking_tick + ZMPx_prev *(40-walking_tick))/40;
                            ZMPy_test = (zmp_mpcy * walking_tick + ZMPy_prev *(40-walking_tick))/40;
                            
                            if(mpc_cycle == 0)
                                q_pinocchio_desired = pinocchio::integrate(model, q_pinocchio_desired, qd_pinocchio * 0.0005);
                            else
                                q_pinocchio_desired = pinocchio::integrate(model, q_pinocchio_desired, qd_pinocchio * 0.0005);
                        
                            rfoot_ori.setZero();
                            lfoot_ori.setZero();
                            q_pinocchio_desired1 = q_pinocchio_desired;
                            qd_pinocchio_desired1 = qd_pinocchio;
                        }
                    }

                    if(walking_tick == 1)
                    {
                        if(mpc_cycle <= controlwalk_time)
                            mpc_start_init = 1;
                    }
                
                    if(walking_tick == 40)
                    {
                        walking_tick = 0;
                        walking_tick_stop = true;
                        mpc_cycle = mpc_cycle + 1;
                    }

                    if (walking_tick_stop == false)
                        walking_tick = walking_tick + 1;

                    //auto endTime = std::chrono::system_clock::now();
                }
            }
        }
        endTime = std::chrono::system_clock::now();
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
        ///*
        MatrixXd J1, H1, A1;
        VectorXd X1, g1, lb1, ub1, lbA1, ubA1;
        X1.setZero(constraint_size2);
        J1.setZero(constraint_size2, variable_size2);
        H1.setZero(variable_size2, variable_size2);
        A1.setZero(constraint_size2, variable_size2);
        g1.setZero(variable_size2);
        lbA1.setZero(constraint_size2);
        ubA1.setZero(constraint_size2);
           
        lb1.setConstant(variable_size2, -100000);
        ub1.setConstant(variable_size2, 100000);
    
        H1(3,3) = 5.0;
        H1(4,4) = 5.0;
        
        /*
        if(pelv_ori_c(1) != 0)
            H1(4,4) = 0.1/abs(pelv_ori_c(1));
        if(pelv_ori_c(0) != 1)
            H1(3,3) = 0.1/abs(pelv_ori_c(0));

        if(H1(3,3) > 10.0)
            H1(3,3) = 10.0;
        if(H1(3,3) < -10.0)
            H1(3,3) = -10.0;

        if(H1(4,4) > 10.0)
            H1(4,4) = 10.0;
        if(H1(4,4) < -10.0)
            H1(4,4) = -10.0;
        */
        H1(5,5) = 1.0;

        MOMX = CMM.block(3,19,2,2) * upperd; 
        COMX = Robot.link_[COM_id].Jac().block(0,19,3,2) * upperd;

        J.block(0,0,3,18) = Robot.link_[COM_id].Jac().block(0,0,3,18);
        J.block(3,0,6,18) = Robot.link_[Right_Foot].Jac().block(0,0,6,18);
        J.block(9,0,6,18) = Robot.link_[Left_Foot].Jac().block(0,0,6,18);
        J.block(15,0,1,18) = CMM.block(4,0,1,18);
        J.block(16,0,1,18) = CMM.block(3,0,1,18);
   
        X.segment<3>(0) = comd - COMX;
        X.segment<3>(3) = rfootd;
        X.segment<3>(6) = rfootori;
        X.segment<3>(9) = lfootd;
        X.segment<3>(12) = lfootori;
        X(15) = (ang_ - MOMX)(1);
        X(16) = (ang_ - MOMX)(0);
        //J.block(3,0,2,18) = CMM.block(3,0,2,18);
        //X.segment<2>(3) = ang_ - MOMX;

        H1 = H1 + CMM.block(3,0,2,18).transpose() * CMM.block(3,0,2,18);
        g1 = g1 - CMM.block(3,0,2,18).transpose() * (ang_ - MOMX);
        
        A1 = J;
        lbA1 = X;
        ubA1 = X;
        lbA1(15) = lbA1(15)-1.5;
        ubA1(15) = ubA1(15)+1.5;
        lbA1(16) = lbA1(16)-1.5;
        ubA1(16) = ubA1(16)+1.5;

        qp_momentum_control.UpdateMinProblem(H1, g1);
        qp_momentum_control.UpdateSubjectToAx(A1, lbA1, ubA1);
        qp_momentum_control.UpdateSubjectToX(lb1, ub1);
       
        qp_solved = qp_momentum_control.SolveQPoases(100, qp_result);
        
        if(qp_solved == true)
            q_dm = qp_result;
        else
            q_dm.setZero();

        ang_de(0) = ang_(0)- MOMX(0);
        ang_de(1) = ang_(1)- MOMX(1);
        ang_de(2) = (CMM.block(3,0,2,18) * q_dm)(0);
        ang_de(3) = (CMM.block(3,0,2,18) * q_dm)(1);
        ang_de(4) = (CMM.block(3,0,2,18) * q_desireddot)(0);
        ang_de(5) = (CMM.block(3,0,2,18) * q_desireddot)(1);
        
        
        
        /*
        MatrixXd J1, H1, A1;
        VectorXd X1, g1, lb1, ub1, lbA1, ubA1;
        X1.setZero(constraint_size2);
        J1.setZero(constraint_size2, variable_size2);
        H1.setZero(variable_size2, variable_size2);
        A1.setZero(constraint_size2, variable_size2);
        g1.setZero(variable_size2);
        lbA1.setZero(constraint_size2);
        ubA1.setZero(constraint_size2);
           
        lb1.setConstant(variable_size2, -100000);
        ub1.setConstant(variable_size2, 100000);
       
        H1(3,3) = 1;
        H1(4,4) = 10;
        H1(5,5) = 1;
       
        MOMX = CMM.block(3,19,2,2) * upperd; 
        COMX = Robot.link_[COM_id].Jac().block(0,19,3,2) * upperd;

        J.block(0,0,3,18) = Robot.link_[COM_id].Jac().block(0,0,3,18);
        J.block(5,0,6,18) = Robot.link_[Right_Foot].Jac().block(0,0,6,18);
        J.block(11,0,6,18) = Robot.link_[Left_Foot].Jac().block(0,0,6,18);
   
        X.segment<3>(0) = comd - COMX;
        X.segment<3>(5) = rfootd;
        X.segment<3>(8) = rfootori;
        X.segment<3>(11) = lfootd;
        X.segment<3>(14) = lfootori;
        J.block(3,0,2,18) = CMM.block(3,0,2,18);
        X.segment<2>(3) = ang_ - MOMX;

        //H1 = H1 + CMM.block(3,0,2,18).transpose() * CMM.block(3,0,2,18);
        //g1 = g1 + CMM.block(3,0,2,18).transpose() * (ang_ - MOMX);
        
        
        A1 = J;
        lbA1 = X;
        ubA1 = X;

        qp_momentum_control.UpdateMinProblem(H1, g1);
        qp_momentum_control.UpdateSubjectToAx(A1, lbA1, ubA1);
        qp_momentum_control.UpdateSubjectToX(lb1, ub1);
       
        qp_solved = qp_momentum_control.SolveQPoases(100, qp_result);
        
        if(qp_solved == true)
            q_dm = qp_result;
        else
            q_dm.setZero();
        
        ang_de(0) = MOMX(0);
        ang_de(1) = MOMX(1);
        ang_de(2) = (CMM.block(3,0,2,18) * q_dm)(0);
        ang_de(3) = (CMM.block(3,0,2,18) * q_dm)(1);
        ang_de(4) = ang_(0);
        ang_de(5) = ang_(1);
        */
    }
}

void CustomController::zmpControl(RobotData &Robot)
{
    std::cout << "ZMP";
}