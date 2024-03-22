#include <vector>
#include <array>
#include <string>
#include <chrono>
#include <thread>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <std_msgs/String.h>
#include "math_type_define.h"
#include "walking.h"
#include "wholebody_functions.h"
#include <sys/socket.h>
#include <arpa/inet.h>
#include <sys/shm.h>
#include <sys/types.h>
#include <sys/ipc.h>

#include <future>


class CustomController : virtual public WalkingController
{
public:
    CustomController(RobotData &rd);
    
    const std::string FILE_NAMES[2] =
        {
            ///change this directory when you use this code on the other computer///
            "/home/jhk/data/walking/0_tocabi_.txt",
            "/home/jhk/data/walking/1_tocabi_.txt",
        };

    std::fstream file[2];
    Eigen::VectorQd getControl();

    //void taskCommandToCC(TaskCommand tc_);

    void computeSlow();
    void computeFast();
    void computePlanner();
    void copyRobotData(RobotData &rd_l);
    void GuiCommandCallback(const std_msgs::StringConstPtr &msg);
    void momentumControl(RobotData &Robot, Eigen::Vector3d comd,  Eigen::Vector2d ang_, Eigen::Vector3d rfootd, Eigen::Vector3d lfootd, Eigen::Vector2d upperd, Eigen::Vector3d rfootori, Eigen::Vector3d lfootori);
    void zmpControl(RobotData &rd_l);
    void dspForceControl(RobotData &Robot, double alpha);

    void proc_recv();
    void proc_recv1();

    std::future<void> solverx;
    std::future<void> solvery;

    bool mpc_s = false;

    RobotData &rd_;
    RobotData rd_cc_;
    
    std::atomic<double> lfootz;
    std::atomic<double> rfootz;

    std::atomic<int> mpc_start_init;
    std::atomic<int> statemachine;

    int LFjoint_id, RFjoint_id, RFframe_id, LFframe_id, RFcframe_id, LFcframe_id;

    CQuadraticProgram QP_m;

    //pinocchiio
    Eigen::VectorQd q_;
    Eigen::VectorQd qdot, qddot, qdot_, qddot_;
    Eigen::MatrixXd CMM;
    Eigen::MatrixQQd Cor_;
    Eigen::MatrixVQVQd M_;
    Eigen::VectorQd G_;
    Eigen::VectorVQd q_dot_virtual_lpf_;
    Eigen::VectorXd q_dm; 
    Eigen::VectorXd qddot_command; 
    Eigen::VectorXd qddot_command_prev; 
    Eigen::VectorXd qdot_command; 
    Eigen::VectorXd q_command; 

    bool walking_tick_stop = false;

    bool mob_start = false;
    bool control_start = false;
    bool stateestimation = false;
    bool stateestimation_mode = false;
    int stateestimation_count = 0;
    
    //Contact Redistribution
    Eigen::VectorQd TorqueContact;
    Eigen::VectorQd TorqueGrav;
    Eigen::VectorQVQd q_pinocchio;
    Eigen::VectorQVQd q_pinocchio_desired;
    Eigen::VectorQVQd q_pinocchio_desired2;
    Eigen::VectorQVQd q_pinocchio_desired1;
    Eigen::VectorVQd qd_pinocchio_desired1;
    Eigen::VectorVQd qd_pinocchio_desired1_prev;
    Eigen::VectorVQd qd_pinocchio_prev;
    Eigen::VectorVQd qdd_pinocchio_desired1;
    Eigen::VectorVQd qdd_pinocchio_desired1_;
    Eigen::VectorXd qp_result, qp_result_prev;

    double com_z_init;

    Eigen::MatrixXd RFj, LFj, RFdj, LFdj;
    MatrixXd G_temp;
    Vector12d g_temp;
            
    
    Eigen::Vector3d rfoot_mpc, lfoot_mpc;

    int solved = 0;
    double vector_tes = 0;
        
    int control_time = 0;

    //temp
    Eigen::VectorQVQd q_pinocchio_desired_prev;
    Eigen::VectorVQd q_dot_virtual_11;

    Eigen::Vector2d COM_prev;
    Eigen::Vector2d COM_vel;

    Eigen::Vector3d COM_prev1;

    bool cod_first = false;
    double mpc_latency;


    ros::Subscriber gui_sub_;
    ros::CallbackQueue queue_cc_;
    double rate;
    ros::NodeHandle nh;

    //MPC
    std::atomic<bool> wlk_on;
    std::atomic<bool> mpc_on;

    double debug_temp;
    bool debug = false;
    double com_alpha;
    int cycle = 0;
    double Ts = 0.01;
    double KK_temp = 0.0;

    bool mpcxsol = true;
    bool mpcysol = true;

    Eigen::Vector2d virtual_temp;
    Eigen::Vector2d virtual_temp1;
    Eigen::Vector2d virtual_temp2;
    Eigen::Vector2d foot_temp;

    int nx_;
    int nu_;
    double timeHorizon = 1.0;
    size_t K;

    int socket_send, socket_receive;
    
    double H_temp_22;

    int dist;

    std::atomic<int> mpc_cycle;
    std::atomic<int> mpc_cycle_prev;
    Eigen::Vector3d rfootd, lfootd,rfootd1, lfootd1, comd, com_mpc,vjoint_prev, vjoint_dot, comprev;
    Eigen::Vector2d angm, angm_prev, upperd, comdt_;

    Eigen::VectorQd joint_prev, jointdot;
    bool state_init_ = true;
    Eigen::VectorVQd qd_pinocchio;
    Eigen::VectorVQd qd_pinocchio_;
    Eigen::Vector2d ZMP_FT_law;

   int mpc_cycle_int = 0;
   int mpc_cycle_int1 = 0;
   Eigen::Vector6d ang_de;


   bool upper_on;
   bool state_esti = false;
    //WholebodyController &wbc_;
    //TaskCommand tc;

    Eigen::VectorXd q_desired;
    Eigen::VectorXd q_desireddot;
    int aaa = 0;
    bool qp_solved;
    Eigen::MatrixXd RF_matrix;
    Eigen::MatrixXd LF_matrix;
    Eigen::MatrixXd ZMP_bound_matrix;
    Eigen::MatrixXd RF_matrix_ssp2;
    Eigen::MatrixXd LF_matrix_ssp2;
    Eigen::MatrixXd RF_matrix_ssp1;
    Eigen::MatrixXd LF_matrix_ssp1;
    Eigen::MatrixXd ZMP_bound_matrix_ssp2;
    Eigen::Vector3d ZMP_measured;
    Eigen::VectorXd COMX, MOMX;
    Eigen::Vector3d comd_;
    Eigen::Vector2d angd_;
    Eigen::Vector2d angd_1;
    Eigen::Vector4d control_input, control_input1, control_input1prev;
    Eigen::Vector2d LT, RT, LT_l, RT_l, LT_prev, RT_prev;
    Eigen::Vector3d F_diff, F_diff_m;
    Eigen::Vector3d desired_ankle_torque, pr_temp, pl_temp;

    std::atomic<double> LTroll;
    std::atomic<double> RTroll;
    std::atomic<double> LTpitch;
    std::atomic<double> RTpitch;
    std::atomic<double> LFz;
    std::atomic<double> RFz;
    double ZMPx_prev;
    std::atomic<double> ZMPx_test;
    std::atomic<double> alpha_test;

    std::atomic<double> xi_test;
    std::atomic<double> yi_test;
    std::atomic<double> LZ1_test;
    std::atomic<double> LZ2_test;

    double ZMPy_prev;
    std::atomic<double> ZMPy_test;
    std::atomic<bool> walk_start;

    Eigen::VectorXd Fc, tau_, nle;

    int variable_size1, constraint_size1;
    int variable_size2, constraint_size2;
    Eigen::Vector2d zmp_bx;

   CQuadraticProgram qp_momentum_control;
   CQuadraticProgram qp_torque_control;


   double zmpy, zmpx;
    double forcex;
    bool mpc_ok = true;
    Eigen::Vector3d rfoot_ori, lfoot_ori,rfoot_ori_c, lfoot_ori_c, pelv_ori_c;

    int controlwalk_time;


   std::atomic<bool> state_init_bool;
   std::atomic<bool> desired_init_bool;

   std::mutex thread1_lock, thread2_lock;

   Eigen::VectorXd state_init;
   Eigen::VectorXd desired_val;
   Eigen::VectorXd state_init_mu;
   Eigen::VectorXd desired_val_mu;

   int new_socket;
   bool mpc_start_init_bool = false;
   bool mpc_start_init_bool1 = false;
   bool mpc_start_init_bool2 = false;
   bool mpc_start_init_bool3 = false;

   double buffer[51] = {1.0, 2, 3, 4, 5, 6, 
    0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 
    0, 0, 0, 0, 1.0, 2, 
    3, 4, 5, 6, 0, 0,
    0, 99, 100};

   double buffer1[50] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
   

   MatrixXd J;
   VectorXd X;

private:
    Eigen::VectorQd ControlVal_;
};
