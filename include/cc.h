#include <vector>
#include <array>
#include <string>
#include <chrono>
#include <thread>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <std_msgs/String.h>
#include <sys/shm.h>
#include "math_type_define.h"
#include "walking.h"
#include "wholebody_functions.h"


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

    std::future<void> solverx;
    std::future<void> solvery;

    bool mpc_s = false;

    RobotData &rd_;
    RobotData rd_cc_;

    int LFjoint_id, RFjoint_id, RFframe_id, LFframe_id, RFcframe_id, LFcframe_id;

    CQuadraticProgram QP_m;

    //pinocchiio
    Eigen::VectorQd q_;
    Eigen::VectorQd qdot, qddot, qdot_, qddot_;
    Eigen::MatrixXd CMM;
    Eigen::MatrixQQd Cor_;
    Eigen::MatrixQQd M_;
    Eigen::VectorQd G_;
    Eigen::VectorVQd q_dot_virtual_lpf_;
    Eigen::VectorXd q_dm; 

    bool walking_tick_stop = false;

    bool mob_start = false;
    bool stateestimation = false;
    bool stateestimation_mode = false;
    int stateestimation_count = 0;
    
    //Contact Redistribution
    Eigen::VectorQd TorqueContact;
    Eigen::VectorQd TorqueGrav;
    Eigen::VectorQVQd q_pinocchio;
    Eigen::VectorQVQd q_pinocchio_desired;

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

    int cycle = 0;
    double Ts = 0.01;

    bool mpcxsol = true;
    bool mpcysol = true;

    int nx_;
    int nu_;
    double timeHorizon = 1.0;
    size_t K;
    
    int dist;

    std::atomic<int> mpc_cycle;
    std::atomic<int> mpc_cycle_prev;
    Eigen::Vector3d rfootd, lfootd, comd, com_mpc,vjoint_prev, vjoint_dot;
    Eigen::Vector2d angm, upperd;
    Eigen::VectorQd joint_prev, jointdot;
    bool state_init_ = true;
    Eigen::VectorVQd qd_pinocchio;
    Eigen::VectorVQd qd_pinocchio_;
    Eigen::VectorVQd qd_pinocchio_prev;
    Eigen::Vector2d ZMP_FT_law;


      bool upper_on;
    //WholebodyController &wbc_;
    //TaskCommand tc;

    Eigen::VectorXd q_desired;
    int aaa = 0;
    Eigen::MatrixXd RF_matrix;
    Eigen::MatrixXd LF_matrix;
    Eigen::MatrixXd ZMP_bound_matrix;
    Eigen::MatrixXd RF_matrix_ssp2;
    Eigen::MatrixXd LF_matrix_ssp2;
    Eigen::MatrixXd ZMP_bound_matrix_ssp2;
    Eigen::Vector3d ZMP_measured;
    Eigen::VectorXd COMX, MOMX;
    Eigen::Vector3d comd_;
    Eigen::Vector2d angd_;
    Eigen::Vector4d control_input, control_input1, control_input1prev;
    Eigen::Vector2d LT, RT, LT_l, RT_l, LT_prev, RT_prev;
    Eigen::Vector3d F_diff, F_diff_m;

    int controlwalk_time;
private:
    Eigen::VectorQd ControlVal_;
};

class CSharedMemory
{
 
private :
    

 
 
public :
 
    void setShmId(int key);
    int getShmId();
    void setKey(key_t key);
 
    void setupSharedMemory(int size);
    void setupSharedMemoryRead(int size);
    void attachSharedMemory();
    void attachSharedMemoryint();
    void close();
    int m_shmid;   
    key_t m_key;
    double *m_shared_memory;
    int *m_shared_memory_int;
};
 
 
void CSharedMemory::setShmId( int id )
{
    m_shmid = id;
}
 
 
void CSharedMemory::setKey( key_t key )
{
    m_key = key;
}
 
 
void CSharedMemory::setupSharedMemory(int size)
{
   // Setup shared memory, 11 is the size
   if ((m_shmid = shmget(m_key, size , IPC_CREAT | 0666)) < 0)
   {
      printf("Error getting shared memory id");
      exit( 1 );
   }
}

void CSharedMemory::setupSharedMemoryRead(int size)
{
    m_shmid = shmget(m_key, size , IPC_CREAT | 0666);
   // Setup shared memory, 11 is the size
   if ((m_shmid = shmget(m_key, size , 0666|IPC_EXCL)) < 0)
   {
      printf("Error getting shared memory id");
      exit( 1 );
   }
}
 
void CSharedMemory::attachSharedMemory()
{
   // Attached shared memory
   m_shared_memory = (double*)shmat(m_shmid,NULL,0);
   if ((*m_shared_memory) == -1)
   {
      printf("Error attaching shared memory id");
      exit(1);
   }
}

void CSharedMemory::attachSharedMemoryint()
{
   // Attached shared memory
   m_shared_memory_int = (int*)shmat(m_shmid,NULL,0);
   if ((*m_shared_memory_int) == -1)
   {
      printf("Error attaching shared memoryint id");
      exit(1);
   }
}
 
void CSharedMemory::close()
{
   // Detach and remove shared memory
   shmctl(m_shmid,IPC_RMID,NULL);
 
}