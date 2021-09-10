#include "tocabi_lib/robot_data.h"
#include <vector>
#include <mutex>
#include <array>
#include <string>
#include <chrono>
#include <atomic>

class WalkingController
{
public:

    const int Pelvis = 0;
    const int Upper_Body = 3;
    const int Left_Foot = 9;
    const int Right_Foot = 15;
    const int Left_Hand = 23;
    const int Right_Hand = 31;
    const int Head = 33;
    const int COM_id = 34;

    const int LEFT = 0;
    const int RIGHT = 1;

    //mutex
    std::mutex cc_mutex;

    //Ui WalkingParameter
    std::atomic<double> wk_Hz;
    std::atomic<double> wk_dt;
    int ik_mode;
    int walking_pattern;
    int foot_step_dir;
    int walking_enable;
    double height;
    double step_length_x;
    double step_length_y;
    bool dob;
    bool imu;
    bool mom;
    Eigen::Vector4d target;
    int vibration_control;
    bool com_control_mode;
    bool gyro_frame_flag;
    double com_gain;
    double pelvis_pgain;
    double pelvis_dgain;
    double pelvis_offsetx;
    int t_rest_init;
    int t_rest_last;
    int t_double1;
    int t_double2;
    int t_total;
    int t_temp;
    int t_last;
    int t_start;
    int t_start_real;
    int t_rest_temp;
    int com_control;
    std::atomic<double> double2Single;
    std::atomic<double> double2Single_pre;
    std::atomic<double> single2Double;
    std::atomic<double> single2Double_pre;
    double t_imp;
    double foot_height;
    std::atomic<double> zc;
    std::atomic<double> lipm_w;
    std::atomic<int> walking_tick;
    std::atomic<double> contactMode;
    std::atomic<double> phaseChange;
    std::atomic<double> phaseChange1;

    //walking
    void walkingCompute(RobotData &rd);
    void getRobotInitState(RobotData &rd);
    void footStepGenerator(RobotData &rd);
    void getUiWalkingParameter();
    void footStepTotal();
    void getRobotState(RobotData &rd);
    void calcRobotState(RobotData &rd);
    void setCpPosition();
    void cpReferencePatternGeneration();
    void cptoComTrajectory();
    void setFootTrajectory();
    void setPelvTrajectory();
    void inverseKinematics(RobotData &Robot, Eigen::Isometry3d PELV_float_transform, Eigen::Isometry3d LF_float_transform, Eigen::Isometry3d RF_float_transform, Eigen::Vector12d &leg_q);
    void inverseKinematicsdob(RobotData &Robot);
    void momentumControl(RobotData &Robot);
    void updateNextStepTime();
    void setWalkingParameter();
    
    Eigen::Isometry3d RF_float_init;
    Eigen::Isometry3d RFx_float_init;
    Eigen::Isometry3d LF_float_init;
    Eigen::Isometry3d LFx_float_init;
    Eigen::Isometry3d COM_float_init;
    Eigen::Isometry3d PELV_float_init;
    Eigen::Isometry3d PELV_float_init1;
    Eigen::Isometry3d SUF_float_init;
    Eigen::Isometry3d SWF_float_init;
    Eigen::Isometry3d PELV_support_init;
    Eigen::Isometry3d COM_support_init;
    Eigen::Isometry3d HLR_float_init;
    Eigen::Isometry3d HRR_float_init;
    Eigen::Isometry3d RF_float_current;
    Eigen::Isometry3d LF_float_current;
    Eigen::Isometry3d RF_support_current;
    Eigen::Isometry3d LF_support_current;
    Eigen::Isometry3d PELV_float_current;
    Eigen::Isometry3d SUF_float_current;
    Eigen::Isometry3d SWF_float_current;
    Eigen::Isometry3d PELV_support_current;
    Eigen::Vector6d SUF_float_currentV;
    Eigen::Vector6d SWF_float_currentV;
    Eigen::Isometry3d COM_float_current;
    Eigen::Isometry3d COM_support_current;
    Eigen::Isometry3d PELV_trajectory_float;
    Eigen::Isometry3d PELVD_trajectory_float;
    Eigen::Vector3d foot_distance;
    Eigen::Vector3d COMV_support_currentV;
    Eigen::Vector3d yx_vibm;
    Eigen::Vector3d yy_vibm;
    Eigen::Vector6d SUF_float_initV;
    Eigen::Vector6d SWF_float_initV;
    Eigen::Isometry3d RF_support_init;
    Eigen::Isometry3d LF_support_init;
    Eigen::Vector3d LF_support_euler_init;
    Eigen::Vector3d RF_support_euler_init;
    Eigen::Vector3d PELV_support_euler_init;

    Eigen::Matrix3x12d Ag_leg;
    Eigen::Matrix3x8d Ag_armR;
    Eigen::Matrix3x8d Ag_armL;
    Eigen::Matrix3x3d Ag_waist;
    Eigen::Matrix3x12d Agl_leg;
    Eigen::Matrix3x8d Agl_armR;
    Eigen::Matrix3x8d Agl_armL;
    Eigen::Matrix3x3d Agl_waist;

    Eigen::Isometry3d LF_trajectory_float;
    Eigen::Isometry3d RF_trajectory_float;
    Eigen::Isometry3d LFD_trajectory_float;
    Eigen::Isometry3d RFD_trajectory_float;

    Eigen::MatrixXd foot_step;
    int desired_foot_step_num;
    int current_step_num;
    int total_step_num;

    //////Capture Point//////
    Eigen::VectorXd capturePoint_ox;
    Eigen::VectorXd capturePoint_oy;
    Eigen::VectorXd capturePoint_offsetx;
    Eigen::VectorXd capturePoint_offsety;
    Eigen::VectorXd capturePoint_refx;
    Eigen::VectorXd capturePoint_refy;
    Eigen::VectorXd zmp_dx;
    Eigen::VectorXd zmp_dy;
    Eigen::VectorXd com_refx;
    Eigen::VectorXd com_refy;
    Eigen::VectorXd com_refdx;
    Eigen::VectorXd com_refdy;
    Eigen::VectorXd zmp_refx;
    Eigen::VectorXd zmp_refy;
    Eigen::VectorXd b_offset;

    Eigen::Vector12d dob_hat;
    Eigen::Vector12d dob_hat_prev;
    Eigen::Vector12d desired_leg_q;
    double dobGain;

    //momentum Control
    Eigen::Matrix6Qd Ag_;

private:
};
