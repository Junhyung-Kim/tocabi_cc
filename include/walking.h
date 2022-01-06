#include "tocabi_lib/robot_data.h"
#include <vector>
#include <mutex>
#include <shared_mutex>
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
    int mpct = 5;

    const int LEFT = 0;
    const int RIGHT = 1;
    Eigen::VectorXd debug;
    bool debug_temp = true;
    double debug_temp1 = 0.0;

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
    std::atomic<double> double2Single1;
    std::atomic<double> double2Single_pre1;
    std::atomic<double> single2Double1;
    std::atomic<double> single2Double_pre1;

    Eigen::Isometry3d supportfoot_float_current_yaw_only;

    double t_imp;
    double foot_height;
    std::atomic<double> zc;
    std::atomic<double> lipm_w;
    std::atomic<double> total_mass;
    std::atomic<int> walking_tick;
    int walking_tick_prev;
    std::atomic<int> walking_init_tick;
    std::atomic<int> debugg_int;
    std::atomic<double> contactMode;
    std::atomic<double> phaseChange;
    std::atomic<double> phaseChange1;

    std::atomic<double> phaseChange2;
    std::atomic<double> phaseChange3;

    //FT
    Eigen::Vector2d RT, LT, RT_prev, LT_prev, RT_l, LT_l, RT_mu, LT_mu;
    Eigen::Vector3d RF_d, LF_d, z_ctrl;
    double K_fx, T_fx, K_fy, T_fy, K_fz, T_fz;

    //mutex
    std::mutex cc_mutex;
    std::mutex cc_mutex1;

    //walkingInit
    Eigen::VectorQd q_target, q_init;

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
    void mpcSoftVariable(RobotData &Robot);
    void mpcStateContraint(RobotData &Robot);
    void setContactMode();
    void saveFootTrajectory();
    void setPelvTrajectory();
    void setIKparam(RobotData &Robot);
    void inverseKinematics(RobotData &Robot, Eigen::Isometry3d PELV_float_transform, Eigen::Isometry3d LF_float_transform, Eigen::Isometry3d RF_float_transform, Eigen::Vector12d &leg_q);
    void jacobianInverseKinematics(RobotData &Robot, Eigen::Isometry3d PELV_float, Eigen::Isometry3d LF_float, Eigen::Isometry3d RF_float, Eigen::Isometry3d PELV_float_pos, Eigen::Isometry3d LF_float_pos, Eigen::Isometry3d RF_float_pos);
    void inverseKinematicsdob(RobotData &Robot);
    void updateNextStepTime(RobotData &rd);
    void setWalkingParameter();
    void setInitPose(RobotData &Robot, Eigen::VectorQd &leg_q);
    void walkingInitialize(RobotData &Robot);
    void comController(RobotData &Robot);
    void supportToFloatPattern(RobotData &Robot);
    void updateInitTime();
    
    Eigen::Isometry3d RF_float_init;
    Eigen::Isometry3d RFx_float_init;
    Eigen::Isometry3d LF_float_init;
    Eigen::Isometry3d LFx_float_init;
    Eigen::Isometry3d COM_float_init;
    Eigen::Isometry3d COM_float_init_mu;
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
    Eigen::Isometry3d PELV_trajectory_float_c;
    Eigen::Isometry3d PELVD_trajectory_float;
    Eigen::Vector3d foot_distance;
    Eigen::Vector3d COMV_support_currentV;
    Eigen::Vector6d SUF_float_initV;
    Eigen::Vector6d SWF_float_initV;
    Eigen::Isometry3d RF_support_init;
    Eigen::Isometry3d LF_support_init;
    Eigen::Vector3d LF_support_euler_init;
    Eigen::Vector3d RF_support_euler_init;
    Eigen::Vector3d PELV_support_euler_init;

    Eigen::VectorXd LFx_trajectory_float;
    Eigen::VectorXd RFx_trajectory_float;
    Eigen::VectorXd LFy_trajectory_float;
    Eigen::VectorXd RFy_trajectory_float;
    Eigen::VectorXd LFz_trajectory_float;
    Eigen::VectorXd RFz_trajectory_float;

    Eigen::VectorXd LFx_trajectory_float_mu;
    Eigen::VectorXd RFx_trajectory_float_mu;
    Eigen::VectorXd LFy_trajectory_float_mu;
    Eigen::VectorXd RFy_trajectory_float_mu;
    Eigen::VectorXd LFz_trajectory_float_mu;
    Eigen::VectorXd RFz_trajectory_float_mu;

    Eigen::VectorXd LFvx_trajectory_float;
    Eigen::VectorXd RFvx_trajectory_float;
    Eigen::VectorXd LFvy_trajectory_float;
    Eigen::VectorXd RFvy_trajectory_float;
    Eigen::VectorXd LFvz_trajectory_float;
    Eigen::VectorXd RFvz_trajectory_float;

    Eigen::VectorXd LFvx_trajectory_float_mu;
    Eigen::VectorXd RFvx_trajectory_float_mu;
    Eigen::VectorXd LFvy_trajectory_float_mu;
    Eigen::VectorXd RFvy_trajectory_float_mu;
    Eigen::VectorXd LFvz_trajectory_float_mu;
    Eigen::VectorXd RFvz_trajectory_float_mu;

    Eigen::Matrix6Vd Ag_;
    Eigen::Matrix6d Ag_v;
    Eigen::Matrix6x12d Ag_leg;
    Eigen::Matrix6x8d Ag_armR;
    Eigen::Matrix6x8d Ag_armL;
    Eigen::Matrix6x3d Ag_waist;
    Eigen::Vector3d H_leg;
    Eigen::Vector2d Hl_leg;
    Eigen::Vector6d H_data;
    Eigen::Vector3d WH_data;
    double lmom;

    Eigen::Isometry3d LF_trajectory_float;
    Eigen::Isometry3d RF_trajectory_float;
    Eigen::Isometry3d LFD_trajectory_float;
    Eigen::Isometry3d RFD_trajectory_float;

    Eigen::MatrixXd foot_step;
    Eigen::MatrixXd foot_step_mu;
    std::atomic<int> desired_foot_step_num;
    std::atomic<int> current_step_num;
    std::atomic<int> total_step_num;

    Eigen::Vector6d Fr, Fl, Fr_mu, Fl_mu, Fr_l, Fl_l, Fr_prev, Fl_prev;
    Eigen::Vector3d pr, pl;
    Eigen::Vector2d zmpl, zmpr;

    Eigen::Vector4d control_input;
    Eigen::Vector2d posture_input;

    Eigen::Vector3d com_support_current_;
    Eigen::Vector4d ZMP_ref, ZMP_real, ZMP_sup, ZMP_r_sup;
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
    Eigen::VectorXd com_refx_mu;
    Eigen::VectorXd com_refy_mu;
    Eigen::VectorXd com_refdx;
    Eigen::VectorXd com_refdy;
    Eigen::VectorXd zmp_refx;
    Eigen::VectorXd zmp_refy;
    Eigen::VectorXd zmp_refx_mu;
    Eigen::VectorXd zmp_refy_mu;
    Eigen::VectorXd b_offset;

    double pelv_xp, pelv_yp, zmp_xp, zmp_yp, com_offset, com_gain1;

    Eigen::Vector3d ZMP_FT, ZMP_FT_l, ZMP_FT_prev, ZMP_FT_mu, ZMP_FT_l_mu;
    Eigen::Vector4d com_sup;
    Eigen::Vector4d comR_sup;
    Eigen::Vector4d pelvR_sup;
    Eigen::Vector4d RF_sup;    
    Eigen::Vector4d LF_sup;
    
    Eigen::Vector3d pelvR_sup1;
    Eigen::Vector4d pelvPR_sup;
    Eigen::Vector4d SUP_foot;

    //Ankle Controller
    double arp_dl, ark_dl, app_dl, apk_dl, arp_sl, ark_sl, app_sl, apk_sl, arp_dr, ark_dr, app_dr, apk_dr, arp_sr, ark_sr, app_sr, apk_sr, mobgain1, mobgain2, mobgain3, mobgain4, mobgain5, mobgain6, kc_r, tc_r, kc_p, tc_p;
    std::vector<double> mobgain;
    Eigen::VectorQd torque_est;
    int ft_ok;

    double pelv_tmp;

    //MPC variable
    double **softBoundx, **softBoundy, *softBoundx1, *softBoundy1, *softBoundx2, *softBoundy2, **softCx, **softCy, **xL, **xU, **yL, **yU, **zmpx, **zmpy;
    double **softCx_s, **softCy_s, **softBoundx_s, **softBoundy_s, **softCx_s1, **softCy_s1, **softBoundx_s1, **softBoundy_s1, **zmpx_s, **zmpy_s, **xL_s, **xU_s, **yL_s, **yU_s, **zmpx_s1, **zmpy_s1, **xL_s1, **xU_s1, **yL_s1, **yU_s1;
    double RF_mass, LF_mass;
    double Qx1_mpc, Qx2_mpc, Qx3_mpc, Qx4_mpc, Qx5_mpc, Rx1_mpc, Rx2_mpc, Zl0x_mpc, Zu0x_mpc, zl0x_mpc, zu0x_mpc, Zl1x_mpc, Zu1x_mpc, zl1x_mpc, zu1x_mpc, ZlNx_mpc, ZuNx_mpc, zlNx_mpc, zuNx_mpc;
    double Qy1_mpc, Qy2_mpc, Qy3_mpc, Qy4_mpc, Qy5_mpc, Ry1_mpc, Ry2_mpc, Zl0y_mpc, Zu0y_mpc, zl0y_mpc, zu0y_mpc, Zl1y_mpc, Zu1y_mpc, zl1y_mpc, zu1y_mpc, ZlNy_mpc, ZuNy_mpc, zlNy_mpc, zuNy_mpc;
    std::atomic<int> N;
    Eigen::Vector12d dob_hat;
    Eigen::Vector12d dob_hat_prev;
    Eigen::Vector12d desired_leg_q;
    Eigen::Vector12d desired_leg_q_temp;
    Eigen::VectorQd desired_init_q;
    double dobGain;

    //vibrationcontrol
    Eigen::Vector2d x_est;
    Eigen::Isometry3d SF_float;
    double u;
    Eigen::Vector4d u_1;

    std::atomic<double> com_mpcx;
    std::atomic<double> com_mpcy;

    std::atomic<double> com_mpcdx;
    std::atomic<double> com_mpcdy;

    std::atomic<double> cp_mpcx;
    std::atomic<double> cp_mpcy;

    std::atomic<double> cp_meax;
    std::atomic<double> cp_meay;

    std::atomic<double> cp_errx;
    std::atomic<double> cp_erry;

    std::atomic<double> mom_mpcx;
    std::atomic<double> mom_mpcy;

    std::atomic<double> zmp_mpcx;
    std::atomic<double> zmp_mpcy;

    std::atomic<double> zmp_delx;
    std::atomic<double> zmp_dely;

    std::atomic<double> H_pitch;
    double ux_vib;
    double uy_vib;

    double m;
    bool vib_est = false;

private:
};
