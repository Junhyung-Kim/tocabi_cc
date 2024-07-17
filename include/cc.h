#include "tocabi_lib/robot_data.h"
#include "wholebody_functions.h"
#include <std_msgs/String.h>

#include "math_type_define.h"
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include "tocabi_msgs/matrix_3_4.h"
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/String.h>
#include <sstream>
#include <fstream>

#include <stdio.h>
#include <unistd.h>

#include <iomanip>
#include <iostream>
#include <string.h>
#include <stdlib.h>

#include <ros/ros.h>
#include "tocabi_msgs/WalkingCommand.h"
#include <std_msgs/Float32.h>

#include <sys/socket.h>
#include <arpa/inet.h>
#include <sys/shm.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <thread>
#include <future>


const int FILE_CNT1 = 15;

const std::string FILE_NAMES1[FILE_CNT1] =
{
  ///change this directory when you use this code on the other computer///
    "/home/dyros/data/0_flag_mj.txt",
    "/home/dyros/data/1_flag_mj.txt",
    "/home/dh-sung/data/dg/1_com_.txt",
    "/home/dh-sung/data/dg/2_zmp_.txt",
    "/home/dh-sung/data/dg/3_foot_.txt",
    "/home/dh-sung/data/dg/4_torque_.txt",
    "/home/dh-sung/data/dg/5_joint_.txt",
    "/home/dh-sung/data/dg/6_hand_.txt",
    "/home/dh-sung/data/dg/7_elbow_.txt",
    "/home/dh-sung/data/dg/8_shoulder_.txt",
    "/home/dh-sung/data/dg/9_acromion_.txt",
    "/home/dh-sung/data/dg/10_hmd_.txt",
    "/home/dh-sung/data/dg/11_tracker_.txt",
    "/home/dh-sung/data/dg/12_qpik_.txt",
    "/home/dh-sung/data/dg/13_tracker_vel_.txt"
};

class CustomController
{
public:
    CustomController(RobotData &rd);
    // ~CustomController();
    Eigen::VectorQd getControl();
    std::ofstream file[FILE_CNT1];
    std::ofstream calibration_log_file_ofstream_[4];
    std::ifstream calibration_log_file_ifstream_[4];
    //void taskCommandToCC(TaskCommand tc_);

    void computeSlow();
    void computeFast();
    void computePlanner();
    void copyRobotData(RobotData &rd_l);


    RobotData &rd_;
    RobotData rd_cc_;

    ros::NodeHandle nh_avatar_;
    ros::CallbackQueue queue_avatar_;
    void avatar_callback(const std_msgs::StringConstPtr& msg);
    ros::Subscriber sub_1;

    CQuadraticProgram QP_qdot;
    CQuadraticProgram QP_qdot_larm;
    CQuadraticProgram QP_qdot_rarm;
    CQuadraticProgram QP_qdot_upperbody_;
    CQuadraticProgram QP_qdot_wholebody_;
    std::vector<CQuadraticProgram> QP_qdot_hqpik_;        
    std::vector<CQuadraticProgram> QP_qdot_hqpik2_;
    std::vector<CQuadraticProgram> QP_cam_hqp_;

    CQuadraticProgram QP_motion_retargeting_lhand_;
    CQuadraticProgram QP_motion_retargeting_rhand_;
    CQuadraticProgram QP_motion_retargeting_[3];    // task1: each arm, task2: relative arm, task3: hqp second hierarchy

    Eigen::VectorQd CAM_upper_init_q_; 
    //lQR-HQP (Lexls)
    // LexLS::tools::HierarchyType type_of_hierarfchy;
    // LexLS::Index number_of_variables;
    // LexLS::Index number_of_objectives;
    // std::vector<LexLS::Index> number_of_constraints;
    // std::vector<LexLS::ObjectiveType> types_of_objectives;
    // std::vector<Eigen::MatrixXd> objectives;
    // LexLS::ParametersLexLSI parameters;

    // LexLS::internal::LexLSI lsi_;
    int as = 1;
    Eigen::Vector2d left_zmp, right_zmp;
    Eigen::VectorVQd qd_temp; 
    Eigen::VectorVQd q_temp;
    bool torque_control = true; 
    bool torquecontrol_first = true;
    bool hipcompen = true; 
    bool momentumControlMode = true;
    bool pelvroll = false; 
    Eigen::Vector3d ZMP_float;
    double left_hip_roll_temp , right_hip_roll_temp , left_hip_xpitch_temp , right_hip_pitch_temp , left_ank_pitch_temp , right_ank_pitch_temp;

    string rf,rf1,rf2,rf3,rf4,rf5,rf6,rf7;

    Eigen::MatrixXd RF_matrix;
    Eigen::MatrixXd LF_matrix;
    Eigen::MatrixXd ZMP_bound_matrix, com_matrix_matrix, comd_matrix_matrix, angm_ref_matrix, zmp_matrix;
    Eigen::MatrixXd RF_matrix_ssp2;
    Eigen::MatrixXd LF_matrix_ssp2;
    Eigen::MatrixXd RF_matrix_ssp1;
    Eigen::MatrixXd LF_matrix_ssp1;
    Eigen::MatrixXd ZMP_bound_matrix_ssp2;
    Eigen::Isometry3d supportfoot_float_current_yaw_only;


    std::atomic<bool> atb_grav_update_{false};
    std::atomic<bool> atb_desired_q_update_{false};
    std::atomic<bool> atb_walking_traj_update_{false};
    std::atomic<bool> atb_qddot_update_{false};
    std::atomic<bool> atb_desired_update_{false};
    std::atomic<bool> atb_state_update_{false};
    std::atomic<bool> atb_phase_update_{false};

    RigidBodyDynamics::Model model_d_;  //updated by desired q
    RigidBodyDynamics::Model model_c_;  //updated by current q
    RigidBodyDynamics::Model model_C_;  //for calcuating Coriolis matrix
    RigidBodyDynamics::Model model_JJ_;  //for calcuating CMM

    //////////dg custom controller functions////////
    void setGains();
    void getRobotData();
    void getProcessedRobotData();
    void walkingStateManager();
    void motionGenerator();
    void getCOMTrajectory_dg();

    void getSwingFootXYTrajectory(double phase, Eigen::Vector3d com_pos_current, Eigen::Vector3d com_vel_current, Eigen::Vector3d com_vel_desired);
    void getSwingFootXYZTrajectory();
    
    void cpCompensator();
    
    Eigen::VectorQd comVelocityControlCompute();
    Eigen::VectorQd swingFootControlCompute();
    Eigen::VectorQd jointTrajectoryPDControlCompute();
    Eigen::VectorQd dampingControlCompute();
    Eigen::VectorQd jointLimit(); 
    Eigen::VectorQd gravityCompensator(Eigen::VectorQd current_q);
    Eigen::VectorQd hipAngleCompensator(Eigen::VectorQd desired_q);
    
    Eigen::VectorQVQd q_virtual_state;

    //estimator
    Eigen::VectorXd momentumObserver(VectorXd current_momentum, VectorXd current_torque, VectorXd nonlinear_term, VectorXd mob_residual_pre, double dt, double k);

    bool balanceTrigger(Eigen::Vector2d com_pos_2d, Eigen::Vector2d com_vel_2d);
    int checkZMPinWhichFoot(Eigen::Vector2d zmp_measured); // check where the zmp is
    Eigen::VectorQd tuneTorqueForZMPSafety(Eigen::VectorQd task_torque); // check where the zmp is
    // Eigen::VectorQd zmpAnkleControl();
    // Eigen::VectorQd jointComTrackingTuning();
    void fallDetection();
    
    //preview related functions
    void getComTrajectory_Preview();
    void modifiedPreviewControl_MJ();
    void previewParam_MJ(double dt, int NL, double zc, Eigen::Matrix4d& K, Eigen::MatrixXd& Gi, Eigen::VectorXd& Gd, Eigen::MatrixXd& Gx, Eigen::MatrixXd& A, Eigen::VectorXd& B, Eigen::MatrixXd& C, Eigen::MatrixXd& D, Eigen::MatrixXd& A_bar, Eigen::VectorXd& B_bar);
    void preview_MJ(double dt, int NL, double x_i, double y_i, Eigen::Vector3d xs, Eigen::Vector3d ys, double& UX, double& UY, Eigen::MatrixXd Gi, Eigen::VectorXd Gd, Eigen::MatrixXd Gx, Eigen::MatrixXd A, Eigen::VectorXd B, Eigen::MatrixXd C, Eigen::Vector3d &XD, Eigen::Vector3d &YD);
    Eigen::MatrixXd discreteRiccatiEquationPrev(Eigen::MatrixXd a, Eigen::MatrixXd b, Eigen::MatrixXd r, Eigen::MatrixXd q);

    void getCentroidalMomentumMatrix(MatrixXd mass_matrix, MatrixXd &CMM);
    void updateCMM_DG();
    void CentroidalMomentCalculator();

    void setContact_1(RobotData &Robot, bool left_foot, bool right_foot, bool left_hand, bool right_hand);

    void proc_recv();
    void proc_recv1();
    
    void savePreData();
    void printOutTextFile();

    unsigned int upper_body_mode_;                          // 1: init pose,  2: zero pose, 3: swing arm 4: motion retarggeting
    bool walking_mode_on_;                                  // turns on when the walking control command is received and truns off after saving start time
    double stop_vel_threshold_;                             // acceptable capture point deviation from support foot
    bool chair_mode_;                                       // For chair sitting mode

    int foot_contact_;                                      // 1:left,   -1:right,   0:double
    int foot_contact_pre_;
    bool foot_swing_trigger_;                               // trigger swing if the robot needs to balance.
    bool first_step_trigger_;                               // ture if this is first swing foot. turn on at the start of the swing.
    bool start_walking_trigger_;                            // true when the walking_speed_ is not zero and leg-swing do not start.
    bool stop_walking_trigger_;                             // turns on when the robot's speed become zero and lands last foot step
    bool falling_detection_flag_;                           // turns on when the robot is falling and is considered that it can not recover balance.

    int stop_walking_counter_;                              // Stepping number after walking speed command is zero
    int max_stop_walking_num_;                              // maximum stepping number robot will walk after walking speed is commanded zero 

    double stance_start_time_;
    double program_start_time_;
    
    double program_ready_duration_;                         // during [program_start_time, program_start_time + program_ready_duration_], init parameters are calculated and the robot is position-controlled to the inin_q
    double walking_control_transition_duration_;            

    double walking_duration_;
    double walking_duration_cmd_;
    double walking_duration_start_delay_;
    double walking_phase_;

    std::atomic<double> step_test;

    Eigen::VectorXd COMX, MOMX;
    Eigen::MatrixXd CMM;
    // double dsp_phase_;
    // double ssp_phase_;

    double turning_duration_;
    double turning_phase_;
    double switching_phase_duration_;
    double dsp_duration_;
    double dsp_ratio_;

    double current_time_;
    double pre_time_;
    double start_time_;
    double dt_;
    double init_leg_time_;  //for first smothing of leg joint angle

    double walking_speed_;
    double walking_speed_side_;
    double yaw_angular_vel_;
    double knee_target_angle_;

    double step_width_;
    double step_length_;

    double swing_foot_height_;
    double com_target_height_;

    double ankle2footcenter_offset_;

    double first_torque_supplier_;                         // this increase with cubic function from 0 to 1 during [program_start_time + program_ready_duration_, program_start_time + program_ready_duration_ + walking_control_transition_duration_]
    double swingfoot_force_control_converter_;
    double swingfoot_highest_time_;
    
    // CoM variables in global frame
    Eigen::Vector3d com_pos_desired_; 
    Eigen::Vector3d com_vel_desired_;
    Eigen::Vector3d com_acc_desired_;
    Eigen::Vector3d com_pos_current_;
    Eigen::Vector3d com_vel_current_;
    Eigen::Vector3d com_acc_current_;

    Eigen::Vector3d com_d, com_d2, rfoot_d, lfoot_d, com_d1, rfoot_d1, lfoot_d1, rfoot_d2, lfoot_d2, rfoot_ori, lfoot_ori, rfoot_ori1, lfoot_ori1, rfoot_ori_c, lfoot_ori_c;
    Eigen::Vector2d ang_d;
    double gain_xz, gain_ori;
    std::atomic<double> lfootz, rfootz;
    int mpc_cycle_int,mpc_cycle_int1;

    Eigen::Isometry3d pelv_trajectory_float_pre, lfoot_trajectory_float_pre, rfoot_trajectory_float_pre;
    Eigen::Isometry3d pelv_trajectory_float_pre1, lfoot_trajectory_float_pre1, rfoot_trajectory_float_pre1;

    Eigen::Vector2d F_d;
    Eigen::Vector3d zero_center;

    double com_vel_cutoff_freq_;
    double wn_;
    double com_mass_;

    Eigen::Vector3d com_pos_init_;
    Eigen::Vector3d com_vel_init_;
    Eigen::Vector3d com_acc_init_;
    Eigen::Vector3d com_pos_desired_pre_; 
    Eigen::Vector3d com_vel_desired_pre_;
    Eigen::Vector3d com_acc_desired_pre_;
    Eigen::Vector3d com_pos_desired_last_; 
    Eigen::Vector3d com_vel_desired_last_;
    Eigen::Vector3d com_acc_desired_last_;
    
    Eigen::Vector3d com_pos_pre_desired_from_;
    Eigen::Vector3d com_vel_pre_desired_from_;
    Eigen::Vector3d com_acc_pre_desired_from_;

    Eigen::Vector3d com_pos_error_;
    Eigen::Vector3d com_vel_error_;

    Eigen::Vector3d com_vel_est1_;  // Jv*q_dot
    Eigen::Vector3d com_vel_est2_;  // Jv*q_dot + r X w_f

    Eigen::Matrix3d kp_compos_; // 196(sim) (tune)
    Eigen::Matrix3d kd_compos_;  // 28(sim) (tune)

    // Pevlis related variables
    Eigen::Vector3d pelv_pos_current_;
    Eigen::Vector6d pelv_vel_current_;
    Eigen::Vector3d pelv_angvel_current_;
    Eigen::Matrix3d pelv_rot_current_;
    Eigen::Vector3d pelv_rpy_current_;
    Eigen::Matrix3d pelv_rot_current_yaw_aline_;
    Eigen::Matrix3d pelv_yaw_rot_current_from_global_;
    Eigen::Isometry3d pelv_transform_current_from_global_;
    double pelv_height_offset_;
     
    Eigen::Vector3d pelv_pos_init_;
    Eigen::Vector6d pelv_vel_init_;
    Eigen::Matrix3d pelv_rot_init_;
    Eigen::Vector3d pelv_rpy_init_;
    Eigen::Matrix3d pelv_rot_init_yaw_aline_;
    Eigen::Isometry3d pelv_transform_init_from_global_;
    Eigen::Isometry3d pelv_trajectory_support_init_;


    Eigen::Vector3d phi_pelv_;
    Eigen::Vector3d torque_pelv_;
    Eigen::VectorQd torque_stance_hip_;
    Eigen::VectorQd torque_swing_assist_;

    Eigen::Matrix3d kp_pelv_ori_; // 196(sim) (tune)
    Eigen::Matrix3d kd_pelv_ori_;    // 28(sim) (tune)
    // Joint related variables
    Eigen::VectorQd current_q_;
    Eigen::VectorQd current_q_dot_;
    Eigen::VectorQd current_q_ddot_;
    Eigen::VectorQd desired_q_;
    Eigen::VectorQd desired_q_dot_;
    Eigen::VectorQd desired_q_ddot_;
    Eigen::VectorQd pre_q_;
    Eigen::VectorQd pre_desired_q_;
    Eigen::VectorQd pre_desired_q_dot_;
    Eigen::VectorQd last_desired_q_;
    Eigen::VectorQVQd pre_desired_q_qvqd_;
    Eigen::VectorVQd pre_desired_q_dot_vqd_;
    Eigen::VectorVQd pre_desired_q_ddot_vqd_;

    Eigen::VectorQd desired_q_fast_;
    Eigen::VectorQd desired_q_dot_fast_;
    Eigen::VectorQd desired_q_slow_;
    Eigen::VectorQd desired_q_dot_slow_;

    Eigen::VectorQd motion_q_;
    Eigen::VectorQd motion_q_dot_;
    Eigen::VectorQd motion_q_pre_;
    Eigen::VectorQd motion_q_dot_pre_;
    Eigen::VectorQd init_q_;
    Eigen::VectorQd zero_q_;
    Eigen::VectorQVQd init_q_virtual_;

    Eigen::MatrixVVd A_mat_;
    Eigen::MatrixVVd A_mat_pre_;
    Eigen::MatrixVVd A_inv_mat_;
    Eigen::MatrixVVd A_dot_mat_;

    Eigen::MatrixVVd motor_inertia_mat_;
    Eigen::MatrixVVd motor_inertia_inv_mat_;

    Eigen::MatrixVVd C_mat_;
    Eigen::MatrixVVd C_T_mat_;
    Eigen::VectorVQd nonlinear_torque_;

    Eigen::VectorQd kp_joint_;
    Eigen::VectorQd kv_joint_;

    Eigen::VectorQd kp_stiff_joint_;
    Eigen::VectorQd kv_stiff_joint_;
    Eigen::VectorQd kp_soft_joint_;
    Eigen::VectorQd kv_soft_joint_;
    // walking controller variables
    double alpha_x_;
    double alpha_y_;
    double alpha_lpf_;
    double alpha_x_command_;    
    double alpha_y_command_;
    Eigen::VectorQd pd_control_mask_; //1 for joint ik pd control

    Eigen::Vector2d target_foot_landing_from_pelv_;
    Eigen::Vector2d target_foot_landing_from_sup_;
    Eigen::Vector3d swing_foot_pos_trajectory_from_global_;
    Eigen::Vector6d swing_foot_vel_trajectory_from_global_;
    Eigen::Vector6d swing_foot_acc_trajectory_from_global_;
    Eigen::Matrix3d swing_foot_rot_trajectory_from_global_;

    Eigen::Vector3d swing_foot_pos_trajectory_from_support_;
    Eigen::Vector6d swing_foot_vel_trajectory_from_support_;
    Eigen::Vector6d swing_foot_acc_trajectory_from_support_;
    Eigen::Matrix3d swing_foot_rot_trajectory_from_support_;

    Eigen::Vector3d swing_foot_pos_error_from_support_;

    Eigen::Isometry3d swing_foot_transform_init_;
    Eigen::Vector3d swing_foot_rpy_init_;
    Eigen::Isometry3d support_foot_transform_init_;
    Eigen::Vector3d support_foot_rpy_init_;

    Eigen::Isometry3d swing_foot_transform_current_;
    Eigen::Isometry3d support_foot_transform_current_;

    Eigen::Isometry3d swing_foot_transform_pre_;
    Eigen::Isometry3d support_foot_transform_pre_;

    Eigen::Vector6d swing_foot_vel_current_;
    Eigen::Vector6d swing_foot_vel_init_;

    Eigen::Vector6d support_foot_vel_current_;

    //getLegIK
    Eigen::Isometry3d lfoot_transform_desired_;
    Eigen::Isometry3d rfoot_transform_desired_;
    Eigen::Isometry3d pelv_transform_desired_;
    Eigen::Isometry3d lfoot_transform_desired_last_;
    Eigen::Isometry3d rfoot_transform_desired_last_;
    Eigen::Isometry3d pelv_transform_desired_last_;

    Eigen::MatrixXd jac_com_;
    Eigen::MatrixXd jac_com_pos_;
    Eigen::MatrixXd jac_rhand_;
    Eigen::MatrixXd jac_lhand_;
    Eigen::MatrixXd jac_rfoot_;
    Eigen::MatrixXd jac_lfoot_;

    Eigen::MatrixXd lfoot_to_com_jac_from_global_;
    Eigen::MatrixXd rfoot_to_com_jac_from_global_;
    
    Eigen::Isometry3d pelv_transform_start_from_global_;
    Eigen::Isometry3d rfoot_transform_start_from_global_;
    Eigen::Isometry3d lfoot_transform_start_from_global_;

    Eigen::Isometry3d upperbody_transform_init_from_global_;
    Eigen::Isometry3d head_transform_init_from_global_;
    Eigen::Isometry3d lfoot_transform_init_from_global_;
    Eigen::Isometry3d rfoot_transform_init_from_global_;
    Eigen::Isometry3d lhand_transform_init_from_global_;
    Eigen::Isometry3d rhand_transform_init_from_global_;
    Eigen::Isometry3d lelbow_transform_init_from_global_;
    Eigen::Isometry3d relbow_transform_init_from_global_;
    Eigen::Isometry3d lupperarm_transform_init_from_global_; //4rd axis of arm joint 
    Eigen::Isometry3d rupperarm_transform_init_from_global_; 
    Eigen::Isometry3d lshoulder_transform_init_from_global_; //3rd axis of arm joint 
    Eigen::Isometry3d rshoulder_transform_init_from_global_; 
    Eigen::Isometry3d lacromion_transform_init_from_global_; //2nd axis of arm joint (견봉)
    Eigen::Isometry3d racromion_transform_init_from_global_;
    Eigen::Isometry3d larmbase_transform_init_from_global_; //1st axis of arm joint (견봉)
    Eigen::Isometry3d rarmbase_transform_init_from_global_;

    Eigen::Isometry3d upperbody_transform_current_from_global_;
    Eigen::Isometry3d head_transform_current_from_global_;
    Eigen::Isometry3d lfoot_transform_current_from_global_;
    Eigen::Isometry3d rfoot_transform_current_from_global_;
    Eigen::Isometry3d lfootc_transform_current_from_global_;
    Eigen::Isometry3d rfootc_transform_current_from_global_;
    Eigen::Isometry3d lhand_transform_current_from_global_;
    Eigen::Isometry3d rhand_transform_current_from_global_;
    Eigen::Isometry3d lelbow_transform_current_from_global_;
    Eigen::Isometry3d relbow_transform_current_from_global_;
    Eigen::Isometry3d lupperarm_transform_current_from_global_; //4th axis of arm joint
    Eigen::Isometry3d rupperarm_transform_current_from_global_;
    Eigen::Isometry3d lshoulder_transform_current_from_global_; //3rd axis of arm joint
    Eigen::Isometry3d rshoulder_transform_current_from_global_;
    Eigen::Isometry3d lacromion_transform_current_from_global_; //2nd axis of arm joint (견봉)
    Eigen::Isometry3d racromion_transform_current_from_global_;
    Eigen::Isometry3d larmbase_transform_current_from_global_; //1st axis of arm joint 
    Eigen::Isometry3d rarmbase_transform_current_from_global_;

    Eigen::Isometry3d lknee_transform_current_from_global_;
    Eigen::Isometry3d rknee_transform_current_from_global_;

    Eigen::Vector3d lhand_rpy_current_from_global_;
    Eigen::Vector3d rhand_rpy_current_from_global_;
    Eigen::Vector3d lelbow_rpy_current_from_global_;
    Eigen::Vector3d relbow_rpy_current_from_global_;
    Eigen::Vector3d lupperarm_rpy_current_from_global_;
    Eigen::Vector3d rupperarm_rpy_current_from_global_;
    Eigen::Vector3d lshoulder_rpy_current_from_global_;
    Eigen::Vector3d rshoulder_rpy_current_from_global_;
    Eigen::Vector3d lacromion_rpy_current_from_global_;
    Eigen::Vector3d racromion_rpy_current_from_global_;

    Eigen::Isometry3d upperbody_transform_pre_desired_from_;
    Eigen::Isometry3d head_transform_pre_desired_from_;
    Eigen::Isometry3d lfoot_transform_pre_desired_from_;
    Eigen::Isometry3d rfoot_transform_pre_desired_from_;
    Eigen::Isometry3d lhand_transform_pre_desired_from_;
    Eigen::Isometry3d rhand_transform_pre_desired_from_;
    Eigen::Isometry3d lelbow_transform_pre_desired_from_;
    Eigen::Isometry3d relbow_transform_pre_desired_from_;
    Eigen::Isometry3d lupperarm_transform_pre_desired_from_;
    Eigen::Isometry3d rupperarm_transform_pre_desired_from_;
    Eigen::Isometry3d lshoulder_transform_pre_desired_from_;
    Eigen::Isometry3d rshoulder_transform_pre_desired_from_;
    Eigen::Isometry3d lacromion_transform_pre_desired_from_;
    Eigen::Isometry3d racromion_transform_pre_desired_from_;
    Eigen::Isometry3d larmbase_transform_pre_desired_from_; //1st axis of arm joint
    Eigen::Isometry3d rarmbase_transform_pre_desired_from_;

    Eigen::Vector3d lhand_control_point_offset_, rhand_control_point_offset_;   //red_hand made by sy

    Eigen::Vector6d lfoot_vel_current_from_global_;
    Eigen::Vector6d rfoot_vel_current_from_global_;
    Eigen::Vector6d lhand_vel_current_from_global_;
    Eigen::Vector6d rhand_vel_current_from_global_;
    Eigen::Vector6d lelbow_vel_current_from_global_;
    Eigen::Vector6d relbow_vel_current_from_global_;
    Eigen::Vector6d lupperarm_vel_current_from_global_;
    Eigen::Vector6d rupperarm_vel_current_from_global_;
    Eigen::Vector6d lshoulder_vel_current_from_global_;
    Eigen::Vector6d rshoulder_vel_current_from_global_;
    Eigen::Vector6d lacromion_vel_current_from_global_;
    Eigen::Vector6d racromion_vel_current_from_global_;

    Eigen::Isometry3d lfoot_transform_init_from_support_;
    Eigen::Isometry3d rfoot_transform_init_from_support_;
    Eigen::Isometry3d pelv_transform_init_from_support_;
    Eigen::Vector3d pelv_rpy_init_from_support_;

    Eigen::Isometry3d lfoot_transform_start_from_support_;
    Eigen::Isometry3d rfoot_transform_start_from_support_;
    Eigen::Isometry3d pelv_transform_start_from_support_;

    Eigen::Isometry3d lfoot_transform_current_from_support_;
    Eigen::Isometry3d rfoot_transform_current_from_support_;
    Eigen::Isometry3d lfootc_transform_current_from_support_;
    Eigen::Isometry3d rfootc_transform_current_from_support_;
    Eigen::Isometry3d pelv_transform_current_from_support_;

    Eigen::Vector6d lfoot_vel_current_from_support_;
    Eigen::Vector6d rfoot_vel_current_from_support_;

    Eigen::Isometry3d swing_foot_transform_init_from_support_;
    Eigen::Vector3d swing_foot_rpy_init_from_support_;
    Eigen::Isometry3d support_foot_transform_init_from_support_;
    Eigen::Vector3d support_foot_rpy_init_from_support_;

    Eigen::Isometry3d swing_foot_transform_current_from_support_;
    Eigen::Isometry3d support_foot_transform_current_from_support_;

    Eigen::Isometry3d swing_foot_transform_pre_from_support_;
    Eigen::Isometry3d support_foot_transform_pre_from_support_;

    Eigen::Vector3d middle_of_both_foot_;
    Eigen::Vector3d middle_of_both_foot_init_;

    Eigen::Vector3d com_pos_init_from_support_;

    Eigen::Vector3d com_pos_desired_from_support_; 
    Eigen::Vector3d com_vel_desired_from_support_;
    Eigen::Vector3d com_acc_desired_from_support_;
    Eigen::Vector3d com_jerk_desired_from_support_;

    Eigen::Vector3d com_pos_pre_desired_from_support_; 
    Eigen::Vector3d com_vel_pre_desired_from_support_;
    Eigen::Vector3d com_acc_pre_desired_from_support_;
    Eigen::Vector3d com_jerk_pre_desired_from_support_;

    Eigen::Vector3d com_pos_current_from_support_;
    Eigen::Vector3d com_vel_current_from_support_;
    Eigen::Vector3d com_acc_current_from_support_;
    Eigen::Vector3d com_pos_pre_from_support_;
    Eigen::Vector3d com_vel_pre_from_support_;
    Eigen::Vector3d com_acc_pre_from_support_;
    Eigen::Vector3d com_pos_ppre_from_support_;
    Eigen::Vector3d com_vel_ppre_from_support_;
    Eigen::Vector3d com_acc_ppre_from_support_;
    
    Eigen::Vector3d com_vel_current_lpf_from_support_;
    Eigen::Vector3d com_vel_pre_lpf_from_support_;
    Eigen::Vector3d com_vel_ppre_lpf_from_support_;

    Eigen::Vector6d com_pos_limit_; //min x y z max x y z
    Eigen::Vector6d com_vel_limit_;
    Eigen::Vector6d com_acc_limit_;

    Eigen::Vector3d cp_current_from_suppport_;

    Eigen::Vector6d contact_force_lfoot_;
    Eigen::Vector6d contact_force_rfoot_;
    
    Eigen::Vector6d contact_force_lfoot_local_;
    Eigen::Vector6d contact_force_rfoot_local_;

    Eigen::Vector3d zmp_measured_;
    Eigen::Vector3d zmp_measured_pre_;
    Eigen::Vector3d zmp_measured_ppre_;
    Eigen::Vector3d zmp_dot_measured_;

    Eigen::Vector3d zmp_measured_local_; //calc zmp with F/T sensors according to the Robot.ee_[0].contact
    Eigen::Vector3d zmp_dot_measured_local_;

    Eigen::Vector3d zmp_local_lfoot_;
    Eigen::Vector3d zmp_local_rfoot_;
    Eigen::Vector3d zmp_local_lfoot_pre_;
    Eigen::Vector3d zmp_local_rfoot_pre_;
    Eigen::Vector3d zmp_dot_local_rfoot_;
    Eigen::Vector3d zmp_dot_local_lfoot_;

    Eigen::Vector3d zmp_measured_lfoot_; //calc only left foot zmp with a F/T sensor
    Eigen::Vector3d zmp_measured_rfoot_;

    Eigen::Vector3d zmp_current_by_com_from_support_;

    Eigen::Vector3d zmp_desired_from_global_;
    Eigen::Vector3d zmp_desired_pre_;

    double zmp_y_offset_;
    
    Eigen::Vector6d l_ft_;
    Eigen::Vector6d r_ft_;

    Eigen::Vector6d l_ft_LPF;
    Eigen::Vector6d r_ft_LPF;

    double F_F_input_dot = 0;
    double F_F_input = 0;

    double F_T_L_x_input = 0;
    double F_T_L_x_input_dot = 0;
    double F_T_R_x_input = 0;
    double F_T_R_x_input_dot = 0;  

    double F_T_L_y_input = 0;
    double F_T_L_y_input_dot = 0;
    double F_T_R_y_input = 0;
    double F_T_R_y_input_dot = 0;

    Eigen::Vector2d f_star_xy_;
    Eigen::Vector2d f_star_xy_pre_;
    Eigen::Vector6d f_star_6d_;
    Eigen::Vector6d f_star_6d_pre_;
    Eigen::Vector6d f_star_l_;   
    Eigen::Vector6d f_star_r_;
    Eigen::Vector6d f_star_l_pre_;   
    Eigen::Vector6d f_star_r_pre_;

    Eigen::VectorQd torque_task_;
    Eigen::VectorQd torque_init_;
    Eigen::VectorQd torque_grav_;
    Eigen::VectorQd torque_task_pre_;
    Eigen::VectorQd torque_grav_pre_;
    Eigen::VectorQd torque_qp_;
    Eigen::VectorQd torque_g_;
    Eigen::VectorQd torque_upper_;
    Eigen::VectorQd torque_upper_fast_;
    Eigen::VectorQd torque_lower_;

    Eigen::VectorQd torque_task_min_;
    Eigen::VectorQd torque_task_max_;
    //getComTrajectory() variables
    double xi_;
    double yi_;
    Eigen::Vector3d xs_;
    Eigen::Vector3d ys_;
    Eigen::Vector3d xd_;
    Eigen::Vector3d yd_;
    Eigen::Vector3d xd_b;
    Eigen::Vector3d yd_b;

    //Preview Control
    double preview_horizon_;
    double preview_hz_;
    double preview_update_time_;
    double last_preview_param_update_time_;
    
    Eigen::Vector3d preview_x, preview_y, preview_x_b, preview_y_b, preview_x_b2, preview_y_b2;
    double ux_, uy_, ux_1_, uy_1_;
    double zc_;
    double gi_;
    double zmp_start_time_; //원래 코드에서는 start_time, zmp_ref 시작되는 time같음
    Eigen::Matrix4d k_;
    Eigen::Matrix4d K_act_;
    Eigen::VectorXd gp_l_;
    Eigen::Matrix1x3d gx_;
    Eigen::Matrix3d a_;
    Eigen::Vector3d b_;
    Eigen::Matrix1x3d c_;

    //Preview CPM
    Eigen::MatrixXd A_;
    Eigen::VectorXd B_;
    Eigen::MatrixXd C_;
    Eigen::MatrixXd D_;
    Eigen::Matrix3d K_;
    Eigen::MatrixXd Gi_;
    Eigen::MatrixXd Gx_;
    Eigen::VectorXd Gd_;
    Eigen::MatrixXd A_bar_;
    Eigen::VectorXd B_bar_;
    Eigen::Vector2d Preview_X, Preview_Y, Preview_X_b, Preview_Y_b;
    Eigen::VectorXd X_bar_p_, Y_bar_p_;
    Eigen::Vector2d XD_;
    Eigen::Vector2d YD_;
    double UX_, UY_;

    int zmp_size_;
    Eigen::MatrixXd ref_zmp_;
    Eigen::Vector3d com_pos_desired_preview_;
    Eigen::Vector3d com_vel_desired_preview_;
    Eigen::Vector3d com_acc_desired_preview_;

    Eigen::Vector3d com_pos_desired_preview_pre_;
    Eigen::Vector3d com_vel_desired_preview_pre_;
    Eigen::Vector3d com_acc_desired_preview_pre_;

    //siwngFootControlCompute
    Vector3d swingfoot_f_star_l_;
    Vector3d swingfoot_f_star_r_;
    Vector3d swingfoot_f_star_l_pre_;
    Vector3d swingfoot_f_star_r_pre_;

    //dampingControlCompute
    Vector3d f_lfoot_damping_;
    Vector3d f_rfoot_damping_;  
    Vector3d f_lfoot_damping_pre_;
    Vector3d f_rfoot_damping_pre_;  
    Matrix3d support_foot_damping_gain_;

    //MotionRetargeting variables
    int upperbody_mode_recieved_;
    double upperbody_command_time_;
    Eigen::VectorQd upperbody_mode_q_init_;

    Eigen::Isometry3d master_lhand_pose_raw_;
    Eigen::Isometry3d master_rhand_pose_raw_;
    Eigen::Isometry3d master_head_pose_raw_;
    Eigen::Isometry3d master_lelbow_pose_raw_;
    Eigen::Isometry3d master_relbow_pose_raw_;
    Eigen::Isometry3d master_lshoulder_pose_raw_;
    Eigen::Isometry3d master_rshoulder_pose_raw_;
    Eigen::Isometry3d master_upperbody_pose_raw_;

    Eigen::Isometry3d master_lhand_pose_raw_pre_;
    Eigen::Isometry3d master_rhand_pose_raw_pre_;
    Eigen::Isometry3d master_head_pose_raw_pre_;
    Eigen::Isometry3d master_lelbow_pose_raw_pre_;
    Eigen::Isometry3d master_relbow_pose_raw_pre_;
    Eigen::Isometry3d master_lshoulder_pose_raw_pre_;
    Eigen::Isometry3d master_rshoulder_pose_raw_pre_;
    Eigen::Isometry3d master_upperbody_pose_raw_pre_;

    Eigen::Isometry3d master_lhand_pose_raw_ppre_;
    Eigen::Isometry3d master_rhand_pose_raw_ppre_;
    Eigen::Isometry3d master_head_pose_raw_ppre_;
    Eigen::Isometry3d master_lelbow_pose_raw_ppre_;
    Eigen::Isometry3d master_relbow_pose_raw_ppre_;
    Eigen::Isometry3d master_lshoulder_pose_raw_ppre_;
    Eigen::Isometry3d master_rshoulder_pose_raw_ppre_;
    Eigen::Isometry3d master_upperbody_pose_raw_ppre_;

    Eigen::Isometry3d master_lhand_pose_;
    Eigen::Isometry3d master_rhand_pose_;
    Eigen::Isometry3d master_head_pose_;
    Eigen::Isometry3d master_lelbow_pose_;
    Eigen::Isometry3d master_relbow_pose_;
    Eigen::Isometry3d master_lshoulder_pose_;
    Eigen::Isometry3d master_rshoulder_pose_;
    Eigen::Isometry3d master_upperbody_pose_;

    Eigen::Isometry3d master_lhand_pose_pre_;
    Eigen::Isometry3d master_rhand_pose_pre_;
    Eigen::Isometry3d master_head_pose_pre_;
    Eigen::Isometry3d master_lelbow_pose_pre_;
    Eigen::Isometry3d master_relbow_pose_pre_;
    Eigen::Isometry3d master_lshoulder_pose_pre_;
    Eigen::Isometry3d master_rshoulder_pose_pre_;
    Eigen::Isometry3d master_upperbody_pose_pre_;

    Eigen::Isometry3d master_lhand_pose_ppre_;
    Eigen::Isometry3d master_rhand_pose_ppre_;
    Eigen::Isometry3d master_head_pose_ppre_;
    Eigen::Isometry3d master_lelbow_pose_ppre_;
    Eigen::Isometry3d master_relbow_pose_ppre_;
    Eigen::Isometry3d master_lshoulder_pose_ppre_;
    Eigen::Isometry3d master_rshoulder_pose_ppre_;
    Eigen::Isometry3d master_upperbody_pose_ppre_;

    Eigen::Vector6d master_lhand_vel_;
    Eigen::Vector6d master_rhand_vel_;
    Eigen::Vector6d master_head_vel_;
    Eigen::Vector6d master_lelbow_vel_;
    Eigen::Vector6d master_relbow_vel_;
    Eigen::Vector6d master_lshoulder_vel_;
    Eigen::Vector6d master_rshoulder_vel_;
    Eigen::Vector6d master_upperbody_vel_;

    Eigen::Vector3d master_lhand_rqy_;
    Eigen::Vector3d master_rhand_rqy_;
    Eigen::Vector3d master_lelbow_rqy_;
    Eigen::Vector3d master_relbow_rqy_;
    Eigen::Vector3d master_lshoulder_rqy_;
    Eigen::Vector3d master_rshoulder_rqy_;
    
    Eigen::Vector3d master_head_rqy_;

    Eigen::Vector3d master_relative_lhand_pos_raw_;
    Eigen::Vector3d master_relative_rhand_pos_raw_;
    Eigen::Vector3d master_relative_lhand_pos_;
    Eigen::Vector3d master_relative_rhand_pos_;
    Eigen::Vector3d master_relative_lhand_pos_pre_;
    Eigen::Vector3d master_relative_rhand_pos_pre_;

    double robot_arm_max_l_;
    double robot_upperarm_max_l_;
    double robot_lowerarm_max_l_;
    double robot_shoulder_width_;
    ////////////HMD + VIVE TRACKER////////////
    bool hmd_init_pose_calibration_;
    // double hmd_init_pose_cali_time_;

    bool hmd_tracker_status_raw_;   //1: good, 0: bad
    bool hmd_tracker_status_;   //1: good, 0: bad
    bool hmd_tracker_status_pre_;   //1: good, 0: bad

    double tracker_status_changed_time_;
    double calibration_x_l_scale_;
    double calibration_x_r_scale_;
    double calibration_y_l_scale_;
    double calibration_y_r_scale_;
    double calibration_z_l_scale_;
    double calibration_z_r_scale_;
    

    double hmd_larm_max_l_;
    double hmd_rarm_max_l_;
    double hmd_shoulder_width_;

    bool hmd_check_pose_calibration_[5];   // 0: still cali, 1: T pose cali, 2: Forward Stretch cali, 3: Calibration is completed, 4: read calibration from log file
    bool still_pose_cali_flag_;
    bool t_pose_cali_flag_;
    bool forward_pose_cali_flag_;
    bool read_cali_log_flag_;

    Eigen::Isometry3d hmd_head_pose_raw_;
    Eigen::Isometry3d hmd_lshoulder_pose_raw_;
    Eigen::Isometry3d hmd_lupperarm_pose_raw_;
    Eigen::Isometry3d hmd_lhand_pose_raw_;
    Eigen::Isometry3d hmd_rshoulder_pose_raw_;
    Eigen::Isometry3d hmd_rupperarm_pose_raw_;
    Eigen::Isometry3d hmd_rhand_pose_raw_;
    Eigen::Isometry3d hmd_chest_pose_raw_;
    Eigen::Isometry3d hmd_pelv_pose_raw_;

    Eigen::Isometry3d hmd_head_pose_raw_last_;
    Eigen::Isometry3d hmd_lshoulder_pose_raw_last_;
    Eigen::Isometry3d hmd_lupperarm_pose_raw_last_;
    Eigen::Isometry3d hmd_lhand_pose_raw_last_;
    Eigen::Isometry3d hmd_rshoulder_pose_raw_last_;
    Eigen::Isometry3d hmd_rupperarm_pose_raw_last_;
    Eigen::Isometry3d hmd_rhand_pose_raw_last_;
    Eigen::Isometry3d hmd_chest_pose_raw_last_;
    Eigen::Isometry3d hmd_pelv_pose_raw_last_;

    Eigen::Isometry3d hmd_head_pose_;
    Eigen::Isometry3d hmd_lshoulder_pose_;
    Eigen::Isometry3d hmd_lupperarm_pose_;
    Eigen::Isometry3d hmd_lhand_pose_;
    Eigen::Isometry3d hmd_rshoulder_pose_;
    Eigen::Isometry3d hmd_rupperarm_pose_;
    Eigen::Isometry3d hmd_rhand_pose_;
    Eigen::Isometry3d hmd_chest_pose_;
    Eigen::Isometry3d hmd_pelv_pose_;

    Eigen::Isometry3d hmd_head_pose_pre_;
    Eigen::Isometry3d hmd_lshoulder_pose_pre_;
    Eigen::Isometry3d hmd_lupperarm_pose_pre_;
    Eigen::Isometry3d hmd_lhand_pose_pre_;
    Eigen::Isometry3d hmd_rshoulder_pose_pre_;
    Eigen::Isometry3d hmd_rupperarm_pose_pre_;
    Eigen::Isometry3d hmd_rhand_pose_pre_;
    Eigen::Isometry3d hmd_chest_pose_pre_;
    Eigen::Isometry3d hmd_pelv_pose_pre_;

    Eigen::Isometry3d hmd_head_pose_init_;  
    Eigen::Isometry3d hmd_lshoulder_pose_init_; 
    Eigen::Isometry3d hmd_lupperarm_pose_init_;
    Eigen::Isometry3d hmd_lhand_pose_init_;
    Eigen::Isometry3d hmd_rshoulder_pose_init_; 
    Eigen::Isometry3d hmd_rupperarm_pose_init_;
    Eigen::Isometry3d hmd_rhand_pose_init_;
    Eigen::Isometry3d hmd_chest_pose_init_;
    Eigen::Isometry3d hmd_pelv_pose_init_;

    Eigen::Vector3d hmd2robot_lhand_pos_mapping_;
    Eigen::Vector3d hmd2robot_rhand_pos_mapping_;
    Eigen::Vector3d hmd2robot_lelbow_pos_mapping_;
    Eigen::Vector3d hmd2robot_relbow_pos_mapping_;

    Eigen::Vector3d hmd2robot_lhand_pos_mapping_init_;
    Eigen::Vector3d hmd2robot_rhand_pos_mapping_init_;

    Eigen::Vector3d hmd_still_cali_lhand_pos_;
    Eigen::Vector3d hmd_still_cali_rhand_pos_;
    Eigen::Vector3d hmd_tpose_cali_lhand_pos_;
    Eigen::Vector3d hmd_tpose_cali_rhand_pos_;
    Eigen::Vector3d hmd_forward_cali_lhand_pos_;
    Eigen::Vector3d hmd_forward_cali_rhand_pos_;
    
    Eigen::Vector3d hmd_lshoulder_center_pos_;
    Eigen::Vector3d hmd_rshoulder_center_pos_;

    Eigen::Vector3d hmd_chest_2_lshoulder_center_pos_;
    Eigen::Vector3d hmd_chest_2_rshoulder_center_pos_;

    //global frame of vive tracker
    Eigen::Vector6d hmd_head_vel_global_;
    Eigen::Vector6d hmd_lupperarm_vel_global_;
    Eigen::Vector6d hmd_lhand_vel_global_;
    Eigen::Vector6d hmd_rupperarm_vel_global_;
    Eigen::Vector6d hmd_rhand_vel_global_;
    Eigen::Vector6d hmd_chest_vel_global_;
    Eigen::Vector6d hmd_pelv_vel_global_; 

    //pelvis frame
    Eigen::Vector6d hmd_head_vel_;
    Eigen::Vector6d hmd_lshoulder_vel_;
    Eigen::Vector6d hmd_lupperarm_vel_;
    Eigen::Vector6d hmd_lhand_vel_;
    Eigen::Vector6d hmd_rshoulder_vel_;
    Eigen::Vector6d hmd_rupperarm_vel_;
    Eigen::Vector6d hmd_rhand_vel_;
    Eigen::Vector6d hmd_chest_vel_;
    Eigen::Vector6d hmd_pelv_vel_; 
    
    int hmd_head_abrupt_motion_count_;
    int hmd_lupperarm_abrupt_motion_count_;
    int hmd_lhand_abrupt_motion_count_;
    int hmd_rupperarm_abrupt_motion_count_;
    int hmd_rhand_abrupt_motion_count_;
    int hmd_chest_abrupt_motion_count_;
    int hmd_pelv_abrupt_motion_count_;

    ///////////QPIK///////////////////////////
    Eigen::Vector3d lhand_pos_error_;
    Eigen::Vector3d rhand_pos_error_;
    Eigen::Vector3d lhand_ori_error_;
    Eigen::Vector3d rhand_ori_error_;

    Eigen::Vector3d lelbow_ori_error_;
    Eigen::Vector3d relbow_ori_error_;
    Eigen::Vector3d lshoulder_ori_error_;
    Eigen::Vector3d rshoulder_ori_error_;

    Eigen::Vector6d lhand_vel_error_;
    Eigen::Vector6d rhand_vel_error_;
    Eigen::Vector3d lelbow_vel_error_;
    Eigen::Vector3d relbow_vel_error_;
    Eigen::Vector3d lacromion_vel_error_;
    Eigen::Vector3d racromion_vel_error_;
    //////////////////////////////////////////

    /////////////QPIK UPPERBODY /////////////////
    const int hierarchy_num_upperbody_ = 4;
    const int variable_size_upperbody_ = 21;
    const int constraint_size1_upperbody_ = 21; //[lb <=    x   <=  ub] form constraints
    const int constraint_size2_upperbody_ = 12; //[lb <=    Ax  <=  ub] from constraints
    const int control_size_upperbody_[4] = {3, 14, 4, 4};       //1: upperbody, 2: head + hand, 3: upperarm, 4: shoulder

    // const int control_size_hand = 12;        //2
    // const int control_size_upperbody = 3;    //1
    // const int control_size_head = 2;     //2
    // const int control_size_upperarm = 4;     //3
    // const int control_size_shoulder = 4; //4

    double w1_upperbody_;
    double w2_upperbody_;
    double w3_upperbody_;
    double w4_upperbody_;
    double w5_upperbody_;
    double w6_upperbody_;

    Eigen::MatrixXd H_upperbody_, A_upperbody_;
    Eigen::MatrixXd J_upperbody_[4];
    Eigen::VectorXd g_upperbody_, u_dot_upperbody_[4], qpres_upperbody_, ub_upperbody_, lb_upperbody_, ubA_upperbody_, lbA_upperbody_;
    Eigen::VectorXd q_dot_upperbody_;

    Eigen::MatrixXd N1_upperbody_, N2_aug_upperbody_, N3_aug_upperbody_;
    Eigen::MatrixXd I3_upperbody_, I6_upperbody_, I15_upperbody_, I21_upperbody_;
    /////////////////////////////////////////////

    /////////////HQPIK//////////////////////////
    const unsigned int hierarchy_num_hqpik_ = 4;
    const unsigned int variable_size_hqpik_ = 21;
    const unsigned int constraint_size1_hqpik_ = 21;    //[lb <=    x   <=  ub] form constraints
    const unsigned int constraint_size2_hqpik_[4] = {12, 15, 17, 21};   //[lb <=    Ax  <=  ub] or [Ax = b]
    const unsigned int control_size_hqpik_[4] = {3, 14, 4, 4};      //1: upperbody, 2: head + hand, 3: upperarm, 4: shoulder

    double w1_hqpik_[4];
    double w2_hqpik_[4];
    double w3_hqpik_[4];
    double w4_hqpik_[4];
    double w5_hqpik_[4];
    double w6_hqpik_[4];
    
    Eigen::MatrixXd H_hqpik_[4], A_hqpik_[4];
    Eigen::MatrixXd J_hqpik_[4], J_temp_;
    Eigen::VectorXd g_hqpik_[4], u_dot_hqpik_[4], qpres_hqpik_, ub_hqpik_[4],lb_hqpik_[4], ubA_hqpik_[4], lbA_hqpik_[4];
    Eigen::VectorXd q_dot_hqpik_[4];

    int last_solved_hierarchy_num_;
    const double equality_condition_eps_ = 1e-8;
    const double damped_puedoinverse_eps_ = 1e-5;
    ///////////////////////////////////////////////////
    
    /////////////HQPIK2//////////////////////////
    const int hierarchy_num_hqpik2_ = 5;
    const int variable_size_hqpik2_ = 21;
    const int constraint_size1_hqpik2_ = 21;    //[lb <=    x   <=  ub] form constraints
    const int constraint_size2_hqpik2_[5] = {12, 16, 19, 19, 23};   //[lb <=    Ax  <=  ub] or [Ax = b]
    const int control_size_hqpik2_[5] = {4, 3, 12, 4, 4};       //1: head ori(2)+pos(2), 2: hand, 3: upper body ori, 4: upper arm ori(2) 5: shoulder ori(2)

    double w1_hqpik2_[5];
    double w2_hqpik2_[5];
    double w3_hqpik2_[5];
    double w4_hqpik2_[5];
    double w5_hqpik2_[5];
    double w6_hqpik2_[5];
    
    Eigen::MatrixXd H_hqpik2_[5], A_hqpik2_[5];
    Eigen::MatrixXd J_hqpik2_[5];
    Eigen::VectorXd g_hqpik2_[5], u_dot_hqpik2_[5], qpres_hqpik2_, ub_hqpik2_[5],lb_hqpik2_[5], ubA_hqpik2_[5], lbA_hqpik2_[5];
    Eigen::VectorXd q_dot_hqpik2_[5];
    /////////////////////////////////////////////////////

    ////////////QP RETARGETING//////////////////////////////////
    const int variable_size_retargeting_ = 6;
    const int constraint_size1_retargeting_ = 6;    //[lb <=    x   <=  ub] form constraints
    const int constraint_size2_retargeting_[3] = {6, 6, 9}; //[lb <=    Ax  <=  ub] from constraints
    const int control_size_retargeting_[3] = {3, 3, 3};     //1: left hand, 2: right hand, 3: relative hand

    Eigen::Vector3d robot_still_pose_lhand_, robot_t_pose_lhand_, robot_forward_pose_lhand_, robot_still_pose_rhand_, robot_t_pose_rhand_, robot_forward_pose_rhand_;
    Eigen::Vector3d lhand_mapping_vector_, rhand_mapping_vector_, lhand_mapping_vector_pre_, rhand_mapping_vector_pre_, lhand_mapping_vector_dot_, rhand_mapping_vector_dot_;
    Eigen::MatrixXd lhand_master_ref_stack_, lhand_robot_ref_stack_, rhand_master_ref_stack_, rhand_robot_ref_stack_, lhand_master_ref_stack_pinverse_, rhand_master_ref_stack_pinverse_;
    
    Eigen::MatrixXd H_retargeting_lhand_,  A_retargeting_lhand_, H_retargeting_rhand_, A_retargeting_rhand_;
    Eigen::VectorXd qpres_retargeting_[3], g_retargeting_lhand_, g_retargeting_rhand_, ub_retargeting_, lb_retargeting_, ubA_retargeting_[3], lbA_retargeting_[3];
    Eigen::MatrixXd E1_, E2_, E3_, H_retargeting_, A_retargeting_[3];
    Eigen::VectorXd g_retargeting_, u1_, u2_, u3_;

    Eigen::Vector3d h_d_lhand_, h_d_rhand_, h_pre_lhand_, h_pre_rhand_, r_pre_lhand_, r_pre_rhand_;
    double w1_retargeting_, w2_retargeting_, w3_retargeting_, human_shoulder_width_; 
    const double control_gain_retargeting_ = 100;
    const double human_vel_min_ = -2;
    const double human_vel_max_ = 2;
    const double w_dot_min_ = -30;
    const double w_dot_max_ = 30;

    ////////////////////////////////////////////////////////////

    /////////////CAM-HQP//////////////////////////
    const int hierarchy_num_camhqp_ = 2;
    const int variable_size_camhqp_ = 8; // original number -> 6 (DG)
    const int constraint_size1_camhqp_ = 8; //[lb <=    x   <=  ub] form constraints // original number -> 6 (DG)
    //const int constraint_size2_camhqp_[2] = {0, 3};   //[lb <=    Ax  <=  ub] or [Ax = b]/ 0223 except Z axis control
    const int constraint_size2_camhqp_[2] = {0, 2}; //[lb <=    Ax  <=  ub] or [Ax = b] 
    //const int control_size_camhqp_[2] = {3, 8}; //1: CAM control, 2: init pose // original number -> 6 (DG)
    const int control_size_camhqp_[2] = {2, 8}; //1: CAM control, 2: init pose // original number -> 6 (DG) / 0223 except Z axis control

    double w1_camhqp_[2];
    double w2_camhqp_[2];
    double w3_camhqp_[2];
    
    Eigen::MatrixXd H_camhqp_[2], A_camhqp_[2];
    Eigen::MatrixXd J_camhqp_[2];
    Eigen::VectorXd g_camhqp_[2], u_dot_camhqp_[2], qpres_camhqp_, ub_camhqp_[2],lb_camhqp_[2], ubA_camhqp_[2], lbA_camhqp_[2];
    Eigen::VectorXd q_dot_camhqp_[2];

    int control_joint_idx_camhqp_[8]; // original number -> 6 (DG)
    int last_solved_hierarchy_num_camhqp_;
    unsigned int torque_flag_x = 0, torque_flag_y = 0; 
    ///////////////////////////////////////////////////

    //fallDetection variables
    Eigen::VectorQd fall_init_q_;
    double fall_start_time_;
    int foot_lift_count_;
    int foot_landing_count_;
    ///////////////////////////////////////////////////////////////

private:
    Eigen::VectorQd ControlVal_;
        
    void initWalkingParameter();

    //Arm controller
    Eigen::VectorXd joint_limit_l_;
    Eigen::VectorXd joint_limit_h_;
    Eigen::VectorXd joint_vel_limit_l_;
    Eigen::VectorXd joint_vel_limit_h_;

    Eigen::Vector3d left_x_traj_pre_;
    Eigen::Vector3d right_x_traj_pre_;
    Eigen::Matrix3d left_rotm_pre_;
    Eigen::Matrix3d right_rotm_pre_;

    Eigen::VectorQVQd q_virtual_clik_;
    Eigen::Vector8d integral;

    bool first_loop_larm_;
    bool first_loop_rarm_;
    bool first_loop_upperbody_;
    bool first_loop_hqpik_;
    bool first_loop_hqpik2_;
    bool first_loop_qp_retargeting_;
    bool first_loop_camhqp_;
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////MJ CustomCuntroller//////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
public:
    //////////////////////////////// Myeong-Ju
    void circling_motion();
    void computeIkControl_MJ(Eigen::Isometry3d float_trunk_transform, Eigen::Isometry3d float_lleg_transform, Eigen::Isometry3d float_rleg_transform, Eigen::Vector12d& desired_leg_q);
    void Joint_gain_set_MJ();
    void updateInitialState();
    void updateNextStepTime();
    void parameterSetting();
    void getRobotState();
    void calculateFootStepTotal();
    void calculateFootStepTotal_MJ();
    void supportToFloatPattern();
    void floatToSupportFootstep();
    void GravityCalculate_MJ();
    void setContact_custom();
    
    void getZmpTrajectory();
    void zmpGenerator(const unsigned int norm_size, const unsigned planning_step_num);
    void onestepZmp(unsigned int current_step_number, Eigen::VectorXd& temp_px, Eigen::VectorXd& temp_py);
    void getComTrajectory();
    void getMPCTrajectory();
    void getMPCTrajectoryInit();
    void getFootTrajectory();
    void getPelvTrajectory();
    void previewcontroller(double dt, int NL, int tick, double x_i, double y_i, Eigen::Vector3d xs, Eigen::Vector3d ys, double& UX, double& UY, 
    Eigen::MatrixXd Gi, Eigen::VectorXd Gd, Eigen::MatrixXd Gx, Eigen::MatrixXd A, Eigen::VectorXd B, Eigen::MatrixXd C, Eigen::Vector3d &XD, Eigen::Vector3d &YD);  
    void preview_Parameter(double dt, int NL, Eigen::MatrixXd& Gi, Eigen::VectorXd& Gd, Eigen::MatrixXd& Gx, Eigen::MatrixXd& A, Eigen::VectorXd& B, Eigen::MatrixXd& C);
    void addZmpOffset();
    void hip_compensator();
    void Compliant_control(Eigen::Vector12d desired_leg_q);
    
    void SC_err_compen(double x_des, double y_des);

    void CP_compen_MJ();
    void CP_compen_MJ_FT();
    void CLIPM_ZMP_compen_MJ(double XZMP_ref, double YZMP_ref);
    void momentumControl(RobotData &Robot, Eigen::Vector3d comd,  Eigen::Vector2d ang_, Eigen::Vector3d rfootd, Eigen::Vector3d lfootd, Eigen::Vector2d upperd, Eigen::Vector3d rfootori, Eigen::Vector3d lfootori);

    //momentumControl
    bool qp_solved;
    Eigen::VectorXd qp_result1, q_dm;
    Eigen::VectorVQd q_dm_test;
    double a_temp1 = 0.0;
    Eigen::MatrixXd J2, H2, A2;
    Eigen::VectorXd X2, g2, lb2, ub2, lbA2, ubA2;
    int variable_size2, constraint_size2;

    Eigen::Matrix6d PELV_YaW6D;
    Eigen::Matrix6d sup_YaW6D;
    Eigen::MatrixXd Hg;
    Eigen::MatrixXd Hg_;
    Eigen::MatrixXd Hg_slow_;

    Eigen::Vector3d hg;
    Eigen::Vector3d hg_;
    Eigen::Vector3d hg_slow_;

    Eigen::Vector2d upper_d;
    bool upper_on = true;
    bool walking_tick_stop = true;
    bool step_change = false;
    Eigen::VectorVQd qd_pinocchio_, q_dm_mom;
    Eigen::VectorVQd qd_pinocchio;
    Eigen::VectorQVQd q_pinocchio_desired;
    Eigen::VectorQVQd q_pinocchio_desired1, qd_pinocchio1;

    Eigen::Vector3d rfootd, lfootd, rfootd1, lfootd1, comd, comd1, comd_init, com_mpc, com_mpc1, com_mpc2, comprev, rfoot_mpc, lfoot_mpc;
    Eigen::Vector2d angm, angm_prev, upperd, comdt_, comd_s;
    Eigen::Vector3d ZMP_gl;

    Eigen::Vector3d zmp_temp1;
    double ZMPx_prev, ZMPy_prev, zmp_mpcx, zmp_mpcy, com_mpcx, com_mpcy, com_mpcy_prev, com_mpcx_prev, ZMPx_test, ZMPy_test;
    int controlwalk_time;

    Eigen::Vector3d virtual_temp, virtual_temp_sup, virtual_temp1, virtual_temp2, foot_temp;

    double U_ZMP_y_ssp = 0;
    double U_ZMP_y_ssp_LPF = 0;
    double U_ZMP_x_ssp = 0;
    double U_ZMP_x_ssp_LPF = 0;
    double damping_x = 0;
    double damping_y = 0;
    Eigen::Vector2d Tau_R;
    Eigen::Vector2d Tau_L;
    Eigen::Matrix6Vd comj, J_LFF, J_RFF;

    CQuadraticProgram qp_momentum_control;

    Eigen::VectorQd Tau_CP;

    Eigen::Vector2d q_upper;

    Eigen::Vector12d pre_motor_q_leg_;
    Eigen::Vector12d current_motor_q_leg_;
    Eigen::Vector12d d_hat_b;
    Eigen::Vector12d DOB_IK_output_b_;
    Eigen::Vector12d DOB_IK_output_;
    Eigen::VectorQd ref_q_;
    Eigen::VectorQd ref_q_fast_;
    Eigen::VectorQd Kp;
    Eigen::VectorQd Kd;
    Eigen::VectorQd desired_q_not_compensated_, desired_q_not_compensated_1;

    Eigen::VectorXd com_prev;
    
    Eigen::VectorQd q_prev_MJ_;

    Eigen::Vector12d q_des_;
    Eigen::Vector12d q_des_1;
    Eigen::Vector3d q_des_virtual;
    Eigen::Vector12d qd_des_;
    Eigen::Vector2d qdd_des_upper;
    Eigen::Vector2d upperd_prev;
    Eigen::Vector3d qd_des_virtual;
    Eigen::Vector12d qdd_des_;
    Eigen::Vector2d qdd_des_upper_lpf;
    Eigen::Vector3d qdd_des_virtual;
    Eigen::Vector3d qdd_des_virtual_ori;
    Eigen::Vector3d qd_des_ori_prev;
    Eigen::Vector12d qd_des_prev;
    Eigen::Vector3d qd_des_virtual_prev;
    Eigen::Vector12d q_des_prev;
    Eigen::Vector3d q_des_virtual_prev;
    Eigen::Vector12d qd_des_lpf;
    Eigen::Vector12d qdd_des_lpf;
    Eigen::Vector3d qd_des_virtual_lpf;
    Eigen::Vector3d qdd_des_virtual_lpf;
    Eigen::Vector3d qdd_des_virtual_ori_lpf;
    Eigen::VectorVQd qdd_des_virtual_prev;
    Eigen::VectorVQd qdd_des_virtual_;
    Eigen::VectorVQd qdd_des_virtual_1;
    Eigen::VectorVQd qdd_des_virtual_fast;
    
    Eigen::Isometry3d pelv_trajectory_support_, pelv_trajectory_support_1; //local frame
    Eigen::Isometry3d rfoot_trajectory_support_, rfoot_trajectory_support_1;  //local frame
    Eigen::Isometry3d lfoot_trajectory_support_, lfoot_trajectory_support_1;
    Eigen::Vector3d rfoot_trajectory_euler_support_;
    Eigen::Vector3d lfoot_trajectory_euler_support_;

    Eigen::Isometry3d pelv_trajectory_float_; //pelvis frame
    Eigen::Isometry3d rfoot_trajectory_float_;
    Eigen::Isometry3d lfoot_trajectory_float_;

    Eigen::Isometry3d pelv_trajectory_float_1; //pelvis frame
    Eigen::Isometry3d rfoot_trajectory_float_1;
    Eigen::Isometry3d lfoot_trajectory_float_1;

    Eigen::Isometry3d lfoot_trajectory_float_fast_;
    Eigen::Isometry3d lfoot_trajectory_float_slow_;

    Eigen::Isometry3d rfoot_trajectory_float_fast_;
    Eigen::Isometry3d rfoot_trajectory_float_slow_;

    Eigen::Vector3d pelv_support_euler_init_;
    Eigen::Vector3d lfoot_support_euler_init_;
    Eigen::Vector3d rfoot_support_euler_init_;

    Eigen::Vector2d del_cmp;
    Eigen::Vector3d del_tau_;
    Eigen::Vector3d del_ang_momentum_;
    Eigen::Vector3d del_ang_momentum_prev_;

    Eigen::Vector3d del_ang_momentum_slow_;
    Eigen::Vector3d del_ang_momentum_fast_;

    Eigen::VectorQd del_cmm_q_;
    unsigned int cmp_control_mode = 0;

    Eigen::Isometry3d pelv_support_start_;
    Eigen::Isometry3d pelv_support_init_;
    Eigen::Vector2d del_zmp;
    Eigen::Vector2d cp_desired_;
    Eigen::Vector2d cp_measured_;
    Eigen::Vector2d cp_measured_LPF;
    Eigen::Vector3d com_support_init_;
    Eigen::Vector3d com_float_init_;
    Eigen::Vector3d com_float_current_;
    Eigen::Vector3d com_support_current_;
    Eigen::Vector3d com_support_current_dot_;
    Eigen::Vector3d com_support_current_LPF;
    Eigen::Vector3d com_float_current_LPF;
    Eigen::Vector3d com_support_current_prev;
    Eigen::Vector3d com_support_cp_;

    Eigen::Vector3d com_float_current_dot;
    Eigen::Vector3d com_float_current_dot_prev;
    Eigen::Vector3d com_float_current_dot_LPF;
    Eigen::Vector3d com_support_current_dot_LPF;

    Eigen::Vector3d pelv_rpy_current_mj_;
    Eigen::Vector3d rfoot_rpy_current_;
    Eigen::Vector3d lfoot_rpy_current_;
    Eigen::Isometry3d pelv_yaw_rot_current_from_global_mj_;
    Eigen::Isometry3d rfoot_roll_rot_;
    Eigen::Isometry3d lfoot_roll_rot_;
    Eigen::Isometry3d rfoot_pitch_rot_;
    Eigen::Isometry3d lfoot_pitch_rot_;

    Eigen::Vector3d RFc_vector;
    Eigen::Vector3d LFc_vector;
    Eigen::Vector3d RFc_vector_prev;
    Eigen::Vector3d LFc_vector_prev;

    Eigen::Isometry3d pelv_float_current_;
    Eigen::Isometry3d lfoot_float_current_;
    Eigen::Isometry3d rfoot_float_current_;
    Eigen::Isometry3d pelv_float_init_;
    Eigen::Isometry3d lfoot_float_init_;
    Eigen::Isometry3d rfoot_float_init_;
    double wn = 0;

    MatrixXd J1, H1, A1;
    VectorXd X1, g1, lb1, ub1, lbA1, ubA1;
    Eigen::VectorVQd qdd_pinocchio_desired1;
    Eigen::VectorVQd qdd_pinocchio_desired1_;
    Eigen::VectorXd nonlineareff;
    int variable_size1, constraint_size1;
    CQuadraticProgram qp_torque_control;
    bool control_start = false;
    bool solved;
    std::atomic<double>  com_alpha, lfoot_sx, lfoot_sy, rfoot_sx, rfoot_sy, lfoot_sx_float, lfoot_sy_float, rfoot_sx_float, rfoot_sy_float;
    double zmpx_, zmpy_, com_alpha_;
    double zmpx_fast, zmpy_fast;
    double com_alpha_fast = 0.5;
    double zmpx_d, zmpy_d;
    bool torquecontrol_Ok = false;
    Eigen::VectorXd qp_result, nle, g1_temp;
    Eigen::VectorVQd q_dot_virtual_lpf_;
    Eigen::MatrixXd RFj, LFj, RFdj, LFdj, H1_temp;
    Eigen::MatrixVQVQd M_;
    MatrixXd G_temp;
    Vector12d g_temp;
    int control_time = 0, temp_mom = 0;
    int contactMode;
    int contactMode_; 
    int contactMode_fast = 1; 
    double supportFoot,supportFoot_,supportFoot_fast;
    Eigen::Vector2d zmp_bx;
    Eigen::Isometry3d LFc_float_current, RFc_float_current;
    Eigen::Vector6d ang_de;
    Eigen::VectorXd q_desireddot;

    Eigen::VectorQVQd q_pinocchio, q_pinocchio1, q_pinocchio1_test;

    Eigen::Vector2d sc_err_before;
    Eigen::Vector2d sc_err_after;
    Eigen::Vector2d SC_com;
    Eigen::Vector2d sc_err;

    Eigen::Vector12d sc_joint_before;
    Eigen::Vector12d sc_joint_after;
    Eigen::Vector12d SC_joint;
    Eigen::Vector12d sc_joint_err;

    
    int LFcframe_id, RFcframe_id, RFframe_id, LFframe_id, RFjoint_id, LFjoint_id;
    
    double walking_end_flag = 0;
    double foot_legnth;
    
    Eigen::Isometry3d supportfoot_float_current_; 

    Eigen::Isometry3d pelv_support_current_;
    Eigen::Isometry3d lfoot_support_current_;
    Eigen::Isometry3d rfoot_support_current_;

    Eigen::Isometry3d pelv_global_current_init;
    Eigen::Isometry3d lfoot_global_current_init;
    Eigen::Isometry3d rfoot_global_current_init;
    Eigen::Isometry3d com_global_current_init;

    Eigen::Isometry3d pelv_float_current_init;
    Eigen::Isometry3d lfoot_float_current_init;
    Eigen::Isometry3d rfoot_float_current_init;
    Eigen::Isometry3d com_float_current_init;

    Eigen::Isometry3d pelv_support_current_init;
    Eigen::Isometry3d lfoot_support_current_init;
    Eigen::Isometry3d rfoot_support_current_init;
    Eigen::Isometry3d com_support_current_init;

    Eigen::Isometry3d lfoot_support_init_;
    Eigen::Isometry3d rfoot_support_init_;
    
    Eigen::Vector6d supportfoot_support_init_offset_;
    Eigen::Vector6d supportfoot_float_init_;
    Eigen::Vector6d supportfoot_support_init_;
    Eigen::Vector6d swingfoot_float_init_;
    Eigen::Vector6d swingfoot_support_init_;

    double com_x_int = 0.0;
    double com_y_int = 0.0;

    std::atomic<int> mpc_cycle;
    std::atomic<int> walking_tick;

    Eigen::MatrixXd ref_zmp_mj_;
    Eigen::VectorXd tau_, tau_1;

    Eigen::Vector3d xs_mj_;
    Eigen::Vector3d ys_mj_;
    Eigen::Vector3d xd_mj_;
    Eigen::Vector3d yd_mj_; 
    Eigen::Vector3d preview_x_mj, preview_y_mj, preview_x_b_mj, preview_y_b_mj;

    Eigen::MatrixXd Gi_mj_;
    Eigen::MatrixXd Gx_mj_;
    Eigen::VectorXd Gd_mj_;
    Eigen::MatrixXd A_mj_;
    Eigen::VectorXd B_mj_;
    Eigen::MatrixXd C_mj_;

    Eigen::VectorQd Gravity_MJ_fast_;
    Eigen::VectorQd Gravity_MJ_;
    Eigen::VectorQd Gravity_MJ_1;
    Eigen::VectorQd Gravity_DSP_;
    Eigen::VectorQd Gravity_DSP_last_;
    Eigen::VectorQd Gravity_SSP_;
    Eigen::VectorQd Gravity_SSP_last_;
    Eigen::VectorQd q_dot_LPF_MJ;

    Eigen::Vector6d r_ft_mj_;
    Eigen::Vector6d l_ft_mj_;
    Eigen::Vector2d zmp_measured_mj_;
    Eigen::Vector2d zmp_err_;
    Eigen::Vector2d zmp_measured_LPF_;

    double P_angle_i = 0;
    double P_angle = 0;
    double P_angle_input_dot = 0;
    double P_angle_input = 0;
    double R_angle = 0;
    double R_angle_input_dot = 0;
    double R_angle_input = 0;
    double aa = 0; 

    double del_t = 0.0005;
    double xi_mj_;
    double yi_mj_;
    double zc_mj_;

    double ZMP_X_REF;
    double ZMP_X_REF_1;
    double ZMP_Y_REF;
    double ZMP_Y_REF_1;
    double ZMP_Y_REF_alpha = 0;

    double t_last_;
    double t_start_;
    double t_start_real_;
    double t_temp_;  
    double t_rest_init_;
    double t_rest_init_1;
    double t_rest_last_1;
    double t_rest_init_2;
    double t_rest_last_2;
    double t_mpc_init_;
    double t_rest_last_;
    double t_double1_;
    double t_double2_;
    double t_total_;
    double foot_height_;
    int total_step_num_;
    int current_step_num_;

    Eigen::Vector2d ang_d_temp;

    double step_length_x_;
    double step_length_y_;
    double target_theta_;
    double target_x_;
    double target_y_;
    double target_z_;
    double com_height_;
    int is_right_foot_swing_;

    bool time_tick = false;
    bool time_tick_next = true;
    std::chrono::system_clock::time_point startTime;
    std::chrono::system_clock::time_point startTime1;
    std::chrono::system_clock::time_point endTime;

    double zmp_start_time_mj_;
    double UX_mj_, UY_mj_; 
    Eigen::Vector3d com_desired_;
    Eigen::Vector3d com_dot_desired_;
    Eigen::MatrixXd foot_step_;
    Eigen::MatrixXd foot_step_support_frame_;
    Eigen::MatrixXd foot_step_support_frame_offset_;

    // Com damping control - ZMP tracking controller
    Eigen::MatrixXd A_y_ssp;
    Eigen::MatrixXd B_y_ssp;
    Eigen::MatrixXd Ad_y_ssp;
    Eigen::MatrixXd Bd_y_ssp;
    Eigen::MatrixXd C_y_ssp;
    Eigen::MatrixXd D_y_ssp;
    Eigen::MatrixXd K_y_ssp;
    Eigen::MatrixXd Y_y_ssp;
    Eigen::Vector2d X_y_ssp;
    
    Eigen::MatrixXd A_x_ssp;
    Eigen::MatrixXd B_x_ssp;

    Eigen::MatrixXd Ad_x_ssp;
    Eigen::MatrixXd Bd_x_ssp;
    Eigen::MatrixXd C_x_ssp;
    Eigen::MatrixXd D_x_ssp;
    Eigen::MatrixXd K_x_ssp;
    Eigen::MatrixXd Y_x_ssp;
    Eigen::Vector2d X_x_ssp;
    Eigen::MatrixXd ff_gain_y_ssp;
    Eigen::MatrixXd ff_gain_x_ssp;
    //
    Eigen::VectorQd contact_torque_MJ;
    Eigen::VectorQd Initial_ref_q_;
    Eigen::VectorQd Initial_ref_upper_q_;
    Eigen::VectorQd Initial_current_q_;
    Eigen::VectorQd Initial_ref_q_walk_;
    bool walking_enable_ ;

    int new_socket;
    bool mpc_start_init_bool = false;
    bool mpc_start_init_bool1 = false;
    bool mpc_start_init_bool2 = false;
    bool mpc_start_init_bool3 = false;

    std::atomic<int> mpc_start_init_;
    std::atomic<int> statemachine_;

    int socket_send, socket_receive;

    std::mutex thread1_lock, thread2_lock;

    
    Eigen::VectorXd desired_val_;
    Eigen::VectorXd desired_val_slow;
    Eigen::VectorXd state_init_;
    Eigen::VectorXd state_init_slow;
    Eigen::VectorXd state_init_mu;
    Eigen::VectorXd desired_val_mu;

    double buffer[51] = {1.0,  0.00000000e+00,  0.00000000e+00,  8.24730000e-01,  0.00000000e+00,
    0.00000000e+00,  0.00000000e+00,  1.00000000e+00,  0.00000000e+00,
    0.00000000e+00, -5.50000000e-01,  1.26000000e+00, -7.10000000e-01,
    0.00000000e+00,  0.00000000e+00,  0.00000000e+00, -5.50000000e-01,
    1.26000000e+00, -7.10000000e-01,  0.00000000e+00,  0.00000000e+00,
    0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
    0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
    0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
    0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
    0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
    0.00000000e+00,  8.61938268e-02,  0.00000000e+00,  8.61938268e-02,
    0.00000000e+00,  5.18219890e-06,  0.00000000e+00,  5.18219890e-06,
    0.00000000e+00, 8.06938795e-01};

    double buffer1[50] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    Eigen::Vector3d pelv_vtran, rf_vtran, lf_vtran;
    int delay_time = 0;

    //pedal_
    ros::NodeHandle nh;
    ros::Subscriber pedal_command;
    void PedalCommandCallback(const tocabi_msgs::WalkingCommandConstPtr &msg);
    Eigen::Vector4d joystick_input;
    Eigen::Vector4d joystick_input_;

    //// joystick&pedal Footstep
    void updateInitialStateJoy();
    void calculateFootStepTotal_MJoy();
    void calculateFootStepTotal_MJoy_End();
    void updateNextStepTimeJoy();
    void comGainTrajectory();

    double alpha_temp1 = 0;
    double alpha;
    int joy_index_ = 0;
    int walking_tick_jk = 0;
    Eigen::MatrixXd foot_step_joy_temp_;
    bool joy_enable_ = false;
    bool joy_input_enable_ = false;
    int a_temp = 0;

    Eigen::VectorQd q_mj;
    Eigen::VectorQd q_mj_prev;

    sockaddr_in serveraddr;
    sockaddr_in serveraddr1;


private:    
    //////////////////////////////// Myeong-Ju
    unsigned int walking_tick_mj = 0;
    
    unsigned int initial_tick_mj = 0;
    unsigned int initial_flag = 0;
    const double hz_ = 2000.0;  
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