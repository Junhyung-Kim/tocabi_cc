#include "tocabi_lib/robot_data.h"
#include "wholebody_functions.h"
#include <vector>
#include <array>
#include <string>
#include <time.h>
#include "math_type_define.h"
#include "hpipm_d_ocp_qp_dim.h"
#include "hpipm_d_ocp_qp_sol.h"
#include "hpipm_d_ocp_qp_utils.h"
#include "d_tools.c"


class CustomController
{
public:
    CustomController(RobotData &rd);
    Eigen::VectorQd getControl();

    //void taskCommandToCC(TaskCommand tc_);
    
    void computeSlow();
    void computeFast();
    void computePlanner();
    void copyRobotData(RobotData &rd_l);

    RobotData &rd_;
    RobotData rd_cc_;

    //Ui WalkingParameter
    double Hz_;
    double dt;
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
    double t_imp;
    double foot_height;
    double zc;
    double lipm_w;

    //walking
    void walkingCompute();
    void getRobotInitState();
    void footStepGenerator();
    void footStepTotal();
    void getRobotState();
    void calcRobotState();
    void setCpPosition();

    int walking_tick = 0;
    double contactMode;

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
    Eigen::Vector3d foot_distance;
    Eigen::Vector3d COMV_support_currentV;
    Eigen::Vector3d yx_vibm;
    Eigen::Vector3d yy_vibm;

    Eigen::Matrix3x12d Ag_leg;
    Eigen::Matrix3x8d Ag_armR;
    Eigen::Matrix3x8d Ag_armL;
    Eigen::Matrix3x3d Ag_waist;
    Eigen::Matrix3x12d Agl_leg;
    Eigen::Matrix3x8d Agl_armR;
    Eigen::Matrix3x8d Agl_armL;
    Eigen::Matrix3x3d Agl_waist;

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

    //pinocchiio
    Eigen::VectorQd q_; 
    Eigen::VectorQd qdot, qddot, qdot_, qddot_;
    Eigen::MatrixXd CMM;

    //Joint velocity Estimator

    //GravityCompensation & redistribution
    Eigen::VectorQd TorqueGrav;

    //WholebodyController &wbc_;
    //TaskCommand tc;
    
    //MPC
    void FlywheelModel(double Ts, int nx, int nu, double *A, double *B);
    void mpcVariableInit();
    void mpcSolverSetup();
    void mpcModelSetup();

    int cycle = 0;
    double Ts = 0.01;
    int mpc_init = true;
    int nx_;
    int nu_;
    int N;
    double timeHorizon = 1.1;
    size_t K;
    time_t start, start1, endt;
    int ii, jj;
    int *nx, *nu, *nbu, *nbx, *nb, *ng, *nsbx, *nsbu, *nsg, *ns, *nbxe, *idxbx1, *idxbu1, *idxbx0, *idxbu0, *idxs0, *idxs1, *idxsN;
    double *Ax, *Bx, *bx, *x0x, *Qx, *Rx, *Sx, *qx, *rx, *d_ubu1, *d_lbu1, *d_lg1, *d_ug1, *d_ubx1, *d_lbx1, *d_ubx0, *d_lbu0, *d_ubu0, *d_lg0, *d_ug0, *d_lbx0;
    int rep;
    double **hAx, **hBx, **hbx, **hQx, **hSx, **hRx, **hqx, **hrx, **hd_lbxx, **hd_ubxx, **hd_lbux, **hd_ubux, **hCx, **hDx, **hd_lgx, **hd_ugx, **hZlx, **hZu, **hzl, **hzu, **hd_ls, **hd_us;
    double *d_lbxN, *d_ubxN, *d_lgN, *d_ugN, *C0x, *D0x, *C1x, *D1x, *CNx, *DNx, *Zl0x, *Zu0x, *zl0x, *zu0x, *d_ls0x, *d_us0x, *Zl1x, *Zu1x, *zl1x, *zu1x, *d_ls1x, *d_us1x, *ZlNx, *ZuNx, *zlNx, *zuNx, *d_lsNx, *d_usNx;
    int **hidxbx, **hidxbu, **hidxs, *idxbxN;;
    double mu0;
    double **ux, **xx, **lsx, **usx, **pix, **lam_lbx, **lam_lgx, **lam_ubx, **lam_ugx, **lam_lsx, **lam_usx, *x11x, *slx, *sux;
    struct d_ocp_qp_dim dimx;
    hpipm_size_t ipm_arg_sizex;
    hpipm_size_t dim_sizex;    
    hpipm_size_t qp_sol_sizex;
    hpipm_size_t ipm_sizex;
    void *qp_sol_memx;
    void *ipm_arg_memx;
    void *dim_memx;  
    void *ipm_memx;
    struct d_ocp_qp_ipm_arg argx;
    struct d_ocp_qp_sol qp_solx;
    struct d_ocp_qp_ipm_ws workspacex;
    hpipm_size_t qp_sizex;
    void *qp_memx;
    struct d_ocp_qp qpx;
    int hpipm_statusx; // 0 normal; 1 max iter

private:
    Eigen::VectorQd ControlVal_;
};



