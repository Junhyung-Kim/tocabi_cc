#include <vector>
#include <array>
#include <string>
#include <chrono>
#include <thread>
#include "hpipm_d_ocp_qp_dim.h"
#include "hpipm_d_ocp_qp_sol.h"
#include "hpipm_d_ocp_qp_utils.h"
#include "math_type_define.h"
#include "d_tools.c"
#include "walking.h"
#include "std_msgs/String.h"

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
    void jointVelocityEstimate();
    void flyWheelModel(double Ts, int nx, int nu, double *Ax, double *Bx, double *Ay, double *By);
    void mpcVariableInit();
    void mpcModelSetup();
    void momentumControl(RobotData &Robot);
    void zmpControl(RobotData &Robot);

    void mpc_variablex();
    void mpc_variabley();
    void walking_x();
    void walking_y();

    std::future<void> solverx;
    std::future<void> solvery;

    //Joint velocity Estimator
    bool velEst = false;
    std::atomic<bool> velEst_f;
    Eigen::VectorQd q_est, q_dot_est, q_dot_est_mu;

    bool mpc_s = false;

    RobotData &rd_;
    RobotData rd_cc_;

    //pinocchiio
    Eigen::VectorQd q_;
    Eigen::VectorQd qdot, qddot, qdot_, qddot_;
    Eigen::MatrixXd CMM;
    Eigen::MatrixQQd Cor_;
    Eigen::VectorQd G_;

    //Contact Redistribution
    Eigen::VectorQd TorqueContact;
    Eigen::VectorQd TorqueGrav;
    double rate;

    std::atomic<double> com_mpcx;
    std::atomic<double> com_mpcy;

    std::vector<double> com_mpcx_mu;
    std::vector<double> com_mpcy_mu;

    std::vector<double> mom_mpcx;
    std::vector<double> mom_mpcy;

    std::vector<double> mom_mpcx_mu;
    std::vector<double> mom_mpcy_mu;

    //MPC
    std::atomic<bool> wlk_on;
    std::atomic<bool> mpc_on;

    //dist 
    ros::Publisher dist_pub;

    Eigen::Vector3d pelv_lp;
    Eigen::Vector3d pelv_lp_mu;
    Eigen::Vector3d pelv_lp_prev;
    double debug_temp;
    bool debug = false;

    int cycle = 0;
    double Ts = 0.005;
    int nx_;
    int nu_;
    double timeHorizon = 1.0;
    size_t K;
    int ii, jj;
    int *nx, *nu, *nbu, *nbx, *nb, *ng, *nsbx, *nsbu, *nsg, *ns, *idxbx1, *idxbu1, *idxbx0, *idxbu0, *idxs0, *idxs1, *idxsN, **hidxbx, **hidxbu, **hidxs, *idxbxN;
    double *Ax, *Bx, *bx, *x0x, *Qx, *Rx, *Sx, *qx, *rx, *d_ubu1x, *d_lbu1x, *d_lg1x, *d_ug1x, *d_ubx1x, *d_lbx1x, *d_ubx0x, *d_lbu0x, *d_ubu0x, *d_lg0x, *d_ug0x, *d_lbx0x;
    double *Ay, *By, *by, *x0y, *Qy, *Ry, *Sy, *qy, *ry, *d_ubu1y, *d_lbu1y, *d_lg1y, *d_ug1y, *d_ubx1y, *d_lbx1y, *d_ubx0y, *d_lbu0y, *d_ubu0y, *d_lg0y, *d_ug0y, *d_lbx0y;
    double **hAx, **hBx, **hbx, **hQx, **hSx, **hRx, **hqx, **hrx, **hd_lbxx, **hd_ubxx, **hd_lbux, **hd_ubux, **hCx, **hDx, **hd_lgx, **hd_ugx, **hZlx, **hZux, **hzlx, **hzux, **hd_lsx, **hd_usx;
    double **hAy, **hBy, **hby, **hQy, **hSy, **hRy, **hqy, **hry, **hd_lbxy, **hd_ubxy, **hd_lbuy, **hd_ubuy, **hCy, **hDy, **hd_lgy, **hd_ugy, **hZly, **hZuy, **hzly, **hzuy, **hd_lsy, **hd_usy;
    double *d_lbxNx, *d_ubxNx, *d_lgNx, *d_ugNx, *C0x, *D0x, *C1x, *D1x, *CNx, *DNx, *Zl0x, *Zu0x, *zl0x, *zu0x, *d_ls0x, *d_us0x, *Zl1x, *Zu1x, *zl1x, *zu1x, *d_ls1x, *d_us1x, *ZlNx, *ZuNx, *zlNx, *zuNx, *d_lsNx, *d_usNx;
    double *d_lbxNy, *d_ubxNy, *d_lgNy, *d_ugNy, *C0y, *D0y, *C1y, *D1y, *CNy, *DNy, *Zl0y, *Zu0y, *zl0y, *zu0y, *d_ls0y, *d_us0y, *Zl1y, *Zu1y, *zl1y, *zu1y, *d_ls1y, *d_us1y, *ZlNy, *ZuNy, *zlNy, *zuNy, *d_lsNy, *d_usNy;
    double mu0;
    double *x11x, *slx, *sux, *x11y, *sly, *suy, *s1x, *s1u, *u11x, *u11y;
    double **softCx_mu, **softCy_mu, **zmpx_mu, **zmpy_mu, **xL_mu, **xU_mu, **yL_mu, **yU_mu, **softBoundx_mu, **softBoundy_mu, **softCx1_mu, **softCy1_mu, **zmpx1_mu, **zmpy1_mu, **xL1_mu, **xU1_mu, **yL1_mu, **yU1_mu, **softBoundx1_mu, **softBoundy1_mu;
    struct d_ocp_qp_dim dimx;
    struct d_ocp_qp_dim dimy;
    hpipm_size_t ipm_arg_sizex;
    hpipm_size_t dim_sizex;
    hpipm_size_t qp_sol_sizex;
    hpipm_size_t ipm_sizex;
    hpipm_size_t ipm_arg_sizey;
    hpipm_size_t dim_sizey;
    hpipm_size_t qp_sol_sizey;
    hpipm_size_t ipm_sizey;
    void *qp_sol_memx;
    void *ipm_arg_memx;
    void *dim_memx;
    void *ipm_memx;
    void *qp_sol_memy;
    void *ipm_arg_memy;
    void *dim_memy;
    void *ipm_memy;

    struct d_ocp_qp_ipm_arg argx;
    struct d_ocp_qp_sol qp_solx;
    struct d_ocp_qp_ipm_ws workspacex;
    struct d_ocp_qp_ipm_arg argy;
    struct d_ocp_qp_sol qp_soly;
    struct d_ocp_qp_ipm_ws workspacey;
    hpipm_size_t qp_sizex;
    hpipm_size_t qp_sizey;
    void *qp_memx;
    void *qp_memy;
    struct d_ocp_qp qpx;
    struct d_ocp_qp qpy;
    int hpipm_statusx; // 0 normal; 1 max iter
    int hpipm_statusy; // 0 normal; 1 max iter
    int nx_max;
    std::atomic<int> mpc_cycle = 0;
    std::atomic<int> mpc_cycle_prev = 0;

    //momentumControl
    CQuadraticProgram QP_m;
    Eigen::VectorXd q_dm;
    Eigen::Vector5d q_w;

    //WholebodyController &wbc_;
    //TaskCommand tc;

    int aaa = 0;

private:
    Eigen::VectorQd ControlVal_;
};
