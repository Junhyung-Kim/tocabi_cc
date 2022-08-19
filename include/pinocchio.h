//#include "crocoddyl/core/solvers/ddp.hpp"
/*#include "pinocchio/algorithm/joint-configuration.hpp"
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
#include "pinocchio/parsers/sample-models.hpp"*/
//#include <pinocchio/autodiff/casadi.hpp>
#include <casadi_kin_dyn/casadi_kin_dyn.h>
#include <casadi/casadi.hpp>
#include <string.h>



pinocchio::Model model;
pinocchio::Data model_data;

pinocchio::Model model1;
pinocchio::Data model_data1;

casadi_kin_dyn::CasadiKinDyn *model3;


casadi::Opti* opti;
casadi::MX X_mpc;
casadi::MX U_mpc;
casadi::Dict solver_opts;
std::string solver_name = "ipopt";
casadi::Function solver;

casadi::MX flywheel_casadi(const casadi::MX& x, const casadi::MX& u)
{
    return vertcat(x(0), 0.5 * x(0) - 0.5 * x(2) - x(3) / 900, u(0), u(1));
}

// dx/dt = f(x,u)
casadi::MX f(const casadi::MX& x, const casadi::MX& u) {
  return vertcat(x(1), u-x(1));
}