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
#include <pinocchio/autodiff/casadi.hpp>
#include <casadi/casadi.hpp>
#include <casadi_kin_dyn/casadi_kin_dyn.h>
#include <urdf_parser/urdf_parser.h>

pinocchio::Model model;
pinocchio::Data model_data;

pinocchio::Model model1;
pinocchio::Data model_data1;

casadi_kin_dyn::CasadiKinDyn *model3;
//auto model3;
//casadi_kin_dyn::CasadiKinDyn a;