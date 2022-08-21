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
#include "pinocchio/parsers/sample-models.hpp"

#include "crocoddyl/core/mathbase.hpp"
#include "crocoddyl/multibody/states/multibody.hpp"
#include "crocoddyl/multibody/contacts/contact-6d.hpp"
#include "crocoddyl/multibody/contacts/contact-3d.hpp"
#include "crocoddyl/multibody/contacts/multiple-contacts.hpp"
#include "crocoddyl/multibody/actions/contact-fwddyn.hpp"
#include "crocoddyl/core/costs/cost-sum.hpp"
#include "crocoddyl/core/utils/callbacks.hpp"
#include "crocoddyl/core/solvers/ddp.hpp"
#include "crocoddyl/core/utils/timer.hpp"
#include "crocoddyl/core/optctrl/shooting.hpp"
#include "crocoddyl/core/utils/exception.hpp"
#include <stdexcept>
#include "crocoddyl/core/fwd.hpp"
#include "crocoddyl/core/action-base.hpp"
#include "crocoddyl/core/states/euclidean.hpp"
#include <functional> 
//#include <pinocchio/autodiff/casadi.hpp>
#include <casadi_kin_dyn/casadi_kin_dyn.h>
#include <casadi/casadi.hpp>       
#include <string.h>

using namespace std;

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
casadi::MX f(const casadi::MX& x, const casadi::MX& u) {
  return vertcat(x(1), u-x(1));
  // dx/dt = f(x,u)
}

namespace crocoddyl {
template <typename _Scalar>
struct ActionDataFlywheelTpl : public ActionDataAbstractTpl<_Scalar> {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef _Scalar Scalar;
  typedef MathBaseTpl<Scalar> MathBase;
  typedef ActionDataAbstractTpl<Scalar> Base;
  using Base::cost;
  using Base::Fu;
  using Base::Fx;
  using Base::Lu;
  using Base::Luu;
  using Base::Lx;
  using Base::Lxu;
  using Base::Lxx;
  using Base::r;
  using Base::xnext;

  template <template <typename Scalar> class Model>
  explicit ActionDataFlywheelTpl(Model<Scalar>* const model) : Base(model) {
    Fx.diagonal().array() = Scalar(1.);
  }
};

template <typename _Scalar>
class ActionModelFlywheelTpl : public ActionModelAbstractTpl<_Scalar> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef _Scalar Scalar;
  typedef ActionDataAbstractTpl<Scalar> ActionDataAbstract;
  typedef ActionModelAbstractTpl<Scalar> Base;
  typedef ActionDataFlywheelTpl<Scalar> Data;
  typedef MathBaseTpl<Scalar> MathBase;
  typedef typename MathBase::VectorXs VectorXs;
  typedef typename MathBase::Vector2s Vector2s;

  ActionModelFlywheelTpl();
  virtual ~ActionModelFlywheelTpl();

  virtual void calc(const boost::shared_ptr<ActionDataAbstract>& data, const Eigen::Ref<const VectorXs>& x,
                    const Eigen::Ref<const VectorXs>& u);
  virtual void calc(const boost::shared_ptr<ActionDataAbstract>& data, const Eigen::Ref<const VectorXs>& x);
  virtual void calcDiff(const boost::shared_ptr<ActionDataAbstract>& data, const Eigen::Ref<const VectorXs>& x,
                        const Eigen::Ref<const VectorXs>& u);
  virtual void calcDiff(const boost::shared_ptr<ActionDataAbstract>& data, const Eigen::Ref<const VectorXs>& x);
  virtual boost::shared_ptr<ActionDataAbstract> createData();
  virtual bool checkData(const boost::shared_ptr<ActionDataAbstract>& data);

  const Vector2s& get_cost_weights() const;
  void set_cost_weights(const Vector2s& weights);

  Scalar get_dt() const;
  void set_dt(const Scalar dt);

  /**
   * @brief Print relevant information of the Flywheel model
   *
   * @param[out] os  Output stream object
   */
  virtual void print(std::ostream& os) const;

 protected:
  using Base::nu_;     //!< Control dimension
  using Base::state_;  //!< Model of the state

 private:
  Vector2s cost_weights_;
  Scalar dt_;
};
}

namespace crocoddyl {
template <typename Scalar>
ActionModelFlywheelTpl<Scalar>::ActionModelFlywheelTpl()
    : ActionModelAbstractTpl<Scalar>(boost::make_shared<StateVectorTpl<Scalar> >(4), 2, 5), dt_(Scalar(0.1)) {
  cost_weights_ << Scalar(10.), Scalar(1.);
}

template <typename Scalar>
ActionModelFlywheelTpl<Scalar>::~ActionModelFlywheelTpl() {}

template <typename Scalar>
void ActionModelFlywheelTpl<Scalar>::calc(const boost::shared_ptr<ActionDataAbstractTpl<Scalar> >& data,
                                          const Eigen::Ref<const VectorXs>& x, const Eigen::Ref<const VectorXs>& u) {
  if (static_cast<std::size_t>(x.size()) != state_->get_nx()) {
    throw_pretty("Invalid argument: "
                 << "x has wrong dimension (it should be " + std::to_string(state_->get_nx()) + ")");
  }
  if (static_cast<std::size_t>(u.size()) != nu_) {
    throw_pretty("Invalid argument: "
                 << "u has wrong dimension (it should be " + std::to_string(nu_) + ")");
  }
  Data* d = static_cast<Data*>(data.get());

  //const Scalar c = cos(x[2]);
  //const Scalar s = sin(x[2]);
  d->xnext << x[1], 0.5 * x[0] - 0.5 * x[2] - x[3] / 900, u[0], u[1];

  d->r.template head<4>() = cost_weights_[0] * x;
  d->r.template tail<2>() = cost_weights_[1] * u;
  d->cost = Scalar(0.5) * d->r.dot(d->r);
}

template <typename Scalar>
void ActionModelFlywheelTpl<Scalar>::calc(const boost::shared_ptr<ActionDataAbstractTpl<Scalar> >& data,
                                          const Eigen::Ref<const VectorXs>& x) {
  if (static_cast<std::size_t>(x.size()) != state_->get_nx()) {
    throw_pretty("Invalid argument: "
                 << "x has wrong dimension (it should be " + std::to_string(state_->get_nx()) + ")");
  }
  Data* d = static_cast<Data*>(data.get());
  d->r.template head<4>() = cost_weights_[0] * x;
  d->r.template tail<2>().setZero();
  d->cost = Scalar(0.5) * d->r.template head<4>().dot(d->r.template head<4>());
}

template <typename Scalar>
void ActionModelFlywheelTpl<Scalar>::calcDiff(const boost::shared_ptr<ActionDataAbstractTpl<Scalar> >& data,
                                              const Eigen::Ref<const VectorXs>& x,
                                              const Eigen::Ref<const VectorXs>& u) {
  if (static_cast<std::size_t>(x.size()) != state_->get_nx()) {
    throw_pretty("Invalid argument: "
                 << "x has wrong dimension (it should be " + std::to_string(state_->get_nx()) + ")");
  }
  if (static_cast<std::size_t>(u.size()) != nu_) {
    throw_pretty("Invalid argument: "
                 << "u has wrong dimension (it should be " + std::to_string(nu_) + ")");
  }
  Data* d = static_cast<Data*>(data.get());

 // const Scalar c = cos(x[2]);
 // const Scalar s = sin(x[2]);
  const Scalar w_x = cost_weights_[0] * cost_weights_[0];
  const Scalar w_u = cost_weights_[1] * cost_weights_[1];
  d->Lx = x * w_x;
  d->Lu = u * w_u;
  d->Lxx.diagonal().setConstant(w_x);
  d->Luu.diagonal().setConstant(w_u);
  //d->Fx(0, 2) = -s * u[0] * dt_;
  //d->Fx(1, 2) = c * u[0] * dt_;
//  d->Fu(0, 0) = c * dt_;
//  d->Fu(1, 0) = s * dt_;
//  d->Fu(2, 1) = dt_;
 
  d->Fx(0, 1) = 1.0;
  d->Fx(1, 0) = 0.5;
  d->Fx(1, 2) = -0.5;
  d->Fx(1, 3) = -1/900;

  d->Fu(2, 0) = 1.0;
  d->Fu(3, 1) = 1.0;
  //x[1], 0.5 * x[0] - 0.5 * x[2] - x[3] / 900, u[0], u[1]
  //x[0] + c * u[0] * dt_, x[1] + s * u[0] * dt_, x[2] + u[1] * dt_;
}

template <typename Scalar>
void ActionModelFlywheelTpl<Scalar>::calcDiff(const boost::shared_ptr<ActionDataAbstractTpl<Scalar> >& data,
                                              const Eigen::Ref<const VectorXs>& x) {
  if (static_cast<std::size_t>(x.size()) != state_->get_nx()) {
    throw_pretty("Invalid argument: "
                 << "x has wrong dimension (it should be " + std::to_string(state_->get_nx()) + ")");
  }
  Data* d = static_cast<Data*>(data.get());

  const Scalar w_x = cost_weights_[0] * cost_weights_[0];
  d->Lx = x * w_x;
  d->Lxx.diagonal().setConstant(w_x);
}

template <typename Scalar>
boost::shared_ptr<ActionDataAbstractTpl<Scalar> > ActionModelFlywheelTpl<Scalar>::createData() {
  return boost::allocate_shared<Data>(Eigen::aligned_allocator<Data>(), this);
}

template <typename Scalar>
bool ActionModelFlywheelTpl<Scalar>::checkData(const boost::shared_ptr<ActionDataAbstract>& data) {
  boost::shared_ptr<Data> d = boost::dynamic_pointer_cast<Data>(data);
  if (d != NULL) {
    return true;
  } else {
    return false;
  }
}

template <typename Scalar>
void ActionModelFlywheelTpl<Scalar>::print(std::ostream& os) const {
  os << "ActionModelFlywheel {dt=" << dt_ << "}";
}

template <typename Scalar>
const typename MathBaseTpl<Scalar>::Vector2s& ActionModelFlywheelTpl<Scalar>::get_cost_weights() const {
  return cost_weights_;
}

template <typename Scalar>
void ActionModelFlywheelTpl<Scalar>::set_cost_weights(const typename MathBase::Vector2s& weights) {
  cost_weights_ = weights;
}

template <typename Scalar>
Scalar ActionModelFlywheelTpl<Scalar>::get_dt() const {
  return dt_;
}

template <typename Scalar>
void ActionModelFlywheelTpl<Scalar>::set_dt(const Scalar dt) {
  if (dt <= 0) throw_pretty("Invalid argument: dt should be strictly positive.");
  dt_ = dt;
}
}