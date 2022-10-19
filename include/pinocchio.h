#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/algorithm/model.hpp"
#include <pinocchio/spatial/se3.hpp>
#include <pinocchio/multibody/frame.hpp>
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"
#include "pinocchio/algorithm/compute-all-terms.hpp"
#include "pinocchio/algorithm/centroidal.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/algorithm/centroidal-derivatives.hxx"
#include "pinocchio/algorithm/aba-derivatives.hxx"
#include "pinocchio/parsers/sample-models.hpp"

#include "crocoddyl/core/fwd.hpp"
#include "crocoddyl/multibody/fwd.hpp"
#include "crocoddyl/multibody/residuals/frame-translation.hpp"
#include "crocoddyl/multibody/residuals/frame-velocity.hpp"
#include "crocoddyl/core/activation-base.hpp"
#include "crocoddyl/core/activations/quadratic-barrier.hpp"
#include "crocoddyl/core/activations/weighted-quadratic.hpp"
#include "crocoddyl/core/activations/weighted-quadratic-barrier.hpp"
#include "crocoddyl/core/mathbase.hpp"
#include "crocoddyl/multibody/states/multibody.hpp"
#include "crocoddyl/multibody/contacts/contact-6d.hpp"
#include "crocoddyl/multibody/contacts/contact-3d.hpp"
#include "crocoddyl/multibody/contacts/multiple-contacts.hpp"
#include "crocoddyl/multibody/residuals/frame-translation.hpp"
#include "crocoddyl/multibody/actions/contact-fwddyn.hpp"
#include "crocoddyl/core/integrator/euler.hpp"
#include "crocoddyl/core/integrator/rk.hpp"
#include "crocoddyl/core/costs/cost-sum.hpp"
#include "crocoddyl/core/costs/residual.hpp"
#include "crocoddyl/multibody/residuals/frame-placement.hpp"
#include "crocoddyl/multibody/residuals/state.hpp"
#include "crocoddyl/core/residuals/control.hpp"
#include "crocoddyl/multibody/actuations/full.hpp"
#include "crocoddyl/multibody/actuations/floating-base.hpp"
#include "crocoddyl/core/utils/callbacks.hpp"
#include "crocoddyl/core/solvers/ddp.hpp"
#include "crocoddyl/core/utils/timer.hpp"
#include "crocoddyl/core/solvers/ddp.hpp"
#include "crocoddyl/core/solvers/fddp.hpp"
#include "crocoddyl/core/solvers/box-fddp.hpp"
#include "crocoddyl/core/solvers/box-ddp.hpp"
#include "crocoddyl/core/utils/math.hpp"
#include "pinocchio/parsers/urdf.hpp"
//#include <pinocchio/autodiff/casadi.hpp>
#include <string.h>

#define STDDEV(vec) std::sqrt(((vec - vec.mean())).square().sum() / (static_cast<double>(vec.size()) - 1))
#define AVG(vec) (vec.mean())

using namespace std;

pinocchio::Model model;
pinocchio::Data model_data;

pinocchio::Model model1;
pinocchio::Data model_data1;

namespace crocoddyl {
template <typename _Scalar>
struct DifferentialActionDataKinoDynamicsTpl : public DifferentialActionDataAbstractTpl<_Scalar> {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef _Scalar Scalar;
  typedef MathBaseTpl<Scalar> MathBase;
  typedef DifferentialActionDataAbstractTpl<Scalar> Base;
  typedef typename MathBase::VectorXs VectorXs;
  typedef typename MathBase::MatrixXs MatrixXs;

  template <template <typename Scalar> class Model>
  explicit DifferentialActionDataKinoDynamicsTpl(Model<Scalar>* const model)
      : Base(model),
        pinocchio(pinocchio::DataTpl<Scalar>(model->get_pinocchio())),
        multibody(&pinocchio, model->get_actuation()->createData()),
        costs(model->get_costs()->createData(&multibody)),
        Minv(model->get_state()->get_nv(), model->get_state()->get_nv()),
        u_drift(model->get_nu()),
        dtau_dx(model->get_nu(), model->get_state()->get_ndx()),
        tmp_xstatic(model->get_state()->get_nx()) {
    costs->shareMemory(this);
    Minv.setZero();
    u_drift.setZero();
    dtau_dx.setZero();
    tmp_xstatic.setZero();
  }

  pinocchio::DataTpl<Scalar> pinocchio;
  DataCollectorActMultibodyTpl<Scalar> multibody;
  boost::shared_ptr<CostDataSumTpl<Scalar> > costs;
  MatrixXs Minv;
  VectorXs u_drift;
  MatrixXs dtau_dx;
  VectorXs tmp_xstatic;

  using Base::cost;
  using Base::Fu;
  using Base::Fx;
  using Base::Lu;
  using Base::Luu;
  using Base::Lx;
  using Base::Lxu;
  using Base::Lxx;
  using Base::r;
  using Base::xout;
  using Base::xout2;
  //using Base::dhg;
};

}  // namespace crocoddyl


namespace crocoddyl {

/**
 * @brief State Kinodynamic representation
 *
 * A Kinodynamic state is described by the configuration point and its tangential velocity, or in other words, by the
 * generalized position and velocity coordinates of a rigid-body system. For this state, we describe its operators:
 * difference, integrates, transport and their derivatives for any Pinocchio model.
 *
 * For more details about these operators, please read the documentation of the `StateAbstractTpl` class.
 *
 * \sa `diff()`, `integrate()`, `Jdiff()`, `Jintegrate()` and `JintegrateTransport()`
 */
template <typename _Scalar>
class StateKinodynamicTpl : public StateAbstractTpl<_Scalar> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef _Scalar Scalar;
  typedef MathBaseTpl<Scalar> MathBase;
  typedef StateAbstractTpl<Scalar> Base;
  typedef pinocchio::ModelTpl<Scalar> PinocchioModel;
  typedef typename MathBase::VectorXs VectorXs;
  typedef typename MathBase::MatrixXs MatrixXs;

  /**
   * @brief Initialize the Kinodynamic state
   *
   * @param[in] model  Pinocchio model
   */
  explicit StateKinodynamicTpl(boost::shared_ptr<PinocchioModel> model);
  StateKinodynamicTpl();
  virtual ~StateKinodynamicTpl();

  /**
   * @brief Generate a zero state.
   *
   * Note that the zero configuration is computed using `pinocchio::neutral`.
   */
  virtual VectorXs zero() const;

  /**
   * @brief Generate a random state
   *
   * Note that the random configuration is computed using `pinocchio::random` which satisfies the manifold definition
   * (e.g., the quaterion definition)
   */
  virtual VectorXs rand() const;

  virtual void diff(const Eigen::Ref<const VectorXs>& x0, const Eigen::Ref<const VectorXs>& x1,
                    Eigen::Ref<VectorXs> dxout) const;
  virtual void diff1(const Eigen::Ref<const VectorXs>& x0, const Eigen::Ref<const VectorXs>& x1,
                    Eigen::Ref<VectorXs> dxout) const;                  
  virtual void integrate(const Eigen::Ref<const VectorXs>& x, const Eigen::Ref<const VectorXs>& dx,
                         Eigen::Ref<VectorXs> xout) const;
  virtual void Jdiff(const Eigen::Ref<const VectorXs>&, const Eigen::Ref<const VectorXs>&, Eigen::Ref<MatrixXs> Jfirst,
                     Eigen::Ref<MatrixXs> Jsecond, const Jcomponent firstsecond = both) const;
  virtual void Jdiff1(const Eigen::Ref<const VectorXs>&, const Eigen::Ref<const VectorXs>&, Eigen::Ref<MatrixXs> Jfirst,
                     Eigen::Ref<MatrixXs> Jsecond, const Jcomponent firstsecond = both) const;
  virtual void Jintegrate(const Eigen::Ref<const VectorXs>& x, const Eigen::Ref<const VectorXs>& dx,
                          Eigen::Ref<MatrixXs> Jfirst, Eigen::Ref<MatrixXs> Jsecond,
                          const Jcomponent firstsecond = both, const AssignmentOp = setto) const;
  virtual void JintegrateTransport(const Eigen::Ref<const VectorXs>& x, const Eigen::Ref<const VectorXs>& dx,
                                   Eigen::Ref<MatrixXs> Jin, const Jcomponent firstsecond) const;

  /**
   * @brief Return the Pinocchio model (i.e., model of the rigid body system)
   */
  const boost::shared_ptr<PinocchioModel>& get_pinocchio() const;

 protected:
  using Base::has_limits_;
  using Base::lb_;
  using Base::ndx_;
  using Base::nq_;
  using Base::nv_;
  using Base::nx_;
  using Base::ub_;

 private:
  boost::shared_ptr<PinocchioModel> pinocchio_;  //!< Pinocchio model
  VectorXs x0_;                                  //!< Zero state
};

}  // namespace crocoddyl


namespace crocoddyl {

template <typename Scalar>
StateKinodynamicTpl<Scalar>::StateKinodynamicTpl(boost::shared_ptr<PinocchioModel> model)
    : Base(model->nq + model->nv, 2 * model->nv), pinocchio_(model), x0_(VectorXs::Zero(model->nq + model->nv + 4)) {
  
  const std::size_t nq0 = model->joints[1].nq();
  x0_.head(nq_) = pinocchio::neutral(*pinocchio_.get());

  // In a Kinodynamic system, we could define the first joint using Lie groups.
  // The current cases are free-flyer (SE3) and spherical (S03).
  // Instead simple represents any joint that can model within the Euclidean manifold.
  // The rest of joints use Euclidean algebra. We use this fact for computing Jdiff.

  // Define internally the limits of the first joint

  lb_.head(nq0) = -std::numeric_limits<Scalar>::infinity() * VectorXs::Ones(nq0);
  ub_.head(nq0) = std::numeric_limits<Scalar>::infinity() * VectorXs::Ones(nq0);
  lb_.segment(nq0, nq_ - nq0) = pinocchio_->lowerPositionLimit.tail(nq_ - nq0);
  ub_.segment(nq0, nq_ - nq0) = pinocchio_->upperPositionLimit.tail(nq_ - nq0);
  lb_.segment(nq_, nv_) = -pinocchio_->velocityLimit;
  ub_.segment(nq_, nv_) = pinocchio_->velocityLimit;
  lb_.tail(4) = -std::numeric_limits<Scalar>::infinity() * VectorXs::Ones(4);
  ub_.tail(4) = std::numeric_limits<Scalar>::infinity() * VectorXs::Ones(4);
  Base::update_has_limits();
}

template <typename Scalar>
StateKinodynamicTpl<Scalar>::StateKinodynamicTpl() : Base(), x0_(VectorXs::Zero(0)) {}

template <typename Scalar>
StateKinodynamicTpl<Scalar>::~StateKinodynamicTpl() {}

template <typename Scalar>
typename MathBaseTpl<Scalar>::VectorXs StateKinodynamicTpl<Scalar>::zero() const {
  return x0_;
}

template <typename Scalar>
typename MathBaseTpl<Scalar>::VectorXs StateKinodynamicTpl<Scalar>::rand() const {
  VectorXs xrand = VectorXs::Random(nx_);
  xrand.head(nq_) = pinocchio::randomConfiguration(*pinocchio_.get());
  return xrand;
}

template <typename Scalar>
void StateKinodynamicTpl<Scalar>::diff(const Eigen::Ref<const VectorXs>& x0, const Eigen::Ref<const VectorXs>& x1,
                                     Eigen::Ref<VectorXs> dxout) const {
 // std::cout << "diff " << x0.tail(4).transpose() << std::endl;
 // std::cout << "diffx " << x1.tail(4).transpose() << std::endl;

  if (static_cast<std::size_t>(x0.size()) != nx_ + 4) {
    throw_pretty("Invalid argument: "
                 << "x0 has wrong dimension (it should be " + std::to_string(nx_) + ")");
  }
  if (static_cast<std::size_t>(x1.size()) != nx_ + 4) {
    throw_pretty("Invalid argument: "
                 << "x1 has wrong dimension (it should be " + std::to_string(nx_) + ")");
  }
  if (static_cast<std::size_t>(dxout.size()) != ndx_) {
    throw_pretty("Invalid argument: "
                 << "dxout has wrong dimension (it should be " + std::to_string(ndx_) + ")");
  }
  pinocchio::difference(*pinocchio_.get(), x0.head(nq_), x1.head(nq_), dxout.head(nv_));
  dxout.segment(nq_,nv_) = x1.segment(nq_,nv_) - x0.segment(nq_,nv_);
  dxout.tail(4) = x1.tail(4) - x0.tail(4);
}

template <typename Scalar>
void StateKinodynamicTpl<Scalar>::diff1(const Eigen::Ref<const VectorXs>& x0, const Eigen::Ref<const VectorXs>& x1,
                                     Eigen::Ref<VectorXs> dxout) const {
 // std::cout << "diff " << x0.tail(4).transpose() << std::endl;
 // std::cout << "diffx " << x1.tail(4).transpose() << std::endl;
  if (static_cast<std::size_t>(x0.size()) != nx_ + 4) {
    throw_pretty("Invalid argument: "
                 << "x0 has wrong dimension (it should be " + std::to_string(nx_) + ")");
  }
  if (static_cast<std::size_t>(x1.size()) != nx_ + 4) {
    throw_pretty("Invalid argument: "
                 << "x1 has wrong dimension (it should be " + std::to_string(nx_) + ")");
  }
  if (static_cast<std::size_t>(dxout.size()) != ndx_) {
    throw_pretty("Invalid argument: "
                 << "dxout has wrong dimension (it should be " + std::to_string(ndx_) + ")");
  }

  dxout.setZero();
  dxout.tail(2) = x1.tail(2) - x0.tail(2);
  dxout.tail(1).setZero();
}

template <typename Scalar>
void StateKinodynamicTpl<Scalar>::integrate(const Eigen::Ref<const VectorXs>& x, const Eigen::Ref<const VectorXs>& dx,
                                          Eigen::Ref<VectorXs> xout) const {
  if (static_cast<std::size_t>(x.size()) != nx_ + 4) {
    throw_pretty("Invalid argument: "
                 << "x has wrong dimension (it should be " + std::to_string(nx_) + ")");
  }
  pinocchio::integrate(*pinocchio_.get(), x.head(nq_), dx.head(nv_), xout.head(nq_));
  xout.segment(nq_,nv_) = x.segment(nq_,nv_) + dx.segment(nq_ - 1, nv_); 
  xout.tail(4) = x.tail(4) + dx.tail(4);
}

template <typename Scalar>
void StateKinodynamicTpl<Scalar>::Jdiff(const Eigen::Ref<const VectorXs>& x0, const Eigen::Ref<const VectorXs>& x1,
                                      Eigen::Ref<MatrixXs> Jfirst, Eigen::Ref<MatrixXs> Jsecond,
                                      const Jcomponent firstsecond) const {
  assert_pretty(is_a_Jcomponent(firstsecond), ("firstsecond must be one of the Jcomponent {both, first, second}"));
  if (static_cast<std::size_t>(x0.size()) != nx_ + 4) {
    throw_pretty("Invalid argument: "
                 << "x0 has wrong dimension (it should be " + std::to_string(nx_) + ")");
  }
  if (static_cast<std::size_t>(x1.size()) != nx_ + 4) {
    throw_pretty("Invalid argument: "
                 << "x1 has wrong dimension (it should be " + std::to_string(nx_) + ")");
  }

  if (firstsecond == first) {
    if (static_cast<std::size_t>(Jfirst.rows()) != ndx_ || static_cast<std::size_t>(Jfirst.cols()) != ndx_) {
      throw_pretty("Invalid argument: "
                   << "Jfirst has wrong dimension (it should be " + std::to_string(ndx_) + "," + std::to_string(ndx_) +
                          ")");
    }

    pinocchio::dDifference(*pinocchio_.get(), x0.head(nq_), x1.head(nq_), Jfirst.topLeftCorner(nv_, nv_),
                           pinocchio::ARG0);
    Jfirst.block(nv_, nv_, nv_, nv_).diagonal().array() = (Scalar)-1;
    Jfirst.bottomRightCorner(4,4).diagonal().array() = (Scalar)-1;
  } else if (firstsecond == second) {
    if (static_cast<std::size_t>(Jsecond.rows()) != ndx_ || static_cast<std::size_t>(Jsecond.cols()) != ndx_) {
      throw_pretty("Invalid argument: "
                   << "Jsecond has wrong dimension (it should be " + std::to_string(ndx_) + "," +
                          std::to_string(ndx_) + ")");
    }
    pinocchio::dDifference(*pinocchio_.get(), x0.head(nq_), x1.head(nq_), Jsecond.topLeftCorner(nv_, nv_),
                           pinocchio::ARG1);
    Jsecond.block(nv_, nv_, nv_, nv_).diagonal().array() = (Scalar)1;
    Jsecond.bottomRightCorner(4,4).diagonal().array() = (Scalar)1;
  } else {  // computing both
    if (static_cast<std::size_t>(Jfirst.rows()) != ndx_ || static_cast<std::size_t>(Jfirst.cols()) != ndx_) {
      throw_pretty("Invalid argument: "
                   << "Jfirst has wrong dimension (it should be " + std::to_string(ndx_) + "," + std::to_string(ndx_) +
                          ")");
    }
    if (static_cast<std::size_t>(Jsecond.rows()) != ndx_ || static_cast<std::size_t>(Jsecond.cols()) != ndx_) {
      throw_pretty("Invalid argument: "
                   << "Jsecond has wrong dimension (it should be " + std::to_string(ndx_) + "," +
                          std::to_string(ndx_) + ")");
    }
    pinocchio::dDifference(*pinocchio_.get(), x0.head(nq_), x1.head(nq_), Jfirst.topLeftCorner(nv_, nv_),
                           pinocchio::ARG0);
    pinocchio::dDifference(*pinocchio_.get(), x0.head(nq_), x1.head(nq_), Jsecond.topLeftCorner(nv_, nv_),
                           pinocchio::ARG1);
    Jfirst.block(nv_, nv_, nv_, nv_).diagonal().array() = (Scalar)-1;
    Jsecond.block(nv_, nv_, nv_, nv_).diagonal().array() = (Scalar)1;
    Jfirst.bottomRightCorner(4,4).diagonal().array() = (Scalar)-1;
    Jsecond.bottomRightCorner(4,4).diagonal().array() = (Scalar)1;
  }
}
template <typename Scalar>
void StateKinodynamicTpl<Scalar>::Jdiff1(const Eigen::Ref<const VectorXs>& x0, const Eigen::Ref<const VectorXs>& x1,
                                      Eigen::Ref<MatrixXs> Jfirst, Eigen::Ref<MatrixXs> Jsecond,
                                      const Jcomponent firstsecond) const {
  assert_pretty(is_a_Jcomponent(firstsecond), ("firstsecond must be one of the Jcomponent {both, first, second}"));
  if (static_cast<std::size_t>(x0.size()) != nx_ + 4) {
    throw_pretty("Invalid argument: "
                 << "x0 has wrong dimension (it should be " + std::to_string(nx_) + ")");
  }
  if (static_cast<std::size_t>(x1.size()) != nx_ + 4) {
    throw_pretty("Invalid argument: "
                 << "x1 has wrong dimension (it should be " + std::to_string(nx_) + ")");
  }

  if (firstsecond == first) {
    if (static_cast<std::size_t>(Jfirst.rows()) != ndx_ || static_cast<std::size_t>(Jfirst.cols()) != ndx_) {
      throw_pretty("Invalid argument: "
                   << "Jfirst has wrong dimension (it should be " + std::to_string(ndx_) + "," + std::to_string(ndx_) +
                          ")");
    }

    pinocchio::dDifference(*pinocchio_.get(), x0.head(nq_), x1.head(nq_), Jfirst.topLeftCorner(nv_, nv_),
                           pinocchio::ARG0);
    Jfirst.block(nv_, nv_, nv_, nv_).diagonal().array() = (Scalar)-1;
    Jfirst.bottomRightCorner(4,4).diagonal().array() = (Scalar)-1;
  } else if (firstsecond == second) {
    if (static_cast<std::size_t>(Jsecond.rows()) != ndx_ || static_cast<std::size_t>(Jsecond.cols()) != ndx_) {
      throw_pretty("Invalid argument: "
                   << "Jsecond has wrong dimension (it should be " + std::to_string(ndx_) + "," +
                          std::to_string(ndx_) + ")");
    }
   // pinocchio::dDifference(*pinocchio_.get(), x0.head(nq_), x1.head(nq_), Jsecond.topLeftCorner(nv_, nv_),
                          // pinocchio::ARG1);
   // Jsecond.block(nv_, nv_, nv_, nv_).diagonal().array() = (Scalar)1;
    Jsecond.setZero();
    Jsecond.bottomRightCorner(2,2).diagonal().array() = (Scalar)1;
    Jsecond.bottomRightCorner(1,1).setZero();
  } else {  // computing both
    if (static_cast<std::size_t>(Jfirst.rows()) != ndx_ || static_cast<std::size_t>(Jfirst.cols()) != ndx_) {
      throw_pretty("Invalid argument: "
                   << "Jfirst has wrong dimension (it should be " + std::to_string(ndx_) + "," + std::to_string(ndx_) +
                          ")");
    }
    if (static_cast<std::size_t>(Jsecond.rows()) != ndx_ || static_cast<std::size_t>(Jsecond.cols()) != ndx_) {
      throw_pretty("Invalid argument: "
                   << "Jsecond has wrong dimension (it should be " + std::to_string(ndx_) + "," +
                          std::to_string(ndx_) + ")");
    }
    pinocchio::dDifference(*pinocchio_.get(), x0.head(nq_), x1.head(nq_), Jfirst.topLeftCorner(nv_, nv_),
                           pinocchio::ARG0);
    pinocchio::dDifference(*pinocchio_.get(), x0.head(nq_), x1.head(nq_), Jsecond.topLeftCorner(nv_, nv_),
                           pinocchio::ARG1);
    Jfirst.block(nv_, nv_, nv_, nv_).diagonal().array() = (Scalar)-1;
    Jsecond.block(nv_, nv_, nv_, nv_).diagonal().array() = (Scalar)1;
    Jfirst.bottomRightCorner(4,4).diagonal().array() = (Scalar)-1;
    Jsecond.bottomRightCorner(4,4).diagonal().array() = (Scalar)1;
  }
}

template <typename Scalar>
void StateKinodynamicTpl<Scalar>::Jintegrate(const Eigen::Ref<const VectorXs>& x, const Eigen::Ref<const VectorXs>& dx,
                                           Eigen::Ref<MatrixXs> Jfirst, Eigen::Ref<MatrixXs> Jsecond,
                                           const Jcomponent firstsecond, const AssignmentOp op) const {
  assert_pretty(is_a_Jcomponent(firstsecond), ("firstsecond must be one of the Jcomponent {both, first, second}"));
  assert_pretty(is_a_AssignmentOp(op), ("op must be one of the AssignmentOp {settop, addto, rmfrom}"));

  if (firstsecond == first || firstsecond == both) {
    if (static_cast<std::size_t>(Jfirst.rows()) != ndx_ || static_cast<std::size_t>(Jfirst.cols()) != ndx_) {
      throw_pretty("Invalid argument: "
                   << "Jfirst has wrong dimension (it should be " + std::to_string(ndx_) + "," + std::to_string(ndx_) +
                          ")");
    }
    switch (op) {
      case setto:
        pinocchio::dIntegrate(*pinocchio_.get(), x.head(nq_), dx.head(nv_), Jfirst.topLeftCorner(nv_, nv_),
                              pinocchio::ARG0, pinocchio::SETTO);
        Jfirst.bottomRightCorner(nv_ + 4, nv_ + 4).diagonal().array() = (Scalar)1;
        break;
      case addto:
        pinocchio::dIntegrate(*pinocchio_.get(), x.head(nq_), dx.head(nv_), Jfirst.topLeftCorner(nv_, nv_),
                              pinocchio::ARG0, pinocchio::ADDTO);
        Jfirst.bottomRightCorner(nv_ + 4, nv_ + 4).diagonal().array() += (Scalar)1;
        break;
      case rmfrom:
        pinocchio::dIntegrate(*pinocchio_.get(), x.head(nq_), dx.head(nv_), Jfirst.topLeftCorner(nv_, nv_),
                              pinocchio::ARG0, pinocchio::RMTO);
        Jfirst.bottomRightCorner(nv_ + 4, nv_ + 4).diagonal().array() -= (Scalar)1;
        break;
      default:
        throw_pretty("Invalid argument: allowed operators: setto, addto, rmfrom");
        break;
    }
  }
  if (firstsecond == second || firstsecond == both) {
    if (static_cast<std::size_t>(Jsecond.rows()) != ndx_ || static_cast<std::size_t>(Jsecond.cols()) != ndx_) {
      throw_pretty("Invalid argument: "
                   << "Jsecond has wrong dimension (it should be " + std::to_string(ndx_) + "," +
                          std::to_string(ndx_) + ")");
    }
    switch (op) {
      case setto:
        pinocchio::dIntegrate(*pinocchio_.get(), x.head(nq_), dx.head(nv_), Jsecond.topLeftCorner(nv_, nv_),
                              pinocchio::ARG1, pinocchio::SETTO);
         Jsecond.setZero();
        Jsecond.bottomRightCorner(nv_+4, nv_+4).diagonal().array() = (Scalar)1;
        break;
      case addto:
        pinocchio::dIntegrate(*pinocchio_.get(), x.head(nq_), dx.head(nv_), Jsecond.topLeftCorner(nv_, nv_),
                              pinocchio::ARG1, pinocchio::ADDTO);
        Jsecond.setZero();
        Jsecond.bottomRightCorner(nv_+4, nv_+4).diagonal().array() += (Scalar)1;
        break;
      case rmfrom:
        pinocchio::dIntegrate(*pinocchio_.get(), x.head(nq_), dx.head(nv_), Jsecond.topLeftCorner(nv_, nv_),
                              pinocchio::ARG1, pinocchio::RMTO);
         Jsecond.setZero();
                             
        Jsecond.bottomRightCorner(nv_+4, nv_+4).diagonal().array() -= (Scalar)1;
        break;
      default:
        throw_pretty("Invalid argument: allowed operators: setto, addto, rmfrom");
        break;
    }
  }
}

template <typename Scalar>
void StateKinodynamicTpl<Scalar>::JintegrateTransport(const Eigen::Ref<const VectorXs>& x,
                                                    const Eigen::Ref<const VectorXs>& dx, Eigen::Ref<MatrixXs> Jin,
                                                    const Jcomponent firstsecond) const {
  assert_pretty(is_a_Jcomponent(firstsecond), ("firstsecond must be one of the Jcomponent {both, first, second}"));

  switch (firstsecond) {
    case first:
      pinocchio::dIntegrateTransport(*pinocchio_.get(), x.head(nq_), dx.head(nv_), Jin.topRows(nv_), pinocchio::ARG0);
      break;
    case second:
      pinocchio::dIntegrateTransport(*pinocchio_.get(), x.head(nq_), dx.head(nv_), Jin.topRows(nv_), pinocchio::ARG1);
      break;
    default:
      throw_pretty(
          "Invalid argument: firstsecond must be either first or second. both not supported for this operation.");
      break;
  }
}

template <typename Scalar>
const boost::shared_ptr<pinocchio::ModelTpl<Scalar> >& StateKinodynamicTpl<Scalar>::get_pinocchio() const {
  return pinocchio_;
}

}  // namespace crocoddyl


namespace crocoddyl {

/**
 * @brief Floating-base actuation model
 *
 * It considers the first joint, defined in the Pinocchio model, as the floating-base joints.
 * Then, this joint (that might have various DoFs) is unactuated.
 *
 * The main computations are carrying out in `calc`, and `calcDiff`, where the former computes actuation signal
 * \f$\mathbf{a}\f$ from a given control input \f$\mathbf{u}\f$ and state point \f$\mathbf{x}\f$, and the latter
 * computes the Jacobians of the actuation-mapping function. Note that `calcDiff` requires to run `calc` first.
 *
 * \sa `ActuationModelAbstractTpl`, `calc()`, `calcDiff()`, `createData()`
 */
template <typename _Scalar>
class ActuationModelFloatingKinoBaseTpl : public ActuationModelAbstractTpl<_Scalar> {
 public:
  typedef _Scalar Scalar;
  typedef MathBaseTpl<Scalar> MathBase;
  typedef ActuationModelAbstractTpl<Scalar> Base;
  typedef ActuationDataAbstractTpl<Scalar> Data;
  typedef StateKinodynamicTpl<Scalar> StateKinodynamic;
  typedef typename MathBase::VectorXs VectorXs;
  typedef typename MathBase::MatrixXs MatrixXs;

  /**
   * @brief Initialize the floating-base actuation model
   *
   * @param[in] state  State of a multibody system
   * @param[in] nu     Dimension of control vector
   */
  explicit ActuationModelFloatingKinoBaseTpl(boost::shared_ptr<StateKinodynamic> state)
      : Base(state, state->get_nv()){};
  virtual ~ActuationModelFloatingKinoBaseTpl(){};

  /**
   * @brief Compute the floating-base actuation signal from the control input \f$\mathbf{u}\in\mathbb{R}^{nu}\f$
   *
   * @param[in] data  Actuation data
   * @param[in] x     State point \f$\mathbf{x}\in\mathbb{R}^{ndx}\f$
   * @param[in] u     Control input \f$\mathbf{u}\in\mathbb{R}^{nu}\f$
   */
  virtual void calc(const boost::shared_ptr<Data>& data, const Eigen::Ref<const VectorXs>& x,
                    const Eigen::Ref<const VectorXs>& u) {
    if (static_cast<std::size_t>(u.size()) != nu_ + 2) {
      throw_pretty("Invalid argument: "
                   << "u has wrong dimension (it should be " + std::to_string(nu_) + ")");
    }
    data->tau.segment(0,nu_) = u.head(nu_);
    data->u_x = u.tail(2);
  };

    /**
     * @brief Compute the Jacobians of the floating-base actuation function
     *
     * @param[in] data  Actuation data
     * @param[in] x     State point \f$\mathbf{x}\in\mathbb{R}^{ndx}\f$
     * @param[in] u     Control input \f$\mathbf{u}\in\mathbb{R}^{nu}\f$
     */
#ifndef NDEBUG
  virtual void calcDiff(const boost::shared_ptr<Data>& data, const Eigen::Ref<const VectorXs>& x,
                        const Eigen::Ref<const VectorXs>& u) {
#else
  virtual void calcDiff(const boost::shared_ptr<Data>&, const Eigen::Ref<const VectorXs>& x,
                        const Eigen::Ref<const VectorXs>& u) {
#endif
    // The derivatives has constant values which were set in createData.
    assert_pretty(data->dtau_dx.isZero(), "dtau_dx has wrong value");
    assert_pretty(MatrixXs(data->dtau_du).isApprox(dtau_du_), "dtau_du has wrong value");
  };

  /**
   * @brief Create the floating-base actuation data
   *
   * @return the actuation data
   */
  virtual boost::shared_ptr<Data> createData() {
    typedef StateKinodynamicTpl<Scalar> StateKinodynamic;
    boost::shared_ptr<StateKinodynamic> state = boost::static_pointer_cast<StateKinodynamic>(state_);
    boost::shared_ptr<Data> data = boost::allocate_shared<Data>(Eigen::aligned_allocator<Data>(), this);
    data->dtau_du.diagonal(nu_).setOnes();
#ifndef NDEBUG
    dtau_du_ = data->dtau_du;
#endif
    return data;
  };

 protected:
  using Base::nu_;
  using Base::state_;

#ifndef NDEBUG
 private:
  MatrixXs dtau_du_;
#endif
};

}  // namespace crocoddyl

namespace crocoddyl {
template <typename _Scalar>
class DifferentialActionModelKinoDynamicsTpl : public DifferentialActionModelAbstractTpl<_Scalar> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef _Scalar Scalar;
  typedef DifferentialActionModelAbstractTpl<Scalar> Base;
  typedef DifferentialActionDataKinoDynamicsTpl<Scalar> Data;
  typedef MathBaseTpl<Scalar> MathBase;
  typedef CostModelSumTpl<Scalar> CostModelSum;
  typedef StateKinodynamicTpl<Scalar> StateKinodynamic;
  typedef ActuationModelAbstractTpl<Scalar> ActuationModelAbstract;
  typedef DifferentialActionDataAbstractTpl<Scalar> DifferentialActionDataAbstract;
  typedef typename MathBase::VectorXs VectorXs;
  typedef typename MathBase::MatrixXs MatrixXs;

  DifferentialActionModelKinoDynamicsTpl(boost::shared_ptr<StateKinodynamic> state,
                                            boost::shared_ptr<ActuationModelAbstract> actuation,
                                            boost::shared_ptr<CostModelSum> costs);
  virtual ~DifferentialActionModelKinoDynamicsTpl();

  virtual void calc(const boost::shared_ptr<DifferentialActionDataAbstract>& data, const Eigen::Ref<const VectorXs>& x,
                    const Eigen::Ref<const VectorXs>& u);

  virtual void calc(const boost::shared_ptr<DifferentialActionDataAbstract>& data,
                    const Eigen::Ref<const VectorXs>& x);

  virtual void calcDiff(const boost::shared_ptr<DifferentialActionDataAbstract>& data,
                        const Eigen::Ref<const VectorXs>& x, const Eigen::Ref<const VectorXs>& u);

  virtual void calcDiff(const boost::shared_ptr<DifferentialActionDataAbstract>& data,
                        const Eigen::Ref<const VectorXs>& x);

  virtual boost::shared_ptr<DifferentialActionDataAbstract> createData();

  virtual bool checkData(const boost::shared_ptr<DifferentialActionDataAbstract>& data);

  virtual void quasiStatic(const boost::shared_ptr<DifferentialActionDataAbstract>& data, Eigen::Ref<VectorXs> u,
                           const Eigen::Ref<const VectorXs>& x, const std::size_t maxiter = 100,
                           const Scalar tol = Scalar(1e-9));

  const boost::shared_ptr<ActuationModelAbstract>& get_actuation() const;

  const boost::shared_ptr<CostModelSum>& get_costs() const;

  pinocchio::ModelTpl<Scalar>& get_pinocchio() const;

  const VectorXs& get_armature() const;

  void set_armature(const VectorXs& armature);

  virtual void print(std::ostream& os) const;

 protected:
  using Base::nu_;     //!< Control dimension
  using Base::state_;  //!< Model of the state

 private:
  boost::shared_ptr<ActuationModelAbstract> actuation_;  //!< Actuation model
  boost::shared_ptr<CostModelSum> costs_;                //!< Cost model
  pinocchio::ModelTpl<Scalar>& pinocchio_;               //!< Pinocchio model
  bool without_armature_;                                //!< Indicate if we have defined an armature
  VectorXs armature_;                                    //!< Armature vector
};
}

namespace crocoddyl {

template <typename Scalar>
DifferentialActionModelKinoDynamicsTpl<Scalar>::DifferentialActionModelKinoDynamicsTpl(
    boost::shared_ptr<StateKinodynamic> state, boost::shared_ptr<ActuationModelAbstract> actuation,
    boost::shared_ptr<CostModelSum> costs)
    : Base(state, actuation->get_nu(), costs->get_nr()),
      actuation_(actuation),
      costs_(costs),
      pinocchio_(*state->get_pinocchio().get()),
      without_armature_(true),
      armature_(VectorXs::Zero(state->get_nv())) {
  if (costs_->get_nu() != nu_ + 2) {
    throw_pretty("Invalid argument: "
                 << "Costs doesn't have the same control dimension (it should be " + std::to_string(nu_) + ")");
  }
  VectorXs temp;
  temp.resize(actuation->get_nu() + 2);
  temp.setZero();
  temp.head(nu_) = pinocchio_.effortLimit.head(nu_);
  temp(nu_+1) = 1000000;
  temp(nu_) = 1000000;
  Base::set_u_lb(Scalar(-1.) * temp);
  Base::set_u_ub(Scalar(+1.) * temp);
}

template <typename Scalar>
DifferentialActionModelKinoDynamicsTpl<Scalar>::~DifferentialActionModelKinoDynamicsTpl() {}

template <typename Scalar>
void DifferentialActionModelKinoDynamicsTpl<Scalar>::calc(
    const boost::shared_ptr<DifferentialActionDataAbstract>& data, const Eigen::Ref<const VectorXs>& x,
    const Eigen::Ref<const VectorXs>& u) {
  if (static_cast<std::size_t>(x.size()) != state_->get_nx() + 4) {
    throw_pretty("Invalid argument: "
                 << "x has wrong dimension (it should be " + std::to_string(state_->get_nx()) + ")");
  }
  if (static_cast<std::size_t>(u.size()) != nu_ + 2) {
    throw_pretty("Invalid argument: "
                 << "u has wrong dimension (it should be " + std::to_string(nu_) + ")");
  }

  Data* d = static_cast<Data*>(data.get());
  const Eigen::VectorBlock<const Eigen::Ref<const VectorXs>, Eigen::Dynamic> q = x.head(state_->get_nq());
  const Eigen::VectorBlock<const Eigen::Ref<const VectorXs>, Eigen::Dynamic> v = x.segment(state_->get_nq(),state_->get_nv());
  const Eigen::VectorBlock<const Eigen::Ref<const VectorXs>, Eigen::Dynamic> x_state = x.tail(4);
  const Eigen::VectorBlock<const Eigen::Ref<const VectorXs>, Eigen::Dynamic> a = u.head(state_->get_nv());

  actuation_->calc(d->multibody.actuation, x, u);

  // Computing the dynamics using ABA or manually for armature case
 /* if (without_armature_) {
    d->xout = pinocchio::aba(pinocchio_, d->pinocchio, q, v, d->multibody.actuation->tau.segment(0,state_->get_nv()));
    pinocchio::updateGlobalPlacements(pinocchio_, d->pinocchio);
  } else {

    pinocchio::computeAllTerms(pinocchio_, d->pinocchio, q, v);
    d->pinocchio.M.diagonal() += armature_;
    pinocchio::cholesky::decompose(pinocchio_, d->pinocchio);
    d->Minv.setZero();
    pinocchio::cholesky::computeMinv(pinocchio_, d->pinocchio, d->Minv);
    d->u_drift = d->multibody.actuation->tau - d->pinocchio.nle;
    d->xout.noalias() = d->Minv * d->u_drift;
  }*/

  //d->xout = d->multibody.actuation->tau;
  d->xout =  d->multibody.actuation->tau.segment(0,state_->get_nv());
  d->xout2 << x_state[1], 10 * x_state[0] - 10 * x_state[2] - d->multibody.actuation->u_x[1] * 1.0/ 70.0, d->multibody.actuation->u_x[0], d->multibody.actuation->u_x[1];//d->dhg; 
  
  // Computing the cost value and residuals
  costs_->calc(d->costs, x, u);
  d->cost = d->costs->cost;
}

template <typename Scalar>
void DifferentialActionModelKinoDynamicsTpl<Scalar>::calc(
    const boost::shared_ptr<DifferentialActionDataAbstract>& data, const Eigen::Ref<const VectorXs>& x) {
  if (static_cast<std::size_t>(x.size()) != state_->get_nx() + 4) {
    throw_pretty("Invalid argument: "
                 << "x has wrong dimension (it should be " + std::to_string(state_->get_nx()) + ")");
  }
  Data* d = static_cast<Data*>(data.get());
  const Eigen::VectorBlock<const Eigen::Ref<const VectorXs>, Eigen::Dynamic> q = x.head(state_->get_nq());
  const Eigen::VectorBlock<const Eigen::Ref<const VectorXs>, Eigen::Dynamic> v = x.segment(state_->get_nq(),state_->get_nv());
  const Eigen::VectorBlock<const Eigen::Ref<const VectorXs>, Eigen::Dynamic> x_state = x.tail(4);
  //pinocchio::computeAllTerms(pinocchio_, d->pinocchio, q, v);

  costs_->calc(d->costs, x);
  d->cost = d->costs->cost;
}

template <typename Scalar>
void DifferentialActionModelKinoDynamicsTpl<Scalar>::calcDiff(
    const boost::shared_ptr<DifferentialActionDataAbstract>& data, const Eigen::Ref<const VectorXs>& x,
    const Eigen::Ref<const VectorXs>& u) {
  if (static_cast<std::size_t>(x.size()) != state_->get_nx() + 4) {
    throw_pretty("Invalid argument: "
                 << "x has wrong dimension (it should be " + std::to_string(state_->get_nx()) + ")");
  }
  if (static_cast<std::size_t>(u.size()) != nu_ + 2) {
    throw_pretty("Invalid argument: "
                 << "u has wrong dimension (it should be " + std::to_string(nu_) + ")");
  }

  const std::size_t nv = state_->get_nv();
  const Eigen::VectorBlock<const Eigen::Ref<const VectorXs>, Eigen::Dynamic> q = x.head(state_->get_nq());
  const Eigen::VectorBlock<const Eigen::Ref<const VectorXs>, Eigen::Dynamic> v = x.segment(state_->get_nq(),nv);
  const Eigen::VectorBlock<const Eigen::Ref<const VectorXs>, Eigen::Dynamic> a = u.head(state_->get_nv());
  const Eigen::VectorBlock<const Eigen::Ref<const VectorXs>, Eigen::Dynamic> x_state = x.tail(4);

  Data* d = static_cast<Data*>(data.get());
  actuation_->calcDiff(d->multibody.actuation, x, u);
  // Computing the dynamics derivatives
 /* if (without_armature_) {
    pinocchio::computeABADerivatives(pinocchio_, d->pinocchio, q, v, d->multibody.actuation->tau, d->Fx.topLeftCorner(nv,nv),
                                     d->Fx.topLeftCorner(nv,nv), d->pinocchio.Minv);   
    d->Fx.topLeftCorner(nv,state_->get_ndx()).noalias() += d->pinocchio.Minv * d->multibody.actuation->dtau_dx;
    d->Fu.topLeftCorner(nv,nu_).noalias() = (d->pinocchio.Minv * d->multibody.actuation->dtau_du);
  } else {
    pinocchio::computeRNEADerivatives(pinocchio_, d->pinocchio, q, v, d->xout);
    d->dtau_dx.leftCols(nv) = d->multibody.actuation->dtau_dx.leftCols(nv) - d->pinocchio.dtau_dq;
    d->dtau_dx.rightCols(nv) = d->multibody.actuation->dtau_dx.rightCols(nv) - d->pinocchio.dtau_dv;
    d->Fx.topLeftCorner(nv,state_->get_ndx()).noalias() = d->Minv * d->dtau_dx;
    d->Fu.topLeftCorner(nv,nu_).noalias() = d->Minv * d->multibody.actuation->dtau_du;
  }*/

  d->Fx.bottomRightCorner(4,4) << 0.0, 1.0, 0.0, 0.0, 10.0, 0.0, -10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  d->Fx.block(0, state_->get_nv(), state_->get_nv(), state_->get_nv()).setIdentity();
  d->Fu.topLeftCorner(nu_, nu_).setIdentity();
  d->Fu.bottomRightCorner(4,2) << 0.0, 0.0, 0.0, -1.0/ 70.0, 1.0, 0.0, 0.0, 1.0;
/*
  std::cout << "d->Fx" << std::endl;
  std::cout << d->Fx << std::endl;
  
  std::cout << "d->Fu" << std::endl;
  std::cout << d->Fu << std::endl;
  */
  costs_->calcDiff(d->costs, x,  u);
}

template <typename Scalar>
void DifferentialActionModelKinoDynamicsTpl<Scalar>::calcDiff(
    const boost::shared_ptr<DifferentialActionDataAbstract>& data, const Eigen::Ref<const VectorXs>& x) {
  if (static_cast<std::size_t>(x.size()) != state_->get_nx() + 4) {
    throw_pretty("Invalid argument: "
                 << "x has wrong dimension (it should be " + std::to_string(state_->get_nx()) + ")");
  }
  Data* d = static_cast<Data*>(data.get());
  costs_->calcDiff(d->costs, x);
}

template <typename Scalar>
boost::shared_ptr<DifferentialActionDataAbstractTpl<Scalar> >
DifferentialActionModelKinoDynamicsTpl<Scalar>::createData() {
  return boost::allocate_shared<Data>(Eigen::aligned_allocator<Data>(), this);
}

template <typename Scalar>
bool DifferentialActionModelKinoDynamicsTpl<Scalar>::checkData(
    const boost::shared_ptr<DifferentialActionDataAbstract>& data) {
  boost::shared_ptr<Data> d = boost::dynamic_pointer_cast<Data>(data);
  if (d != NULL) {
    return true;
  } else {
    return false;
  }
}
template <typename Scalar>
void DifferentialActionModelKinoDynamicsTpl<Scalar>::quasiStatic(
    const boost::shared_ptr<DifferentialActionDataAbstract>& data, Eigen::Ref<VectorXs> u,
    const Eigen::Ref<const VectorXs>& x, const std::size_t, const Scalar) {
  if (static_cast<std::size_t>(u.size()) != nu_ + 2) {
    throw_pretty("Invalid argument: "
                 << "u has wrong dimension (it should be " + std::to_string(nu_) + ")");
  }
  if (static_cast<std::size_t>(x.size()) != state_->get_nx() + 4) {
    throw_pretty("Invalid argument: "
                 << "x has wrong dimension (it should be " + std::to_string(state_->get_nx()) + ")");
  }
  // Static casting the data
  Data* d = static_cast<Data*>(data.get());
  const Eigen::VectorBlock<const Eigen::Ref<const VectorXs>, Eigen::Dynamic> q = x.head(state_->get_nq());

  const std::size_t nq = state_->get_nq();
  const std::size_t nv = state_->get_nv();

  // Check the velocity input is zero
  assert_pretty(x.segment(nq,nv).isZero(), "The velocity input should be zero for quasi-static to work.");

  d->tmp_xstatic.head(nq) = q;
  d->tmp_xstatic.segment(nq,nv).setZero();
  u.setZero();

  pinocchio::rnea(pinocchio_, d->pinocchio, q, d->tmp_xstatic.segment(nq,nv), d->tmp_xstatic.segment(nq,nv));
  actuation_->calc(d->multibody.actuation, d->tmp_xstatic, u);
  actuation_->calcDiff(d->multibody.actuation, d->tmp_xstatic, u);

  u.noalias() = pseudoInverse(d->multibody.actuation->dtau_du) * d->pinocchio.tau;
  d->pinocchio.tau.setZero();
}

template <typename Scalar>
void DifferentialActionModelKinoDynamicsTpl<Scalar>::print(std::ostream& os) const {
  os << "DifferentialActionModelKinoDynamics {nx=" << state_->get_nx() << ", ndx=" << state_->get_ndx()
     << ", nu=" << nu_ << "}";
}

template <typename Scalar>
pinocchio::ModelTpl<Scalar>& DifferentialActionModelKinoDynamicsTpl<Scalar>::get_pinocchio() const {
  return pinocchio_;
}

template <typename Scalar>
const boost::shared_ptr<ActuationModelAbstractTpl<Scalar> >&
DifferentialActionModelKinoDynamicsTpl<Scalar>::get_actuation() const {
  return actuation_;
}

template <typename Scalar>
const boost::shared_ptr<CostModelSumTpl<Scalar> >& DifferentialActionModelKinoDynamicsTpl<Scalar>::get_costs()
    const {
  return costs_;
}

template <typename Scalar>
const typename MathBaseTpl<Scalar>::VectorXs& DifferentialActionModelKinoDynamicsTpl<Scalar>::get_armature() const {
  return armature_;
}

template <typename Scalar>
void DifferentialActionModelKinoDynamicsTpl<Scalar>::set_armature(const VectorXs& armature) {
  if (static_cast<std::size_t>(armature.size()) != state_->get_nv()) {
    throw_pretty("Invalid argument: "
                 << "The armature dimension is wrong (it should be " + std::to_string(state_->get_nv()) + ")");
  }

  armature_ = armature;
  without_armature_ = false;
}

}  // namespace crocoddyl


namespace crocoddyl {
template <typename _Scalar>
struct ResidualDataCentroidalAngularMomentumTpl : public ResidualDataAbstractTpl<_Scalar> {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef _Scalar Scalar;
  typedef MathBaseTpl<Scalar> MathBase;
  typedef ResidualDataAbstractTpl<Scalar> Base;
  typedef DataCollectorAbstractTpl<Scalar> DataCollectorAbstract;
  typedef typename MathBase::Matrix6xs Matrix6xs;

  template <template <typename Scalar> class Model>
  ResidualDataCentroidalAngularMomentumTpl(Model<Scalar>* const model, DataCollectorAbstract* const data)
      : Base(model, data), dhd_dq(6, model->get_state()->get_nv()), dhd_dv(6, model->get_state()->get_nv()), dhd_da(6, model->get_state()->get_nv()), dh_dq(6, model->get_state()->get_nv()) {
    dhd_dq.setZero();
    dhd_dv.setZero();

    // Check that proper shared data has been passed
    DataCollectorMultibodyTpl<Scalar>* d = dynamic_cast<DataCollectorMultibodyTpl<Scalar>*>(shared);
    if (d == NULL) {
      throw_pretty("Invalid argument: the shared data should be derived from DataCollectorMultibody");
    }

    // Avoids data casting at runtime
    pinocchio = d->pinocchio;
  }

  pinocchio::DataTpl<Scalar>* pinocchio;  //!< Pinocchio data
  Matrix6xs dhd_dq;                       //!< Jacobian of the centroidal momentum
  Matrix6xs dhd_dv;                       //!< Jacobian of the centroidal momentum
  Matrix6xs dh_dq;                       //!< Jacobian of the centroidal momentum
  Matrix6xs dhd_da;                       //!< Jacobian of the centroidal momentum
  
  using Base::r;
  using Base::Ru;
  using Base::Rx;
  using Base::shared;
};

}  // namespace crocoddyl


namespace crocoddyl {

/**
 * @brief Centroidal momentum residual
 *
 * This residual function defines the centroidal momentum tracking as \f$\mathbf{r}=\mathbf{h}-\mathbf{h}^*\f$, where
 * \f$\mathbf{h},\mathbf{h}^*\in~\mathcal{X}\f$ are the current and reference centroidal momenta, respectively. Note
 * that the dimension of the residual vector is 6.
 * Furthermore, the Jacobians of the residual function are computed analytically.
 *
 * As described in `ResidualModelAbstractTpl()`, the residual value and its Jacobians are calculated by `calc` and
 * `calcDiff`, respectively.
 *
 * \sa `ResidualModelAbstractTpl`, `calc()`, `calcDiff()`, `createData()`
 */
template <typename _Scalar>
class ResidualModelCentroidalAngularMomentumTpl : public ResidualModelAbstractTpl<_Scalar> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef _Scalar Scalar;
  typedef MathBaseTpl<Scalar> MathBase;
  typedef ResidualModelAbstractTpl<Scalar> Base;
  typedef ResidualDataCentroidalAngularMomentumTpl<Scalar> Data;
  typedef StateKinodynamicTpl<Scalar> StateKinodynamic;
  typedef ResidualDataAbstractTpl<Scalar> ResidualDataAbstract;
  typedef ActivationModelAbstractTpl<Scalar> ActivationModelAbstract;
  typedef DataCollectorAbstractTpl<Scalar> DataCollectorAbstract;
  typedef typename MathBase::Vector6s Vector6s;
  typedef typename MathBase::VectorXs VectorXs;
  typedef typename MathBase::Matrix6xs Matrix6xs;

  /**
   * @brief Initialize the centroidal momentum residual model
   *
   * @param[in] state  State of the multibody system
   * @param[in] href   Reference centroidal momentum
   * @param[in] nu     Dimension of the control vector
   */
  ResidualModelCentroidalAngularMomentumTpl(boost::shared_ptr<StateKinodynamic> state, const Vector6s& href,
                                     const std::size_t nu);

  /**
   * @brief Initialize the centroidal momentum residual model
   *
   * The default `nu` is obtained from `StateAbstractTpl::get_nv()`.
   *
   * @param[in] state  State of the multibody system
   * @param[in] href   Reference centroidal momentum
   */
  ResidualModelCentroidalAngularMomentumTpl(boost::shared_ptr<StateKinodynamic> state, const std::size_t nu);
  virtual ~ResidualModelCentroidalAngularMomentumTpl();

  /**
   * @brief Compute the centroidal momentum residual
   *
   * @param[in] data  Centroidal momentum residual data
   * @param[in] x     State point \f$\mathbf{x}\in\mathbb{R}^{ndx}\f$
   * @param[in] u     Control input \f$\mathbf{u}\in\mathbb{R}^{nu}\f$
   */
  virtual void calc(const boost::shared_ptr<ResidualDataAbstract>& data, const Eigen::Ref<const VectorXs>& x,
                    const Eigen::Ref<const VectorXs>& u);

  /**
   * @brief Compute the derivatives of the centroidal momentum residual
   *
   * @param[in] data  Centroidal momentum residual data
   * @param[in] x     State point \f$\mathbf{x}\in\mathbb{R}^{ndx}\f$
   * @param[in] u     Control input \f$\mathbf{u}\in\mathbb{R}^{nu}\f$
   */
  virtual void calcDiff(const boost::shared_ptr<ResidualDataAbstract>& data, const Eigen::Ref<const VectorXs>& x,
                        const Eigen::Ref<const VectorXs>& u);

  /**
   * @brief Create the centroidal momentum residual data
   */
  virtual boost::shared_ptr<ResidualDataAbstract> createData(DataCollectorAbstract* const data);

  /**
   * @brief Return the reference centroidal momentum
   */
  const Vector6s& get_reference() const;

  /**
   * @brief Modify the reference centroidal momentum
   */
  void set_reference(const Vector6s& href);

  /**
   * @brief Print relevant information of the centroidal-momentum residual
   *
   * @param[out] os  Output stream object
   */
  virtual void print(std::ostream& os) const;

 protected:
  using Base::nu_;
  using Base::state_;
  using Base::u_dependent_;
  using Base::unone_;

 private:
  Vector6s href_;                                                         //!< Reference centroidal momentum
  boost::shared_ptr<typename StateKinodynamic::PinocchioModel> pin_model_;  //!< Pinocchio model
};
}

namespace crocoddyl {

template <typename Scalar>
ResidualModelCentroidalAngularMomentumTpl<Scalar>::ResidualModelCentroidalAngularMomentumTpl(boost::shared_ptr<StateKinodynamic> state,
                                                                               const Vector6s& href,
                                                                               const std::size_t nu)
    : Base(state, 1, nu, true, true, true, false), href_(href), pin_model_(state->get_pinocchio()) {}

template <typename Scalar>
ResidualModelCentroidalAngularMomentumTpl<Scalar>::ResidualModelCentroidalAngularMomentumTpl(boost::shared_ptr<StateKinodynamic> state,
                                                                               const std::size_t nu)
    : Base(state, 1, nu, true, true, true, false), pin_model_(state->get_pinocchio()) {}

template <typename Scalar>
ResidualModelCentroidalAngularMomentumTpl<Scalar>::~ResidualModelCentroidalAngularMomentumTpl() {}

template <typename Scalar>
void ResidualModelCentroidalAngularMomentumTpl<Scalar>::calc(const boost::shared_ptr<ResidualDataAbstract>& data,
                                                      const Eigen::Ref<const VectorXs>& x,
                                                      const Eigen::Ref<const VectorXs>& u) {
  // Compute the residual residual give the reference centroidal momentum
  Data* d = static_cast<Data*>(data.get());
  const Eigen::VectorBlock<const Eigen::Ref<const VectorXs>, Eigen::Dynamic> q = x.head(state_->get_nq());
  const Eigen::VectorBlock<const Eigen::Ref<const VectorXs>, Eigen::Dynamic> v = x.segment(state_->get_nq(),state_->get_nv());
  const Eigen::VectorBlock<const Eigen::Ref<const VectorXs>, Eigen::Dynamic> a = u.head(state_->get_nv());
  const Eigen::VectorBlock<const Eigen::Ref<const VectorXs>, Eigen::Dynamic> x_state = x.tail(4);
  const Eigen::VectorBlock<const Eigen::Ref<const VectorXs>, Eigen::Dynamic> u_state = u.tail(2);
  pinocchio::computeCentroidalMomentum(*pin_model_.get(), *d->pinocchio, q, v);
  data->r(0) = d->pinocchio->hg.toVector()(4) - x_state(3);

}

template <typename Scalar>
void ResidualModelCentroidalAngularMomentumTpl<Scalar>::calcDiff(const boost::shared_ptr<ResidualDataAbstract>& data,
                                                          const Eigen::Ref<const VectorXs>& x,
                                                          const Eigen::Ref<const VectorXs>& u) {
  Data* d = static_cast<Data*>(data.get());
  const std::size_t& nv = state_->get_nv();
  const Eigen::VectorBlock<const Eigen::Ref<const VectorXs>, Eigen::Dynamic> q = x.head(state_->get_nq());
  const Eigen::VectorBlock<const Eigen::Ref<const VectorXs>, Eigen::Dynamic> v = x.segment(state_->get_nq(),state_->get_nv());
  const Eigen::VectorBlock<const Eigen::Ref<const VectorXs>, Eigen::Dynamic> a = u.head(state_->get_nv());
  pinocchio::computeRNEADerivatives(*pin_model_.get(), *d->pinocchio, q, v, a);
  pinocchio::getCentroidalDynamicsDerivatives(*pin_model_.get(), *d->pinocchio,  d->dh_dq, d->dhd_dq, d->dhd_dv,  d->dhd_da);
  data->Rx.rightCols(1)(0) = -1;
  data->Rx.leftCols(nv) = d->dh_dq.block(4,0, 1, nv);
  data->Rx.block(0,nv,1,nv) = d->dhd_da.block(4,0, 1, nv);
}

template <typename Scalar>
boost::shared_ptr<ResidualDataAbstractTpl<Scalar> > ResidualModelCentroidalAngularMomentumTpl<Scalar>::createData(
    DataCollectorAbstract* const data) {
  return boost::allocate_shared<Data>(Eigen::aligned_allocator<Data>(), this, data);
}

template <typename Scalar>
void ResidualModelCentroidalAngularMomentumTpl<Scalar>::print(std::ostream& os) const {
  const Eigen::IOFormat fmt(2, Eigen::DontAlignCols, ", ", ";\n", "", "", "[", "]");
  os << "ResidualModelCentroidalAngularMomentum {href=" << href_.transpose().format(fmt) << "}";
}
}  // namespace crocoddyl

namespace crocoddyl {
template <typename _Scalar>
class ResidualModelCoMKinoPositionTpl : public ResidualModelAbstractTpl<_Scalar> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef _Scalar Scalar;
  typedef MathBaseTpl<Scalar> MathBase;
  typedef ResidualModelAbstractTpl<Scalar> Base;
  typedef ResidualDataCoMPositionTpl<Scalar> Data;
  typedef StateKinodynamicTpl<Scalar> StateKinodynamic;
  typedef ResidualDataAbstractTpl<Scalar> ResidualDataAbstract;
  typedef DataCollectorAbstractTpl<Scalar> DataCollectorAbstract;
  typedef typename MathBase::Vector3s Vector3s;
  typedef typename MathBase::VectorXs VectorXs;

  /**
   * @brief Initialize the CoM position residual model
   *
   * @param[in] state  State of the multibody system
   * @param[in] cref   Reference CoM position
   * @param[in] nu     Dimension of the control vector
   */
  ResidualModelCoMKinoPositionTpl(boost::shared_ptr<StateKinodynamic> state, const std::size_t nu);

  /**
   * @brief Initialize the CoM position residual model
   *
   * The default `nu` value is obtained from `StateAbstractTpl::get_nv()`.
   *
   * @param[in] state  State of the multibody system
   * @param[in] cref   Reference CoM position
   */
  ResidualModelCoMKinoPositionTpl(boost::shared_ptr<StateKinodynamic> state);
  virtual ~ResidualModelCoMKinoPositionTpl();

  /**
   * @brief Compute the CoM position residual
   *
   * @param[in] data  CoM position residual data
   * @param[in] x     State point \f$\mathbf{x}\in\mathbb{R}^{ndx}\f$
   * @param[in] u     Control input \f$\mathbf{u}\in\mathbb{R}^{nu}\f$
   */
  virtual void calc(const boost::shared_ptr<ResidualDataAbstract>& data, const Eigen::Ref<const VectorXs>& x,
                    const Eigen::Ref<const VectorXs>& u);

  /**
   * @brief Compute the derivatives of the CoM position residual
   *
   * @param[in] data  CoM position residual data
   * @param[in] x     State point \f$\mathbf{x}\in\mathbb{R}^{ndx}\f$
   * @param[in] u     Control input \f$\mathbf{u}\in\mathbb{R}^{nu}\f$
   */
  virtual void calcDiff(const boost::shared_ptr<ResidualDataAbstract>& data, const Eigen::Ref<const VectorXs>& x,
                        const Eigen::Ref<const VectorXs>& u);
  virtual boost::shared_ptr<ResidualDataAbstract> createData(DataCollectorAbstract* const data);

  /**
   * @brief Print relevant information of the com-position residual
   *
   * @param[out] os  Output stream object
   */
  virtual void print(std::ostream& os) const;

 protected:
  using Base::nu_;
  using Base::state_;
  using Base::u_dependent_;
  using Base::unone_;
  using Base::v_dependent_;

 private:
  Vector3s cref_;  //!< Reference CoM position
  boost::shared_ptr<typename StateKinodynamic::PinocchioModel> pin_model_;  //!< Pinocchio model
};

template <typename _Scalar>
struct ResidualDataCoMPositionTpl : public ResidualDataAbstractTpl<_Scalar> {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef _Scalar Scalar;
  typedef MathBaseTpl<Scalar> MathBase;
  typedef ResidualDataAbstractTpl<Scalar> Base;
  typedef DataCollectorAbstractTpl<Scalar> DataCollectorAbstract;
  typedef typename MathBase::Matrix3xs Matrix3xs;

  template <template <typename Scalar> class Model>
  ResidualDataCoMPositionTpl(Model<Scalar>* const model, DataCollectorAbstract* const data) : Base(model, data) {
    // Check that proper shared data has been passed
    DataCollectorMultibodyTpl<Scalar>* d = dynamic_cast<DataCollectorMultibodyTpl<Scalar>*>(shared);
    if (d == NULL) {
      throw_pretty("Invalid argument: the shared data should be derived from DataCollectorMultibody");
    }

    // Avoids data casting at runtime
    pinocchio = d->pinocchio;
  }

  pinocchio::DataTpl<Scalar>* pinocchio;  //!< Pinocchio data
  using Base::r;
  using Base::Ru;
  using Base::Rx;
  using Base::shared;
};

}  // namespace crocoddyl

namespace crocoddyl {

template <typename Scalar>
ResidualModelCoMKinoPositionTpl<Scalar>::ResidualModelCoMKinoPositionTpl(boost::shared_ptr<StateKinodynamic> state, const std::size_t nu)
    : Base(state, 1, nu, true, false, true, false), pin_model_(state->get_pinocchio())  {}

template <typename Scalar>
ResidualModelCoMKinoPositionTpl<Scalar>::ResidualModelCoMKinoPositionTpl(boost::shared_ptr<StateKinodynamic> state)
    : Base(state, 1, true, false, true, false), pin_model_(state->get_pinocchio())  {}

template <typename Scalar>
ResidualModelCoMKinoPositionTpl<Scalar>::~ResidualModelCoMKinoPositionTpl() {}

template <typename Scalar>
void ResidualModelCoMKinoPositionTpl<Scalar>::calc(const boost::shared_ptr<ResidualDataAbstract>& data,
                                               const Eigen::Ref<const VectorXs>& x, const Eigen::Ref<const VectorXs>& u) {
  // Compute the residual residual give the reference CoMPosition position
  Data* d = static_cast<Data*>(data.get());
  const Eigen::VectorBlock<const Eigen::Ref<const VectorXs>, Eigen::Dynamic> q = x.head(state_->get_nq());
  const Eigen::VectorBlock<const Eigen::Ref<const VectorXs>, Eigen::Dynamic> x_state = x.tail(4);
  pinocchio::centerOfMass(*pin_model_.get(), *d->pinocchio, q, false);
  data->r(0) = d->pinocchio->com[0](0) - x_state(0);
}

template <typename Scalar>
void ResidualModelCoMKinoPositionTpl<Scalar>::calcDiff(const boost::shared_ptr<ResidualDataAbstract>& data,
                                                   const Eigen::Ref<const VectorXs>& x,
                                                   const Eigen::Ref<const VectorXs>& u) {
  Data* d = static_cast<Data*>(data.get());
  const Eigen::VectorBlock<const Eigen::Ref<const VectorXs>, Eigen::Dynamic> q = x.head(state_->get_nq());

  pinocchio::jacobianCenterOfMass(*pin_model_.get(), *d->pinocchio, q, false);

  // Compute the derivatives of the frame placement
  const std::size_t nv = state_->get_nv();
  data->Rx.leftCols(nv) = d->pinocchio->Jcom.block(0,0,1,nv);
  (data->Rx.rightCols(4)).leftCols(1).setOnes();
  (data->Rx.rightCols(4)).leftCols(1) = -1 * (data->Rx.rightCols(4)).leftCols(1);
}

template <typename Scalar>
boost::shared_ptr<ResidualDataAbstractTpl<Scalar> > ResidualModelCoMKinoPositionTpl<Scalar>::createData(
    DataCollectorAbstract* const data) {
  return boost::allocate_shared<Data>(Eigen::aligned_allocator<Data>(), this, data);
}

template <typename Scalar>
void ResidualModelCoMKinoPositionTpl<Scalar>::print(std::ostream& os) const {
  const Eigen::IOFormat fmt(2, Eigen::DontAlignCols, ", ", ";\n", "", "", "[", "]");
  os << "ResidualModelCoMPosition {cref=" << cref_.transpose().format(fmt) << "}";
}
}  // namespace crocoddyl

namespace crocoddyl {

/**
 * @brief State residual
 *
 * This residual function defines the state tracking as \f$\mathbf{r}=\mathbf{x}\ominus\mathbf{x}^*\f$, where
 * \f$\mathbf{x},\mathbf{x}^*\in~\mathcal{X}\f$ are the current and reference states, respectively, which belong to the
 * state manifold \f$\mathcal{X}\f$. Note that the dimension of the residual vector is obtained from
 * `StateAbstract::get_ndx()`. Furthermore, the Jacobians of the residual function are
 * computed analytically.
 *
 * As described in `ResidualModelAbstractTpl()`, the residual value and its derivatives are calculated by `calc` and
 * `calcDiff`, respectively.
 *
 * \sa `ResidualModelAbstractTpl`, `calc()`, `calcDiff()`, `createData()`
 */
template <typename _Scalar>
class ResidualFlyStateTpl : public ResidualModelAbstractTpl<_Scalar> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef _Scalar Scalar;
  typedef MathBaseTpl<Scalar> MathBase;
  typedef ResidualModelAbstractTpl<Scalar> Base;
  typedef StateKinodynamicTpl<Scalar> StateKinodynamic;
  typedef ResidualDataAbstractTpl<Scalar> ResidualDataAbstract;
  typedef ActivationModelAbstractTpl<Scalar> ActivationModelAbstract;
  typedef DataCollectorAbstractTpl<Scalar> DataCollectorAbstract;
  typedef typename MathBase::VectorXs VectorXs;
  typedef typename MathBase::MatrixXs MatrixXs;

  /**
   * @brief Initialize the state residual model
   *
   * @param[in] state       State of the multibody system
   * @param[in] xref        Reference state
   * @param[in] nu          Dimension of the control vector
   */
  ResidualFlyStateTpl(boost::shared_ptr<typename Base::StateAbstract> state, const VectorXs& xref,
                        const std::size_t nu);

  /**
   * @brief Initialize the state residual model
   *
   * The default `nu` value is obtained from `StateAbstractTpl::get_nv()`.
   *
   * @param[in] state       State of the multibody system
   * @param[in] xref        Reference state
   */
  ResidualFlyStateTpl(boost::shared_ptr<typename Base::StateAbstract> state, const VectorXs& xref);

  /**
   * @brief Initialize the state residual model
   *
   * The default reference state is obtained from `StateAbstractTpl::zero()`.
   *
   * @param[in] state  State of the multibody system
   * @param[in] nu     Dimension of the control vector
   */
  ResidualFlyStateTpl(boost::shared_ptr<typename Base::StateAbstract> state, const std::size_t nu);

  /**
   * @brief Initialize the state residual model
   *
   * The default state reference is obtained from `StateAbstractTpl::zero()`, and `nu` from
   * `StateAbstractTpl::get_nv()`.
   *
   * @param[in] state       State of the multibody system
   * @param[in] activation  Activation model
   */
  ResidualFlyStateTpl(boost::shared_ptr<typename Base::StateAbstract> state);
  virtual ~ResidualFlyStateTpl();

  /**
   * @brief Compute the state residual
   *
   * @param[in] data  State residual data
   * @param[in] x     State point \f$\mathbf{x}\in\mathbb{R}^{ndx}\f$
   * @param[in] u     Control input \f$\mathbf{u}\in\mathbb{R}^{nu}\f$
   */
  virtual void calc(const boost::shared_ptr<ResidualDataAbstract>& data, const Eigen::Ref<const VectorXs>& x,
                    const Eigen::Ref<const VectorXs>& u);

  /**
   * @brief Compute the Jacobians of the state residual
   *
   * @param[in] data  State residual data
   * @param[in] x     State point \f$\mathbf{x}\in\mathbb{R}^{ndx}\f$
   * @param[in] u     Control input \f$\mathbf{u}\in\mathbb{R}^{nu}\f$
   */
  virtual void calcDiff(const boost::shared_ptr<ResidualDataAbstract>& data, const Eigen::Ref<const VectorXs>& x,
                        const Eigen::Ref<const VectorXs>& u);

  /**
   * @brief Return the reference state
   */
  const VectorXs& get_reference() const;

  /**
   * @brief Modify the reference state
   */
  void set_reference(const VectorXs& reference);

  /**
   * @brief Print relevant information of the state residual
   *
   * @param[out] os  Output stream object
   */
  virtual void print(std::ostream& os) const;

 protected:
  using Base::nu_;
  using Base::state_;
  using Base::u_dependent_;
  using Base::unone_;

 private:
  VectorXs xref_;  //!< Reference state
};

}  // namespace crocoddyl



namespace crocoddyl {

template <typename Scalar>
ResidualFlyStateTpl<Scalar>::ResidualFlyStateTpl(boost::shared_ptr<typename Base::StateAbstract> state,
                                                     const VectorXs& xref, const std::size_t nu)
    : Base(state, state->get_ndx(), nu, false, false, false, false, true), xref_(xref) {
  if (static_cast<std::size_t>(xref_.size()) != state_->get_nx() + 4) {
    throw_pretty("Invalid argument: "
                 << "xref has wrong dimension (it should be " + std::to_string(state_->get_nx()) + ")");
  }
}

template <typename Scalar>
ResidualFlyStateTpl<Scalar>::ResidualFlyStateTpl(boost::shared_ptr<typename Base::StateAbstract> state,
                                                     const VectorXs& xref)
    : Base(state, state->get_ndx(), false, false, false, false, true), xref_(xref) {
  if (static_cast<std::size_t>(xref_.size()) != state_->get_nx() + 4) {
    throw_pretty("Invalid argument: "
                 << "xref has wrong dimension (it should be " + std::to_string(state_->get_nx()) + ")");
  }
}

template <typename Scalar>
ResidualFlyStateTpl<Scalar>::ResidualFlyStateTpl(boost::shared_ptr<typename Base::StateAbstract> state,
                                                     const std::size_t nu)
    : Base(state, state->get_ndx(), nu, false, false, false, false, true), xref_(state->zero()) {}

template <typename Scalar>
ResidualFlyStateTpl<Scalar>::ResidualFlyStateTpl(boost::shared_ptr<typename Base::StateAbstract> state)
    : Base(state, state->get_ndx(), false, false, false, false, true), xref_(state->zero()) {}

template <typename Scalar>
ResidualFlyStateTpl<Scalar>::~ResidualFlyStateTpl() {}

template <typename Scalar>
void ResidualFlyStateTpl<Scalar>::calc(const boost::shared_ptr<ResidualDataAbstract>& data,
                                         const Eigen::Ref<const VectorXs>& x, const Eigen::Ref<const VectorXs>& u) {
  if (static_cast<std::size_t>(x.size()) != state_->get_nx() + 4) {
    throw_pretty("Invalid argument: "
                 << "x has wrong dimension (it should be " + std::to_string(state_->get_nx()) + ")");
  }
  state_->diff1(xref_, x, data->r); //diff1
} 

template <typename Scalar>
void ResidualFlyStateTpl<Scalar>::calcDiff(const boost::shared_ptr<ResidualDataAbstract>& data,
                                             const Eigen::Ref<const VectorXs>& x, const Eigen::Ref<const VectorXs>& u) {
  if (static_cast<std::size_t>(x.size()) != state_->get_nx() + 4) {
    throw_pretty("Invalid argument: "
                 << "x has wrong dimension (it should be " + std::to_string(state_->get_nx()) + ")");
  }
  state_->Jdiff1(xref_, x, data->Rx, data->Rx, second);//diff1
}

template <typename Scalar>
void ResidualFlyStateTpl<Scalar>::print(std::ostream& os) const {
  os << "ResidualFlyState";
}

template <typename Scalar>
const typename MathBaseTpl<Scalar>::VectorXs& ResidualFlyStateTpl<Scalar>::get_reference() const {
  return xref_;
}

template <typename Scalar>
void ResidualFlyStateTpl<Scalar>::set_reference(const VectorXs& reference) {
  xref_ = reference;
}
}  // namespace crocoddyl

namespace crocoddyl {

/**
 * @brief Frame translation residual
 *
 * This residual function defines the tracking of a frame translation as \f$\mathbf{r}=\mathbf{t}-\mathbf{t}^*\f$,
 * where \f$\mathbf{t},\mathbf{t}^*\in~\mathbb{R}^3\f$ are the current and reference frame translations, respectively.
 * Note that the dimension of the residual vector is 3. Furthermore, the Jacobians of the residual function are
 * computed analytically.
 *
 * As described in `ResidualModelAbstractTpl()`, the residual value and its Jacobians are calculated by `calc` and
 * `calcDiff`, respectively.
 *
 * \sa `ResidualModelAbstractTpl`, `calc()`, `calcDiff()`, `createData()`
 */
template <typename _Scalar>
class ResidualKinoFrameTranslationTpl : public ResidualModelAbstractTpl<_Scalar> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef _Scalar Scalar;
  typedef MathBaseTpl<Scalar> MathBase;
  typedef ResidualModelAbstractTpl<Scalar> Base;
  typedef ResidualDataFrameTranslationTpl<Scalar> Data;
  typedef StateKinodynamicTpl<Scalar> StateKinodynamic;
  typedef ResidualDataAbstractTpl<Scalar> ResidualDataAbstract;
  typedef DataCollectorAbstractTpl<Scalar> DataCollectorAbstract;
  typedef typename MathBase::VectorXs VectorXs;
  typedef typename MathBase::Vector3s Vector3s;

  /**
   * @brief Initialize the frame translation residual model
   *
   * @param[in] state  State of the multibody system
   * @param[in] id     Reference frame id
   * @param[in] xref   Reference frame translation
   * @param[in] nu     Dimension of the control vector
   */
  ResidualKinoFrameTranslationTpl(boost::shared_ptr<StateKinodynamic> state, const pinocchio::FrameIndex,
                                   const Vector3s& xref, const std::size_t nu);

  /**
   * @brief Initialize the frame translation residual model
   *
   * The default `nu` is equals to StateAbstractTpl::get_nv().
   *
   * @param[in] state  State of the multibody system
   * @param[in] id     Reference frame id
   * @param[in] xref   Reference frame translation
   */
  ResidualKinoFrameTranslationTpl(boost::shared_ptr<StateKinodynamic> state, const pinocchio::FrameIndex id,
                                   const Vector3s& xref);
  virtual ~ResidualKinoFrameTranslationTpl();

  /**
   * @brief Compute the frame translation residual
   *
   * @param[in] data  Frame translation residual data
   * @param[in] x     State point \f$\mathbf{x}\in\mathbb{R}^{ndx}\f$
   * @param[in] u     Control input \f$\mathbf{u}\in\mathbb{R}^{nu}\f$
   */
  virtual void calc(const boost::shared_ptr<ResidualDataAbstract>& data, const Eigen::Ref<const VectorXs>& x,
                    const Eigen::Ref<const VectorXs>& u);

  /**
   * @brief Compute the derivatives of the frame translation residual
   *
   * @param[in] data  Frame translation residual data
   * @param[in] x     State point \f$\mathbf{x}\in\mathbb{R}^{ndx}\f$
   * @param[in] u     Control input \f$\mathbf{u}\in\mathbb{R}^{nu}\f$
   */
  virtual void calcDiff(const boost::shared_ptr<ResidualDataAbstract>& data, const Eigen::Ref<const VectorXs>& x,
                        const Eigen::Ref<const VectorXs>& u);

  /**
   * @brief Create the frame translation residual data
   */
  virtual boost::shared_ptr<ResidualDataAbstract> createData(DataCollectorAbstract* const data);

  /**
   * @brief Return the reference frame id
   */
  pinocchio::FrameIndex get_id() const;

  /**
   * @brief Return the reference frame translation
   */
  const Vector3s& get_reference() const;

  /**
   * @brief Modify the reference frame id
   */
  void set_id(const pinocchio::FrameIndex id);

  /**
   * @brief Modify the reference frame translation reference
   */
  void set_reference(const Vector3s& reference);

  /**
   * @brief Print relevant information of the frame-translation residual
   *
   * @param[out] os  Output stream object
   */
  virtual void print(std::ostream& os) const;

 protected:
  using Base::nu_;
  using Base::state_;
  using Base::u_dependent_;
  using Base::unone_;
  using Base::v_dependent_;

 private:
  pinocchio::FrameIndex id_;                                              //!< Reference frame id
  Vector3s xref_;                                                         //!< Reference frame translation
  boost::shared_ptr<typename StateKinodynamic::PinocchioModel> pin_model_;  //!< Pinocchio model
};

}  // namespace crocoddyl

namespace crocoddyl {

template <typename Scalar>
ResidualKinoFrameTranslationTpl<Scalar>::ResidualKinoFrameTranslationTpl(boost::shared_ptr<StateKinodynamic> state,
                                                                           const pinocchio::FrameIndex id,
                                                                           const Vector3s& xref, const std::size_t nu)
    : Base(state, 3, nu, true, false, false, false), id_(id), xref_(xref), pin_model_(state->get_pinocchio()) {}

template <typename Scalar>
ResidualKinoFrameTranslationTpl<Scalar>::ResidualKinoFrameTranslationTpl(boost::shared_ptr<StateKinodynamic> state,
                                                                           const pinocchio::FrameIndex id,
                                                                           const Vector3s& xref)
    : Base(state, 3, true, false, false, false), id_(id), xref_(xref), pin_model_(state->get_pinocchio()) {}

template <typename Scalar>
ResidualKinoFrameTranslationTpl<Scalar>::~ResidualKinoFrameTranslationTpl() {}

template <typename Scalar>
void ResidualKinoFrameTranslationTpl<Scalar>::calc(const boost::shared_ptr<ResidualDataAbstract>& data,
                                                    const Eigen::Ref<const VectorXs>&,
                                                    const Eigen::Ref<const VectorXs>&) {
  // Compute the frame translation w.r.t. the reference frame
  Data* d = static_cast<Data*>(data.get());
  pinocchio::updateFramePlacement(*pin_model_.get(), *d->pinocchio, id_);
  data->r = d->pinocchio->oMf[id_].translation() - xref_;
}

template <typename Scalar>
void ResidualKinoFrameTranslationTpl<Scalar>::calcDiff(const boost::shared_ptr<ResidualDataAbstract>& data,
                                                        const Eigen::Ref<const VectorXs>&,
                                                        const Eigen::Ref<const VectorXs>&) {
  Data* d = static_cast<Data*>(data.get());

  // Compute the derivatives of the frame translation
  const std::size_t nv = state_->get_nv();
  pinocchio::getFrameJacobian(*pin_model_.get(), *d->pinocchio, id_, pinocchio::LOCAL, d->fJf);
  d->Rx.leftCols(nv).noalias() = d->pinocchio->oMf[id_].rotation() * d->fJf.template topRows<3>();
  ;
}

template <typename Scalar>
boost::shared_ptr<ResidualDataAbstractTpl<Scalar> > ResidualKinoFrameTranslationTpl<Scalar>::createData(
    DataCollectorAbstract* const data) {
  return boost::allocate_shared<Data>(Eigen::aligned_allocator<Data>(), this, data);
}

template <typename Scalar>
void ResidualKinoFrameTranslationTpl<Scalar>::print(std::ostream& os) const {
  const Eigen::IOFormat fmt(2, Eigen::DontAlignCols, ", ", ";\n", "", "", "[", "]");
  os << "ResidualKinoFrameTranslation {frame=" << pin_model_->frames[id_].name
     << ", tref=" << xref_.transpose().format(fmt) << "}";
}

template <typename Scalar>
pinocchio::FrameIndex ResidualKinoFrameTranslationTpl<Scalar>::get_id() const {
  return id_;
}

template <typename Scalar>
const typename MathBaseTpl<Scalar>::Vector3s& ResidualKinoFrameTranslationTpl<Scalar>::get_reference() const {
  return xref_;
}

template <typename Scalar>
void ResidualKinoFrameTranslationTpl<Scalar>::set_id(const pinocchio::FrameIndex id) {
  id_ = id;
}

template <typename Scalar>
void ResidualKinoFrameTranslationTpl<Scalar>::set_reference(const Vector3s& translation) {
  xref_ = translation;
}

}  // namespace crocoddyl





typedef crocoddyl::StateKinodynamicTpl<double> StateKinodynamic;
typedef crocoddyl::ActuationModelFloatingKinoBaseTpl<double> ActuationModelFloatingKinoBase;
typedef crocoddyl::DifferentialActionModelKinoDynamicsTpl<double> DifferentialActionModelContactKinoDynamics;
typedef crocoddyl::DifferentialActionDataKinoDynamicsTpl<double> DifferentialActionDataKinoDynamics;
typedef crocoddyl::ActivationModelWeightedQuadTpl<double> ActivationModelWeightedQuad;
typedef crocoddyl::ResidualFlyStateTpl<double> ResidualFlyState;
typedef crocoddyl::ResidualModelCentroidalAngularMomentumTpl<double> ResidualModelCentroidalAngularMomentum;
typedef crocoddyl::ActivationModelQuadraticBarrierTpl<double> ActivationModelQuadraticBarrier;
typedef crocoddyl::ActivationBoundsTpl<double> ActivationBounds;
typedef crocoddyl::ResidualModelCoMKinoPositionTpl<double> ResidualModelCoMKinoPosition;
typedef crocoddyl::ResidualKinoFrameTranslationTpl<double> ResidualKinoFrameTranslation;
typedef crocoddyl::MathBaseTpl<double> MathBase;
typename MathBase::VectorXs VectorXs;

std::vector<boost::shared_ptr<StateKinodynamic>> state_vector;
std::vector<boost::shared_ptr<ActuationModelFloatingKinoBase>> actuation_vector;
std::vector<boost::shared_ptr<crocoddyl::CostModelSum>> runningCostModel_vector;
std::vector<boost::shared_ptr<DifferentialActionModelContactKinoDynamics>> runningDAM_vector;
std::vector<boost::shared_ptr<crocoddyl::ActionModelAbstract>> runningModelWithRK4_vector;
std::vector<boost::shared_ptr<crocoddyl::ActionDataAbstract>> runningModelWithRK4_data;
std::vector<boost::shared_ptr<crocoddyl::CostModelAbstract>> xRegCost_vector;
std::vector<boost::shared_ptr<crocoddyl::CostModelAbstract>> uRegCost_vector;
std::vector<boost::shared_ptr<crocoddyl::CostModelAbstract>> stateBoundCost_vector;
std::vector<boost::shared_ptr<crocoddyl::CostModelAbstract>> camBoundCost_vector;
std::vector<boost::shared_ptr<crocoddyl::CostModelAbstract>> comBoundCost_vector;
std::vector<boost::shared_ptr<crocoddyl::CostModelAbstract>> foot_trackR;
std::vector<boost::shared_ptr<crocoddyl::CostModelAbstract>> foot_trackL;
std::vector<boost::shared_ptr<ActivationModelQuadraticBarrier>> state_activations;
std::vector<boost::shared_ptr<ActivationModelQuadraticBarrier>> state_activations2;
std::vector<boost::shared_ptr<ActivationModelQuadraticBarrier>> state_activations3;
std::vector<ActivationBounds> state_bounds;
std::vector<ActivationBounds> state_bounds2;
std::vector<ActivationBounds> state_bounds3;
std::vector<boost::shared_ptr<crocoddyl::DifferentialActionDataAbstract>> runningDAM_data;

boost::shared_ptr<StateKinodynamic> state;
boost::shared_ptr<ActuationModelFloatingKinoBase> actuation;

Eigen::VectorXd traj_, u_traj_, weight_quad, weight_quad_u;
Eigen::MatrixXd lb_, ub_, lb_2, ub_2, lb_3, ub_3;

boost::shared_ptr<DifferentialActionModelContactKinoDynamics> terminalDAM;
boost::shared_ptr<crocoddyl::CostModelSum> terminalCostModel;
boost::shared_ptr<crocoddyl::ActionModelAbstract> terminalModel;
Eigen::VectorXd x0;
Eigen::VectorXd u0;
boost::shared_ptr<crocoddyl::ShootingProblem> problemWithRK4;
std::vector<Eigen::VectorXd> xs, us;
std::vector<boost::shared_ptr<crocoddyl::CallbackAbstract>> cbs;

double dt_;

/*
unsigned int N = 10; // number of nodes
unsigned int T = 1;  // number of trials
unsigned int MAXITER = 100;*/

int N, T, MAXITER;
