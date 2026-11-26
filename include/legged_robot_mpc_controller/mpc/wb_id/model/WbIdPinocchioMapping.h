#pragma once

#include <utility>

#include <ocs2_core/Types.h>
#include <ocs2_core/automatic_differentiation/Types.h>
#include <ocs2_pinocchio_interface/PinocchioStateInputMapping.h>

namespace ocs2::humanoid {

/**
 * OCS2 <-> pinocchio mapping for the whole-body state x = [q, v]: q is the state head,
 * v the state tail, and nothing in the input enters the kinematics.
 */
template <typename SCALAR_T>
class WbIdPinocchioMappingTpl final : public PinocchioStateInputMapping<SCALAR_T> {
 public:
  using vector_t = typename PinocchioStateInputMapping<SCALAR_T>::vector_t;
  using matrix_t = typename PinocchioStateInputMapping<SCALAR_T>::matrix_t;

  //! stateDim defaults to [q, v]; pass the model's state dimension when it carries more (wrench states).
  WbIdPinocchioMappingTpl(size_t generalizedCoordinatesDim, size_t inputDim, size_t stateDim = 0)
      : nq_(generalizedCoordinatesDim), inputDim_(inputDim), stateDim_(stateDim == 0 ? 2 * generalizedCoordinatesDim : stateDim) {}
  ~WbIdPinocchioMappingTpl() override = default;
  WbIdPinocchioMappingTpl* clone() const override { return new WbIdPinocchioMappingTpl(*this); }

  vector_t getPinocchioJointPosition(const vector_t& state) const override { return state.head(nq_); }

  vector_t getPinocchioJointVelocity(const vector_t& state, const vector_t& /*input*/) const override {
    return state.segment(nq_, nq_);
  }

  std::pair<matrix_t, matrix_t> getOcs2Jacobian(const vector_t& /*state*/, const matrix_t& Jq, const matrix_t& Jv) const override {
    matrix_t dfdx = matrix_t::Zero(Jq.rows(), stateDim_);
    dfdx.leftCols(2 * nq_) << Jq, Jv;
    return {dfdx, matrix_t::Zero(Jq.rows(), inputDim_)};
  }

 private:
  WbIdPinocchioMappingTpl(const WbIdPinocchioMappingTpl& rhs) = default;

  size_t nq_;
  size_t inputDim_;
  size_t stateDim_;
};

using WbIdPinocchioMapping = WbIdPinocchioMappingTpl<scalar_t>;
using WbIdPinocchioMappingCppAd = WbIdPinocchioMappingTpl<ad_scalar_t>;

}  // namespace ocs2::humanoid
