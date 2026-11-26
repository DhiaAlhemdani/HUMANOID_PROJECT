#include "legged_robot_mpc_controller/mpc/wb_id/dynamics/WbIdDynamics.h"

namespace ocs2::humanoid {

WbIdDynamics::WbIdDynamics(const WbIdMpcRobotModel<scalar_t>& mpcRobotModel)
    : SystemDynamicsBase(),
      nv_(mpcRobotModel.getGenCoordinatesDim()),
      accelerationStart_(mpcRobotModel.getGeneralizedAccelerationStartIndex()) {
  const auto stateDim = static_cast<Eigen::Index>(mpcRobotModel.getStateDim());
  const auto inputDim = static_cast<Eigen::Index>(mpcRobotModel.getInputDim());
  const auto nv = static_cast<Eigen::Index>(nv_);
  dfdx_ = matrix_t::Zero(stateDim, stateDim);
  dfdx_.block(0, nv, nv, nv).setIdentity();
  dfdu_ = matrix_t::Zero(stateDim, inputDim);
  dfdu_.block(nv, static_cast<Eigen::Index>(accelerationStart_), nv, nv).setIdentity();
  // Wrench states integrate the wrench-rate inputs.
  if (mpcRobotModel.hasWrenchState()) {
    wrenchDim_ = 6 * N_CONTACTS;
    wrenchStateStart_ = mpcRobotModel.getWrenchStateStartIndex(0);
    wrenchRateStart_ = mpcRobotModel.getWrenchRateStartIndex(0);
    dfdu_.block(static_cast<Eigen::Index>(wrenchStateStart_), static_cast<Eigen::Index>(wrenchRateStart_),
                static_cast<Eigen::Index>(wrenchDim_), static_cast<Eigen::Index>(wrenchDim_)).setIdentity();
  }
}

vector_t WbIdDynamics::computeFlowMap(scalar_t /*time*/, const vector_t& state, const vector_t& input, const PreComputation& /*preComp*/) {
  const auto nv = static_cast<Eigen::Index>(nv_);
  vector_t stateDerivative = vector_t::Zero(state.size());
  stateDerivative.head(nv) = state.segment(nv, nv);
  stateDerivative.segment(nv, nv) = input.segment(static_cast<Eigen::Index>(accelerationStart_), nv);
  if (wrenchDim_ > 0) {
    stateDerivative.segment(static_cast<Eigen::Index>(wrenchStateStart_), static_cast<Eigen::Index>(wrenchDim_)) =
        input.segment(static_cast<Eigen::Index>(wrenchRateStart_), static_cast<Eigen::Index>(wrenchDim_));
  }
  return stateDerivative;
}

VectorFunctionLinearApproximation WbIdDynamics::linearApproximation(scalar_t time,
                                                                    const vector_t& state,
                                                                    const vector_t& input,
                                                                    const PreComputation& preComp) {
  VectorFunctionLinearApproximation approximation;
  approximation.f = computeFlowMap(time, state, input, preComp);
  approximation.dfdx = dfdx_;
  approximation.dfdu = dfdu_;
  return approximation;
}

}  // namespace ocs2::humanoid
