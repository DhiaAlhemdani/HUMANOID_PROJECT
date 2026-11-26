#include "legged_robot_mpc_controller/mpc/wb_id/cost/WbIdWrenchRateCost.h"

#include <stdexcept>
#include <utility>

namespace ocs2::humanoid {

WbIdWrenchRateCost::WbIdWrenchRateCost(vector_t weights, const SwitchedModelReferenceManager& referenceManager,
                                       const WbIdMpcRobotModel<scalar_t>& mpcRobotModel)
    : weights_(std::move(weights)),
      referenceManagerPtr_(&referenceManager),
      stateDim_(mpcRobotModel.getStateDim()),
      inputDim_(mpcRobotModel.getInputDim()),
      rateStart_(mpcRobotModel.getWrenchRateStartIndex(0)) {
  if (!mpcRobotModel.hasWrenchState()) {
    throw std::invalid_argument("[WbIdWrenchRateCost] the robot model has no wrench states (model.wrenchState is false)");
  }
  if (weights_.size() != 6 * N_CONTACTS || !(weights_.array() >= 0.0).all()) {
    throw std::invalid_argument("[WbIdWrenchRateCost] weights must be 6 * N_CONTACTS non-negative values");
  }
}

bool WbIdWrenchRateCost::isActive(scalar_t time) const {
  return !referenceManagerPtr_->isFirstNode(time);
}

scalar_t WbIdWrenchRateCost::getValue(scalar_t /*time*/, const vector_t& /*state*/, const vector_t& input,
                                      const TargetTrajectories& /*targetTrajectories*/, const PreComputation& /*preComp*/) const {
  const auto n = static_cast<Eigen::Index>(weights_.size());
  const vector_t rate = input.segment(static_cast<Eigen::Index>(rateStart_), n);
  return 0.5 * rate.dot(weights_.cwiseProduct(rate));
}

ScalarFunctionQuadraticApproximation WbIdWrenchRateCost::getQuadraticApproximation(scalar_t time, const vector_t& state,
                                                                                   const vector_t& input,
                                                                                   const TargetTrajectories& targetTrajectories,
                                                                                   const PreComputation& preComp) const {
  const auto n = static_cast<Eigen::Index>(weights_.size());
  const auto start = static_cast<Eigen::Index>(rateStart_);
  const vector_t rate = input.segment(start, n);
  ScalarFunctionQuadraticApproximation approx;
  approx.f = getValue(time, state, input, targetTrajectories, preComp);
  approx.dfdx = vector_t::Zero(static_cast<Eigen::Index>(stateDim_));
  approx.dfdu = vector_t::Zero(static_cast<Eigen::Index>(inputDim_));
  approx.dfdu.segment(start, n) = weights_.cwiseProduct(rate);
  approx.dfdxx = matrix_t::Zero(static_cast<Eigen::Index>(stateDim_), static_cast<Eigen::Index>(stateDim_));
  approx.dfdux = matrix_t::Zero(static_cast<Eigen::Index>(inputDim_), static_cast<Eigen::Index>(stateDim_));
  approx.dfduu = matrix_t::Zero(static_cast<Eigen::Index>(inputDim_), static_cast<Eigen::Index>(inputDim_));
  approx.dfduu.block(start, start, n, n).diagonal() = weights_;
  return approx;
}

}  // namespace ocs2::humanoid
