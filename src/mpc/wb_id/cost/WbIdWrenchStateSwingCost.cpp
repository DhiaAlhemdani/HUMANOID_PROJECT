#include "legged_robot_mpc_controller/mpc/wb_id/cost/WbIdWrenchStateSwingCost.h"

#include <stdexcept>
#include <utility>

namespace ocs2::humanoid {

WbIdWrenchStateSwingCost::WbIdWrenchStateSwingCost(vector6_t weights, const SwitchedModelReferenceManager& referenceManager,
                                                   const WbIdMpcRobotModel<scalar_t>& mpcRobotModel, size_t contactIndex)
    : weights_(std::move(weights)),
      referenceManagerPtr_(&referenceManager),
      contactIndex_(contactIndex),
      stateDim_(mpcRobotModel.getStateDim()),
      wrenchStateStart_(mpcRobotModel.getWrenchStateStartIndex(contactIndex)) {
  if (!mpcRobotModel.hasWrenchState()) {
    throw std::invalid_argument("[WbIdWrenchStateSwingCost] the robot model has no wrench states (model.wrenchState is false)");
  }
  if (contactIndex >= N_CONTACTS) {
    throw std::invalid_argument("[WbIdWrenchStateSwingCost] contact index out of range");
  }
  if (!(weights_.array() >= 0.0).all()) {
    throw std::invalid_argument("[WbIdWrenchStateSwingCost] weights must be non-negative");
  }
}

bool WbIdWrenchStateSwingCost::isActive(scalar_t time) const {
  return !referenceManagerPtr_->getContactFlags(time)[contactIndex_];
}

scalar_t WbIdWrenchStateSwingCost::getValue(scalar_t /*time*/, const vector_t& state, const TargetTrajectories& /*targetTrajectories*/,
                                            const PreComputation& /*preComp*/) const {
  const vector6_t ws = state.segment<6>(static_cast<Eigen::Index>(wrenchStateStart_));
  return 0.5 * ws.dot(weights_.cwiseProduct(ws));
}

ScalarFunctionQuadraticApproximation WbIdWrenchStateSwingCost::getQuadraticApproximation(
    scalar_t time, const vector_t& state, const TargetTrajectories& targetTrajectories, const PreComputation& preComp) const {
  const auto start = static_cast<Eigen::Index>(wrenchStateStart_);
  const vector6_t ws = state.segment<6>(start);
  ScalarFunctionQuadraticApproximation approx;
  approx.f = getValue(time, state, targetTrajectories, preComp);
  approx.dfdx = vector_t::Zero(static_cast<Eigen::Index>(stateDim_));
  approx.dfdx.segment<6>(start) = weights_.cwiseProduct(ws);
  approx.dfdxx = matrix_t::Zero(static_cast<Eigen::Index>(stateDim_), static_cast<Eigen::Index>(stateDim_));
  approx.dfdxx.block<6, 6>(start, start).diagonal() = weights_;
  return approx;
}

}  // namespace ocs2::humanoid
