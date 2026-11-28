#include "legged_robot_mpc_controller/mpc/wb_id/constraint/WbIdWrenchStateConstraint.h"

#include <stdexcept>

namespace ocs2::humanoid {

WbIdWrenchStateConstraint::WbIdWrenchStateConstraint(const SwitchedModelReferenceManager& referenceManager,
                                                     const WbIdMpcRobotModel<scalar_t>& mpcRobotModel, size_t contactIndex)
    : StateInputConstraint(ConstraintOrder::Linear),
      referenceManagerPtr_(&referenceManager),
      contactIndex_(contactIndex),
      stateDim_(mpcRobotModel.getStateDim()),
      inputDim_(mpcRobotModel.getInputDim()),
      wrenchInputStart_(mpcRobotModel.getContactWrenchStartIndices(contactIndex)),
      wrenchStateStart_(mpcRobotModel.getWrenchStateStartIndex(contactIndex)) {
  if (!mpcRobotModel.hasWrenchState()) {
    throw std::invalid_argument("[WbIdWrenchStateConstraint] the robot model has no wrench states (model.wrenchState is false)");
  }
  if (contactIndex >= N_CONTACTS) {
    throw std::invalid_argument("[WbIdWrenchStateConstraint] contact index out of range");
  }
}

bool WbIdWrenchStateConstraint::isActive(scalar_t time) const {
  return referenceManagerPtr_->getContactFlags(time)[contactIndex_];
}

vector_t WbIdWrenchStateConstraint::getValue(scalar_t /*time*/, const vector_t& state, const vector_t& input,
                                             const PreComputation& /*preComp*/) const {
  return input.segment<6>(static_cast<Eigen::Index>(wrenchInputStart_)) - state.segment<6>(static_cast<Eigen::Index>(wrenchStateStart_));
}

VectorFunctionLinearApproximation WbIdWrenchStateConstraint::getLinearApproximation(scalar_t time, const vector_t& state,
                                                                                    const vector_t& input,
                                                                                    const PreComputation& preComp) const {
  VectorFunctionLinearApproximation approx;
  approx.f = getValue(time, state, input, preComp);
  approx.dfdx = matrix_t::Zero(6, static_cast<Eigen::Index>(stateDim_));
  approx.dfdx.middleCols<6>(static_cast<Eigen::Index>(wrenchStateStart_)) = -matrix_t::Identity(6, 6);
  approx.dfdu = matrix_t::Zero(6, static_cast<Eigen::Index>(inputDim_));
  approx.dfdu.middleCols<6>(static_cast<Eigen::Index>(wrenchInputStart_)).setIdentity();
  return approx;
}

}  // namespace ocs2::humanoid
