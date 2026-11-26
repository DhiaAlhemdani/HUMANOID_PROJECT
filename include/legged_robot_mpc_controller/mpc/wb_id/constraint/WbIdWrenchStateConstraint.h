#pragma once

#include <ocs2_core/constraint/StateInputConstraint.h>

#include "legged_robot_mpc_controller/mpc/common/reference_manager/SwitchedModelReferenceManager.h"
#include "legged_robot_mpc_controller/mpc/wb_id/model/WbIdMpcRobotModel.h"

namespace ocs2::humanoid {

/**
 * Ties the input wrench of one contact to its wrench state in the wrench-as-state layout
 * (ModelSettings::wrenchState): W_c(u) - Ws_c(x) = 0 while the foot is in contact.
 *
 * With d(Ws)/dt = dW (WbIdDynamics) the stance wrench becomes a continuous quantity across the
 * horizon whose step between nodes is a priced input, while every wrench consumer (RNEA, friction
 * cone, zero-wrench, load share, controller) keeps reading the input.
 *
 * In swing the foot's ZeroWrenchConstraint already pins W_c = 0 through the same input columns;
 * this constraint is inactive then (the SQP projection needs independent rows) and
 * WbIdWrenchStateSwingCost pulls Ws_c to zero instead.
 */
class WbIdWrenchStateConstraint final : public StateInputConstraint {
 public:
  WbIdWrenchStateConstraint(const SwitchedModelReferenceManager& referenceManager, const WbIdMpcRobotModel<scalar_t>& mpcRobotModel,
                            size_t contactIndex);
  ~WbIdWrenchStateConstraint() override = default;
  WbIdWrenchStateConstraint* clone() const override { return new WbIdWrenchStateConstraint(*this); }

  bool isActive(scalar_t time) const override;
  size_t getNumConstraints(scalar_t /*time*/) const override { return 6; }
  vector_t getValue(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) const override;
  VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                           const PreComputation& preComp) const override;

 private:
  WbIdWrenchStateConstraint(const WbIdWrenchStateConstraint& rhs) = default;

  const SwitchedModelReferenceManager* referenceManagerPtr_;
  size_t contactIndex_;
  size_t stateDim_;
  size_t inputDim_;
  size_t wrenchInputStart_;
  size_t wrenchStateStart_;
};

}  // namespace ocs2::humanoid
