#pragma once

#include <ocs2_core/cost/StateCost.h>

#include "legged_robot_mpc_controller/mpc/common/reference_manager/SwitchedModelReferenceManager.h"
#include "legged_robot_mpc_controller/mpc/wb_id/model/WbIdMpcRobotModel.h"

namespace ocs2::humanoid {

/**
 * Quadratic pull of a swing foot's wrench state to zero: 0.5 * Ws_c' diag(weights) Ws_c while the
 * foot is not in contact (wrench-as-state layout, see WbIdWrenchStateConstraint).
 *
 * The input wrench is already pinned to zero in swing by ZeroWrenchConstraint; this keeps the
 * state copy there too, so the rate cost prices the unloading before lift-off and the loading
 * after touchdown rather than a free drift of Ws during the swing. Weighted like one node of the
 * rate cost (contactWrenchRates / dt^2), holding a load through a swing costs many times a
 * one-node dump, so Ws reaches zero within a couple of nodes.
 */
class WbIdWrenchStateSwingCost final : public StateCost {
 public:
  WbIdWrenchStateSwingCost(vector6_t weights, const SwitchedModelReferenceManager& referenceManager,
                           const WbIdMpcRobotModel<scalar_t>& mpcRobotModel, size_t contactIndex);
  ~WbIdWrenchStateSwingCost() override = default;
  WbIdWrenchStateSwingCost* clone() const override { return new WbIdWrenchStateSwingCost(*this); }

  bool isActive(scalar_t time) const override;
  scalar_t getValue(scalar_t time, const vector_t& state, const TargetTrajectories& targetTrajectories,
                    const PreComputation& preComp) const override;
  ScalarFunctionQuadraticApproximation getQuadraticApproximation(scalar_t time, const vector_t& state,
                                                                 const TargetTrajectories& targetTrajectories,
                                                                 const PreComputation& preComp) const override;

 private:
  WbIdWrenchStateSwingCost(const WbIdWrenchStateSwingCost& rhs) = default;

  vector6_t weights_;
  const SwitchedModelReferenceManager* referenceManagerPtr_;
  size_t contactIndex_;
  size_t stateDim_;
  size_t wrenchStateStart_;
};

}  // namespace ocs2::humanoid
