#pragma once

#include <ocs2_core/cost/StateInputCost.h>

#include "legged_robot_mpc_controller/mpc/common/reference_manager/SwitchedModelReferenceManager.h"
#include "legged_robot_mpc_controller/mpc/wb_id/model/WbIdMpcRobotModel.h"

namespace ocs2::humanoid {

/**
 * Quadratic cost on the wrench-rate inputs, 0.5 * dW' diag(weights) dW, on every node but the first.
 *
 * The first node's wrench is anchored to what the previous plan was applying (W_0 == Ws_0 with Ws_0
 * from the observation). Pricing the rate there as well makes every plan inherit the last plan's
 * force — a memory that lowers the force bandwidth of the balance loop (standing sway, then falls,
 * in the 2026-11-25 sweep at 1e-3 and 1e-2). This cost carries the in-horizon weight, where it shapes
 * the loading/unloading ramps at contact changes; the R block keeps only the first-node share
 * (costs.wrenchRateFirstNodeScale), which must stay positive for the SQP's input Hessian.
 */
class WbIdWrenchRateCost final : public StateInputCost {
 public:
  WbIdWrenchRateCost(vector_t weights, const SwitchedModelReferenceManager& referenceManager,
                     const WbIdMpcRobotModel<scalar_t>& mpcRobotModel);
  ~WbIdWrenchRateCost() override = default;
  WbIdWrenchRateCost* clone() const override { return new WbIdWrenchRateCost(*this); }

  bool isActive(scalar_t time) const override;
  scalar_t getValue(scalar_t time, const vector_t& state, const vector_t& input, const TargetTrajectories& targetTrajectories,
                    const PreComputation& preComp) const override;
  ScalarFunctionQuadraticApproximation getQuadraticApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                                 const TargetTrajectories& targetTrajectories,
                                                                 const PreComputation& preComp) const override;

 private:
  WbIdWrenchRateCost(const WbIdWrenchRateCost& rhs) = default;

  vector_t weights_;  // 6 * N_CONTACTS
  const SwitchedModelReferenceManager* referenceManagerPtr_;
  size_t stateDim_;
  size_t inputDim_;
  size_t rateStart_;
};

}  // namespace ocs2::humanoid
