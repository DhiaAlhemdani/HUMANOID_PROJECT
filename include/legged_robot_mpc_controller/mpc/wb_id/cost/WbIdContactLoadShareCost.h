#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

#include <ocs2_core/cost/StateInputGaussNewtonCostAd.h>

#include "legged_robot_mpc_controller/mpc/common/model/ModelSettings.h"
#include "legged_robot_mpc_controller/mpc/common/reference_manager/SwitchedModelReferenceManager.h"
#include "legged_robot_mpc_controller/mpc/wb_id/model/WbIdMpcRobotModel.h"

namespace ocs2::humanoid {

// [load_share_free_functions_begin]
// Free helpers shared with tests/tuning/loadshare_microbench.cpp (extracted verbatim by
// tests/tuning/run_loadshare_microbench.py). Plain doubles only: no ocs2/pinocchio types.
namespace load_share {

//! C1 smoothstep on [0, 1]: 3x^2 - 2x^3 (zero slope at both ends).
inline double smoothstep01(double x) {
  if (x <= 0.0) {
    return 0.0;
  }
  if (x >= 1.0) {
    return 1.0;
  }
  return x * x * (3.0 - 2.0 * x);
}

//! Load-transfer progress inside one double-support window (smoothstepped, clamped).
inline double transferProgress(double time, double entryTime, double exitTime) {
  const double span = exitTime - entryTime;
  if (!(span > 0.0)) {
    return 0.5;
  }
  return smoothstep01((time - entryTime) / span);
}

//! Share targets [phi_entering, phi_preDs]; they always sum to 1.
inline void shareTargets(double progress, double& phiEntering, double& phiPreDs) {
  phiEntering = progress;
  phiPreDs = 1.0 - progress;
}

//! Gauss-Newton rows r_i = sqrtW * (W_iz - phi_i * (W_0z + W_1z)); total-invariant:
//! zero whenever the split matches phi, regardless of the load total, so it shapes only
//! the free exchange direction and never the wrench sum pinned by the RNEA base rows.
template <typename S>
inline void splitResidual(S sqrtW, const S& wz0, const S& wz1, const S& phi0, const S& phi1, S* residual) {
  const S total = wz0 + wz1;
  residual[0] = sqrtW * (wz0 - phi0 * total);
  residual[1] = sqrtW * (wz1 - phi1 * total);
}

//! Exact double-support brackets from the mode-schedule event times: the last event
//! before `time` (entry) and the next one (exit). Returns false outside a bracketed
//! window (caller falls back to an even split); the pre-DS stance flags must contain
//! exactly one foot, i.e. the entry really is a single->double transition.
template <typename GetFlags>
inline bool dsBrackets(double time, const std::vector<double>& eventTimes, GetFlags getFlags, double& entryTime,
                       double& exitTime, bool (&flagsBefore)[2]) {
  if (eventTimes.empty()) {
    return false;
  }
  const auto next = std::upper_bound(eventTimes.begin(), eventTimes.end(), time);
  if (next == eventTimes.begin() || next == eventTimes.end()) {
    return false;  // before the first event or past the last: no complete window.
  }
  exitTime = *next;
  entryTime = *(next - 1);
  if (!(exitTime > entryTime)) {
    return false;
  }
  const auto now = getFlags(time);
  const auto before = getFlags(entryTime - 1e-9);
  const int beforeCount = (before[0] ? 1 : 0) + (before[1] ? 1 : 0);
  if (beforeCount != 1 || (before[0] == now[0] && before[1] == now[1])) {
    return false;  // not a single->double transition (or a redundant event).
  }
  flagsBefore[0] = before[0];
  flagsBefore[1] = before[1];
  return true;
}

}  // namespace load_share
// [load_share_free_functions_end]

/**
 * Double-support load-share cost (structural D2/D3 lever).
 *
 * ocs2 costs are per-stage functions of (t, x, u): a literal cross-node Δf_z term cannot
 * be expressed in this architecture. The adjacent-node force rate at contact handover is
 * nevertheless shaped here through the one direction the RNEA leaves free in double
 * support: the vertical force split between the feet. The cost pulls that split toward a
 * C1 smoothstep transfer across each DS window (phi = 0 for the landing foot at entry,
 * ramping to 1 by exit), which is continuous with the swing zero-wrench and the
 * single-stance constraints at both ends. The residual is total-invariant (zero at the
 * right split for any load), so it never fights the RNEA base rows.
 *
 * Inactive outside double support, where the split is pinned. The interface registers
 * this cost only when the weight is > 0 (shipped default 0 = disabled).
 */
class WbIdContactLoadShareCost final : public StateInputCostGaussNewtonAd {
 public:
  WbIdContactLoadShareCost(scalar_t weight, const SwitchedModelReferenceManager& referenceManager,
                           const WbIdMpcRobotModel<ad_scalar_t>& mpcRobotModelAD, const ModelSettings& modelSettings);
  ~WbIdContactLoadShareCost() override = default;
  WbIdContactLoadShareCost* clone() const override { return new WbIdContactLoadShareCost(*this); }

  //! Only double support leaves the split free; single stance and swing nodes are skipped.
  bool isActive(scalar_t time) const override;

  //! [sqrt(weight), phi_left, phi_right] for this evaluation time.
  vector_t getParameters(scalar_t time, const TargetTrajectories& targetTrajectories,
                         const PreComputation& preComputation) const override;

 private:
  WbIdContactLoadShareCost(const WbIdContactLoadShareCost& other);

  ad_vector_t costVectorFunction(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& input,
                                 const ad_vector_t& parameters) const override;

  const SwitchedModelReferenceManager* referenceManagerPtr_;
  double sqrtWeight_;
};

}  // namespace ocs2::humanoid
