#include "legged_robot_mpc_controller/mpc/wb_id/cost/WbIdContactLoadShareCost.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ocs2::humanoid {

WbIdContactLoadShareCost::WbIdContactLoadShareCost(scalar_t weight, scalar_t liftOffWindow, scalar_t touchDownWindow,
                                                   const SwitchedModelReferenceManager& referenceManager,
                                                   const WbIdMpcRobotModel<ad_scalar_t>& mpcRobotModelAD,
                                                   const ModelSettings& modelSettings)
    : StateInputCostGaussNewtonAd(),
      referenceManagerPtr_(&referenceManager),
      liftOffWindow_(liftOffWindow),
      touchDownWindow_(touchDownWindow) {
  if (!std::isfinite(weight) || weight <= 0.0) {
    throw std::invalid_argument(
        "[WbIdContactLoadShareCost] weight must be finite and > 0 (the interface skips registration at 0)");
  }
  if (!std::isfinite(liftOffWindow) || liftOffWindow < 0.0 || !std::isfinite(touchDownWindow) || touchDownWindow < 0.0) {
    throw std::invalid_argument("[WbIdContactLoadShareCost] the end windows must be finite and >= 0 (0 = whole DS)");
  }
  sqrtWeight_ = std::sqrt(weight);
  initialize(mpcRobotModelAD.getStateDim(), mpcRobotModelAD.getInputDim(), 3, "wb_id_contact_load_share_v1",
             modelSettings.modelFolderCppAd, modelSettings.recompileLibrariesCppAd, modelSettings.verboseCppAd);
}

WbIdContactLoadShareCost::WbIdContactLoadShareCost(const WbIdContactLoadShareCost& other)
    : StateInputCostGaussNewtonAd(other),
      referenceManagerPtr_(other.referenceManagerPtr_),
      sqrtWeight_(other.sqrtWeight_),
      liftOffWindow_(other.liftOffWindow_),
      touchDownWindow_(other.touchDownWindow_) {}

double WbIdContactLoadShareCost::endWindowScale(double time, double entryTime, double exitTime, double liftOffWindow,
                                                double touchDownWindow) {
  if (!(liftOffWindow > 0.0) && !(touchDownWindow > 0.0)) {
    return 1.0;
  }
  const auto ramp = [](double tau, double window) {
    if (!(window > 0.0) || tau >= window) {
      return 0.0;
    }
    const double x = 1.0 - std::max(tau, 0.0) / window;
    return x * x;
  };
  return std::min(1.0, ramp(exitTime - time, liftOffWindow) + ramp(time - entryTime, touchDownWindow));
}

bool WbIdContactLoadShareCost::isActive(scalar_t time) const {
  const contact_flag_t flags = referenceManagerPtr_->getContactFlags(time);
  return flags[0] && flags[1];
}

vector_t WbIdContactLoadShareCost::getParameters(scalar_t time, const TargetTrajectories& /*targetTrajectories*/,
                                                 const PreComputation& /*preComputation*/) const {
  vector_t params(3);
  params(0) = sqrtWeight_;
  // Fallback: neutral even split unless a complete single->double->single window brackets time
  // (with end windows: no bracket, no shaping).
  double phi0 = 0.5;
  double phi1 = 0.5;
  const bool windowed = liftOffWindow_ > 0.0 || touchDownWindow_ > 0.0;
  if (windowed) {
    params(0) = 0.0;
  }
  const auto& schedule = referenceManagerPtr_->getModeSchedule();
  const auto getter = [this](double t) { return referenceManagerPtr_->getContactFlags(t); };
  double entryTime = 0.0;
  double exitTime = 0.0;
  bool flagsBefore[2] = {false, false};
  if (load_share::dsBrackets(time, schedule.eventTimes, getter, entryTime, exitTime, flagsBefore)) {
    const double progress = load_share::transferProgress(time, entryTime, exitTime);
    double phiEntering = 0.0;
    double phiPreDs = 0.0;
    load_share::shareTargets(progress, phiEntering, phiPreDs);
    // The pre-DS stance foot hands its load over to the foot that landed at entry.
    phi0 = flagsBefore[0] ? phiPreDs : phiEntering;
    phi1 = flagsBefore[1] ? phiPreDs : phiEntering;
    if (windowed) {
      params(0) = sqrtWeight_ * std::sqrt(endWindowScale(time, entryTime, exitTime, liftOffWindow_, touchDownWindow_));
    }
  }
  params(1) = phi0;
  params(2) = phi1;
  return params;
}

ad_vector_t WbIdContactLoadShareCost::costVectorFunction(ad_scalar_t /*time*/, const ad_vector_t& /*state*/,
                                                         const ad_vector_t& input,
                                                         const ad_vector_t& parameters) const {
  const ad_scalar_t sqrtW = parameters(0);
  const ad_scalar_t phi0 = parameters(1);
  const ad_scalar_t phi1 = parameters(2);
  // Contact wrench layout u = [W_l(6), W_r(6), a, tau_j]; W = [f; m] world-aligned -> f_z at 6*i + 2.
  const ad_scalar_t wz0 = input(2);
  const ad_scalar_t wz1 = input(8);
  ad_vector_t residual(N_CONTACTS);
  load_share::splitResidual(sqrtW, wz0, wz1, phi0, phi1, residual.data());
  return residual;
}

}  // namespace ocs2::humanoid
