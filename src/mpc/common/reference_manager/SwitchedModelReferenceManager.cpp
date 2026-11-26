/******************************************************************************
Copyright (c) 2025, Manuel Yves Galliker. All rights reserved.
Copyright (c) 2024, 1X Technologies. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include "legged_robot_mpc_controller/mpc/common/reference_manager/SwitchedModelReferenceManager.h"

#include <legged_robot_mpc_controller/mpc/common/model/DynamicsHelperFunctions.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

namespace ocs2::humanoid {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SwitchedModelReferenceManager::SwitchedModelReferenceManager(std::shared_ptr<GaitSchedule> gaitSchedulePtr,
                                                             std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr,
                                                             const PinocchioInterface& pinocchioInterface,
                                                             const MpcRobotModelBase<scalar_t>& mpcRobotModel)
    : ReferenceManager(TargetTrajectories(), ModeSchedule()),
      gaitSchedulePtr_(std::move(gaitSchedulePtr)),
      swingTrajectoryPtr_(std::move(swingTrajectoryPtr)),
      // The reference manager gets a copy of the pinocchio model to use for initializing the ground height
      pinocchioInterface_(pinocchioInterface),
      mpcRobotModelPtr_(&mpcRobotModel) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
contact_flag_t SwitchedModelReferenceManager::getContactFlags(scalar_t time) const {
  return modeNumber2StanceLeg(this->getModeSchedule().modeAtTime(time));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

scalar_t SwitchedModelReferenceManager::getPhaseVariable(scalar_t time) const {
  // This feeds the arm-swing reference through getDesiredState(), so it runs on
  // every solver iteration. It used to dereference unconditionally: *it is out of
  // bounds when `time` is past the last event (it == end()), and *(it - 1) is out
  // of bounds when `time` precedes the first (it == begin()). Both happen in
  // normal operation, because the cost is evaluated across a horizon that can
  // extend beyond the schedule that was planned for it. The values read were
  // whatever happened to be adjacent in memory, which could make the phase
  // variable - and hence the commanded arm angles - jump arbitrarily.
  const auto& eventTimes = modeSchedule_.eventTimes;
  if (eventTimes.empty()) {
    return 0.0;
  }
  const auto it = std::upper_bound(eventTimes.begin(), eventTimes.end(), time);
  if (it == eventTimes.begin() || it == eventTimes.end()) {
    // Outside the planned schedule there is no meaningful phase; report the start
    // of the cycle rather than extrapolating from out-of-range samples.
    return 0.0;
  }
  const scalar_t nextEventTime = *it;
  const scalar_t prevEventTime = *(it - 1);
  const scalar_t cycleDuration = nextEventTime - prevEventTime;
  if (!(cycleDuration > 0.0)) {
    return 0.0;  // coincident events: no phase to interpolate, and no division
  }

  if (modeSchedule_.modeAtTime(time) == LF) {
    return (0.5 * (time - prevEventTime) / cycleDuration);
  } else if (modeSchedule_.modeAtTime(time) == RF) {
    return (0.5 + 0.5 * (time - prevEventTime) / cycleDuration);
  } else {
    if (modeSchedule_.modeAtTime(prevEventTime - 0.01) == LF) {
      return 0.5;
    } else {
      return 0;
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t SwitchedModelReferenceManager::adaptToCurrentGroundHeight(TargetTrajectories& targetTrajectories,
                                                                   const vector_t& initState,
                                                                   size_t initMode) {
  scalar_t terrainHeight = computeGroundHeightEstimate(pinocchioInterface_, *mpcRobotModelPtr_,
                                                       mpcRobotModelPtr_->getGeneralizedCoordinates(initState), initMode);

  // This value is the swing planner's ground reference and the base-height target
  // offset, so pinning it to zero hard-codes a flat floor at the world origin -
  // the estimate above was computed and then thrown away. That is the single
  // thing preventing the stack from walking on sloped or stepped ground, no
  // matter how well the state estimator tracks the terrain: the swing foot would
  // still be aimed at z = 0.
  //
  // Off by default because every gain in this configuration was tuned against the
  // flat assumption, and because the estimate is only as good as the foot heights
  // the estimator supplies - on flat ground it is strictly worse than the
  // constant it replaces. Turn it on together with an InEKF height source that
  // does not itself assume a flat plane ("inekf" or "anchored"; "kinematic" and
  // "blend" both pin the feet to groundZ and would defeat the point).
  if (!useTerrainHeightEstimate_) {
    terrainHeight = 0.0;
  } else if (!std::isfinite(terrainHeight)) {
    terrainHeight = previousGroundHeightEstimate_;  // never feed a NaN to the planner
  } else {
    // Rate-limit per solve: a mis-detected contact can otherwise step the
    // reference by most of a leg length in one iteration, which the swing planner
    // turns into an impossible foot trajectory.
    terrainHeight = std::clamp(terrainHeight,
                               previousGroundHeightEstimate_ - maxTerrainHeightStep_,
                               previousGroundHeightEstimate_ + maxTerrainHeightStep_);
  }

  // adapt target Trajectories to current terrain height
  // Since they are published in the past the current observations ground height might have drifted.

  // Adapt the ground height difference for every state in the target Trajectories.
  // The height difference between last update and the current update is applied here
  // to prevent applying the same difference twice in case the trajectories have not been updated.
  for (size_t i = 0; i < targetTrajectories.stateTrajectory.size(); i++) {
    vector_t& targetState = targetTrajectories.stateTrajectory[i];
    scalar_t heightDifference = terrainHeight - previousGroundHeightEstimate_;
    mpcRobotModelPtr_->adaptBasePoseHeight(targetState, heightDifference);
  }
  previousGroundHeightEstimate_ = terrainHeight;
  return terrainHeight;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

vector_t SwitchedModelReferenceManager::getDesiredState(const TargetTrajectories& targetTrajectories,
                                                        const vector_t& state,
                                                        scalar_t time) const {
  vector_t xNominal = targetTrajectories.getDesiredState(time);

  if (armSwing_.enabled) {
    vector_t desiredJointAngles = mpcRobotModelPtr_->getJointAngles(xNominal);
    addArmSwingOffsets(desiredJointAngles, xNominal, state, time);
    mpcRobotModelPtr_->setJointAngles(xNominal, desiredJointAngles);
  }
  if (humanGait_.enabled) {
    addHumanGaitTemplate(xNominal, time);
  }
  return xNominal;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
namespace {

// Normative sagittal joint angles of normal human walking over one gait cycle of a leg, every 5 %
// from its heel strike (0 %) back to its heel strike (100 %), in degrees. Flexion positive for the hip
// and the knee, dorsiflexion positive for the ankle. The same values live in tests/tuning/human_gait.py
// (the human-likeness score). Toe-off of this leg is at 60 %, the other leg strikes at 50 %.
constexpr std::array<double, 21> kHumanHipFlexionDeg = {30, 29, 26, 22, 17, 12, 7, 2, -3, -7, -10, -10, -6, 2, 12, 21, 28, 32, 33, 32, 30};
constexpr std::array<double, 21> kHumanKneeFlexionDeg = {5, 10, 16, 18, 16, 13, 10, 7, 5, 6, 10, 19, 35, 52, 62, 62, 55, 42, 27, 13, 5};
constexpr std::array<double, 21> kHumanAnkleDorsiflexionDeg = {0, -4, -6, -3, 0, 3, 6, 8, 9, 10, 9, 4, -8, -17, -16, -10, -5, -2, 0, 1, 0};

// Cycle mean of a curve (the 21st knot repeats the first).
scalar_t curveMeanDeg(const std::array<double, 21>& curve) {
  double sum = 0.0;
  for (size_t i = 0; i < 20; ++i) {
    sum += curve[i];
  }
  return sum / 20.0;
}

scalar_t humanCurveRad(const std::array<double, 21>& curve, scalar_t phase) {
  const scalar_t p = phase - std::floor(phase);
  const scalar_t x = p * 20.0;
  size_t i = static_cast<size_t>(x);
  if (i >= 20) {
    i = 19;
  }
  const scalar_t a = x - static_cast<scalar_t>(i);
  return ((1.0 - a) * curve[i] + a * curve[i + 1]) * M_PI / 180.0;
}

}  // namespace

void SwitchedModelReferenceManager::setHumanGaitSettings(const HumanGaitSettings& settings) {
  humanGait_ = settings;
  if (!settings.enabled) {
    return;
  }
  if (settings.jointNames.size() != humanGaitJointIndices_.size()) {
    throw std::invalid_argument("[SwitchedModelReferenceManager] humanGait.jointNames needs 7 entries "
                                "[l_hip_pitch, l_knee, l_ankle_pitch, r_hip_pitch, r_knee, r_ankle_pitch, waist_yaw]");
  }
  const auto& names = mpcRobotModelPtr_->modelSettings.mpcModelJointNames;
  for (size_t i = 0; i < humanGaitJointIndices_.size(); ++i) {
    const auto it = std::find(names.begin(), names.end(), settings.jointNames[i]);
    if (it == names.end()) {
      throw std::invalid_argument("[SwitchedModelReferenceManager] humanGait joint '" + settings.jointNames[i] +
                                  "' is not an MPC joint");
    }
    humanGaitJointIndices_[i] = static_cast<size_t>(it - names.begin());
  }
  if (settings.arms && !mpcRobotModelPtr_->modelSettings.hasArmSwingJoints) {
    throw std::invalid_argument("[SwitchedModelReferenceManager] humanGait.arms needs reference.armSwing.jointNames");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SwitchedModelReferenceManager::StridePhase SwitchedModelReferenceManager::getLegStridePhase(size_t leg, scalar_t time) const {
  StridePhase out;
  const auto& eventTimes = modeSchedule_.eventTimes;
  const auto& modes = modeSchedule_.modeSequence;
  const size_t n = modes.size();
  if (n < 2 || eventTimes.size() + 1 != n) {
    return out;
  }
  const size_t other = 1 - leg;
  const auto inContact = [&](size_t i, size_t l) { return modeNumber2StanceLeg(modes[i])[l]; };
  // modes[i] is active on [eventTimes[i-1], eventTimes[i]); the change into modes[i] happens at eventTimes[i-1].
  const auto changeTime = [&](size_t i) { return eventTimes[i - 1]; };
  // First i > from where leg l changes into `toContact`; n when there is none.
  const auto nextChange = [&](size_t from, size_t l, bool toContact) {
    for (size_t i = from + 1; i < n; ++i) {
      if (inContact(i, l) == toContact && inContact(i - 1, l) != toContact) {
        return i;
      }
    }
    return n;
  };

  size_t k = static_cast<size_t>(std::upper_bound(eventTimes.begin(), eventTimes.end(), time) - eventTimes.begin());
  if (k >= n) {
    k = n - 1;
  }
  // Latest contact change of this leg at or before now, and the heel strike that started its cycle.
  size_t latest = n;
  for (size_t i = k; i > 0; --i) {
    if (inContact(i, leg) != inContact(i - 1, leg)) {
      latest = i;
      break;
    }
  }
  if (latest == n) {
    return out;  // this leg never changed contact: standing
  }
  size_t heelStrike = n;
  if (inContact(latest, leg)) {
    heelStrike = latest;
  } else {
    for (size_t i = latest - 1; i > 0; --i) {
      if (inContact(i, leg) && !inContact(i - 1, leg)) {
        heelStrike = i;
        break;
      }
    }
  }

  if (heelStrike == n) {
    // First swing out of standing: no heel strike before it, only the touchdown ahead.
    const size_t touchdown = nextChange(latest, leg, true);
    if (touchdown >= n) {
      return out;
    }
    const scalar_t t0 = changeTime(latest);
    const scalar_t t1 = changeTime(touchdown);
    if (!(t1 > t0)) {
      return out;
    }
    out.valid = true;
    out.stepTime = (t1 - t0) / 0.8;  // the swing is 40 % of the cycle = 0.8 step
    out.phase = std::clamp(0.6 + 0.4 * (time - t0) / (t1 - t0), 0.6, 1.0 - 1e-9);
    return out;
  }

  const scalar_t hs = changeTime(heelStrike);
  const size_t iToOther = nextChange(heelStrike, other, false);
  const size_t iHsOther = iToOther < n ? nextChange(iToOther, other, true) : n;
  const size_t iTo = nextChange(heelStrike, leg, false);
  const size_t iHsNext = iTo < n ? nextChange(iTo, leg, true) : n;

  scalar_t stepTime = std::numeric_limits<scalar_t>::quiet_NaN();
  if (iHsOther < n) {
    stepTime = changeTime(iHsOther) - hs;
  } else if (iHsNext < n) {
    stepTime = 0.5 * (changeTime(iHsNext) - hs);
  } else if (iTo < n) {
    stepTime = (changeTime(iTo) - hs) / 1.2;
  }
  if (!(stepTime > 0.0)) {
    return out;  // nobody steps after this heel strike: the robot stops here
  }
  // Missing boundaries (schedule ends inside the cycle) follow the human proportions.
  const scalar_t tToOther = iToOther < n ? changeTime(iToOther) : hs + 0.2 * stepTime;
  const scalar_t tHsOther = iHsOther < n ? changeTime(iHsOther) : hs + stepTime;
  if (iTo >= n && iHsOther < n && time >= tHsOther) {
    return out;  // the other leg lands and this one never leaves the ground again: the last step, then standing
  }
  const scalar_t tTo = iTo < n ? changeTime(iTo) : hs + 1.2 * stepTime;
  const scalar_t tHsNext = iHsNext < n ? changeTime(iHsNext) : hs + 2.0 * stepTime;

  scalar_t phase;
  if (time < tToOther && tToOther > hs) {
    phase = 0.10 * (time - hs) / (tToOther - hs);
  } else if (time < tHsOther && tHsOther > tToOther) {
    phase = 0.10 + 0.40 * (time - tToOther) / (tHsOther - tToOther);
  } else if (time < tTo && tTo > tHsOther) {
    phase = 0.50 + 0.10 * (time - tHsOther) / (tTo - tHsOther);
  } else if (tHsNext > tTo) {
    phase = 0.60 + 0.40 * (time - tTo) / (tHsNext - tTo);
  } else {
    return out;
  }
  out.valid = true;
  out.stepTime = stepTime;
  out.phase = std::clamp(phase, 0.0, 1.0 - 1e-9);
  return out;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SwitchedModelReferenceManager::addHumanGaitTemplate(vector_t& xNominal, scalar_t time) const {
  const HumanGaitSettings& g = humanGait_;
  const StridePhase left = getLegStridePhase(0, time);
  const StridePhase right = getLegStridePhase(1, time);

  // Commanded forward speed in the heading frame (the reference velocity, as for the arm swing).
  const vector3_t linVelCommand = mpcRobotModelPtr_->getBaseComLinearVelocity(xNominal);
  const vector6_t nominalPose = mpcRobotModelPtr_->getBasePose(xNominal);
  const scalar_t yaw = nominalPose[3];
  const scalar_t forwardSpeed = std::cos(yaw) * linVelCommand[0] + std::sin(yaw) * linVelCommand[1];
  const scalar_t stepTime = right.valid ? right.stepTime : (left.valid ? left.stepTime : 0.0);
  // Leg curves scale with the commanded step length relative to the leg; arms with speed. Backward,
  // lateral and turning commands give 0: the stance posture, the MPC's own stepping.
  const scalar_t legScale = std::clamp(forwardSpeed * stepTime / (g.legLength * g.humanStepRatio), 0.0, g.maxScale);
  const scalar_t armScale = std::clamp(std::max(forwardSpeed, 0.0) / g.referenceSpeed, 0.0, g.maxScale);
  constexpr scalar_t kTwoPi = 2.0 * M_PI;

  vector_t q = mpcRobotModelPtr_->getJointAngles(xNominal);

  if (g.legs) {
    const scalar_t half = 0.5 * g.kneeOffset;
    // The hip and ankle tables are trunk-relative angles of a trunk that leans ~10 deg forward in human
    // walking (hip cycle mean +13 deg); this pelvis is held level, so by default the curves are used as
    // motion about the robot's own stance posture (mean removed). The knee is absolute: 0 is straight.
    const scalar_t hipMean = g.removeHipMean ? curveMeanDeg(kHumanHipFlexionDeg) * M_PI / 180.0 : 0.0;
    const scalar_t ankleMean = g.removeHipMean ? curveMeanDeg(kHumanAnkleDorsiflexionDeg) * M_PI / 180.0 : 0.0;
    const auto legReference = [&](size_t leg, const StridePhase& w) {
      const scalar_t phase = w.valid ? w.phase : 0.3;  // standing: the mid-stance point of the curves at scale 0
      const scalar_t scale = w.valid ? legScale : 0.0;
      // Robot conventions (G1): hip pitch and ankle pitch positive = extension / plantarflexion, knee positive = flexion.
      q[humanGaitJointIndices_[3 * leg + 0]] = -(scale * (humanCurveRad(kHumanHipFlexionDeg, phase) - hipMean)) - half;
      q[humanGaitJointIndices_[3 * leg + 1]] = scale * humanCurveRad(kHumanKneeFlexionDeg, phase) + g.kneeOffset;
      q[humanGaitJointIndices_[3 * leg + 2]] = -(scale * (humanCurveRad(kHumanAnkleDorsiflexionDeg, phase) - ankleMean)) - half;
    };
    legReference(0, left);
    legReference(1, right);
  }

  // The base and trunk follow the right leg's cycle (the left is the same cycle half a period on).
  const bool cycleValid = right.valid || left.valid;
  const scalar_t cyclePhase = right.valid ? right.phase : (left.valid ? left.phase + 0.5 : 0.0);

  if (g.arms && mpcRobotModelPtr_->modelSettings.hasArmSwingJoints) {
    const ModelSettings& ms = mpcRobotModelPtr_->modelSettings;
    const auto clampArm = [&](scalar_t v) { return std::clamp(v, -g.armMaxOffset, g.armMaxOffset); };
    // The arm is back at its own leg's heel strike and forward at the other leg's: cos of the own-leg phase.
    // Shoulder pitch positive = arm back; elbow positive = flexion, largest with the arm forward.
    const auto armReference = [&](size_t shoulder, size_t elbow, const StridePhase& w) {
      const scalar_t c = w.valid ? std::cos(kTwoPi * w.phase) : 0.0;
      q[shoulder] += clampArm(g.shoulderAmplitude * armScale * c);
      q[elbow] += clampArm(g.elbowMean - g.elbowAmplitude * armScale * c);
    };
    armReference(ms.j_l_shoulder_y_index, ms.j_l_elbow_y_index, left);
    armReference(ms.j_r_shoulder_y_index, ms.j_r_elbow_y_index, right);
  }

  if (g.waist && cycleValid) {
    // Trunk counter-rotation: at right heel strike the pelvis has the right side forward and the
    // shoulders the left side forward, i.e. positive yaw of the trunk relative to the pelvis.
    q[humanGaitJointIndices_[6]] += g.waistYawAmplitude * legScale * std::cos(kTwoPi * cyclePhase);
  }
  mpcRobotModelPtr_->setJointAngles(xNominal, q);

  if (g.base && cycleValid) {
    vector6_t pose = nominalPose;
    // Vertical excursion: high at mid single stance (30 %, 80 %), low in double support (5 %, 55 %).
    pose[2] += g.heightOscillation * legScale * legScale * (-std::cos(2.0 * kTwoPi * (cyclePhase - 0.05)));
    // Pelvis pitch, twice per cycle (sign left to the amplitude).
    pose[4] += g.pelvisPitchAmplitude * legScale * (-std::cos(2.0 * kTwoPi * cyclePhase));
    // Obliquity: the swing-side hip drops; right swing is centred at 80 % (roll positive = right side down).
    pose[5] += g.pelvisRollAmplitude * legScale * std::sin(kTwoPi * (cyclePhase - 0.55));
    mpcRobotModelPtr_->setBasePose(xNominal, pose);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SwitchedModelReferenceManager::addArmSwingOffsets(vector_t& desiredJointAngles,
                                                       const vector_t& xNominal,
                                                       const vector_t& state,
                                                       scalar_t time) const {
  // ------------------------------------------------------------------------
  // Phase-1 law: an open-loop sinusoid locked to the gait phase and scaled by
  // the commanded forward speed. Standing still (speed 0) leaves the arms
  // exactly at the nominal posture, which is what keeps them hanging at the
  // sides instead of drifting.
  //
  // ------------------------------------------------------------------------
  const scalar_t phaseVariable = this->getPhaseVariable(time);
  const vector3_t linVelCommand = mpcRobotModelPtr_->getBaseComLinearVelocity(xNominal);
  const scalar_t currentEulerZ = mpcRobotModelPtr_->getBasePose(state)[3];

  // Commanded velocity resolved into the heading frame: the swing follows how
  // fast the robot is asked to walk FORWARD, not how fast it moves in world x.
  const scalar_t localVelXCommand =
      (std::cos(currentEulerZ) * linVelCommand[0] + std::sin(currentEulerZ) * linVelCommand[1]);

  const scalar_t gaitCycleFactor =
      std::sin(2.0 * M_PI * (phaseVariable - armSwing_.phaseOffset)) * localVelXCommand;

  // The MPC carries no self-collision constraint, so this clamp is the only
  // thing holding the arm reference inside the envelope that was measured
  // collision-free on the model. It has no effect at ordinary walking speeds
  // (0.15 rad per m/s against a 0.6 rad bound); it exists so that a large speed
  // command, or a future phase-2 law, cannot drive the hands into the hips.
  const auto clampToEnvelope = [this](scalar_t offset) {
    return std::max(-armSwing_.maxOffset, std::min(armSwing_.maxOffset, offset));
  };
  const scalar_t shoulderOffset = clampToEnvelope(armSwing_.shoulderPitchAmplitude * gaitCycleFactor);
  const scalar_t elbowOffset = clampToEnvelope(armSwing_.elbowAmplitude * gaitCycleFactor);

  // Antisymmetric across the body: the arms counter-swing, so their net linear
  // momentum cancels while the pair still applies a yaw moment opposing the one
  // the swinging leg generates.
  const ModelSettings& modelSettings = mpcRobotModelPtr_->modelSettings;
  if (!modelSettings.hasArmSwingJoints) {
    return;
  }
  desiredJointAngles[modelSettings.j_l_shoulder_y_index] += -shoulderOffset;
  desiredJointAngles[modelSettings.j_r_shoulder_y_index] += shoulderOffset;
  desiredJointAngles[modelSettings.j_l_elbow_y_index] += -elbowOffset;
  desiredJointAngles[modelSettings.j_r_elbow_y_index] += elbowOffset;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SwitchedModelReferenceManager::modifyReferences(scalar_t initTime,
                                                     scalar_t finalTime,
                                                     const vector_t& initState,
                                                     size_t initMode,
                                                     TargetTrajectories& targetTrajectories,
                                                     ModeSchedule& modeSchedule) {
  solveInitTime_ = initTime;
  // Swap in the latest external targets on the solver thread before the costs read them.
  externalJointTargets_.updateFromBuffer();
  stairClimbingPlan_.updateFromBuffer();
  terrainWalkActive_.updateFromBuffer();
  baseTrackingMode_.updateFromBuffer();

  const auto timeHorizon = finalTime - initTime;
  modeSchedule = gaitSchedulePtr_->getModeSchedule(initTime - timeHorizon, finalTime + timeHorizon);
  const auto& swingHeights = gaitSchedulePtr_->getSwingHeightSequence();

  const auto& stairPlan = stairClimbingPlan_.get();
  if (stairPlan) {
    // Fixed-sequence stair climbing: feed the planned per-phase support heights
    // to the swing planner instead of the flat-ground estimate.
    feet_array_t<scalar_array_t> liftOffHeightSequence;
    feet_array_t<scalar_array_t> touchDownHeightSequence;
    stairPlan->getHeightSequences(modeSchedule, liftOffHeightSequence, touchDownHeightSequence);
    swingTrajectoryPtr_->update(modeSchedule, liftOffHeightSequence, touchDownHeightSequence, swingHeights);
  } else if (isTerrainWalkActive()) {
    // Terrain-aware walking: re-plan the footsteps over the ground-truth
    // terrain for the current window, terrain-adapt the base height reference,
    // and feed the planned support heights to the swing planner.
    auto& planner = *terrainFootholdPlannerPtr_;
    planner.update(modeSchedule, targetTrajectories, computeFeetPositions(initState), initState, initTime, *mpcRobotModelPtr_);

    // Resolve the changing support height throughout the horizon, not only at the
    // command trajectory's three knots. Position and vertical velocity share the same smooth support path.
    const TargetTrajectories originalTargets = targetTrajectories;
    targetTrajectories.timeTrajectory.clear();
    targetTrajectories.stateTrajectory.clear();
    targetTrajectories.inputTrajectory.clear();
    for (scalar_t time = initTime; time < finalTime - 1e-6; time += 0.05) {
      targetTrajectories.timeTrajectory.push_back(time);
      targetTrajectories.stateTrajectory.push_back(originalTargets.getDesiredState(time));
      targetTrajectories.inputTrajectory.push_back(originalTargets.getDesiredInput(time));
    }
    targetTrajectories.timeTrajectory.push_back(finalTime);
    targetTrajectories.stateTrajectory.push_back(originalTargets.getDesiredState(finalTime));
    targetTrajectories.inputTrajectory.push_back(originalTargets.getDesiredInput(finalTime));

    // Keep the velocity-command trajectory on the flat approach. Near the
    // terrain, constrain its lead over the planned support and adapt its height
    // using committed swing footholds. This preserves persistent forward intent
    // instead of rebuilding the reference origin from measured feet every solve.
    const size_t velStart = mpcRobotModelPtr_->getBaseComVelocityStartindex();
    for (size_t i = 0; i < targetTrajectories.stateTrajectory.size(); ++i) {
      vector_t& targetState = targetTrajectories.stateTrajectory[i];
      const scalar_t knotTime = targetTrajectories.timeTrajectory[i];
      vector6_t basePose = mpcRobotModelPtr_->getBasePose(targetState);

      if (!planner.isNearStairs(basePose.head<2>())) {
        continue;
      }

      const vector3_t supportMidpoint =
          0.5 * (planner.getAnticipatedFootPosition(0, knotTime) + planner.getAnticipatedFootPosition(1, knotTime));
      const scalar_t yaw = basePose(3);
      const Eigen::Vector2d forwardDirection(std::cos(yaw), std::sin(yaw));
      const Eigen::Vector2d lateralDirection(-std::sin(yaw), std::cos(yaw));
      const Eigen::Vector2d horizontalOffset = basePose.head<2>() - supportMidpoint.head<2>();
      const scalar_t forwardLead = horizontalOffset.dot(forwardDirection);
      const scalar_t lateralLead = horizontalOffset.dot(lateralDirection);
      const scalar_t maxLead = std::max(planner.getMaxBaseLead(), 1e-6);

      const scalar_t velocityScale =
          std::clamp(2.0 * (maxLead - std::max(forwardLead, 0.0)) / maxLead, 0.0, 1.0);
      targetState(velStart) *= velocityScale;
      targetState(velStart + 1) *= velocityScale;

      basePose.head<2>() =
          supportMidpoint.head<2>() +
          std::clamp(forwardLead, -maxLead, maxLead) * forwardDirection +
          std::clamp(lateralLead, -maxLead, maxLead) * lateralDirection;
      basePose(2) =
          supportMidpoint(2) + std::min(basePose(2), planner.getMaxBaseHeightAboveSupport());
      targetState(velStart + 2) = planner.getAnticipatedSupportVelocity(knotTime)(2);
      basePose(4) = 0.0;
      basePose(5) = 0.0;
      mpcRobotModelPtr_->setBasePose(targetState, basePose);
    }

    feet_array_t<scalar_array_t> liftOffHeightSequence;
    feet_array_t<scalar_array_t> touchDownHeightSequence;
    planner.getHeightSequences(modeSchedule, liftOffHeightSequence, touchDownHeightSequence);
    swingTrajectoryPtr_->update(modeSchedule, liftOffHeightSequence, touchDownHeightSequence, swingHeights);
  } else {
    scalar_t terrainHeight = adaptToCurrentGroundHeight(targetTrajectories, initState, initMode);
    swingTrajectoryPtr_->update(modeSchedule, terrainHeight, swingHeights);
  }

  modeSchedule_ = modeSchedule;

  // Recompute the flat-ground capture-point footholds from the state this solve
  // starts at, so the swing target reflects the CURRENT lateral velocity rather
  // than a fixed nominal step.
  updateCaptureFootholds(initTime, initState);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

void SwitchedModelReferenceManager::updateCaptureFootholds(scalar_t initTime, const vector_t& initState) {
  captureFootholdValid_ = {false, false};
  if (!captureFootPlacement_.enabled) {
    return;
  }

  const auto& model = pinocchioInterface_.getModel();
  auto& data = pinocchioInterface_.getData();
  const vector_t q = mpcRobotModelPtr_->getGeneralizedCoordinates(initState);
  pinocchio::centerOfMass(model, data, q, false);

  const vector2_t com = data.com[0].head<2>();
  const scalar_t comHeight = std::max(data.com[0](2), 0.1);
  // Pelvis velocity approximates CoM velocity for the capture-point target.
  const vector2_t comVelocity = mpcRobotModelPtr_->getBaseComLinearVelocity(initState).head<2>();

  // omega = sqrt(g / z_com); the capture point is com + v/omega, i.e. where the
  // CoM would come to rest if the foot were placed there.
  const scalar_t omega = std::sqrt(9.81 / comHeight);
  const vector2_t capturePoint = com + comVelocity / omega;

  const vector6_t basePose = mpcRobotModelPtr_->getBasePose(initState);
  const scalar_t yaw = basePose(3);
  const vector2_t lateralDirection(-std::sin(yaw), std::cos(yaw));
  const vector2_t forwardDirection(std::cos(yaw), std::sin(yaw));

  // Use the schedule this solve was built on, the same one the event times below come from.
  const contact_flag_t contactFlags = modeNumber2StanceLeg(modeSchedule_.modeAtTime(initTime));
  const scalar_t halfWidth = 0.5 * captureFootPlacement_.stepWidth;
  const feet_array_t<vector3_t> feetPositions = computeFeetPositions(initState);

  // Time remaining until the next contact switch, i.e. until the swinging foot
  // lands. The DCM diverges from the stance foot as exp(omega * t), so a
  // correction built from the CURRENT capture point is systematically too small
  // by that factor - which shows up as a roll that still grows, only slower.
  // Propagating the DCM to touchdown is what makes the placement deadbeat.
  scalar_t timeToTouchdown = 0.0;
  {
    const auto& eventTimes = modeSchedule_.eventTimes;
    const auto it = std::upper_bound(eventTimes.begin(), eventTimes.end(), initTime);
    if (it != eventTimes.end()) {
      timeToTouchdown = std::max(*it - initTime, 0.0);
    }
    timeToTouchdown = std::min(timeToTouchdown, 1.0);  // guard against a stale schedule
  }

  // Step period, from the spacing of the scheduled contact switches. Used for the
  // feedforward step length below; a walking robot must land its foot AHEAD of
  // the body, which the capture point on its own never asks for.
  scalar_t stepPeriod = 0.0;
  {
    const auto& eventTimes = modeSchedule_.eventTimes;
    const auto it = std::upper_bound(eventTimes.begin(), eventTimes.end(), initTime);
    if (it != eventTimes.end() && std::next(it) != eventTimes.end()) {
      stepPeriod = std::clamp(*std::next(it) - *it, 0.0, 2.0);
    } else if (it != eventTimes.end() && it != eventTimes.begin()) {
      stepPeriod = std::clamp(*it - *std::prev(it), 0.0, 2.0);
    }
  }
  // Bound the DCM extrapolation separately: exp() of a long horizon turns a small
  // velocity error into an unreachable foothold, and the clamp below would then
  // saturate every step and destroy the feedback. Measured: a 0.4 s horizon is
  // worse than none at all.
  // Each airborne foot's own touchdown and the stance that follows it. The next
  // event is not this foot's touchdown once both feet can be airborne (running
  // flight), where it would aim the foot that just lifted off at the other
  // foot's landing time. Walking gaits give the same time as the next event.
  const auto& eventTimes = modeSchedule_.eventTimes;
  const auto& modeSequence = modeSchedule_.modeSequence;
  const auto firstEvent = static_cast<size_t>(
      std::upper_bound(eventTimes.begin(), eventTimes.end(), initTime) - eventTimes.begin());
  const auto footSchedule = [&](size_t foot, scalar_t& touchdown, scalar_t& stanceDuration) {
    for (size_t k = firstEvent; k < eventTimes.size() && k + 1 < modeSequence.size(); ++k) {
      if (modeNumber2StanceLeg(modeSequence[k + 1])[foot]) {
        touchdown = eventTimes[k];
        for (size_t j = k + 1; j < eventTimes.size() && j + 1 < modeSequence.size(); ++j) {
          if (!modeNumber2StanceLeg(modeSequence[j + 1])[foot]) {
            stanceDuration = eventTimes[j] - eventTimes[k];
            break;
          }
        }
        return true;
      }
    }
    return false;
  };

  for (size_t foot = 0; foot < captureFoothold_.size(); ++foot) {
    if (contactFlags[foot]) {
      continue;  // a foot already on the ground is not being placed
    }
    // Foot 0 is the left foot, so it sits on the positive lateral side.
    const scalar_t side = (foot == 0) ? 1.0 : -1.0;

    scalar_t footTouchdown = initTime + timeToTouchdown;
    scalar_t stanceDuration = 0.0;
    footSchedule(foot, footTouchdown, stanceDuration);
    const scalar_t footTimeToTouchdown = std::clamp(footTouchdown - initTime, 0.0, 1.0);
    const scalar_t dcmHorizon = std::min(footTimeToTouchdown, captureFootPlacement_.projectionHorizon);

    // The nominal foothold has to sit under where the BASE WILL BE when the foot
    // lands, not under where it is now. Using the current base position biases
    // every foothold backwards by v * timeToTouchdown - about 0.08 m at 0.2 m/s
    // over a 0.4 s swing - so the robot is perpetually stepping short and pitches
    // forward, and the error grows with speed. That matches the measured failure:
    // pitch creeping up through a walk and diverging sooner the faster it goes.
    const vector2_t baseAtTouchdown = basePose.head<2>() + comVelocity * footTimeToTouchdown;

    // Feedforward step length: the foot has to land ahead of the base so the body
    // can pass over it, which the capture point (a stopping placement) never
    // asks for. See CaptureFootPlacementSettings::stepLengthGain and stanceTimeGain.
    const vector2_t stepAhead =
        (captureFootPlacement_.stepLengthGain * stepPeriod + captureFootPlacement_.stanceTimeGain * stanceDuration) *
        comVelocity;

    // Centre the SUPPORT POLYGON under the base, not the ankle. The ankle is the
    // contact frame, but the sole extends 0.125 m ahead of it and only 0.055 m
    // behind, so putting the ankle under the base gives a forward-travelling CoM
    // 0.125 m of margin and a backward-travelling one 0.055 m. See
    // CaptureFootPlacementSettings::footCenterOffset - that asymmetry accounted
    // for every fall observed with foot placement enabled.
    const vector2_t nominal = baseAtTouchdown + stepAhead -
                              captureFootPlacement_.footCenterOffset * forwardDirection +
                              side * halfWidth * lateralDirection;

    // Propagate the capture point about the supporting foot to the moment this
    // foot lands: xi_td = p_stance + (xi_now - p_stance) * exp(omega * dt).
    const size_t stanceFoot = 1 - foot;
    const vector2_t stancePosition = feetPositions[stanceFoot].head<2>();
    const scalar_t growth = std::exp(omega * dcmHorizon);
    const vector2_t capturePointAtTouchdown =
        stancePosition + (capturePoint - stancePosition) * growth;

    // The correction is what turns a fixed step into feedback: how far the
    // predicted capture point sits from where the base will be when the foot
    // lands. Measured against the same predicted base as the nominal, so the two
    // do not double-count the forward travel.
    // Clamped so a transient cannot command a step the leg cannot reach.
    vector2_t correction =
        captureFootPlacement_.gain * (capturePointAtTouchdown - baseAtTouchdown);
    const scalar_t limit = captureFootPlacement_.maxAdjustment;
    correction.x() = std::clamp(correction.x(), -limit, limit);
    correction.y() = std::clamp(correction.y(), -limit, limit);

    captureFoothold_[foot].head<2>() = nominal + correction;
    captureFoothold_[foot](2) = 0.0;  // flat ground
    captureFootholdValid_[foot] = true;
    captureFootholdTouchdownTime_[foot] = footTouchdown;
  }
}

feet_array_t<vector3_t> SwitchedModelReferenceManager::computeFeetPositions(const vector_t& initState) {
  const auto& model = pinocchioInterface_.getModel();
  auto& data = pinocchioInterface_.getData();
  const vector_t q = mpcRobotModelPtr_->getGeneralizedCoordinates(initState);
  pinocchio::forwardKinematics(model, data, q);

  feet_array_t<vector3_t> feetPositions;
  const auto& contactNames = mpcRobotModelPtr_->modelSettings.contactNames;
  for (size_t foot = 0; foot < feetPositions.size(); ++foot) {
    const auto frameId = model.getFrameId(contactNames[foot]);
    feetPositions[foot] = pinocchio::updateFramePlacement(model, data, frameId).translation();
  }
  return feetPositions;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

bool SwitchedModelReferenceManager::getSwingFootholdReference(size_t foot,
                                                              scalar_t time,
                                                              vector3_t& positionReference,
                                                              scalar_t& trackingWeight) const {
  // Terrain-aware plans win: they encode real geometry, whereas the capture-point
  // foothold below assumes flat ground.
  const auto& stairPlanForFoothold = stairClimbingPlan_.get();
  if (!stairPlanForFoothold && !isTerrainWalkActive() && captureFootPlacement_.enabled &&
      foot < captureFootholdValid_.size() && captureFootholdValid_[foot] &&
      (!captureFootPlacement_.currentSwingOnly || time <= captureFootholdTouchdownTime_[foot])) {
    positionReference = captureFoothold_[foot];
    trackingWeight = captureFootPlacement_.trackingWeight;
    return true;
  }

  const auto& stairPlan = stairClimbingPlan_.get();
  if (stairPlan && stairPlan->getSwingFootReference(foot, time, positionReference)) {
    trackingWeight = stairPlan->getFootholdTrackingWeight();
    return true;
  }
  if (isTerrainWalkActive() && terrainFootholdPlannerPtr_->getSwingFootReference(foot, time, positionReference)) {
    // Reach-scaled tracking weight: a foothold that is far ahead (an up-step the
    // foot cannot reach yet) gets a WEAK weight so the swing foot emerges from
    // base motion and the base keeps advancing (strong tracking of a far
    // foothold pins the feet and stalls the robot at the riser). As the base
    // approaches and the swing reach shrinks, the weight ramps to full so the
    // foot commits precisely onto the tread.
    scalar_t weight = terrainFootholdPlannerPtr_->getFootholdTrackingWeight();
    for (const auto& fs : terrainFootholdPlannerPtr_->getFootsteps()[foot]) {
      if (time >= fs.liftOffTime && time <= fs.touchDownTime) {
        const scalar_t reach = (fs.touchDownPosition.head<2>() - fs.liftOffPosition.head<2>()).norm();
        constexpr scalar_t commitReach = 0.20;  // reach at/below which the foot can commit -> full weight
        constexpr scalar_t farReach = 0.34;      // reach at/above which the foot cannot reach -> min weight
        constexpr scalar_t minScale = 0.05;
        weight *= std::clamp((farReach - reach) / (farReach - commitReach), minScale, 1.0);
        break;
      }
    }
    trackingWeight = weight;
    return true;
  }
  return false;
}

}  // namespace ocs2::humanoid
