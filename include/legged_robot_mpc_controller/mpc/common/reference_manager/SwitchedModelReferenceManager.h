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

#pragma once

#include <array>
#include <cmath>
#include <limits>

#include <ocs2_core/thread_support/BufferedValue.h>
#include <ocs2_core/thread_support/Synchronized.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include "legged_robot_mpc_controller/mpc/common/model/MpcRobotModelBase.h"
#include <atomic>

#include "legged_robot_mpc_controller/mpc/common/gait/ContactSync.h"
#include "legged_robot_mpc_controller/mpc/common/gait/GaitSchedule.h"
#include "legged_robot_mpc_controller/mpc/common/gait/MotionPhaseDefinition.h"
#include "legged_robot_mpc_controller/mpc/common/gait/SlopeFootReference.h"
#include "legged_robot_mpc_controller/mpc/common/gait/TerrainSpeedCap.h"
#include "legged_robot_mpc_controller/mpc/common/reference_manager/StairClimbingPlan.h"
#include "legged_robot_mpc_controller/mpc/common/reference_manager/TerrainFootholdPlanner.h"
#include "legged_robot_mpc_controller/mpc/common/swing_foot_planner/SwingTrajectoryPlanner.h"

namespace ocs2::humanoid {

/**
 * Manages the ModeSchedule and the TargetTrajectories for switched model.
 */
class SwitchedModelReferenceManager : public ReferenceManager {
 public:
  enum class BaseTrackingMode { RelativeTwist, AbsolutePose };

  SwitchedModelReferenceManager(std::shared_ptr<GaitSchedule> gaitSchedulePtr,
                                std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr,
                                const PinocchioInterface& pinocchioInterface,
                                const MpcRobotModelBase<scalar_t>& mpcRobotModel);

  ~SwitchedModelReferenceManager() override = default;

  /** Disable copy / move */
  SwitchedModelReferenceManager& operator=(const SwitchedModelReferenceManager&) = delete;
  SwitchedModelReferenceManager(const SwitchedModelReferenceManager&) = delete;
  SwitchedModelReferenceManager& operator=(SwitchedModelReferenceManager&&) = delete;
  SwitchedModelReferenceManager(SwitchedModelReferenceManager&&) = delete;

  /**
   * Capture-point foot placement for flat ground.
   *
   * On flat ground there is no foothold reference at all: the stair plan and the
   * terrain planner are both inactive, getSwingFootholdReference() returns false,
   * and the foot task-space cost has zero position weight. The landing xy is then
   * whatever the optimizer happens to produce. That is fine while the robot is
   * upright, but it leaves nothing to arrest a lateral fall - and lateral walking
   * dynamics are an inverted pendulum whose only real authority IS where the next
   * foot lands. Measured behaviour matches: step width stayed pinned near 0.18 m
   * while roll grew step over step until the robot went over.
   *
   * This closes that loop the standard way. The swing foot is aimed at the
   * capture point (CoM plus CoM velocity over omega), offset laterally by half
   * the nominal step width so the feet stay apart, with the correction clamped so
   * a transient cannot command an unreachable step.
   */
  struct CaptureFootPlacementSettings {
    bool enabled = false;
    scalar_t gain = 0.6;             //!< scales the capture-point correction; 0 reproduces the nominal foothold
    scalar_t stepWidth = 0.18;       //!< nominal lateral spacing between the feet [m]
    scalar_t maxAdjustment = 0.20;   //!< bound on the correction away from nominal [m]
    //! xy foothold tracking weight handed to the foot cost. The dominant knob,
    //! and it wants to be LARGE: on the vx ladder 30 -> 48.5 s, 100 -> 57.5,
    //! 400 -> 65.0, 1600 -> 78.6, 3200 -> 79.2 (saturating). Too weak and the
    //! optimizer simply ignores the foothold.
    scalar_t trackingWeight = 1600.0;
    //! How far ahead to propagate the capture point about the stance foot, in
    //! seconds, capped at the time remaining until touchdown. The DCM diverges as
    //! exp(omega*t), so in principle projecting to touchdown makes the placement
    //! deadbeat. In practice omega ~ 3.7 turns a 0.4 s projection into a 4.5x
    //! amplification that saturates maxAdjustment on every step and destroys the
    //! feedback it was meant to sharpen - measured worse than no projection.
    //! Default 0 is the instantaneous capture point, which measured best; raise
    //! it together with maxAdjustment if you want to explore the deadbeat end.
    scalar_t projectionHorizon = 0.0;
    //! Feedforward step length, as a fraction of (velocity x step period).
    //!
    //! The capture point alone is the DEADBEAT STOPPING placement: put the foot
    //! there and the CoM comes to rest. Steady walking needs the opposite - the
    //! foot must land ahead of the base by about v * T_step / 2 so the body can
    //! keep passing over it. With omega ~ 3.7 the capture term supplies only
    //! 0.27 * v against the ~0.43 * v that a 0.85 s step needs, leaving the robot
    //! a few centimetres short on every step and slowly pitching forward - which
    //! is worse the faster it walks.
    //!
    //! MEASURED NEUTRAL-TO-HARMFUL here, so it defaults to 0: the nominal already
    //! advances by v * timeToTouchdown (see baseAtTouchdown), which covers most of
    //! the same travel, and adding this term on top double-counts it. On the vx
    //! ladder: 0.0 -> 56.3 s, 0.5 -> 56.1 s, 0.8 -> 55.5 s, 1.2 -> 47.1 s. Kept as
    //! a knob because the right split between the two depends on the gait period.
    scalar_t stepLengthGain = 0.0;
    //! Distance from the contact (ankle) frame to the CENTRE of the foot's
    //! support polygon, along the heading direction [m].
    //!
    //! The contact frame is the ankle, but the sole is not centred on it. On the
    //! G1 it spans -0.055 m at the heel to +0.125 m at the toe, so the toe reach
    //! is 2.27x the heel reach. Placing the ankle under the predicted base
    //! therefore leaves 0.125 m of margin for a CoM travelling forward and only
    //! 0.055 m for one travelling backward.
    //!
    //! That asymmetry was the dominant failure mode: with foot placement enabled,
    //! forward walking at 0.05-0.30 m/s never fell across six runs, while every
    //! single fall (4 of 4) happened 1.6-3.8 s into the -0.10 m/s phase.
    //! Subtracting this offset centres the polygon under the base and equalises
    //! the margins at 0.09 m - a 64% gain backward for a reduction forward that
    //! the robot never came close to needing.
    //!
    //! Measured from the model geometry: (0.125 - 0.055) / 2 = 0.035.
    scalar_t footCenterOffset = 0.035;
    //! Raibert neutral point, as a fraction of (velocity x the landing foot's
    //! scheduled stance duration).
    //!
    //! Constant-speed running needs the foot to land as far ahead of the CoM as it
    //! leaves behind, i.e. about v * T_stance / 2 ahead of the base at touchdown;
    //! landing shorter nets propulsion every stance and the robot runs away.
    //! Unlike stepLengthGain this uses the landing foot's own stance time, so it is
    //! well defined for gaits with double support or flight between steps.
    scalar_t stanceTimeGain = 0.0;
    //! Apply the foothold only until this foot's scheduled touchdown. Otherwise the
    //! target also pulls the foot's later swings within the horizon back to it,
    //! which matters once the horizon spans several steps (running).
    bool currentSwingOnly = false;
  };

  /**
   * Use the measured stance-foot height as the swing planner's ground reference,
   * instead of assuming a flat floor at z = 0.
   *
   * adaptToCurrentGroundHeight() computes a ground-height estimate from the
   * stance feet and then, historically, discarded it with `terrainHeight = 0.0`.
   * That constant is what the swing trajectory is built around and what the
   * base-height target is offset by, so it hard-codes a flat floor at the world
   * origin - and it is the single thing that stops this stack walking on sloped
   * or stepped ground however well the state estimator tracks the terrain,
   * because the swing foot is still aimed at z = 0.
   *
   * Off by default: the shipped gains were all tuned against the flat assumption,
   * and on genuinely flat ground the estimate is strictly worse than the constant
   * it replaces. Enable it together with an InEKF height source that does not
   * itself assume a flat plane - "inekf" or "anchored". "kinematic" and "blend"
   * both pin the stance feet to groundZ, which would feed the assumption straight
   * back in and defeat the purpose.
   *
   * @param maxStepPerSolve bounds how far the reference may move in one solver
   *        iteration, so a mis-detected contact cannot demand an impossible swing.
   */
  void setUseTerrainHeightEstimate(bool enabled, scalar_t maxStepPerSolve = 0.01) {
    useTerrainHeightEstimate_ = enabled;
    maxTerrainHeightStep_ = maxStepPerSolve;
  }
  bool usesTerrainHeightEstimate() const { return useTerrainHeightEstimate_; }

  void setCaptureFootPlacementSettings(const CaptureFootPlacementSettings& settings) {
    captureFootPlacement_ = settings;
  }
  const CaptureFootPlacementSettings& getCaptureFootPlacementSettings() const { return captureFootPlacement_; }

  contact_flag_t getContactFlags(scalar_t time) const;

  bool isInStancePhase(scalar_t time) const { return (getContactFlags(time)[0] && getContactFlags(time)[1]); }

  bool isInContact(scalar_t time, size_t contactIndex) const { return getContactFlags(time)[contactIndex]; };

  /// Start time of the horizon being solved (set in modifyReferences, before the solve reads the costs);
  /// NaN before the first solve. Lets a cost or constraint treat the first node differently.
  scalar_t getSolveInitTime() const { return solveInitTime_; }
  bool isFirstNode(scalar_t time) const { return !(time > solveInitTime_ + 1e-6); }

  /**
   * Swing superimposed on the nominal arm posture while walking.
   *
   * Two separable pieces, kept deliberately apart because only the second one is
   * going to be replaced:
   *
   *  - The NOMINAL arm posture is NOT here. It is the arm block of
   *    `reference.defaultJointState`, which is the posture the arms hold when the
   *    swing evaluates to zero. That is the right home for it - it is the same
   *    posture the terminal cost and the stair planner track - and it is what
   *    decides whether the arms hang at the sides or stick out in front.
   *  - The SWING about that posture is configured here.
   *
   * WHAT THIS IS (phase 1): an open-loop sinusoid locked to the gait phase and
   * scaled by the commanded forward speed. It is a kinematic mimicry of a human
   * arm swing: it does not know what the trunk is actually doing and cannot react
   * to a disturbance. Its only stabilising effect is indirect, through the yaw
   * moment the counter-swinging arms apply against the one the swinging leg
   * generates.
   *
   */
  struct ArmSwingSettings {
    bool enabled = false;
    //! Shoulder-pitch amplitude per unit of commanded forward speed [rad/(m/s)].
    scalar_t shoulderPitchAmplitude = 0.15;
    //! Elbow amplitude per unit of commanded forward speed [rad/(m/s)].
    scalar_t elbowAmplitude = 0.15;
    //! Shifts the swing relative to the gait phase, in fractions of a cycle.
    scalar_t phaseOffset = 0.15;
    //! Hard bound on the offset away from the nominal posture [rad]. Sized from
    //! the collision-free envelope measured on the G1 model: at the shipped
    //! nominal (shoulder roll +-0.20, elbow 1.20) a shoulder-pitch excursion of
    //! +-0.6 rad clears the torso, pelvis and hips with no interpenetration and
scalar_t maxOffset = 0.6;
  };

  /**
   * Contact-synchronised touchdown (ContactSync.h): applied in modifyReferences to the schedule
   * each solve is built on, from the measured sole heights of the solve's initial state and the
   * current support-plane estimate. Off for the stair and terrain modes, whose footholds are not
   * on the flat support plane. Settings are read on the solver thread only.
   */
  void setContactSyncSettings(const ContactSyncSettings& settings) {
    validateContactSyncSettings(settings);
    contactSync_ = settings;
  }
  const ContactSyncSettings& getContactSyncSettings() const { return contactSync_; }
  //! Diagnostics: touchdowns advanced / held so far and the summed shifts [s] (any thread).
  size_t contactSyncAdvancedCount() const { return contactSyncAdvanced_.load(); }
  size_t contactSyncHeldCount() const { return contactSyncHeld_.load(); }
  scalar_t contactSyncAdvancedSeconds() const { return contactSyncAdvancedSeconds_.load(); }
  size_t contactSyncBlockedCount() const { return contactSyncBlocked_.load(); }
  size_t contactSyncDescendingCount() const { return contactSyncDescending_.load(); }
  size_t contactSyncProbedCount() const { return contactSyncProbed_.load(); }  //!< solves that held a still-descending foot (the probe)
  scalar_t contactSyncProbeDepth() const { return contactSyncProbeDepth_.load(); }  //!< [m] how far the probed foot's target is lowered now (0 = none)

  /**
   * Blind terrain speed cap (TerrainSpeedCap.h): the sole-height difference in double support marks
   * uneven ground and caps the commanded planar speed for a while. Updated on the solver thread in
   * modifyReferences() for the flat-ground modes; the motion manager reads terrainSpeedCapValue()
   * before it conditions the velocity command. Settings are read on the solver thread only.
   */
  void setTerrainSpeedCapSettings(const TerrainSpeedCapSettings& settings) { terrainSpeedCap_.setSettings(settings); }
  const TerrainSpeedCapSettings& getTerrainSpeedCapSettings() const { return terrainSpeedCap_.getSettings(); }
  //! [m/s] cap in force for the planar velocity command, +inf when none (any thread).
  scalar_t terrainSpeedCapValue() const { return terrainSpeedCapValue_.load(); }
  //! Diagnostics (any thread): cap active, double supports that tripped it, the last step [m].
  bool terrainSpeedCapActive() const { return std::isfinite(terrainSpeedCapValue_.load()); }
  size_t terrainStepCount() const { return terrainStepCount_.load(); }
  scalar_t terrainLastStep() const { return terrainLastStep_.load(); }
  scalar_t terrainSlope() const { return terrainSlope_.load(); }  //!< [-] the plane's rise along the heading (telemetry; 0 = none / flat)
  bool slopePlaneRead() const { return slopeGradientX_.load() != 0.0 || slopeGradientY_.load() != 0.0; }  //!< a plane stands and the references follow it

  /**
   * Foot references follow the incline the cap reads (SlopeFootReference.h, run 25 C): per-foot support
   * heights on the plane for the swing planner and the plane's normal for the foot orientation terms
   * (getContactNormal). Needs the ground height estimate (the heights are measured) and the cap (the
   * slope). Solver thread only; off = the flat-ground references.
   */
  void setSlopeFootReference(bool enabled) { slopeFootReference_ = enabled; }
  bool slopeFootReferenceEnabled() const { return slopeFootReference_; }
  //! [m] the plane height the last solve planned for the next touchdown (diagnostics; NaN while flat).
  scalar_t slopeNextTouchDownHeight() const { return slopeNextTouchDownHeight_.load(); }

  void setArmSwingSettings(const ArmSwingSettings& settings) { armSwing_ = settings; }
  const ArmSwingSettings& getArmSwingSettings() const { return armSwing_; }

  void setArmSwingReferenceActive(bool armSwingReferenceActive) { armSwing_.enabled = armSwingReferenceActive; }

  /**
   * Human gait template: the whole-body kinematics of normal human walking as the
   * reference, instead of one fixed posture.
   *
   * The nominal reference is a single joint posture (`reference.defaultJointState`)
   * plus a pelvis velocity; nothing in it says what a walking human does with the
   * hips, knees, trunk and arms, so the optimizer fills those in by effort alone
   * (bent knees throughout, arms used as a flywheel, no trunk motion). This
   * template replaces the posture with normative human gait curves (Winter-type
   * sagittal hip / knee / ankle angles over the gait cycle, pelvis obliquity and
   * vertical excursion, trunk counter-rotation, arm swing) evaluated at each leg's
   * own gait-cycle phase. The phase is time-warped onto the planned mode schedule
   * (heel strike -> other toe-off -> other heel strike -> toe-off -> heel strike map
   * to 0 / 0.10 / 0.50 / 0.60 / 1.0 of the human cycle), so the curves follow the
   * actual step and double-support timing of whatever gait is running.
   *
   * Amplitudes scale with the commanded step: the leg curves by the step length
   * relative to the leg (humans: 0.78 leg lengths per step at 1.3 m/s), the arms
   * by speed. Standing (no planned step) evaluates to the static stance posture.
   * All of it is a soft reference: the joint-position / base-pose weights decide
   * how closely it is followed, and the physics (contacts, torques, balance) stay
   * with the MPC. Backward / lateral / turning commands fall back to the stance
   * posture (the curves are for forward walking).
   */
  struct HumanGaitSettings {
    bool enabled = false;
    bool legs = true;   //!< hip pitch / knee / ankle pitch curves (absolute angles, replace the nominal)
    bool arms = true;   //!< shoulder pitch / elbow swing about the nominal (needs the arm-swing joints)
    bool waist = true;  //!< waist yaw counter-rotation about the nominal
    bool base = true;   //!< pelvis height oscillation, roll (obliquity), pitch about the nominal
    //! Joint names: [l_hip_pitch, l_knee, l_ankle_pitch, r_hip_pitch, r_knee, r_ankle_pitch, waist_yaw].
    std::vector<std::string> jointNames;
    scalar_t legLength = 0.65;        //!< hip-to-ankle length [m]; with the step length it sets the hip range
    scalar_t humanStepRatio = 0.78;   //!< human step length / leg length at referenceSpeed
    scalar_t referenceSpeed = 1.3;    //!< speed at which the curves and arm amplitudes are defined [m/s]
    scalar_t maxScale = 1.2;          //!< cap on the amplitude scales
    //! Added knee flexion everywhere (hip and ankle take half, keeping the trunk upright and the foot
    //! flat) [rad]. Humans stand on a nearly straight knee; the whole-body ID wants a margin from the
    //! singularity. The standing posture is then hip -kneeOffset/2, knee kneeOffset, ankle -kneeOffset/2.
    scalar_t kneeOffset = 0.3;
    //! Flat-foot closure in stance: the ankle reference is -hip - knee (sole level under a level pelvis)
    //! instead of the human ankle curve, blended in over the last 10 % of swing and out over the first
    //! 10 % of swing. The human curve has the heel up for the last quarter of stance and the heel only
    //! on the ground at strike; a flat-foot robot cannot do either, and the raw curve then asks the
    //! stance foot to pitch by 10-40 deg against the contact constraint.
    bool stanceAnkleClosure = true;
    //! With the closure, the stance knee is capped so that the closed ankle stays within this
    //! dorsiflexion [rad]: the human knee flexes to ~45 deg before toe-off with the heel up; flat-footed
    //! that would need 45 deg of dorsiflexion and a 6 cm pelvis drop. The trailing leg stays straighter.
    scalar_t maxStanceDorsiflexion = 0.44;
    //! Pelvis height reference from the stance legs' template (planar FK with the foot flat), as an
    //! offset from the standing posture; averaged over both legs in double support. Replaces
    //! heightOscillation when `base` is on. A fixed height cannot be met by human leg angles once the
    //! legs are split (the pelvis is 2-5 cm lower at heel strike than at mid stance), so a fixed
    //! reference and the leg curves pull against each other.
    bool heightFromLegs = true;
    //! Swing-knee flexion is reduced so that the template's own foot clearance (swing-leg FK against the
    //! stance-leg pelvis height) peaks at this value [m]; <= 0 disables. The swing-foot height is a hard
    //! constraint on the planner's path (5 cm); the human swing knee implies 5 cm at 0.3 m/s but 7.5 cm at
    //! 0.5 and 10 cm at 0.8 m/s, and raising the planner's path to match made the robot fall.
    scalar_t swingClearance = 0.05;
    scalar_t thighLength = 0.3366;  //!< hip pitch axis to knee axis [m] (G1)
    scalar_t shankLength = 0.3;     //!< knee axis to ankle pitch axis [m] (G1)
    scalar_t shoulderAmplitude = 0.35;  //!< shoulder pitch swing at referenceSpeed [rad]
    scalar_t elbowAmplitude = 0.15;     //!< elbow swing at referenceSpeed [rad]
    scalar_t elbowMean = 0.0;           //!< constant elbow offset from the nominal while enabled [rad]
    scalar_t armMaxOffset = 0.6;        //!< clamp on the arm offsets [rad] (collision envelope)
    scalar_t waistYawAmplitude = 0.08;  //!< trunk counter-rotation at full scale [rad]
    scalar_t pelvisRollAmplitude = 0.05;   //!< pelvis obliquity at full scale [rad], swing side down
    scalar_t pelvisPitchAmplitude = 0.0;   //!< pelvis pitch oscillation at full scale [rad], twice per cycle
    scalar_t heightOscillation = 0.015;    //!< pelvis vertical excursion at full scale [m] when !heightFromLegs
  };

  /// Installs the template; resolves the joint names against the MPC joint list (throws on a missing name).
  void setHumanGaitSettings(const HumanGaitSettings& settings);
  const HumanGaitSettings& getHumanGaitSettings() const { return humanGait_; }

  struct StridePhase {
    bool valid = false;     //!< false: the leg has no planned step around `time` (standing)
    scalar_t phase = 0.0;   //!< human gait-cycle phase in [0, 1): 0 heel strike, 0.5 other heel strike, 0.6 toe-off
    scalar_t stepTime = 0.0;  //!< planned step time (heel strike to the other heel strike) [s]
    //! Amplitude factor in [0, 1]: ramps in over the first swing out of standing and out after the last
    //! heel strike (phase held), so the template does not step at walk start / stop.
    scalar_t fade = 1.0;
  };

  /// Human gait-cycle phase of one leg (0 = LF, 1 = RF), time-warped onto the planned mode schedule.
  StridePhase getLegStridePhase(size_t leg, scalar_t time) const;

  /// Superimposes the human gait template on the nominal state (joints and base pose), in place.
  void addHumanGaitTemplate(vector_t& xNominal, scalar_t time) const;

  /**
   * Selects how the base tracking cost interprets the target trajectory.
   * RelativeTwist ignores the unobservable world x/y/yaw pose error and tracks
   * horizontal motion in the target heading frame. AbsolutePose retains full
   * world-frame pose tracking for explicit base-pose commands.
   */
  void setBaseTrackingMode(BaseTrackingMode mode) { baseTrackingMode_.setBuffer(mode); }
  BaseTrackingMode getBaseTrackingMode() const { return baseTrackingMode_.get(); }
  bool usesRelativeBaseTwistTracking() const {
    return getBaseTrackingMode() == BaseTrackingMode::RelativeTwist;
  }

  /**
   * External joint-target channel (command_type "joint", mirroring the
   * mpc_controllers reference-manager targets). When a non-empty trajectory is
   * buffered, JointTrackingCost tracks it instead of the internal arm-swing
   * reference; an empty trajectory reverts to the internal reference. States are
   * sized and ordered like the tracked-joint subset. Buffered: setters are
   * thread-safe, consumers read the value swapped in before the solver run.
   */
  void setExternalJointTargets(TargetTrajectories jointTargets) { externalJointTargets_.setBuffer(std::move(jointTargets)); }
  const TargetTrajectories& getExternalJointTargets() const { return externalJointTargets_.get(); }
  bool hasExternalJointTargets() const { return !externalJointTargets_.get().empty(); }

  /**
   * Fixed-sequence stair climbing plan channel. When a plan is set, the swing
   * trajectory planner is fed the plan's per-phase lift-off / touch-down height
   * sequences (instead of flat ground) and the swing-foot cost tracks the
   * planned footholds. Buffered: the setter is thread-safe, consumers read the
   * value swapped in before the solver run. Set nullptr to clear.
   */
  void setStairClimbingPlan(std::shared_ptr<const StairClimbingPlan> plan) { stairClimbingPlan_.setBuffer(std::move(plan)); }
  const std::shared_ptr<const StairClimbingPlan>& getStairClimbingPlan() const { return stairClimbingPlan_.get(); }

  /**
   * Online terrain-aware foothold planning channel (terrain_walk mode).
   * The planner is installed once at configure time; the activation flag is
   * buffered (thread-safe). When active, modifyReferences re-plans the
   * footsteps each cycle from the fetched mode schedule and the desired base
   * trajectory, feeds the resulting height sequences to the swing planner, and
   * terrain-adapts the base height reference.
   */
  void setTerrainFootholdPlanner(std::shared_ptr<TerrainFootholdPlanner> planner) {
    terrainFootholdPlannerPtr_ = std::move(planner);
  }
  void setTerrainWalkActive(bool active) { terrainWalkActive_.setBuffer(active); }
  bool isTerrainWalkActive() const { return terrainWalkActive_.get() && terrainFootholdPlannerPtr_ != nullptr; }
  vector3_t getContactNormal(size_t foot, scalar_t time) const {
    if (isTerrainWalkActive()) {
      return terrainFootholdPlannerPtr_->getContactNormal(foot, time);
    }
    // The incline the blind cap reads, when the foot references follow it (zero gradient otherwise).
    const scalar_t gx = slopeGradientX_.load();
    const scalar_t gy = slopeGradientY_.load();
    if (gx != 0.0 || gy != 0.0) {
      return vector3_t(-gx, -gy, 1.0).normalized();
    }
    return vector3_t(0.0, 0.0, 1.0);
  }

  /**
   * Unified swing-foot foothold reference used by the task-space foot cost:
   * dispatches to the fixed stair climbing plan or the online terrain planner.
   * @return true when a foothold reference is active for `foot` at `time`.
   */
  bool getSwingFootholdReference(size_t foot, scalar_t time, vector3_t& positionReference, scalar_t& trackingWeight) const;

  const std::shared_ptr<GaitSchedule>& getGaitSchedule() const { return gaitSchedulePtr_; }

  const std::shared_ptr<SwingTrajectoryPlanner>& getSwingTrajectoryPlanner() const { return swingTrajectoryPtr_; }

  scalar_t getPhaseVariable(scalar_t time) const;

  vector_t getDesiredState(const TargetTrajectories& targetTrajectories, const vector_t& state, scalar_t time) const;

  /**
   * Superimpose the arm swing on the nominal arm posture, in place.
   *
   * @param desiredJointAngles [in/out] the nominal joint reference; only the four
   *        arm entries are modified, each clamped to +-ArmSwingSettings::maxOffset.
   */
  void addArmSwingOffsets(vector_t& desiredJointAngles,
                          const vector_t& xNominal,
                          const vector_t& state,
                          scalar_t time) const;

 protected:
  virtual void modifyReferences(scalar_t initTime,
                                scalar_t finalTime,
                                const vector_t& initState,
                                size_t initMode,
                                TargetTrajectories& targetTrajectories,
                                ModeSchedule& modeSchedule) override;

  // Adjusts the height of the target trajectories to current terrain height and returns that height.
  scalar_t adaptToCurrentGroundHeight(TargetTrajectories& targetTrajectories, const vector_t& initState, size_t initMode);
  scalar_t previousGroundHeightEstimate_{0.0};

  PinocchioInterface pinocchioInterface_;
  const MpcRobotModelBase<scalar_t>* mpcRobotModelPtr_;
  ModeSchedule modeSchedule_;

  ArmSwingSettings armSwing_;
  HumanGaitSettings humanGait_;
  std::array<size_t, 7> humanGaitJointIndices_{};
  static constexpr size_t kSwingKneeTableSize = 65;
  std::vector<scalar_t> swingKneeTable_;  //!< swing-knee factor over legScale in [0, maxScale]

  BufferedValue<TargetTrajectories> externalJointTargets_{TargetTrajectories()};
  BufferedValue<std::shared_ptr<const StairClimbingPlan>> stairClimbingPlan_{nullptr};
  BufferedValue<BaseTrackingMode> baseTrackingMode_{BaseTrackingMode::RelativeTwist};

  // Terrain-aware walking: planner mutated only in modifyReferences (solver
  // thread, pre-solve); read-only during the solve, same pattern as the swing planner.
  std::shared_ptr<TerrainFootholdPlanner> terrainFootholdPlannerPtr_;
  BufferedValue<bool> terrainWalkActive_{false};

  /// Measured feet (contact frame) world positions from FK of the current state.
  feet_array_t<vector3_t> computeFeetPositions(const vector_t& initState);
  //! Contact-frame vertical velocities [m/s] from the state's generalized velocities (base twist + joint rates).
  feet_array_t<scalar_t> computeFeetVerticalVelocities(const vector_t& initState);

  /// Recompute the capture-point footholds from the current state. Solver thread
  /// only, called from modifyReferences() before the solve reads them.
  void updateCaptureFootholds(scalar_t initTime, const vector_t& initState);

  bool useTerrainHeightEstimate_ = false;
  scalar_t maxTerrainHeightStep_ = 0.01;

  scalar_t solveInitTime_ = std::numeric_limits<scalar_t>::quiet_NaN();

  CaptureFootPlacementSettings captureFootPlacement_;
  feet_array_t<vector3_t> captureFoothold_{vector3_t::Zero(), vector3_t::Zero()};
  feet_array_t<bool> captureFootholdValid_{false, false};
  feet_array_t<scalar_t> captureFootholdTouchdownTime_{0.0, 0.0};

  /// Contact-synchronised touchdown: solver thread only (modifyReferences).
  void applyContactSync(scalar_t initTime, const vector_t& initState, ModeSchedule& modeSchedule);
  ContactSyncSettings contactSync_;
  scalar_t contactSyncPreviousTime_ = std::numeric_limits<scalar_t>::quiet_NaN();  //!< [s] the previous solve's initTime: the solve period a held event must clear
  feet_array_t<ContactSyncHold> contactSyncHold_{};  //!< per-foot held touchdown (ContactSync.h), kept between solves
  std::atomic<size_t> contactSyncAdvanced_{0};
  std::atomic<size_t> contactSyncHeld_{0};
  std::atomic<scalar_t> contactSyncAdvancedSeconds_{0.0};
  std::atomic<size_t> contactSyncBlocked_{0};     //!< landings accepted by the blockedHeight rule
  std::atomic<size_t> contactSyncDescending_{0};  //!< near-plane soles rejected by the descentVelocity rule
  std::atomic<size_t> contactSyncProbed_{0};      //!< solves whose hold was the probe's
  std::atomic<scalar_t> contactSyncProbeDepth_{0.0};
  /// Probe (ContactSync.h): per foot, how far below its planned touchdown height the held foot's target is
  /// taken this solve (probeDescent x the hold so far; 0 when the foot is not held). Solver thread only.
  feet_array_t<scalar_t> probeOffset_{0.0, 0.0};
  /// Lowers the probed feet's touchdown targets for their current swing in a per-foot height sequence.
  void applyProbeOffsets(scalar_t initTime, const ModeSchedule& modeSchedule, feet_array_t<scalar_array_t>& touchDownHeights) const;

  /// Terrain speed cap: solver thread only (modifyReferences); the atomics are its published state.
  void updateTerrainSpeedCap(scalar_t initTime, const vector_t& initState, const ModeSchedule& modeSchedule);
  /// Swing planner update with per-foot heights on the incline the cap reads (solver thread only).
  void updateSwingOnSlope(scalar_t initTime, const vector_t& initState, const ModeSchedule& modeSchedule,
                          const scalar_array_t& swingHeights);
  TerrainSpeedCap terrainSpeedCap_;
  std::atomic<scalar_t> terrainSpeedCapValue_{std::numeric_limits<scalar_t>::infinity()};
  std::atomic<size_t> terrainStepCount_{0};
  std::atomic<scalar_t> terrainLastStep_{0.0};
  std::atomic<scalar_t> terrainSlope_{0.0};

  /// Incline foot references: solver thread only (modifyReferences); the gradient atomics feed getContactNormal.
  bool slopeFootReference_ = false;
  std::atomic<scalar_t> slopeGradientX_{0.0};  //!< the plane's gradient, world xy (TerrainSpeedCap::gradient()); zero = flat references
  std::atomic<scalar_t> slopeGradientY_{0.0};
  std::atomic<scalar_t> slopeNextTouchDownHeight_{std::numeric_limits<scalar_t>::quiet_NaN()};

  std::shared_ptr<GaitSchedule> gaitSchedulePtr_;
  std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr_;
};

}  // namespace ocs2::humanoid
