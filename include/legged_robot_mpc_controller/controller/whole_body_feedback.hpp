// Phase 2.B S1 — whole-body feedback layer (control.wholeBody).
//
// Replaces the contact-blind joint PD as the main feedback: every servo tick the measured
// state and the MPC policy at the current time are turned into desired generalized
// accelerations (plan + task PD on base / joints / swing feet), and the contact-consistent
// least-squares core (whole_body_feedback_core.hpp) returns the joint torques that realise
// them under the floating-base dynamics and the stance constraints. The hardware PD is
// kept as a small residual servo (residualKp / residualKd).
#pragma once

#include "legged_robot_mpc_controller/controller/whole_body_feedback_core.hpp"

#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include <array>
#include <string>

namespace legged_robot_mpc_controller::wbc
{

struct Settings
{
  bool enabled{false};
  //! Base task gains / weights, generalized-coordinate order [x y z yaw pitch roll]
  //! (Euler ZYX angles and their rates, as in the MPC state).
  Vector6 baseKp{Vector6::Zero()};
  Vector6 baseKd{Vector6::Zero()};
  Vector6 baseWeight{Vector6::Ones()};
  double jointKp{0.0};
  double jointKd{0.0};
  double jointWeight{1.0};
  Eigen::VectorXd jointWeights;  //!< per joint; empty = jointWeight for all
  //! Optional link orientation task (e.g. the torso): world-frame angular acceleration
  //! = plan + kp log(R_plan R_measᵀ) + kd (ω_plan − ω). Empty linkName = off.
  std::string orientationLink;
  Eigen::Vector3d orientationKp{Eigen::Vector3d::Zero()};
  Eigen::Vector3d orientationKd{Eigen::Vector3d::Zero()};
  double orientationWeight{0.0};
  Eigen::Vector3d swingKp{Eigen::Vector3d::Zero()};
  Eigen::Vector3d swingKd{Eigen::Vector3d::Zero()};
  double swingWeight{0.0};
  double wrenchWeight{1e-2};
  WrenchLimits limits;
};

struct Input
{
  Eigen::VectorXd qMeasured;  //!< generalized coordinates (nq == nv for this base joint)
  Eigen::VectorXd vMeasured;
  Eigen::VectorXd qPlan;      //!< policy state at the current time
  Eigen::VectorXd vPlan;
  Eigen::VectorXd accPlan;    //!< policy generalized accelerations (base rows then joints)
  std::array<Vector6, kContacts> wrenchPlan{};
  std::array<bool, kContacts> inContact{{false, false}};
};

struct Output
{
  bool valid{false};
  Eigen::VectorXd jointTorque;
  double dynamicsResidual{0.0};
  int wrenchClips{0};
};

class WholeBodyFeedback
{
public:
  WholeBodyFeedback(Settings settings, const std::array<std::string, kContacts>& contactFrames,
                    const ocs2::PinocchioInterface& pinocchioInterface);

  //! Assemble and solve this tick's problem. `pinocchioInterface` data is used as scratch.
  Output compute(ocs2::PinocchioInterface& pinocchioInterface, const Input& input);

  const Settings& settings() const { return settings_; }

private:
  Settings settings_;
  std::array<std::size_t, kContacts> frameIds_{};
  std::size_t orientationFrameId_{0};
  bool hasOrientationTask_{false};
  Problem problem_;  //!< reused allocation
};

}  // namespace legged_robot_mpc_controller::wbc
