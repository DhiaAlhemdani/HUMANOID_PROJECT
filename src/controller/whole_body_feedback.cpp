#include <pinocchio/fwd.hpp>  // must precede any other pinocchio / boost include

#include "legged_robot_mpc_controller/controller/whole_body_feedback.hpp"

#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/spatial/explog.hpp>

#include <cmath>
#include <limits>
#include <stdexcept>

namespace legged_robot_mpc_controller::wbc
{

WholeBodyFeedback::WholeBodyFeedback(Settings settings, const std::array<std::string, kContacts>& contactFrames,
                                     const ocs2::PinocchioInterface& pinocchioInterface)
: settings_(std::move(settings))
{
  const auto& model = pinocchioInterface.getModel();
  for (std::size_t c = 0; c < kContacts; ++c) {
    if (!model.existFrame(contactFrames[c])) {
      throw std::invalid_argument("[WholeBodyFeedback] contact frame '" + contactFrames[c] + "' is not in the model");
    }
    frameIds_[c] = model.getFrameId(contactFrames[c]);
  }
  if (model.nq != model.nv) {
    throw std::invalid_argument("[WholeBodyFeedback] expects nq == nv (translation + Euler ZYX base joint)");
  }
  if (!settings_.orientationLink.empty()) {
    if (!model.existFrame(settings_.orientationLink)) {
      throw std::invalid_argument("[WholeBodyFeedback] orientation task link '" + settings_.orientationLink +
                                  "' is not in the model");
    }
    orientationFrameId_ = model.getFrameId(settings_.orientationLink);
    hasOrientationTask_ = settings_.orientationWeight > 0.0;
  }
  if (settings_.jointWeights.size() != 0 && settings_.jointWeights.size() != model.nv - kBaseDof) {
    throw std::invalid_argument("[WholeBodyFeedback] jointWeights needs one entry per MPC joint (or none)");
  }
  problem_.tasks.resize(hasOrientationTask_ ? 1 : 0);
}

Output WholeBodyFeedback::compute(ocs2::PinocchioInterface& pinocchioInterface, const Input& in)
{
  Output out;
  const auto& model = pinocchioInterface.getModel();
  auto& data = pinocchioInterface.getData();
  const Eigen::Index nv = model.nv;
  const Eigen::Index nj = nv - kBaseDof;
  if (in.qMeasured.size() != nv || in.vMeasured.size() != nv || in.qPlan.size() != nv || in.vPlan.size() != nv ||
      in.accPlan.size() != nv) {
    return out;
  }

  // 1. Swing-foot targets from the plan's configuration.
  std::array<Eigen::Vector3d, kContacts> footPlan, footVelPlan;
  pinocchio::forwardKinematics(model, data, in.qPlan, in.vPlan);
  pinocchio::updateFramePlacements(model, data);
  for (std::size_t c = 0; c < kContacts; ++c) {
    footPlan[c] = data.oMf[frameIds_[c]].translation();
    footVelPlan[c] = pinocchio::getFrameVelocity(model, data, frameIds_[c], pinocchio::LOCAL_WORLD_ALIGNED).linear();
  }
  Eigen::Matrix3d orientationPlan = Eigen::Matrix3d::Identity();
  Eigen::Vector3d angularVelPlan = Eigen::Vector3d::Zero();
  if (hasOrientationTask_) {
    orientationPlan = data.oMf[orientationFrameId_].rotation();
    angularVelPlan = pinocchio::getFrameVelocity(model, data, orientationFrameId_, pinocchio::LOCAL_WORLD_ALIGNED).angular();
  }

  // 2. Measured kinematics and dynamics.
  const Eigen::VectorXd zeroAcc = Eigen::VectorXd::Zero(nv);
  pinocchio::forwardKinematics(model, data, in.qMeasured, in.vMeasured, zeroAcc);
  pinocchio::updateFramePlacements(model, data);
  pinocchio::computeJointJacobians(model, data);
  pinocchio::crba(model, data, in.qMeasured);
  pinocchio::nonLinearEffects(model, data, in.qMeasured, in.vMeasured);

  Problem& p = problem_;
  p.massMatrix = data.M;
  p.massMatrix.triangularView<Eigen::StrictlyLower>() = p.massMatrix.transpose();  // crba fills the upper triangle
  p.nonlinearEffects = data.nle;
  p.swingWeight = settings_.swingWeight;
  p.wrenchWeight = settings_.wrenchWeight;

  // 3. Desired accelerations: plan + task PD.
  p.desiredAcc = in.accPlan;
  p.accWeight.resize(nv);
  p.accWeight.head(kBaseDof) = settings_.baseWeight;
  if (settings_.jointWeights.size() == nj) {
    p.accWeight.tail(nj) = settings_.jointWeights;
  } else {
    p.accWeight.tail(nj).setConstant(settings_.jointWeight);
  }
  {
    Vector6 poseError = in.qPlan.head(kBaseDof) - in.qMeasured.head(kBaseDof);
    poseError(3) = std::remainder(poseError(3), 2.0 * M_PI);  // yaw
    const Vector6 twistError = in.vPlan.head(kBaseDof) - in.vMeasured.head(kBaseDof);
    p.desiredAcc.head(kBaseDof) +=
      settings_.baseKp.cwiseProduct(poseError) + settings_.baseKd.cwiseProduct(twistError);
    p.desiredAcc.tail(nj) += settings_.jointKp * (in.qPlan.tail(nj) - in.qMeasured.tail(nj)) +
                             settings_.jointKd * (in.vPlan.tail(nj) - in.vMeasured.tail(nj));
  }

  // 4. Touchdown search: a foot the plan has in contact but whose measured sole still descends is not
  //    on the ground; the layer keeps it a swing foot (Settings::searchWindow).
  std::array<Eigen::Vector3d, kContacts> foot, footVel;
  for (std::size_t c = 0; c < kContacts; ++c) {
    foot[c] = data.oMf[frameIds_[c]].translation();
    footVel[c] = pinocchio::getFrameVelocity(model, data, frameIds_[c], pinocchio::LOCAL_WORLD_ALIGNED).linear();
  }
  std::array<bool, kContacts> inContact = in.inContact;
  for (std::size_t c = 0; c < kContacts; ++c) {
    out.searching[c] = touchdownSearch(settings_.search, in.time, in.inContact[c], in.inContact[1 - c], footVel[c].z(),
                                       contactSince_[c], searching_[c]);
  }
  if (out.searching[0] && out.searching[1]) {
    // Never both: the foot whose switch came later is loaded where it is, the earlier one keeps searching.
    const std::size_t drop = contactSince_[1] >= contactSince_[0] ? 1 : 0;
    searching_[drop] = false;
    out.searching[drop] = false;
  }
  for (std::size_t c = 0; c < kContacts; ++c) {
    inContact[c] = in.inContact[c] && !out.searching[c];
  }

  // 5. Contact and swing tasks.
  for (std::size_t c = 0; c < kContacts; ++c) {
    ContactTask& contact = p.contacts[c];
    contact.jacobian.setZero(6, nv);
    pinocchio::getFrameJacobian(model, data, frameIds_[c], pinocchio::LOCAL_WORLD_ALIGNED, contact.jacobian);
    contact.driftAcc =
      pinocchio::getFrameClassicalAcceleration(model, data, frameIds_[c], pinocchio::LOCAL_WORLD_ALIGNED).toVector();
    contact.inContact = inContact[c];
    contact.wrenchPlan = inContact[c] ? in.wrenchPlan[c] : Vector6::Zero();

    SwingTask& swing = p.swings[c];
    swing.active = !inContact[c] && settings_.swingWeight > 0.0;
    if (!swing.active) {
      continue;
    }
    swing.jacobian = contact.jacobian.topRows(3);
    swing.driftAcc = contact.driftAcc.head(3);
    if (out.searching[c]) {
      // Searching: hold the plan's foothold horizontally, go down at the search velocity. A velocity
      // target, not a position below the ground to overshoot into - the swing's own touchdown velocity.
      Eigen::Vector3d positionError = footPlan[c] - foot[c];
      positionError.z() = 0.0;
      Eigen::Vector3d velocityDesired = Eigen::Vector3d::Zero();
      velocityDesired.z() = -settings_.search.descentVelocity;
      swing.desiredAcc = settings_.swingKp.cwiseProduct(positionError) + settings_.swingKd.cwiseProduct(velocityDesired - footVel[c]);
      continue;
    }
    const Eigen::Vector3d planAcc = swing.jacobian * in.accPlan + swing.driftAcc;
    swing.desiredAcc = planAcc + settings_.swingKp.cwiseProduct(footPlan[c] - foot[c]) +
                       settings_.swingKd.cwiseProduct(footVelPlan[c] - footVel[c]);
  }

  // 6. Link orientation task (world axes, as the LOCAL_WORLD_ALIGNED angular rows).
  if (hasOrientationTask_) {
    LinearTask& task = p.tasks.front();
    Matrix6X J = Matrix6X::Zero(6, nv);
    pinocchio::getFrameJacobian(model, data, orientationFrameId_, pinocchio::LOCAL_WORLD_ALIGNED, J);
    task.jacobian = J.bottomRows(3);
    task.driftAcc =
      pinocchio::getFrameClassicalAcceleration(model, data, orientationFrameId_, pinocchio::LOCAL_WORLD_ALIGNED).angular();
    const Eigen::Matrix3d orientation = data.oMf[orientationFrameId_].rotation();
    const Eigen::Vector3d angularVel =
      pinocchio::getFrameVelocity(model, data, orientationFrameId_, pinocchio::LOCAL_WORLD_ALIGNED).angular();
    const Eigen::Vector3d orientationError = pinocchio::log3(orientationPlan * orientation.transpose());
    const Eigen::Vector3d planAcc = task.jacobian * in.accPlan + task.driftAcc;
    task.desiredAcc = planAcc + settings_.orientationKp.cwiseProduct(orientationError) +
                      settings_.orientationKd.cwiseProduct(angularVelPlan - angularVel);
    task.weight = settings_.orientationWeight;
    task.active = true;
  }

  const Solution s = solve(p, settings_.limits);
  out.valid = s.valid;
  if (s.valid) {
    out.jointTorque = s.jointTorque;
    out.dynamicsResidual = s.dynamicsResidual;
    out.wrenchClips = s.wrenchClips;
  }
  return out;
}

}  // namespace legged_robot_mpc_controller::wbc
