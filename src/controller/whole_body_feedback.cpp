#include <pinocchio/fwd.hpp>  // must precede any other pinocchio / boost include

#include "legged_robot_mpc_controller/controller/whole_body_feedback.hpp"

#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>

#include <cmath>
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
  p.accWeight.tail(nj).setConstant(settings_.jointWeight);
  {
    Vector6 poseError = in.qPlan.head(kBaseDof) - in.qMeasured.head(kBaseDof);
    poseError(3) = std::remainder(poseError(3), 2.0 * M_PI);  // yaw
    const Vector6 twistError = in.vPlan.head(kBaseDof) - in.vMeasured.head(kBaseDof);
    p.desiredAcc.head(kBaseDof) +=
      settings_.baseKp.cwiseProduct(poseError) + settings_.baseKd.cwiseProduct(twistError);
    p.desiredAcc.tail(nj) += settings_.jointKp * (in.qPlan.tail(nj) - in.qMeasured.tail(nj)) +
                             settings_.jointKd * (in.vPlan.tail(nj) - in.vMeasured.tail(nj));
  }

  // 4. Contact and swing tasks.
  for (std::size_t c = 0; c < kContacts; ++c) {
    ContactTask& contact = p.contacts[c];
    contact.jacobian.setZero(6, nv);
    pinocchio::getFrameJacobian(model, data, frameIds_[c], pinocchio::LOCAL_WORLD_ALIGNED, contact.jacobian);
    contact.driftAcc =
      pinocchio::getFrameClassicalAcceleration(model, data, frameIds_[c], pinocchio::LOCAL_WORLD_ALIGNED).toVector();
    contact.inContact = in.inContact[c];
    contact.wrenchPlan = in.inContact[c] ? in.wrenchPlan[c] : Vector6::Zero();

    SwingTask& swing = p.swings[c];
    swing.active = !in.inContact[c] && settings_.swingWeight > 0.0;
    if (swing.active) {
      swing.jacobian = contact.jacobian.topRows(3);
      swing.driftAcc = contact.driftAcc.head(3);
      const Eigen::Vector3d foot = data.oMf[frameIds_[c]].translation();
      const Eigen::Vector3d footVel =
        pinocchio::getFrameVelocity(model, data, frameIds_[c], pinocchio::LOCAL_WORLD_ALIGNED).linear();
      const Eigen::Vector3d planAcc = swing.jacobian * in.accPlan + swing.driftAcc;
      swing.desiredAcc = planAcc + settings_.swingKp.cwiseProduct(footPlan[c] - foot) +
                         settings_.swingKd.cwiseProduct(footVelPlan[c] - footVel);
    }
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
