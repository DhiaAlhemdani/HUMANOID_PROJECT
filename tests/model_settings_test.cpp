#include <pinocchio/fwd.hpp>

#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>

#include <ocs2_core/PreComputation.h>

#include "legged_robot_mpc_controller/mpc/common/model/DynamicsHelperFunctions.h"
#include "legged_robot_mpc_controller/mpc/common/model/createPinocchioModel.h"
#include "legged_robot_mpc_controller/mpc/wb_id/dynamics/WbIdDynamics.h"
#include "legged_robot_mpc_controller/mpc/wb_id/model/WbIdMpcRobotModel.h"
#include "legged_robot_mpc_controller/mpc/wb_id/model/WbIdPinocchioMapping.h"

using namespace ocs2;
using namespace ocs2::humanoid;

namespace
{

void require(bool condition, const std::string & message)
{
  if (!condition) {throw std::runtime_error(message);}
}

template<typename Function>
void requireInvalid(Function function)
{
  try {
    function();
  } catch (const std::invalid_argument &) {
    return;
  }
  throw std::runtime_error("invalid fixed-joint configuration was accepted");
}

void checkRobot(const std::filesystem::path & root, const std::string & name, size_t fullJointCount)
{
  ModelSettings::Params params;
  params.robotName = name;
  params.recompileLibrariesCppAd = false;
  params.verboseCppAd = false;
  std::string urdfName;
  if (name == "upkie") {
    urdfName = "upkie.urdf";
    params.contactNames6DoF = {"left_tire_center", "right_tire_center"};
    params.contactParentJointNames = {"left_wheel", "right_wheel"};
  } else {
    urdfName = name == "g1" ? "g1_29dof.urdf" : "h2_31dof.urdf";
    params.fixedJointNames = {"left_wrist_roll_joint", "left_wrist_pitch_joint",
      "left_wrist_yaw_joint",
      "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint"};
    params.contactNames6DoF = {"foot_l_contact", "foot_r_contact"};
    params.contactParentJointNames = name == "g1" ?
      std::vector<std::string>{"left_ankle_roll_joint", "right_ankle_roll_joint"} :
    std::vector<std::string>{"left_ankle_pitch_joint", "right_ankle_pitch_joint"};
  }
  const std::string urdf = (root / name / urdfName).string();
  const ModelSettings settings(params, urdf, "model_settings_test_");
  require(settings.full_joint_dim == fullJointCount, name + ": unexpected full joint count");
  require(settings.mpc_joint_dim == fullJointCount - params.fixedJointNames.size(),
          name + ": unexpected active joint count");
  require(settings.mpcModelToFullJointsIndices.size() == settings.mpc_joint_dim,
          name + ": active-to-full joint index vector has the wrong size");
  for (size_t i = 0; i < settings.mpc_joint_dim; ++i) {
    const size_t fullIndex = settings.mpcModelToFullJointsIndices.at(i);
    require(fullIndex < settings.full_joint_dim, name + ": joint index is out of bounds");
    require(settings.fullJointNames.at(fullIndex) == settings.mpcModelJointNames.at(i),
            name + ": active joint maps to the wrong full joint");
  }

  const WbIdMpcRobotModel<scalar_t> robot(settings);
  const vector_t full = vector_t::LinSpaced(settings.full_joint_dim, -0.5, 0.5);
  const vector_t active = robot.getMpcModelJointAngles(full);
  const vector_t defaults = vector_t::Constant(settings.full_joint_dim, 0.75);
  const vector_t restored = robot.getFullModelJointAngles(active, defaults);
  for (size_t i = 0; i < settings.full_joint_dim; ++i) {
    const bool isActive = settings.jointIndexMap.count(settings.fullJointNames.at(i)) != 0;
    require(restored(i) == (isActive ? full(i) : defaults(i)),
        name + ": joint-angle mapping changed a value");
  }
  require(robot.getMpcModelJointAngles(restored).isApprox(active),
      name + ": joint-angle round trip failed");

  const auto interface = createCustomPinocchioInterface(urdf, settings);
  const auto contacts = getContactFrameIndices(interface, robot);
  require(contacts.size() == N_CONTACTS, name + ": contact index vector has the wrong size");
  for (size_t i = 0; i < N_CONTACTS; ++i) {
    require(contacts.at(i) == interface.getModel().getFrameId(settings.contactNames.at(i)),
            name + ": contact index does not match its frame");
  }

  auto invalid = params;
  invalid.fixedJointNames.push_back("not_a_robot_joint");
  requireInvalid([&] {ModelSettings rejected(invalid, urdf, "invalid_");});
  invalid.fixedJointNames = {settings.fullJointNames.front(), settings.fullJointNames.front()};
  requireInvalid([&] {ModelSettings rejected(invalid, urdf, "invalid_");});
  invalid.fixedJointNames = settings.fullJointNames;
  requireInvalid([&] {ModelSettings rejected(invalid, urdf, "invalid_");});
  std::cout << name <<
    ": joint mapping, contact indices, and invalid configuration checks passed\n";
}

// Wrench-as-state layout: x = [q, v, Ws], u = [W, a, tau, dW], dWs/dt = dW, W == Ws.
void checkWrenchStateLayout(const std::filesystem::path & root)
{
  ModelSettings::Params params;
  params.robotName = "g1";
  params.recompileLibrariesCppAd = false;
  params.verboseCppAd = false;
  params.fixedJointNames = {"left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
    "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint"};
  params.contactNames6DoF = {"foot_l_contact", "foot_r_contact"};
  params.contactParentJointNames = {"left_ankle_roll_joint", "right_ankle_roll_joint"};
  const std::string urdf = (root / "g1" / "g1_29dof.urdf").string();

  const ModelSettings plain(params, urdf, "layout_test_");
  params.wrenchState = true;
  const ModelSettings withWrench(params, urdf, "layout_test_");
  require(withWrench.modelFolderCppAd != plain.modelFolderCppAd,
          "wrench-state layout must generate its CppAD code into a folder of its own");

  const WbIdMpcRobotModel<scalar_t> base(plain);
  const WbIdMpcRobotModel<scalar_t> robot(withWrench);
  const size_t nv = robot.getGenCoordinatesDim();
  const size_t nj = robot.getJointDim();
  require(!base.hasWrenchState() && robot.hasWrenchState(), "hasWrenchState does not follow the setting");
  require(base.getStateDim() == 2 * nv && base.getInputDim() == 12 + nv + nj, "plain layout dimensions changed");
  require(robot.getStateDim() == 2 * nv + 12 && robot.getInputDim() == 12 + nv + nj + 12,
          "wrench-state layout dimensions are wrong");
  require(robot.getWrenchStateStartIndex(0) == 2 * nv && robot.getWrenchStateStartIndex(1) == 2 * nv + 6,
          "wrench states must follow [q, v]");
  require(robot.getWrenchRateStartIndex(0) == 12 + nv + nj && robot.getWrenchRateStartIndex(1) == 12 + nv + nj + 6,
          "wrench rates must follow the joint torques");
  require(robot.getContactWrenchStartIndices(1) == 6 && robot.getGeneralizedAccelerationStartIndex() == 12 &&
          robot.getJointTorqueStartIndex() == 12 + nv, "the leading input blocks must not move");

  // Accessors address their own block only, regardless of the trailing wrench states.
  vector_t state = vector_t::LinSpaced(static_cast<Eigen::Index>(robot.getStateDim()), 1.0, 100.0);
  vector_t input = vector_t::LinSpaced(static_cast<Eigen::Index>(robot.getInputDim()), -100.0, -1.0);
  const vector_t q = state.head(static_cast<Eigen::Index>(nv));
  const vector_t v = state.segment(static_cast<Eigen::Index>(nv), static_cast<Eigen::Index>(nv));
  require(robot.getGeneralizedCoordinates(state).isApprox(q), "generalized coordinates block");
  require(robot.getGeneralizedVelocities(state).isApprox(v), "generalized velocities block");
  require(robot.getJointVelocities(state, input).isApprox(v.tail(static_cast<Eigen::Index>(nj))), "joint velocities block");
  require(robot.getJointTorques(input).isApprox(
      input.segment(static_cast<Eigen::Index>(12 + nv), static_cast<Eigen::Index>(nj))), "joint torques block");
  const vector6_t wrench = vector6_t::LinSpaced(6, 10.0, 60.0);
  robot.setWrenchState(state, wrench, 1);
  robot.setWrenchRate(input, -wrench, 0);
  require(robot.getWrenchState(state, 1).isApprox(wrench) && robot.getWrenchRate(input, 0).isApprox(-wrench),
          "wrench state / rate accessors");
  require(robot.getGeneralizedCoordinates(state).isApprox(q) && robot.getGeneralizedVelocities(state).isApprox(v),
          "setting a wrench state must not touch [q, v]");
  vector_t velocities = vector_t::Constant(static_cast<Eigen::Index>(nv), 3.0);
  robot.setGeneralizedVelocities(state, velocities);
  require(robot.getWrenchState(state, 1).isApprox(wrench), "setting the velocities must not touch the wrench states");
  vector_t torques = vector_t::Constant(static_cast<Eigen::Index>(nj), -3.0);
  robot.setJointTorques(input, torques);
  require(robot.getWrenchRate(input, 0).isApprox(-wrench), "setting the torques must not touch the wrench rates");

  // Dynamics: q' = v, v' = a, Ws' = dW; linearisation consistent with the flow map.
  WbIdDynamics dynamics(robot);
  const vector_t xdot = dynamics.computeFlowMap(0.0, state, input, PreComputation());
  require(xdot.head(static_cast<Eigen::Index>(nv)).isApprox(robot.getGeneralizedVelocities(state)), "q' = v");
  require(xdot.segment(static_cast<Eigen::Index>(nv), static_cast<Eigen::Index>(nv))
            .isApprox(robot.getGeneralizedAccelerations(input)), "v' = a");
  require(xdot.segment<6>(static_cast<Eigen::Index>(robot.getWrenchStateStartIndex(0))).isApprox(robot.getWrenchRate(input, 0)) &&
          xdot.segment<6>(static_cast<Eigen::Index>(robot.getWrenchStateStartIndex(1))).isApprox(robot.getWrenchRate(input, 1)),
          "Ws' = dW");
  const auto lin = dynamics.linearApproximation(0.0, state, input, PreComputation());
  require(lin.f.isApprox(xdot) && (lin.dfdx * state + lin.dfdu * input).isApprox(xdot),
          "the dynamics are linear and its approximation must reproduce the flow map");

  // Pinocchio mapping ignores the trailing wrench states and pads their Jacobian columns.
  const WbIdPinocchioMapping mapping(nv, robot.getInputDim(), robot.getStateDim());
  require(mapping.getPinocchioJointVelocity(state, input).isApprox(robot.getGeneralizedVelocities(state)),
          "pinocchio mapping velocity block");
  const matrix_t Jq = matrix_t::Random(3, static_cast<Eigen::Index>(nv));
  const matrix_t Jv = matrix_t::Random(3, static_cast<Eigen::Index>(nv));
  const auto [dfdx, dfdu] = mapping.getOcs2Jacobian(state, Jq, Jv);
  require(dfdx.rows() == 3 && dfdx.cols() == static_cast<Eigen::Index>(robot.getStateDim()) &&
          dfdx.leftCols(static_cast<Eigen::Index>(nv)).isApprox(Jq) &&
          dfdx.middleCols(static_cast<Eigen::Index>(nv), static_cast<Eigen::Index>(nv)).isApprox(Jv) &&
          dfdx.rightCols(12).isZero() && dfdu.cols() == static_cast<Eigen::Index>(robot.getInputDim()),
          "pinocchio mapping Jacobian padding");
  const WbIdDynamics plainDynamics(base);
  (void)plainDynamics;
  std::cout << "g1: wrench-as-state layout checks passed\n";
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    require(argc == 2, "usage: model_settings_test ROBOT_URDF_ROOT");
    checkRobot(argv[1], "g1", 29);
    checkRobot(argv[1], "h2", 31);
    checkRobot(argv[1], "upkie", 6);
    checkWrenchStateLayout(argv[1]);
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
