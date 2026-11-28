// Phase 2.B S1 — whole-body feedback layer: the contact-consistent least-squares core.
//
// Eigen only (no pinocchio, no ROS) so the linear algebra is unit-testable on its own
// (tests/whole_body_feedback_core_test.cpp). The controller assembles a Problem from the
// measured state and the MPC policy (src/controller/whole_body_feedback.cpp) and turns the
// Solution into the effort command.
//
// Problem solved at every servo tick, with the measured q, v:
//
//   min_{a, F}  ½ Σ_i w_a,i (a_i − a_i^des)²                       posture / base tasks (acceleration space)
//             + ½ Σ_sw w_sw ‖J_sw a + J̇_sw v − ẍ_sw^des‖²           swing-foot tasks
//             + ½ Σ_c  w_F ‖F_c − F_c^plan‖²                        wrench tracking (MPC's load distribution)
//   s.t.        M_b a + h_b = Σ_c J_c,bᵀ F_c                       floating-base dynamics (6 rows)
//               J_c a + J̇_c v = 0            for each stance foot  contact does not move (6 rows each)
//
// Joint torques follow from the actuated rows:  τ = M_j a + h_j − Σ_c J_c,jᵀ F_c.
//
// Equality-constrained strictly convex QP -> one dense KKT solve (n ≤ 29 + 12 + 6 + 12 = 59).
// Friction cone / centre-of-pressure limits are not constraints in S1; the solved wrench is
// clipped to them afterwards and τ recomputed with the clipped wrench (S2 adds them to the QP).
#pragma once

#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace legged_robot_mpc_controller::wbc
{

using Vector6 = Eigen::Matrix<double, 6, 1>;
using Matrix6X = Eigen::Matrix<double, 6, Eigen::Dynamic>;
using Matrix3X = Eigen::Matrix<double, 3, Eigen::Dynamic>;

constexpr std::size_t kContacts = 2;
constexpr int kBaseDof = 6;

struct ContactTask
{
  bool inContact{false};
  Matrix6X jacobian;      //!< 6 x nv, LOCAL_WORLD_ALIGNED at the contact frame
  Vector6 driftAcc;       //!< J̇ v (classical acceleration of the frame at zero generalized acceleration)
  Vector6 wrenchPlan;     //!< the MPC's wrench for this foot (zero when swinging)
};

struct SwingTask
{
  bool active{false};
  Matrix3X jacobian;              //!< 3 x nv, linear rows of the foot Jacobian
  Eigen::Vector3d driftAcc;       //!< linear rows of J̇ v
  Eigen::Vector3d desiredAcc;     //!< ẍ_plan + Kp (x_plan − x) + Kd (ẋ_plan − ẋ)
};

//! Any further least-squares task linear in the generalized acceleration:
//! ‖J a + drift − desired‖²_weight (e.g. a link orientation).
struct LinearTask
{
  bool active{false};
  Eigen::MatrixXd jacobian;     //!< k x nv
  Eigen::VectorXd driftAcc;     //!< k
  Eigen::VectorXd desiredAcc;   //!< k
  double weight{0.0};
};

struct Problem
{
  Eigen::MatrixXd massMatrix;      //!< nv x nv, symmetric
  Eigen::VectorXd nonlinearEffects;  //!< nv, Coriolis + gravity
  Eigen::VectorXd desiredAcc;      //!< nv, base rows then joints
  Eigen::VectorXd accWeight;       //!< nv, > 0
  std::array<ContactTask, kContacts> contacts;
  std::array<SwingTask, kContacts> swings;
  std::vector<LinearTask> tasks;
  double swingWeight{0.0};
  double wrenchWeight{0.0};
  //! Tikhonov term on the constraint multipliers: keeps the KKT system solvable when the
  //! stance constraints are redundant (both feet flat on the same plane).
  double constraintRegularization{1e-8};
};

struct Solution
{
  bool valid{false};
  Eigen::VectorXd acc;                       //!< nv
  std::array<Vector6, kContacts> wrench{};   //!< as solved (before clipping)
  Eigen::VectorXd jointTorque;               //!< nv − 6, from the clipped wrench
  double dynamicsResidual{0.0};              //!< ‖M_b a + h_b − Σ J_bᵀ F‖ of the solved (unclipped) point
  int wrenchClips{0};                        //!< components moved by the cone / CoP / unilateral clip
};

//! Touchdown search (run 29): a foot the plan has just switched to contact whose measured sole is still
//! going DOWN faster than `landedVelocity` is not on the ground (a sole on the ground cannot descend);
//! the layer keeps it a swing foot - no wrench, no stance rows, the body held on the other foot - and
//! drives it down at `descentVelocity` at the plan's horizontal foothold until it stops (landed, wherever
//! the ground was) or `window` after the switch (a hole: loaded where it is). Decided every servo tick on
//! the measured foot; nothing goes through the solver, the plan and its schedule are untouched. The
//! search ends early if the plan lifts the other foot. window 0 = off.
struct TouchdownSearchSettings
{
  double window{0.0};           //!< [s]
  double landedVelocity{0.05};  //!< [m/s] a sole descending slower than this has landed
  double descentVelocity{0.15}; //!< [m/s] the searching foot's downward velocity target, at least
  double descentAcceleration{0.0};  //!< [m/s^2] the target grows this fast the longer the search runs (0 = constant)
};

//! The searching foot's downward velocity target (positive = down), `elapsed` seconds into the search. Never
//! slower than the sole came down at the switch (`startVelocity`, positive = down): a search that interrupts a
//! swing's descent (run 29 C: the deceleration ramps re-tile the gait and switch the foot to contact while it
//! is still several centimetres up) must not brake it - at 0.15 m/s it ran out the whole double support
//! without reaching the ground. `descentAcceleration` bounds the time a deep search takes.
inline double searchDescentVelocity(const TouchdownSearchSettings& s, double startVelocity, double elapsed)
{
  const double start = std::isfinite(startVelocity) ? std::max(startVelocity, 0.0) : 0.0;
  return std::max(s.descentVelocity, start) + s.descentAcceleration * std::max(elapsed, 0.0);
}

//! The per-foot decision. `contactSince` is the caller's state (NaN = the plan has the foot swinging),
//! `searching` the previous tick's decision; both are updated in place. Returns the decision.
inline bool touchdownSearch(const TouchdownSearchSettings& s, double time, bool planContact, bool otherPlanContact,
                            double soleVelocityZ, double& contactSince, bool& searching)
{
  if (!planContact) {
    contactSince = std::numeric_limits<double>::quiet_NaN();
    searching = false;
    return false;
  }
  const bool descending = std::isfinite(soleVelocityZ) && soleVelocityZ < -s.landedVelocity;
  if (!std::isfinite(contactSince) || time < contactSince) {
    // The plan's switch to contact (or time went back): search only if the sole is still coming down.
    contactSince = time;
    searching = s.window > 0.0 && descending;
  } else if (searching) {
    // Keep searching while the sole descends inside the window; once it has stopped it is landed (on
    // whatever it found) and stays landed for the rest of this contact phase.
    searching = descending && time - contactSince <= s.window;
  }
  // The body must have a foot to stand on: the plan lifting the other foot ends the search.
  if (searching && !otherPlanContact) {
    searching = false;
  }
  return searching;
}

struct WrenchLimits
{
  double frictionCoefficient{0.6};
  double soleHalfLengthX{0.10};  //!< CoP box, metres, about the contact frame
  double soleHalfLengthY{0.04};
};

//! Unilateral, linearised friction cone and CoP box on a LOCAL_WORLD_ALIGNED wrench
//! [fx, fy, fz, mx, my, mz]. Returns the number of components changed.
inline int clipWrench(Vector6& w, const WrenchLimits& lim)
{
  int clips = 0;
  auto clampTo = [&](double& value, double bound) {
    const double before = value;
    value = std::clamp(value, -bound, bound);
    if (value != before) {
      ++clips;
    }
  };
  if (w(2) < 0.0) {
    w(2) = 0.0;
    ++clips;
  }
  const double fz = w(2);
  clampTo(w(0), lim.frictionCoefficient * fz);
  clampTo(w(1), lim.frictionCoefficient * fz);
  clampTo(w(3), lim.soleHalfLengthY * fz);  // roll moment limited by the sole width
  clampTo(w(4), lim.soleHalfLengthX * fz);  // pitch moment limited by the sole length
  clampTo(w(5), lim.frictionCoefficient * fz * std::max(lim.soleHalfLengthX, lim.soleHalfLengthY));
  return clips;
}

//! Joint rows of the inverse dynamics with the given accelerations and wrenches.
inline Eigen::VectorXd jointTorques(const Problem& p, const Eigen::VectorXd& acc,
                                    const std::array<Vector6, kContacts>& wrench)
{
  const Eigen::Index nv = p.massMatrix.rows();
  const Eigen::Index nj = nv - kBaseDof;
  Eigen::VectorXd tau = p.massMatrix.bottomRows(nj) * acc + p.nonlinearEffects.tail(nj);
  for (std::size_t c = 0; c < kContacts; ++c) {
    if (p.contacts[c].inContact) {
      tau.noalias() -= p.contacts[c].jacobian.rightCols(nj).transpose() * wrench[c];
    }
  }
  return tau;
}

inline Solution solve(const Problem& p, const WrenchLimits& limits)
{
  Solution s;
  const Eigen::Index nv = p.massMatrix.rows();
  if (nv <= kBaseDof || p.nonlinearEffects.size() != nv || p.desiredAcc.size() != nv || p.accWeight.size() != nv) {
    return s;
  }
  int nc = 0;
  for (const auto& c : p.contacts) {
    nc += c.inContact ? 1 : 0;
  }
  const Eigen::Index nF = 6 * nc;
  const Eigen::Index nx = nv + nF;
  const Eigen::Index nEq = kBaseDof + nF;
  const Eigen::Index n = nx + nEq;

  // Hessian / gradient of the weighted least squares.
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(nx, nx);
  Eigen::VectorXd g = Eigen::VectorXd::Zero(nx);
  H.topLeftCorner(nv, nv).diagonal() = p.accWeight;
  g.head(nv) = p.accWeight.cwiseProduct(p.desiredAcc);
  for (const auto& sw : p.swings) {
    if (sw.active && p.swingWeight > 0.0 && sw.jacobian.cols() == nv) {
      H.topLeftCorner(nv, nv).noalias() += p.swingWeight * sw.jacobian.transpose() * sw.jacobian;
      g.head(nv).noalias() += p.swingWeight * sw.jacobian.transpose() * (sw.desiredAcc - sw.driftAcc);
    }
  }
  for (const auto& task : p.tasks) {
    if (task.active && task.weight > 0.0 && task.jacobian.cols() == nv && task.jacobian.rows() == task.driftAcc.size() &&
        task.jacobian.rows() == task.desiredAcc.size()) {
      H.topLeftCorner(nv, nv).noalias() += task.weight * task.jacobian.transpose() * task.jacobian;
      g.head(nv).noalias() += task.weight * task.jacobian.transpose() * (task.desiredAcc - task.driftAcc);
    }
  }
  // Equalities A x = b.
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(nEq, nx);
  Eigen::VectorXd b = Eigen::VectorXd::Zero(nEq);
  A.topLeftCorner(kBaseDof, nv) = p.massMatrix.topRows(kBaseDof);
  b.head(kBaseDof) = -p.nonlinearEffects.head(kBaseDof);
  Eigen::Index col = nv;
  Eigen::Index row = kBaseDof;
  for (const auto& c : p.contacts) {
    if (!c.inContact) {
      continue;
    }
    if (c.jacobian.cols() != nv) {
      return s;
    }
    A.block(0, col, kBaseDof, 6) = -c.jacobian.leftCols(kBaseDof).transpose();
    A.block(row, 0, 6, nv) = c.jacobian;
    b.segment(row, 6) = -c.driftAcc;
    H.block(col, col, 6, 6).diagonal().setConstant(p.wrenchWeight);
    g.segment(col, 6) = p.wrenchWeight * c.wrenchPlan;
    col += 6;
    row += 6;
  }

  // KKT system  [H Aᵀ; A −εI] [x; λ] = [g; b].
  Eigen::MatrixXd K = Eigen::MatrixXd::Zero(n, n);
  K.topLeftCorner(nx, nx) = H;
  K.topRightCorner(nx, nEq) = A.transpose();
  K.bottomLeftCorner(nEq, nx) = A;
  K.bottomRightCorner(nEq, nEq).diagonal().setConstant(-p.constraintRegularization);
  Eigen::VectorXd rhs(n);
  rhs << g, b;
  const Eigen::VectorXd sol = K.partialPivLu().solve(rhs);
  if (!sol.allFinite()) {
    return s;
  }
  s.acc = sol.head(nv);
  col = nv;
  for (std::size_t c = 0; c < kContacts; ++c) {
    s.wrench[c].setZero();
    if (p.contacts[c].inContact) {
      s.wrench[c] = sol.segment<6>(col);
      col += 6;
    }
  }
  // Residual of the floating-base rows at the solved point (regularisation and conditioning check).
  {
    Vector6 r = p.massMatrix.topRows(kBaseDof) * s.acc + p.nonlinearEffects.head(kBaseDof);
    for (std::size_t c = 0; c < kContacts; ++c) {
      if (p.contacts[c].inContact) {
        r.noalias() -= p.contacts[c].jacobian.leftCols(kBaseDof).transpose() * s.wrench[c];
      }
    }
    s.dynamicsResidual = r.norm();
  }
  // Physical limits the QP did not know about: clip, then take the torques of the clipped wrench.
  std::array<Vector6, kContacts> clipped = s.wrench;
  for (std::size_t c = 0; c < kContacts; ++c) {
    if (p.contacts[c].inContact) {
      s.wrenchClips += clipWrench(clipped[c], limits);
    }
  }
  s.jointTorque = jointTorques(p, s.acc, clipped);
  s.valid = s.jointTorque.allFinite();
  return s;
}

}  // namespace legged_robot_mpc_controller::wbc
