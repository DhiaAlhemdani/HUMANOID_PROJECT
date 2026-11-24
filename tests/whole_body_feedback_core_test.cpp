// Eigen-only regression test of the Phase 2.B S1 whole-body feedback core:
// constraint satisfaction, exact recovery of a dynamically consistent plan, swing task
// tracking, flight-phase projection and the wrench clip.
#include "legged_robot_mpc_controller/controller/whole_body_feedback_core.hpp"

#include <cassert>
#include <cstdio>
#include <random>

using namespace legged_robot_mpc_controller::wbc;

namespace
{
constexpr int kNv = 6 + 8;  // base + 8 joints keeps the test small; the core is dimension-agnostic

struct Rig
{
  Problem p;
  std::mt19937 rng{7};
  Eigen::MatrixXd randomMatrix(int r, int c)
  {
    std::normal_distribution<double> d(0.0, 1.0);
    Eigen::MatrixXd m(r, c);
    for (int i = 0; i < r; ++i) {
      for (int j = 0; j < c; ++j) {
        m(i, j) = d(rng);
      }
    }
    return m;
  }
  Rig()
  {
    const Eigen::MatrixXd L = randomMatrix(kNv, kNv);
    p.massMatrix = L * L.transpose() + 0.5 * Eigen::MatrixXd::Identity(kNv, kNv);
    p.nonlinearEffects = randomMatrix(kNv, 1);
    p.desiredAcc = randomMatrix(kNv, 1);
    p.accWeight = Eigen::VectorXd::Ones(kNv);
    for (auto& c : p.contacts) {
      c.jacobian = randomMatrix(6, kNv);
      c.driftAcc = randomMatrix(6, 1);
      c.wrenchPlan.setZero();
      c.inContact = true;
    }
    for (auto& s : p.swings) {
      s.active = false;
      s.jacobian = randomMatrix(3, kNv);
      s.driftAcc = randomMatrix(3, 1);
      s.desiredAcc.setZero();
    }
    p.swingWeight = 0.0;
    p.wrenchWeight = 1e-3;
  }
};

const WrenchLimits kLoose{10.0, 10.0, 10.0};

void checkConstraints(const Problem& p, const Solution& s, double tol)
{
  assert(s.valid);
  Vector6 r = p.massMatrix.topRows(6) * s.acc + p.nonlinearEffects.head(6);
  for (std::size_t c = 0; c < kContacts; ++c) {
    if (p.contacts[c].inContact) {
      r -= p.contacts[c].jacobian.leftCols(6).transpose() * s.wrench[c];
      const Vector6 contact = p.contacts[c].jacobian * s.acc + p.contacts[c].driftAcc;
      assert(contact.norm() < tol);
    } else {
      assert(s.wrench[c].norm() == 0.0);
    }
  }
  assert(r.norm() < tol);
  assert(s.dynamicsResidual < tol);
}

void testDoubleSupportConstraints()
{
  Rig rig;
  const Solution s = solve(rig.p, kLoose);
  checkConstraints(rig.p, s, 1e-6);
  // Torques are the joint rows of the inverse dynamics at the solution.
  const Eigen::VectorXd tau = jointTorques(rig.p, s.acc, s.wrench);
  assert((tau - s.jointTorque).norm() < 1e-9);  // loose limits -> nothing clipped
  assert(s.wrenchClips == 0);
  std::puts("double support: constraints hold, torques consistent");
}

void testRecoversConsistentPlan()
{
  // Build a plan (a*, F*) that satisfies both constraint sets, then ask for exactly it.
  Rig rig;
  Problem& p = rig.p;
  // Pick F* first, then solve the constraints for an a* (least norm) and set drift accordingly.
  std::array<Vector6, 2> F{Vector6::Constant(1.0), Vector6::Constant(-0.5)};
  Eigen::VectorXd aStar = rig.randomMatrix(kNv, 1);
  // Make the base rows consistent by choosing h_b from a*, F*.
  Eigen::VectorXd h = p.nonlinearEffects;
  h.head(6) = -(p.massMatrix.topRows(6) * aStar);
  for (int c = 0; c < 2; ++c) {
    h.head(6) += p.contacts[c].jacobian.leftCols(6).transpose() * F[c];
    p.contacts[c].driftAcc = -(p.contacts[c].jacobian * aStar);
    p.contacts[c].wrenchPlan = F[c];
  }
  p.nonlinearEffects = h;
  p.desiredAcc = aStar;
  const Solution s = solve(p, kLoose);
  checkConstraints(p, s, 1e-6);
  assert((s.acc - aStar).norm() < 1e-6);
  for (int c = 0; c < 2; ++c) {
    assert((s.wrench[c] - F[c]).norm() < 1e-5);
  }
  std::puts("consistent plan: recovered exactly");
}

void testSingleSupportAndSwingTask()
{
  Rig rig;
  Problem& p = rig.p;
  p.contacts[1].inContact = false;
  p.swings[1].active = true;
  p.swings[1].desiredAcc << 1.0, -2.0, 3.0;
  p.swingWeight = 1e6;
  const Solution s = solve(p, kLoose);
  checkConstraints(p, s, 1e-5);
  const Eigen::Vector3d swingAcc = p.swings[1].jacobian * s.acc + p.swings[1].driftAcc;
  assert((swingAcc - p.swings[1].desiredAcc).norm() < 1e-3);
  std::puts("single support: swing task tracked under the stance constraint");
}

void testFlight()
{
  Rig rig;
  Problem& p = rig.p;
  p.contacts[0].inContact = false;
  p.contacts[1].inContact = false;
  const Solution s = solve(p, kLoose);
  checkConstraints(p, s, 1e-6);
  // Joint accelerations are free; only the 6 base rows bind, so the joint part of a
  // can only differ from a_des through those rows' least-squares projection.
  assert(s.acc.size() == kNv);
  std::puts("flight: floating-base rows hold with no contact");
}

void testClip()
{
  WrenchLimits lim{0.5, 0.1, 0.05};
  Vector6 w;
  w << 3.0, -3.0, 4.0, 1.0, -1.0, 0.0;
  const int clips = clipWrench(w, lim);
  assert(clips == 4);
  assert(w(0) == 2.0 && w(1) == -2.0 && w(2) == 4.0);
  assert(std::abs(w(3) - 0.2) < 1e-12 && std::abs(w(4) + 0.4) < 1e-12);
  Vector6 pulling;
  pulling << 0.1, 0.0, -5.0, 0.0, 0.0, 0.0;
  clipWrench(pulling, lim);
  assert(pulling.norm() == 0.0);  // unilateral: a pulling foot carries nothing
  std::puts("clip: cone, CoP box and unilateral");
}

void testRejectsBadSizes()
{
  Rig rig;
  rig.p.accWeight.resize(3);
  const Solution s = solve(rig.p, kLoose);
  assert(!s.valid);
  std::puts("bad sizes: rejected");
}
}  // namespace

int main()
{
  testDoubleSupportConstraints();
  testRecoversConsistentPlan();
  testSingleSupportAndSwingTask();
  testFlight();
  testClip();
  testRejectsBadSizes();
  std::puts("whole_body_feedback_core_test: OK");
  return 0;
}
