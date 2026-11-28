// Eigen-only regression test of the Phase 2.B S1 whole-body feedback core:
// constraint satisfaction, exact recovery of a dynamically consistent plan, swing task
// tracking, flight-phase projection and the wrench clip.
#include "legged_robot_mpc_controller/controller/whole_body_feedback_core.hpp"

#include <cassert>
#include <cstdio>
#include <limits>
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

void testLinearTaskTracked()
{
  // A 3-row orientation-like task with a large weight is met under the stance constraints.
  Rig rig;
  Problem& p = rig.p;
  p.contacts[1].inContact = false;  // single support: 14 - 6 = 8 free acceleration directions for a 3-row task
  LinearTask task;
  task.active = true;
  task.jacobian = rig.randomMatrix(3, kNv);
  task.driftAcc = rig.randomMatrix(3, 1);
  task.desiredAcc = Eigen::Vector3d(0.5, -0.25, 2.0);
  task.weight = 1e4;  // large but keeps the KKT conditioning realistic (production weights are <= 100)
  p.tasks.push_back(task);
  const Solution s = solve(p, kLoose);
  checkConstraints(p, s, 1e-4);  // the multiplier regularisation (1e-8) times the task weight bounds the violation
  const Eigen::Vector3d got = task.jacobian * s.acc + task.driftAcc;
  assert((got - task.desiredAcc).norm() < 1e-2);
  // An inactive or mis-sized task is ignored, not fatal.
  p.tasks.front().active = false;
  p.tasks.push_back(LinearTask{true, rig.randomMatrix(3, kNv + 1), rig.randomMatrix(3, 1), rig.randomMatrix(3, 1), 1.0});
  assert(solve(p, kLoose).valid);
  std::puts("linear task: tracked under constraints; inactive/mis-sized ignored");
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
void testTouchdownSearch()
{
  constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
  TouchdownSearchSettings s;
  s.window = 0.3;
  // The crest: the plan switches the foot to contact at t = 10.0 while the sole still comes down at the
  // swing's end velocity; it searches until the sole stops at 10.2 (the ramp, 2.6 cm lower), then stays landed.
  double since = kNaN;
  bool searching = false;
  assert(!touchdownSearch(s, 9.99, false, true, -0.11, since, searching) && std::isnan(since));
  assert(touchdownSearch(s, 10.0, true, true, -0.11, since, searching) && since == 10.0);
  assert(touchdownSearch(s, 10.1, true, true, -0.14, since, searching));
  assert(!touchdownSearch(s, 10.2, true, true, -0.01, since, searching) && since == 10.0);
  assert(!touchdownSearch(s, 10.25, true, true, -0.2, since, searching));  // a later wobble does not restart it
  assert(!touchdownSearch(s, 10.5, false, true, -0.2, since, searching) && std::isnan(since));  // lifted: state cleared
  // An ordinary touchdown: the sole has stopped by the switch - no search.
  since = kNaN;
  searching = false;
  assert(!touchdownSearch(s, 20.0, true, true, 0.0, since, searching) && since == 20.0);
  assert(!touchdownSearch(s, 20.01, true, true, -0.3, since, searching));
  // A hole: still descending at the end of the window - loaded where it is.
  since = kNaN;
  searching = false;
  assert(touchdownSearch(s, 30.0, true, true, -0.15, since, searching));
  assert(touchdownSearch(s, 30.29, true, true, -0.15, since, searching));
  assert(!touchdownSearch(s, 30.31, true, true, -0.15, since, searching));
  assert(!touchdownSearch(s, 30.32, true, true, -0.15, since, searching));
  // The plan lifts the other foot mid-search: the body needs a foot, the search ends.
  since = kNaN;
  searching = false;
  assert(touchdownSearch(s, 40.0, true, true, -0.15, since, searching));
  assert(!touchdownSearch(s, 40.05, true, false, -0.15, since, searching));
  assert(!touchdownSearch(s, 40.06, true, true, -0.15, since, searching));  // and does not come back
  // Unknown velocity: no search. Off (window 0): never.
  since = kNaN;
  searching = false;
  assert(!touchdownSearch(s, 50.0, true, true, kNaN, since, searching));
  TouchdownSearchSettings off;
  since = kNaN;
  searching = false;
  assert(!touchdownSearch(off, 60.0, true, true, -0.3, since, searching) && since == 60.0);
  // The descent target (run 29 D): at least descentVelocity, never slower than the sole arrived, growing with
  // descentAcceleration; an unknown or upward start velocity counts as zero.
  const auto nearTo = [](double a, double b) { return std::abs(a - b) < 1e-12; };
  assert(nearTo(searchDescentVelocity(s, 0.11, 0.0), 0.15));
  assert(nearTo(searchDescentVelocity(s, 0.3, 0.0), 0.3));
  assert(nearTo(searchDescentVelocity(s, -0.2, 0.0), 0.15));
  assert(nearTo(searchDescentVelocity(s, kNaN, 0.1), 0.15));
  s.descentAcceleration = 1.5;
  assert(nearTo(searchDescentVelocity(s, 0.11, 0.1), 0.30));
  assert(nearTo(searchDescentVelocity(s, 0.3, 0.2), 0.60));
  assert(nearTo(searchDescentVelocity(s, 0.3, -1.0), 0.3));
  // Time going back (a reset) is a fresh switch.
  since = kNaN;
  searching = false;
  assert(!touchdownSearch(s, 70.0, true, true, 0.0, since, searching));
  assert(touchdownSearch(s, 5.0, true, true, -0.2, since, searching) && since == 5.0);
  std::puts("touchdown search: crest, ordinary landing, hole, other foot lifted, unknown velocity, off, reset");
}
}  // namespace

int main()
{
  testTouchdownSearch();
  testDoubleSupportConstraints();
  testRecoversConsistentPlan();
  testSingleSupportAndSwingTask();
  testLinearTaskTracked();
  testFlight();
  testClip();
  testRejectsBadSizes();
  std::puts("whole_body_feedback_core_test: OK");
  return 0;
}
