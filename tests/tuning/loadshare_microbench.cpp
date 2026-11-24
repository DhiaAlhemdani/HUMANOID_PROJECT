// Harness for the extracted double-support load-share free functions.
// tests/tuning/run_loadshare_microbench.py extracts the marked section of
// include/legged_robot_mpc_controller/mpc/wb_id/cost/WbIdContactLoadShareCost.h into
// loadshare_free.inc (on the include path), compiles this file with plain g++ and runs
// the gates below. No ROS, no ocs2, no pinocchio.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <vector>

#include "loadshare_free.inc"

namespace {

int failures = 0;

#define GATE(name, cond)                                                    \
  do {                                                                      \
    if (cond) {                                                             \
      std::printf("GATE %s: PASS\n", name);                                 \
    } else {                                                                \
      std::printf("GATE %s: FAIL\n", name);                                 \
      ++failures;                                                           \
    }                                                                       \
  } while (0)

using Flags = std::array<bool, 2>;

// Synthetic walk schedule: [0,0.3) LF, [0.3,0.4) DS, [0.4,0.7) RF, [0.7,0.8) DS,
// [0.8,1.1) LF, [1.1,1.2) DS, >=1.2 RF — walk switchingTimes [0,0.3,0.4,0.7,0.8,...].
Flags syntheticFlags(double t) {
  static const std::vector<double> events{0.3, 0.4, 0.7, 0.8, 1.1, 1.2};
  static const std::vector<Flags> modes{
      {true, false}, {true, true}, {false, true}, {true, true}, {false, true}, {true, true}, {false, true}};
  std::size_t idx = 0;
  while (idx < events.size() && t >= events[idx]) {
    ++idx;
  }
  return modes[idx];
}

}  // namespace

int main() {
  using namespace load_share;
  const std::vector<double> events{0.3, 0.4, 0.7, 0.8, 1.1, 1.2};
  const auto getter = [](double t) { return syntheticFlags(t); };

  // G1: exact brackets + pre-DS flags for both DS windows.
  double entry = 0.0;
  double exit = 0.0;
  bool before[2] = {false, false};
  bool ok = dsBrackets(0.35, events, getter, entry, exit, before);
  GATE("brackets_first_ds", ok && std::fabs(entry - 0.3) < 1e-12 && std::fabs(exit - 0.4) < 1e-12 && before[0] &&
                                !before[1]);
  ok = dsBrackets(0.75, events, getter, entry, exit, before);
  GATE("brackets_second_ds", ok && std::fabs(entry - 0.7) < 1e-12 && std::fabs(exit - 0.8) < 1e-12 && !before[0] &&
                                  before[1]);

  // G2: transfer endpoints — entering foot 0 at entry, 1 at exit; pre-DS foot opposite.
  double pe = 0.0;
  double pp = 0.0;
  shareTargets(transferProgress(0.3 + 1e-9, 0.3, 0.4), pe, pp);
  GATE("entry_targets", pe < 1e-6 && std::fabs(pp - 1.0) < 1e-6);
  shareTargets(transferProgress(0.4 - 1e-9, 0.3, 0.4), pe, pp);
  GATE("exit_targets", std::fabs(pe - 1.0) < 1e-6 && pp < 1e-6);

  // G3: targets sum to 1 and the entering share is monotone across the window.
  double maxSumErr = 0.0;
  double prevPe = -1.0;
  bool monotone = true;
  for (double t = 0.3; t <= 0.4 + 1e-12; t += 1e-4) {
    shareTargets(transferProgress(t, 0.3, 0.4), pe, pp);
    maxSumErr = std::max(maxSumErr, std::fabs(pe + pp - 1.0));
    if (pe < prevPe - 1e-12) {
      monotone = false;
    }
    prevPe = pe;
  }
  GATE("sum_to_one", maxSumErr < 1e-12);
  GATE("monotone_transfer", monotone);

  // G4: zero target slope at both ends (C1 handover — no rate jump into/exit the ramp).
  const double slopeEntry = (transferProgress(0.3 + 1e-6, 0.3, 0.4) - transferProgress(0.3, 0.3, 0.4)) / 1e-6;
  const double slopeExit = (transferProgress(0.4, 0.3, 0.4) - transferProgress(0.4 - 1e-6, 0.3, 0.4)) / 1e-6;
  GATE("zero_edge_slopes", std::fabs(slopeEntry) < 1.0 && std::fabs(slopeExit) < 1.0);

  // G5: residual is zero at the proportional split and antisymmetric on imbalance.
  const double weight = 1e-4;
  const double sqrtW = std::sqrt(weight);
  const double total = 400.0;   // N, nominal G1 load share sum
  const double imbalance = 30.0;  // N
  shareTargets(0.37, pe, pp);
  double r[2] = {0.0, 0.0};
  splitResidual(sqrtW, pe * total, pp * total, pe, pp, r);
  GATE("zero_at_proportional", std::fabs(r[0]) < 1e-12 && std::fabs(r[1]) < 1e-12);
  splitResidual(sqrtW, pe * total + imbalance, pp * total - imbalance, pe, pp, r);
  GATE("imbalance_antisymmetric", std::fabs(r[0] - sqrtW * imbalance) < 1e-12 &&
                                      std::fabs(r[1] + sqrtW * imbalance) < 1e-12);

  // G6: Gauss-Newton cost identity 0.5*(r0^2+r1^2) = weight * imbalance^2.
  const double cost = 0.5 * (r[0] * r[0] + r[1] * r[1]);
  GATE("cost_identity", std::fabs(cost - weight * imbalance * imbalance) <
                            1e-9 * weight * imbalance * imbalance + 1e-15);

  // G7: bracket fallbacks (no complete window -> even split upstream).
  ok = dsBrackets(0.1, events, getter, entry, exit, before);
  GATE("fallback_before_first", !ok);
  ok = dsBrackets(1.5, events, getter, entry, exit, before);
  GATE("fallback_past_last", !ok);
  ok = dsBrackets(0.4, events, getter, entry, exit, before);  // exactly on the exit event
  GATE("fallback_on_event", !ok);                             // pre-DS flags are DS here -> rejected

  // G8: smoothstep basics.
  GATE("smoothstep_basics", smoothstep01(-1.0) == 0.0 && smoothstep01(2.0) == 1.0 &&
                                std::fabs(smoothstep01(0.5) - 0.5) < 1e-12);

  if (failures > 0) {
    std::printf("FAILURES: %d\n", failures);
    return 1;
  }
  std::printf("ALL PASS\n");
  return 0;
}
