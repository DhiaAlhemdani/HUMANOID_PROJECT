// Standalone microbench for CommandSlewLimiter.h (roadmap #1.6 S-curve command slew).
// Built by run_slew_microbench.py with plain g++; prints one JSON object of gate results.
#include <cmath>
#include <cstdio>
#include <vector>

#include "legged_robot_mpc_controller/mpc/common/target/CommandSlewLimiter.h"

using ocs2::humanoid::slewLimitStep;

struct Trace {
  std::vector<double> y, r;  // output and rate per step
};

static Trace run(double target, double rateLimit, double jerkLimit, double dt, int n, double y0 = 0.0) {
  Trace tr;
  double y = y0, rate = 0.0;
  for (int k = 0; k < n; ++k) {
    y = slewLimitStep(target, y, rate, dt, rateLimit, jerkLimit);
    tr.y.push_back(y);
    tr.r.push_back(rate);
  }
  return tr;
}

static double maxAbsDiff(const std::vector<double>& a, double dt) {
  double m = 0.0;
  for (size_t k = 1; k < a.size(); ++k) m = std::max(m, std::fabs(a[k] - a[k - 1]) / dt);
  return m;
}

int main() {
  const double dt = 0.0125;  // ~80 Hz MPC
  int pass = 0, total = 0;
  auto gate = [&](const char* name, bool ok, double value) {
    ++total;
    pass += ok;
    std::printf("%s  \"%s\": {\"ok\": %s, \"value\": %.6g}", total > 1 ? ",\n" : "\n", name, ok ? "true" : "false", value);
  };
  std::printf("{");

  // 1. Legacy path (jerk 0): identical to the old clamp — linear ramp at exactly the rate limit.
  {
    auto tr = run(0.5, 0.5, 0.0, dt, 200);
    double maxRate = maxAbsDiff(tr.y, dt);
    gate("legacy_rate_equals_limit", std::fabs(maxRate - 0.5) < 1e-9, maxRate);
    int kReach = 0;
    while (kReach < 200 && tr.y[kReach] < 0.5 - 1e-12) ++kReach;
    gate("legacy_reaches_in_1s", std::fabs((kReach + 1) * dt - 1.0) < 2 * dt, (kReach + 1) * dt);
  }
  // 2. S-curve: rate never exceeds the limit, jerk never exceeds its limit, no overshoot, converges.
  {
    const double J = 1.0;
    auto tr = run(0.5, 0.5, J, dt, 400);
    gate("scurve_rate_within_limit", maxAbsDiff(tr.y, dt) <= 0.5 + 1e-9, maxAbsDiff(tr.y, dt));
    gate("scurve_jerk_within_limit", maxAbsDiff(tr.r, dt) <= J + 1e-9, maxAbsDiff(tr.r, dt));
    double maxY = 0.0;
    for (double v : tr.y) maxY = std::max(maxY, v);
    gate("scurve_overshoot_below_jerk_dt2", maxY <= 0.5 + J * dt * dt + 1e-12, maxY - 0.5);
    gate("scurve_converges", std::fabs(tr.y.back() - 0.5) < 1e-6, tr.y.back());
    gate("scurve_rate_returns_to_zero", std::fabs(tr.r.back()) < 1e-6, tr.r.back());
    // First step is tiny (jerk-limited start), unlike the legacy step of rate*dt.
    gate("scurve_soft_start", tr.y[0] <= dt * dt * J + 1e-12 && tr.y[0] > 0.0, tr.y[0]);
    // Duration: analytic S-curve 0->0.5 with a=0.5, j=1: accel phase 0.5 s covers 0.125, cruise
    // (0.5-0.25)/0.5 = 0.5 s, decel 0.5 s -> 1.5 s total (legacy 1.0 s).
    int kReach = 0;
    while (kReach < 400 && tr.y[kReach] < 0.5 - 1e-4) ++kReach;
    gate("scurve_duration_about_1p5s", std::fabs((kReach + 1) * dt - 1.5) < 0.1, (kReach + 1) * dt);
  }
  // 3. Reversal mid-ramp: target flips while moving; still no jerk violation, converges to new target.
  {
    const double J = 1.0;
    double y = 0.0, rate = 0.0;
    std::vector<double> ys, rs;
    for (int k = 0; k < 400; ++k) {
      const double target = k < 40 ? 0.5 : -0.3;
      y = slewLimitStep(target, y, rate, dt, 0.5, J);
      ys.push_back(y);
      rs.push_back(rate);
    }
    gate("reversal_jerk_within_limit", maxAbsDiff(rs, dt) <= J + 1e-9, maxAbsDiff(rs, dt));
    gate("reversal_converges", std::fabs(ys.back() + 0.3) < 1e-6, ys.back());
    double minY = 0.0;
    for (double v : ys) minY = std::min(minY, v);
    gate("reversal_undershoot_below_jerk_dt2", minY >= -0.3 - J * dt * dt - 1e-12, minY + 0.3);
  }
  // 4. Small step below one jerk-step: converges without chatter.
  {
    auto tr = run(1e-4, 0.5, 1.0, dt, 100);
    gate("tiny_step_converges", std::fabs(tr.y.back() - 1e-4) < 1e-9, tr.y.back());
    bool mono = true;
    for (size_t k = 1; k < tr.y.size(); ++k) mono &= tr.y[k] >= tr.y[k - 1] - 1e-12;
    gate("tiny_step_monotone", mono, mono);
  }
  // 5. Disabled axis passes the target through; zero dt holds.
  {
    double rate = 0.3;
    gate("disabled_axis_passthrough", slewLimitStep(0.7, 0.1, rate, dt, 0.0, 1.0) == 0.7 && rate == 0.0, rate);
    rate = 0.3;
    gate("zero_dt_holds", slewLimitStep(0.7, 0.1, rate, 0.0, 0.5, 1.0) == 0.1, rate);
  }
  // 6. Stall guard scenario: a 50 ms dt (the caller's clamp) still respects both limits per step.
  {
    double rate = 0.0;
    double y = slewLimitStep(1.0, 0.0, rate, 0.05, 0.5, 1.0);
    gate("large_dt_rate_within_limit", std::fabs(rate) <= 0.05 * 1.0 + 1e-12 && y == rate * 0.05, rate);
  }
  std::printf(",\n  \"summary\": {\"pass\": %d, \"total\": %d}\n}\n", pass, total);
  return pass == total ? 0 : 1;
}
