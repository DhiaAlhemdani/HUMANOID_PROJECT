// Micro-benchmark for the solve-handover command blend (control.solveBlendSeconds).
//
// run_blend_microbench.py extracts apply_policy_swap_blend() VERBATIM from
// src/controller/controller.cpp and compiles it together with this file, so the code
// measured here is exactly the shipped code. This file only provides stub types
// (mirroring the controller's) and a synthetic 1 kHz MRT / per-solve policy trace
// calibrated to the documented G1 walk measurements (docs/wb_id_tuning_benchmark.md:
// legs cmd_tau_jump_at_switch 0.79 Nm vs 0.31 Nm on other ticks).
//
// This is a mechanism-level test: it proves the ramp removes the per-solve command
// step without steady-state lag. It does NOT replace the MuJoCo A/B (run_blend_ab.sh).
//
// usage: blend_microbench <blend_extracted.inc>   (see run_blend_microbench.py)
// prints one JSON object on stdout; exits non-zero if a gate fails.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <string>
#include <vector>

// ---- stubs mirroring the controller (real vector_t is ocs2::vector_t = Eigen::VectorXd) ----
struct vector_t : std::vector<double> {
  vector_t() = default;
  vector_t(std::initializer_list<double> l) : std::vector<double>(l) {}
  explicit vector_t(std::size_t n, double v = 0.0) : std::vector<double>(n, v) {}
};
inline vector_t operator*(double s, const vector_t& v) {
  vector_t r; r.reserve(v.size());
  for (double x : v) r.push_back(s * x);
  return r;
}
inline vector_t operator*(const vector_t& v, double s) { return s * v; }
inline vector_t operator+(const vector_t& a, const vector_t& b) {
  vector_t r; r.reserve(a.size());
  for (std::size_t i = 0; i < a.size(); ++i) r.push_back(a[i] + b[i]);
  return r;
}

struct HumanoidWholeBodyInverseDynamicsMpcController {
  struct JointActionCommand {
    vector_t policy_position;
    vector_t policy_velocity;
    vector_t feedforward;
  };
  struct Params {
    struct Control { double solveBlendSeconds = 0.0; } control;
  } parameters_;
  bool has_last_written_command_{false};
  double policy_blend_start_time_{-1.0};
  JointActionCommand last_written_command_;
  JointActionCommand last_policy_blend_from_;

  // Real clamp_torque() clamps to the configured URDF/override limits; the synthetic
  // torques stay far below any real limit, so the bound only matters as a presence check.
  void clamp_torque(vector_t& tau) const {
    for (double& t : tau) t = std::clamp(t, -1.0e9, 1.0e9);
  }
  JointActionCommand apply_policy_swap_blend(JointActionCommand command, double now, bool policy_updated);
};

#include BLEND_EXTRACTED_INC  // the verbatim apply_policy_swap_blend() definition

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDt = 0.001;           // 1 kHz MRT loop
constexpr double kDuration = 60.0;      // s
constexpr double kWalkA = 5.0;          // metrics window start [s]
constexpr double kWalkB = 55.0;         // metrics window end [s]
constexpr int kNJ = 20;                 // 12 leg + 8 arm joints (G1 MPC-active legs + arms)
constexpr int kNLeg = 12;
constexpr double kOffTarget = 0.31;     // documented legs step on non-switch ticks [Nm]
constexpr double kOnTarget = 0.79;      // documented legs jump at the switch tick [Nm]

struct Case {
  int solve_ticks;
  double blend_ms;
  bool jitter;
  const char* label;
};

struct Schedule {
  std::vector<int> solve_of_tick;
  std::vector<char> is_switch;  // tick where updatePolicy() would return true
  int switches = 0;
};

Schedule make_schedule(int period, bool jitter, unsigned seed) {
  const int n = static_cast<int>(kDuration / kDt);
  Schedule s;
  s.solve_of_tick.assign(n, 0);
  s.is_switch.assign(n, 0);
  std::mt19937 rng(seed);
  int next = period, k = 0;
  for (int t = 0; t < n; ++t) {
    if (t == next) {
      s.is_switch[t] = 1;
      if (t > 0) ++s.switches;
      int step = period;
      if (jitter) step += static_cast<int>(rng() % 5) - 2;  // +-2 ms solve jitter
      step = std::max(step, 3);
      next = t + step;
      ++k;
    }
    s.solve_of_tick[t] = k;
  }
  return s;
}

// Per-joint synthetic policy: smooth gait-band signal + a per-solve disagreement term.
// raw(t) is what a hard policy swap would write at time t.
std::vector<std::vector<double>> make_raw(const Schedule& sched, double amp_scale, double dj_scale,
                                          unsigned delta_seed) {
  const int n = static_cast<int>(sched.solve_of_tick.size());
  const int solves = sched.solve_of_tick.back() + 1;
  static const double leg_amp[6] = {40.0, 25.0, 25.0, 45.0, 18.0, 14.0};  // hip p/r/y, knee, ank p/r; j%6 mirrors both legs
  std::mt19937 rng(delta_seed);
  std::uniform_real_distribution<double> uni(-0.5, 0.5);
  // delta[k][j]: each solve's plan disagrees with its neighbours by ~dj_scale [Nm].
  std::vector<std::vector<double>> delta(solves, std::vector<double>(kNJ, 0.0));
  for (int k = 0; k < solves; ++k)
    for (int j = 0; j < kNJ; ++j)
      delta[k][j] = dj_scale * ((j < kNLeg ? 1.0 : 0.25) * uni(rng));

  std::vector<std::vector<double>> raw(kNJ, std::vector<double>(n, 0.0));
  for (int j = 0; j < kNJ; ++j) {
    const double a = amp_scale * (j < kNLeg ? leg_amp[j % 6] : 0.25 * leg_amp[j % 6]);
    const double ph1 = 0.7 * j, ph2 = 1.3 * j;
    for (int t = 0; t < n; ++t) {
      const double time = t * kDt;
      raw[j][t] = a * (std::sin(2.0 * kPi * 1.5 * time + ph1) + 0.4 * std::sin(2.0 * kPi * 0.4 * time + ph2)) +
                  delta[sched.solve_of_tick[t]][j];
    }
  }
  return raw;
}

struct Stats {
  double jump_on = 0.0, step_off = 0.0, max_step = 0.0, p99_step = 0.0;
  double lag_rms = 0.0, lag_after_ramp_max = 0.0;
  int switches = 0;
};

Stats evaluate(const Schedule& sched, const std::vector<std::vector<double>>& raw, double blend_ms) {
  using JC = HumanoidWholeBodyInverseDynamicsMpcController::JointActionCommand;
  HumanoidWholeBodyInverseDynamicsMpcController c;
  c.parameters_.control.solveBlendSeconds = blend_ms / 1000.0;  // parameter is seconds
  const int n = static_cast<int>(raw[0].size());
  const int walk_a = static_cast<int>(kWalkA / kDt), walk_b = static_cast<int>(kWalkB / kDt);
  const int ramp_ticks = static_cast<int>(std::ceil(blend_ms / 1000.0 / kDt));
  std::vector<char> in_ramp(n, 0);
  for (int t = 0; t < n; ++t)
    if (sched.is_switch[t])
      for (int r = 0; r <= ramp_ticks && t + r < n; ++r) in_ramp[t + r] = 1;

  std::vector<double> on, off, all;  // leg |Δcmd| samples by tick class
  double lag_sq = 0.0;  long long lag_n = 0;
  double lag_after = 0.0;
  for (int j = 0; j < kNJ; ++j) {
    JC cmd;
    cmd.policy_position = vector_t(1, raw[j][0]);
    cmd.policy_velocity = vector_t(1, 0.0);
    cmd.feedforward = vector_t(1, raw[j][0]);
    c.has_last_written_command_ = false;
    c.policy_blend_start_time_ = -1.0;
    double prev = 0.0;
    for (int t = 0; t < n; ++t) {
      JC target;
      target.policy_position = vector_t(1, raw[j][t]);
      target.policy_velocity = vector_t(1, 0.0);
      target.feedforward = vector_t(1, raw[j][t]);
      cmd = c.apply_policy_swap_blend(std::move(target), t * kDt, sched.is_switch[t] != 0);
      const double out = cmd.feedforward[0];
      if (t > 0) {
        const double d = std::abs(out - prev);
        if (t >= walk_a && t < walk_b && j < kNLeg) {
          all.push_back(d);
          if (sched.is_switch[t]) {
            on.push_back(d);
          } else {
            off.push_back(d);  // plot_tracking.py's cmd_tau_step_otherwise: all
          }                    // non-switch ticks (ramp ticks included, as shipped)
        }
      }
      if (t >= walk_a && t < walk_b && j < kNLeg) {
        const double lag = std::abs(out - raw[j][t]);
        lag_sq += lag * lag; ++lag_n;
        if (!in_ramp[t]) lag_after = std::max(lag_after, lag);
      }
      prev = out;
    }
  }
  auto mean = [](const std::vector<double>& v) {
    if (v.empty()) return 0.0;
    double s = 0.0; for (double x : v) s += x;
    return s / static_cast<double>(v.size());
  };
  Stats st;
  st.switches = sched.switches;
  st.jump_on = mean(on);
  st.step_off = mean(off);
  double mx = 0.0; for (double d : all) mx = std::max(mx, d);
  st.max_step = mx;
  std::vector<double> sorted = all;
  std::sort(sorted.begin(), sorted.end());
  st.p99_step = sorted.empty() ? 0.0 : sorted[static_cast<std::size_t>(0.99 * (sorted.size() - 1))];
  st.lag_rms = lag_n ? std::sqrt(lag_sq / static_cast<double>(lag_n)) : 0.0;
  st.lag_after_ramp_max = lag_after;
  return st;
}

void emit(const char* label, const char* status, const Stats& s, double solve_hz, double blend_ms) {
  printf("  {\"label\": \"%s\", \"status\": \"%s\", \"solve_hz\": %.1f, \"blend_ms\": %.1f, "
         "\"jump_on\": %.4f, \"step_off\": %.4f, \"max_step\": %.4f, \"p99_step\": %.4f, "
         "\"lag_rms\": %.4f, \"lag_after_ramp_max\": %.3e, \"switches\": %d}",
         label, status, solve_hz, blend_ms, s.jump_on, s.step_off, s.max_step, s.p99_step,
         s.lag_rms, s.lag_after_ramp_max, s.switches);
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) { std::fprintf(stderr, "usage: %s <extracted.inc>\n", argv[0]); return 2; }
  (void)argv;

  // ---- calibration at G1's solve period (11 ticks = 90.9 Hz): baseline must reproduce
  // the documented 0.79 / 0.31 Nm before any blend case is trusted. ----
  const unsigned seed = 20261124u;
  Schedule cal = make_schedule(11, false, seed);
  double amp = 1.0, dj = 1.0;
  Stats cal_off, cal_on;
  for (int it = 0; it < 8; ++it) {
    cal_off = evaluate(cal, make_raw(cal, amp, 0.0, seed), 0.0);
    if (cal_off.step_off > 1e-9) amp *= kOffTarget / cal_off.step_off;
    cal_off = evaluate(cal, make_raw(cal, amp, 0.0, seed), 0.0);  // amp is linear: off == target now
    cal_on = evaluate(cal, make_raw(cal, amp, dj, seed), 0.0);
    const double excess = cal_on.jump_on - cal_off.step_off;
    if (excess > 1e-9) dj *= (kOnTarget - kOffTarget) / excess;
    cal_on = evaluate(cal, make_raw(cal, amp, dj, seed), 0.0);
    if (std::abs(cal_on.jump_on - kOnTarget) < 0.005 && std::abs(cal_off.step_off - kOffTarget) < 0.005) break;
  }
  cal_off = evaluate(cal, make_raw(cal, amp, 0.0, seed), 0.0);
  cal_on = evaluate(cal, make_raw(cal, amp, dj, seed), 0.0);
  const bool cal_ok = std::abs(cal_on.jump_on - kOnTarget) < 0.02 && std::abs(cal_off.step_off - kOffTarget) < 0.02;

  const Case cases[] = {
      {17, 0.0, false, "58.8 Hz baseline"},          {17, 5.0, false, "58.8 Hz + 5 ms"},
      {11, 0.0, false, "90.9 Hz baseline"},          {11, 5.0, false, "90.9 Hz + 5 ms (G1 shipped)"},
      {10, 0.0, false, "100 Hz baseline"},           {10, 5.0, false, "100 Hz + 5 ms"},
      {5, 0.0, false, "200 Hz baseline"},
      {5, 2.0, false, "200 Hz + 2 ms (Upkie shipped)"},
      {5, 5.0, false, "200 Hz + 5 ms (overlap stress)"},
      {11, 0.0, true, "90.9 Hz jittered baseline"},
      {11, 5.0, true, "90.9 Hz jittered +-2 ms + 5 ms"},
  };

  printf("{\"calibration\": {\"off_target\": %.2f, \"off_measured\": %.4f, \"on_target\": %.2f, "
         "\"on_measured\": %.4f, \"ok\": %s},\n \"cases\": [\n",
         kOffTarget, cal_off.step_off, kOnTarget, cal_on.jump_on, cal_ok ? "true" : "false");
  bool first = true;
  for (const Case& cs : cases) {
    const unsigned sseed = seed + static_cast<unsigned>(cs.solve_ticks) * 7u + (cs.jitter ? 1u : 0u);
    const Schedule sc = make_schedule(cs.solve_ticks, cs.jitter, sseed);
    const auto raw = make_raw(sc, amp, dj, sseed);
    const Stats st = evaluate(sc, raw, cs.blend_ms);
    if (!first) printf(",\n");
    first = false;
    emit(cs.label, cs.blend_ms > 0.0 ? "blend" : "baseline", st, 1000.0 / cs.solve_ticks, cs.blend_ms);
  }
  printf("\n  ]\n}");
  return cal_ok ? 0 : 3;
}
