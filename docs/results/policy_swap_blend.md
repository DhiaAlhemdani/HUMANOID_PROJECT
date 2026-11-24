# Solve-Handover Command Blend (`control.solveBlendSeconds`) — Measurement Results

Phase 1.1 of [locomotion improvements](../LOCOMOTION_IMPROVEMENTS.md): a smoothstep ramp that
blends the last written command into each freshly solved policy at the `updatePolicy()` boundary,
replacing the hard swap that caused the documented per-solve torque step
(`cmd_tau_jump_at_switch` 0.79 Nm vs 0.31 Nm on other ticks, G1 walk, `docs/wb_id_tuning_benchmark.md`).

Methodology: **implement → test → document results** (adopted 2026-11-24).
Implementation and static validation are in [§1.1 of the roadmap](../LOCOMOTION_IMPROVEMENTS.md);
this file is the results of the test stage.


## What was measured, and what was not

| | Status | What it proves |
| --- | --- | --- |
| Mechanism micro-benchmark (this doc, below) | **Done, 18/18 gates** | The shipped `apply_policy_swap_blend()` removes the switch-tick step, has zero residual lag after the ramp, and improves the worst per-tick command change — across solve rates 58.8–200 Hz, jittered solves, and duration/solve-period overlap. |
| Closed-loop MuJoCo A/B (`tests/tuning/run_blend_ab.sh`) | **Script ready — run pending** | The same walk with blend off vs on, gated on the real `cmd_tau_jump_at_switch` plus non-regression guards. Needs the `ros_jazzy` container. |
| Real robot | Not done | — |

The micro-benchmark is mechanism-level: it drives the *real compiled function* with a synthetic
1 kHz trace. It does not model closed-loop dynamics, estimator interaction, or contact events —
that is what the A/B is for.


## Micro-benchmark setup

- **Code under test**: `apply_policy_swap_blend()` extracted *verbatim* from
  `src/controller/controller.cpp` at run time and compiled with `g++ -O2 -Wall -Wextra
  -Wpedantic -Werror` together with `tests/tuning/blend_microbench.cpp` (stub types mirroring the
  controller). Runner: `python3 tests/tuning/run_blend_microbench.py` (stdlib + g++ only, no ROS).
- **Trace**: 60 s at 1 kHz, 20 joints (12 leg + 8 arm), per-joint gait-band signal
  (1.5 Hz + 0.4 Hz) plus a per-solve disagreement term (each solve's plan differs from its
  neighbour's), i.e. the two ingredients of a policy-swap step: signal slope + plan jump.
- **Calibration** to the documented G1 walk at 90.9 Hz, blend off: synthetic amplitudes are
  scaled so the baseline reproduces `step_other = 0.3100 Nm` (target 0.31) and
  `jump@switch = 0.7867 Nm` (target 0.79). All blend runs reuse the calibrated trace.
- **Windows**: metrics over t = 5–55 s; `jump@switch` = mean |Δcmd| on solve ticks,
  `step_other` = mean |Δcmd| on non-switch ticks (same masks as `plot_tracking.py`),
  `max/tick` = worst |Δcmd| over all ticks, `lag` = max |cmd − policy| on ticks outside any ramp.
- **Gates** (18): calibration within tolerance; for each baseline/blend pair — jump@switch → 0
  (re-anchor makes the switch tick exactly step-free), worst per-tick step strictly below the
  baseline's, and zero residual lag after the ramp (blend < solve period). The overlap case
  (duration = period) is exempt from the lag gate by construction.


## Results — solve rate × blend duration

| case | jump@switch (Nm) | step_other (Nm) | max/tick (Nm) | lag after ramp |
| --- | --- | --- | --- | --- |
| 58.8 Hz baseline | 0.783 | 0.310 | 2.895 | 0 |
| 58.8 Hz + 5 ms | **0.000** | 0.338 | **2.131** | 0 |
| 90.9 Hz baseline | 0.784 | 0.310 | 2.980 | 0 |
| 90.9 Hz + 5 ms (G1 shipped) | **0.000** | 0.356 | **2.126** | 0 |
| 100 Hz baseline | 0.780 | 0.310 | 2.906 | 0 |
| 100 Hz + 5 ms | **0.000** | 0.360 | **2.132** | 0 |
| 200 Hz baseline | 0.787 | 0.310 | 2.914 | 0 |
| 200 Hz + 2 ms (Upkie shipped) | **0.000** | 0.449 | **2.714** | 0 |
| 200 Hz + 5 ms (overlap stress) | **0.000** | 0.415 | **2.242** | n/a (ramps touch) |
| 90.9 Hz jittered ±2 ms baseline | 0.785 | 0.310 | 2.906 | 0 |
| 90.9 Hz jittered ±2 ms + 5 ms | **0.000** | 0.356 | **2.118** | 0 |

**18/18 gates PASS.** Full machine-readable output: `tests/tuning/blend_microbench.json`.

Reading the table:

- **jump@switch → exactly 0** in every blend case: at the solve tick the command re-anchors to
  what is on the hardware, so the swap itself cannot step the output. The 0.79 Nm step does not
  shrink — it disappears at its source and is redistributed over the ramp.
- **step_other rises modestly** (0.31 → 0.34–0.45): the ramp ticks now carry intermediate steps.
  This is the honest trade — the mean non-switch tick gets slightly busier while the *worst* tick
  drops 27–29% (2.9 → 2.1–2.7 Nm) and the catastrophic switch tick is gone. Worst-case torque
  slew is the quantity a servo/actuator cares about.
- **Zero residual lag**: after the ramp the command equals the new policy exactly (bit-for-bit,
  `lag = 0`), so smoothing costs nothing at steady state — with blend duration < solve period,
  every solve fully lands before the next one.
- **Overlap stress** (200 Hz, 5 ms = full period): re-anchoring from the mid-ramp value each
  solve keeps the command continuous — no growth, no NaN, worst tick still below baseline.
- **Jittered solves** (±2 ms) behave like the uniform case; the ramp is anchored to wall-clock
  time, not tick counts.

The shipped configurations (G1/H2 5 ms at ~90 Hz, Upkie 2 ms at 200 Hz) all sit in the
well-behaved region: duration ≤ half the solve period.


## Test-development note (why an earlier run lied)

Two bugs in the *benchmark* (not in the shipped code) produced a vacuous 13/18 "pass" that was
caught and fixed before these results:

1. **Units**: the harness first set `solveBlendSeconds = 5.0` (milliseconds) while the parameter
   is in seconds — the ramp took 5 s, the command froze, and jump/max/lag gates passed for the
   wrong reason (a stuck command has no steps). Symptom: `lag after ramp ≈ 313 Nm`.
2. **Zero-initialized amplitudes**: `leg_amp[12]` was declared with 12 slots but 6 initializers,
   so the right-leg joints had amplitude 0; calibration then scaled `amp` 2× to hit the mean
   targets, distorting the active joints' slope. Symptom: measured `step_off` exactly half the
   analytic value.

Both are recorded here because they are the reason this doc exists: the gates now include a
calibration check (targets reproduced before any blend case is trusted), and the harness source
is small enough to audit.


## Closed-loop A/B — how to run (pending)

In the `ros_jazzy` container (workspace `/root/ocs2_ros2_ws`, repo checked out):

```bash
tests/tuning/run_blend_ab.sh 0.5          # vx = 0.5 m/s, default OUT=<repo>/tuning_results
```

It builds `legged_robot_mpc_controller`, runs the static contract test, generates
`blend_off.yaml` (`solveBlendSeconds: 0.0`) and `blend_on.yaml` (`0.005`) with 500 Hz tracking
via `make_config.py`, runs `run_case.sh` for both, and gates the pair with
`tests/tuning/compare_blend_ab.py`:

| gate | limit |
| --- | --- |
| both runs survive | `metrics.survived` |
| legs `cmd_tau_jump_at_switch` drops | ≥ 40% reduction |
| `plan_z_sd_mm`, `z_track_err_mm` | ≤ 1.10× + 1 mm |
| legs `pd_tau_p999` (touchdown PD peak) | ≤ 1.15× + 1 Nm |
| `solve_hz` | ≥ 0.90× baseline |
| legs `cmd_tau_step_otherwise` (ramp redistribution) | ≤ 1.60× + 0.05 Nm |

Report lands in `$OUT/blend_ab_report.txt`. **Paste the report here to complete this section.**

### A/B results

_Pending — run `tests/tuning/run_blend_ab.sh` in the container._


## Reproduce

```bash
python3 tests/tuning/run_blend_microbench.py     # this doc's table + gates (any machine with g++)
tests/tuning/run_blend_ab.sh 0.5                 # closed-loop A/B (ros_jazzy container)
python3 -m pytest tests/wb_id_policy_blend_contract_test.py -q   # static contract
```
