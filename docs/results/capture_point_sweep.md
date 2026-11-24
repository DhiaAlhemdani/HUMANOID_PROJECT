# Phase 2.A — capture-point / Raibert foothold sweep (G1)

Roadmap item: Phase 2 #4 (`docs/LOCOMOTION_IMPROVEMENTS.md`). Tooling: `tests/tuning/run_capture_sweep.sh`
(config-only — `ocs2.costs.captureFootPlacement`, shipped `enabled: false`).

**Question.** Does placing the swing foot at the instantaneous capture point
(`com + v/ω`, plus an optional Raibert velocity term `gain·(v − v_ref)`, tracked by a 50-weight
task-space cost on the swing foot) reduce lateral sway/roll in steady walking and improve balance
around start/stop steps, compared with leaving foot placement entirely to the MPC optimizer?

Candidates share the H2-proven block (`stepWidth 0.24` from the G1 hip offsets, `trackingWeight 50`,
`maxAdjustment 0.10`, `stanceTimeGain 0.5`, `currentSwingOnly true`) and vary only `enabled` / `gain`.

## Set B — start/stop balance (`PROFILE=stops`, 2026-11-24)

Scoring (`compare_ab.py --no-default-guards`): primaries `command.transient_roll_max_deg ≤0.85×`,
`command.transient_pitch_max_deg ≤0.85×`, `walk.pelvis_acc_hf ≤0.92×`; guards vx ≥0.90×, solve ≥0.90×,
roll_rms ≤1.15×+0.05. Windows [−0.3, +2.0] s around each command step.

| case | capture | gain | transient roll max (°) | transient pitch max (°) | acc_hf | transient z_pp max (mm) | vx | solve Hz | verdict |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `cps_base` | off | — | **0.999** | **1.422** | 1.294 | 31.27 | 0.244 | 64.3 | baseline |
| `cps_g0` | on | 0.0 | 1.314 (+32 %) | 1.840 (+29 %) | 1.253 (−3 %) | 25.20 (−19 %) | 0.247 | — | **no gain** — roll/pitch primaries worse than the 1.1× not-worse limit |
| `cps_g06` | on | 0.6 | 1.726 (+73 %) | 1.906 (+34 %) | 1.325 (+2 %) | 25.34 (−19 %) | 0.244 | — | **no gain** — same, monotonically worse with gain |

Result: **0/2.** All candidates survived.

Reading:

- Both capture-point variants make the start/stop transient *worse* in roll and pitch, and the
  degradation grows with the Raibert gain. With `trackingWeight 50` the swing-foot task cost pulls
  the foothold toward a point computed from the *current* CoM velocity, which during a 0.5 m/s²
  decel/accel step is exactly when that estimate differs most from where the optimizer's full-body
  plan wants the foot. The two placement authorities fight; the body absorbs the disagreement as tilt.
- The z-bounce transient does improve (−19 %) and `acc_hf` is within noise — the capture foothold
  does shorten/standardise steps around the transient — but the benchmark priority is balance first,
  and a 30–70 % larger roll excursion at every start/stop is the opposite of that.
- This matches the H2 experience recorded in `config/h2/ros2_controllers.yaml` (weight 20 fell on
  decel, 100 nearly fell at 2 m/s): the heuristic is fragile exactly at speed changes. On the H2 it
  earns its place at running speeds that the G1 benchmark does not reach.

## Set A — steady walk (`PROFILE=vx` 0.5, 2026-11-24)

Scoring (`compare_ab.py`, default guards): primaries `walk.roll_rms_deg ≤0.85×`, `walk.pelvis_acc_hf ≤0.92×`,
`base.roll_track_err_deg ≤0.85×`. Shown: `vy_mean`, `yaw_change_deg`, `legs.track_err_q_deg`.

| case | capture | gain | roll rms (°) | acc_hf | roll track err (°) | vy mean | yaw change (°) | q track err (°) | jump@switch (Nm) | pd_tau_hf | vx | verdict |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `cp_base` | off | — | **0.670** | **1.094** | **0.104** | 0.001 | 0.01 | 0.169 | **0.110** | 2.128 | 0.425 | baseline (solve 67.6 Hz) |
| `cp_g0` | on | 0.0 | 0.765 (+14 %) | 1.056 (−3 %) | 0.108 | −0.003 | −1.29 | 0.193 | 0.157 (+43 %) | — | 0.429 | **guards-fail** |
| `cp_g03` | on | 0.3 | 0.763 (+14 %) | 1.116 (+2 %) | 0.113 | 0.000 | −0.96 | 0.200 | 0.156 (+42 %) | — | 0.427 | **guards-fail** |
| `cp_g06` | on | 0.6 | 0.791 (+18 %) | 1.215 (+11 %) | 0.124 (+19 %) | −0.003 | 0.23 | 0.210 | 0.201 (+83 %) | — | 0.425 | **guards-fail** |
| `cp_g10` | on | 1.0 | 0.816 (+22 %) | 1.328 (+21 %) | 0.124 (+19 %) | −0.001 | −0.65 | 0.217 | 0.161 (+46 %) | 2.561 (+20 %) | 0.423 | **guards-fail** |

Result: **0/4.** All candidates survived; vx unaffected; no lateral/yaw drift (|vy| ≤ 3 mm/s, |Δyaw| ≤ 1.3° over the run).

Reading:

- Roll gets *worse* the moment the capture foothold is on (+14 % at gain 0) and degrades further with
  the Raibert term — the opposite of the hypothesis. The optimizer's free placement was already
  better at lateral balance than the LIP capture point; forcing the foot toward `com + v/ω` on a robot
  whose CoM dynamics are not LIP-like (heavy legs, arm swing, torso compliance) adds a steady
  disturbance that the ankles must then counter.
- `cmd_tau_jump_at_switch` +42…83 %: the swing-foot target now moves *during* swing (it is recomputed
  from the live CoM velocity every solve), so the foot arrives with a different plan than it left
  with and the stance handover is rougher. This is the same mechanism — a reference changing under
  the solver — that Phase 1's `solveBlendSeconds` was introduced to tame.
- `acc_hf` −3 % at gain 0 is noise; at gain ≥ 0.6 the pelvis vibration rises in step with the
  placement authority.

## Decision — Phase 2.A closed (0/6)

- `captureFootPlacement` stays **`enabled: false`** on the G1. Both regimes are negative: steady walk
  (roll +14…22 %, handover jump +42…83 %) and start/stop (transient roll +32…73 %). The block remains
  in the config only as the H2-compatible schema anchor; it is an **established negative** for the
  G1 at ≤ 0.5 m/s and is not to be retried with different weights.
- Carried forward: the G1's MPC already places feet better than a LIP heuristic can, so Phase 2.D
  (DCM/CoM reference) should shape the **base reference** the optimizer tracks, not override its
  foothold choice. Any future placement aid must (a) be gated off around command transients and
  (b) freeze its target once swing begins — the two failure modes measured here.
- Next: 2.B whole-body QP layer (roadmap execution table).
