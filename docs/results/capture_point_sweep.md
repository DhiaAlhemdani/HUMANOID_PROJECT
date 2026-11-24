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

## Set A — steady walk (`PROFILE=vx` 0.5)

_Pending rescore._ The first run's scoring was overwritten by the set B invocation (the script
truncated the shared report file at start-up — fixed in the same commit as this doc: per-set reports
`capture_sweep_report_{A,B}.txt`). The `cp_*` case directories are intact; rescore without re-running:

```bash
cd ~/Documents/GitHub/humanoid_ws/src/legged_robot_mpc_controller
python3 tests/tuning/compare_ab.py \
  --primary walk.roll_rms_deg:0.85 --primary walk.pelvis_acc_hf:0.92 --primary base.roll_track_err_deg:0.85 \
  --show walk.vy_mean --show walk.yaw_change_deg --show legs.track_err_q_deg \
  tuning_results/cp_base tuning_results/cp_g0 tuning_results/cp_g03 tuning_results/cp_g06 tuning_results/cp_g10 \
  | tee tuning_results/capture_sweep_report_A.txt
```

## Decision (provisional, pending set A)

- `captureFootPlacement` stays **`enabled: false`** on the G1. Set B rules it out as a start/stop
  balance aid regardless of set A: a steady-walk-only win would still have to be gated off around
  command transients, and nothing in the current implementation does that.
- If set A shows a clean steady-walk roll reduction (≤0.85×) with no vy/yaw drift, the follow-up is
  *not* to ship the block but to note it as a candidate for the DCM reference item (2.D), where the
  lateral target comes from the plan rather than the instantaneous velocity.
