# Phase 1.6 — command & upper-body polish sweep

- **Date**: 2026-11-24
- **Branch**: `arena/47d87774-humanoid-project` (implementation `9397d8e`/`77f82f6`)
- **Levers**: (1) S-curve command slew — `reference.velocityCommandJerkLimits` (new, shipped
  `[0,0,0,0]`); (2) `solver.sqp.useFeedbackPolicy: true` (LQR gains between solves); arm swing not
  swept (phase-locked by construction, no smoothness defect attached).
- **Tooling**: `run_polish_sweep.sh` → `compare_ab.py`; transient windows from the `command`
  section of `plot_tracking.py` ([−0.3, +2.0] s around each probe ramp/step, worst event).

## Set B — command steps (`PROFILE=stops`, 0 ↔ 0.5 m/s every 4 s / 3 s)

Primaries (any-of, ≤ 0.85×): worst-step pitch / HF acc / height excursion. Guards: survived,
vx ≥ 0.90×, solve ≥ 0.90×, hold acc_hf ≤ 1.10×, roll ≤ 1.15×+0.05.

| run | jerk [m/s³] | step pitch max [°] | step acc_hf | step z pp [mm] | step roll max [°] | hold acc_hf | hold vx | verdict |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| **ps_base** | 0 (clamp) | **1.224** | **1.727** | **26.8** | **1.05** | 1.430 | 0.241 | — |
| ps_jerk1 | 1.0 | 1.357 (+11 %) | 1.657 (−4 %) | 27.9 (+4 %) | 1.05 | **1.299 (−9 %)** | 0.244 | no gain |
| ps_jerk05 | 0.5 | 1.117 (−9 %) | 1.620 (−6 %) | 25.5 (−5 %) | 1.42 (**+35 %**) | **1.283 (−10 %)** | 0.251 | no gain |

All survived. Baseline solve rate in this profile 56 Hz (vs 67 Hz steady walk — the stance↔walk
switches cost solver time); candidates within guard.

### Reading

1. **No primary clears 0.85×.** The worst-step transients move by single digits in both
   directions (pitch +11 % at jerk 1, −9 % at jerk 0.5; z pp ±5 %), i.e. inside the run-to-run
   spread for a *max-over-events* statistic. The S-curve does not remove the pelvis kick at a
   command step — that kick is the gait starting/stopping (stance↔walk schedule change), not the
   reference's acceleration discontinuity.
2. **Secondary, consistent signal**: hold-window `acc_hf` fell **−9 % / −10 %** on both
   candidates (1.430 → 1.299 / 1.283). In this profile the hold window contains six
   starts/stops, so this is the integrated effect of longer, softer ramps (jerk 1.0: 1.0 → 1.5 s
   per step; jerk 0.5: → 2.0 s). Same sign and magnitude on two independent runs, and above the
   ~3–4 % same-config spread seen for `acc_hf` elsewhere — plausible, but it was a *guard* here,
   not a pre-registered primary, and the gain comes with a slower command response.
3. **jerk 0.5 lurches laterally** at one step (roll max +35 %) — a 2 s ramp holds the robot
   in the slow-walk band (0.02–0.45 m/s) longer at each start, where the gait is least regular.
   Not acceptable under the smoothness priority.

### Decision (set B)

- **Not shipped** on this evidence. If a softer command response is wanted for teleoperation
  (joystick steps), `velocityCommandJerkLimits: [1.0, 1.0, 0.0, 4.0]` is safe (all guards pass,
  −9 % hold acc_hf in a start/stop-heavy profile, no roll penalty) and is the value to choose;
  for the benchmark's steady-walk metrics it is inert by construction (set A).

## Set A — steady walk + probe ramps (`PROFILE=vx`, 0.5 m/s)

Primaries (any-of): `legs.cmd_tau_jump_at_switch` ≤ 0.85× / hold `acc_hf` ≤ 0.92× / worst ramp
pitch ≤ 0.85×; default guard set.

| run | config | jump@switch [Nm] | hold acc_hf | ramp pitch max [°] | ramp z pp [mm] | leg PD HF | vx | solve [Hz] | verdict |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| **po_base** | shipped | **0.137** | **1.096** | 1.71 | 27.3 | 2.15 | 0.425 | 60.5 | — |
| po_fb | `useFeedbackPolicy: true` | 0.269 (**+96 %**) | 1.162 (+6 %) | **1.08 (−37 %)** | 22.6 (−17 %) | 2.14 | 0.422 | **49.7 (−18 %)** | guards-fail |
| po_jerk1 | jerk `[1,1,0,4]` | 0.152 (+11 %) | 1.104 (+1 %) | 1.87 (+10 %) | 26.0 | 2.12 | 0.424 | 60.5 | no gain |
| po_fb_jerk1 | both | 0.336 (**+145 %**) | 1.186 (+8 %) | 1.12 (−34 %) | 41.1 (+50 %) | 2.12 | 0.421 | 50.8 (−16 %) | guards-fail |

All survived.

### Reading

1. **`useFeedbackPolicy: true` is rejected.** It does what an LQR term promises *within* one
   solution — the probe-ramp pitch transient drops 37 % — but it **doubles the per-solve command
   jump** (0.137 → 0.269 Nm; with the jerk limiter 0.336): every new solution brings a new gain
   `K(t)` and a new nominal `x_plan(t)`, so the feedback term `K·(x − x_plan)` is discontinuous at
   exactly the handover the Phase 1.1 blend was built to smooth (the blend interpolates the
   feed-forward and the joint targets, not `K`). It also raises hold `acc_hf` 6–8 % and costs
   **16–18 % of the solve rate** (Riccati gains per solve). Two of three benchmark priorities
   worse for one transient gain → no.
2. **S-curve slew is inert in steady walk**, as designed (jump +11 % and pitch +10 % are inside
   the run spread; the probe's own 3 s linear ramp is far below the 0.5 m/s² clamp, so the
   limiter never engages). Combined with set B: no benchmark gain anywhere → shipped value stays
   `[0,0,0,0]`.
3. The baseline solve rate was 60.5 Hz in this batch (67 Hz in the earlier ones) — machine load
   varies between batches; only within-batch ratios are compared, which is why every sweep runs
   its own baseline.

## Decisions — Phase 1.6 closed

- Nothing shipped: `useFeedbackPolicy` stays **false**, `velocityCommandJerkLimits` stays
  **`[0,0,0,0]`** (the limiter remains available; `[1.0, 1.0, 0.0, 4.0]` is the tested-safe value
  for a softer teleop response).
- **Lead for later** (outside Phase 1): the feedback policy's within-solve benefit is real; to
  use it, the policy-swap blend would have to interpolate the feedback term too (blend `K` and
  `x_plan` across the handover, or fade `K` in after each swap). That is a controller change
  with a clear hypothesis and the measurement (`cmd_tau_jump_at_switch`) already in place.
