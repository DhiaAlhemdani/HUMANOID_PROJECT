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

*pending — report section not yet received.*
