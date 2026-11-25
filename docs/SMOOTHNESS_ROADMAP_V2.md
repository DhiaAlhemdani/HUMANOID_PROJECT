# Walking smoothness roadmap v2 — the full command space

Adopted 2026-11-25, superseding the execution order of `LOCOMOTION_IMPROVEMENTS.md` (which stays as
the record of Phases 1–2.D and the gait sweep). Decisions taken with the project owner:

- **Scope:** smoothness across the *full velocity-command space including transitions* — speeds
  through every gait rung (walk / trot / run), turning, lateral and diagonal stepping, start/stop,
  backward, and every gait change the ladder makes along the way.
- **Target:** *visibly smooth to a human observer.* The proxies are pelvis jerk and leg torque
  rate (see §1), scored per command segment; the owner judges replay videos of the same runs.
  Numbers decide whether a change ships; videos decide whether the project is done.
- **Approach:** MPC formulation first — **B. continuous gait adaptation**, then **A. force-rate
  state augmentation** — then **residual RL** on top of the resulting controller.

## 0. Why the previous work cannot get there

Phases 1–2.C took the *steady 0.5 m/s line* from pelvis `acc_hf` 1.18 to 0.44 (−63 %). In the
same logs every *transition* — ramp, stop, gait change — sits at 1.3–1.8, three to four times the
steady value, and nothing shipped moved them (2.A lost, the splice sweep lost, stops transients are
noise-level unchanged). Across varied trajectories the roughness is dominated by two structural
things:

1. **how gaits are chosen and changed** — four discrete templates on a speed ladder, spliced at the
   next contact event; a joystick that wanders across 0.45 m/s flips the whole contact pattern;
   turning and lateral stepping reuse the straight-line templates unchanged;
2. **how contacts are switched** — the plan activates a contact in one node, so ~60 Nm of hip
   torque and the whole weight transfer happen in one tick at every step; the gait sweep showed the
   plan does not spread it even when a double-support window is offered, because nothing charges
   for a force *rate*.

B addresses 1, A addresses 2. Neither is reachable by tuning; both are reference-manager /
formulation changes.

## 1. Step 0 — measure the command space (built)

`tests/tuning/suite/` holds five command scripts (`speeds`, `turns`, `lateral`, `stops`, `omni`;
README there). `run_suite.sh <tag> <config> [baseline]` runs them all through the existing probe
(`PROFILE=script`), `score_suite.py` scores each by segment:

| metric | what | why it is the proxy for "looks smooth" |
|---|---|---|
| `jerk` | pelvis 3-D jerk RMS, 0–15 Hz | the quantity human motion perception is most sensitive to; a rider feels it |
| `acc_hf` / `lat_acc_hf` | pelvis vertical / lateral 4–40 Hz acceleration | the legacy number; impact content |
| `ang_jerk` | pelvis angular acceleration RMS | head/torso wobble |
| `roll_pp`, `pitch_pp` | peak-to-peak tilt | visible lean |
| `tau_rate` | leg measured torque rate RMS | the hardware-side proxy of torque steps; what motors and gearboxes feel |
| `v_err`, `w_err` | command tracking | a smooth robot that does not do what it is told does not count |

Segments: every constant-command hold (after 1.5 s) is **steady**; every ramp plus 2 s is a
**transition** (the ladder's splice lands 0.5–2.5 s after a command change). `compare_suite.py`
aggregates over the whole suite — steady means, transition maxima — with gates:

- primaries: `transition_jerk_max` 0.8×, `transition_acc_hf_max` 0.8×, `steady_jerk_mean` 0.85×,
  `steady_acc_hf_mean` 0.85×;
- guards: no falls, `v_err_max` ≤ 1.2×+0.03, `tau_rate` (steady mean, transition max) ≤ 1.15×.

`render_replay.py <case> out.mp4` replays a run kinematically in MuJoCo (recorded joint angles +
ground-truth base, side camera following the pelvis, command/mode overlay) — `VIDEO=1` in
`run_suite.sh`. The baseline suite of the shipped controller is the first deliverable; its videos
are the owner's reference for "before".

## 2. Step B — continuous gait adaptation

**Replace the four-rung ladder with a continuous gait law** in the reference manager: cadence,
duty factor (double-support fraction), swing height and the single-support asymmetry become
continuous functions of the commanded planar speed and yaw rate (Froude-number based: step
frequency ∝ √(|v|/L) with floor/ceiling, duty factor falling from ~0.65 at a crawl to 0.5 at the
trot threshold and below 0.5 — flight — only above the run threshold). The mode sequence is still
a template (LF / STANCE / RF / STANCE / FLY) but its switching times are regenerated each solve
from the current command, with phase continuity (the running phase is preserved across a parameter
change so no splice is needed) and rate limits on the parameters themselves.

What it removes: the flip of the whole contact pattern at 0.45 and 0.70 m/s; the splice delay and
its transient; the straight-line templates being used for turning (yaw rate shortens the inner
foot's step and should lengthen the support) and lateral stepping (needs a wider stance and a
different swing law).

Where: `ProceduralMpcMotionManager` (ladder, `preSolverRun`), `GaitSchedule` /
`ModeSequenceTemplate` generation, `SwingTrajectoryPlanner` (swing height per phase). Config block
`gait.continuous` with `enabled` (default off → ladder unchanged) and the law's parameters.
Measurement: the suite, especially `speeds` (every rung up and down) and `omni`. Expected: the
transition maxima approach the steady values; steady unchanged.

## 3. Step A — force-rate state augmentation

**Make the contact wrench a state and its rate the input** in the WB-ID OCP: `x = [q, v, W]`,
`u = [Ẇ, a, τ]`, with a quadratic cost on `Ẇ` and the existing wrench constraints (cone, limits,
zero in swing) moved to the state. The plan then cannot step a contact force; the weight transfer
at touchdown is a ramp whose slope the `Ẇ` weight sets, and the swing-leg unloading before lift-off
is planned instead of instantaneous. This is the rate penalty that 1.3's per-stage cost could not
provide (it penalised the force, not its rate, and had no state to carry it across nodes).

Where: `WbIdDynamics` (flow map gains the wrench integrator), `WbIdRneaConstraintCppAd` (reads W
from the state), the wrench cone / friction / swing-zero constraints (state-only now), the
`WbIdMpcTargetTrajectoriesCalculator` (W reference = static load share), the whole-body layer
(`WbIdFeedback` consumes the planned W — unchanged interface), `ModelSettings` dimensions, the
parameter contract tests. Solve-rate cost: +12 states, +12 inputs on 23 joints; to be measured, and
the gate is the existing `solve_hz ≥ 0.9×`.
Measurement: the suite's `tau_rate` and `acc_hf` everywhere, plus the per-event
`plan_tau_step_at_contact_change` (should fall from 18 Nm to a few Nm).

## 4. Step C — residual RL (after A and B)

Prerequisites from `LOCOMOTION_IMPROVEMENTS.md` §3.1 stand: actuator realism and a domain
randomisation harness before any policy is trained. Then a residual on the WB-ID layer's torque (or
on the joint targets), reward = suite proxies (jerk, torque rate) + tracking + upright, trained in a
GPU simulator with the MPC in the loop. Its success criterion is the same suite and the same
videos. Sized after A and B land; not before.

## 5. Order and gates

| step | deliverable | ships when |
|---|---|---|
| 0 | baseline suite + videos of the shipped controller | — (reference) |
| B | `gait.continuous` law, default off; suite A/B | suite WIN (§1 gates), owner's video verdict not worse |
| A | wrench-state OCP, default off; suite A/B | suite WIN, `solve_hz` guard, video verdict |
| C | residual policy | suite WIN on the randomised scene, video verdict |

Every step keeps the default config unchanged until its gate is passed, as before; PR #3 stays
open and accumulates until the owner judges the walk smooth.
