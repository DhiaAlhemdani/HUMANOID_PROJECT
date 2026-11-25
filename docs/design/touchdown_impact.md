# Phase 2.C — touchdown impact model in the MPC (jump map at contact events)

Status: **jump map deferred; superseded for now by a timing finding** (2026-11-25, §7). Roadmap
Phase 2 #3, re-sequenced to 2.C. Brief sharpened by 2.B: the whole remaining torque transient of the
shipped controller sits at the touchdown event (hip-pitch |max| 39 Nm vs 33 baseline, the residual
6 Nm leg HF; `docs/results/wbc_s1_sweep.md`), and the planned torque itself carries ~30 Nm there.

> **Read §7 first.** The baseline measurements taken for this design (new `touchdown.*` block in
> `plot_tracking.py`) show the robot lands **14–20 ms before the plan's contact event at every
> step**, because the shipped swing planner aims the touchdown 1 mm below the ground
> (`swingTrajectory.touchDownHeightOffset: -0.001`, inherited from upstream, never swept). An
> impulse model at the planned event cannot represent a contact that has already happened; the
> timing is fixed first, by config (`tests/tuning/run_touchdown_sweep.sh`), and the jump map is
> re-evaluated on the corrected baseline.

## 0. What the plan assumes at touchdown today (code facts)

| fact | where |
| --- | --- |
| State `x = [q, v]`, input `u = [W, a, τ]`; flow map is a double integrator, rigid-body dynamics are the RNEA equality | `src/mpc/wb_id/dynamics/WbIdDynamics.cpp` (`SystemDynamicsBase`, analytic constant Jacobians), `WbIdRneaConstraintCppAd` |
| **No jump map**: `WbIdDynamics` inherits `SystemDynamicsBase::computeJumpMap` = identity → planned velocity is continuous across every contact switch | `WbIdDynamics.h` |
| Event nodes already exist in every solve: `SqpSolver` discretises with `timeDiscretizationWithEvents(initTime, finalTime, dt, modeSchedule.eventTimes)` and calls `multiple_shooting::setupEventNode`, which uses `dynamicsPtr->jumpMapLinearApproximation(t, x)` as the node's equality `x_{k+1} = g(x_k)` | `ocs2_ros2/mpc/ocs2_sqp/.../SqpSolver.cpp:197–199, 381–383`; `core/ocs2_oc/src/multiple_shooting/Transcription.cpp:165–182` (verified in `wei-hsuan-cheng/ocs2_ros2` main) |
| Swing foot ends at `touchDownVelocity −0.05 m/s` (Phase-1 winner) via the 1-row vertical swing constraint; horizontal foot velocity at touchdown is **unconstrained** | `WbIdFootAccelerationConstraintCppAd` (SwingVertical), `config/g1` `swingTrajectory.touchDownVelocity` |
| Post-event node: stance constraint `Ka·a + Kd·v + Kp·(z − z_ref) = 0` with `Kd = 50 /s`, i.e. the plan decelerates the foot's residual velocity over ~20 ms through planned accelerations → planned GRF/torque transient (the ~30 Nm `plan_tau` seen at the switch) | `footMotionEquality.*` gains, `config/g1:495–507` |
| At 0.5 m/s the gait is **trot** (`[LF, RF]`, no double support, `phaseTransitionStanceTime 0`): touchdown of one foot and lift-off of the other are the *same* event | `config/g1:751`, sweep-2 spike anatomy |
| `SystemDynamicsBaseAD` offers `systemJumpMap(time, state, parameters)` + `getJumpMapParameters(time, preComp)` with its own CppAD model; `HumanoidPreComputation` does not override `requestPreJump` (nothing needed) | `ocs2_core/dynamics/SystemDynamicsBaseAD.h`, `HumanoidPreComputation.h` |
| `pinocchio::crba` is already instantiated for `ad_scalar_t` in this build (`computeJointTorques<ad_scalar_t>`) → the mass matrix is available on the AD tape | `src/mpc/common/model/DynamicsHelperFunctions.cpp:292` |

So the solver infrastructure for a hybrid jump is in place and running (with an identity jump);
the item is the jump map itself and its derivative.

## 1. Why it should help, and the honest counter-case

The measured robot loses vertical base velocity at impact; the plan does not. The MPC then sees a
state that its own prediction did not produce, every step, at the worst possible moment (new
support leg, full load transfer in trot). Everything downstream — the planned post-touchdown foot
deceleration, the feedback torque spike the WBC fade now hides, the `base_vz` mismatch — is the
plan being surprised by an event it schedules itself. A rigid inelastic impact at the event node
makes the predicted and measured post-touchdown states agree to first order, and lets the
optimiser *shape* the pre-impact state (leg configuration, base velocity, horizontal foot velocity)
because the impulse is now inside the dynamics it differentiates.

Counter-case, stated up front: MuJoCo's contact is soft; the real impact is spread over several
ms and the foot arrives at a measured speed that is not exactly −0.05 m/s. A rigid impulse of the
full magnitude may over-predict the velocity loss. That is why the model carries an
`impulseFraction ∈ [0, 1]` (0 = today, 1 = rigid inelastic) and the sweep runs both 1.0 and 0.5.
The new `touchdown.*` metrics (§4) measure the mismatch directly, so the result will say which
fraction matches the simulator rather than which one "feels" right.

## 2. Formulation

At an event time `t_e` with contact flags `s⁻ → s⁺`, the landing set is `L = {f : s⁺_f ∧ ¬s⁻_f}`.
With `J_L` the stacked 6-row LOCAL_WORLD_ALIGNED contact Jacobians of the landing feet (same rows as
the stance constraint; sole lands flat) and `M(q)` from `crba`:

```
Λ  = (J_L M⁻¹ J_Lᵀ + ε I)⁻¹ J_L v⁻           # rigid, inelastic (restitution 0)
v⁺ = v⁻ − κ · M⁻¹ J_Lᵀ Λ                    # κ = impulseFraction
q⁺ = q⁻
```

Lift-off-only events (`L = ∅`) keep the identity. Two feet landing at once (`FLY → STANCE`) does not
occur in any configured gait; handled per foot independently (no cross-coupling), documented as
such. `ε = 1e-6` guards a straight-knee Delassus matrix.

Derivative: the whole map is one CppAD function of `(x, parameters)` with parameters
`[flag_LF, flag_RF, κ]`; per-foot terms are multiplied by their flag so one generated model serves
every event. Both per-foot Delassus matrices are always invertible (each leg is non-singular at the
planned touchdown), so the flag can be a multiplier rather than a structural switch. `M⁻¹` via
`pinocchio::computeMinverse` (branch-free, AD-safe; symmetrise the upper triangle), the 6×6 solves
via `Eigen::LLT` (no pivoting → tape-safe). Jacobian is dense 58×58 at ≤3 event nodes per solve —
negligible against the RNEA constraint already on the tape at every node.

Interaction with the post-event stance constraint: with `κ = 1`, `J_L v⁺ = 0` exactly, so the
post-event node's `Kd·v` term is zero and the planned acceleration there is only the height/tilt
correction — the planned GRF transient disappears from the plan.

## 3. Implementation plan (S1-sized, no new dependency)

1. `WbIdDynamics` → `WbIdDynamicsAD : SystemDynamicsBaseAD` (flow map unchanged, now on the tape;
   it is linear, so the AD Jacobians are exact and constant). `systemJumpMap` as §2;
   `getJumpMapParameters(t)` reads the mode schedule through the reference manager:
   `getContactFlags(t − δ)` / `getContactFlags(t + δ)` with `δ = 1e-4 s` → flags, plus κ.
   Generated with the other models (`modelFolderCppAd`, `recompileLibrariesCppAd`).
2. Config: `ocs2.dynamics.touchdownImpact: {activate: false, impulseFraction: 1.0,
   regularisation: 1e-6}` in `parameters.yaml` + G1 (inert) → `WbIdMpcInterface::Config` →
   `config_builder.cpp`, following `torsoTiltCost`. `activate: false` keeps the analytic
   `WbIdDynamics` (zero behavioural change; the AD class is only constructed when on).
3. Tests: Eigen/pinocchio-free unit test of the impact algebra on a 2-DoF toy (impulse removes the
   constrained velocity, momentum conservation in the free directions, κ = 0 identity); contract
   test for schema/config/plumbing. The CppAD model is exercised by the first native run.
4. Metrics: `touchdown.*` block in `plot_tracking.py` (**already added**, back-fillable from any
   `data.npz`): per planned touchdown in the walk window, over the first 60 ms —
   `base_vz_mismatch` (rms obs−plan base vz), `base_vz_drop` (measured loss), `foot_vz_obs_pre`
   vs `foot_vz_plan_pre` (real vs planned landing speed), `leg_tau_peak`,
   `leg_cmd_minus_plan_peak` (what the servo/WBC adds at impact).

Not in scope: contact *timing* changes (2.F), restitution > 0, impact on the lift-off foot.

## 4. Sweep (declared here; gates fixed before the run)

Baseline: make_config'd shipped config (WBC on, fade 0.03). Set A `vx` 0.5 (trot; one event per
step): `td_base`, `td_full` (κ 1.0), `td_half` (κ 0.5), `td_fade0` (κ 1.0 with
`contactBlendSeconds 0` — does the plan-side model make the servo-side fade unnecessary?).
Set B `stops`: `tds_base`, `tds_full`.

Primaries (any one ≤ bar wins, others not-worse 1.1×): `touchdown.base_vz_mismatch ≤ 0.80×`,
`legs.meas_tau_hf ≤ 0.90×`, `walk.pelvis_acc_hf ≤ 0.92×`, `touchdown.leg_cmd_minus_plan_peak
≤ 0.80×`. Guards: the sweep-2/3 set (`roll/pitch_rms`, `cmd_tau_jump_at_switch`, `track_err_q`,
`z_sd`, `z_track`, `arm_swing`, `vx ≥ 0.90×`, **`solver.solve_hz ≥ 0.90×`** — the AD jump map
must not slow the solve) plus `base.z_track_err_mm ≤ 1.10×+0.5`. Set B: transient pitch/roll
not-worse 1.1×, `z_pp ≤ 1.15×+2`. Falls disqualify.

What would close the item as a negative: `base_vz_mismatch` not improved at κ = 1 (the simulator's
impact is not impulsive at this landing speed) or `solve_hz` below guard (AD cost).

## 5. Risks

- **CppAD model generation**: new tape with `crba` + `computeMinverse` + two frame Jacobians;
  generation adds to the first-start compile. Mitigated by `activate: false` default (class not
  built) and the existing JIT cache.
- **AD-unsafe branches**: `LLT` is pivot-free; `computeMinverse` is ABA-structured; no `if` on
  tape values. `crba` AD is proven in this build.
- **Mode-schedule lookup at the event**: `getContactFlags(t ± δ)` must straddle the event exactly;
  the same lookup drives `isActive` of the stance constraints, so pre/post consistency is by
  construction.
- **Solver conditioning**: the jump map makes the event node's equality nonlinear in `q` (Jacobian
  via `J(q)`, `M(q)`); SQP handles it, but a bad κ could add iterations — covered by `solve_hz`.
- **Observer**: the measured post-impact state already includes the impulse; the plan's initial
  condition is unaffected.

## 6. Decision needed

Go/no-go on the implementation above (≈300 lines: AD dynamics class, config plumbing, toy test,
sweep script). Before that, two numbers from data you already have would anchor the baseline —
the new `touchdown` block is back-fillable:

```bash
cd ~/Documents/GitHub/humanoid_ws/src/legged_robot_mpc_controller
git pull --no-rebase origin arena/47d87774-humanoid-project
for c in wb3_base wb3_fade30; do python3 tests/tuning/plot_tracking.py tuning_results/$c >/dev/null 2>&1; \
  python3 -c "import json;d=json.load(open('tuning_results/$c/diag.json'));print('$c',json.dumps(d['touchdown']))"; done
```

If `base_vz_mismatch` and `foot_vz_obs_pre` (measured landing speed vs the planned −0.05) are
both small, the plan-side impact is a small target and 2.C should be demoted behind 2.D; if the
landing speed is well above 0.05 m/s or the vz mismatch is a sizeable fraction of the planned vz
rms (`base.plan_vz_rms`), the item is worth its ~300 lines.

## 7. Baseline measurement (2026-11-25) — the plan's touchdown is 14–20 ms late, by construction

`touchdown.*` on the sweep-3 recordings (34 touchdowns per `vx` run, 26 per `stops` run):

| case | early_ms | plan vz at the measured stop | obs−plan foot z, 30 ms before stop | rest height obs / plan [mm] | base vz mismatch (60 ms) / plan vz rms |
| --- | --- | --- | --- | --- | --- |
| `wb3_base` (PD) | **+20** | −0.075 m/s | +0.28 mm | 0.13 / 0.43 | 0.017 / 0.030 (57 %) |
| `wb3_fade30` (shipped) | **+20** | −0.081 m/s | +0.18 mm | −0.28 / −0.15 | 0.012 / 0.029 (42 %) |
| `wbs3_base` (walk gait) | **+14** | −0.067 m/s | +0.24 mm | 0.01 / 0.02 | 0.024 |

What this rules out: a resting-height offset (the foot rests where the plan rests, ±0.3 mm) and a
descent tracking lag (the foot is slightly *above* the plan during the descent). What remains:
`SwingTrajectoryPlanner::update` sets the touchdown node at `terrainHeight + touchDownHeightOffset`
= −1 mm (`SwingTrajectoryPlanner.cpp:91,143`), so the planned foot crosses the real ground while it
still has ~1 mm and 13–20 ms of planned descent left (1 mm at the 0.05–0.075 m/s terminal approach;
the walk's 0.75× velocity scaling gives its 14 ms). During those milliseconds the plan constrains the
loaded foot to **zero wrench** (`swingFootZeroWrench`) and the other leg to carry everything; the
servo drives the planned joints a further millimetre into the ground; the base decelerates against a
foot the optimiser is not allowed to use; at `t_e` the load transfers in one node. That is the
measured base-vz mismatch and, plausibly, much of the switch-tick transient the 2.B fade hides.

The item therefore splits:

1. **2.C-timing (config, now):** `touchDownHeightOffset` ∈ {0.0, +0.001, −0.0005} vs shipped −0.001,
   with the shipped controller otherwise — `tests/tuning/run_touchdown_sweep.sh`. Primaries
   `touchdown.abs_early_ms ≤ 0.4×` (|20| → ≤ 8 ms; a *late* landing must not pass as fixed — hence
   the absolute value), `base_vz_mismatch ≤ 0.80×`, `acc_hf ≤ 0.92×`, `meas_tau_hf ≤ 0.90×`; guards
   the sweep-3 set plus foot z tracking ≤ 1.2×+0.3 mm. The expected failure mode of a positive offset
   is the mirror image (plan loads a foot still 1 mm in the air) — the ladder shows which side the
   simulator prefers. Set B checks the stops.
2. **2.C-impact (the jump map above):** re-decided on the corrected baseline. If `base_vz_mismatch`
   drops to a small fraction of the planned vz rms once the timing is right, the impulse is not
   worth 300 lines and the item closes; if a sizeable mismatch remains *with* `abs_early_ms` small,
   it is a genuine impact and the design in §1–§5 applies unchanged.

Sweep-1 outcome (2026-11-25, `docs/results/touchdown_timing_sweep.md`): offset 0/+1 mm win the
steady walk on acc_hf (0.84× / 0.75×) but fail the stops' not-worse rule on the post-touchdown vz
mismatch; the offset accounts for only ~a third of the early landing (21 → 10 ms across −1…+1 mm)
and the mismatch is insensitive to timing — so the residual earliness is a terminal-approach
velocity lag, and the jump map remains deferred.
Correction after the trace (`tests/tuning/touchdown_trace.py`, same recordings): there is no
tracking lag — the robot follows the MPC's foot plan to ≤ 0.2 mm down to 2.6 mm above the ground.
The *plan itself* crosses the ground 41 ms before its contact event at 0.24 m/s and continues to
−2.2 mm (it would dig in if the ground were not there); the ground stops the foot early, at
≈ 0.3 m/s instead of the planned 0.05 m/s. Candidate mechanism: the swing-foot acceleration
equality `a_z = a_ref − kd (v − v_ref) − kp (z − z_ref)` (`constraints.footMotionEquality`,
kp = kd = 50) has error poles −1 / −49 rad/s, so a height sag accumulated from zero-order-held
node accelerations on a decelerating spline is never corrected within a swing. 2.C-timing sweep 2
(`run_touchdown_sweep2.sh`, kp 200/400/625) tests it; a real impact model only becomes relevant
once the plan and the ground agree on where the foot lands.

Sweep 2 confirmed the mechanism (kp 200/400/625: landing 8/6/4 ms early at 0.19 m/s, acc_hf −26 to
−33 %, no more dive; 400 regresses the stops through the shared stance gain). Shipped as a new
swing-only `swingPositionErrorGain_z: 200` with the stance gain untouched (sweep 3 confirms the
split). What remains at touchdown is the reference spline's own terminal speed (≈ 0.19 m/s at
2 mm, i.e. `touchDownVelocity` and the swing duration), which is a planner lever, not an impact
model: §1–§5 stay deferred until a run shows a sizeable post-touchdown mismatch with the landing on
time — and `base_vz_mismatch` itself needs a less noisy replacement first (23–46 % between
same-config runs).
