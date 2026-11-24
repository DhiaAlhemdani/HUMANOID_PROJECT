# Locomotion Quality Improvements (Smoothness, Gait, Learning)

Scope: **control/algorithm** improvements — walking smoothness, naturalness, robustness, and
learning-based methods. (Repo engineering hygiene is tracked separately in `IMPROVEMENTS.md`.)

Grounding: this plan starts from the measured gaps in the
[tuning benchmark](wb_id_tuning_benchmark.md) — the shipped G1 config (`d_kn10`) already
passes "no falls", but three smoothness defects remain:

| # | Measured defect (G1, 0.5 m/s walk) | Evidence |
|---|---|---|
| D1 | **MPC-planned pelvis bounce** ≈ 11–12 mm (z sd), 4–40 Hz vertical accel 1.19 | benchmark §"What remains" |
| D2 | **Touchdown torque spikes** ≈ 50 Nm (knee/ankle 99.9th pct) | benchmark; `impactProximityFactor*` |
| D3 | **Per-solve command ripple** — torque jumps at each new solution: legs 0.79 vs 0.31 Nm on other ticks | `cmd_tau_jump_at_switch` in `plot_tracking.py` |

User priority order (from the benchmark): **falls → smoothness (bounce, jitter) → velocity tracking.**

The measurement harness already exists (`tests/tuning/`: `run_case.sh`, `plot_tracking.py`
SUMMARY/DIAG metrics) — every item below is designed to be A/B-tested with it.

---

## Phase 1 — Smoothness with the existing MPC (highest ROI)

These need no new architecture; most are config or small controller changes.

### 1.1 Kill the per-solve ripple (D3) — *benchmark's own #1 next step*
- **Policy blend on solve handover** ✅ *implemented*: `control.solveBlendSeconds`
  (G1/H2 5 ms, Upkie 2 ms) ramps position, velocity and feedforward from the last
  written command whenever `updatePolicy()` swaps in a new solution
  (`apply_policy_swap_blend()` in `controller.cpp`; contract test:
  `tests/wb_id_policy_blend_contract_test.py`). Validate the ripple reduction with
  `cmd_tau_jump_at_switch` from `tests/tuning/plot_tracking.py` before/after.

  **Methodology (implement → test → document results), status:**

  | step | status | artifact |
  | --- | --- | --- |
  | implement | ✅ done | `controller.cpp` / `parameters.yaml` / configs / contract test |
  | test (mechanism) | ✅ done, **18/18 gates** | `python3 tests/tuning/run_blend_microbench.py` → `tests/tuning/blend_microbench.json`; results in [`docs/results/policy_swap_blend.md`](results/policy_swap_blend.md) |
  | test (closed-loop A/B) | ✅ done, **8/8 gates** (2026-11-24) | `tests/tuning/run_blend_ab.sh` → jump 0.924 → 0.182 Nm (−80%), no guard regressed; report in [`docs/results/policy_swap_blend.md`](results/policy_swap_blend.md) |
  | document results | ✅ done | [`docs/results/policy_swap_blend.md`](results/policy_swap_blend.md) (micro-bench 18/18 + closed-loop A/B 8/8) |

  Phase 1.1 measurement is complete (both test stages ran and are documented); 1.2+ is
  unblocked under the implement → test → document methodology.
- **Raise the effective MPC rate**: `f_thr6_120` reached 104 Hz with `sqpIteration: 1`.
  Options: trim horizon nodes / `fineHorizon`, profile CppAD eval, spend saved time on rate
  rather than iterations (benchmark already concluded rate > iterations).
- **Plan consistency**: warm-start each solve with the previous solution shifted in time
  (OCS2 supports warm input seeds) so successive solutions disagree less — directly reduces
  `cmd_tau_jump_at_switch`.

### 1.2 Reduce planned bounce (D1)
- **Vertical cost shaping**: the shipped `q.baseVelocity[2]` (vz) is 10 and benchmark round 6
  showed vz 30 moves the motion into arms/roll — so raise the **base-height tracking cost**
  (`q.basePose[2]`) and the **vertical acceleration** entry of `r.baseAccelerations`
  instead of vz alone; sweep with `make_config.py`.
- **Smooth the height reference**: `defaultBaseHeight` steps are already rate-limited
  (`velocityCommandRateLimits`), but the *gait-phase height reference* itself can add bounce —
  check `SwitchedModelReferenceManager` height reference vs. swing phase and consider a
  phase-synchronized sinusoid (RAIBERT-style) or low-pass the reference at ~2 Hz.
- **Soft-constraint tuning near zero force**: friction cone `epsilon = 25 N²` is deliberately
  restrictive near zero load; the contact-moment barrier (`mu 0.6, delta 0.03`) plus the
  four-sphere sole (defect below) can force vertical force chattering — see 1.4.
- **Estimator height noise**: `observationVelocityFilterCutoffHz` (30 Hz) and the KF's
  swing-foot noise scaling directly feed the planner; a bounce that is *planned* shows up in
  `plan_*` columns — confirm with `~/tracking` before tuning the estimator.

### 1.3 Soften touchdown (D2)
- **Touchdown vertical velocity**: `swingTrajectory.touchDownVelocity` defaults to `0.0`;
  a slightly *negative* target plus a larger `impactProximityFactorTouchDownVelocity`
  weight makes the optimizer prefer a slower landing (the factor currently *gates costs*,
  it doesn't command landing speed).
- **Late-swing damping**: raise `footMotionEquality.linearVelocityErrorGain_z` only in the
  last ~100 ms of swing (time-varying gain) — currently constant across the mode.
- **Vertical force-rate limiting**: add a soft cost on `Δf_z` between adjacent contact
  nodes (the wrench decision variable already exists; this is one extra quadratic term in
  `WbIdCost`) — the cleanest structural fix for impact chatter.
- **Swing spline**: replace the two-segment `CubicSpline`/`SplineCpg` with a quintic /
  min-jerk profile so acceleration is continuous at lift-off, apex and touchdown
  (`src/mpc/common/swing_foot_planner/`).

### 1.4 Foot / contact model realism (benchmark's #2)
- MJCF sole = **four spheres spanning 6 cm** → CoP soft constraint sits at/over its limit
  (cost ~5.5), which makes lateral/vertical force distribution brittle and noisy.
- Replace with a **sole box** (or 2×2 spheres + explicit rectangle) matching
  `model.contactGeometry.rectangle`, giving real CoP margin. This is a *model decision*
  (ask before changing) but it likely improves both bounce and lateral tracking.

### 1.5 Gait-transition smoothness
- `phaseTransitionStanceTime: 0.0` on all robots — gait ladder switches splice with **no
  double-support buffer**. Try 0.1–0.2 s stance splice at transitions (walk↔trot especially);
  `GaitSchedule` already supports it.
- The ladder's `minDwellTime: 0.2` + `switchDelayHorizonFraction: 0.3` hysteresis is good;
  consider **cost-based smoothing**: penalize contact-schedule changes (extra SQP variable
  or heuristic) so the gait doesn't oscillate between rungs near band edges.
- **Overlap bands** already exist between rungs — verify with the gait-regularity analyzer
  (`tests/tuning/analyze_gait_regularity.py`) that transitions produce symmetric step
  timing, not one long + one short step.

### 1.6 Command & upper-body polish
- Velocity command slew is linear (`velocityCommandRateLimits`); an **S-curve (jerk-limited)**
  profile removes acceleration discontinuities at ramp start/stop — small change in
  `WalkingVelocityTarget.cpp`.
- **Arm swing**: reference exists (`reference.armSwing`); verify phase locking to gait and
  add a small shoulder *velocity* weight reduction (already 0.06) so arms don't look stiff
  against the torso cost (`torsoTiltCost` active at 10° corridor).
- Revisit `useFeedbackPolicy: false` — the LQR feedback term from SQP is exactly the
  disturbance-rejection smoother; it was disabled for compute. If 1.1 raises the rate,
  re-enable and re-measure (benchmark round 5–6 tables are the baseline).

---

## Phase 2 — MPC formulation upgrades (medium effort)

Ordered by expected smoothness payoff per unit work.

1. **Contact-implicit / force-rate constraints** — replace prescribed wrench continuity with
   explicit `‖f_{k+1} − f_k‖` penalties or rate bounds; directly targets D2/D3 at the plan
   level. (Formulation doc §"Contact timing is prescribed rather than optimized".)
2. **Optimize contact timing** — make switching times decision variables (or phase-duration
   scaling per solve) instead of fixed template `switchingTimes`; removes the discrete jolt
   when a schedule change lands mid-horizon. Heavier: touches `GaitSchedule`, OCP dims,
   CppAD codegen.
3. **Touchdown impact model** — the state flow has *no impulsive reset* at touchdown
   (documented). Even a soft impulse/velocity-reset constraint on the swing foot at the
   scheduled touchdown node would let the plan anticipate impact instead of discovering it.
4. **Capture-point / DCM foothold** — `captureFootPlacement` is enabled on H2 but with
   `gain: 0.0` (capture term off; only Raibert `stanceTimeGain: 0.5`). Turning on a modest
   capture-point gain (sweep 0.3–1.0) is the standard fix for lateral sway and stop/start
   balance — benchmark item #3 (lateral tracking 35%) lives here.
5. **DCM/CoM reference trajectory** — generate the base reference from a linear inverted
   pendulum (DCM) model instead of raw velocity + height; known to remove bounce and give
   natural weight transfer. Moderate: new reference generator alongside
   `SwitchedModelReferenceManager`.
6. **Whole-body QP fallback layer** (optional) — a real-time QP projecting MPC torques
   through friction/contact limits each MRT tick (classic WBC) as a *safety smoother* under
   the MPC; only worth it if plan-level fixes leave residual violations.
7. **Terrain anticipatory height** — extend `terrain_walk`'s anticipated support-height
   trajectory to default flat-ground walking with a small look-ahead (currently only the
   terrain branch uses it), smoothing stairs/ramp entries.

---

## Phase 3 — Deep RL / learning-based (long-term)

Nothing RL-related exists in the repo today; treat this as a roadmap with explicit
prerequisites. Recommended path keeps the MPC as the safety layer and lets learning fix
what analytic tuning can't (model mismatch, contact transients, style).

### 3.1 Prerequisites (do first — needed for *any* sim2real, RL or not)
- **Actuator realism**: current model is ideal torque + joint PD (`effort_pd`,
  kp 200/kd 20). Docs admit "not a validated model of transmission coupling, actuator
  bandwidth or communication delay". Add motor dynamics (first-order lag / delay
  2–5 ms, torque-rate limit, PD saturation) to the MuJoCo actuator config.
- **Domain randomization harness**: friction, base mass ±10%, CoM offset, joint damping,
  sensor delay — a `randomized_scene.xml` generator; RL *requires* it, and it also
  stress-tests the MPC (push-recovery config is the seed).
- **Metrics for "naturalness"**: extend `plot_tracking.py` with step-time symmetry,
  duty-factor variance, jerk (d/dt of CoM acceleration), and a style score — you cannot
  optimize what you don't measure. (`analyze_gait_regularity.py` already computes parts.)

### 3.2 Tier 1 — Residual RL on top of MPC *(recommended first target)*
- Learn `τ = τ_MPC + τ_residual` (or residual on joint *targets* fed to the PD), policy
  observes `[q, v, τ_MPC, phase, contact flags]`, small bounded output (tanh × scale).
- **Why here**: MPC already walks and is validated; the residual only needs to learn
  smoothness/compensation (impact absorption, friction mismatch). Safe to train in sim with
  the MPC in the loop; falls during training don't matter.
- **Algorithm**: PPO (Isaac Gym / MJX / MuJoCo Playground style) at 50–200 Hz policy rate;
  reward = tracking + action-rate penalty (smoothness!) + torque penalty + upright.
- **Integration point**: `feedforward: policy_torque` in `control` — the residual slots
  into the existing feedforward path; no OCP changes.
- **Success criterion**: same `tests/tuning` suite shows D2/D3 improved with zero falls.

### 3.3 Tier 2 — Learned reference / gait generation
- **Policy outputs the MPC reference**: gait selection, step timing, swing height, CoM
  height/speed profile — i.e. learn the *reference manager*, keep WB-ID MPC as tracking
  controller. Closes the "prescribed contact timing" gap (Phase 2.2) without making
  timing a decision variable in the SQP.
- Alternatives: **motion imitation** (AMP-style style reward on reference generator), or
  distilled teacher (a fully-trained RL locomotion policy distilled into a reference
  network — Lee et al. 2020 / Unitree H1-style pipeline).

### 3.4 Tier 3 — Full RL locomotion policy (replaces MPC)
- End-to-end velocity-tracking policy (the Unitree G1 public RL policy approach).
- **Only after** 3.1: without actuator realism + randomization, sim2real fails.
- Trade-off vs. this repo's purpose: loses whole-body torque limits, contact-wrench
  feasibility and interpretability that WB-ID provides. Position it as a *baseline
  comparison* (benchmark smoothness metrics MPC vs RL), not a replacement, unless the
  project goal changes.

### 3.5 Learning *inside* the MPC (alternative to residuals)
- **Learned cost/constraint weights** (meta-RL or Bayesian tuning over the existing
  `make_config.py` search space) — automates the manual rounds in the benchmark.
- **Learned terminal value function** for the OCP (shortens horizon → higher MPC rate →
  helps 1.1); research-grade, OCS2 would need custom terminal cost hooks.

---

## Suggested order of work

```
Now          Phase 1.1 (blend / rate / warm-start)     → kills D3, biggest perceived jitter
Then         Phase 1.3 + 1.4 (touchdown + sole box)    → kills D2 spikes
Then         Phase 1.2 (height/vertical cost sweep)    → attacks D1 bounce
Parallel     Phase 1.5 gait splice + 1.6 arm/S-curve   → cheap polish
Then         Phase 2.4 capture-point gain on H2/G1     → lateral + stop/start naturalness
Later        Phase 3.1 prerequisites → 3.2 residual RL → 3.3 learned reference
Baseline     Phase 3.4 RL-only as comparison, if sim2real is a goal
```

## Risks / notes
- Every MPC change must re-run the layout-regression style checks (no solver exceptions,
  no torque-window overruns, rate ≥ 85% of baseline) — see `docs/results/`.
- Don't chase smoothness by detuning safety: friction `mu 0.4`, leg self-collision and
  torque limits are validated; keep them as acceptance gates in each A/B case.
- RL tiers need compute (GPU sim) this repo doesn't assume; scope them as a separate
  workspace package (e.g. `legged_robot_rl`) that imports this package's robot descriptions
  and reward-relevant configs, keeping the C++ controller untouched.
