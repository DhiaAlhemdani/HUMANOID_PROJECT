# Locomotion Quality Improvements (Smoothness, Gait, Learning)

Scope: **control/algorithm** improvements — walking smoothness, naturalness, robustness, and
learning-based methods. (Repo engineering hygiene is tracked separately in `IMPROVEMENTS.md`.)

Grounding: this plan starts from the measured gaps in the
[tuning benchmark](wb_id_tuning_benchmark.md) — the shipped G1 config (`d_kn10`) already
passes "no falls", but three smoothness defects remain:

| # | Measured defect (G1, 0.5 m/s walk) | Evidence |
|---|---|---|
| D1 | **MPC-planned pelvis bounce** ≈ 11–12 mm (z sd), 4–40 Hz vertical accel 1.19 — **CLOSED 2026-11-24**: z sd not reproduced (GT 1.6 / plan 2.14 mm), cost sweep 0/4 → residual `acc_hf` 1.18 is contact-driven, carried into 1.3 | benchmark §"What remains"; blend A/B `summary.txt`; [`bounce_cost_sweep.md`](results/bounce_cost_sweep.md) |
| D2 | **Touchdown torque spikes** ≈ 50 Nm (knee/ankle 99.9th pct) — **IMPROVED then CLOSED 2026-11-24**: `touchDownVelocity −0.05` shipped → `acc_hf` −8.7% (best of project); the 50.25 peak proved **invariant across all three lever rounds** (trajectory configs, touchdown velocity, structural load-share: ≤0.02 Nm over a 1000× span) at ≈ ankle effort-limit scale → **root-caused 2026-11-24: ankle actuator saturation** (`pd_tau_p999` is |meas−cmd| with meas clamped at the 50 Nm ankle limit; contact-model sweep neutral) → metric retired, replaced by `legs.tau_sat_pct`; residual D2 = servo layer (ankle kp 200 × touchdown error) | benchmark; [`touchdown_sweep.md`](results/touchdown_sweep.md); [`load_share_cost.md`](results/load_share_cost.md); [`foot_contact_sweep.md`](results/foot_contact_sweep.md) |
| D3 | **Per-solve command ripple** — torque jumps at each new solution: legs 0.79 vs 0.31 Nm on other ticks — **closed by the Phase 1.1 blend** (jump 1.317 → 0.105–0.137 on the 2026-11-24 baselines); load-share round **rejected** (+27…41 %), `useFeedbackPolicy` **rejected** (+96 %, −18 % solve) | `cmd_tau_jump_at_switch` in `plot_tracking.py`; [`load_share_cost.md`](results/load_share_cost.md) |

User priority order (from the benchmark): **falls → smoothness (bounce, jitter) → velocity tracking.**

The measurement harness already exists (`tests/tuning/`: `run_case.sh`, `plot_tracking.py`
SUMMARY/DIAG metrics) — every item below is designed to be A/B-tested with it.

---

## Phase 1 — Smoothness with the existing MPC (highest ROI)

These need no new architecture; most are config or small controller changes.

### Phase 1 scorecard (closed 2026-11-24)

Every section followed implement → closed-loop sweep → results doc; **28 scored candidates in 8
sweeps, 2 shipped, 0 falls**. Shipped config/controller changes (all in `config/g1`):

| change | effect (same-build A/B) | source |
| --- | --- | --- |
| `control.solveBlendSeconds` policy-swap blend (1.1) | jump@switch **1.317 → 0.105–0.137 Nm**, arm ripple 0.206 → 0.035 | [`policy_swap_blend.md`](results/policy_swap_blend.md) |
| `touchDownVelocity: −0.05` (1.3) | `acc_hf` **1.18 → 1.08 (−8.7 %)**, leg PD HF −7.8 % | [`touchdown_sweep.md`](results/touchdown_sweep.md) |

Rejected with evidence (do not retry as-is): bounce cost levers (1.2), swing-trajectory configs
and `impactProximityFactor` (1.3), DS load-share cost (1.3 structural), box sole / fitted CoP
rectangle (1.4), stance splice at gait changes (1.5), S-curve command slew and
`useFeedbackPolicy` (1.6). Inert-but-available tooling shipped at weight/limit 0:
`contactLoadShareWeight`, `velocityCommandJerkLimits`, `make_model.py` sole variants.

What the negatives established (the real deliverable of the phase):
1. **The 50.25 Nm "touchdown peak" is ankle actuator saturation** (50 Nm limit, servo kp 200);
   no plan-side lever can move it — D2 is a servo-layer item, now measured by `legs.tau_sat_pct`.
2. **Steady bounce is contact-driven and gait-driven**: trot `acc_hf` 1.40 vs walk 1.05–1.08;
   at ≥0.5 m/s the trot template is the largest remaining smoothness source.
3. **Transitions are best left continuous**: any inserted pause (stance splice) or discontinuous
   feedback (per-solve `K`) adds lurch; smoothing must interpolate across the handover.

Instrumentation added to `plot_tracking.py` (back-fillable from any `data.npz`): `gait`
(templates, transitions, per-change window metrics), `command` (ramp/step transients),
`tau_sat_pct`; generic scorer `tests/tuning/compare_ab.py`.

Recommended next steps, in benchmark-priority order: (a) servo-layer D2 — ankle kp/kd vs
`tau_sat_pct` and touchdown tracking error; (b) trot template smoothness or a double-support
rung between walk and trot; (c) feedback policy with the blend extended to `K`.

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

**Measurement status (2026-11-24, blend A/B runs, G1 vx 0.5, `summary.txt`):** the benchmark's
11–12 mm planned bounce **does not reproduce** — GT z sd 1.6 mm, plan 2.14, obs 1.8 (the branch
now carries main's quintic swing-foot spline). D1's headline is resolved; what remains is the
**4–40 Hz vertical acceleration `pelvis_acc_hf` = 1.16** (benchmark's post-tuning best: 1.19)
plus plan_z 2.14 mm. Recon: the walk height target is a *constant* (`defaultBaseHeight`, terrain
estimate off) — there is no gait-phase z reference to smooth, so the "smooth the height
reference" lever is a dead end; the bounce was and is an optimizer choice.

**Sweep status (implement → test → document) — COMPLETE, no winner:**

| step | status | artifact |
| --- | --- | --- |
| implement (sweep cases + tooling) | ✅ done | `tests/tuning/run_bounce_sweep.sh` (cases `accz10`, `accz100`, `z100`, `vz10`) + `tests/tuning/compare_bounce.py`; shipped config unchanged |
| test (closed-loop sweep) | ✅ done, **0/4** | `tuning_results/bounce_sweep_report.txt`: only the primary `acc_hf` gate failed (all candidates within ±2% noise on it, z100 +4.5% worse); every other guard passed |
| document results | ✅ done | [`docs/results/bounce_cost_sweep.md`](results/bounce_cost_sweep.md) |

**Conclusion:** `acc_hf ≈ 1.18` is insensitive to planned-trajectory costs (the `r.baseAccelerations`
z lever was inert even at ×100) while the plan itself is smooth — the residual enters through
**contact/servo execution, not the plan**. D1 closed; the residual + D2's 50.25 Nm touchdown
peak move to **Phase 1.3**. Notable near-miss: vz 5→10 cut GT z sd −15% (1.53→1.30 mm) but
raised `acc_hf` — not adopted (z sd was already at the noise floor).
Doc/config note: the benchmark's "shipped vz = 10" does not match the repo (config has been 5
since the root commit).

- **Vertical cost shaping**: the shipped `q.baseVelocity[2]` (vz) is 5 (the benchmark doc says
  10) and benchmark round 6
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

**Recon (2026-11-24):** main already landed the quintic swing-height spline (roadmap's
"spline" item — `SwingTrajectoryPlanner` uses `QuinticSpline` nodes with velocity boundary
conditions), so the untested levers are the two swing-trajectory configs the benchmark
explicitly left as its next step (§"Touchdown impact… the swing touchdown velocity").
Semantics read from code: node velocity = `min(1, swingDuration/swingTimeScale) × config`
→ **0.75× for the 0.3 s walk swing**; z-up world, so `touchDownVelocity` sign is ambiguous
in the docs (negative = still descending at contact) — the sweep tests **both signs**.
`impactProximityFactor*` are boundary *slopes* of the velocity-error gating cubic (value 1
at both swing ends, 0 mid-swing): a larger touchdown slope tracks the landing velocity more
tightly approaching contact. Baseline: legs `pd_tau_p999` **50.25 Nm** (right ankle),
`acc_hf` 1.18, leg PD HF 2.22 (blend A/B `summary.txt`).

**Sweep status (implement → test → document):**

| step | status | artifact |
| --- | --- | --- |
| implement (sweep cases + tooling) | ✅ done | `tests/tuning/run_touchdown_sweep.sh` (cases `tdvn05`, `tdvp05`, `impf06`, `impf10`) + `tests/tuning/compare_touchdown.py` (`77977ff`) |
| test (closed-loop sweep) | ✅ done 2026-11-24 | **0/4 gates** — but primary `pd_tau_p999` shown **config-invariant** (50.24–50.26 across all four edits); report `tuning_results/touchdown_sweep_report.txt` |
| document results | ✅ done | [`docs/results/touchdown_sweep.md`](results/touchdown_sweep.md) |
| implement (structural: DS load-share cost + tooling) | ✅ done | `WbIdContactLoadShareCost` (registered only when `contactLoadShareWeight > 0`, shipped 0), microbench 14/14, contract test (`8c704da`) |
| test (structural sweep, 4 log-spaced weights) | ✅ done 2026-11-24 | **0/4** — primary `jump@switch` **worse +27…41%** (0.105 → 0.133–0.148); peak unmoved (50.24–50.25); `acc_hf` no gain; report `tuning_results/loadshare_sweep_report.txt` |
| document structural results | ✅ done | [`docs/results/load_share_cost.md`](results/load_share_cost.md) |

Gates: legs `pd_tau_p999` ≤ 0.90× (original primary), survived, vx ≥ 0.90×, `acc_hf` ≤ 1.10×,
leg PD HF ≤ 1.10×+0.1, leg track err ≤ 1.20×+0.05°, plan z sd / z-track ≤ 1.10×+0.5 mm,
roll ≤ 1.15×+0.05°, arm swing ≤ 1.20×+1°, solve ≥ 0.90×.

**Outcome:** the peak is set at the contact/servo layer (≤0.02 Nm movement across a
∓0.075 m/s × 3.3× sweep) — no swing-trajectory config can pass the original primary, so the
effectiveness gate re-bases on `pelvis_acc_hf` (benchmark priority #1). **Shipped:
`touchDownVelocity: -0.05` (g1)** → `acc_hf` 1.1805 → **1.078 (−8.7%, best of project)**,
leg PD HF −7.8%, all other guards pass. Rejected: `+0.05` (acc_hf +29% — sign confirmed),
`impactProximityFactor` 0.6/1.0 (solve 67→56 Hz, acc_hf +6%). The structural Δf_z round
then **also failed to move the peak and worsened D3** ([`load_share_cost.md`](results/load_share_cost.md))
→ D2 closed as plan-inert; the bounce-energy improvement stands.

- **Touchdown vertical velocity** — ✅ **tested & shipped**: `touchDownVelocity: -0.05` in the
  g1 config (`acc_hf` −8.7%; `+0.05` was +29% — sign resolved empirically; sweep:
  [`results/touchdown_sweep.md`](results/touchdown_sweep.md)). The
  `impactProximityFactorTouchDownVelocity` slope was rejected (solve 67→56 Hz, no softening).
- **Late-swing damping**: raise `footMotionEquality.linearVelocityErrorGain_z` only in the
  last ~100 ms of swing (time-varying gain) — currently constant across the mode.
- **Vertical force-rate limiting** — ❌ **tested & rejected** (structural round, 0/4):
  implemented as the `WbIdContactLoadShareCost` DS load-share cost (ocs2 costs are
  per-stage `(t, x, u)`, so a literal cross-node `Δf_z` term is not expressible; the cost
  instead shapes the split direction the RNEA leaves free toward a C1 smoothstep transfer;
  shipped weight **0**). Sweep: jump@switch **+27…41% worse** at every weight (moving φ
  target + per-node activation as the horizon slides → more solution disagreement at
  switches), peak unmoved, `acc_hf` no gain — full analysis:
  [`results/load_share_cost.md`](results/load_share_cost.md). Tooling kept:
  microbench 14/14, contract test, sweep pair.
- **Swing spline**: replace the two-segment `CubicSpline`/`SplineCpg` with a quintic /
  min-jerk profile so acceleration is continuous at lift-off, apex and touchdown
  (`src/mpc/common/swing_foot_planner/`).

### 1.4 Foot / contact model realism (benchmark's #2)
- ~~MJCF sole = four spheres spanning 6 cm~~ **Recon correction (2026-11-24):** the G1 sole is
  **four r = 5 mm spheres** per foot at x ∈ {−0.05, 0.12}, y ∈ {±0.025 heel, ±0.03 toe},
  z −0.03 (ankle-roll frame; bottoms at −0.035). The MPC rectangle (`frameTranslation` x 0.035,
  `rectangle [0.09, −0.09, 0.03, −0.03]`) spans x −0.055…0.125 / y ±0.03 — i.e. the planner's
  CoP polygon is **5 mm larger than the physical footprint on every edge**, so the CoP soft
  constraint can sit on the real tipping edge while the plan believes it has margin.
  Also: the robot's `solref 0.001` is **inert** — the floor geom has `priority="1"` with no
  solref, so MuJoCo's default 20 ms contact governs (the MJCF header comment overstates it).
- Levers (both tested headlessly, same harness): **sole box** (shape change, one box per foot
  at the same ground height; `tests/tuning/make_model.py`, hooked into `run_case.sh` via
  `MODEL=`) and a **fitted CoP rectangle** `[0.085, −0.085, 0.025, −0.025]` (config-only, 5 mm
  margin inside the sphere hull). Model variant = user-approved (asked; "4,5,then 6").

**Sweep status (implement → test → document):**

| step | status | artifact |
| --- | --- | --- |
| implement (model tool + scene hook + sweep + comparator) | ✅ done | `tests/tuning/make_model.py`, `mujoco_scene.py --robot-model`, `run_case.sh MODEL=`, `run_footcontact_sweep.sh` (cases `fc_rectfit`, `fc_box`, `fc_box_rectfit`, `fc_boxrect`), `compare_footcontact.py`; contract `tests/foot_contact_model_test.py` |
| test (closed-loop sweep) | ✅ done 2026-11-24 | **0/4, all guards pass, no primary improves** — contact shape is neutral on acc_hf/roll; `fc_boxrect` peak 48.99 (−2.5 %) is the first movement of the "plateau" and explains it (see below); report `tuning_results/footcontact_sweep_report.txt` |
| document results | ✅ done | [`docs/results/foot_contact_sweep.md`](results/foot_contact_sweep.md) |

**Outcome:** nothing shipped (MJCF and rectangle unchanged); §1.4 closed. Key finding: the
50.24 Nm `pd_tau_p999` is **ankle actuator saturation** (`actuatorfrcrange ±50` = URDF effort,
servo `kp 200` ⇒ 14° touchdown error saturates) — a clipped signal, which is why every OCP/contact
lever left it at 50.25 ± 0.01. `plot_tracking.py` now reports `legs.tau_sat_pct` (saturation duty)
and `pd_tau_p999` is retired as a D2 effectiveness metric. D2 is a servo-layer item from here
(ankle kp/kd vs touchdown error), to be measured with `tau_sat_pct` first.

Gates (any-of primaries — the roadmap expects *both* bounce and lateral gains and the D2 peak
lives at this layer): `acc_hf` ≤ 0.92× **or** roll ≤ 0.85× **or** legs `pd_tau_p999` ≤ 0.90×;
guards: survived, the non-improved primaries must not worsen (≤1.10× / ≤1.15×+0.05 / ≤1.05×),
jump@switch ≤ 1.15×+0.02, leg PD HF ≤ 1.10×+0.1, track err ≤ 1.20×+0.05°, plan z sd / z-track
≤ 1.10×+0.5 mm, arm swing ≤ 1.20×+1°, vx ≥ 0.90×, solve ≥ 0.90×.

### 1.5 Gait-transition smoothness
- `phaseTransitionStanceTime: 0.0` on all robots — gait ladder switches splice with **no
  double-support buffer**. Try 0.1–0.2 s stance splice at transitions (walk↔trot especially);
  `GaitSchedule` already supports it.

**Recon (2026-11-24):** `GaitScheduleUpdater` splices the new template in at the event before
an `LF` phase, and `GaitSchedule::insertModeSequenceTemplate` adds the stance only when the
outgoing mode is not already stance. Net effect: **walk→trot deletes the walk's last double
support** and inserts `phaseTransitionStanceTime` s of stance instead (shipped 0 → swing straight
into swing); trot→walk inserts it after a swing; stand↔gait changes are unaffected. The 0.5 m/s
probe sits on the walk→trot edge (walk band tops at 0.45, trot engages at measured >0.40), so the
sweep runs at **0.6 m/s** to cross walk→trot on the ramp up and trot→walk on the way down.

**Sweep status (implement → test → document):**

| step | status | artifact |
| --- | --- | --- |
| implement (transition metrics + sweep + comparator) | ✅ done | `plot_tracking.py` `gait` section (template classification from `plan_mode`, per-change window metrics, back-fillable), `tests/tuning/synth_run.py`, `run_gaitsplice_sweep.sh` (cases `gs_t005`…`gs_t020`), `compare_gaitsplice.py`; contract `tests/gait_transition_metrics_test.py` |
| test (closed-loop sweep @ 0.6 m/s) | ✅ done 2026-11-24 | **0/4 — every splice length is worse**: transition roll +36…48 %, z excursion +31 % from 0.15 s, first-step asymmetry 0.99 at 0.2 s, jump@switch +32…39 % at 0.1/0.2 s; all survived, 2 transitions each; report `tuning_results/gaitsplice_sweep_report.txt` |
| document results | ✅ done | [`docs/results/gait_splice_sweep.md`](results/gait_splice_sweep.md) |

**Outcome:** `phaseTransitionStanceTime` stays **0.0** — a stance pause at a gait change lets the
base keep its momentum over planted feet, so the first step of the new template starts from a
larger CoM offset (bigger lateral lurch), and longer pauses break the step rhythm. The shipped
swing-into-swing splice is the best of the five. §1.5 closed. Side finding: **trot itself** is the
remaining bounce source at ≥0.5 m/s (steady `acc_hf` 1.40 in trot vs 1.05–1.08 in walk).

Gates (any-of primaries over the worst transition window [−0.5, +1.5] s): `transition_acc_hf_max`
≤ 0.85× **or** `transition_z_pp_mm_max` ≤ 0.85× **or** `transition_roll_max_deg` ≤ 0.85×; guards:
survived, same number of gait-to-gait transitions as the baseline, non-improved primaries not
worse, `first_step_asym_max` ≤ 1.20×+0.05, plus the steady-state guard set (acc_hf, roll, jump,
PD HF, track err, z, arms, vx, solve).
- The ladder's `minDwellTime: 0.2` + `switchDelayHorizonFraction: 0.3` hysteresis is good;
  consider **cost-based smoothing**: penalize contact-schedule changes (extra SQP variable
  or heuristic) so the gait doesn't oscillate between rungs near band edges. *(Not pursued:
  the 0.6 m/s sweep showed no rung oscillation — exactly one up and one down change per run.)*
- **Overlap bands** already exist between rungs — ~~verify with the gait-regularity analyzer~~
  verified by the `gait` diag section: the shipped splice gives first-step asymmetry **0.00**
  at both changes; only the stance-pause candidates break symmetry.

### 1.6 Command & upper-body polish
- Velocity command slew is linear (`velocityCommandRateLimits`); an **S-curve (jerk-limited)**
  profile removes acceleration discontinuities at ramp start/stop — ✅ **implemented**:
  `CommandSlewLimiter.h` (header-only `slewLimitStep`, discrete-safe brake cap, jerk 0 =
  bit-identical legacy clamp) used by `WalkingVelocityTarget`; new parameter
  `reference.velocityCommandJerkLimits` (shipped `[0,0,0,0]` → inert). Recon: the command path is
  a 5 Hz low-pass → 0.5 m/s² clamp, so a command step was a linear ramp with an acceleration
  *step* at both ends. Microbench `tests/tuning/run_slew_microbench.py` **17/17**.
- **Arm swing**: reference exists (`reference.armSwing`); phase locking is by construction
  (reference generated from the gait phase) and the shipped swing is 8.4° pp with no smoothness
  defect attached — **not swept** (no measurable criterion under the benchmark priorities).
- Revisit `useFeedbackPolicy: false` — the LQR feedback term from SQP is exactly the
  disturbance-rejection smoother; it was disabled for compute. Recon: the controller evaluates
  the MRT policy with the *measured* state, so the flag is live (config-only A/B in the sweep).

**Sweep status (implement → test → document):**

| step | status | artifact |
| --- | --- | --- |
| implement (S-curve limiter + wiring, `command` transient metrics, generic comparator, sweep) | ✅ done | `CommandSlewLimiter.h`, `plot_tracking.py` `command` section, `tests/tuning/compare_ab.py`, `run_polish_sweep.sh` (set A: `po_base/po_fb/po_jerk1/po_fb_jerk1` @ vx; set B: `ps_base/ps_jerk1/ps_jerk05` @ `stops`); contract `tests/command_slew_contract_test.py` |
| test (closed-loop) | ✅ done 2026-11-24 | **0/3 (set A) + 0/2 (set B)** — `useFeedbackPolicy` doubles jump@switch (+96 %) and costs 18 % solve rate despite −37 % ramp pitch; S-curve inert in steady walk, no primary win at command steps (hold `acc_hf` −9/−10 % in the stops profile noted, with a roll penalty at jerk 0.5); report `tuning_results/polish_sweep_report.txt` |
| document results | ✅ done | [`docs/results/polish_sweep.md`](results/polish_sweep.md) |

**Outcome:** nothing shipped; `useFeedbackPolicy` stays false, `velocityCommandJerkLimits` stays
`[0,0,0,0]` (limiter kept; `[1,1,0,4]` is the tested-safe teleop value). Lead: the feedback
policy's within-solve gain is real — usable only if the policy-swap blend also interpolates `K`.
**§1.6 closed — Phase 1 complete.**

Gates — set A (steady walk, probe ramps): any-of `legs.cmd_tau_jump_at_switch` ≤ 0.85× /
`acc_hf` ≤ 0.92× / `command.transient_pitch_max_deg` ≤ 0.85×, default guard set (incl. solve
≥ 0.90× — the feedback policy's known cost). Set B (command steps 0↔0.5 m/s): any-of transient
pitch / acc_hf / z pp ≤ 0.85×; guards survived, vx, solve, steady acc_hf/roll.

---

## Phase 2 — MPC formulation upgrades (medium effort)

Ordered by expected smoothness payoff per unit work *(as estimated before Phase 1)*.

### Phase 2 execution order (set 2026-11-24, after Phase 1's evidence)

Same methodology per item: implement → closed-loop sweep against a fresh baseline → results doc;
shipped defaults stay inert until a sweep wins. Re-sequenced because Phase 1 showed the
touchdown peak is actuator saturation (servo layer) and the remaining bounce is gait/contact
driven, which lowers the expected payoff of plan-level force shaping (items 1–2):

| order | item | why here | status |
| --- | --- | --- | --- |
| 2.A | **#4 capture-point / Raibert foothold** | config-only; the only lever aimed at lateral sway/roll and start-stop balance (benchmark #3) | **closed 0/6** ✅ — steady: roll +14…22 %, jump@switch +42…83 %; stops: transient roll +32…73 %. Established negative → `docs/results/capture_point_sweep.md`; block stays disabled |
| 2.B | **#6 whole-body QP layer** (promoted from "optional") | the 1 kHz PD at kp 200 is what turns touchdown error into saturated torque; a high-rate WBC tracking the MPC's wrenches/accelerations with the *measured* state is the largest smoothness step available with the current MPC | S1 implemented (`control.wholeBody`, ships disabled); **sweep 1: 0/4 + 0/2 by gates, 3 falls at the original gains** — soft set survives with **acc_hf −35 %**, ankle saturation −90…100 %, roll track −11…18 %, but `meas_tau_hf` +15 % and stops pitch transient 4.75° → `docs/results/wbc_s1_sweep.md`; defaults lowered to the soft set; **sweep 2: 0/4 + 0/2** — waist weight / torso orientation task / softer pelvis all leave hip_pitch |max| at 86 Nm; spike anatomy: every peak is a 2 ms impulse on the landing leg at the plan's contact switch (layer swaps its contact set before the robot does) → **sweep 3: 2/3 + 2/2, SHIPPED** ✅ — `contactBlendSeconds 0.03` fade of the layer torque into the plan torque around planned switches removes the impulses (hip_pitch |max| 86 → 39 Nm): **acc_hf 1.086 → 0.580 (−47 %)**, ankle saturation → 0, pitch track −29 %, stop pitch transient −26 %, 0 falls; G1 ships `control.wholeBody.enabled: true` (soft set + fade); `enabled: false` restores the Phase-1 path. Costs: walk z sd +0.5 mm, stops `meas_tau_hf` +5 %. Not swept: `vy`/`yaw`, other speeds, trot |
| 2.C | **#3 touchdown impact model** | contained (dynamics jump map at mode events, ocs2 SQP supports event nodes); lets the plan anticipate impact — 2.B located the whole remaining torque transient at this event | design `docs/design/touchdown_impact.md` (jump map at the existing SQP event nodes); **baseline measurement (`touchdown.*` metrics) found the robot lands 14–20 ms before the plan's event at every step** — `swingTrajectory.touchDownHeightOffset: -0.001` (upstream default, never swept) aims the swing 1 mm below the ground, so the loaded foot sits at planned zero wrench until `t_e` (base-vz mismatch 42–57 % of plan vz rms). **sweep 1 (config, offset −0.5/0/+1 mm): 3/3 in steady walk (acc_hf 0.619 → 0.52 / 0.467), 0/2 in stops** (post-touchdown vz mismatch 1.26×/1.58× vs a 1.1× not-worse gate) → nothing ships; established: the offset explains ~1/3 of the early landing (21 → 10 ms); the post-touchdown vz mismatch is not a timing effect. **Trace (`touchdown_trace.py`): the robot tracks the MPC foot plan to ≤ 0.2 mm all the way down; the *plan* crosses the ground 41 ms before its contact event at 0.24 m/s and continues to −2.2 mm — the real landing speed is ≈ 0.3 m/s (6× `touchDownVelocity`), the ground stops the foot early.** **Root cause (sweep 2, confirmed): the swing acceleration equality (`footMotionEquality` kp 50 / kd 50) has a −1 rad/s height-error pole, so the plan's sag below its spline is never corrected; kp 200/400/625 → landing 8/6/4 ms early at 0.19 m/s, acc_hf −26/−31/−33 %, plan dive gone.** 400 regresses the stops' pitch transient 1.33× (gain shared with the stance height correction, lowered 250 → 50 earlier for ramps) → **shipped as a new swing-only gain `swingPositionErrorGain_z: 200` (stance stays 50)**: steady-walk acc_hf 0.617 → 0.455, early 20 → 8 ms, landing 0.26 → 0.19 m/s, stops not-worse. Sweep 3 confirmed the split (acc_hf −28 % on repeat; swing 400 regresses stops twice → out). Sweep 4: offset 0/+0.5 mm on top rejected (late landings, acc_hf +14/+23 %); the split's torque-jump excess at contact changes is carried by sporadic ~20 Nm single-switch jumps that also occur in the pre-ship baseline → traced: they are the plan's own inverse-dynamics step at a scheduled contact change (60 Nm hip pitch, handed to the plan by the 2.B fade), caught only when a solve delivery lands on the event's tick; `plot_tracking.py` now excludes those ticks from the metric. **2.C-timing closed, shipped as is.** `base_vz_mismatch` found to be 23–46 % noisy between same-config runs (sweep-1 set-B verdict was noise). Residual 0.19 m/s at 2 mm is the spline's own terminal speed (`touchDownVelocity`), jump map still deferred → `docs/results/touchdown_timing_sweep.md` |
| 2.D | **#5 DCM/CoM reference** | moderate; natural lateral weight transfer; measured against 2.A's result | after 2.C |
| 2.E | **#1 force-rate** | only as *state augmentation* (forces as state, rates as input — the per-stage cost route was tried and rejected in 1.3); value re-assessed once 2.B/2.C land | deferred |
| 2.F | **#2 contact-timing optimization** | heaviest; last | deferred |
| — | **#7 terrain anticipatory height** | flat-ground benchmark does not exercise it | out of scope for smoothness |

**Phase 2 scorecard so far (2026-11-25):** 2.A 0/6 (closed), 2.B 2/3 + 2/2 after two 0-winner
sweeps (shipped) — 22 scored candidates in 5 sweeps, 1 feature shipped, 3 falls (all 2.B sweep 1 at
the original task gains, none since the soft set).

| change | effect (same-build A/B) | source |
| --- | --- | --- |
| `control.wholeBody` S1 layer on, soft gains + `contactBlendSeconds 0.03` (2.B) | `acc_hf` **1.086 → 0.580 (−47 %)**, `tau_sat_pct` 0.19 → 0 %, pitch track 0.085 → 0.060°, stops: pitch transient **1.21 → 0.90°**, acc_hf −16 % | [`wbc_s1_sweep.md`](results/wbc_s1_sweep.md) |

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
