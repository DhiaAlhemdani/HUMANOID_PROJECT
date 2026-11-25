# Phase 2.B — whole-body feedback layer between the MPC and the joint servos

Status: **S1 implemented, ships disabled, awaiting the closed-loop sweep** (2026-11-24). Roadmap
Phase 2 #6, promoted to 2.B by the Phase 1 evidence. Decisions taken: S1 go (user); S2's dense QP
will be HPIPM, which `ocs2_sqp` already links through `hpipm_catkin` / `blasfeo_catkin` in the
`wei-hsuan-cheng/ocs2_ros2` stack this repo builds against (no new dependency).

## 0. What S1 is in the code

| piece | file |
| --- | --- |
| Eigen-only least-squares core (KKT solve, wrench clip, joint-row torques) | `include/legged_robot_mpc_controller/controller/whole_body_feedback_core.hpp` |
| Pinocchio assembly (plan swing targets, measured `M, h, J, J̇v`, task PD) | `include/.../controller/whole_body_feedback.hpp`, `src/controller/whole_body_feedback.cpp` |
| Controller wiring: replaces the feedforward in `compute_mpc_joint_action`, residual servo gains ramped in Locomotion only, `wbc/*` tracking columns | `src/controller/controller.cpp`, `controller.hpp` |
| Parameters (`control.wholeBody.*`, shipped `enabled: false`) | `src/controller/parameters.yaml`, `config/g1/ros2_controllers.yaml` |
| Tests | `tests/whole_body_feedback_core_test.cpp` (Eigen, in `BUILD_TESTING`; passes in the sandbox), `tests/whole_body_feedback_contract_test.py` |
| Sweep | `tests/tuning/run_wbc_sweep.sh`; new metric `legs.meas_tau_hf` in `plot_tracking.py` (total applied torque HF — `pd_tau_hf` is not comparable once the feedback moves into `cmd_tau`) |

Sweep 1 (`docs/results/wbc_s1_sweep.md`): −35 % pelvis acc_hf and no ankle saturation with soft
gains, but the pelvis base task folds the torso at the waist (3 falls at the original gains).
Added for sweep 2: per-joint posture weights and a link-orientation task (`torso_link`), see §2.

Behavioural notes that differ from the sketch below: the swing-foot task uses the plan's foot
position/velocity from forward kinematics of the policy state and the measured-configuration
Jacobian for the planned acceleration; the residual servo gains are smooth-stepped from
`jointEffortPd` to `residualKp/Kd` over `gainRampSeconds` after the bring-up reaches Locomotion
(the Blend phase keeps the shipped gains); an invalid solve falls back to the plan's torque for
that tick and is counted (`whole-body feedback solve invalid` log, `wbc/active` column).

## 1. Why this is the next item

Phase 1 established where the remaining roughness comes from (`docs/LOCOMOTION_IMPROVEMENTS.md`,
Phase 1 scorecard):

- `legs.pd_tau_p999 ≈ 50.25 Nm` = the ankle actuator limit. At every touchdown the 1 kHz joint PD
  (`kp 200`, `kd 20/10`) converts a few millimetres of foot/joint error into a saturated ankle torque.
  No plan-side lever moved this; it is produced *below* the MPC.
- Between two 60 Hz solves the controller is open loop on the base: `evaluatePolicy` interpolates the
  plan, so base roll/pitch error only gets corrected when the next solve sees it, and then with a
  full-horizon re-plan (the per-solve torque ripple that `solveBlendSeconds` now hides).
- Lateral balance: 2.A showed the optimizer's foothold is already good; what is missing is a fast
  *reaction* to the measured base state, not a different plan.

Today's servo path (`src/controller/controller.cpp::compute_mpc_joint_action`):

```
MPC 60 Hz ──policy──▶ evaluatePolicy(t) ──▶ q*, v* (joint PD targets, kp 200 / kd 20)
                                       └──▶ τ_ff = MPC torque (or RNEA of the plan)
hardware: τ = kp (q* − q) + kd (v* − v) + τ_ff            ← per joint, contact-blind
```

Joint-space PD is contact-blind: a stance ankle gets the same `kp 200` as a swinging hip, although
its reflected inertia is two orders of magnitude smaller and its foot is glued to the ground. The
physically right feedback is in **acceleration space, through the dynamics, with the contacts as
constraints** — that is what a whole-body controller does.

## 2. Formulation

Decision variables at each 1 kHz tick, with the **measured** `q, v` (23 joints + floating base, n_v = 29):

- `a ∈ ℝ²⁹` generalized accelerations
- `F ∈ ℝ¹²` contact wrenches (6 per foot, zero for a swing foot)
- `τ ∈ ℝ²³` joint torques (eliminated analytically from the joint rows of the dynamics)

Equality constraints
1. Floating-base dynamics rows: `M_b(q) a + h_b(q, v) = J_bᵀ F`  (6 rows)
2. Stance foot no-motion: `J_c a + J̇_c v = 0` for each foot in contact per the MPC mode at `t` (6 rows per stance foot)

Inequalities (Stage 2 only)
3. Friction cone + unilateral: `|F_x|, |F_y| ≤ μ F_z`, `F_z ≥ 0` (linearised, 5 rows per foot)
4. Centre of pressure inside the sole: `|τ_y| ≤ ℓ_x F_z`, `|τ_x| ≤ ℓ_y F_z` (4 rows per foot) — this is the
   constraint that **bounds ankle torque physically** instead of by actuator clipping
5. Torque limits: `|τ| ≤ τ_max` with `τ = M_j a + h_j − J_jᵀ F`

Weighted least-squares cost (single-level, weights are parameters)
- base task: `‖a_base − a_base^des‖²_{W_b}`, with `a_base^des = a_base^plan + K_p,b ⊖(pose_plan, pose) + K_d,b (twist_plan − twist)`
- swing-foot task: `‖J_sw a + J̇_sw v − ẍ_sw^des‖²_{W_sw}`, `ẍ_sw^des = ẍ_plan + K_p,sw e + K_d,sw ė` (position only, 3 rows)
- joint posture: `‖a_j − (a_j^plan + K_p,j (q*−q) + K_d,j (v*−v))‖²_{W_j}` — the role the kp 200 PD plays today, now inertia-weighted
- link orientation (optional, `orientationLink`): `‖J_ω a + J̇_ω v − ω̇^des‖²_{W_o}`, `ω̇^des = ω̇_plan + K_p log3(R_plan Rᵀ) + K_d (ω_plan − ω)` in world axes — puts the attitude task on the torso (the mass) instead of the pelvis
- wrench tracking: `‖F − F^plan‖²_{W_F}` — keeps the MPC's load distribution in double support
- regularisation on `a`, `F`

All plan quantities (`a^plan`, `F^plan`, `q*`, `v*`, swing trajectory) come from the same
`evaluatePolicy` call used today; the mode comes from `tracking_policy_mode_` / `contact_flags_at(t)`.

Output: `τ_WBC` is written as the `effort` command; the joint PD gains written alongside are reduced to a
small damping (`kp ≈ 10–30`, `kd ≈ 2–5`) that covers model error and the 1 ms hold.

## 3. Staged delivery

| stage | what | QP needed | new deps | est. C++ | what it should move |
| --- | --- | --- | --- | --- | --- |
| **S1 — inertia-shaped, contact-consistent feedback** | Closed-form projected inverse dynamics: `a^des` from the plan + task/joint PD above, `F` from the plan corrected by the least-squares solution of (1)–(2), `τ` from the joint rows. No inequalities; cone/CoP violations only clipped. | no | none | ~350 lines (`WholeBodyFeedback.{h,cpp}` + wiring) | `pd_tau_hf`, `tau_sat_pct`, `pelvis_acc_hf` at touchdown; `roll_track_err`, `pitch_track_err` between solves |
| **S2 — QP with cone / CoP / torque limits** | Same tasks, (3)–(5) enforced. Dense QP, 41 vars, ~18 eq, ~30 ineq, warm-started → < 100 µs. | yes | hpipm dense-QP (already linked through `ocs2_sqp` → `hpipm_catkin`) **or** qpOASES / ProxSuite if present | +~400 lines | adds a hard bound on ankle torque via CoP — the actual saturation fix |
| S3 — swing-foot impedance + early/late touchdown | Measured contact (foot force or height) switches a foot's constraint set independently of the MPC's prescribed timing. | — | none | +~150 | what Phase 2 #2/#3 try to do in the plan, done in the servo layer |

S1 alone is a real experiment: it moves joint PD into the dynamics (so the ankle gets inertia-scaled
gains automatically) and adds the between-solve base feedback. If S1 shows no movement on
`tau_sat_pct` / `pelvis_acc_hf`, S2 is unlikely to either and the item closes cheaply.

## 4. Integration points (all found, nothing speculative)

- `compute_mpc_joint_action` (`controller.cpp` ~1784): after `evaluatePolicy`, branch on
  `control.feedforward == "whole_body"` → call the new layer with `observation.state`, `policy_state`,
  `policy_input`, `policy_mode`; it returns `feedforward` (= τ_WBC) and leaves `policy_position/velocity`
  as today (they still drive the residual PD).
- `apply_policy_swap_blend` continues to blend the written command — unchanged (the blend was the
  Phase 1 win and the WBC command is at least as continuous).
- Pinocchio: `control_pinocchio_` / `control_model_` already give `M`, `nle`, frame Jacobians via
  `pinocchio::crba / nonLinearEffects / computeFrameJacobian / getFrameJacobianTimeVariation`;
  `computeJointTorques` (`DynamicsHelperFunctions.h:332`) is the joint-row ID used for the output.
- Gains: new `control.wholeBody.*` block in `parameters.yaml` (+ G1/H2 configs, shipped `enabled: false`
  → identical behaviour until a sweep wins, like every Phase 1/2 lever). The reduced `jointEffortPd`
  for the WBC mode is a second gain set in the same block (`residualKp`, `residualKd`), swapped in
  only when the layer is active so the `make_config` A/B stays one flag.
- Diagnostics: `tracking` topic already logs `cmd_tau`, `pd_tau`, plan/obs base pose — the existing
  metrics (`legs.pd_tau_hf`, `legs.tau_sat_pct`, `base.*_track_err`, `walk.pelvis_acc_hf`) score it
  without new plumbing. Add `wbc.solve_us` and `wbc.cone_clip_count` for S2.

## 5. Sweep plan (`tests/tuning/run_wbc_sweep.sh`, after S1 builds)

- Baseline: shipped (PD kp 200, `feedforward: policy_torque`).
- `wb_s1`: WBC on, residual PD `kp 20 / kd 3`, task gains `K_p,b = diag(0, 0, 100, 200, 200, 50)`,
  `K_d,b = 2√K_p,b`, `K_p,j = 100` inertia-weighted, `W_b = 10`, `W_sw = 5`, `W_j = 1`, `W_F = 0.01`.
- `wb_s1_soft` / `wb_s1_stiff`: ×0.5 / ×2 on the base task gains.
- `wb_s1_pd50`: residual PD `kp 50` — tells us how much residual PD the model error needs.
- Primaries (vx 0.5): `walk.pelvis_acc_hf ≤ 0.92×`, `legs.pd_tau_hf ≤ 0.85×`, `base.roll_track_err_deg ≤ 0.85×`;
  default guards **plus** `legs.tau_sat_pct ≤ 1.0×` and `legs.track_err_q_deg ≤ 1.5× + 0.1` (joint
  tracking is expected to loosen — that is the point — but not blow up). Set B on `stops`.
- Falls at any stage end the candidate; a fall in `wb_s1` with soft gains ends S1 and the item.

## 6. Risks and what would stop it

- **Build / test**: cannot be compiled in the sandbox (no pinocchio). Each stage will be delivered with
  a pinocchio-free unit test of the linear algebra (`tests/wbc_formulation_test.cpp`, Eigen only, built
  under `BUILD_TESTING`) and a Python contract test on the config schema; the first real feedback is
  the user's `colcon build` + sweep, so S1 will be kept small and compiled-in but disabled by default.
- **Model mismatch**: WBC relies on `M, h` matching the MuJoCo model; the MPC already uses the same
  URDF for its RNEA constraints and walks, so the mismatch is bounded. The residual PD is the safety net.
- **Mode mismatch at touchdown** (foot lands early/late vs. the MPC's prescribed timing): S1/S2 take
  the MPC's mode, so a late foot gets a stance constraint in the air for a few ms — same as today's
  plan. S3 addresses it; it is also why S1's gains start soft.
- **Solve budget**: S1 is one 29×29 Cholesky and two small least squares per tick (≪ 50 µs). S2's QP
  is the only real-time risk; hpipm dense IPM on this size is well under 100 µs, but it must be
  measured (`wbc.solve_us`) before the gains are tuned.

## 7. Decision needed

1. Go/no-go on S1 (closed form, no new dependency, ~350 lines, ships disabled).
2. For S2, which dense QP is available in the workspace (checked by
   `ls /opt/ros/*/include ~/Documents/GitHub/humanoid_ws/install/*/include 2>/dev/null | grep -iE "hpipm|qpoases|proxsuite|osqp"`);
   hpipm is the default assumption because `ocs2_sqp` already links it.
