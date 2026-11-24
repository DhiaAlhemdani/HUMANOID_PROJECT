# Repository Analysis & Summary

**Repo:** `DhiaAlhemdani/HUMANOID_PROJECT` (checkout of `legged_robot_mpc_controller`)
**Branch:** `arena/47d87774-humanoid-project` ← `main` @ `4baf2ac` ("Workspace Directory Addition", 2026-11-23)
**Snapshot size:** ~50 MB, 423 files, single squashed import commit (upstream history not present locally)

---

## 1. What this repository is

A **ROS 2 (Jazzy) controller package** for **legged robot whole-body Model Predictive Control**, tested in **MuJoCo** simulation. It is the `ros2_control` implementation of a **whole-body inverse dynamics MPC (WB-ID MPC)** built on:

| Layer | Technology |
|---|---|
| Middleware / control | ROS 2 Jazzy, `ros2_control`, `controller_interface`, pluginlib |
| Optimization | **OCS2** (SQP backend only — DDP/SLQ/iLQR explicitly not supported) |
| Rigid-body dynamics | **Pinocchio** (RNEA, frames, Jacobians) |
| Simulation | **mujoco_ros2_control** |
| Codegen | **CppAD** libraries, cached per robot (`auto_generated/<robot>`) |
| Estimation (external) | `legged_robot_state_estimator` (separate repo, VCS-imported) |
| Teleop (external) | `legged_robot_teleop` (DualSense gamepad, separate repo) |

The MPC jointly optimizes floating-base/joint states and velocities, contact wrenches, generalized accelerations, and joint torques. Base dynamics are enforced over the whole horizon via RNEA; actuated-joint inverse dynamics and torque limits are enforced inside a configurable **initial torque window**. The formulation adapts Khazoom et al. (RA-L 2024) and Molnar et al. (RA-L 2025) to OCS2's SQP.

---

## 2. Supported robots

| Robot | `robotName` | Actuated DoFs | MPC-active DoFs | Nominal height | Notes |
|---|---|---|---|---|---|
| Unitree G1 | `g1` | 29 | 23 | 0.795 m | 6 wrist joints fixed in MPC model |
| Unitree H2 | `h2` | 31 | 25 | 1.02 m | head joints active; `torsoTiltCost` |
| Upkie (wheeled-biped) | `upkie` | 6 | — | kneeling spawn | rolling-contact constraints, 200 Hz MPC, stand-up/sit-down posture control |

Validated behaviors include velocity/height tracking, walking/trot/running gait scheduling (run ≥ 0.75 m/s command, H2 grounded running up to 2.0 m/s), uneven-terrain and stair locomotion, push recovery, arm joint-target tracking with arm-vs-body collision sphere penalties, and Upkie stand-up/drive over steps and ramps.

---

## 3. Code layout (~18.2k lines of C++; 76 Python files; 130 headers)

```
legged_robot_mpc_controller/
├── CMakeLists.txt            # ament_cmake, generate_parameter_library, GLOB over src/mpc/*
├── package.xml               # format-3 manifest; plugin export
├── legged_robot_mpc_controller.repos   # vcs import: mujoco_ros2_control, ocs2_ros2, estimator, teleop
├── Codex.md                  # 1,226-line engineering handoff / change log
├── include/legged_robot_mpc_controller/   # single public header root (mirrors src/)
├── src/
│   ├── controller/   (2,570 LOC)  # ros2_control plugin: controller.cpp (2,197), config_builder,
│   │                              #   generated params from src/controller/parameters.yaml
│   ├── common/         (462 LOC)  # ROS command adapters, config helpers, mpc targets parser
│   ├── mpc/common/   (6,162 LOC)  # models, costs, constraints, contacts, gaits,
│   │                              #   reference manager, swing-foot planner, targets
│   ├── mpc/wb_id/     (1,319 LOC) # WB-ID core: RNEA constraints (CppAD), torque/velocity limits,
│   │                              #   torque window, wheel contact, arm-body collision,
│   │                              #   initializer, dynamics, torso-tilt cost, OCP assembly
│   └── visualization/  (753 LOC)  # trajectories, robot ghosts, collision-sphere markers
├── config/             # per-robot ros2_controllers.yaml + initial_pose, terrains, objects, rviz, tests
├── description/        # URDF/xacro (robot + env), MuJoCo MJCF scenes, meshes (g1/h2/upkie), terrains
├── launch/             # robot.launch.py, GUI base-command, figure-eight & push-recovery scripts
├── tests/              # 48 test files: native gtest (gait, torque window, wheel kinematics, …)
│   │                   #   + large pytest suite (contracts, launch, scenes, package health)
│   │                   #   + tests/tuning/ analysis & plotting tooling
├── docs/               # formulation + implementation guides, per-feature notes, results reports
└── LICENSE
```

**Module → responsibility** (as documented in the README):

- `controller/` — ROS controller, configuration builder, generated parameter schema
- `common/` — command adapters and configuration helpers
- `mpc/common/` — robot models, costs, constraints, contacts, gaits, reference generation
- `mpc/wb_id/` — whole-body dynamics, inverse-dynamics constraints, initialization, OCP assembly
- `visualization/` — trajectories, robot ghosts, collision markers

### Key WB-ID components

- `WbIdRneaConstraintCppAd` — full RNEA dynamics constraint (base over horizon, joints in torque window)
- `WbIdJointTorqueLimitConstraint` / `WbIdJointVelocityLimitConstraint` — actuator limits
- `WbIdTorqueWindow` — configurable horizon window where joint ID + torque limits bind
- `WbIdFootAccelerationConstraintCppAd`, `WbIdWheelContactConstraintCppAd` — contact consistency (wheels: rolling contact)
- `ArmBodyCollisionConstraint` — soft polynomial sphere penalty (arms vs torso/pelvis/head; 96 pairs G1, 128 pairs H2)
- `WbIdInitializer`, `WbIdMpcInterface`, `WbIdDynamics` — warm start, OCP assembly, Pinocchio mapping

---

## 4. Architecture (runtime data flow)

```
GUI / joystick / scripted topic ──► WalkingVelocityCommand ──► reference manager
                                                                      │  gait schedule + swing-foot planner
MuJoCo ◄── effort commands ── ros2_control HW ──┐                      ▼
        ── joint states / base ──► estimator ──► state [q,v] ──► WB-ID OCP (OCS2 SQP @ mpcFreq)
                                     │                                   │
                                     └── TF odom→pelvis                  ▼
                                                     optimal inputs [W, a, tau_j] @ mrtFreq
                                                     → joint servo (inner loop) → torques
```

- **MPC frequency** default ~90–100 Hz (Upkie 200 Hz); **MRT loop** 1 kHz; MuJoCo physics/PD 2 kHz.
- State/input: `[q, v]` / `[W (contact wrenches), a (accelerations), tau_j (joint torques)]`.
- One controller plugin: `humanoid_whole_body_inverse_dynamics_mpc_controller` (plugin XML at repo root).
- TF: `world → odom` (static identity) → `pelvis` (estimator) → links (robot_state_publisher); environment published separately as `/environment/robot_description`.

---

## 5. Documentation strength

The repo is unusually well documented (~12 docs + 1,226-line `Codex.md` handoff log):

| Doc | Content |
|---|---|
| `docs/humanoid_whole_body_inverse_dynamics_mpc.md` | Full formulation + signal-flow diagrams |
| `docs/humanoid_whole_body_inverse_dynamics_mpc_impl.md` | Code-line/parameter mapping guide |
| `docs/h2_wb_id_mpc.md`, `docs/wheeled_biped_*.md`, `docs/wb_id_running.md` | Per-robot / per-feature integration notes |
| `docs/terrain_walking.md`, `docs/estimation_control_coupling.md` | Terrain pipeline; KF-vs-MPC stability analysis |
| `docs/results/*.md` | Regression, health-scan, ledge-drop reports with metrics |
| `docs/design_notes/wheeled_biped/` | Archived survey & checklist (working notes) |

---

## 6. Testing & quality posture

- **Native (gtest):** ~8 regressions — model settings/joint maps, gait swing height, running transitions, torque-window, wheel contact kinematics, arm-body collision Jacobians, swing-flight.
- **Python (pytest):** 100+ checks — parameter/cost/constraint/solver/time-grid *contract tests*, launch discovery, MuJoCo scene layout, terrain config sharing, package layout/health, push-recovery sequence, Upkie stand-up.
- **Analysis tooling** (`tests/tuning/`): bag→npz conversion, tracking/estimator/gait-regularity plots, run comparison, RViz/MuJoCo video rendering.
- **Known caveats (from `docs/results/`):**
  - G1 timing: new build passed motion checks but failed strict overrun-count parity vs old build (tail-latency spikes; unresolved — explicitly *not* to be "fixed" by retuning).
  - Repo-wide style lint (Flake8/PEP257/Uncrustify) still fails; functional checks pass.
  - `package.xml` license field still `TODO`.
  - No automatic fall recovery (falls latch damping on Upkie).

A prior **package health scan** fixed two real `reserve()`-then-index UB bugs (joint map, contact-frame helper), removed dead headers/caches, and declared missing direct dependencies.

---

## 7. Build & dependency notes

- Colcon workspace package; deps pulled via `vcs import` from `legged_robot_mpc_controller.repos` (forks of `mujoco_ros2_control`, `ocs2_ros2` + submodules, `legged_robot_state_estimator`, `legged_robot_teleop`).
- First launch per robot compiles CppAD codegen libraries (minutes), then cached.
- Typical build: `colcon build --packages-up-to legged_robot_mpc_controller --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release` (Release incremental ≈ 42 s; clean ≈ 9 min @ 2 jobs).

---

## 8. Strengths / risks

**Strengths**
- Research-grade, paper-traced MPC formulation with deep docs and contract-test culture.
- Clean single-header-root, module-mirrored layout; WB-ID-only after a deliberate cleanup pass.
- Multi-robot generality (G1, H2, wheeled-biped) sharing one controller plugin.
- Honest validation reporting (regressions, timing caveats, untested speed ranges).

**Risks / gaps**
- **Single squashed commit** in this checkout — no incremental history for review.
- Upstream is personal-research software: license undeclared in manifest, hard-coded user paths remain in README examples.
- External, fork-owned dependency stack (OCS2 fork, estimator, teleop) must be kept in step manually.
- Timing parity on G1 unresolved; running gait for G1's revised sequence not simulation-tested.
- No CI configuration visible in the repo.

---

## 9. One-paragraph summary

`HUMANOID_PROJECT` is a snapshot of **wei-hsuan-cheng's `legged_robot_mpc_controller`**: a ROS 2 Jazzy `ros2_control` package implementing **whole-body inverse dynamics MPC** (OCS2 SQP + Pinocchio RNEA + CppAD) for the **Unitree G1 and H2 humanoids** and the **Upkie wheeled-biped**, validated in MuJoCo. It contains ~18k lines of C++ organized into `controller / common / mpc/common / mpc/wb_id / visualization` modules, per-robot configs and descriptions, a large contract-test suite, and extensive formulation/implementation documentation plus a running engineering log (`Codex.md`). Entry point is `ros2 launch legged_robot_mpc_controller robot.launch.py robotName:=g1|h2|upkie`; the codebase's main caveats are an unresolved G1 timing-parity issue, style-lint failures, an undeclared license, and its reliance on several fork-owned external packages.
