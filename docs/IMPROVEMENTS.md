# Recommended Improvements for `legged_robot_mpc_controller`

Prioritized list derived from a full-repo scan (2026-11-24). Effort labels: **S** small (<½ day), **M** medium (1–3 days), **L** large (1+ week).

---

## P0 — Correctness & safety

| # | Improvement | Why | Effort |
|---|---|---|---|
| 1 | **Investigate the G1 timing overrun / tail-latency regression** (documented in `docs/results/wb_id_layout_regression.md`) | The only known failed acceptance check. Profile scheduling vs. compute tail (CPU affinity, solver SQP iteration budget, CppAD eval spikes) rather than retuning the controller. | L |
| 2 | **Add a real-hardware safety layer before any real-robot run**: command watchdog, independent software E-stop topic/service, actuator fault handling, controller lifecycle (deactivate → damping) that survives process death | Explicitly flagged as a P0 gap in `docs/design_notes/wheeled_biped/upkie_hardware_validation_handoff.md`; the current joystick watchdog is best-effort only. | L |
| 3 | **Replace `assert()`-based native tests with a real framework (gtest/Catch2)** | All 8 native regressions use C `assert` (45 sites, 0 gtest macros). `assert` is compiled out under `NDEBUG` — the CMake `-UNDEBUG` workaround is fragile and gives no failure reporting, filtering, or death-test support. | M |
| 4 | **Wire the Python suite into `colcon test`** | 39 pytest files exist but CMake registers only the 8 native `add_test`s — no `ament_add_pytest_test`/`launch_testing`. Today the majority of the contract suite only runs if someone invokes pytest manually, so CI can't gate on it. | S |
| 5 | **Fix `package.xml` license metadata** | `<license>TODO: License declaration</license>` while `LICENSE` is MIT. Set `<license>MIT</license>` (and confirm copyright holder matches intent). | S |

---

## P1 — Reproducibility & engineering hygiene

| # | Improvement | Why | Effort |
|---|---|---|---|
| 6 | **Pin dependencies to SHAs in `legged_robot_mpc_controller.repos`** | All six repos track floating branches (`main`, `jazzy`, `ros2_x86`). A break in the OCS2/estimator forks silently breaks builds. Pin commits; add a documented bump procedure. | S |
| 7 | **Add CI (GitHub Actions)**: `rosdep` + `colcon build` on `ros:jazzy`, `colcon test`, lint gates, plus a headless MuJoCo smoke test | Repo has no `.github/` at all. Even build+pytest-only CI would catch most regressions documented by hand in `Codex.md`. | M |
| 8 | **Add `.gitignore`** | The file is absent. The health scan removed committed `__pycache__`/`.pytest_cache` and worries about `auto_generated/` — nothing prevents them from coming back. | S |
| 9 | **Add `.clang-format` + `pre-commit` and burn down the style debt** | Full ament lint currently **fails** on Flake8/PEP257/Uncrustify; compiler still emits unused-parameter, signedness, and init-order warnings. Format once, then enforce incrementally in pre-commit. | M |
| 10 | **Remove hard-coded user paths from README** | `cd /home/alhemdani/Documents/GitHub/humanoid_ws` and `source /home/alhemdani/install/setup.bash` are personal leftovers in the run instructions. | S |
| 11 | **Bump `version` from `0.0.0` and add a `CHANGELOG.md`** | Version 0.0.0 with a 1,226-line `Codex.md` as the only history. A short changelog (even generated from commit subjects) makes releases trackable. | S |

---

## P2 — Code quality & architecture

| # | Improvement | Why | Effort |
|---|---|---|---|
| 12 | **Split `src/controller/controller.cpp` (2,197 lines)** | One file holds activation/bring-up, observation building, gait handoff, stand-up state machine, command writing, visualization hooks, and solver threading. Natural seams: `bring_up`, `state_estimation_io`, `command_output`, `solver_thread`. Testability and reviewability both improve. | L |
| 13 | **Replace polling sleeps with `condition_variable`/events in the solver thread** | `start_deferred_solver_thread` and friends spin on `sleep_for(1–2 ms)` loops; the activation path busy-waits 10 ms for joint states. CV notify removes latency jitter and CPU wakeups (relevant to #1). | M |
| 14 | **Audit real-time safety of the command path** | `realtime_tools` is used for the mode schedule, tracking publisher, and performance snapshots, but subscriber callbacks and several shared flags use plain atomics/mutexes. Document/enforce the RT contract: no allocations, no locks, no logging in `update()`. | M |
| 15 | **Promote hard-coded constants to parameters** — e.g. `footholdForwardBias = 0.06` in `TerrainFootholdPlanner.cpp:267` (the repo's only `TODO`) | One-line TODO in source; consistent with the repo's `generate_parameter_library` discipline. | S |
| 16 | **Tighten config validation in `config_builder.cpp`** | Validation exists but is per-field; consider a single schema-level validation pass (ranges, cross-field invariants like `sqp.dt ≤ timeGrid.maxStep`) reported all-at-once with full error lists instead of first-throw. | M |
| 17 | **Add code coverage measurement** (`colcon test --event-handlers console_direct+` + lcov/gcovr) | No coverage tooling exists; with 18k LOC it's unclear which paths (error branches, Upkie-only code) are exercised. | M |

---

## P3 — Testing depth & validation

| # | Improvement | Why | Effort |
|---|---|---|---|
| 18 | **Add `launch_testing` integration tests** for `robot.launch.py` | `tests/test_robot_launch.py` only static-checks launch files; a bring-up smoke test (controller activates, MPC reports ready, no ERROR logs) in CI would catch wiring regressions. | M |
| 19 | **Property/randomized tests for dynamics helpers** | The arm-body test already does 100 randomized Jacobian checks — extend that pattern to RNEA constraint consistency (analytic vs. numeric), joint-map round trips, and torque-window edge cases. | M |
| 20 | **Formalize the benchmark harness** (`tests/tuning/`, `wb_id_tuning_benchmark.md`) into a runnable script with stored baselines | The tuning benchmark doc describes round-based methodology; packaging it as `tests/run_benchmark.sh` with checked-in baseline JSON makes A/B comparisons of MPC changes one command. | M |
| 21 | **Sim-to-real gap checklist as executable tests** | Docs repeatedly list what is *not* validated (transmission coupling, actuator bandwidth, comms delay, mixed-axis pushes). A documented hardware-readiness checklist with pass/fail ownership would convert prose into a gate. | S |

---

## P4 — Documentation & project structure

| # | Improvement | Why | Effort |
|---|---|---|---|
| 22 | **Add `docs/README.md` as a documentation index** | 12+ docs with no table of contents linking formulation → implementation → per-robot → results. | S |
| 23 | **Retire `Codex.md` into git history / dated changelog entries** | 138 KB handoff log is valuable but unstructured; split into `docs/history/` by date or convert to CHANGELOG so the repo root stays clean. | S |
| 24 | **Resolve upstream attribution** | README credits `wei-hsuan-cheng/legged_robot_mpc_controller`; this checkout is `DhiaAlhemdani/HUMANOID_PROJECT` with a squashed single commit. Preserve upstream history/remote link so fixes can flow back and provenance stays clear. | S |
| 25 | **Document the estimators/teleop cross-repo contract** | `legged_robot_state_estimator` and `legged_robot_teleop` are runtime-critical but version-coupled only by branch name. A short interface doc (topics, msg types, expected rates) in `docs/` would ease the pinning done in #6. | S |

---

## Suggested sequencing

```
Week 1:   #5 #8 #10 #15 #22  (quick wins, zero risk)
Week 2:   #4 #6 #3           (test gating + reproducibility)
Week 3-4: #7 #9              (CI + lint burn-down)
Then:     #1 (timing investigation) and #12 (controller split) in parallel tracks
Ongoing:  #2 before any real hardware; #18–#21 as validation depth grows
```

## Explicitly *not* recommended

- **Retuning MPC parameters to silence the G1 timing comparison** — the layout-regression report explicitly warns against masking a scheduling/latency issue with tuning changes.
- **Rewriting the SQP backend or adding DDP/iLQR** — out of scope by design; the README states OCS2 SQP is the supported path.
- **Deleting `docs/results/` or `Codex.md` history** before it has been folded into a changelog (see #23).
