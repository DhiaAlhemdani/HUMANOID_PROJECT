# Phase 2.D — lateral (DCM / LIPM) pelvis reference

Status: **recon done, measurement step pending** (2026-11-25). Roadmap Phase 2 #5. Aimed at the two
numbers nothing has moved cleanly: steady-walk roll and the start/stop roll–pitch transient
(2.A capture-point footholds lost on both; `docs/results/capture_point_sweep.md`).

## 0. What the plan asks of the pelvis sideways today (code facts)

| fact | where |
| --- | --- |
| The velocity-command reference is the current pelvis pose re-anchored each solve, integrated forward with the **commanded** twist; base velocity reference = commanded `[vx, vy, 0, yaw_rate, 0, 0]` at three knots (0, 0.7 H, H) | `src/mpc/wb_id/command/WbIdMpcTargetTrajectoriesCalculator.cpp` `commandedVelocityToTargetTrajectories` |
| `BaseMotionTrackingCost`: x, y, yaw **position are gauge** in relative-twist mode (not tracked); z, roll, pitch position tracked; horizontal velocity tracked **in the pelvis frame**, i.e. lateral velocity is pulled to the commanded `vy` = 0 in a straight walk | `src/mpc/common/cost/BaseMotionTrackingCost.cpp:40–90` |
| Weights: `basePose [0 0 50 0 100 100]`, `baseVelocity [30 15 5 30 30 30]` → lateral velocity weight 15, roll 100 / roll-rate 30 | `config/g1/ros2_controllers.yaml:369–370` |
| `SwitchedModelReferenceManager::getDesiredState` is the one place the reference is modified with knowledge of the mode schedule (arm-swing offsets, phase variable from `eventTimes`) | `src/mpc/common/reference_manager/SwitchedModelReferenceManager.cpp:158` |
| Trot at 0.5 m/s: `[LF, RF]`, 0.5 s per single support, no double support | `config/g1:754` |
| A DCM already exists in the code — for footholds only (2.A, `captureFootPlacement`, shipped off) | `SwitchedModelReferenceManager.cpp:344–461` |

**So no lateral motion of the pelvis is planned at all.** The reference says "move straight, lateral
velocity 0", and the solver finds the cheapest compromise between that cost and the single-support
feasibility (wrench cone / centre of pressure inside one foot): some sway it is charged for, plus
ankle-roll and hip-roll torque, plus body roll. The physics of alternating single support wants a
lateral pendulum: a linear inverted pendulum at pelvis height z = 0.79 m (ω = √(g/z) ≈ 3.5 /s) with
0.5 s supports and a 0.2 m step width approaches the stance foot to ≈ 0.07 m at mid-stance and
crosses the switch at ≈ 0.25 m/s laterally — a ≈ 3 cm sway per step that costs no torque. Charging
for that velocity at weight 15 is the lever 2.D pulls: give the reference the pendulum's lateral
velocity, synchronised with the mode schedule, so the cost stops fighting the gait's own mechanics.

## 1. Measurement first (no sim run needed)

`tests/tuning/lateral_anatomy.py <case>` on the existing walk logs (`so_base`, `fk_base`) prints,
over the steady walk: the plan's lateral velocity reference (expect 0), the observed lateral velocity
and pelvis-vs-stance-foot offset binned over the single-support phase, the LIPM prediction for the
same footholds, and what pays for the difference — stance ankle-roll / hip-roll torque, roll and
roll-rate RMS, and the 4–40 Hz lateral pelvis acceleration (candidate metric `pelvis_lat_acc_hf`).

Three outcomes decide the item:

1. **Observed sway ≈ LIPM sway** already (the cost is weak enough): the lever is inert; 2.D closes
   without C++ and the remaining roll is elsewhere (foot roll compliance, swing-leg reaction).
2. **Observed sway ≪ LIPM sway and the stance ankle-roll torque carries the difference**: implement
   §2; primary gates are walk roll RMS and stance ankle-roll torque, `acc_hf` not-worse.
3. **Observed sway ≫ LIPM sway** (the solver is over-swaying to buy margin): the reference still
   helps, but the sign of the expected roll change flips; gates as in 2.

## 2. Implementation sketch (only after §1 lands in outcome 2 or 3)

In `SwitchedModelReferenceManager::getDesiredState`, next to the arm-swing offsets: for a
single-support mode at `time`, with stance foot position from the foothold plan and the LIPM
constants, add the pendulum's lateral velocity (pelvis frame, rotated to world with the reference
yaw) to the desired base velocity, and optionally the lateral position offset to the desired pose
(inert while x/y are gauge, harmless otherwise). Config block `reference.lateralSway`:
`enabled`, `amplitudeScale` (1 = LIPM, swept 0.5 / 1.0 / 1.5), `heightOverride` (−1 = measured
pelvis z). Double support / stance: velocity reference 0 (unchanged). Start/stop: the first and last
supports get the pendulum law too, which is where the stops' roll transient should move.

Cost is a few dozen lines in the reference manager, a config block, a contract test and
`run_lateral_sweep.sh`; no change to constraints, dynamics or the WBC layer.

## 3. Gates

Primaries (any one, others not-worse by the standard guards): walk `roll_rms_deg` ≤ 0.85×, stance
ankle-roll torque RMS ≤ 0.85×, stops `transient_roll_max` ≤ 0.85×. Guards: standard set
(`acc_hf` ≤ 1.10×, `z_sd`, `q_track`, `vx`, `solve_hz`, falls disqualify) plus `vy_mean` drift
≤ 0.02 m/s (a lateral reference must not walk the robot sideways).
