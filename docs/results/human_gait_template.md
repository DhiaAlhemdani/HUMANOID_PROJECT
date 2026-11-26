# Step H — human gait template (increment 1, 2026-11-25)

Owner's target: the robot must mimic human movement — upper body, lower body, arms, legs. The
pelvis-smoothness campaign (steps A/B) cannot deliver that: its references are one fixed posture plus
a pelvis velocity, so the optimizer decides the limb motion by effort alone.

## What the shipped reference does to the kinematics

| | human | shipped G1 reference |
|---|---|---|
| stance knee | ~5–10° at mid stance | nominal 1.6 rad (92°) with weight 0.1 — the height reference (0.795 m ≈ straight leg) and the posture disagree, the robot walked at 0.747 m (knee ~40°) |
| hip range | ~40° at 1.3 m/s, ~30° at 0.5 | unreferenced (weight 0.05) |
| swing knee | ~60° | unreferenced |
| arms | swing opposite to the legs, ±15–25° | fixed posture (elbow 1.2 rad = 69° bend), weight 5; `armSwing` generator shipped **off** |
| trunk | counter-rotates the pelvis by a few degrees | waist weight 2, no reference |
| pelvis | ±4° obliquity, ±2 cm vertical | flat height reference |

## Built

- `reference.humanGait.*` (`SwitchedModelReferenceManager::addHumanGaitTemplate`, default off):
  normative sagittal hip / knee / ankle curves (Winter-type, 5 % tables), pelvis roll / height /
  pitch, waist yaw, shoulder / elbow swing. Each leg gets its own gait-cycle phase from the planned
  mode schedule (`getLegStridePhase`: heel strike → other toe-off → other heel strike → toe-off map
  to 0 / 0.10 / 0.50 / 0.60 of the human cycle), so the curves follow whatever step and
  double-support timing the gait law plans. Leg amplitude scales with commanded step length / leg
  length (0.65 m on the G1) over the human 0.78; arm amplitude with speed / 1.3 m/s. Standing, or
  backward / lateral / turning commands: the static stance posture (hip −k/2, knee k, ankle −k/2
  with `kneeOffset` k = 0.3 rad as the singularity margin; matching pelvis height 0.785 m).
- Human-likeness score in `score_suite.py` for forward-walking holds (`hl_leg`, `knee_ms`,
  `hip_rom`, `knee_sw`, `arm_amp`, `arm_ph`), shown by `compare_suite.py`; the Python phase port is
  tested against the C++ logic on synthetic schedules (`tests/tuning/human_gait.py`).
- `make_wrench_configs.sh … human`: template on, leg weights hip 10 / knee 10 / ankle 3 (running
  cost only; the terminal costs track the raw posture), elbow −0.4 rad, pelvis 0.785 m.

## First candidates

`humanH` = `human,arms,waist` on the 1e-4 / scale 1.0 force-rate setting (the step A winner), and
`humanHF` = the same on 1e-3 / scale 0.05 (best pelvis numbers, lagging). Gate: no falls,
`hl_leg` and `knee_ms` move toward the human, arm swing visible (`arm_amp`, `arm_ph` ≈ 0), owner's
video verdict on the whole body.

Known limits of increment 1: flat-foot contact (no heel-to-toe roll yet), the stance knee is a
0.3 rad margin away from the human, the swing-foot task (5 cm clearance) and the leg curves are
two references the weights have to reconcile, and the curves are at human proportions — a
mocap-retargeted clip is the next data source if the owner wants a specific gait.

## Run 1 — `humanHV` (2026-11-26, owner's machine)

`humanHV` = `0.0001 1.0 human,arms,waist,vel`, HEIGHT 0.785, leg weights 10 / 10 / 3, pelvis template on.
(A first attempt was invalid: the generated yaml had a mixed int/double list, which the rcl YAML
parser rejects, so the simulator never started — `make_config.py` now refuses such files.)

| hold | knee_ms ° | hip_rom ° | knee_sw ° | arm_amp ° | arm_ph % | step_l m | v_err | jerk | pitch_pp ° |
|---|---|---|---|---|---|---|---|---|---|
| 0.2 m/s | 19.8 (base 33.7) | 36.7 (42) | 53 (57) | 4.7 (1.1) | 6 | 0.24 (0.25) | 0.155 (0.10) | — | 9.3 (first ramp) |
| 0.5 m/s | 23.2 (36.4) | 40.7 (50) | 55 (60) | 5.8 (1.2) | 7 | 0.28 (0.29) | 0.116 (0.12) | — | — |
| 0.8 m/s | 31.6 (42.4) | 48.8 (62) | 58 (62) | 8.4 (1.6) | 6 | 0.39 (0.38) | 0.171 (0.16) | 248 (88.7) | 33.7 |
| stop | | | | | | | | 32 (0.2) | 4.2 |

Speeds suite survived; the omni suite fell at 22.5 s on `ramp → vx 0.2 vy 0.1` right after the
0.6 m/s hold (pitch_pp already 33.7°, MPC equality SSE mean 62 / max 3000; the speeds run had
0.18 / 5.1). Reading: up to 0.5 m/s the template does what it should — stance knee halved, arm
swing ×4 with the right phase, step length and tracking kept — but from 0.6 m/s the pelvis pitches
±17° and the solver's constraints stop being met.

Root cause found in the template, not the weights: the hip (and ankle) tables are angles relative
to a human trunk that leans ~10° forward while walking; their cycle mean is +13° flexion. The G1's
pelvis is held level by the base-pose cost, so asking both hips for 13°·scale of extra flexion
through the whole cycle can only be satisfied by pitching the pelvis forward — the harder the
weights and the larger the scale (0.87 at 0.8 m/s vs 0.57 at 0.2), the stronger the pitch
fight, which is exactly the speed-dependent pitch oscillation seen. Fix (`removeHipMean`, default
on): hip and ankle curves are applied as motion about the stance posture (mean removed), knee stays
absolute. Side effect: the standing posture and the walking mean now coincide, so walk ↔ stand
transitions no longer step the hip reference.

Also for run 2 (`make_wrench_configs.sh human` defaults): leg weights 5 / 5 / 2 (softer, the knee
curve still reaches the robot), pelvis template (`base`) off until the legs are quiet, and the
continuous gait's swing clearance 7–9 cm instead of 5–6 cm so the swing-foot task and the 60° swing
knee ask for the same leg instead of two different ones. Overridable with `HUMAN_LEGW`,
`HUMAN_BASE=true`, `HUMAN_SWING="z1 z2 z3 z4"`.
