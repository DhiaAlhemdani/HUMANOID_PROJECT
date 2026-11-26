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

First diagnosis (commit `3000088`, wrong): "the hip curve's +13° mean is a trunk lean, remove it".
Run 2 (`humanHV2`: mean removed, leg weights 5/5/2, pelvis template off, swing 7–9 cm) was worse —
fell at 10.8 s on the ramp to 0.5 m/s and at 20.2 s in omni; 0.2–0.3 m/s still good (knee_ms 16–17°,
arm_amp 4.7°). The mean is not a trunk lean: during flat-foot stance (10–40 % of the cycle) the raw
curves satisfy hip − knee + ankle ≈ 0 to within 2°, i.e. they are already consistent with a level
pelvis and a flat foot; removing 13°·scale of hip flexion broke that closure in every stance and the
pelvis had to pitch to make up for it. Reverted.

What actually conflicts (planar FK of the G1 leg, 0.3366 / 0.300 m, pelvis level):

| phase | human foot | raw template asks the stance foot to pitch | implied pelvis height vs standing |
|---|---|---|---|
| 0–10 % (heel strike → foot flat) | heel only | −14° (scale 0.57) … −22° (0.87) | −2.8 … −6.1 cm |
| 15–40 % (foot flat) | flat | < 3° | −2 … −0.3 cm |
| 45–60 % (heel off → toe off) | heel up | +6 … +28° (0.57), +10 … +43° (0.87) | −2 … −7 cm |

A flat-foot robot cannot pitch the stance foot (6-row contact constraint), and the base-height cost
holds a fixed 0.785 m. So around every heel strike and every toe-off the hip/knee/ankle references,
the contact constraint and the height reference disagree by tens of degrees and several cm — growing
with scale, i.e. with speed. That is the speed-dependent pitch oscillation of run 1.

## Increment 2 (commit after `2084f8b`)

- `stanceAnkleClosure` (default on): in stance the ankle reference is −hip − knee (sole flat under a
  level pelvis); the stance knee is capped so the closed ankle stays within `maxStanceDorsiflexion`
  (0.44 rad: the trailing leg stays straighter instead of flexing 45° with the heel down). Blended
  over the first and last 10 % of swing, so references are continuous at contact changes.
- `heightFromLegs` (default on, with `base`): the pelvis height reference is the stance legs' template
  through planar FK, as an offset from the standing posture, weighted over both legs in double
  support with the same fades. p-p 2 cm at scale 0.57, 3.6 cm at 0.87, lowest in double support,
  highest at mid stance — the human pattern, and now what the legs ask for.
- Recipe defaults back to leg weights 10/10/3, pelvis template on; swing clearance knots
  0.06/0.065/0.09/0.11 m = the ankle height the swing-leg curves imply at 0/0.45/0.8/1.15 m/s.
- Python mirror: `human_gait.leg_template_rad(..., stance_ankle_closure, max_stance_dorsiflexion)`,
  `leg_height`, `pelvis_height_offset`.

Run 3 = `humanHV3`, same recipe string. Gate as before; the specific expectation is pitch_pp at
0.6–0.8 m/s back under 10° and no fall in omni.
