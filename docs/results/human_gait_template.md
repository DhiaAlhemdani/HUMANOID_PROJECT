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

## Run 3 — `humanHV3` (increment 2)

Omni **survived** throughout (0.3 m/s, turn, 0.6 m/s, diagonal, backward, stops); speeds fell at
15.4 s on the ramp to 0.8 m/s. The low end is now better than the baseline on every axis:

| hold | jerk (base) | acc_hf (base) | knee_ms | hip_rom | arm_amp | v_err |
|---|---|---|---|---|---|---|
| 0.2 m/s | 42.7 (47.0) | 0.61 (0.94) | 18.9 (33.7) | 40 (42) | 4.7 (1.1) | 0.15 (0.10) |
| 0.3 m/s | 41.1 (47.8) | 0.65 (0.90) | 18.8 (34.0) | 39 (42) | 4.6 (1.1) | 0.08 (0.11) |
| 0.5 m/s | 142.7 (56.6) | 1.38 (1.02) | 19.1 (36.4) | 42 (50) | 5.8 (1.2) | 0.14 (0.12) |
| 0.6 m/s | 96.5 (65.6) | 1.06 (1.04) | 26.8 (38.6) | 49 (56) | 6.8 (1.4) | 0.13 (0.14) |
| 0.2/0.1 after 0.6 | 198 (48) | 2.36 (0.90) | | | | 0.18 (0.16) |

Pattern: the template is a clear win up to 0.3 m/s, costs smoothness from 0.5 m/s, and does not
survive the ramp to 0.8. The segment after the 0.6 m/s hold stays rough (leg jerk 2925, the robot is
still recovering through the following stop), as in run 1 where that ramp was the fall. Next: two
isolation runs on the speeds script — `HUMAN_MAXSCALE=0.6` (amplitude frozen at the 0.45 m/s value)
and `HUMAN_LEGS=false` (arms / waist / pelvis only) — to attribute the high-speed loss to the leg
amplitude before designing a speed taper.

## Isolation runs (speeds script)

| run | legs | leg cap | swing knots [cm] | 0.2 hold jerk | 0.5 hold jerk | 0.8 m/s |
|---|---|---|---|---|---|---|
| baseline `human` | – | – | 5/5/5/6 | 47.0 | 56.6 | 88.7, survives |
| HV3 | on | 1.2 | 6/6.5/9/11 | 42.7 | 142.7 | fell on the ramp |
| HV3s | on | 0.6 | 6/6.5/9/11 | 52.1 | 78.2 | fell in the hold |
| HV3n | off | – | 6/6.5/9/11 | 38.6 | 46.3 | fell in the hold |
| **HV3nw** | off | – | **5/5/5/6** | **43.2 / 37.1** | 187.6* / **43.7** | **66.4, survives** |

\* one stumble-and-recover episode in the first 0.5 m/s hold (step_l 0.37, leg jerk 2546); the
second 0.5 m/s hold of the same run is 43.7.

The 0.8 m/s falls were the swing clearance I raised to match the swing-leg curves (9 cm apex in a
0.43 s swing), not the template: with the shipped 5 cm knots the upper-body template (arms, waist,
pelvis) survives the whole speeds script and is smoother than the baseline at every hold, 0.8 m/s
included (66 vs 89), with arm swing 4.8° at 0.2 m/s to 8.5° at 0.8 m/s. The recipe default is back
to the shipped knots. Solve rate in that run was 38.5 Hz (baseline 45.9) — to be checked on a repeat;
the other variants were at 43–44 Hz.

Open: `humanHV3w` (full template, shipped swing) decides whether the leg curves can ride along at
0.8 m/s or need a speed taper; A vs HV3 at 0.5 m/s (78 vs 143) suggests the amplitude matters there,
but the run-to-run spread at 0.5 m/s (HV3nw: 188 then 44 within one run) is as large as that gap.

`humanHV3w` (full template, shipped swing): 0.2 hold 52.1 (base 47.0, knee_ms 19.8, arm 4.7°),
0.5 hold 79.1 (56.6, knee_ms 23.5), fell in the 0.8 m/s hold (270, v_err 0.59). With the stance
side made consistent (increment 2), what is left to disagree is the swing side: the human swing knee
implies a foot clearance of 5.3 cm at scale 0.57 (0.3 m/s) — the planner's 5 cm, hence the clean
low-speed runs — but 7.5 cm at 0.71 (0.5 m/s) and 10.4 cm at 0.87 (0.8 m/s), against a hard 5 cm
swing-foot constraint; and the planner cannot be raised (the isolation runs).

## Increment 3

`swingClearance` (0.05 m, default on): the swing-knee curve is scaled down (bisection on the planar
FK, four swing phases) so the template's own clearance peaks at the planner's value. No change below
0.4 m/s (factor 0.97 at scale 0.57); 0.83 at 0.5 m/s, 0.68 at 0.8 m/s — the swing knee peaks at 54°
instead of 71°. Run 4 = `humanHV4`, speeds + omni.

`humanHV4`: worse, not better — fell on the 0.8 ramp again and, new, in the omni turning hold
(0.3 m/s + 0.4 rad/s, jerk 367) which HV3 passed at 60; 0.2 hold 67.7 (HV3 42.7, HV3w 52.1) although
the swing-knee factor is 0.97 there; solve rate 37.3 Hz (HV3 42.0, baseline 45.5). Two readings:
(1) the swing-knee solve ran a 16-step bisection inside the per-node reference evaluation — moved
into a 65-entry table built once in `setHumanGaitSettings`; (2) the leg template with weights
10/10/3 is marginal even at 0.3 m/s once the MPC has to deviate from the forward pattern (turn,
diagonal), and the spread between repeats at the same speed (42.7 / 52.1 / 67.7 at 0.2 m/s;
188 → 44 inside one run at 0.5) is now as large as the effects being tested. Stiff posture weights
leave the solver no room to place feet for recovery; the human curves should bias the posture, not
dictate it.

Plan: validate the upper-body template alone across omni (`humanUB` = `HUMAN_LEGS=false`, the D
configuration, already 0 falls and −18 % jerk on speeds), and repeat the full template with soft leg
weights (`humanHV4L` = `HUMAN_LEGW="3 3 1"`).

## Run 5 — `humanUB` and `humanHV4L`

**`humanUB` (arms / waist / pelvis template, no leg curves): 0 falls in speeds and omni, smoother than
the baseline on every walking segment** — 0.2 m/s 39.5 (47.0), 0.3 m/s 34.8 (47.8), turning 35.9
(51.1), 0.5 m/s 42.6 (56.6), 0.6 m/s 65.3 (65.6), 0.8 m/s 67.6 (88.7), diagonal 44.1 (48.1), backward
42.6 (48.5); transitions likewise (max 69.0 vs 73.6); acc_hf −25 % to −35 %; tracking equal or better
(v_err max 0.195 vs 0.267); arm swing 4.7° (0.2 m/s) to 8.5° (0.8 m/s), in phase; 41.8 Hz. One
defect: the stop holds are not quiet (jerk 51 / 17 / 33 against 0.2 / 1.0 / 0.3 for the baseline;
`humanHV3nw`, the same configuration, had 0.8 once). Cause in the reference: at the last heel strike
`getLegStridePhase` went invalid in one step, so the arm swing (cos = ±1 at that moment, 0.1 rad at
weight 50), waist yaw and pelvis roll snapped to zero; the same step happened at the first toe-off.
Fix: `StridePhase::fade` — amplitude ramps in over the first 0.4 step times of the first swing and out
over 0.4 step times after the last heel strike (posture held), applied to legs, arms, waist and pelvis.

**`humanHV4L` (full template, leg weights 3/3/1): 2 falls** — 0.8 m/s ramp and the 0.6 m/s ramp in
omni, with the 0.3 m/s hold after the turn already at 234. Softer weights did not help the legs; the
low-speed knee is at 17–22° as before. Decision: the leg curves stay experimental (opt-in,
`HUMAN_LEGS=true`), the recipe default becomes the upper body (`HUMAN_LEGS=false`). The human-like
lower body needs a different mechanism than a joint-space reference against a flat-foot MPC —
heel-to-toe contact or a learned residual — and is parked as the next design item.

## Run 6 — `humanUB2` + repeats

`humanUB2` (fade in): stops quiet again (3.8 / 4.0; baseline 1 / 0.3), omni as before, **but the
0.8 m/s hold fell** (192, v_err 0.58). Repeats of the same yaml on the speeds script: `humanUB2r1`
survived (0.8 hold 73.6, stop 33.8), `humanUB2r2` survived (0.8 hold 67.5, stop 8.4).
`humanUBnb` (pelvis template off) fell on the ramp to 0.8 (430). Tally for the upper-body template at
0.8 m/s: 4 survived (66, 68, 74, 68) / 1 fell; the pelvis part is not the margin. Stops: 3.8, 4.0,
34, 8 — the fade removed the systematic snap, what is left varies run to run.

A 1-in-5 fall rate at the baseline's own roughest segment cannot be resolved by more A/B runs; the
next step is anatomy. `tests/tuning/fall_anatomy.py <case>` prints the first-anomaly table, a
timeline (mode, base pitch / roll / height error, feet, top torques and tracking errors), peak torque
per joint against the MJCF limits and, with `mujoco` importable, the arm-body sphere clearance
(the polynomial penalty is active below minimumDistance + delta = 25 mm; the nominal clearance of
the upper arm to the torso is 29 mm at shoulder pitch 0, i.e. 4 mm of margin before the arm swing
starts paying a 6000-curvature penalty — the current prime suspect).
