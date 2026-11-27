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

### Run 6 anatomy: the 0.8 m/s falls are a controller defect, not a template effect

Tools: `tests/tuning/fall_anatomy.py` (window report), `touchdown_table.py` (one row per heel strike,
62 strides at 0.8 m/s over 7 runs), `extract_window.py` (CSV slices of five strides). Findings, in the
order they were established:

1. Not the estimator: estimate vs ground truth within 1.0 deg pitch / 0.3 deg roll / 0.5 cm through both
   onsets. The pitch step is real: TRUE pitch rate 354 and 665 deg/s (survivor max 66).
2. Not a reference jump, not a solve gap: the large `cmd_q` steps come 50-80 ms *after* the onset;
   policy intervals 22-24 ms median, 32-34 ms max, same as the survivors.
3. Not a hard landing: every 0.8 m/s stride lands at 0.15-0.18 m/s; the normal stride has the foot on
   the ground 49 +- 22 ms *before* the planned switch (baseline 67 ms).
4. The three strides that pitched >= 16 deg (UB2 35.25, UBnb 33.15 - both fell - and UB2r2 38.62, a
   near-miss) share two features no harmless stride combines: the foot still airborne at the planned
   switch (`lead_ms` 0-4) *and* a 38-45 Nm waist-roll torque spike in the 80 ms before it. Either alone
   is harmless (5 late strides, 3 spike strides, pitch max <= 5.7 deg).
5. The CSV slices show what the spike is. In the 30 ms fade before the switch (`wbc/contact_alpha`
   -> 0) the command is the plan's torque. Within the torque window that is the MPC policy torque and
   `cmd_tau == plan_tau` (HV3nw 39.25: the MPC itself planned -40 Nm waist roll, the robot executed it,
   nothing happened). But `torqueConstraintNodes: 3` with `dt 0.02` gives a window of (3-0.5)*dt = 50 ms
   that the interpolation check can only honour up to 40 ms of policy age, and at 41-42 Hz solve rate
   (the template costs 3-4 Hz) the policy is 43-60 ms old on roughly a third of the ticks. On those the
   feedforward is the fallback: the plan's inverse dynamics at the planned state, with planned joint
   accelerations that no joint row bounds beyond the window. Across a late touchdown that RNEA reaches
   the actuator limits - waist roll 50/50, right hip roll 139/139, hip pitch 88/88 - and the fade hands
   exactly that to the robot. In UB2 the next policy arrives at 35.241 (age 32 ms) and the command
   collapses from +50 to -1.7 Nm within one tick; the pelvis was already pitching at 350 deg/s.

Fix (`control.wholeBody.holdOnTorqueWindowOverrun`, default true, shipped on): on a torque-window
overrun tick the contact fade is skipped and the whole-body layer's torque is kept - it is contact- and
limit-consistent at the measured state. Alternative knob for comparison: `torqueConstraintNodes: 4`
(window 70 ms, effective 60) makes the fallback rare at the cost of one more constrained node.
The baseline `human` run is exposed to the same defect (its ages are shorter at 45.5 Hz), which is
consistent with its 0.8 m/s hold being its roughest segment (jerk 88.7).

## Run 7 — `humanUB3` x3: hold on torque-window overruns (`d2dc4f8`)

Same upper-body yaml, speeds script, three repeats. **0 falls in 3** (before: 1 fall in 5 holds plus the
pelvis-off ramp fall). Hold jerk 0.2 / 0.5 / 0.8: 35.4 / 47.0 / 81.8, 36.7 / 47.3 / 84.6, 36.2 / 46.3 / 79.8;
final stops 0.18 / 3.7 / 1.6; solve 43.7 / 40.5 / 41.5 Hz; v_err 0.19. UB3 had one rough ramp to 0.8
(91.0 / tau-rate 481, a guard fail); r2 is a clean winner (62.4 ramp, 79.8 hold).

The price: the 0.8 m/s hold is ~15 % rougher than the surviving pre-fix runs (80-85 vs 66-74; baseline
88.7). On overrun ticks the fade is now skipped entirely, so the support transient the fade was added
for (2.B sweep 3) comes back on those ticks. The hold removes the fall; it does not remove the cause,
which is the policy being older than the 40 ms the window honours on ~12 % of ticks (5574 overruns in
the UB2 log). Next: `torqueConstraintNodes: 4` (window 70 ms, effective 60) on top of the hold, so the
fade sees the real policy torque on most ticks and the hold only catches the rest.

## Run 8 — `humanUB3n4` x2: hold + `torqueConstraintNodes: 4`

| | n4 | n4r1 | UB3 (hold only, x3) | pre-fix survivors | baseline |
|---|---|---|---|---|---|
| hold 0.2 / 0.5 | 34.5 / 40.0 | 35.1 / 42.5 | 35-37 / 46-47 | 38-39 / 41-49 | 47.0 / 56.6 |
| hold 0.8 | **66.5** | 77.0 | 80-85 | 66-74 | 88.7 |
| ramp -> 0.8 | 71.7 | 63.2 | 62-91 | 60-125 | 73.6 |
| final stop | 0.88 | 0.19 | 0.2-3.7 | 3.8-34 | 0.2 |
| steady_jerk_mean | 36.2 | 38.4 | 41.3-43.5 | 41-44 | 48.0 |
| solve Hz | 39.7 | 43.3 | 40.5-43.7 | 41-42 | 45.5 |
| falls | 0 | 0 | 0 | 1 in 5 | 0 |

The fourth constrained node gives the fade the real policy torque on most ticks again: 0.8 hold back
to the high 60s / 70s, 0.5 hold 40-42 (the best so far, -27 % vs baseline), stops quiet, 0 falls.
The solve-rate guard tripped once (39.7 Hz, the other run 43.3) - run-to-run spread, not a cost of the
node. Shipped: `torqueConstraintNodes: 4` in `config/g1/ros2_controllers.yaml` (comment explains the
(N-1)*dt effective window), `holdOnTorqueWindowOverrun: true`. Both apply to the baseline too; a
`human4` baseline run with the shipped config is the pending comparison.

## Run 9 — fixed-controller baseline `humanC4` and `humanUB4` (speeds + omni)

`humanC4` = shipped config (nodes 4 + hold) + `gait.continuous.enabled=true`, HEIGHT 0.795 — the
`human` baseline rerun on the fixed controller. (A first attempt used `default`, i.e. the shipped
four-rung ladder: not comparable, but it measured the ladder at 0.8 m/s for the first time — hold
jerk **270.8** / acc_hf 5.66 / tau-rate 1062 against 88.7 for the continuous law, and 7 cm trot steps
at jerk 69 at 0.2 m/s against 47. The law is a large win on its own and should ship on.)

**Controller fix on the baseline: neutral.** humanC4 vs human: steady 43.5 / 0.799 / 214 vs
43.0 / 0.802 / 216; 0.8 hold 88.6 vs 88.7; the only guard fail is the 0.8 ramp 83.2 vs 73.6
(ramp spread across repeats is +-15, see runs 7/8). The baseline never fell before the fix and is
not smoother after it: the fallback mechanism cost the *template* runs their 0.8 m/s hold, it is not
what makes the baseline rough at 0.8. humanC4 solved at 64.1 Hz against 45.5 for `human`, with no
config change that could explain it (shipped yaml diff since `9754d1b`: new keys, all off) — the
machine load differs between sessions, so solve rates are only comparable within a session.

**Template on the fixed controller: ~20 % smoother everywhere but the turn, 0 falls.**

| | humanC4 (baseline) | humanUB4 | delta |
|---|---|---|---|
| steady jerk / acc_hf / tau-rate | 43.5 / 0.80 / 214 | **34.9 / 0.58 / 189** | -20 % / -27 % / -12 % |
| transition max jerk / acc_hf / tau-rate | 83.2 / 1.22 / 399 | **67.2 / 1.05 / 362** | -19 % |
| speeds hold 0.2 / 0.5 / 0.8 | 49.3 / 55.5 / 88.6 | **35.4 / 44.2 / 69.7** | -28 / -20 / -21 % |
| speeds 0.5 / 0.2 on the way down | 54.4 / 47.2 | 42.0 / 37.8 | -23 / -20 % |
| omni 0.3 / +turn 0.4 / 0.3 | 49.7 / 49.9 / 48.9 | 32.8 / **53.7** / 39.9 | -34 / +8 / -18 % |
| omni 0.6 / diag / back | 67.7 / 45.2 / 48.0 | 51.5 / 39.4 / 38.7 | -24 / -13 / -19 % |
| stops (speeds; omni x2) | 3.6; 0.2 / 0.3 | 0.15; 1.1 / 2.3 | quiet both |
| v_err max | 0.172 | 0.193 | |
| arm swing pp (0.5 / 0.8) | 1.1 / 1.5 deg | 5.9 / 8.5 deg | template |
| solve Hz | 64.1 | 40.6 | **-37 %, guard fail** |

The turn (+8 %, ~4 units) is the one segment the template does not improve; the arm / waist
template is phase-locked to the stride and the yaw-rate command is not in it. The solve-rate drop is
the open cost: the recipe carries the wrench-state layout (+12 states, +12 inputs) and the
per-node template; earlier sessions measured both at a few percent (44.1 / 42.0 vs 45.5) when
everything ran near 45 Hz, which now looks like a load-bound number. To be split before shipping:
a `humanC4W` run (humanC4 + wrench state 1e-4, no template) in the same session as a `humanUB4`
repeat, and `[MPC_TIMING] average advance` from both sim.logs.

Decision: nodes 4 + hold stay shipped (robustness, neutral on the baseline). Continuous law:
ship on. Template: suite says ship for 0.2-0.8 m/s + omni; owner's video verdict and the solve-rate
split are the two gates left.

## Run 10 — splitting the recipe: `humanC4W` (layout only) and `humanUB4r1` (speeds, vs humanC4)

Speeds-only numbers (humanC4 speeds: steady 49.8 / 0.88 / 246, transition max 83.2):

| | humanC4 | humanC4W (wrench state 1e-4 + vel, no template) | humanUB4r1 | humanUB4 |
|---|---|---|---|---|
| steady jerk / acc_hf / tau-rate | 49.8 / 0.88 / 246 | 39.5 / 0.68 / 209 (-21 %) | 37.4 / 0.62 / 203 (-25 %) | 36.4 / 0.63 / 207 |
| transition max | 83.2 | 72.2 | 67.0 | 67.2 |
| hold 0.2 / 0.5 / 0.8 | 49.3 / 55.5 / 88.6 | - | 34.8 / 42.8 / 66.3 | 35.4 / 44.2 / 69.7 |
| stop ramp / hold | 32.3 / 3.6 | - | **62.8** / 2.3 | 25.5 / 0.15 |
| solve Hz | 64.1 | (vs file pending) | 42.8 | 40.6 |

Reading: most of the pelvis smoothing is step A's wrench-rate layout (-21 %, matching the sweep's
humanW_1e-4); the template adds ~5 % on top and the human look (arm swing 6-8 deg, waist). The
template's holds are consistent across the two repeats (0.8: 66-70). The one blemish repeats too:
UB4r1's final stop ramp is 62.8 (UB3 r1 33.8, UB4 25.5) - the 1-in-3 noisy stop is still open.
The solve-rate split needs humanC4W's rate (compare file / `[MPC_TIMING]`), not in the suite line.

**Solve-rate split (same session, speeds):**

| | average advance | solve Hz | torque-window overruns / run |
|---|---|---|---|
| humanC4 (law only) | 15.6 ms | 64.1 | 0 |
| humanC4W (+ wrench state 1e-4) | 24.3 ms | 41.1 | 738 |
| humanUB4 (+ template) | 24.1 ms | 40.6 | 1156 |
| humanUB4r1 | 23.3 ms | 42.8 | 1279 |

The whole cost is the wrench-state layout (+56 % per solve: x 58 -> 70, u 64 -> 76; the Riccati
pass scales ~nx^3, (70/58)^3 = 1.76); **the template is free**. The MPC target is 100 Hz and no
configuration has met it (31-64 Hz across sessions). With `torqueConstraintNodes 4` the overruns
at 41 Hz are ~2 % of ticks (nodes 3 at the same rate: 5574, 12 %) and the hold keeps them harmless:
0 falls in 7 layout runs on the fixed controller. If 41 Hz is not acceptable the one cheap lever is
a force-only wrench state (6 instead of 12 rate states, ~(64/58)^3 -> ~20 ms, ~48 Hz), at the price
of unpriced moment rates - untested.

## Shipped (2026-11-27) — owner's video verdict: `humanUB4_speeds_side` is the best run so far

`config/g1/ros2_controllers.yaml` is now parameter-for-parameter the `humanUB4` recipe
(`make_wrench_configs.sh … 0.0001 1.0 human,arms,waist,vel`, verified by a key-by-key diff against
the generated yaml; the only difference is the suite's `diagnostics.trackingRate`):
`gait.continuous.enabled` on, `model.wrenchState` on with `contactWrenchRates` 1e-4, base velocity
weights 100 / 50, waist 20 / 1.0, arm positions x10 (q only) with arm velocities 0.6 / 0.2 and
arm accelerations 0.05, `humanGait.enabled` on with `legs: false` and `elbowMean -0.4`,
`defaultBaseHeight 0.785`. The suite's default HEIGHT follows (0.785). `run_suite.sh <tag> default`
is therefore the humanUB4 configuration from here on; a straight-leg / ladder baseline needs the
old values set explicitly.

Next: push it (owner: "can it be pushed to the maximum performance"). Config-only candidates on
the new default, in order of expected payoff: the leg curves (`HUMAN_LEGS=true`, run 3-5 falls were
the torque-window fallback, now held), wrench-rate 2e-4 / 5e-4 (1e-3 made the arms shake before
the arm template existed), template amplitude (`HUMAN_MAXSCALE`), then the C++ items: force-only
wrench state for the solve rate, yaw rate into the waist template for the turn, the 1-in-3 noisy stop.
