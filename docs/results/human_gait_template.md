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

## Run 11 — pushing the shipped config: leg curves and the wrench-rate weight (speeds, vs humanUB4)

| | humanUB4 | humanL5 (legs on) | humanW2 (2e-4) | humanW5 (5e-4) |
|---|---|---|---|---|
| steady jerk / acc_hf / tau-rate | 38.2 / 0.64 / 210 | 155 / 2.77 / 287, **fell** (0.8 ramp) | 37.8 / 0.61 / 205 | 38.9 / 0.59 / 193 |
| transition max jerk / tau-rate | 67.2 / 362 | 290 / 713 | 65.5 / 327 | 64.8 / 309 |
| hold 0.2 / 0.5 / 0.8 | 35.4 / 44.2 / 69.7 | 55.7 / 67.4 / - | 36.5 / 43.2 / 70.0 | 37.5 / 45.3 / **60.4** |
| final stop | 0.15 | - | 0.10 | 5.9 |
| solve Hz | 41.4 | 44.0 | 43.5 | 37.8 |

- **Leg curves: dead.** On the fixed controller the template's hip / knee / ankle reference still
  falls at the 0.8 ramp and is 55 % rougher at 0.2 and 0.5 before it (arm acc x3: the arms work
  against the leg reference). The falls of runs 3-5 had two causes, the fallback and the reference
  itself; the fix removed one. `legs: false` is the shipped value with that note.
- **Wrench-rate weight: on its plateau.** 2e-4 is a wash. 5e-4 buys the 0.8 hold (-13 %), acc_hf
  (-7 %) and tau-rate (-8 / -15 %) and pays at 0.2 (+6 %), the stop (5.9) and the solve rate
  (-9 %). Pelvis jerk is flat from 1e-4 to 5e-4: the force-rate pricing has given what it can; 1e-4
  stays shipped (5e-4 is the choice if high-speed smoothness is worth 4 Hz).

What is left is not a weight. The remaining jerk sits at the events of the stride; the normal
0.8 m/s stride lands 49 +- 22 ms before the planned contact switch (run 6), i.e. for ~9 % of each
step the robot stands on a foot the plan still treats as airborne. `tests/tuning/jerk_attribution.py`
(new) measures how much of a hold's jerk energy lies within +-40 ms of the measured touchdowns, the
planned switches and the lift-offs, and the energy profile over the stride phase - the input for
deciding between contact-triggered schedule adaptation (switch when the foot lands) and a
touchdown-aware swing end.

## Run 12 (analysis) — where the jerk is: the lift-off dump

`jerk_attribution.py` on humanUB4 / humanC4 / humanW5 speeds (holds 0.2-0.8, +-40 ms windows):

| hold 0.2 / 0.5 | planned lift-off: energy share, concentration | measured touchdown | phase bins 3-4 (end of DS) |
|---|---|---|---|
| humanC4 | 45-55 %, **4.0-4.5x** | 5-10 %, 0.5-0.9x | 50-60 % |
| humanUB4 | 37-49 %, **3.5-3.9x** | 14-23 %, 1.3-1.9x | 40-55 % |
| humanW5 | 19-26 %, 2.0x | 28-34 %, 2.5x | 25-30 % |

The early touchdown is a constant 50 +- 2 ms at 0.2 / 0.5 (C4 58-62), not the jerk source. The
jerk sits at the **lift-off**. `switch_profile.py` (event-aligned means, 0.2 m/s hold, planned fz of
the lifting foot in the last 80 ms and the jerk peak):

| | fz at -80 / -40 / -10 ms | step at the switch | jerk peak | az swing |
|---|---|---|---|---|
| humanC4 | 194 / 223 / **260** N (rising) | 260 N | 168 | +2.6 -> -2.3 m/s^2 |
| humanUB4 | 189 / 174 / 113 | 113 N | 104 | +1.0 -> -1.6 |
| humanW5 | 162 / 125 / 75 | 75 N | 80 | +0.5 -> -1.2 |

The plan drops the lifting foot's force to zero in one node; the other foot ramps (priced), so the
planned total force dips and the pelvis is *planned* to accelerate downward 1-2 m/s^2 right after
lift-off. The wrench-rate layout only prices the dump indirectly (in swing the state copy decays
with no physical consequence), which is why 5x the weight halves the step instead of removing it.
At the touchdown switch the landing foot is loaded 129 N in one node the same way, 50 ms after it
physically landed.

The lever already exists: `costs.contactLoadShareWeight` (`WbIdContactLoadShareCost`, D2/D3,
2026-11-24) pulls the vertical split in double support onto a smoothstep - landing foot 0 -> 1,
lifting foot 1 -> 0 - so both switches are force-continuous; total-invariant, so it does not fight
the dynamics. Its sweep then (1e-6..1e-3, old layout, trot, `cmd_tau_jump_at_switch` as primary)
shipped nothing; with the rate-priced layout and pelvis jerk at lift-off as the target it is the
first candidate, at weights 10-1000x the old sweep.

## Run 13 — whole-DS load share (`contactLoadShareWeight` 0.01 / 0.1 / 1.0, speeds, vs humanUB4)

All three **fell on the 0.8 m/s hold**; at 0.2 / 0.5 the hold jerk is unchanged (36.3 / 46.3 vs
35.4 / 44.2) although the lift-off energy is gone:

| 0.2 hold | lift-off share, conc | touchdown share, conc | bin 0 | lift-off fz at -10 ms / step |
|---|---|---|---|---|
| humanUB4 | 37-49 %, 3.5-3.9x | 14-23 %, 1.3-1.9x | 15-18 % | 113 N / 113 N |
| LS 0.1 | **4 %, 0.4x** | 33 %, 2.9x | **47 %** | 5.7 N / ~0 (ramp 224 -> 0 over 150 ms) |

The jerk moved: lift-off energy 540 -> 53 (units of jerk^2 share), touchdown 250 -> 435-620. The
profile shows why. The foot is physically down at -45 ms while the plan still drives it to -2 mm;
the body is pushed up (az +1.6 at -10 ms, jerk 84), then the plan starts loading the landed foot
from **zero** at the planned switch (5 N at +10 ms, 35 N at +50, 105 N at +90) - before, it put
129 N on it at once, which was closer to the physics. At 0.8 m/s (DS 0.12 s) a landing foot that
reaches half load only 60 ms after the switch is a fall; the whole-DS shape also takes the split,
the optimizer's balance lever in double support, away at every weight tried.

Built: end windows for the cost (`costs.contactLoadShareLiftOffWindow`, `...TouchDownWindow`, s):
the weight ramps up as (1 - tau/window)^2 towards the switch and is zero elsewhere in the DS, so
the lifting foot's force reaches zero continuously (the dump) while the mid-DS split and the
touchdown loading stay as they are. Same CppAD function (the scale goes in through the parameter),
no codegen. Run 14 = lift-off window only: (0.1, 0.06), (0.1, 0.04), (0.01, 0.06).

Touchdown is the next item once the dump is gone: the plan reaches the ground 50 ms after the foot
(20-40 ms of it by construction - touchDownVelocity -0.05 and offset -1 mm cross z = 0 before the
switch - plus the tracking lead). Candidates: the 2.C-timing set A winners (offset 0 / +1 mm,
acc_hf -16 / -25 %, never shipped because of a noisy stop metric), then a contact-triggered switch.

## Run 14 — lift-off end window (speeds, vs humanUB4) → shipped

| | humanUB4 | **LO 0.1 / 60 ms** | LO 0.1 / 40 ms | LO 0.01 / 60 ms |
|---|---|---|---|---|
| steady jerk / acc_hf / tau-rate | 38.2 / 0.64 / 210 | **33.1 / 0.59 / 213** (-13 / -9 %) | 35.8 / 0.61 / 217 | 37.3 / 0.63 / 220 |
| hold 0.2 / 0.5 / 0.8 / 0.5 / 0.2 | 35.4 / 44.2 / 69.7 / 42.0 / 37.8 | **27.8 / 39.6 / 62.8 / 39.6 / 28.5** | 31.6 / 35.3 / 74.6 / 43.1 / 30.3 | 32.1 / 36.4 / 83.7 / 41.3 / 29.8 |
| transition max | 67.2 | 73.4 (0.8 ramp) | 60.8 | 79.4 (fail) |
| final stop | 0.15 | 0.10 | 0.22 | 0.47 |
| falls / solve Hz | 0 / 41.4 | **0** / 45.8 | 0 / 43.9 | 0 / 40.3 |
| lift-off conc (0.2 / 0.5) | 3.7-3.9 / 3.1-3.5 | 1.1-1.5 / 1.4-1.6 | 1.7-2.3 / 1.5-1.8 | 1.1-1.2 / 1.5-2.0 |

Profile at 0.2 m/s: the lifting foot's planned force 164 N at -90 ms -> 6.5 N at -10 ms -> 0
(was 113 -> 0 in one node); jerk at the switch 26-41 (was 104). At 0.8 m/s a 16 N residual and the
whole-body motion remain (jerk 90 around the switch, az -2.5 at -10 ms). Touchdown unchanged
(139 N step at the planned switch, 50 ms after the foot) and now the dominant event everywhere
(sw_td concentration 3.0-3.6).

**Shipped: `contactLoadShareWeight 0.1`, `contactLoadShareLiftOffWindow 0.06`.** -21 / -10 / -10 %
at the 0.2 / 0.5 / 0.8 holds, -25 % on the way down, 0 falls, no solve-rate cost (the cost was
already compiled). Against the fixed-controller baseline humanC4 (49.3 / 55.5 / 88.6) the holds
are now **-44 / -29 / -29 %**.

Next: the touchdown. The plan reaches the ground 50 +- 2 ms after the foot at every step at 0.2 /
0.5 (20-40 ms of it by construction: `touchDownVelocity -0.05` with `touchDownHeightOffset -1 mm`
crosses z = 0 before the switch, then presses to -2 mm). Config candidates first (2.C-timing set A:
offset 0 / +1 mm gave acc_hf -16 / -25 % in steady walk; `touchDownVelocity` 0 / -0.02), then a
contact-triggered switch.

## Run 15 — shipped repeat (speeds + omni) and the touchdown reference knobs

**Repeat of the shipped config (`humanLO`, speeds + omni, vs humanUB4).** Steady jerk 31.6 vs 34.9
(-9 %), 0 falls. Speeds holds 29.7 / 33.8 / 72.0 / 37.4 / 36.1 (UB4 35.4 / 44.2 / 69.7 / 42.0 / 37.8);
omni holds 0.3 27.9 vs 32.8, turn 0.3+0.4 rad/s 36.1 vs 53.7 (-33 %), 0.3 35.5 vs 39.9, 0.6 66.3 vs
51.5, 0.2/0.1 35.1 vs 39.4, -0.2 32.9 vs 38.7. Across the two LO runs the 0.2 / 0.5 holds repeat to
+-2 (27.8-29.7 / 33.8-39.6) while the 0.8 hold spans 62.8-72.0 and the 0.6 hold 51-66: **the
lift-off window is a repeatable -16..-24 % at 0.2-0.5 m/s and a wash within noise at >= 0.6**,
where the stride-level balance motion dominates. Decision stands.

**Touchdown reference knobs (speeds, vs the humanLO repeat), all 0 falls:**

| | humanLO | o0 (offset 0) | op1 (offset +1 mm) | v0 (vel 0, offset 0) |
|---|---|---|---|---|
| steady jerk / acc_hf | 34.8 / 0.59 | 31.9 / 0.57 | 33.3 / 0.59 | 35.8 / 0.68 |
| hold 0.2 / 0.5 / 0.8 / 0.5 / 0.2 | 29.7 / 33.8 / 72.0 / 37.4 / 36.1 | 26.1 / 37.9 / 62.5 / 34.7 / 28.9 | 28.7 / 37.0 / 60.6 / 43.1 / 29.4 | 32.3 / 35.9 / 78.5 / 39.7 / 28.5 |
| transition max / 0.8 ramp | 67.4 | 65.9 | **93.1** (fail) | 78.1 (fail) |
| touchdown lead ms (0.2 / 0.5 / 0.8) | 50 / 51 / 39+-24 | 46 / 48 / 44 | 40 / 43 / 45 | 56 / 58 / -2+-29 |
| sw_td concentration (0.2) | 2.8-3.4 | 3.1-3.3 | 2.6-3.1 | 3.5-5.3 |

The reference knobs barely move the lead (50 -> 46 -> 40 ms; zero approach velocity makes it
*longer*) and none moves the touchdown concentration: the earliness is not in the spline. The
profiles say where it is: with the spline ending at **+1 mm** the plan's foot (`plan_foot`, FK of
the optimized state) is at +0.7 mm 30 ms before the switch, **-1.0 mm at -10 ms**, +0.4 mm after
it; with the spline ending at 0 at zero velocity it is -0.9 / -1.9 / -2.2 / -1.6 mm at -40..-10 ms.
The plan runs **~2 mm below its own spline in the last 40 ms of every swing** and the real foot
(0.3-0.5 mm, on the ground) lands when the plan crosses it, 25-35 ms before the switch. This is
the sag of `docs/results/touchdown_timing_sweep.md` (acceleration-level swing equality, poles
-4.4 / -45.6 rad/s at kp 200 / kd 50, zero-order-held node accelerations on a decelerating spline)
measured again on the human gait, where it was last seen "gone" on the trot. 2 mm at the 37 mm/s
approach is the 50 ms.

Next: the swing gain, config only (`constraints.footMotionEquality.swingPositionErrorGain_z`
is a CppAD parameter): 400 (poles -10 / -40) and 600 (-20 / -30) on the shipped config, plus
600 with offset 0. Watch `lead_ms` and `plan_z` at -30..-10 ms; a result is only real if the plan
stays on its spline *and* the 0.2 / 0.5 holds move (the 0.8 hold is noise-bound). Stops and the
0.8 ramp are the guards (sweep 2 saw a 1.33x stops pitch regression at a shared 400).

## Run 16 — swing height gain 400 / 600 (speeds, vs the humanLO repeat): not the lever

| | humanLO | SK 400 | SK 600 | SK 600 + offset 0 |
|---|---|---|---|---|
| steady jerk / acc_hf | 34.8 / 0.59 | 27.9 / 0.50 | 34.2 / 0.50 | 29.8 / 0.58 |
| hold 0.2 / 0.5 / 0.8 / 0.5 / 0.2 | 29.7 / 33.8 / 72.0 / 37.4 / 36.1 | 30.7 / 38.9 / 62.4 / 34.2 / 27.6 | 29.6 / 38.2 / **154.4** / 38.4 / 26.9 | 27.7 / 35.3 / 62.5 / 34.0 / 28.7 |
| solve Hz | 45.1 | 43.4 | 40.3 (fail) | **33.6** (fail) |
| lead ms (0.2 / 0.5) | 50 / 51 | 46 / 48 | 44 / 44 | 37 / 37 |
| sw_td concentration (0.2 / 0.5) | 2.8-3.4 / 2.2-2.4 | 3.1-3.4 / 2.6-2.8 | 3.4-3.5 / 2.5-2.8 | 2.8-3.0 / 2.6-2.8 |
| plan_z at -30 / -20 / -10 / 0 ms (0.2) | +0.7 / -0.2 / -1.0 / -0.8 (op1) | | -0.1 / -1.0 / -1.5 / -0.8 | +1.1 / +0.3 / -0.4 / -0.8 |

kp 600 halves the sag (the plan ends ~0.8 mm under its spline instead of ~2) and the lead follows
by only 6-13 ms, because the spline itself spends its last 27 ms inside the last millimetre
(37 mm/s approach) and the sole sits on the ground at 0.25 mm: the real foot is down from
-30..-20 ms with the plan exactly on its reference. The touchdown concentration does not move at
all (2.5-3.5 everywhere), the 0.2 / 0.5 holds are a wash (SK_400's "winner" verdict is the
noise-bound 0.8 hold), the stiffer equality costs 4-25 % solve rate, and SK_600 stumbled at 0.8
(hold 154, no fall). **Not shipped; `swingPositionErrorGain_z` stays 200.**

What the profiles say the touchdown jerk is: the real foot is already resting on the ground when
the plan flips its contact flag and loads it **0 -> 123 N in one node** (then 130 / 142 / 167 /
180 N at +10..+40 ms) while the WBC's contact alpha fades 0.2 -> 1.0 over +0..+30 ms; the pelvis
jerk peaks 54-62 at -30 and +20..+30 ms, exactly as it peaked at the lift-off switch before the
lift-off window. The lead is now 20-40 ms of an unloaded foot on the ground, not the cause of the
step. Two levers, in order: (1) `contactLoadShareTouchDownWindow` 0.03 / 0.04 on top of the
shipped lift-off window (config only; the cost ramps the landing foot's force from 0 over the
window; the whole-DS smoothstep fell at 0.8 so the window must leave the middle of the 120 ms
double support free - 0.06 + 0.04 is the most that fits); (2) if the window helps at 0.2 / 0.5
but costs 0.8, the contact-triggered switch, which loads the foot when it lands instead of
20-40 ms later.

## Run 17 — touchdown end window 0.03 / 0.04 (speeds, vs the humanLO repeat): rejected

| | humanLO | TW 0.03 | TW 0.04 |
|---|---|---|---|
| steady jerk / acc_hf | 34.8 / 0.59 | 51.6 / 0.77 | 83.1 / 1.16 |
| hold 0.2 / 0.5 / 0.8 | 29.7 / 33.8 / 72.0 | 46.5 / 42.7 / 126.4 | 42.4 / 50.9 / 156.1 |
| falls | 0 | 0 | **1** (0.8 -> 0.5 ramp, jerk 304) |
| sw_td concentration (0.2) | 2.8-3.4 | 3.4-3.5 | 4.3 |

Worse at every speed from the first hold, and worse the longer the window. The profile at 0.2 shows
the mechanism: the landing foot's planned force stays 1.7 / 4.8 / 15 N for the first 20 ms after
the switch and only reaches 66 / 104 / 129 N at +30..+50 ms, while the pelvis jerk runs 85-111
from -20 ms on (the plan keeps the other foot at 288 N and the body falls onto a leg it is not
allowed to load); at 0.8 the plan keeps the "landing" foot 1-8 mm above the ground (measured 5 mm)
through the whole switch, i.e. it never lands. A touchdown ramp is not the mirror of the lift-off
ramp: at lift-off the foot is still on the ground and the body has already moved off it, so
unloading late costs nothing; at touchdown the body arrives with the foot and needs the force the
moment it lands. **Touchdown shaping of any kind (whole-DS run 13, end window run 17) stays off.**

The remaining touchdown lever is timing, not shaping: load the foot *when it lands*. That is the
contact-synchronised switch (planner side: when the swing foot's measured sole height reaches the
ground inside a window before its planned touchdown, move that event to now; when the planned
touchdown passes with the foot still in the air, hold the event until it lands, capped) - the same
mechanism also removes the late-touchdown half of the run-6 fall anatomy at 0.8. Config-only
levers on the touchdown are exhausted (runs 15-17).

## Run 18 — contact-synchronised touchdown (`gait.contactSync`, speeds, vs the humanLO repeat)

| | humanLO | CS (early 60 ms, 2 mm) | CS_t1 (1 mm) | CS_late (+ late 40 ms) |
|---|---|---|---|---|
| steady jerk / acc_hf | 34.8 / 0.59 | 31.9 / 0.60 | 31.1 / 0.59 | 33.5 / 0.61 |
| hold 0.2 / 0.5 / 0.8 / 0.5 / 0.2 | 29.7 / 33.8 / 72.0 / 37.4 / 36.1 | 28.6 / 34.8 / 67.1 / 33.4 / 25.7 | 24.9 / 33.1 / 64.7 / 32.5 / 28.1 | 26.1 / 33.3 / 80.3 / 33.7 / 27.0 |
| transition max | 67.4 | 57.9 | 61.2 | 79.4 (fail) |
| falls / solve Hz | 0 / 45.1 | 0 / 39.9 | 0 / 38.8 | 0 / 39.7 |
| lead ms (0.2 / 0.5 / 0.8) | 50+-2 / 51+-1 / 39+-24 | 30+-4 / 38+-5 / 32+-18 | 37+-1 / 38+-7 / 37+-25 | 31+-5 / 32+-7 / 19+-25 |
| td concentration (0.2 / 0.5) | 1.9-2.1 / 2.0-2.2 | 1.3-1.6 / 1.4-1.7 | 1.3-1.8 / 1.5 | 1.5-1.8 / 1.3-1.7 |
| sw_td concentration (0.2 / 0.5) | 2.8-3.4 / 2.2-2.4 | 2.7 / 2.1-2.8 | 2.7-3.1 / 2.4-2.5 | 3.0-3.3 / 1.9-2.5 |

The rule fires as designed and its limit is the one predicted: the lead drops 50 -> 30-38 ms and
its spread grows from +-1 to +-5 ms, i.e. the landing is caught at the next solve and the policy
carrying the moved event arrives one solve period later. The profile at 0.2 confirms it: the sole
is down at -20 ms (was -30), the plan loads it 0 -> 102 N at the switch, `alpha` goes 0.96 -> 0.00
in one step (the fade is one-sided now: an event the controller did not see coming) and back to
1.0 by +30 ms. The jerk before the switch is lower (25 vs 31-35 at 0.2) but the peak **after** it
is unchanged in shape and size (59 / 72 / 64 at +20..+40 ms, LO 46-62): the foot has been down for
40 ms by then, so this is the weight transfer itself - the new foot 102 -> 169 N and the old one
256 -> 185 N over 40 ms, pelvis az -1.2 -> +0.75 - not the landing. The touchdown concentration
drops by a quarter (1.9-2.2 -> 1.3-1.8); the holds follow at 0.2 (-4 / -29 %, -16 / -22 % with the
1 mm threshold) and are a wash at 0.5 / 0.8. Steady jerk -8 / -11 %. 0 falls.

Late half: the 0.8 hold is worse (80 vs 72) and the 0.8 ramp fails the guard: holding the event
while the foot is still up at 0.8 m/s costs balance, exactly as loading it early did in run 13.
**`lateWindow` stays 0.**

Not shipped from this run: the one guard it fails is the solve rate (39-40 vs 45 Hz), which is
only comparable within a session, and the 0.2 gain is of the order of the repeat spread. An
in-session pair decides it: `humanLO` again and `humanCS` (early 60 ms, threshold 2 mm; the 1 mm
threshold is within noise of it and trips 7 ms later), speeds + omni + stops. Ship if the solve
rate is within the guard against the same-session baseline, 0 falls, and the 0.2 / 0.5 holds are
not worse.

What is left of the touchdown after this is the double-support weight transfer (the post-switch
peak above), the mirror image of the lift-off dump the end window removed - and run 13 / 17 showed
that shaping the landing force costs balance at 0.8. That is the floor of this lever family on the
touchdown side; the next step change is elsewhere (lead 0 at 0.8 remains part of the run-6 anatomy
and the late half does not fix it).

**In-session pair (`humanLOr2` vs `humanCSr1`, speeds + omni + stops, same session):** steady
jerk 30.2 -> 24.0 (-20 %), acc_hf 0.505 -> 0.462, tau-rate 180 -> 159, transition max 62.8 -> 62.9,
0 falls in all six suites, solve rate 43.4 -> 40.9 (-6 %, inside the guard). Speeds holds
27.6 / 48.9 / 65.3 / 41.1 / 33.6 -> 26.8 / 33.8 / 66.4 / 32.6 / 26.8; every omni hold better
(0.3: 45.8 -> 25.9, turn 32.2 -> 27.1, 0.6: 63.0 -> 49.9, 0.2+0.1 lateral 42.2 -> 28.4, -0.2:
33.0 -> 28.2); stops holds 46.2 / 36.2 / 36.2 -> 33.6 / 34.6 / 35.2 and the baseline's one noisy
final stop (17.8) does not recur (1.55). **Shipped: `gait.contactSync.enabled: true`** (early
window 60 ms, threshold 2 mm, late half off). Shipped G1 = humanUB4 + lift-off end window +
contact-synchronised touchdown.

## Run 19 — backward (shipped config, `backward` suite = the speeds ladder in reverse)

| | forward (`humanCSr1__speeds`) | backward (`humanCSr1__backward`) |
|---|---|---|
| steady jerk / acc_hf / tau-rate | 31.4 / 0.61 / 204 | 30.0 / 0.48 / 180 |
| hold 0.2 / 0.5 / 0.8 / 0.5 / 0.2 (|vx|) | 26.8 / 33.8 / 66.4 / 32.6 / 26.8 | 29.6 / 32.9 / 54.7 / 32.8 / 28.0 |
| transition max | 62.9 | 61.5 |
| falls / final stop | 0 / 1.9 | 0 / 2.1 |
| arm swing pp (deg) | 9.6 / 12.4 / 18.4 | 1.9 / 2.9 / 3.4 |
| touchdown lead ms | 28-35 | 33-36 |
| sw_td concentration (0.2 / 0.5 / 0.8) | 2.7-2.9 / 2.0-2.2 / 2.0 | 4.0-4.6 / 3.8 / 2.7 |

Backward is as smooth as forward by the pelvis numbers (0.8 even better, 54.7 vs 66.4, with less
roll: 1.85 vs 3.5 deg pp) and survives the whole ladder. Two differences, both visible in the
video: **the arms do not swing** (1.9-3.4 deg pp vs 10-18: the human-gait arm and trunk templates
were clamped to forward speed >= 0 in the reference manager), and the touchdown is a larger share
of the stride's jerk (concentration 4.0-4.6 vs 2.7-2.9 at 0.2) at the same lead - the plain gait
without the templates puts more of its jerk at the switch.

**Change (this commit):** the templates carry the direction of travel. Backward walking is the
forward cycle in reverse, so the arm swing and the waist counter-rotation keep their coupling to
the legs with the sign flipped (at a foot's contact the own arm is back going forward, forward
going backward); the obliquity and the height oscillation depend only on which leg swings and use
the magnitude; the leg curves (off in the shipped config) stay forward-only - they are angles, not
time-reversible. Lateral and turning commands are unchanged (stance posture). Needs a rebuild and
the backward run again (`humanBW`): expect arm pp back to ~10-18 deg at 0.2-0.8 and no change in
falls; the pelvis numbers decide whether the templates help backward as they did forward.

**Result (`humanBW`, `backward` + `speeds`, same session as the second `humanCSr1`):** the arms swing
backward with the forward amplitude - arm pp 10.4 / 13.2 / 18.2 deg at -0.2 / -0.5 / -0.8 (was
1.9 / 2.9 / 3.4; forward 9.8 / 12.9 / 18-21), the user confirmed the phase in the video. Pelvis numbers
backward are unchanged within the repeat spread (holds 27.5 / 33.2 / 57.4 / 31.8 / 27.4 vs
29.6 / 32.9 / 54.7 / 32.8 / 28.0) and the worst ramp is better (-0.8 ramp 49.8 vs 61.5, acc_hf
0.82 vs 1.17), falls 0, final stop 0.10. Forward is a no-op by construction (all three scales equal the
old ones for forwardSpeed >= 0) and by the data at 0.2 / 0.5 (27.2 / 35.1 / 33.4 / 27.5 vs
26.8 / 33.8 / 32.6 / 26.8, arm pp equal); the 0.8 hold came out at 112 vs 66 with a v_err spike to
0.17 - one recovery step inside the hold, the known 0.8 hold-to-hold spread, not the change (the
candidate gate flags it, read it against the per-segment table). Shipped as is: the templates are
part of the G1 config, backward now gets them.

## Run 20 — commanded height: does the gait hold up away from 0.785 m?

Question: the GUI slider commands the pelvis height; the whole tuning (runs 1-19) was done at
`HEIGHT=0.785`. What depends on it, and can the performance be the same at any height?

**What the height can be.** Straight leg = 0.793 m pelvis (MJCF, flat sole). The shipped
`defaultJointState` (hip -0.8, knee 1.6, ankle -0.8) is the symmetric posture of **0.600 m**, not of
0.785: the posture cost (knee weight 0.1) and the height cost (weight 50) pull against each other and the
walking height is their compromise (`knee_ms` 35-41 deg at mid stance = a pelvis around 0.76 m, not the
commanded 0.785). The old slider range 0.395-1.195 m was generic: above 0.793 the height is unreachable
and the knee is singular, below ~0.60 the robot is in a deep crouch no part of the stack was designed for.

**What is independent of it** (nothing to do): step timing / double support / swing clearance (the gait
law and the swing planner work above the ground, not below the pelvis), contactSync (sole height),
the arm / waist / pelvis templates, the capture point (uses the measured CoM height: omega = sqrt(g/z)).

**Changes (this commit):**
- `minBaseHeight 0.60`, `maxBaseHeight 0.79` (slider ends = the leg's range), height slew 0.5 -> 0.2 m/s
  (a slider move while walking is a ramp over a few stride fractions, not a 0.1 s kick in one single
  support).
- `score_suite.py` reports `z_m`, the pelvis height actually walked at (per segment, also in the
  `compare_suite.py` limbs table).
- `config/g1/compute_posture_table.py`: the symmetric flat-foot posture of every height in the range from
  the real MJCF kinematics (0.60: knee 1.600, 0.70: 1.095, 0.75: 0.739, 0.785: ~0.30, 0.79: 0.194 rad),
  printed as `postureHeights / postureJointStates` or written into a candidate config. With the table the
  joint reference is the posture of the commanded height (the mechanism the Upkie ships), so the only
  height-dependent term in the reference is the height itself.
- `run_height_ladder.sh <tag> <config> [heights]`: the `speeds` ladder at several heights, compared
  against the first.

**Plan.** (1) Ladder on the shipped config: 0.785 / 0.75 / 0.72 / 0.68 (`humanH`). Read `z_m` (where it
really walks per commanded height), falls, the 0.8 hold, `knee_ms`. (2) If the numbers hold to ~0.70 the
answer is "the same within the slider range" and the slider range stays as set; if they fall off, the
posture-table candidate (`humanPT`, same ladder) is the structural fix to test - note it also changes the
0.785 operating point (the posture reference becomes knee ~0.3 instead of 1.6, the robot will walk
higher / straighter), so its 0.785 entry must be read against the shipped run before anything else.
(3) Only after that: per-height scaling of the gait timing (Froude: step time ~ sqrt(h)) if a trend
remains.

**Result, shipped config (`humanH`, `speeds` at commanded 0.785 / 0.75 / 0.72 / 0.68):**

| commanded | walked `z_m` 0.2 / 0.5 / 0.8 / standing | steady jerk | 0.8 hold | trans max | falls | knee_ms 0.2 / 0.8 |
|---|---|---|---|---|---|---|
| 0.785 | 0.740 / 0.735 / 0.722 / 0.743 | 31.6 | 72.0 | 67.4 | 0 | 35.8 / 40.8 |
| 0.75  | 0.733 / 0.731 / 0.720 / 0.718 | 31.0 | 66.3 | 59.1 | 0 | 39.4 / 41.4 |
| 0.72  | 0.726 / 0.727 / 0.719 / 0.695 | 31.3 | 68.3 | 72.1 | 0 | 43.1 / 43.6 |
| 0.68  | 0.720 / 0.722 / 0.715 / 0.664 | 33.4 | 82.1 | 63.3 | 0 | 46.3 / 43.3 |

The performance is flat over the slider (steady 31.0-33.4 = the repeat spread; the 0.8 hold 66-82 inside
its known spread; 0 falls; step length, arm swing, hip range identical) - **because the walking height
barely follows the command**: a 10.5 cm command range moves the walked height 2 cm (0.740 -> 0.720 at
0.2 m/s, 0.722 -> 0.715 at 0.8). The slider sets the standing height (0.743 -> 0.664, tracked within
the posture compromise) and the knee bend; walking, the optimizer settles at 0.72-0.74 m whatever is
asked, and that is the operating point every run since Run 9 was tuned at. The only trend is at the low
end: at 0.68 the 0.8 hold is the worst of the four (82, arm acc 4.4) - a deeper crouch at top speed
costs a little.

**Result, posture-table candidate (`humanPT`, same ladder):** at 0.785 the robot walks at 0.753 m with
an 11.7 deg mid-stance knee and **falls in the 0.2 m/s hold** (steady jerk 197 before the fall); at 0.75
it falls on the ramp to 0.5 (walked 0.745, knee 32). At 0.72 and 0.68 it survives with the same numbers
as the shipped config (29.7 / 32.3 steady, walked 0.733 / 0.723, knee 39 / 44). The straight-leg regime
(walked > ~0.745, knee < ~30 deg) is where this controller falls - the same finding as the human leg
template (runs 3-5, 11): the gait needs the bent knee. **Rejected.** The crouch posture reference is not
a defect to remove: it is what keeps the knee in the band the gait works in.

**Conclusion.** The goal "the same performance at any height" is met by the shipped controller over the
slider range by construction of its cost compromise: walking, the height is the gait's own. The slider
range 0.60-0.79 stays (0.60-0.68 untested walking; standing it is fine). Do not raise the walked height
above ~0.745 (posture table, higher `q.basePose[2]`, straight-leg `defaultJointState`) without
re-tuning the whole stack. If a crouched walk at the low end matters (0.68 at 0.8 m/s), the lever is the
height weight, not the posture.

**`height` script, shipped config (`humanCSr1__height`):** walking at 0.5 m/s the command 0.785 -> 0.70
moves the walked height 0.734 -> 0.725 m (9 mm for an 85 mm command), transitions 29.8 / 32.4 = the
steady level, no visible change in the video; standing, 0.740 -> 0.661 -> 0.741, tracked. So the slider
is a standing-height control; walking, the gait keeps its own height. If the walk is to follow the slider,
the lever is `q.basePose[2]` (50) with the range capped at the natural walking height (`defaultBaseHeight`
0.735, `maxBaseHeight` 0.745) so a stiffer height cost can never push the walk into the straight-leg
regime: `humanHZ200` / `humanHZ400` (z weight 200 / 400, Q and Qf), `height` + `speeds` against
`humanCSr1` - the cost is vertical stiffness at touchdown (acc_hf, jerk), the ladder decides.

**`humanHZ200` / `humanHZ400` (z weight 200 / 400, defaultBaseHeight 0.735, maxBaseHeight 0.745; `height`
+ `speeds` vs `humanCSr1`, one session):**

| | walked at 0.5 m/s, cmd 0.745 -> 0.700 | standing 0.66 | speeds steady jerk / acc_hf | 0.8 hold | trans max acc_hf | falls |
|---|---|---|---|---|---|---|
| shipped (50) | 0.734 -> 0.725 (9 mm of 45) | 0.661 | 24.8 / 0.49 | 66.4 | 1.03 | 0 |
| z 200 | 0.734 -> 0.717 (17 mm) | 0.659 | 23.5 / 0.47 | 58.9 | 0.97 | 0 |
| z 400 | 0.737 -> 0.713 (24 mm) | 0.660 | 25.1 / 0.49 | 73.4 | 1.18 (guard fail) | 0 |

Eight times the height weight buys half of the commanded 4.5 cm. The walking height is set by the
dynamics (the stance-leg vault, the torque cost of a crouch), not by the reference weight: the slider
is a standing-height control with a small walking effect, and no reference weight makes it otherwise.
z 200 is free within one session (every speeds number equal or better, no falls) and gives ~2 cm of
visible dip; z 400 starts to cost at the 0.8 ramp down (acc_hf 1.18, tau rate 362). Decision pending
(ship 200 with the honest range 0.60-0.745 / default 0.735, or keep 50 and document the slider as the
standing height). The lever beyond this is the knee torque cost in R, at the price of knee heat.

**Backward check (`humanHZ200` `backward` + `height_back` vs `humanBW`):** backward the shipped config
already follows the slider three times better than forward - at -0.5 m/s the command 0.785 -> 0.70
moves the walked height 0.716 -> 0.688 (28 mm; forward 9 mm) - and z 200 adds nothing to it
(0.723 -> 0.694, 29 mm of a 45 mm clamped command) while costing a little: -0.2 / -0.5 holds 30.3 / 34.9
vs 27.5 / 33.2, acc_hf 0.56 / 0.59 vs 0.43 / 0.50, the -0.8 ramp acc_hf 0.91 vs 0.82 (guard fail);
arms unchanged (10-18 deg), 0 falls. **Decision: keep z weight 50 (option 2).** The slider is the
standing height; walking, it moves the pelvis ~1 cm forward and ~3 cm backward and that is the
controller's property, not a tuning target. Run 20 closed.

## Run 21 (planned) — a 10 cm crouch on command while walking

Target (user): the slider must move the walking pelvis ~10 cm, not 3. That is a crouch gait, not a
height offset: 0.74 -> 0.64 m means the knee at ~80-90 deg through stance instead of 35-40, roughly
twice the knee torque, and - the hard part - the **ankle dorsiflexion limit** (-0.873 rad = 50 deg).
With a flat foot and an upright trunk a 0.64 m pelvis needs a shank inclination of ~38 deg at mid
stance and 55-60 deg by toe-off: beyond the joint limit, so the late-stance posture has to come from
more knee / hip flexion (and the `jointPositionLimits` barrier, mu 1200 / delta 0.1 rad, is active the
whole time) or from a forward trunk pitch, which the pitch weight 100 forbids. That is why eight times
the height weight bought 2 cm (Run 20): the optimizer is not ignoring the slider, it is paying a barrier.

Step 1 (measurement, references only): `crouch` script (0.3 and 0.5 m/s, 0.72 -> 0.62 -> 0.72) on the
shipped config and on `humanCR` = posture table (legs referenced at the crouch) + z weight 400 +
defaultBaseHeight / maxBaseHeight 0.72 (the table's safe top: 0.72 walked in Run 20, 0.75 fell). Reads:
`z_m` in the 0.62 holds (how much of the 10 cm the walk delivers), `knee_ms`, falls, jerk; `speeds`
for the regression. Step 2, if the ankle limit is the wall: a trunk-pitch reference that follows the
commanded height (e.g. 0 at 0.74, 12 deg at 0.62) and the step length / cadence scaled with the height
(Froude). Step 3: touchdown re-tune at the crouch. Each step is a run cycle; the use case decides how far
to go (ducking under an obstacle at 0.3 m/s is a much smaller target than a crouch gait at 0.8).

## Run 22 (planned) — the same walk on uneven ground, one terrain at a time, then the combined scene

Goal: keep the flat-ground forward / backward performance (humanCSr1 forward holds 26.8 / 33.8 / 66.4 /
32.6 / 26.8, humanBW backward 27.5 / 33.2 / 57.4 / 31.8 / 27.4, 0 falls) on terrain, without changing the
flat-ground walk the user accepted. Plan agreed with the user: work the terrain families individually,
then combine what passes into one scene (`TERRAIN=course`) as the new simulation scene.

What the controller knows about the ground: nothing. The MPC references every foothold at z = 0
(`ocs2.reference.groundHeight.useEstimate false`; `true` uses the stance-foot FK height, limited to
`maxStepPerSolve` 0.01 m per solve), the swing template clears `swingHeight` 0.05 m above that plane, and
touchdown is detected by contactSync (`heightThreshold` 0.002 m, `earlyWindow` 0.06 s): a step that lands
early on higher ground is accepted up to 60 ms before the schedule; lower ground means the foot reaches
the planned height in the air and the schedule switches to stance with no contact under it until the
swing extension (`touchDownVelocity` -0.05 m/s) finds the ground. The estimator pins the stance foot to
its last height, so on a tile the estimated pelvis height is the true height minus the tile. Those are the
three mechanisms the terrain stresses; everything else (load share, blend, templates) is the flat walk.
Each terrain family stresses them differently, which is why they are run apart:

| rung | `TERRAIN` | stresses | tooling |
|---|---|---|---|
| 1 uneven | `suite_mild` (1-2 cm steps, tops to 5 cm), `suite_rough` (2-4 cm, to 10 cm) | random early / late touchdowns every step | 40 x 6 m strips, `config/terrains/uneven_suite_*_params.yaml` |
| 2 slope | `slope_5`, `slope_10` (6 m up, 2 m plateau, 6 m down) | a continuous z-reference error (every foothold is referenced at z = 0 while the ground rises step x tan): `useEstimate` is the lever, pitch posture and ankle range on the descent | `config/terrains/g1/slope_*.yaml` |
| 3 curb | `curb_4`, `curb_8` (3 m plateaus, six curbs per direction) | one isolated step up (toe clearance vs swingHeight 0.05) and down (late touchdown, lateWindow 0 -> falls into stance with no contact) | `config/terrains/g1/curb_*.yaml` |
| 4 stairs | `stairs_3x10` (3 x 10 cm x 0.30 m up, landing, down) | three curbs in a row with a 0.30 m tread the blind foothold may straddle | `config/terrains/g1/stairs_3x10.yaml`; the stair_climbing planner is the non-blind alternative |
| 5 combined | `course` | everything in sequence, the final scene | `config/terrains/g1/course.yaml` (draft) |

All courses are mirrored at -x (`speeds` and `backward` meet the same features); they are generated by
`description/mujoco/terrains/generate_course.py` and verified by `tests/test_g1_courses.py`.

Per rung: Phase A = measure the shipped config (`SCRIPTS="speeds backward"`, compared to humanCSr1 flat):
falls and where, steady jerk, holds, `z_m`, touchdown lead histogram. Phase B = one lever per run, each
re-run on flat `speeds` + `backward`: `groundHeight.useEstimate true`; `swingHeight` 0.05 -> 0.07 / 0.08;
contactSync `earlyWindow` 0.06 -> 0.10 and `lateWindow` 0 -> 0.04 (Run 15 rejected lateWindow 0.04 on
flat ground for jerk, so it is a terrain-only candidate unless the flat rerun is clean); `touchDownVelocity`
-0.05 -> -0.10. Whatever passes a rung's easy level goes to its hard level; the accepted set must leave the
flat `speeds` / `backward` numbers within the Run 19 / 20 spread. The combined course is composed from the
levels that pass (a rung that does not pass blind is left out of the scene or kept at its passing level).

### Run 22 A — rung 1 measured (shipped config, `suite_mild` / `suite_rough`, vs humanCSr1 flat)

(An earlier pair of humanTM / humanTR runs was flat: the user's checkout predated the `TERRAIN` passthrough.
`run_case.sh` now stamps `TERRAIN <module> <file>` into the console and `summary.txt`; `z_m` is the tell:
on the mild strip the pelvis sits 0.746-0.771 m absolute, flat is 0.724-0.741.)

| case | forward | backward |
|---|---|---|
| humanTM `suite_mild` | 0.8 reached; steady jerk 32 / 45 / 109 / 50 vs 27 / 34 / 66 / 33 flat; tau_rate 523 at 0.8; v_err 0.17-0.20 in the holds (0.09-0.12 flat: the robot is slower than commanded); **fell in the second 0.2 hold** (x ~ 14 m, after a 127-jerk ramp with arm_acc 7.4) | survived; steady jerk 42 / 98 / 71 / 61 / 60 vs 30 / 33 / 55 / 33 / 28; v_err 0.17-0.18 at 0.2 / 0.5 |
| humanTR `suite_rough` | **fell in the first 0.5 hold** (x ~ 3 m, as the field reaches full height; z_m 0.787 just before) | **fell in the ramp to -0.5** |

Reading. The estimator in the loop is the linear KF with `footHeightSensorNoise 0.01`: every stance foot is
pulled toward z = 0, so the estimated frame follows the terrain with a lag of a few hundred ms. In that frame
the z = 0 foothold reference is "the tile I am standing on"; each new tile is an error of one neighbour step
(1-2 cm mild, 2-4 cm rough) in the swing's end point, plus whatever the KF has not yet absorbed (the knee at
mid-stance opened from 35 to 25 deg in the 0.5 return hold on mild: the estimate was below the true height and
the body was raised to compensate). A tile that is lower by 2 cm is found by the `touchDownVelocity -0.05`
extension in 400 ms; the schedule has switched to stance long before (`lateWindow 0`) and the load is shared
onto a foot in the air. A tile that is higher meets the foot in the fast part of the descent, more than the
`earlyWindow 0.06` before schedule, and the swing tracking presses the foot into the ground until the switch
(tau_rate 523 at 0.8). Both are touchdown-timing failures, not clearance failures (5 cm swing over 1-2 cm
steps), which puts the touchdown levers first and `swingHeight` last.

Phase B on `suite_mild` (one lever per run, baseline humanTM): humanTV `touchDownVelocity -0.15`; humanTC
contactSync `earlyWindow 0.12`, `lateWindow 0.10`; humanTVC both; humanTE `groundHeight.useEstimate true`
(the stance-foot FK height, i.e. the part of the tile the KF has not absorbed yet, becomes the swing's ground
reference; `maxStepPerSolve 0.002`). Then the flat regression of whatever passes.

### Run 22 B — rung 1 single levers on `suite_mild` (baseline humanTM): none passes

| candidate | forward (`speeds`) | backward | verdict |
|---|---|---|---|
| humanTV `touchDownVelocity -0.15` | **fell in the first 0.5 hold** (x ~ 3-5 m; humanTM got to 14 m) | clearly better: steady 28 / 57 / 68 / 46 / 47 vs 42 / 98 / 71 / 61 / 60 | the only lever that helped anything |
| humanTC contactSync `earlyWindow 0.12`, `lateWindow 0.10` | 0.2 hold jerk 76 (32), 0.5 ramp 163, **fell in the 0.5 hold** | worse everywhere (tau_rate 350-450) | rejected: the windows are not the mechanism |
| humanTVC both | 0.2 hold 149, **fell in the ramp to 0.5** | worse than TM | rejected |
| humanTE `groundHeight.useEstimate true` | the only one to walk the whole ladder (0.8 hold 106 vs 109, last 0.2 hold 71 vs TM's fall); **fell in the final stop ramp** (jerk 426, tau_rate 1023) | mixed: 0.5 holds 57 / 82 vs 98 / 61, 0.8 worse (93 vs 71) | closest, still 2x flat jerk and a fall |

Reading. The falls are position-locked, not speed-locked: TV / TC / TVC / humanTR all fell in the 0.5 hold at
x ~ 3-5 m, TM at x ~ 14 m, TE at the stop near x ~ 15.5 m, while the same speeds are walked elsewhere on the
strip. That is what tile features do, not touchdown timing: the G1 sole is 0.17 m long (collision spheres at
-0.05 / +0.12 m from the ankle) on 0.5 m tiles, so roughly a third of all landings straddle a tile edge and rest
on the heel pair or the toe pair with a 1-3 cm gap under the other end. The WBC loads both contact points as
scheduled, the foot rocks onto the lower tile and the body pitches - no touchdown window or descent speed
changes that, which is why TC made it worse (the switch is held while a rocking foot "searches"). Next:
`tests/tuning/terrain_strides.py` (new) ray-casts the case's own scene under the four sole corners at every
touchdown and classifies landings (edge / step up / step down) against the pitch excursion that follows, on
the existing humanTM / TE / TV / TR cases - no new simulation - to confirm the edge mechanism before the
terrain or the controller is changed. If confirmed: the "uneven ground" rung becomes a continuous heightfield
(the blind walker's realistic rough ground; edge landings are the curb rung's problem, dealt with there), and
the blocks keep their place as a stepping-stone benchmark for a perceptive foothold planner.

### Run 22 C — `terrain_strides.py` on the rung-1 cases: steps, not edges

`tuning_results/terrain_strides.txt` (humanTM / TE / TV / TR `speeds`, TM / TV `backward`). Edge landings are
~half of all landings (47-55 %; 27-38 % for TV) and are **not** followed by larger pitch excursions than
full-sole landings (TM 1.7 vs 4.2 deg, TE 2.3 vs 2.1, TM backward 1.4 vs 2.5): the edge hypothesis is
dropped for walking. The falls line up with the **largest terrain step of each run**, which is a per-stride
quantity (stance and swing foot are 1-2 cells apart in x and often one in y, so it reaches 2-3x the adjacent
bound): TV fell on a +3.1 cm step-up (pitch 33 deg; its largest), TR on a -6.1 cm step-down (14 deg, then
-3.0 and over), TM on a +1.5 after the run's largest edge (2.8 cm) under the push-off foot, TE - the only run
to meet +3.3 / -2.9 cm steps and survive them (pitch 2.6 / 1.9) - fell **in the stop**, with both feet on
1.4-1.5 cm edges and no terrain event (user: "performing great and suddenly fell"). Routine +-1.3-1.7 cm steps
are walked by every config. TE and TR also drift laterally (y -1.85 m over 10 m; 0.5 m over 3 m) where the
flat runs track y = 0: a heading / estimator effect of the terrain to watch.

So the blind walk's tolerance is ~+-1.7 cm per stride on the shipped config and ~3 cm with
`useEstimate true`; 6 cm is out of reach blind. Next: fall anatomy of the TE stop (double support on two
edges is the suspect: the WBC plans the CoP over a full sole that is resting on a line) and of the TM fall;
humanTEV = `useEstimate true` + `touchDownVelocity -0.15` (TE took the big steps, TV took the step-downs
backward; TVC failed because of the windows, not the descent); and the rung ladder re-based on the
per-stride step (`suite_fine`: adjacent 0.5-1.0 cm so the per-stride step stays under ~2 cm).

### Run 22 D — fall anatomy (humanTM, humanTE) and humanTEV: first config to walk `suite_mild` both ways

**humanTM's fall (38.8 s, 0.2 m/s, left landing on a +1.5 cm tile).** The left sole stops at z = 1.5-2.0 cm
in the estimated frame (vz ~ 0 from -50 ms before the switch) - it is standing on the higher tile - but
contactSync only knows "landed" as *sole within 2 mm of the support plane*, so the foot is never accepted as
down; the swing equality keeps driving it to z = 0 against the tile, left_hip_roll goes 27 -> 74 Nm in
50 ms, roll 0.7 -> 13 -> 30 deg, pitch 12 -> 45 deg, over in 250 ms. The mirror case is the lower tile: the
sole passes z = 0 in the air, counts as landed, and the plan loads it (the `touchDownVelocity -0.05`
extension finds the tile 300-400 ms later). The height-only rule is exactly wrong in both directions on
terrain, which is also why the window changes of humanTC could not help. **humanTE's fall (40.6 s, the
stop)**: right single support at ~0 m/s on a foot whose heel pair sits on the x-edge at 10.0 m (1.5 cm
step under the heel); as the weight moves back the foot rocks about the toe line, right_hip_roll 30 -> 139
Nm (limit) in 150 ms, pitch -1 -> -34 deg (backward), pitch rate -627 deg/s. The 4-sphere G1 sole on an
edge is a line support; walking passes over it, standing on it is a balance task the stack does not do.

**humanTEV** (`useEstimate true` + `touchDownVelocity -0.15`, vs humanTE on `suite_mild`): **0 falls both
ways**. Forward steady 48 / 53 / 109 / 35 / 73 (flat humanCSr1 27 / 34 / 66 / 33 / 27), clean stop 22 / 0.2;
backward 34 / 50 / 76 / 44 / 42 (flat 30 / 33 / 55 / 33 / 28), noisy stop (59 / 17). 1.3-1.8x the flat jerk,
v_err_max 0.25. Next: the flat regression of humanTEV (`speeds backward` vs humanCSr1) before it is
adopted; `terrain_strides.py` on it (largest steps, lateral drift); and Run 23 = a terrain-correct touchdown
in ContactSync: landed = within heightThreshold of the plane **or blocked** (sole at least `blockedHeight`
above the planned swing height while the plan descends and the measured vertical speed has stalled), and the
late half holds the switch while the foot is still descending instead of "while above the plane".

### Run 22 E — humanTEV on flat (`humanTEVf` vs humanCSr1) and its strides on `suite_mild`

**Flat regression (`SKIP_BUILD=1 SCRIPTS="speeds backward" run_suite.sh humanTEVf tuning_results/humanTEV.yaml
humanCSr1`): 0 falls, modest.** Steady jerk 33.3 vs 30.7, acc_hf 0.544 vs 0.546, tau_rate 189 vs 192,
leg_jerk 481 vs 494, transition_jerk_max 61.2 vs 62.9, v_err_max 0.192 vs 0.178. Forward holds 35.5 / 44.5 /
60.9 / 41.0 / 34.8 vs 26.8 / 33.8 / 66.4 / 32.6 / 26.8: the **0.2 / 0.5 m/s holds are +30 %**, 0.8 is better;
backward holds 30.5 / 35.2 / 56.1 / 33.1 / 28.2 vs 29.6 / 32.9 / 54.7 / 32.8 / 28.0: equal. Clean stops both
ways (0.09, 0.21). Guards: `steady_arm_acc_mean 1.21 -> 1.50 (limit 1.39)` FAIL, `solve_hz 43.7 -> 39.3
(limit 39.36)` borderline and session-dependent. The low-speed forward holds and the arm acceleration are the
`touchDownVelocity -0.15` signature (humanTV showed the same shape on terrain); `useEstimate true` alone has
not been measured on flat - do that before adopting either half.

**Strides on `suite_mild` (`terrain_strides.py humanTEV__speeds humanTEV__backward`).** Forward 37 landings,
edge 32 % (same as every run: edges are not the driver), largest steps **-4.3 cm** (t 39.3, vx 0.2, pitch
5.6 deg), +2.9, +1.7, +1.7; pitch after step-downs 3.8 deg, after step-ups 1.4 deg. Backward 38 landings,
largest -3.0 (t 32.7, vx -0.8, pitch 12 deg), +1.7, -1.7, -1.6. So the tolerance moved from ~1.7 cm
(shipped) to ~3 cm (TE) to **4.3 cm per stride** (TEV) - the `-0.15` descent finds the lower tile before
the plan loads the foot, the estimate then follows. **But the forward run drifts y = +3.96 m over
x = 10.5 m** (backward y = 0.3 m over 11.3 m; TE drifted -1.85 m). That is a 20 deg course error and
would walk off a 2 m wide course: it is the next thing to diagnose, with `tests/tuning/lateral_drift.py
<case>` (new; data.npz only): per metre of travel it prints ground-truth yaw, the path heading and their
difference (crab angle), body-frame lateral speed and the body-frame lateral offset of each landing foot,
and decomposes the final y into a turning part (integral of v sin yaw) and a crabbing part (integral of
body-frame vy). Turning = yaw drift (yaw-rate tracking / foothold yaw on the tiles); crabbing = lateral
foothold bias or stance slip.

## Run 23 — terrain-correct "landed" in ContactSync (`descentVelocity`, `blockedHeight`)

Run 22 D showed the height-only rule wrong both ways on tiles. `ContactSync` now takes per-foot
observations (`ContactSyncFootObservation`: measured sole height and vertical velocity, the previous
solve's planned swing height and velocity at the solve time) and decides `contactSyncLanded`:

- sole within `heightThreshold` of the plane **and not descending faster than `descentVelocity`** ->
  landed (a sole passing the plane at the touchdown speed towards a lower tile is in the air);
- sole stopped (slower than `descentVelocity`) **at least `blockedHeight` above the planned swing height
  while the plan descends or has finished** -> landed (standing on a higher tile);
- both knobs 0 (shipped default): the height-only rule of runs 12-18, bit for bit.

The reference manager computes the sole velocities from the state's generalized velocities
(`computeFeetVerticalVelocities`, pinocchio `getFrameVelocity` as in the tracking stream) and reads the
swing planner's previous plan (`hasTrajectory` guard for the first solve); both only when a knob is set.
The late half keeps its rule but with the new "landed": with `lateWindow > 0` a foot still descending past
its planned switch is held, a foot stopped on a lower tile (plane estimate not caught up) is released.
Unit-tested (`tests/contact_sync_test.cpp`: default rule unchanged, lower / higher tile, lagging foot,
rising plan, no plan, through the schedule with and without the late half; contract test pins the plumbing).

Proposed first measurements (all need a `colcon build`), `suite_mild`, `speeds backward`, baseline humanTEV:

- humanT23a: humanTEV + `contactSync.descentVelocity 0.05` `blockedHeight 0.01` (TEV's -0.15 descent is well
  above 0.05; the shipped -0.05 descent would need 0.03);
- humanT23b: humanT23a + `lateWindow 0.1` (the late half becomes meaningful only with the velocity rule);
- humanT23c: shipped flat config (`touchDownVelocity -0.05`, `useEstimate true`) + `descentVelocity 0.03`
  `blockedHeight 0.01` - does the velocity rule replace the -0.15 descent, which is what costs the flat
  0.2 / 0.5 holds?

Pass = 0 falls both ways, largest survived step >= TEV's 4.3 cm, holds no worse than TEV, and the flat
`speeds backward` of the winner within the humanCSr1 guards.

### Run 23 A — humanT23c (shipped descent `-0.05` + `useEstimate true` + velocity rule 0.03 / 0.01, `suite_mild`, vs humanTEV): FAIL

Only the T23c comparison came back (T23a / T23b not run). **1 fall, forward 0.8 hold** (TEV survives it at
jerk 109); forward 0.2 / 0.5 holds 34.6 / 58.5 vs TEV 48.2 / 52.6; v_err_max 0.63. Backward 0 falls but
every hold worse: 44.8 / 67.1 / 82.4 / 70.3 / 51.1 vs 34.0 / 50.4 / 75.7 / 44.2 / 41.8 (+20-60 %), clean stop
(0.17 vs TEV's noisy 16.6). So the velocity-aware "landed" with the shipped -0.05 descent does **not**
replace the -0.15 descent: it is no better than humanTE (`useEstimate` alone walked the forward ladder and
fell in the stop). Whether the rule fired at all could not be read from the run - the stream had no
contactSync telemetry - so the solver row now carries cumulative `contact_sync/advanced`, `held`,
`blocked`, `descending` counters (`plot_tracking.py` prints the per-window deltas in its DIAG line and
`diag.json` -> `solver.contact_sync`). Re-run T23c once with the counters before drawing a conclusion
about the rule itself; T23a / T23b (on top of TEV) remain the configs to measure.

### Run 23 B — humanT23a / humanT23b on `suite_mild` vs humanTEV: late half closed for good, early rule has a dead band

**humanT23a** (TEV + `descentVelocity 0.05`, `blockedHeight 0.01`): forward **1 fall in the ramp to 0.8**
(jerk 210 / tau_rate 720 in the ramp, leg_jerk 3091, arm_acc 12.4 - a violent 2-3 s, not a trip); before it
the 0.2 / 0.5 holds were *better* than TEV (31.4 / 44.6 vs 48.2 / 52.6). Backward 0 falls, holds 41.1 /
47.1 / 84.7 / 47.9 / 38.9 vs 34.0 / 50.4 / 75.7 / 44.2 / 41.8 (equal within noise), clean stop (0.8 vs 16.6).
**humanT23b** (T23a + `lateWindow 0.1`): **2 falls**, both in the ramp to 0.5, with the 0.2 holds already
at jerk 152 / 208 (3-4x) - the late half is destructive on tiles, as in Run 22 B (humanTC) and Run 18 on
flat. Mechanism: with `heightThreshold 0.002` and `blockedHeight 0.01` the rule has a **dead band**: a
sole stopped 2-10 mm above the plane (a slightly higher tile, or any tile before the estimate catches up)
is neither "on the plane" nor "blocked". In the early half that only means no advance (the shipped
behaviour); in the late half it means the switch is pushed back 20 ms per solve for up to 100 ms while the
foot stands pressed on the tile and the re-stretched swing spline lifts it again. `lateWindow` stays 0 -
third and last time.

The dead band is also the most likely reason T23a is not better than TEV: on `suite_mild` most tiles
differ by < 1 cm, exactly the band the early rule ignores. Fix without a new mechanism: with the descent
test active, an airborne sole cannot be "stopped" near the plane, so `heightThreshold` can be opened to
~1 cm -> landed = stopped and within 1 cm of the plane, or stopped and >= `blockedHeight` above a descending
plan. Next: **humanT23d** = T23a + `heightThreshold 0.01`; the `contact_sync` counters (new in Run 23 A)
and `fall_anatomy.py` on `humanT23a__speeds` to see whether the 0.8 ramp fall involved the rule at all.

### Run 23 C — humanT23d, T23a fall anatomy, TEV lateral drift: the 0.8 m/s segment on tiles is the problem

**humanT23d** (T23a + `heightThreshold 0.01`, vs humanTEV): forward 0.2 hold **32.5** (TEV 48.2), 0.5 hold 61.1
(52.6), 0.8 hold 146 (109), then **fell in the ramp down to 0.5** (jerk 329, tau_rate 756). Backward every
hold equal or better (33.1 / 51.6 / 73.7 / 47.3 / 37.9 vs 34.0 / 50.4 / 75.7 / 44.2 / 41.8), clean stop
(1.5 vs 16.6), but the file counts 2 falls with no backward segment marked - to be checked in the
backward summary. `contact_sync_counts.txt` came back empty: the solver stream of these runs has no
`contact_sync/*` columns, so either the telemetry commit was not built or `plot_tracking` did not reach
its DIAG line; check `solver_names` in `data.npz` before the next run.

**humanT23a fall anatomy (29.87 s, ramp to 0.8).** 28.63: left lands on a +1.4 cm tile, sole stopped
(vz 0.00 at -50 ms, z 1.4 cm at the switch) and is accepted at once - the blocked rule working as
designed - with plan fz 71 N at +20 ms; within 100 ms pitch 3.5 -> 7.2 deg, roll -2 -> +4 deg. The next
right swing goes wrong: both hip pitches at the 88 Nm limit at 28.92-28.97, right ankle pitch tracking
error 0.32 rad, the right foot rises to 9 cm and is **loaded 8.4 cm in the air, still moving up** at its
scheduled switch (29.20, plan fz 272 N) - the late case `lateWindow 0` cannot hold and the velocity rule
cannot help (the foot is neither near the plane nor stopped). Roll 16 -> 44 deg over the next 300 ms. So
the velocity rule accepted the right landing; what killed the run is the body's response to the +1.4 cm
step-up at ~0.65 m/s (forward pitch, saturated hips, a swing that never comes down).

**TEV lateral drift (`lateral_drift.py`, gt only - the first version mis-read the odom twist as world
velocity, fixed: velocities now come from the positions; the landing table needs `plan_mode` + two
`obs_foot` streams and now says which is missing).** Forward: y +3.88 m, yaw **+43.5 deg**, of which
**22 deg in one metre (x 4.5-5.5 m, the 0.8 m/s hold)** with body-frame vy -0.5 m/s - a yaw slip, not a
bias; before it yaw 1-8 deg, after it crab ~0 (the robot walks straight along its new heading, yaw
30 -> 44 deg over 5 m). Backward: y +0.24 m, yaw -4.8 deg, no event (the 34 deg heading in the last bin
is the stop). So TEV's drift is one near-fall at 0.8 m/s on the tiles, the same segment where T23a /
T23c / T23d fell.

**Reading.** The velocity-aware "landed" delivers what it was built for - the 0.2 / 0.5 m/s forward holds
on tiles go from 48 / 53 (TEV) to 31-35 / 45-61 (T23a / T23d), stops are clean, backward is equal - but
**no configuration is robust at 0.8 m/s on `suite_mild`**: TEV survives with a 22 deg yaw slip, the
others fall within 1-2 strides of a step-up at that speed, with hip pitch torques at the limit. At 0.8 the
stride is 0.40 m over 2-3 cells (step time 0.55 s, DS 0.12 s): the per-stride step reaches 3-4 cm and
the body has 0.12 s of double support to absorb it. Options, in order of cost: (a) **acceptance per rung
= 0.5 m/s on tiles**, 0.8 on flat (the blind walk's envelope); (b) a terrain speed cap: when the ground
estimate has moved more than ~1 cm within the last few strides, clamp the commanded speed to 0.5 until it
settles (blind, adaptive, cheap); (c) a 0.8-on-tiles study (fall anatomies of T23a / T23c / T23d at the
step-up, hip-pitch saturation; the humanTEV yaw slip at 28-30 s) to find a lever at speed.

## Run 24 — blind terrain speed cap (`reference.terrainSpeedCap`): walk the tiles at the speed they allow

Decision after Run 23 C: option (b). The walk is blind; the one terrain signal it has is what the feet
report. `TerrainSpeedCap` (`mpc/common/gait/TerrainSpeedCap.h`): when both feet are in contact, the
difference of their sole heights is the step the last stride took - frame-independent (same instant,
same estimator frame), so it works with `height.source kinematic` and with `useEstimate` on or off. A step
of at least `stepThreshold` (0.01 m) marks the ground as uneven for `holdSeconds` (3.0 s, 4-5 strides);
while that holds the commanded planar speed is capped at `maxSpeed` (0.5 m/s), direction and yaw rate
untouched. The cap is applied to the scaled command *before* the filter and slew limiter
(`WalkingVelocityTarget::scaleCommand`), so `velocityCommandRateLimits` (0.5 m/s^2) turns it into a 0.6 s
ramp; it releases the same way. Flat ground never trips it (both soles on one plane within mm); a foot
loaded in the air (the T23a right foot at 8 cm) does, which is the right response. Updated on the
solver thread in `modifyReferences` after contactSync (an advanced touchdown counts as double support at
once), flat-ground modes only; the motion manager reads it before conditioning the command. Off by
default; unit-tested (`tests/terrain_speed_cap_test.cpp`) and wired (`terrain_speed_cap_contract_test.py`).

Telemetry: solver row `terrain/cap` (cap in force, 0 = none), `terrain/steps` (double supports that
tripped it), `terrain/last_step`; `score_suite.py` adds `cap_frac` / `terrain_steps` per segment and the
cumulative `contact_sync` / terrain counters to the SUITE line and `suite.json` (the DIAG line of
`plot_tracking.py` is not produced for suite cases - that is why `contact_sync_counts.txt` was empty);
`compare_suite.py` shows the per-segment cap share.

**Reading the comparison:** with the cap active the 0.8 m/s segments run at 0.5, so `v_err_max` (script
command vs actual) reads ~0.3 by construction and the `v_err_max` guard fails - expected; judge those
segments by falls / jerk / tau_rate and by `cap` = 100 %. On flat the cap must never engage (cap 0 %
everywhere, metrics identical to the baseline).

Plan (`suite_mild`, `speeds backward`): humanT24a = humanT23d + `terrainSpeedCap.enabled true`
(defaults), vs humanTEV; humanT24b = humanTEV + cap (is the velocity rule still needed once the speed is
right?); then the flat check of the winner vs humanCSr1 (`speeds backward`, cap must read 0 %), then
`suite_rough` and the next rungs with the same config.

### Run 24 A — humanT24a / humanT24b on `suite_mild` vs humanTEV: **both walk, T24a wins**

Build picked up the telemetry (SUITE lines carry the counters). **0 falls both ways for both.** The only
guard that fails is `v_err_max` (0.42 / 0.44 vs limit 0.33): the 0.8 m/s segments ran at 0.5 under the
cap, by construction. (`score_suite.py` now tracks min(|command|, cap) in `v_err` and keeps the script
value as `v_err_raw`, so the next comparisons judge the effective command.)

| segment | humanTEV | **humanT24a** (T23d + cap) | humanT24b (TEV + cap) |
| --- | --- | --- | --- |
| fwd holds 0.2 / 0.5 / 0.8 / 0.5 / 0.2 | 48 / 53 / 109 / 35 / 73 | **37.5 / 46.9 / 64.3 / 36.5 / 61.1** | 30.0 / 69.9 / 55.1 / 45.5 / 35.0 |
| fwd stop | 0.2 | 0.1 | 15.8 (noisy) |
| bwd holds | 34 / 50 / 76 / 44 / 42 | **27.5 / 47.5 / 45.8 / 45.5 / 42.0** | 28.2 / 69.2 / 49.2 / 47.6 / 51.1 |
| bwd stop | 16.6 (noisy) | 25.6 | 0.2 |
| steady jerk / acc_hf / tau_rate | 48.4 / 0.76 / 250 | **40.0 / 0.73 / 218** | 41.4 / 0.71 / 216 |
| transition jerk max / tau_rate max | 96 / 428 | **69 / 339** | 70 / 344 |
| cap share (speeds / backward) | - | 61 % / 71 % | 55 % / 58 % |
| contactSync blocked / descending | - | 9 / 64, 6 / 70 | 0 / 0 |

Cap behaviour as designed: 0 % in the first 0.2 ramp (no tile step yet), 21-25 % in the first 0.2 hold
(first steps), ~100 % from the 0.5 ramp on, released during the stops; 8-13 double supports tripped it
per run. With the speed right, the 0.8 "hold" (at 0.5) is the quietest fast segment any config has
produced on tiles (64 vs 109), and the ramp down from it no longer spikes (T23d fell there). The
velocity-aware "landed" still earns its keep on top of the cap: T24a's 0.5 holds are 47 / 48 vs T24b's
70 / 69 both ways and its forward stop is clean. T24a's remaining soft spots: the 0.5 -> 0.2 ramp
forward (jerk 69, acc_hf 1.67) and the backward stop (25.6).

**Adopt humanT24a as the terrain candidate.** Next: its flat regression vs humanCSr1 (cap must read 0 %
everywhere; the expected cost is the `-0.15` descent's +30 % on the flat 0.2 / 0.5 holds, Run 22 E),
`terrain_strides.py` on the T24a cases (largest steps survived), then `suite_rough` (vs humanTR, the
shipped config's 2-fall run) and the slope rung with the same config.

### Run 24 B — humanT24a flat regression, strides, `suite_rough`: the cap is a schedule, not a switch

**Flat (`humanT24af` vs humanCSr1, `speeds backward`):** 0 falls, **cap 0 % in every segment** (the
step signal never fires on a plane). Cost as known from Run 22 E: forward 0.2 / 0.5 holds 34.2 / 44.6 vs
26.8 / 33.8 (+28 / +32 %), 0.8 hold 63.0 vs 66.4, backward within noise (30.6 / 35.4 / 60.2 / 31.0 / 26.7
vs 29.6 / 32.9 / 54.7 / 32.8 / 28.0); steady 34.2 vs 30.7 (primary fails at 1.11x), `steady_arm_acc` 1.74
(guard 1.39, the TEVf finding). That is the `touchDownVelocity -0.15` descent paying on the flat 0.2 / 0.5
holds, not the cap; the terrain signal is now available to make it conditional (see below).

**Strides on `suite_mild` (`terrain_strides.py`, T24a):** 34 / 35 landings, 29 / 37 % on an edge, steps up
to +-1.8 cm at 0.5-0.8 m/s and -2.8 cm at 0.2, all survived; pitch excursion after landing 1.1-1.5 deg,
height excursion 2.4-4.4 cm (down-steps the largest). Nothing in the mild field is near the limit any more.

**`suite_rough` (`humanT24aR` vs humanTR, 2-4 cm adjacent cells, mode 3 cm):** 2 falls at the same places
as the shipped config - forward in the first 0.5 hold at x ~ 3 m as the field reaches full height, backward
in the ramp to -0.5 - with the cap at 100 % both times. Of course: the cap is 0.5 and the command is 0.5,
so on this field it changes nothing. Before the falls the 0.2 holds are fine both ways (45 / 37 vs 44 / 43).
So the picture is a schedule, from Runs 22-24 together: **1-2 cm steps are walked at 0.5 m/s and lost at
0.8; 2-4 cm steps are walked at 0.2 and lost at 0.5.**

**Implemented (this commit):** the cap is now tiered. `terrainSpeedCap.stepKnots [0.01, 0.025]` /
`speedKnots [0.5, 0.25]`: a step of at least `stepKnots[i]` holds tier i for `holdSeconds`, the cap is the
smallest `speedKnots` of the tiers that hold, so the lower tier releases first and the walk speeds up in
stages (unit test covers the release order; validation requires increasing steps and non-increasing
speeds). The single-tier T24a / T24b configs keep `[0.01] -> [0.5]` for reproducibility;
**humanT24c = humanT23d + the two-tier cap** is the `suite_rough` candidate (on `suite_mild` the second
tier would only have tripped at the single -2.8 cm step during the 0.2 hold, so T24c ~ T24a there).

Open after the rough result: the first 2.5 cm step is taken *before* the cap knows about it, in the 0.5
ramp (the field's 2 m blend lets the step sizes grow from x ~ 0.5 m on). If T24c still falls at the entry,
the next lever is the hold / threshold of tier 2 (0.02) or a terrain-conditional `touchDownVelocity`
(-0.05 on flat, -0.15 once the cap is active), which would also recover the flat 0.2 / 0.5 holds.

### Run 24 C — humanT24c (two-tier cap) on `suite_rough` vs humanT24aR: 3 m further, then an edge

Still 2 falls, but a different run. **Forward:** the 0.5 hold that killed every config so far is walked
(74 vs fall), the 0.8 segments go by at the tier-2 cap (strides: 0.25-0.33 m/s from x ~ 2.7 m, jerk 24 /
48), ten landings at full field height with +-3.3-3.5 cm steps and 2.5-3.3 cm edges survive with 0.4-2.8 deg
pitch (one -3.3 cm step at 0.44 m/s, just before the cap took hold, cost 11.6 deg) - and then at 33.95 s /
x 4.95 m the left foot lands straddling a **5.6 cm edge** (no height step between the feet, the sole half
on a cell 5.6 cm higher), pitch 29 deg in one stride, fall. The cap was at 100 %, the actual speed 0.25.
**Backward:** fell at 22.6 s / x -1.5 m, same place as T24a and the shipped config: the +2.5 cm step at
21.1 s did not register as tier 2 (the estimator saw a hair under 0.025; forward the same step did not trip
it either - tier 2 engaged only at the +3.5 cm step at 24.0 s), so the ramp to -0.5 went ahead and the
next two landings sat on 2.4 / 2.5 cm edges at 0.3-0.4 m/s; pitch 56 deg.

Two readings. (1) **Tier 2 is a hair late and a hair fast**: 0.025 misses the field's typical first
step; 0.02 catches it (the field's adjacent differences start at 2 cm), and 0.2 is the speed at which 2.5
cm edges were taken calmly in the 0.2 holds of every rough run. (2) **Edge landings, not height steps, are
what end the rough runs** (45 % of T24c's landings were on an edge > 4 mm; the two fatal ones were 5.6 and
2.4 / 2.5 cm with *zero* step between the feet). The cap cannot see an edge until the foot is on it; the
walk needs to be slow enough when it happens, which is what the tiers buy. Beyond that the lever is the
foot itself (partial / line contact instead of the flat-sole assumption) - a different piece of work.

Run 24 D (config only, no build): **humanT24d** = T24c with `[0.01, 0.02] -> [0.5, 0.2]`; **humanT24e** =
three tiers `[0.01, 0.02, 0.035] -> [0.5, 0.2, 0.1]` (a 3.5 cm step says "the big edges are here").
Pass: 0 falls both ways on `suite_rough`; then `suite_mild` and flat must be unchanged (tier 2 at 0.02 may
now trip on mild's -2.8 cm step; watch the cap column). Lateral drift forward on rough (y +0.98 m at x 5 m)
is noted for the drift work.

### Run 24 D — humanT24d / humanT24e on `suite_rough`: the speed cap has done what it can

| | humanT24c `[.01,.025]->[.5,.25]` | humanT24d `[.01,.02]->[.5,.2]` | humanT24e `[.01,.02,.035]->[.5,.2,.1]` |
| --- | --- | --- | --- |
| forward | fell x 4.95 m (33.9 s), 5.6 cm edge at 0.25 m/s | fell x 2.35 m (23.9 s): +6.5 cm step onto a 3.5 cm edge in the 0.5 ramp, cap only 36 % in the 0.2 hold (different cells, different timing) | **whole speed ladder walked** (0.5 hold 32, "0.8" at 0.1-0.2: 231 with one +3.3 cm step at 8.8 deg, 0.2 hold 40) - **fell in the stop**, x 4.72 m, standing on a 2.7 cm edge at zero speed |
| backward | fell x -1.48 m | fell x -1.48 m | fell x -1.61 m |
| backward anatomy | right foot on a 2.4 cm edge, left lands on a 2.5 cm edge, 0.25-0.3 m/s actual, pitch 22-56 deg | same | same |

Three for three backward at the same two cells, forward every run ends on an edge - T24e's while standing
still. **Speed is no longer the variable**: the fatal landings have no height step between the feet
(the cap cannot see them) and the last one happened at 0 m/s. The rough field's 0.5 m cells with 2-4 cm
differences put 45-67 % of landings on an edge; a sole half on a cell 2.5 cm higher is a line contact,
and the MPC plans every stance as a full 18 x 6 cm support polygon (`contactGeometry.rectangle`
[0.09, -0.09, 0.03, -0.03], the contactMomentXY CoP constraint) while the whole-body controller clips the
CoP to 10 x 4 cm half-extents. On an edge that ankle authority does not exist; the foot rocks, the
torque rates go to 1200-1400 and the body follows.

**Decisions.** (1) The tiered cap stays as implemented, defaults `[0.01, 0.025] -> [0.5, 0.25]` pending
the `suite_mild` / flat re-check of T24e's tiers (those are the terrain rung's values if the rough
problem is solved by something else). (2) `suite_rough` is the first rung the blind flat-foot walk does
not pass; one cheap mechanism left to try before parking it: plan with the support the foot can count on
on an edge. **humanT24f** = humanT24e + `contactGeometry.rectangle [0.05, -0.05, 0.02, -0.02]` (CoP within
+-5 x +-2 cm of the sole centre, about the human 60 % of the foot) and the WB-ID clip 0.06 / 0.025 to
match. Less planned ankle torque means the MPC leans on step placement and the hips instead - the
strategy an edge allows. Risk: flat 0.8 m/s, where the CoP travels heel to toe; the flat re-check decides.
(3) Slopes next regardless: on a slope the two stance feet differ in height by stride x sin(angle)
(5 deg, 0.4 m: 3.5 cm), so the cap will read a slope as rough and settle around tier 2 - acceptable for
the rung, and the signal to make the cap slope-aware (consistent sign of the step along the direction of
travel) if the slope rung needs the speed.

### Run 24 E — humanT24f (smaller planned support polygon) on `suite_rough`: worse. `suite_rough` parked

2 falls, earlier and harder than T24e: forward at x 2.34 m in the 0.5 hold (+3.5 cm step onto a 3.5 cm
edge, pitch 30 deg), backward at x -1.47 m - the same two cells as every rough run (right foot on a 2.4
cm edge, left onto +2.4 cm). Where it did walk, it walked worse: steady jerk 151 vs 104, acc_hf 2.7 vs
1.6, the 0.2 holds the same. So the CoP constraint was not the mechanism: taking ankle authority away
from the plan costs on every stride and buys nothing on the edge, where the foot rocks whatever the plan
assumed. humanT24f is kept in `config/tuning/` as the record; `contactGeometry.rectangle` stays +-0.09 /
+-0.03.

One more thing the strides tables settle: **an edge landing halves the step the cap can see.** The
estimator reads the sole-frame height, and a sole straddling a 2.5 cm edge sits about 1.2 cm up; that is
why the field's +2.5 cm first steps never engaged tier 2 at 0.025 or at 0.02, in four runs, and the ramp
to 0.5 went ahead each time. Lowering tier 2 to 0.015 would catch them but would also put `suite_mild`
(1-2 cm tiles, walked cleanly at 0.5) at 0.2 - not a trade worth making for a field the walk cannot stand
still on (T24e fell in the stop).

**`suite_rough` is parked behind the foot.** What it needs is a stance that tolerates a line contact: an
edge signal (stance sole tilt from FK against the other foot) feeding either a re-step or a toe/heel
contact mode - a different piece of work, listed with the lateral drift. The ladder goes on with
**humanT24e** (`[0.01, 0.02, 0.035] -> [0.5, 0.2, 0.1]`): `suite_mild` re-check vs T24a (tier 2 at 0.02
may now trip on mild's larger steps - the cap column tells; acceptable if mild stays at 0 falls and the
0.5 segments keep their speed most of the time), flat re-check vs humanT24af (cap must stay 0 %), then
`slope_5`.

### Run 24 F — humanT24e re-checks (`suite_mild`, flat) pass; `slope_5` fails forward, passes backward

**`suite_mild` (humanT24eM vs humanT24a):** 0 falls; steady 36.7 vs 40.0, transition max 60 vs 69; the
tiers cap *less* than T24a's single tier did (0.5 hold 47 % vs 90 %, the last 0.2 hold 31 at 0 % vs 61 at
100 %) - the holds release in stages instead of one 3 s blanket. **Flat (humanT24ef vs humanT24af):** 0
falls, cap 0 % in every segment, steady 34.5 vs 34.2, segment by segment within noise. **The three tiers
`[0.01, 0.02, 0.035] -> [0.5, 0.2, 0.1]` are now the schema and G1 defaults of the block** (still off).

**`slope_5` (humanT24eS5, 6 m up at 5 deg from x 1.5 m, 2 m plateau, 6 m down; mirrored for -x):**
forward fell in the first 0.5 hold - on the first metres of the up-ramp - at cap 100 % (jerk 141, then
389); backward walked the whole ladder *up the mirrored ramp and down the other side* with cap 100 % from
the 0.5 ramp on, holds 34 / 47 / 46 / 47 / 42 - the slope is read as rough, as predicted (two stance feet
on a 5 deg slope differ by stride x sin 5 deg, 2-3 cm, tier 2), and the ramp is walked at 0.2. The
forward / backward asymmetry on the same incline is new. Hypothesis: the blind swing targets the current
support height, so going up a slope the foot lands early and toe-first (the ground rises under the toe),
going backward up the same slope it lands early heel-first - and the heel-first early landing is the one
contactSync's early window handles. Needs the anatomy before anything is built.

`score_suite` / `compare_suite` now print the cap *value* next to the share (`100%@0.20`), so the tier in
force is visible per segment.

Next: `fall_anatomy.py` + `terrain_strides.py` on `humanT24eS5__speeds`. The likely build is a slope-aware
touchdown: the same two-foot signal, signed and divided by the feet's separation along the travel
direction, is a slope estimate; the swing landing height becomes support height + slope x step length and
the cap stops reading a consistent incline as roughness (Run 25).

### Run 24 G — `slope_5` anatomy: the cap's own feedback loop, not the landing

The strides table (ground under each landing, 5 deg ramp from x 1.5 m): forward the walk shortened on
the ramp - strides of 0.11-0.13 m every 0.72 s (0.16 m/s) from 27.4 s, steps of 0.9-1.1 cm between the
feet, pitch 0.3-0.6 deg, nothing wrong - and then one stride of **0.42 m** at 31.0 s, +2.6 cm under the
foot, pitch 1.3 -> 28.6 deg within 100 ms of the scheduled switch, both hip pitches and the waist at
their limits, fall. Mode-switch detail: the right foot was 2.4 cm above the planned ground and moving
*up* (vz +0.01) when the schedule said contact, i.e. loaded in the air on a rising ramp (the T23a
anatomy again).

Why the long stride: on a ramp the stance feet differ in height by stride x slope, so at a 0.12 m stride
the step reads 1.0 cm - **below tier 2 (0.02)**. Tier 2 expired 3 s after the last 2 cm step (27.4 +
3.0 = 30.4 s), the cap jumped from 0.2 to 0.5, the slew limiter ramped the command up and the next swing
was planned 0.42 m ahead on a flat-ground height; the ramp was 3.7 cm higher there. The cap, on a slope,
has a feedback loop: slow -> short strides -> small steps -> release -> long stride -> trip; forward the
release came mid-ramp. Backward it never released (strides of 0.17-0.28 m kept the steps at 1.5-2.6 cm)
and the ramp was walked at 0.2, up and down, with pitch excursions below 1.2 deg at 44 landings.

So the slope itself is walkable at 0.2-0.3 as the stack stands; what is not is reading a slope as
roughness. **Run 25 A (this commit): slope-aware cap.** The signed step between the stance feet divided
by their separation along the heading (`feet xy . [cos yaw, sin yaw]`) is a slope sample, one per double
support; when the last three agree within `slopeConsistency` (0.03, about 1.7 deg) the ground is a plane:
the roughness step becomes the residual out of that plane (zero on a ramp, so no tier flapping), and the
incline gets its own tiers, `slopeKnots [0.05, 0.12] -> slopeSpeedKnots [0.3, 0.2]` (3 deg -> 0.3 m/s,
7 deg -> 0.2), held `holdSeconds` after the last consistent double support (standing on a ramp keeps it;
walking onto the flat ends the agreement at the first flat sample and the flat plane follows two strides
later). Tile fields never agree stride to stride (random sign), so they keep the raw step and the
roughness tiers exactly as before; flat ground reads slope 0, consistent, no cap. Unit test covers the 5
deg ramp (tier 2 for the first two strides, then the incline cap at 0.3 and the roughness residual 0), 10
deg up and down (0.2), the tile sequence (raw step, no incline), flat with the projection, and the
validation. Telemetry row `terrain/slope`; `score_suite` prints the mean slope per segment.

Expected on `slope_5` forward: 100 % cap at 0.3 from the third ramp stride, no release until the plateau,
0.2 strides, no 0.42 m stride. Then `slope_10` (second incline tier, 0.2 m/s; the 10 deg ramp also lands
the swing foot 3.5 cm early per 0.2 m stride, which is where Run 25 B - the slope in the swing touchdown
height - would come in if it is needed).

### Run 25 A — slope-aware cap on `slope_5`: **pass, both ways**

humanT25aS5 (humanT24e config + the slope tiers, rebuilt) vs humanT24eS5: **0 falls**, steady jerk 29.4
(T24e's surviving backward run: 41.4), tau_rate 153, every guard green. The slope telemetry reads
**+0.086 forward / -0.086 backward** on the ramp (tan 5 deg = 0.0875) and 0.000 on the flat; the cap sits
at `100 % @ 0.30` from the third ramp stride to the plateau, with `0.36` in the ramp-up segment (the two
raw-step strides at 0.2 before the plane is agreed, then 0.3), and releases on the plateau (the last 0.2
hold: 41-48 %, slope back to ~0). Forward holds 37.5 / 30.4 / 30.7 / 36.9 / 29.9 at 0.2-0.3 m/s actual,
backward 36.3 / 30.3 / 41.6 / 41.1 / 37.4; 46 landings each way, **pitch excursion 0.45-0.6 deg at every
ramp landing**, strides a steady 0.17-0.18 m, steps 1.4-1.6 cm, no 0.42 m stride anywhere. The roughness
counter stays at 0 on the ramp (residual out of the plane ~0): the cap no longer flaps.

One oddity to keep an eye on: the very first landing of the forward run (standing start, flat) shows a
20.6 deg pitch excursion (segment `ramp -> 0.2`: jerk 59.7, pitch_pp 21). It recovered by the next
stride and no other run of this series shows it; if it repeats, it is a start-up transient of the
estimator / contactSync at the first step and gets its own anatomy.

Rung status: `suite_mild` pass (T24e), `suite_rough` parked (foot), `slope_5` pass (T25a). Next:
`slope_10` (second incline tier, 0.2 m/s; the swing foot lands 3.5 cm early per 0.2 m stride going up and
3.5 cm late going down - this is where the touchdown height may need the slope, Run 25 B), the
`suite_mild` re-check with the slope code in (the slope must never go consistent on tiles), then
`curb_4`.

### Run 25 B — `slope_10`, `curb_4`, and the `suite_mild` re-check with the slope code in

**`curb_4` (humanT25aC4 vs T25aS5): 0 falls both ways - rung 4 passes.** Six 4 cm curbs per direction
(up and down, between 3 m plateaus). The cap reads each curb as a tier-3 step (`43-62 % @ 0.10-0.14` in
the segments with a curb, back to 0 % on the plateaus - exactly the design: a curb is one big step, then
flat), the strides table shows the curb landings at 2.8-3.9 deg pitch forward and one 14.4 deg excursion
backward (the -4 cm curb down at 0.8 m/s, 33.8 s) that recovered. Guards: `tau_rate` max 384 (263 on the
5 deg ramp), transition jerk 75.6 at the curbs; `v_err` by construction. The blind swing (5 cm swing
height) clears a 4 cm curb it does not know about.

**`slope_10` (humanT25aS10): walks the whole ladder both ways at the second incline tier (`100 % @
0.15-0.28`, slope +-0.17 read correctly), then falls in the stop - both ways.** 46 / 47 landings on the
ramp at 0.3-1.5 deg pitch; forward it stalled for 1.5 s mid-ramp (strides of 0.01 m at x 4.66-4.69,
command 0.2, cap 0.15) and recovered. The stop: the last stride steps *back down* (-3.3 cm forward,
-2.4 cm backward), pitch 15 deg, fall. The strides table has the reason in its `edge` column: **3.0 cm at
every landing on the 10 deg ramp** (1.5 on the 5 deg one) - the flat sole on a 10 deg incline rests on
its heel or toe edge (0.2 m x tan 10 deg = 3.5 cm), because the foot orientation reference is flat
(task-space foot cost, orientation weights 1000, and the stance orientation feedback). Walking tolerates
it at 0.2; standing on two edges, with the weight shifting to stop, does not (the `suite_rough` lesson
again). The slope estimate now exists to fix this: a foot pitch reference that follows the plane (swing
orientation target and the stance orientation feedback both), with the touchdown height = support
height + slope x step along the heading. That is **Run 25 C**, a real piece of work in the foot
references; `slope_10` is parked behind it, as a pass-with-a-caveat (walks, cannot stop).

**`suite_mild` re-check (humanT25aM vs T24eM): 1 fall and `solve_hz` 45.5 -> 35.7.** No segment carries
the F marker, so the fall is after the last script segment or in a transition the segmenter does not
own; the cap column reads `@0.50` in every capped segment (tier 1 only - the slope logic never went
consistent on tiles, as intended, but T24eM's tier-2 episodes are gone too, which may just be the
different landing cells of a different run). The holds are T24eM's within noise (33 / 53 / 36 / 49 / 60
vs 31 / 36 / 59 / 50 / 31 forward; backward 28 / 56 / 41 / 44 / 36 vs 38 / 42 / 42 / 43 / 65). The solve
rate drop is the first thing to rule out (the slope code is a handful of flops per solve; three suites
running concurrently on one machine is the likely cause) - a 36 Hz solver changes the walk. Anatomy
requested before any conclusion.

### Run 25 B′ — `suite_mild` re-check anatomy: a standing fall on two tile edges, 5 s after the last step

**Forward (humanT25aM): 0 falls; the anatomy window (45-47.5 s) shows nothing** (height error 2.9 cm is
the only flag). Backward: **1 fall at 44.43 s, standing, in the final 0 m/s hold, 5.2 s after the last
landing (39.18 s).** `plan_mode` is STANCE for the whole window - no swing, no step, no recovery step
possible (the stance rung cannot step; the command is 0). The estimator is not involved (estimated vs
TRUE pitch agree to 0.1 deg until the collapse, the height error appears only after it). There is no
reference discontinuity before 44.3 (largest cmd_q jump is the waist pitch *during* the collapse), the
cap read `@0.50` only and the slope never went consistent - so the slope code is not in the loop.

The timeline is a slow lean and a sudden collapse. 41.9-43.7 s: pitch drifts 1.3 -> 2.7 deg and roll
-0.1 -> -2.0 deg with `right_ankle_pitch` torque rising 15 -> 24 Nm (far from its 50 Nm limit) and
nothing else moving; 43.73-43.98: `right_hip_roll` jumps 12 -> 27 Nm, roll -2.0 -> -3.7 (weight shifting
to the left foot); 44.03-44.18: pitch 3.7 -> 12 deg in 150 ms (TRUE pitch rate ~ 585 deg/s) while both
feet still read z ~ 0; 44.28: both hip pitches at the 88 Nm limit, knees 0.27 rad off their reference;
44.43: `policy_valid` 0, plan nan, 39 deg pitch. The feet: the strides table's last three landings are
`r 37.74 step +1.7 edge 1.7`, `l 38.46 step 0 edge 1.7`, `r 39.18 step -1.7 edge 1.7` - the robot
stopped with the right foot 1.7 cm lower than the left and **both soles across a 1.7 cm tile edge**. A
sole on an edge has a line of support in one axis, so the ankle torque the WBC asks for is not fully
realizable; the foot rocks, the base leans, the lean grows until the weight shift pushes the other foot
over its own edge. That is the `suite_rough` failure class (T24eR: fell in the stop on a 2.7 cm edge)
one tier down, and the same thing that ends `slope_10` (3.0 cm `edge` every landing, falls in the stop).
Walking survives edges because the next step replaces the foot; standing on them has to hold.

Why this run and not T24eM: the edge column is 45 % of landings in both runs; which cells the *final
two feet* land on is a function of the whole stride history, and T24eM stopped on flat cells. The
`solve_hz` drop (45.5 -> 35.7) is the only systematic difference and it moves the history (a 36 Hz
solver re-plans later, the strides shift by millimetres, the stop lands elsewhere). It is almost
certainly the three concurrent suites, not the slope code (a handful of flops per solve). A clean solo
mild re-run (`humanT25aM2`) settles the solve rate; the standing-on-edges mechanism it cannot fix, and
no speed cap can either - the cap is a walking tool and this happened at 0 m/s. Fix lives in the foot:
either the foot orientation follows the local support (Run 25 C's foot pitch to the plane handles ramps
but not a tile edge under the mid-sole), or standing on a rocking foot is detected (ankle torque
drifting against a flat-foot reference while in stance) and answered with a weight shift / a corrective
step. Logged as a watch item; `suite_mild` keeps its pass with the caveat "stops on an edge 1 in 4".

SUITE lines for the record: backward steady jerk 36.8 / tau_rate 179 / v_err 0.117, transition jerk
max 60.2; forward steady 38.7 / 223, transitions 46.4 / 282 (T24eM: 36.7 forward steady).

Rung status unchanged: `suite_mild` pass (1 standing-edge fall in 4 T25a/T24e legs), `suite_rough`
parked (foot), `slope_5` pass, `slope_10` walks / falls in the stop (25 C), `curb_4` pass, `curb_8` not
worth running until the swing height follows the cap tier.

### Run 25 C — foot references on the incline the cap reads (`groundHeight.followSlope`, humanT25c)

**What the flat-ground references do wrong on a ramp.** The swing update gives both feet one support
height - `computeGroundHeightEstimate`, the *mean* of the stance soles - and the stance equality
constraint tracks it with `positionZ` gain 200: on a 10 deg ramp the stance feet differ by 3.5 cm, so
the rear foot is pulled up and the front foot down by 1.75 cm, all the time. Every foot orientation term
- the task-space foot cost (orientation weight 1000), the stance orientation feedback
(`orientationErrorGain` 20), the friction cone and the initializer - reads
`SwitchedModelReferenceManager::getContactNormal()`, which is the world vertical outside the terrain
modes: the sole is held level on a 10 deg plane and rests on its heel or toe edge. That is the
`edge` 3.0 cm at every `slope_10` landing (1.5 on 5 deg), and standing on two edges is how both
`slope_10` runs ended (run 25 B). The swing touchdown height is the same mean, so the foot lands
3.5 cm early going up and late going down per 0.2 m stride, which contactSync has been absorbing.

**Change** (`SlopeFootReference.h/.cpp`, `reference.groundHeight.followSlope`, default off; the G1
file off, `config/tuning/humanT25c.yaml` = T24e / T25a + the flag):
- while the cap's slope samples agree (`terrainSlope() != 0`), the flat-ground branch of
  `modifyReferences` plans per-foot support heights instead of the single mean: **a stance foot's
  reference is its own measured sole height** (no pull, whatever the plane fit); the plan is anchored at
  the leading stance foot (furthest along the travel direction - the one that landed last) and the
  n-th touchdown in the horizon is planned at `anchor height + slope x v x (t_touchdown - t_anchor)`
  (footfalls of a periodic gait at speed v are v x elapsed time apart), the lift-off height of the
  foot in the air from the same rule at its lift-off time; below 0.05 m/s the anchor is the feet's
  midpoint and every touchdown is planned at its height (a first step from standing lands next to the
  feet). `touchDownHeightOffset` is applied as on the flat path. Base height adaptation is unchanged
  (still the mean, rate-limited).
- `getContactNormal()` returns the plane's normal, `normalize(-slope x heading, 1)`, published by
  `updateTerrainSpeedCap` as a world-frame gradient (zero while flat or when the flag is off), so the
  foot cost, the stance orientation feedback, the cone and the initializer all pitch the sole to the
  ramp without touching their code.
- Gates: `useEstimate` (the heights are measured) and `terrainSpeedCap.enabled` (the slope); the
  builder refuses the flag without them. Flat ground and tile fields never read a consistent slope
  (`slopeConsistency` 0.03), so `suite_mild` / flat are unchanged by construction; the plane is dropped
  at the first double support whose sample disagrees (plateau after a ramp: one step of lag, same as
  the cap, and that step lands as it does today).
- Telemetry `terrain/next_touchdown` (the planned plane height of the next touchdown, NaN while flat).
  Tests: `tests/slope_foot_reference_test.cpp` (up / down / standing / flat / flight / normal),
  `tests/slope_foot_reference_contract_test.py` (plumbing, candidate = T24e + the flag).

**Expectation.** `slope_10`: `edge` 3.0 -> < 1 cm at the landings, the stop survivable (both soles flat
on the ramp), pitch per landing lower; `slope_5` at least unchanged. Not addressed: a tile edge under
the mid-sole (`suite_mild` stop fall, `suite_rough`) - a plane has no edges.

Run (the slope_10 pair first, then slope_5 and the mild re-check for no regression):

```bash
TERRAIN=slope_10 SCRIPTS="speeds backward" tests/tuning/run_suite.sh humanT25cS10 config/tuning/humanT25c.yaml humanT25aS10
SKIP_BUILD=1 TERRAIN=slope_5 SCRIPTS="speeds backward" tests/tuning/run_suite.sh humanT25cS5 config/tuning/humanT25c.yaml humanT25aS5
SKIP_BUILD=1 TERRAIN=uneven_suite_mild SCRIPTS="speeds backward" tests/tuning/run_suite.sh humanT25cM config/tuning/humanT25c.yaml humanT25aM
~/.venvs/replay/bin/python tests/tuning/terrain_strides.py tuning_results/humanT25cS10__speeds tuning_results/humanT25cS10__backward
```

### Run 25 C′ — first humanT25c results: `slope_10` forward walks *and stops*, backward stalls; two bugs fixed

**Numbers (solver telemetry in these runs is misaligned, see bug 1: the `cap` / `slope` columns, the
`v_err` guard and the SUITE contactSync / terrain counters are garbage; jerk, tau_rate, falls, z_m,
step_l are from the tracking and ground-truth topics and valid).**

- **`slope_10` forward (humanT25cS10): 0 falls - the ladder and the stop.** The stop that ended T25aS10
  (jerk 310 / tau 484, fall) is a 25.6 / 131 transition and a 0.33-jerk hold: both soles flat on the
  10 deg ramp, standing. Holds 32 / 35 / 37 / 31 / 32 jerk vs T25a's 41 / 40 / 47 / 27 / 41, tau_rate
  201 / 182 / 200 / 176 / 187 vs 226 / 189 / 223 / 140 / 180. The base reached z 1.47 (5.6 m up the
  ramp) vs 1.30.
- **`slope_10` backward: fell** in the stop again, and the walk degraded from the -0.5 ramp on:
  z_m 0.87 -> 0.93 at the -0.8 ramp, then 0.88 / 0.89 / 0.87 / 0.85 / 0.84 - the robot stopped climbing
  about 0.85 m up the ramp (T25a climbed to 1.14) and stood fighting for 30 s (jerk 45-74, tau_rate
  214-462, leg jerk 1282-1492 in the transitions, step_l 0.06-0.08) before the stop toppled it.
  Forward and backward are the same geometry (mirrored ramp, both uphill), so the asymmetry is in the
  walk, not the terrain. Anatomy and strides requested before any conclusion.
- **`slope_5` (humanT25cS5): 0 falls both ways**, holds within noise or better (backward 28-37 jerk vs
  30-42, forward 27-38 vs 30-37; tau_rate 150-220 vs 162-217). The only guard that failed is `v_err`,
  which is bug 1.

**Bug 1 - solver telemetry tail (fixed).** `publish_solver_metrics` wrote the row's tail (mesh,
contactSync, terrain) from `k = row.size() - 12`, a hard-coded count; run 25 C added a 13th tail
column (`terrain/next_touchdown`), so every tail value landed one column late - `terrain/cap` read the
contactSync `descending` counter (hence `100 % @ 3..65`), `v_err` saw no cap and compared the robot's
0.2 m/s with the 0.5 / 0.8 commands - and the last `put` wrote one double past the vector's end. Now
the tail starts at the recorded column of `grid_nodes` (`solver_tail_start_`), whatever its length.
Only runs built from `5fd1ed5` are affected (the three humanT25c suites).

**Bug 2 - the air foot's lift-off height (fixed).** `planSlopeFootReference` put the swinging foot's
lift-off at the plane height of its *lift-off time* under the footfall rule; the rule maps *touchdown*
times to positions (the foot stood still from touchdown to lift-off). Lift-off 0.25 s after the
anchor landed was planned 0.9 cm *above* the anchor instead of one stride x slope *below* it (-2.5 cm
on 10 deg at 0.2 m/s): the swing spline's start node sat 3.4 cm above the real foot, so the swing
height constraint (gain 200) yanked the foot up at every lift-off. Now the air foot's height is the
plane at its own last touchdown (from the schedule, else one period before the anchor's); the unit
test asserts it is below the anchor going up and covers the in-schedule case. Symmetric in direction,
so it does not by itself explain the backward stall, but it is 3.4 cm of error at every lift-off and
had to go before the re-run.

Re-run both `slope_10` legs from the fixed build (`humanT25c2S10`); the `suite_mild` re-check
(`humanT25cM`) is still outstanding (its SUITE file came back empty).

### Run 25 C″ — `slope_10` re-run from the fixed build (humanT25c2S10): forward solved, the *backward stop* still falls

Both legs of `slope_10` with humanT25c from `2528503`, against the T25a baseline (humanT25aS10). The
telemetry tail is right this time (cap 0.12-0.28 m/s on the ramps, 100 % under cap there, contactSync
counters plausible), so the numbers below are the first valid read of `followSlope` on 10 deg.

- **Forward (`speeds`): 0 falls including the stop.** Holds 35.5 / 29.7 / 36.0 / 40.3 / 36.5 jerk vs
  T25a 40.7 / 39.7 / 47.0 / 27.3 / 40.5; the ramp-to-stop transition 28.3 (T25a fell there), the stop
  hold 3.39 at z 1.374 (the top plateau). Steady jerk 30.2, acc_hf 0.47, tau_rate 172; contactSync
  advanced 26 / held 0 / blocked 20 / descending 14; 24 terrain steps, 75 % of the run under cap.
- **Backward: FELL at 36.5 s, in the stop** - the same place as T25a and as the buggy T25c run. The
  walk itself is now *better* than T25a: hold z 0.867 / 1.029 / 1.200 / 1.228 / 1.221 vs
  0.863 / 1.006 / 1.104 / 1.079 / 1.143 (it climbs further at every speed, no stall), hold jerk
  37.5 / 32.7 / 32.4 / 34.8 / 44.3 vs 28-55. But the -0.2 m/s *return* hold is already poor (step_l
  0.06, v_err 0.24, leg_jerk 541) and the ramp-to-stop transition peaks at jerk 424 / tau_rate 509 with
  arm_pp 59.4 deg; the stop hold ends at z 0.364 (on the ground). `descending` counted 45 vs 14 forward.

So `followSlope` fixed what it was built for - the references on the incline - and forward 10 deg is a
pass. What it did not fix is **stopping on the incline while facing downhill**, which fails identically
with or without the slope references. Two things differ between the two stops: (1) the trailing foot
at the stop is the *uphill* one going backward (the robot keeps facing +x and walks −x onto the ramp
mirrored at −x: the rear foot in travel terms is the front foot in body terms); below `standingSpeed`
the plan anchors at the feet's midpoint
with v = 0, so both stance references move to the plane at the midpoint - fine for the sole heights,
but the plane *normal* keeps the heading's sign, and the swing-free stance has nothing to correct a
reference that is tilted the wrong way about the body's y; (2) the backward `descending` count is
3x forward, i.e. contactSync had many more "foot still descending at the switch" events - landings on
the toe (the uphill corner going backward) that never settle. Which of the two it is cannot be read
from the SUITE tables; the strides table decides it.

`terrain_strides.py` now prints a **`tilt` column**: the sole's pitch relative to the ground under it
0.2 s after each touchdown (the ground-truth base pose plus the measured joint angles posed in the
scene, the ankle-roll link's x axis against the ramp's heel-to-toe rise; positive = toe up / resting on
the heel, negative = on the toe). On a ramp `edge` is just the ramp's own 3.0 cm rise under the sole and
says nothing about the foot; `tilt` does. Checked on a synthetic case: a level sole on the +10 deg
up-ramp reads −10.0, the same ankle pitched 10 deg nose-down reads −20.0.

Next: `terrain_strides.py tuning_results/humanT25c2S10__backward tuning_results/humanT25c2S10__speeds`
and `fall_anatomy.py tuning_results/humanT25c2S10__backward` - if backward landings carry a consistent
non-zero `tilt` where forward ones do not, the stop fails because the feet never sat flat on the ramp
(contactSync / touchdown-velocity territory); if `tilt` is small on both and the fall starts after the
last step, it is the standing reference (the midpoint anchor and the normal's sign) and the fix is in
`planSlopeFootReference`'s standing branch and `getContactNormal`. `followSlope` stays off in the
shipped config until this is settled; humanT25cM (`suite_mild`) is still outstanding.

**Strides and anatomy of the backward stop (humanT25c2S10__backward).** The `tilt` column answers the
question: **the feet sit flat on the ramp both ways** — mean |tilt| 0.15 deg backward (max 0.87) and
0.09 deg forward (max 2.2), 0 % of landings over 3 deg, 44 landings each. Cause 2 (toe landings that
never settle) is out; `descending` 45 vs 14 is not a landing-quality signal here. What the table shows
instead is that **the slow backward climb had already stopped before the stop**: in the −0.2 m/s
return hold (t 44-51 s, 2.7 m up the ramp at ground 47 cm) the feet alternate at x −4.3 … −4.1 without
progressing and the pair drifts *downhill* (r landings −1.0 / −1.2 / −1.3 / −2.3 cm below the
previous l, x of r −4.25 → −4.05), with a 9.1 deg pitch excursion and 6.4 cm dz at 49.8 s; that is the
`v_err 0.24` and `leg_jerk 541` of the hold (commanded −0.2 uphill, moving ~+0.02 downhill). Forward
at +0.2 on the same ground climbs at 0.14 m/s with 0.5 deg excursions. The faster rungs were fine
backward: −0.5 at 0.115 m/s vs forward 0.13, −0.8 at 0.16 vs 0.16, the −0.5 return at 0.2.

The fall itself is a clean tip-over from a quiet stance, not an impact: last touchdown 51.93 s, then
STANCE with pitch −0.3 deg; from 52.1 s the true pitch climbs monotonically 0.1 → 0.6 → 1.5 → 3.5 →
8 → 18 → 34 deg (nose-down, i.e. downhill), *while the ankle-pitch torques fall* (right 13 → 9 Nm,
left 11 → 6 Nm up to 52.41 s) - the controller is not pushing the CoP toward the toes to arrest it.
Hip pitch / waist / shoulders saturate at 52.68 s, the estimator matches ground truth throughout
(pitch error 0.4 deg, obs-plan z 0). An inverted pendulum that the whole-body controller is tracking,
not fighting, points at the *reference* (base pitch / position, or the contact wrench the plan asks
for), which the anatomy did not print: its `plan_base/pitch` line was nan-poisoned by the post-fall
samples (fixed: nan-aware), and the timeline now carries the planned base pitch, the x and z obs-plan
gaps and the planned centre of pressure per foot (−my/fz) so the next read shows whether the plan
asked for the tip.

Two scripts to separate the hypotheses without guessing (`tests/tuning/suite/`): `slope_about_face`
walks forward onto the ramp, stops, turns ~180 deg in place, stands 6 s facing downhill, then walks
0.2 m/s downhill and stops - if the stand after the turn falls, standing on the incline facing
downhill is the problem (the reference); if it survives, the slow backward gait leaves the robot in a
state the stop cannot hold. `slope_back_slow` is −0.2 m/s for 20 s and a stop - whether the slow
backward climb stalls on its own at this height or only after the faster rungs.

### Run 25 C‴ — the two scripts and the anatomy with the plan columns: the 10 deg stop fails *both ways*; the plan saturates the CoP from the first stance instant

- **`slope_about_face` fell at the forward stop, before the turn** (19.4 s of script, 0.4 → 0 m/s at
  x 2.9, 25 cm up the ramp). The walk under the cap was fine (jerk 35.5, cap 77 % @ 0.21, robot at
  ~0.17 m/s); the last landing of the stop step (l at 35.59 s) went 16 cm *downhill* of the other
  foot (step −2.7 cm) with a 10 deg pitch excursion, then the stop hold tipped (pitch_pp 83,
  roll_pp 154). The turn and the stand facing downhill were never reached, so the script's question
  is still open - but the premise changed: **the forward stop on 10 deg is not solved either**; the
  `speeds` run's forward stop (0.2 → 0 at 63 cm up, robot at 0.14 m/s) is the one that happened to
  hold (its planned CoP sat at −0.5 … +0.8 cm, dead centre, the whole stance).
- **`slope_back_slow` fell at the stop too** (23.4 s), after a *clean* −0.2 m/s climb: 0.15 m/s over
  2.2 m, v_err 0.13, jerk 38.6, no stall - so the stall of the `backward` script's return hold is not
  required for the backward stop to fail. Stop transition jerk 467 / tau 446, roll_pp 148.
- **Anatomy of the backward stop (humanT25c2S10__backward) with the plan columns.** From the first
  stance sample (51.96 s, pitch −0.2 deg, feet flat, obs−plan x and z = 0) the **planned CoP of both
  feet is at +5.6 / +6.4 cm and climbs to +8.5 / +9.0 cm by 52.4 s - the toe edge of the modelled
  rectangle (±9 cm)** - while the robot is still upright and quiet; during the preceding single
  support the stance foot's planned CoP was already +7.4 cm. The plan's own pitch then *follows* the
  real one (1.9 / 2.0 at 52.46, 4.4 / 5.3 at 52.71, 10.6 / 17.8, 19.2 / 27.0, 25.4 / 30.9): the MPC's
  optimal trajectory *is* the tip-over - with the CoP constraint saturated downhill it has no input
  that keeps the body up, and the hip-pitch / waist / shoulder saturation at 52.68 s is its hip
  strategy, not a tracking failure. The forward stop that survived never asked for more than 0.8 cm.
  So at the backward stop the body is left further downhill of the feet than the sole can hold, and
  nothing in the stance phase brings it back: the base x reference is re-anchored at the *current*
  base pose every solve (`commandedVelocityToTargetTrajectories`), so once the command is 0 the plan
  has no pull toward the support centre - benign on flat ground, fatal on an incline where the last
  steps were placed for a speed the body did not have.

Two places this is made, both in the stop's last steps rather than in the stance itself:
1. `planSlopeFootReference` takes `velocityAlong` from the *measured* base velocity. When the body is
   slower than the command (the stall at −0.2: 0 vs −0.2; the about_face stop: 0.17 under a 0.4
   command ramping down) the feet are still placed per the command while the footfall rule plans them
   at the measured speed - below `standingSpeed` 0.05 every footfall is at the midpoint height, so a
   foot stepping 14 cm uphill is told to land 2.5 cm low and meets the ground early (`descending` 45
   vs 14, the stall sustaining itself). The commanded (conditioned) velocity is the right input: it is
   what places the feet.
2. The stance phase needs the base pulled over the feet on an incline (a base x reference at the
   feet's midpoint, or the capture-point logic that already exists for footholds), since the
   re-anchored reference will hold whatever offset the last step left.

Before touching either, two columns were added to measure them on the existing bags:
`terrain_strides.py` now prints per landing `v` (measured velocity along the heading - the value the
footfall rule used), `base` (base ahead of the feet's midpoint along the heading, cm) and `tdref`
(planned `terrain/next_touchdown` minus the foot's settled height - negative = reference below the
landing, early contact), with a summary line for landings at |v| < 0.05; `fall_anatomy.py`'s timeline
carries `base ahead of feet`. If `tdref` is ≈ −2.5 cm on the stalled landings and ≈ 0 on the clean
ones, (1) is confirmed; if `base` at the last landing before each failed stop is beyond ~+8 cm
(downhill of the feet) and inside ±3 cm for the forward `speeds` stop, (2) is what the stop has to
undo.

### Run 25 D — found: the plane dropped out at every failed stop; the slope estimate now keeps the last agreed plane

The `v` / `base` / `tdref` columns on the four `slope_10` cases (humanT25c2S10 speeds + backward,
humanT25c2X about_face + back_slow):

- **`base` clears hypothesis (2).** At the last landing before each stop the base is within
  −4.6 … +1.7 cm of the feet's midpoint (speeds fwd −4.2, backward +1.7, about_face −4.6, back_slow
  −2.3 / +0.7); the anatomies show the offset *developing during* the tip (about_face −4.7 → −16 cm,
  back_slow −2.3 → +7 cm), not before it. The body was over the feet when the gait stopped.
- **`tdref` = nan is the finding.** `terrain/next_touchdown` is nan only in the flat branch
  (`slopeNextTouchDownHeight_` is stored nan there), so a blank `tdref` means that solve ran *without*
  the plane: level swing targets, mean-height stance pull, contact normal (0, 0, 1) - humanT25a. On the
  one stop that survived (speeds forward) `tdref` is present on every landing from 47.6 s to the stop.
  On the three that fell it is blank through the stop: backward from 44.0 s (the whole −0.2 return hold
  and the stop), back_slow on the three closing steps (39.0-40.4 s), about_face from 30.6 s. **All
  three failed stops ran in the flat branch; T25a, which never has the plane, fell in both stops.**
  Over the ramps the plane was present on only ~1/3 of landings.
- Why it drops: one slope sample per double support, `dz / dd`, accepted down to `dd` = 8 cm, and the
  ramp gait alternates long / short steps (19 / 8 cm along x). On an 8 cm base ±4 mm of sole height is
  ±0.05 of slope, more than the 0.03 consistency band, so every other triple disagrees - and the code
  then *zeroed the plane* (`slopeConsistent ? slope : 0`) until three clean samples lined up. The stop's
  closing steps are short, so a stop almost always landed in the flat branch.
- `tilt` stays ~0 everywhere (max 0.9 deg backward, 3.6 deg on one about_face landing): the feet sit
  flat whichever branch runs; it is the references and the normal in stance that tip the robot.

**Change (code, no config):** `TerrainSpeedCap::update` keeps the last plane three samples agreed on
when a later triple disagrees; the plane changes only when three new samples agree on something else
(three ~0 samples on the plateau return it to flat). While a kept plane and the current triple
disagree, a step counts as roughness only if neither level ground nor the plane explains it
(`min(|dz − slope·dd|, |dz|)`), so the plateau after a ramp does not read as 2.6 cm steps and the
incline cap lifts as before. Unit test: 10 deg plane, two noisy 8 cm samples (0.226 / 0.126) keep
0.176, three flat samples return 0; the walk-off-the-ramp case keeps `stepCount == 2`. Contract test
pins the rule. This also touches humanT24e / T25a (the cap's incline tier now holds through a noisy
triple instead of lapsing for three strides), which is the intended behaviour there too.

`terrain_strides.py` `tdref` is now the planned *rise* over the stance foot minus the true rise (the
estimator frame re-references at touchdown, so planned-minus-landed heights carried that shift), and
the summary prints the share of landings with the plane active - on an incline that number should
be ~100 % after this change.

Rebuild (no `SKIP_BUILD`) and re-run the four `slope_10` cases with humanT25c (`speeds backward
slope_about_face slope_back_slow`); then `slope_5` both ways and `suite_mild` as regression. Expect: plane
active ~100 % on the ramps, all four stops standing. Still open after that: `tdref` on the ramp (the
footfall rule uses the instantaneous base velocity), humanT25cM.

**Run 25 D results (humanT25c3S10, rebuilt with the kept plane; baseline humanT25c2S10).** The plane is
active on 93-96 % of landings (the rest are the flat approach before the ramp), and the footfall rule
is accurate once it is: planned minus true rise mean −0.12 / +0.26 / +0.29 cm, mean |err| 0.4-0.6 cm
(the +1.5-3 cm of run 25 C‴ was the estimator-frame artefact).

| case | before | now |
|---|---|---|
| speeds (fwd ladder + stop) | survived, stop hold jerk 3.39, top z 1.374 | **survived**, stop hold **0.57**, top z 1.464 (climbs further), steady 30.0 / 0.49 / 161 |
| backward (bwd ladder + stop) | **fell** in the stop (424 / 509), −0.2 return hold stalled (v_err 0.24, leg_jerk 541) | **survived**, stop hold **1.10**, z 1.693 (vs 1.221 - no stall), steady **26.4** / 0.37 / 132, v_err 0.09 on every rung, contactSync advanced 2 / blocked 2 (vs 26 / 20), terrain steps 3 (vs 24) |
| slope_back_slow (−0.2 x 20 s + stop) | **fell** in the stop (467 / 446) | **survived**, stop hold 6.5, steady 21.4 / 0.27 / 103 |
| slope_about_face | **fell** at the forward 0.4 → 0 stop | the stop **stands** (hold 0.87) - then **fell in the turn in place** on the ramp (hold w 0.5: jerk 406, roll_pp 145, 44.25 s sim) |

Four stops on 10 deg, four standing, both directions; the backward ladder is now the *smoother* of
the two. The incline cap settles at 0.20 and the roughness tiers stop firing on the ramp (steps 3 vs
24), which is also why the −0.2 return hold no longer stalls: the cap had been pushed to 0.12 by false
roughness steps and the gait was placing feet for a command the body was not given.

**The turn in place is the next limit, and it is by construction.** The plane is a scalar slope *along
the heading* and its gradient is re-oriented with the current heading every solve
(`slopeGradient = slope x heading`); a turn in place takes no samples (the feet are side by side
along the heading, `dd` ~ 0), so the kept +0.19 is swung round with the body - after 90 deg the plane
tilts sideways across the real ramp, after 180 deg it is upside down. The estimate has to become a
world-frame gradient: each double support gives one component, `dz = g . dp` along the feet's
separation vector `dp`, and the separation vector rotates with the robot during a turn, so a small
least-squares window over recent double supports observes both components (walking straight leaves
the across-track component unobserved: regularise it towards its previous value). That is a design
step (run 26), not a tweak. Whether the turn itself is survivable on 10 deg with level references is
the control: run T25a on `slope_about_face` - if it falls in the turn too, the gait is the limit there,
not the plane.

Status of `followSlope`: ready for the regressions (`slope_5` both ways, `suite_mild`, flat `speeds`),
since the kept-plane change also touches the cap under humanT24e / T25a. Then ship as humanT25c.

**Run 25 E: regressions of the kept plane (humanT25c3S5 / M / F; T25aS10X not run).** Flat and
suite_mild are unchanged (humanT25c3F vs humanT24ef: steady 38.6 / 0.63 / 214 vs 38.4 / 0.66 / 219,
0 falls, cap 0 %; humanT25c3M vs humanT24eM: 0 falls, steady 35.2 / 0.59 / 202 vs 36.7 / 0.68 / 206,
"no gain" - the roughness path is untouched). **slope_5 regressed at the crest.** The course is
6 m up, a 2 m plateau, 6 m down; at 5 deg the robot reaches the plateau inside the 0.2 return hold
(z_m 1.27). Forward survived but the last three segments went 43 / 35 / 23 → **79 / 55 / 73** jerk with
pitch_pp 8 / 5 / 10.5 deg, and the stop landed 6 cm down the far ramp (z 1.206 - the candidate is
~0.5 m ahead of the baseline all the way up). Backward **fell at the stop** (39.4 s; ramp → −0.2 at
the crest: jerk 28 → **80**, pitch_pp 15 deg; hold 0 roll_pp 170) where humanT25cS5 stood. The
telemetry says why: `slope` reads +0.086 at 100 % of solves through the ramp → 0.2 transition (the
crest), +0.002 / −0.052 only in the hold after it. That is the kept plane doing on the plateau what it
was meant to prevent on the ramp: `12ed6f4` keeps the plane until three *new* samples agree, so for
three more double supports past the crest the footfall rule plans the feet to keep rising
(0.086 x 0.26 = 2.2 cm per step into the air over level ground, 5 cm per step would be the figure
on slope_10) and the contact normal stays tilted. Before `12ed6f4` the first flat sample zeroed the
plane and the crest was benign (the baseline crossed it at jerk 28-43 both ways) - the two failure
modes are the same rule read from opposite sides: a disagreeing *slope* sample is noise on a short
base and evidence on a long one.

**Fix: agreement in height, not in rise / run.** A standing plane now falls only to a sample whose
rise is off by more than `max(slopeConsistency x run, slopeHeightTolerance)` (new key, 0.01 m): the
4 mm of sole noise on a stop's 8 cm closing step (0.05 of slope - what dropped the plane in run 25 C)
stays inside it, the first full stride on the plateau (2.6 cm off at 0.3 m on 5 deg, 2.5 cm at the
0.14 m strides of the slope_10 cap) contradicts it and drops the plane at once, as before `12ed6f4`.
Building a plane still needs three samples that agree in rise / run (short bases scatter in slope and
cannot fake one; a height-only triple test let three closing steps agree on a slope of 0.23, caught
by the unit test). The roughness step is the plane residual while a plane stands and the raw step
otherwise (`planeFresh_` and the `min(plane, level)` residual are gone). The incline cap keeps the
last slope for its hold after a contradiction (the references read `slopeConsistent()`), as it did
before. Unit test: the walk-off case asserts the drop at the first flat stride; the "plane stands /
falls" block adds a 1.2 cm short-base contradiction, the rebuild, and the crest. Contract test
renamed to `test_plane_falls_only_to_a_contradiction_in_height`; the key is in the schema, the
builder, the G1 block and every tracked tuning config.

Rebuild (no `SKIP_BUILD`) and re-run: `slope_5` both ways (the crest, baseline humanT25cS5),
`slope_10` all four (`speeds backward slope_about_face slope_back_slow`, baseline humanT25c3S10 -
the stops must still stand: this is the check that the closing steps' residuals are under 1 cm on
10 deg), and the T25a control on `slope_about_face`. `terrain_strides` on the slope_5 runs shows the
crest directly: `tdref` should go to ~0 within one landing of the plateau instead of +2 cm for three.
