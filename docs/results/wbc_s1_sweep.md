# Phase 2.B S1 — whole-body feedback layer, first closed-loop sweep (G1)

Design: `docs/design/wbc_layer.md`. Tooling: `tests/tuning/run_wbc_sweep.sh`, `tests/tuning/wbc_diag.py`.
Gates were fixed before the run (script header) and are not moved here.

## Sweep 1 (2026-11-24) — shipped PD vs `control.wholeBody` S1

Candidates: `wb_s1` (original defaults: base kp z 100 / pitch-roll 200, swing kp 200), `wb_s1_soft`
(×0.5), `wb_s1_stiff` (×2), `wb_s1_pd50` (original task gains, residual PD kp 50 / kd 5).
Residual PD otherwise kp 20 / kd 3; joint posture kp 100 / kd 20; weights base 1/1/10/10/10/10,
joint 1, swing 5, wrench 0.01.

### Set A — steady walk (`vx` 0.5)

Primaries `walk.pelvis_acc_hf ≤0.92×`, `legs.meas_tau_hf ≤0.85×`, `legs.tau_sat_pct ≤0.5×+0.02`,
`base.roll_track_err_deg ≤0.85×`; guards: default set minus `pd_tau_hf`, joint tracking relaxed to 1.5×+0.1°.

| case | survived | acc_hf | meas_tau_hf (Nm) | tau_sat % | roll track (°) | pitch track (°) | q track (°) | pd_tau_hf | z_sd (mm) | vx | verdict |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `wb_base` (PD kp 200) | ✓ | **1.072** | **3.853** | 0.118 | 0.097 | 0.081 | 0.166 | 2.062 | 1.54 | 0.426 | baseline (solve 66.6 Hz) |
| `wb_s1` | **fell 11.7 s** | — | 5.913 | 0.035 | 0.522 | 1.351 | 0.790 | 0.955 | — | — | fail |
| `wb_s1_soft` | ✓ | **0.695 (−35 %)** | 4.466 (+16 %) | **0.012 (−90 %)** | **0.086 (−11 %)** | 0.235 | 0.230 | 0.545 | 1.89 | 0.459 | guards-fail: `meas_tau_hf` primary worse than 1.1× |
| `wb_s1_stiff` | **fell 1.5 s** | — | 4.681 | 0.745 | 2.710 | 2.340 | 2.270 | 2.323 | — | — | fail |
| `wb_s1_pd50` | ✓ | **0.694 (−35 %)** | 4.437 (+15 %) | **0.000 (−100 %)** | **0.080 (−18 %)** | 0.167 | 0.188 | 0.705 | 2.04 | 0.457 | guards-fail: `meas_tau_hf` primary worse than 1.1× |

Every other guard passed for the two survivors (roll rms, jump@switch, joint tracking 0.19–0.23° vs
limit 0.35°, plan z sd, z tracking, arm swing, vx +7 %, solve rate).

### Set B — start/stop (`stops`)

| case | survived | transient roll max (°) | transient pitch max (°) | acc_hf | tau_sat % | transient z p-p (mm) | meas_tau_hf | vx | verdict |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `wbs_base` | ✓ | 1.42 | 1.36 | 1.279 | 0.153 | 25.7 | 4.086 | 0.244 | baseline (65.2 Hz) |
| `wbs_s1` | **fell 3.3 s** | — | — | — | 0.000 | — | 3.294 | — | fail |
| `wbs_s1_soft` | ✓ | **1.134 (−20 %)** | **4.753 (+250 %)** | **1.010 (−21 %)** | 0.024 | **54.6 (+112 %)** | 4.143 (+1 %) | 0.256 | guards-fail: pitch primary worse |

### Result: 0/4 + 0/2 by the pre-declared gates — and the first falls of the project (3)

What the numbers establish:

1. **The layer does what it was built for in steady walking.** With the soft task gains the pelvis
   4–40 Hz vibration drops **35 %** (1.07 → 0.69 — by far the largest smoothness movement of the
   project; Phase 1's best single lever was −8 %), ankle saturation effectively disappears
   (0.12 % → 0.00–0.01 % of ticks), base roll tracking improves 11–18 %, and speed tracking improves
   (0.426 → 0.458 m/s). The residual PD at kp 20 carries only 0.55–0.7 Nm of HF torque versus 2.06
   under kp 200.
2. **The original default gains were unstable** — `wb_s1` walked 11.7 s before falling, `wb_s1_stiff`
   fell at once, `wbs_s1` fell on the first stop. Shipped defaults in `parameters.yaml` and the G1
   config are now the soft set (block still `enabled: false`). The stiffness ladder is monotone:
   stiff → immediate fall, original → delayed fall, soft → stable; the instability is gain-driven,
   not a wiring bug (the survivors ran the same code).
3. **Two things block shipping, and they are different problems:**
   - *Steady:* total applied-torque HF rose 15–16 % (3.85 → 4.44–4.47 Nm) even though the body got
     35 % smoother. The feedback now runs at 1 kHz on measured velocities and the base twist; its
     HF content lands in `cmd_tau`. Whether it is noise amplification (fixable by filtering the
     feedback terms / lower Kd) or genuinely useful contact-consistent correction is what
     `wbc_diag.py`'s per-joint split answers.
   - *Stops:* the soft set survived but the stop/start transient became a 4.75° pitch excursion with
     a 55 mm height bounce (baseline 1.36° / 26 mm). The layer tracks the *plan's* base pose with a
     z/pitch PD; at a command step the plan and the body disagree the most, and the base task may be
     amplifying the re-plan rather than damping the body. The event table in `wbc_diag.py`
     (plan pitch vs observed pitch vs their difference) separates "plan pitched" from "body left the plan".
4. `z_sd_mm` in steady walk rose 1.54 → 1.9–2.0 mm (+0.4 mm) — not a guard in this sweep, small in
   absolute terms, noted for the next gate set.

### Decisions

- **Not shipped.** `control.wholeBody.enabled` stays `false`; defaults lowered to the soft set.
- The ladder suggests the stable region is at or below the soft set; the next iteration starts from
  it and changes one thing per candidate, chosen from the diagnostics (not guessed): base-task
  feedback filtering / Kd, base-task weight during command transients, swing task gain.
- Gates for sweep 2 (declared now): same primaries; `legs.meas_tau_hf` stays a primary — a
  smoothness layer that makes the actuators work harder at 4–40 Hz is not finished even if the
  pelvis is quieter; `walk.z_sd_mm ≤ 1.15×+0.3` added as a guard; set B must not exceed the
  baseline's transient pitch by more than 1.1×.

### Diagnostics (`wbc_diag.py`, same case data)

Layer health in every case: active on 100 % of policy ticks, KKT residual ≤ 1e-7, wrench clips on
1–4.5 % of ticks (mostly 1–2 components), zero invalid solves, gains fully residual ~0.5 s after
Locomotion. The problems below are formulation, not implementation.

**Where the +15 % torque HF is (steady, soft set vs baseline, 4–40 Hz Nm, |peak|):**

| joint | baseline total (cmd / pd) | peak | `wb_s1_soft` total (cmd / pd) | peak |
| --- | --- | --- | --- | --- |
| hip pitch | 8.43 (9.20 / 1.80) | 33 | **10.65 (10.5 / 0.8)** | **86** |
| hip roll | 4.34 (5.38 / 2.31) | 37 | 5.08 (5.3 / 0.4) | 40 |
| ankle pitch | 4.08 (1.85 / 3.70) | **50 (limit)** | **3.68 (3.7 / 0.9)** | **26** |
| knee | 2.85 (2.42 / 2.15) | 33 | 3.30 (3.2 / 0.6) | 33 |
| hip yaw | 2.12 (2.64 / 1.35) | 21 | 2.81 (2.9 / 0.3) | 35 |

The ankles got what the layer promised (HF down, peak 50 → 26 Nm, no saturation). The increase is
at the **hips, in the layer's own command**, with a hip-pitch peak 2.6× the baseline's.

**Pitch is the weak axis:** base pitch obs−plan rms 0.081° → 0.235° (soft) / 0.167° (pd50), max
0.53° → 1.7–2.1°; the *plan's* pitch excursion doubles (p-p 1.0° → 2.7°) because the MPC re-plans
around a pelvis that is no longer rigidly held. Roll is unaffected (0.097° → 0.086°). z tracking
improves (0.47 → 0.29 mm rms).

**The falls — last second of `wb_s1` (original gains):** plan and body pitch *together* −5° → −40°,
with `waist_pitch` error 18.5°, hip pitch 15.7°, the waist actuator pinned at its 50 Nm limit and
hip roll at 93 Nm. `wbs_s1` is the same picture (waist 23.8°, hips 17–18°). The stops event table
for the survivor `wbs_s1_soft` shows the step-down transient is a 5.08° *plan* pitch with a 4.15°
plan-vs-body disagreement — the body and the plan fight, the plan follows.

**Root cause (hypothesis for sweep 2):** the base task acts on the **pelvis**, but the mass is the
torso above a 3-DoF waist, and the waist posture rows carry weight 1 against the base task's 10.
When the stance leg cannot deliver the demanded pelvis pitch acceleration, the least squares
realises it by rotating the pelvis against the torso — folding at the waist. The torso tilts the
other way, the CoM moves, the MPC re-plans the pitch, and the loop drifts; stiffer base gains make
it drift faster (the ladder). The kp 200 joint PD never had this failure mode because it held the
waist rigid, so pelvis and torso moved as one body. The same mechanism explains the hip-pitch peaks
(pelvis pitch corrections react through the hips) and the pitch-only degradation.

## Sweep 2 — declared 2026-11-24 (`tests/tuning/run_wbc_sweep2.sh`)

One change per candidate from the soft (now default) set:

| case | change | tests |
| --- | --- | --- |
| `wb2_soft` | none | noise floor vs sweep 1's `wb_s1_soft` |
| `wb2_waist` | waist posture rows weight 50 | "hold the waist and the pelvis task is fine" |
| `wb2_torso` | `torso_link` orientation task (kp 100/100/25, kd 20/20/10, w 10) **replaces** pelvis pitch/roll (base kp/kd pitch, roll = 0) | "the task was on the wrong body" — matches the MPC's own `torsoTiltCost` link |
| `wb2_lowbase` | pelvis pitch/roll kp 25 / kd 10 | ladder direction without changing structure |

Gates (set A): primaries as sweep 1 (`acc_hf ≤0.92×`, `meas_tau_hf ≤0.85×`, `tau_sat_pct ≤0.5×+0.02`,
`roll_track ≤0.85×`); guards as sweep 1 **plus** `walk.pitch_rms_deg ≤1.15×+0.05`,
`base.pitch_track_err_deg ≤1.2×+0.05`, `walk.z_sd_mm ≤1.15×+0.3`. Set B (`wb2_waist`, `wb2_torso`):
transient pitch is a primary (≤0.85×, not-worse 1.1×) and `transient_z_pp_mm_max ≤1.15×+2`.
Note `meas_tau_hf ≤0.85×` is demanding — the baseline's ankles are already quiet below the hips —
so a candidate can be a non-winner on that primary while being correct on everything else; the
not-worse limit (1.1×) is the real bar and the result will be read that way, in writing, before
any gate is touched.

Implementation added for it: per-joint posture weights (`control.wholeBody.jointWeights`) and a
link orientation task (`orientationLink/Kp/Kd/Weight`, world-frame `log3(R_plan Rᵀ)` error on the
LOCAL_WORLD_ALIGNED angular rows); both inert by default (uniform weights, `""`). Core test covers the
generic task under stance constraints.

**First sweep-2 run (invalid, harness):** 6 of 8 cases — both baselines included — never reached
locomotion. Cause: the shipped G1 block carried `jointWeights: []`; ROS 2's YAML parameter parser
cannot type an empty list, so the `double_array` declaration threw at load, `controller_sequence.py`
exited 1 and the probe saw no ground truth. Only the two cases that overrode the key with a real list
(`wb2_waist`, `wbs2_waist`) ran. Fixed by shipping the uniform default as an explicit 23×1.0 list,
plus a contract test that rejects empty lists in any `config/*/ros2_controllers.yaml`; `run_case.sh`
now keeps the "no locomotion" verdict and the launch's error lines in `summary.txt`. Sweep 2 is re-run
in full (the two surviving cases are kept for consistency checks only).

### Sweep 2 result (2026-11-25, valid run): 0/4 + 0/2 — and the real cause of the hip-pitch peaks

Set A (`vx` 0.5, baseline `wb2_base`; gates as declared above):

| case | acc_hf (≤0.92×) | meas_tau_hf (≤0.85×, nw 1.1×) | tau_sat % | roll track (≤0.85×) | pitch track (≤1.2×+0.05) | pitch rms (≤1.15×+0.05) | q track | z_sd | jump@switch (≤1.15×+0.02) | vx | verdict |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `wb2_base` | 1.087 | 3.856 | 0.129 | 0.101 | 0.087 | 0.243 | 0.173 | 1.54 | 0.115 | 0.425 | — |
| `wb2_soft` | 0.629 | 4.448 ✗ | 0.012 | 0.076 ✓ | 0.203 ✗ | 0.462 ✗ | 0.215 | 1.98 | 0.180 ✗ | 0.459 | guards-fail |
| `wb2_waist` | 0.703 | 4.483 ✗ | 0.012 | 0.085 ✓ | 0.237 ✗ | 0.501 ✗ | 0.228 | 1.87 | 0.174 ✗ | 0.458 | guards-fail |
| `wb2_torso` | 0.940 | 4.815 ✗ | 0.000 | 0.135 ✗ | 0.191 ✗ | 0.464 ✗ | 0.256 | 1.61 | 0.249 ✗ | 0.457 | guards-fail |
| `wb2_lowbase` | 0.828 | 4.637 ✗ | 0.035 | 0.117 ✗ | 0.182 ✗ | 0.480 ✗ | 0.238 | 1.71 | 0.166 ✗ | 0.458 | guards-fail |

Set B (`stops`, baseline `wbs2_base`):

| case | transient roll | transient pitch (nw 1.1×) | acc_hf | tau_sat % | z p-p max (≤1.15×+2) | meas_tau_hf | verdict |
| --- | --- | --- | --- | --- | --- | --- | --- |
| `wbs2_base` | 1.414 | 1.255 | 1.253 | 0.153 | 25.5 | 4.009 | — |
| `wbs2_waist` | 1.403 | 4.689 ✗ | 1.090 | 0.012 | 46.2 ✗ | 4.043 | guards-fail |
| `wbs2_torso` | 1.696 ✗ | 1.710 ✗ | 1.162 | 0.024 | 29.1 | 4.930 | guards-fail |

Reading:

- `wb2_soft` reproduces sweep 1's `wb_s1_soft` to within noise (acc_hf 0.629 vs 0.695,
  meas_tau_hf 4.448 vs 4.466, pitch track 0.203 vs 0.235) — the sweep is repeatable.
- **Hip-pitch peaks are not the pelvis task.** `wb2_torso` zeroes the pelvis pitch/roll gains and
  `wb2_lowbase` quarters them; hip_pitch |max| stays at 86 Nm in all five WBC cases and legs HF
  *rises* (10.4 → 11.3 Nm). The waist weight changes nothing in steady walk (±1 %).
- **The waist "stop fix" seen in the invalid run was a one-off.** Same config, two runs: first
  step_down obs−plan pitch 0.41° / 43 mm, then 3.21° / 49 mm with the plan itself pitching 5°.
  That event depends on the gait phase at the moment the command drops; single runs cannot score it.
  The earlier provisional reading is withdrawn.
- Spike anatomy (`wbc_diag.py`, section 2b, added for this): in `wb2_soft` and `wb2_torso` every
  hip-pitch |cmd| > 60 Nm event (17 per run ≈ one per step, median 86–88 Nm, **2 ms wide**) is on
  the leg that **just touched down, 0 ms after the plan's contact switch**, and 54–58 Nm of it is
  the layer's correction over a 30 Nm plan torque. The baseline has no such event.

Diagnosis: the layer takes its contact set from the plan mode (`modeNumber2StanceLeg(policy_mode)`).
At the switch tick the lifting foot's wrench is removed and the landing foot's hard no-motion rows
switch on, while the measured robot is still in the previous contact state for a couple of
milliseconds; the KKT solve then routes the whole support transient through the landing hip as a
one-tick impulse. That impulse is what lifts `meas_tau_hf` (+15 %) and `cmd_tau_jump_at_switch`
(+55 %), and it is invariant to every gain the layer exposes — which is why sweeps 1 and 2 could not
move it. It also explains why `pitch_rms` doubles (0.24 → 0.46°): a 55 Nm hip impulse every step.

## Sweep 3 — declared 2026-11-25 (`tests/tuning/run_wbc_sweep3.sh`)

One lever, the Phase-1 `solveBlendSeconds` pattern applied to the contact switch: around every
planned switch the layer's torque is faded into the plan's torque,
`τ = τ_plan + α (τ_wbc − τ_plan)`, α = smoothstep(|t − t_switch| / `contactBlendSeconds`), with the
previous switch taken from the observed plan-mode change and the next from the policy's
`modeSchedule_.eventTimes` (so the fade starts *before* touchdown). New `wbc/contact_alpha`
tracking column. Ships `contactBlendSeconds: 0.0` (off); the servo residual PD is untouched.

| case | change |
| --- | --- |
| `wb3_soft` | none (control) |
| `wb3_fade30` | `contactBlendSeconds 0.03` |
| `wb3_fade60` | `contactBlendSeconds 0.06` |

Gates: **unchanged from sweep 2** (both sets). Expected if the diagnosis is right: hip_pitch |max|
back to ≈35 Nm, `meas_tau_hf` ≤ baseline, `cmd_tau_jump_at_switch` ≤ baseline, acc_hf still ≤0.7×;
pitch track may stay above its guard (0.2° vs 0.087°) — that would then be the next, separate
question (the plan pitch p-p itself grows 1.0 → 2.5° under the layer). If the fade does not remove
the 86 Nm peaks, the diagnosis is wrong and S1 closes as "not shippable without S2's contact
handling" with this record.
