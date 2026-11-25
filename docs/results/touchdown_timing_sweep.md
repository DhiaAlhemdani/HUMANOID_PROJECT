# Phase 2.C-timing — `swingTrajectory.touchDownHeightOffset` sweep 1 (G1, 2026-11-25)

Context: `docs/design/touchdown_impact.md` §7. The shipped swing planner aims the touchdown node
1 mm below the ground (`touchDownHeightOffset: -0.001`, upstream default); the robot was measured
landing 14–20 ms before the plan's contact event at every step. Tooling:
`tests/tuning/run_touchdown_sweep.sh` (gates in the header, fixed before the run; not moved).
Baselines are make_config'd from the shipped config (WBC on, fade 0.03).

## Set A — steady walk (`vx` 0.5, trot), baseline `td_base`

| case | offset | early_ms (gate: \|·\| ≤ 0.4× = 8.4) | base_vz_mismatch (≤0.80×) | acc_hf (≤0.92×) | meas_tau_hf (≤0.90×) | jump@switch | pitch track ° | vx | verdict |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `td_base` | −1.0 mm | +21 | 0.0106 | 0.619 | 4.096 | 0.132 | 0.056 | 0.455 | — |
| `td_offm05` | −0.5 mm | +15 | 0.010 | 0.524 (0.85×) | 4.107 | 0.114 | 0.063 | 0.456 | WINNER (acc_hf) |
| `td_off0` | 0 | +14 | 0.010 | 0.520 (0.84×) | 3.877 (0.95×) | 0.123 | 0.064 | 0.456 | WINNER (acc_hf) |
| `td_offp1` | +1.0 mm | +10 | 0.011 | **0.467 (0.75×)** | 3.909 | 0.125 | 0.069 | 0.457 | WINNER (acc_hf) |

All guards pass in all three (roll/pitch rms, jump@switch, q tracking, z, arm swing, foot z
tracking, solve rate). Per-joint: ankle-pitch HF 3.05 → 2.5 Nm at offset ≥ 0; no hip-pitch spikes
in any case; ramp events unchanged.

## Set B — start/stop (`stops`, walk gait), baseline `tds_base`

| case | offset | early_ms | base_vz_mismatch (primary, nw 1.1×) | transient pitch (≤0.85×) | transient roll | acc_hf | z p-p max (≤1.15×+2) | meas_tau_hf | verdict |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `tds_base` | −1.0 mm | +16 | 0.0113 | 1.021 | 0.952 | 0.934 | 32.3 | 4.031 | — |
| `tds_off0` | 0 | +14 | 0.0142 (1.26×) ✗ | 1.038 | 0.975 | 0.930 | 34.6 | 3.899 | fail (not-worse) |
| `tds_offp1` | +1.0 mm | +10 | 0.0178 (1.58×) ✗ | **0.836 (0.82×)** | 0.939 | 0.886 | 35.8 | 3.953 | fail (not-worse) |

Events (`touchdown_diag_B.txt`): first step_down obs pitch 1.02° → 1.04° (off0) / **0.64°** (offp1);
all other events 0.4–0.8° in every case; ankle torques 24–28 Nm throughout, no saturation.

## Result by the declared gates: **3/3 in set A, 0/2 in set B → nothing ships from this sweep**

Set B's failures are on one number, `touchdown.base_vz_mismatch`, which was declared a primary
with the not-worse rule in both sets. It is a new metric; its run-to-run noise was not
characterised before the sweep. Same-config repeats available so far: `wb3_fade30` 0.0121 vs
`td_base` 0.0106 (14 % apart, steady walk); in stops the WBC-off `wbs3_base` was 0.0243 against
0.0113 here (different config, not a repeat). 1.26× (`off0`) is within two of those spreads; 1.58×
(`offp1`) is not. The gates are not moved; the result stands.

## What the sweep established (more useful than the verdict)

1. **The −1 mm offset explains about a third of the early landing, not all of it.** early_ms
   21 → 15 → 14 → 10 ms for −1 / −0.5 / 0 / +1 mm: a slope of ~5.5 ms per mm, where a foot that
   tracked the plan's 0.05 m/s terminal approach would give 20 ms per mm. Even aimed 1 mm *above*
   the ground, the foot is down 10 ms before the plan says so. The remaining earliness is the foot
   arriving **faster** than the plan's terminal deceleration (position error is still +0.2 mm 30 ms
   before the stop, so it is a velocity lag in the last ~20 ms). That points at the swing-end
   tracking (WBC swing task, `swingKd`/`swingWeight`, or the quintic's terminal shape via
   `touchDownVelocity`) — not at the offset and not at an impact model. New back-fillable metrics
   `touchdown.approach_vz_obs/plan` (foot vz over the last 25→5 ms before the stop) measure it.
2. **The post-touchdown base-vz mismatch is not a timing effect.** In trot it is 0.010–0.011 for
   every offset while early_ms moves 21 → 10 ms. Whatever the plan and robot disagree about in the
   60 ms after `t_e`, it is not when the foot arrived. (In the walk gait the mismatch *grows* with a
   positive offset: the plan then starts its stance with the foot in the air and settles it down
   through the stance `Kp`, while the real base already stopped.)
3. **Aiming at or above the ground is a real steady-walk smoothness lever:** acc_hf 0.619 → 0.52
   (0 mm) → 0.467 (+1 mm), monotone, the latter well outside the ±7 % acc_hf run-to-run spread seen
   between same-config runs (`wb3_fade30` 0.580 vs `td_base` 0.619). Ankle-pitch HF −18 %.
4. In stops, +1 mm cuts the first-stop pitch transient 1.02 → 0.64° and the overall transient pitch
   0.82× — the one set-B primary that improved — at the cost of the vz mismatch and +3.5 mm z p-p.

## Decision

No config change ships. Two follow-ups, in this order:

- **Characterise before re-gating:** repeat `td_off0` and `tds_off0` once (same gates). If
  `tds_off0`'s vz mismatch sits at ≤1.1× on the repeat, `off0` is a clean 2-set winner on acc_hf
  and ships; if it repeats at ~1.26×, the walk gait genuinely prefers the −1 mm margin and the item
  closes as "offset 0 is better for trot, worse for walk — not shippable as a single value".
- **Measure the terminal-approach velocity lag** (`approach_vz_obs/plan`, back-fill on
  `td_base`/`td_off0`/`td_offp1`): if obs is ≥ 1.5× plan over the last 20 ms, the next 2.C sweep is
  the swing-end tracking (WBC `swingKd` / `swingWeight`), which attacks the remaining 10–14 ms and
  is independent of the offset; the jump map stays deferred until a run shows a sizeable
  post-touchdown mismatch *with* the landing on time.

## Follow-up 1: approach-velocity back-fill (2026-11-24)

`approach_vz_obs/plan` back-filled with `41f8073` on the sweep-1 recordings (median over events,
foot vz over [−25, −5] ms before the measured stop):

| case | early_ms | approach_vz_obs | approach_vz_plan | obs/plan | base_vz_mismatch |
|---|---|---|---|---|---|
| td_base (−1 mm, trot) | 21 | −0.120 | −0.204 | 0.59 | 0.0106 |
| td_off0 (0, trot) | 14 | −0.079 | −0.148 | 0.54 | 0.0099 |
| td_offp1 (+1 mm, trot) | 10 | −0.071 | −0.123 | 0.58 | 0.0115 |
| tds_base (−1 mm, walk) | 16 | −0.164 | −0.227 | 0.72 | 0.0113 |

**The velocity-lag hypothesis is refuted in the form it was stated.** The foot is *slower* than the
plan over the final approach (0.54–0.72×), not faster, for every offset and both gaits. The
swing-end-tracking sweep (raise `swingKd`/`swingWeight` so the foot follows the plan's terminal
deceleration) is therefore *not* the next step — the foot already decelerates more than planned.

But the numbers do not fit together with a foot that merely tracks the spline: 30 ms before the stop
the foot is +0.2 mm *above* the plan (`foot_z_err_descent_mm`), it then descends ~1.6 mm less than
the plan over the next 20 ms (0.08 m/s slower), yet it ends at the ground while the plan is still
~0.3 mm above its endpoint. Those three facts are only compatible if the foot's deceleration begins
*before* the detected stop — i.e. the real ground contact is earlier than the "stop" (vz ≥ −5 mm/s
after the descent minimum) and the `approach` window straddles a soft, already-in-contact
deceleration (`impact_vz_obs` of only −0.010 m/s at 2–4 ms before the stop says the same thing).
The MuJoCo model gives no pre-contact force (default `margin` 0, foot = four r = 5 mm spheres,
`solref 0.001 1.0`), so a deceleration before the geometric contact would have to come from the
controller; a deceleration *at* contact followed by 10–20 ms of creep to vz ≈ 0 would mean the
real earliness is larger than `early_ms` reports and the stop detector is late.

`tests/tuning/touchdown_trace.py <case>` decides this without another run: it prints the median
obs/plan height (above the foot's resting height) and vz at 5 ms steps over [−80, +20] ms around
the detected stop, plus `decel_onset` — the height at which the measured vz first leaves the plan's
by > 0.05 m/s. Onset at ~1–2 mm above rest = contact (geometry / detector lag); onset at 5+ mm =
the controller slows the foot in the air; onset only after the stop = the metric window itself is
wrong.

### Trace result: the plan, not the foot, reaches the ground early

`touchdown_trace.py` on `td_base` (34 touchdowns, medians, heights above the foot's mid-stance
rest height, times relative to the measured stop; `t_e` = stop + 21 ms):

| ms | obs z | plan z | obs vz | plan vz |
|---|---|---|---|---|
| −80 | 26.9 | 26.8 | −0.618 | −0.635 |
| −40 | 6.3 | 6.2 | −0.410 | −0.404 |
| −30 | 2.8 | 2.6 | −0.342 | −0.325 |
| −25 | 1.8 | 1.3 | −0.242 | −0.284 |
| −20 | 1.1 | **0.1** | −0.164 | −0.243 |
| −10 | 0.4 | −1.8 | −0.075 | −0.167 |
| 0 | 0.3 | **−2.2** | 0.000 | −0.087 |
| +20 (≈ t_e) | 0.9 | −0.2 | 0.033 | −0.019 |

`decel_onset` 22 ms before the stop at 1.5 mm above rest (p25/p75 1.1/1.7; 31/34 events).
`td_offp1`: onset 23 ms, 1.5 mm; `tds_base` (walk): onset 16 ms, 0.6 mm (0.3/1.4).

Reading, in order of certainty:

1. **Tracking is not the problem.** Measured and planned foot height agree to ≤ 0.2 mm and vz to
   ≤ 0.02 m/s from 27 mm all the way down to 2.6 mm (−30 ms). The WBC/servo layer does what the
   plan says; the swing-end gain sweep is off the table.
2. **The ground stops the foot, 1.5 mm above its rest height, while the plan keeps going.** From
   −25 ms the measured vz collapses (−0.34 → 0 over 20 ms: the contact compliance of four r = 5 mm
   spheres), while the plan passes 0 mm at −20 ms at −0.24 m/s and continues to **−2.2 mm below
   the foot's rest height** at the stop, 21 ms before its own contact event. The real landing speed
   is therefore ≈ 0.3 m/s (the plan's speed at 2–3 mm), not the planned 0.05 m/s and not the
   0.010 m/s the old `impact_vz_obs` reported (that was the creep). `approach_vz_obs/plan` was
   measuring the same creep: superseded.
3. **The plan's descent runs ahead of the reference spline it is supposed to follow.** The MPC
   foot plan (`plan_foot` = kinematics of the planned state) is still at −0.24 m/s when it crosses
   the ground 41 ms before `t_e`; a quintic ending at `t_e` with `touchDownVelocity` −0.05 is at
   +3 to +5 mm at that instant (fitting the observed velocities at 27 mm and 6 mm to the planner's
   `MinimumJerkSplineCpg` gives a half-swing of ~0.2 s). Position leads the spline by ~25 ms /
   2–5 mm while velocity leads by ~10 ms — a *sag*, not a time shift. After the stop the plan jumps
   back up (−2.2 → −0.2 mm with plan vz still negative): each re-solve restarts from the measured,
   now stationary, foot.

Mechanism (hypothesis, testable by config): the swing foot follows its spline through the hard
acceleration equality `a_z = a_ref − kd (v − v_ref) − kp (z − z_ref)`
(`constraints.footMotionEquality`, kp = kd = 50). Its error dynamics have poles at −1 and
−49 rad/s: velocity errors die in 20 ms, **height errors decay with a 1 s time constant** — never
within a 0.2 s descent. The MPC's accelerations are zero-order-held over 20 ms nodes; on a
decelerating spline the node-start acceleration under-decelerates, each step leaves the plan a
fraction of a millimetre low, and nothing pulls it back. The −1 mm offset explained a third of the
earliness because it is one more millimetre of the same sag. The −2.2 mm the plan reaches is
what the plan would dig into the ground if the ground were not there.

## Sweep 2 (planned): `footMotionEquality.positionErrorGain_z`

`tests/tuning/run_touchdown_sweep2.sh`: `fk_base` (50) / `fk200` / `fk400` / `fk625` (critical
damping with kd 50), set C = vx 0.5, set D = stops with 200 and 400. Primaries
`touchdown.abs_early_ms` ≤ 0.5×, **`touchdown.landing_speed`** ≤ 0.7× (new: measured descent
speed at 2 mm above the rest height, model-free; `impact_early_ms` is the 2 mm crossing vs `t_e`),
`walk.pelvis_acc_hf` ≤ 0.92×; guards sweep-1 set A + `meas_tau_hf`/`tau_sat` not-worse (the gain
also scales the stance height correction — shared parameter — covered by the stance z guards).
Prediction if the mechanism is right: early_ms 21 → ≤ 8, landing speed 0.3 → ≤ 0.15 m/s at kp 400,
and the plan trace no longer dives below 0 before the stop. If kp 400/625 move neither, the sag
is not feedback-limited and the next step is the C++ diagnostic (log `z_ref`/`vz_ref` from the
swing planner next to `plan_foot` — MPC-thread side, the planner's trajectories are rebuilt there)
before any further tuning.

## Follow-up 2: repeat runs

The first repeat attempt (`td_off0`, `tds_off0`) produced no data: the single-case command omitted
`WS=$HOME/Documents/GitHub/humanoid_ws`, so `run_case.sh` sourced the default container workspace,
the launch never started and both cases ended at "no locomotion after 120 s". The sweep-1 runs
were kept as `td_off0_r1` / `tds_off0_r1`. Repeat pending with `WS=` set.
