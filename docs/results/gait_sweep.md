# Gait sweep: double support against the contact-switch torque step

Status: **planned, not yet run** (2026-11-25). Script `tests/tuning/run_gait_sweep.sh`.

## Why

After 2.B and 2.C the walk's remaining transient is the contact event itself. The trace in
`touchdown_timing_sweep.md` ("Resolved") shows the plan's hip-pitch torque stepping −27 → +33 Nm in
one tick as a leg goes swing → stance, at every event; the 2.B fade hands that tick to the plan by
design, so the hardware sees it. At 0.5 m/s the ladder runs `trot` (`[LF, RF]`, 0.5 s supports,
**no double support**), which makes the weight transfer instantaneous by construction. The `walk`
template has a `STANCE` phase between supports — a window the plan can use to move the weight across.
Whether it does, and what it costs elsewhere (a shorter swing at the same step length is a faster
foot), is what the sweep measures. Config only.

## Cases (set I, 0.5 m/s; set J, stops)

| case | change |
|---|---|
| `gw_base` | shipped: trot at 0.5 |
| `gw_walk` | walk/trot boundary 0.45 → 0.55 m/s: stock `walk` (0.3 s swing + 0.1 s DS, 0.8 s cycle, swingHeight 0.08; swing scaled 0.75× by `swingTimeScale`) |
| `gw_walk41` | boundary + `walk` at the trot's cadence `[0, 0.4, 0.5, 0.9, 1.0]`, swingHeight 0.09 — the last 0.1 s of each support becomes double support, period and clearance unchanged (cleanest A/B) |
| `gw_walk3515` | as `walk41` with 0.15 s DS `[0, 0.35, 0.5, 0.85, 1.0]` |

Escalation to a faster gait needs the command ≥ the rung's top, so 0.5 stays in `walk` with the top
at 0.55; trot's lower bound follows to 0.50.

## New metric

`legs.plan_tau_step_at_contact_change` (and `cmd_tau_step_at_contact_change`, `_max`): leg-mean
|Δτ| on the ticks where the plan mode changes, in the walk window — the step itself. Gated at 0.7×
as a primary; `acc_hf` 0.92× and `meas_tau_hf` 0.90× are the other primaries.

## Gates

Set I primaries: `walk.pelvis_acc_hf` ≤ 0.92×, `legs.meas_tau_hf` ≤ 0.90×,
`legs.plan_tau_step_at_contact_change` ≤ 0.7×. Guards: standard set + `cmd_tau_jump_at_switch`
≤ 1.15×+0.02, `tau_sat`, `touchdown.landing_speed` ≤ 1.10×, `vx_mean` ≥ 0.9× (a gait that cannot
hold the speed loses), foot z-track. Set J adds transient pitch/roll 0.85× as primaries and the
stops guards of sweep 4.

## Results

_(to be filled from `tuning_results/gait_sweep_report.txt`)_
