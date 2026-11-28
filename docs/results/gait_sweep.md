# Gait sweep: double support against the contact-switch torque step

Status: **run 2026-11-25, 0/4 walk + 0/2 stops — double support rejected; the plan does not use it to spread the load.** Script `tests/tuning/run_gait_sweep.sh`.

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

Set I, 0.5 m/s (the compare_ab scoring step crashed on a `--show` of a top-level key — fixed — so
the table is from the per-case SUMMARY/DIAG lines; no candidate is within any gate):

| case | gait run | vx | acc_hf | z_sd mm | roll° | leg meas_tau_hf | landing early ms | contact changes /s | plan τ step / change (max) | cmd jump@switch |
|---|---|---|---|---|---|---|---|---|---|---|
| gw_base | trot | 0.458 | **0.48** | **2.2** | 0.27 | **4.19** | 9 | 2.0 | 18.1 (74) | 0.092 |
| gw_walk | walk 0.3+0.1 | 0.441 | 1.73 | 5.3 | 0.34 | 5.80 | 18 | 5.0 | 12.2 (65) | 0.150 |
| gw_walk41 | walk 0.4+0.1 | 0.437 | 1.71 | 7.8 | 0.48 | 5.49 | 12 | 4.0 | 12.5 (71) | 0.117 |
| gw_walk3515 | walk 0.35+0.15 | 0.424 | 1.28 | 8.4 | 0.52 | 6.13 | 16 | 4.0 | 13.9 (75) | 0.126 |

Set J, stops: `gws_walk` / `gws_walk41` fail every primary (pitch transient 0.84 → 1.21 / 1.23°,
roll 0.52 → 0.77 / 1.09°, acc_hf 1.04 → 1.44 / 1.28) and the landing-speed guard (0.26 → 0.48 /
0.32 m/s).

**Reading.**

1. **The double support is not used to transfer the load.** The per-event step in `plan_tau`
   drops only from 18 to 12–14 Nm leg-mean — and there are twice as many events (touchdown into
   STANCE and lift-off out of it are now separate), so the total stepping per second goes *up*
   (36 → 50–61 Nm/s). The largest-jump trace of `gw_walk41` shows it directly: left hip pitch
   −23 → +20 Nm in one tick at touchdown into double support (mode 1 → 3), with 100 ms of double
   support ahead in which the weight could have moved gradually. The plan has no reason to: nothing
   in the cost charges for a force or torque *rate*, so an instantaneous transfer is free and the
   solver takes it. The contact-switch step is a property of the formulation (instantaneous contact
   activation + no rate penalty), not of the gait.
2. **The walk template is much rougher than the trot at this speed**, independent of the step:
   acc_hf ×2.7–3.6, pelvis z sd 2.2 → 5–8 mm, roll ×1.3–1.9, leg torque HF +31–46 %, speed −4–7 %.
   The trot's 0.5 s supports at 0.5 m/s are a near-passive pendulum (2.D's finding); shortening
   the swing to the same step length raises the swing-foot speed (landing 0.26 → 0.32–0.48 m/s in
   the stops set) and the STANCE phases add a vertical bounce the trot does not have.
3. **The cmd_tau_jump definition fix holds**: `gw_base` 0.092 with 2 policy swaps on a
   contact-change tick excluded; the anatomy's "excluding" mean 0.092, max 0.82 Nm — no 20 Nm
   events left in the metric.

**Decision: trot stays at 0.5 m/s; gait bands unchanged.** The only remaining lever on the
contact-switch torque step is the one the roadmap has as 2.E — a rate penalty on the contact forces,
i.e. forces as states and their rates as inputs — and this sweep is the evidence that nothing
cheaper reaches it: timing (2.C), gait (here), load-share cost in DS (1.3) and fades (2.B) have all
been tried.
