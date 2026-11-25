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

_Diagnostics pending: `wbc_diag.py` output for `wb_s1_soft`, `wb_s1_pd50`, `wbs_s1_soft`, `wb_s1`, `wbs_s1`._
