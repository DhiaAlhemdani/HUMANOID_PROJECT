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
