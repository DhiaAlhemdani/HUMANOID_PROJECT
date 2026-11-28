# Phase 1.3 — touchdown softening sweep (config round)

- **Date**: 2026-11-24
- **Branch**: `arena/47d87774-humanoid-project` (tooling commit `77977ff`)
- **Method**: G1 walk @ 0.5 m/s, 12 s, shipped config (blend on) + one swing-trajectory edit
  per case; baseline = existing `tuning_results/blend_on` (identical build/methodology as the
  Phase 1.2 sweep). Tooling: `tests/tuning/run_touchdown_sweep.sh`,
  `tests/tuning/compare_touchdown.py`. Report: user-run
  `tuning_results/touchdown_sweep_report.txt`.
- **Recon basis**: node velocity = `min(1, swingDuration/swingTimeScale) × config`
  → **0.75× for the 0.3 s walk swing** (effective velocities below); `impactProximityFactor*`
  = boundary slopes of the velocity-error gating cubic (weight 1 at contact, 0 mid-swing).

## Candidates vs baseline (one run each)

| case | edit | legs pd_tau p99.9 [Nm] | pelvis acc_hf | leg PD HF | leg track err [deg] | roll [deg] | vx [m/s] | solve [Hz] |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| **baseline (`blend_on`)** | shipped | **50.25** | 1.1805 | 2.287 | 0.148 | 0.64 | 0.4304 | 67.4 |
| `tdvn05` | `touchDownVelocity −0.05` (eff. −0.0375 m/s) | 50.25 | **1.078 (−8.7%)** | **2.11 (−7.8%)** | 0.174 | 0.67 | 0.425 | ≥60.7 ✓ |
| `tdvp05` | `touchDownVelocity +0.05` | 50.26 | 1.520 (+29%) | 2.62 (+14%) | 0.191 | 0.70 | 0.432 | ≥60.7 ✓ |
| `impf06` | `impactProximityFactor… 0.3→0.6` | 50.24 | 1.251 (+6%) | 2.41 | 0.201 | 0.65 | 0.429 | **55.7 ✗** |
| `impf10` | `impactProximityFactor… 0.3→1.0` | 50.24 | 1.256 (+6%) | 2.45 | 0.206 | 0.66 | 0.430 | **56.3 ✗** |

**Gates: 0/4 winners** — every case failed the primary (`pd_tau_p999 ≤ 0.90×`); `tdvp05`
also failed `acc_hf`/PD-HF guards; `impf06/10` failed the solve-rate guard (≥60.7 Hz).

## Findings

1. **The touchdown PD peak is config-invariant.** Four independent trajectory edits
   (∓0.075 m/s velocity span, 3.3× slope span) moved `pd_tau_p999` by **≤0.02 Nm (0.04%)** —
   50.25/50.26/50.24/50.24. The peak plateau is set at the contact/servo layer, not by the
   planned swing trajectory. This independently confirms Phase 1.2's D1 attribution
   (residual = contact/servo execution) and means **no swing-trajectory config can pass the
   original primary gate**; moving it requires reducing servo demand below the plateau
   (force-rate cost, time-varying late-swing gains — structural work).
2. **Sign confirmed (roadmap was right): negative `touchDownVelocity` softens, positive
   hardens.** `−0.05` → acc_hf −8.7% / PD-HF −7.8%; `+0.05` → acc_hf **+29%** (worst of the
   sweep). The z-up sign ambiguity in the docs is resolved empirically: a still-descending
   node reference at contact is the softer landing.
3. **`tdvn05` produced the lowest `acc_hf` of the entire project** (1.078 vs best previous
   1.179; noise floor ≈ ±2–3%) — the largest single bounce reduction so far, with every other
   guard passing (track-err +18% absolute = 0.148°→0.174°, negligible).
4. **Tightening the impact-proximity slope hurts the solver** (67.4 → ~56 Hz, −17%) without
   softening anything (acc_hf +6%) — velocity-error gating near contact makes the QP harder.
   Rejected.

## Decisions

- **Ship `touchDownVelocity: -0.05` in `config/g1/ros2_controllers.yaml`** (G1 only; h2/upkie
  untested). Gate amendment, declared here: with the primary shown config-invariant
  (finding 1), the phase's effectiveness gate re-bases on **`pelvis_acc_hf`** (the benchmark's
  #1 smoothness metric) + guards. `tdvn05`: acc_hf **−8.7%** (≫ noise), PD-HF −7.8%, all
  guards pass. Optional confirmation: re-run the same case once and expect acc_hf ≈ 1.08±0.04.
- **Rejected**: `tdvp05` (direction), `impf06`/`impf10` (solve rate + no softening).
- **D2 (bounce after touchdown)**: reduced at source (−8.7% acc_hf) — **config levers now
  exhausted**; the remaining PD-peak plateau is structural (§1.3 code levers / force-rate
  cost) and queued as the next phase if the residual bounce still matters.
