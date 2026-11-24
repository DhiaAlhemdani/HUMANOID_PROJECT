# Phase 1.2 — Bounce / Vertical-Accel Cost Sweep Results

Methodology: implement → test → document ([roadmap §1.2](../LOCOMOTION_IMPROVEMENTS.md)).
**Outcome: no winner, shipped config unchanged, D1 closed; the residual is handed to Phase 1.3.**


## Context: what the baseline showed (2026-11-24)

The blend A/B baseline (`tuning_results/blend_{off,on}/summary.txt`, G1, vx 0.5 m/s) resolved
the premise of D1:

| metric | benchmark (d_kn10) | now | reading |
| --- | --- | --- | --- |
| GT pelvis z sd (trend removed) | 11.9 mm | **1.5–1.6 mm** | the 11–12 mm planned bounce **does not reproduce** (main's quintic swing-foot spline is in the tree) |
| plan / obs z sd | ≈ GT | 2.0–2.1 / 1.8 mm | plan and execution agree; nothing meaningful at step frequency |
| pelvis 4–40 Hz vertical accel `acc_hf` | 1.19 | **1.16–1.18** | **the residual target of this sweep** |

So Phase 1.2 tested whether cost shaping can cut the residual `acc_hf ≈ 1.18`, with z-sd and
tracking metrics as guards (the old bounce metrics were already at the noise floor).


## Setup

- **Baseline:** `tuning_results/blend_on` (shipped config, tracking 500 Hz; blend does not
  change bounce) — `acc_hf 1.1805`, GT z sd 1.53 mm, plan_z 2.03 mm, roll 0.64°, vx 0.430,
  solve 67.4 Hz. Native ROS 2 Jazzy, G1, vx = 0.5 m/s, one run per cell (benchmark convention).
- **Cases** (`tests/tuning/run_bounce_sweep.sh`, edits via `make_config.py`):

| case | edit | rationale |
| --- | --- | --- |
| `accz10` / `accz100` | `r.baseAccelerations` z 0.005 → 0.05 / 0.5 | penalize the planned vertical-acceleration input — **never swept by the benchmark** |
| `z100` | `q+qFinal.basePose[2]` 50 → 100 | height cost ×2 (300 fell; 100 untested) |
| `vz10` | `q+qFinal.baseVelocity[2]` 5 → 10 | half of the benchmark's rejected 20/30 step |

- **Gates** (`tests/tuning/compare_bounce.py`): survived; **`acc_hf` ≤ 0.90× baseline
  (primary, ≥10% cut)**; vx ≥ 0.90×; GT/plan z sd and z-track ≤ 1.10×+0.5 mm;
  roll ≤ 1.15×+0.05°; arm swing ≤ 1.20×+1°; solve ≥ 0.90×.
- **Noise floor:** two near-identical baseline runs (blend off/on) gave `acc_hf` 1.16 / 1.1805 —
  ≈ ±2% run-to-run. Deltas below that are indistinguishable from noise.


## Results — 0/4 pass (only the primary gate failed; all guards passed everywhere)

| candidate | acc_hf | GT z sd [mm] | plan_z [mm] | roll [°] | swing [°] | vx | verdict |
| --- | --- | --- | --- | --- | --- | --- | --- |
| baseline (`blend_on`) | 1.1805 | 1.53 | 2.03 | 0.64 | — | 0.430 | — |
| `accz10` | 1.187 (+0.5%) | 1.5 | 2.06 | 0.67 | 8.5 | 0.429 | no |
| `accz100` | 1.179 (−0.1%) | 1.6 | 2.15 | 0.66 | 8.3 | 0.431 | no |
| `z100` | 1.234 (+4.5%) | 1.8 | 2.07 | 0.62 | 8.6 | 0.437 | no |
| `vz10` | 1.209 (+2.5%) | **1.30 (−15%)** | **1.86 (−8%)** | 0.67 | 8.5 | 0.427 | no |

Full report: `tuning_results/bounce_sweep_report.txt` (per-run dirs `tuning_results/bounce_*`).


## Findings

1. **`r.baseAccelerations` z is inert for this metric.** ×10 and ×100 moved `acc_hf` by
   +0.5% and −0.1% — both inside the ±2% noise floor. The planned vertical-acceleration
   input cost does not govern the residual 4–40 Hz content (note `rScaling: 0.001` keeps
   these weights small in absolute terms, but ×100 not moving anything says the lever's
   *direction* is wrong, not just its scale).
2. **Height cost ×2 made it slightly worse** (`acc_hf` +4.5%) with z metrics flat — rejected.
3. **`vz` 5→10 does what the benchmark's direction predicted, on the old metric:** GT z sd
   −15% (1.53 → 1.30 mm) and plan_z −8%. But z sd was already at the noise floor, and
   `acc_hf` went the wrong way (+2.5%). Not adopted — the gain is below the problem
   threshold and trades against the actual residual.
4. **Attribution:** `acc_hf` is insensitive to every planned-trajectory cost while the plan
   itself is smooth (plan_z 2.0 mm, z-track 0.5 mm). The residual 4–40 Hz vertical energy
   is therefore entering through **execution — contact impacts and servo/contact chatter —
   not through the plan.** That is exactly Phase 1.3's territory (D2 touchdown spikes,
   still measured: legs PD peak p99.9 = 50.25 Nm at the right ankle) and, secondarily,
   1.4 (sole box / CoP margin).


## Outcome

- **Shipped config unchanged** — no candidate passed; nothing adopted.
- **D1 is closed:** its headline (11–12 mm planned bounce) no longer reproduces, and the
  cost-shaping direction for the residual was tested and shown inert.
- **Carried forward:** `acc_hf ≈ 1.18` + PD peak 50.25 Nm → Phase 1.3 (soften touchdown),
  which attacks the injection point directly.

## Reproduce

```bash
WS=<colcon-workspace> tests/tuning/run_bounce_sweep.sh 0.5     # ~10-12 min, 4 runs
python3 tests/tuning/compare_bounce.py <baseline_dir> <candidate_dir>...
```
