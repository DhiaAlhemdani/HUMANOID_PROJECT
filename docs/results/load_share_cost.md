# Phase 1.3 structural round — double-support load-share cost (D2/D3)

- **Date**: 2026-11-24
- **Branch**: `arena/47d87774-humanoid-project` (implementation `8c704da`, fail-loud
  tooling `fdab4c2`, baseline fix `9a89431`)
- **Method**: G1 walk @ 0.5 m/s, 12 s, shipped config (incl. `touchDownVelocity −0.05`)
  + `ocs2.costs.contactLoadShareWeight ∈ {1e-6, 1e-5, 1e-4, 1e-3}`; baseline = fresh
  `make_config` output of the shipped config (trackingRate 500 — diagnostics only).
  Same build for baseline and candidates (the fix commits touched scripts only).
  Tooling: `tests/tuning/run_loadshare_sweep.sh`, `compare_loadshare.py`.
- **Primary gate**: `legs.cmd_tau_jump_at_switch ≤ 0.85×` (D3). Guards: D2 peak
  ≤1.05×, `acc_hf` ≤1.10×, PD-HF, track err, z metrics, roll, arms, vx, solve, survival.

## Results — 0/4, primary failed everywhere

| run | jump@switch [Nm] | PD peak p99.9 [Nm] | acc_hf | leg vel_hf | leg tau_hf | solve [Hz] | verdict |
| --- | --- | --- | --- | --- | --- | --- | --- |
| **baseline (shipped)** | **0.105** | 50.25 | 1.0801 | 0.240 | 3.84 | 67.0 | — |
| `w1e6` (1e-6) | 0.148 (**+41%**) | 50.24 | 1.103 | 0.294 | 4.07 | 66.0 | no |
| `w1e5` (1e-5) | 0.139 (**+32%**) | 50.24 | 1.118 | 0.243 | 3.86 | 64.2 | no |
| `w1e4` (1e-4) | 0.140 (**+33%**) | 50.25 | 1.092 | 0.282 | 4.07 | 63.8 | no |
| `w1e3` (1e-3) | 0.133 (**+27%**) | 50.24 | 1.087 | 0.285 | 4.08 | 63.0 | no |

All guards passed on every candidate; only the primary failed. All runs survived.

## Findings

1. **The cost hurts its own primary metric — consistently.** Jump@switch rose +27…+41%
   over baseline at every weight (direction consistent across a 1000× span → not noise).
   Likely mechanism: the φ(t) target is schedule-driven and the DS window slides through
   the horizon between solves, and `isActive` flips per-node as windows enter/leave the
   horizon — so consecutive solutions are pulled toward a *moving* split target exactly
   at mode switches, increasing solution-to-solution disagreement (the D3 definition).
2. **D2 plateau: third independent confirmation of invariance.** 50.24–50.25 across a
   1000× weight span (≤0.02 Nm), matching the ankle effort-limit scale (URDF effort,
   ~50 Nm in the torque-limit config example) — consistent with **actuator/contact
   saturation**, not a plan-chosen value. Three lever rounds now: swing-trajectory
   configs (∓0.075 m/s × 3.3× slope, ≤0.02 Nm), touchdown velocity shipped (acc_hf −8.7%,
   peak unmoved), structural load-share (this round, peak unmoved).
3. **No `acc_hf` improvement**: 1.087–1.118 vs baseline 1.080 (w1e5 +3.5%, still inside
   the 1.10× guard). The cost does not soften bounce.
4. **Correction to the first-pass read**: the apparent `w1e5` leg-HF win (vel_hf −17%)
   was run-to-run variance, not a weight effect — two same-config baseline runs span
   vel_hf 0.240–0.294 (−22%) and tau_hf 3.84–4.07 (−6%). Single-run deltas on those
   metrics are untrustworthy at this spread; no candidate shows a monotone trend.
5. **Cost is computationally benign**: solve 63.0–67.0 Hz (baseline 67.0), equality SSE
   unchanged — rejected for behavior, not for cost.

## Decisions

- **Not shipped**: `contactLoadShareWeight` stays **0.0** in `config/g1/ros2_controllers.yaml`;
  shipped config unchanged by this round.
- **D2 closed as plan-inert**: peak ≈ effort-limit saturation; config, trajectory, and
  plan-structural levers are exhausted (3/3). Bounce *energy* was improved earlier
  (`touchDownVelocity −0.05`, `acc_hf` 1.18 → 1.08). Residual D2 lives in the contact
  model (§1.4 sole box) and impact-time servo behavior — not the OCP.
- **D3 status**: largely addressed by Phase 1.1's solve-handover blend (jump 1.317 →
  0.105 on today's baseline); the load-share round actively worsened it → rejected.
