# Phase 1.4 — foot / contact-model sweep (sole box, fitted CoP rectangle)

- **Date**: 2026-11-24
- **Branch**: `arena/47d87774-humanoid-project` (tooling `a7b17bd`)
- **Method**: G1 walk @ 0.5 m/s, 12 s, shipped config/model baseline (`fc_base`, via `make_config`
  → trackingRate 500) vs four variants built by `tests/tuning/make_model.py` (sole) and
  `make_config.py` (CoP rectangle); `run_footcontact_sweep.sh` → `compare_footcontact.py`.
  Same build throughout (no C++ change).
- **Gates**: any-of primaries `acc_hf ≤ 0.92×` / `roll ≤ 0.85×` / `pd_tau_p999 ≤ 0.90×`,
  all guards (survived, non-improved primaries not worse, jump, PD HF, track err, z, arms, vx, solve).

## Recon that reframed the item (before the runs)

| roadmap assumption | measured in the files |
| --- | --- |
| sole = "four spheres spanning 6 cm" | four **r = 5 mm** spheres per foot, hull **x −0.05…0.12 m, y ±0.025 (heel) / ±0.03 (toe)** |
| a box matching the rectangle "gives CoP margin" | the MPC rectangle (x −0.055…0.125, y ±0.03) is already **5 mm *larger*** than the physical footprint on every edge |
| contact is stiff (`solref 0.001`) | **inert** — floor geom has `priority="1"`, no solref → MuJoCo default (0.02 s) governs |

## Results — 0/4, all guards pass, no primary improves

| run | sole | rectangle | acc_hf | roll [°] | PD peak p99.9 [Nm] | jump@switch | z_sd [mm] | vx | verdict |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| **fc_base** | 4 spheres | shipped | **1.055** | **0.646** | **50.24** | 0.128 | 1.99 | 0.425 | — |
| fc_rectfit | 4 spheres | fitted (−5 mm) | 1.060 | 0.63 | 49.97 | 0.123 | 2.0 | 0.426 | no gain |
| fc_box | box = sphere hull | shipped | 1.101 | 0.65 | 50.24 | 0.116 | 2.0 | 0.425 | no gain |
| fc_box_rectfit | box = sphere hull | fitted | 1.044 | 0.64 | 50.25 | 0.145 | 2.0 | 0.425 | no gain |
| fc_boxrect | box = shipped rectangle | shipped | 1.096 | 0.64 | **48.99 (−2.5 %)** | 0.144 | 2.2 | 0.426 | no gain |

All runs survived; solve 63–66 Hz.

## Findings

1. **Contact *shape* is not what drives bounce or roll.** acc_hf moved −1…+4 % (inside the
   same-config run spread seen in the load-share round), roll ±0.02°. Box vs. spheres with the
   same footprint (`fc_box`) is indistinguishable from baseline on every metric. The fitted
   rectangle (`fc_rectfit`) is also neutral — the 5 mm CoP over-reach is not costing anything
   measurable at 0.5 m/s on flat ground.
2. **The 50.24 Nm "touchdown peak" is actuator saturation, not a plan quantity.**
   `pd_tau_p999` = p99.9 of |meas_tau − cmd_tau|; the measured torque is MuJoCo's actuator
   force, clamped at `actuatorfrcrange ±50` (= URDF `effort="50"`) on all four ankles, while
   the swing ankle's feed-forward is ≈0.25 Nm of opposite sign at contact — hence
   50.25 ± 0.01 in **every run of the project** (trajectory configs, touchdown velocity,
   load-share cost, contact model). With servo `kp = 200` a 14° ankle error saturates the motor;
   touchdown errors of that size are routine. This is why three lever rounds "could not move the
   peak": a clipped signal cannot move until saturation falls below 0.1 % of walk ticks — which
   is exactly what `fc_boxrect` (48.99) and marginally `fc_rectfit` (49.97) show.
3. **Consequence for the metric set**: `pd_tau_p999` is retired as a D2 effectiveness metric.
   `plot_tracking.py` now reports **`legs.tau_sat_pct`** (worst joint, % of walk ticks at ≥98 % of
   its URDF effort limit), `tau_sat_pct_all`, `tau_sat_worst`, `tau_sat_limit`. It can be
   back-filled onto any existing run from its `data.npz`
   (`python3 tests/tuning/plot_tracking.py tuning_results/<tag>`).

## Decisions

- **Nothing shipped**: shipped MJCF and `contactGeometry.rectangle` unchanged. `fc_boxrect`'s
  −2.5 % peak does not clear 0.90× and comes with acc_hf +4 % / z_sd +10 % — not a win.
- **§1.4 closed** (negative, documented). The box-sole tooling (`make_model.py`, `MODEL=` hook,
  `--robot-model`) stays for terrain/robustness work where edge contact matters more than on
  flat ground.
- **D2 re-targeted**: the real D2 quantity is **ankle saturation duty at touchdown**, a
  servo-layer effect (`kp 200` × touchdown error vs a 50 Nm motor). Levers live in
  `control.jointEffortPd` (ankle kp/kd) and the impact-time tracking error — to be measured
  with `tau_sat_pct` before any further sweep.
