# Step A — wrench-rate weight sweep (2026-11-25)

Config: `human` (continuous gait law) + `ocs2.model.wrenchState: true`, `costs.r.contactWrenchRates`
uniform at the value below, `wrenchRateFirstNodeScale 1.0` (whole weight on every node including the
anchored first one). Scripts `speeds` + `omni`, baseline `human`. Full table:
`tuning_results/wrench_rate_sweep.txt` (user machine).

| candidate | steady jerk / acc_hf / tau_rate | transition max jerk / acc_hf / tau_rate | v_err | falls | solve Hz | verdict |
|---|---|---|---|---|---|---|
| human (baseline) | 42.95 / 0.802 / 216 | 73.6 / 1.23 / 398 | 0.225 | 0 | 45.5 | — |
| humanW0 (1e-5, layout check) | 45.0 / 0.806 / 222 | 68.9 / 1.20 / 337 | 0.175 | 0 | 44.7 | no gain (as intended) |
| **humanW_1e-4** | **36.7 / 0.574 / 168** | **58.2 / 1.00 / 294** | 0.193 | 0 | 44.1 | **WINNER** |
| humanW_1e-3 | 40.7 / 0.500 / 157 | 83.7 / 0.90 / 274 | 0.223 | 0 | 41.6 | fails: transition jerk; **standing sway after the speeds stop** (hold 0: jerk 82, acc_hf 0.97) |
| humanW_1e-2 | 58.5 / 0.513 / 193 | 317 / 5.9 / 302 | 0.784 | **1** | 43.0 | fall in `omni` at the 0.6 ramp; 0.2 hold jerk 110, 0.8 hold 179 |

Reading:

- The layout itself costs nothing (W0 ≈ human) and **2 % solve rate**; the solve-rate gate is not a concern.
- At 1e-4 every single segment of both scripts improves (steady acc_hf −29 %, tau_rate −22 %, jerk −15 %;
  transition maxima −21 % / −19 % / −26 %). User's eye: step abruptness reduced but **not eliminated**.
- Going up a decade keeps lowering `acc_hf`/`tau_rate` (the between-node force steps *are* being removed)
  but the robot gets **sluggish in force**: sway while standing after a stop at 1e-3, tracking loss and a fall
  at 1e-2. The user saw tracking loss at both 1e-3 and 1e-2.

Diagnosis: with `wrenchRateFirstNodeScale 1.0` the first node is both *anchored* (W_0 == Ws_0, Ws_0 = the
wrench the previous plan was applying — there is no force sensing) and *priced*, so every plan inherits
the previous plan's force and can only leave it at the priced rate. That is a memory across solves, i.e.
a low-pass on the force command, and its time constant grows with the weight: the balance loop loses
bandwidth before the ramps at contact changes get as slow as a human's (~0.1 s). The in-horizon pricing is
what shapes the loading/unloading ramps; the first-node pricing only adds lag.

Next: `wrenchRateFirstNodeScale` (commit after this doc) splits the two — R keeps `scale × w` at every
node (positive-definite floor), `WbIdWrenchRateCost` adds `(1 − scale) × w` from the second node on.
Sweep 1e-3 and 1e-2 with scale 0.05, expecting the acc_hf/tau_rate gains of the high weights without
the sway/falls. If the sway persists with a free first node, the cause is the in-horizon slowness itself
and the next lever is per-phase weighting (price the double-support handover, not mid-stance).

## Free first node (`wrenchRateFirstNodeScale 0.05`), 2026-11-25

`tuning_results/wrench_rate_firstnode.txt`:

| candidate | steady jerk / acc_hf / tau_rate | transition max | v_err | falls | solve Hz | verdict |
|---|---|---|---|---|---|---|
| humanW_1e-4 (scale 1) | 36.7 / 0.574 / 168 | 58.2 / 1.00 / 294 | 0.193 | 0 | 44.1 | WINNER |
| **humanF_1e-3** (scale 0.05) | **32.7 / 0.392 / 146** | **54.7 / 0.655 / 250** | 0.232 | 0 | 42.7 | WINNER on all four primaries |
| humanF_1e-2 (scale 0.05) | 118.7 / 1.55 / 252 | 288 / 2.16 / 552 | 0.749 | 1 | 39.4 | fall (omni, 0.3 hold after the turn) |

- The free first node did what it should: the **tracking loss is gone** at 1e-3 (user), the 0.8 hold goes
  88.7 / 1.41 / 422 → 43.8 / 0.64 / 252, every pelvis number is the best of the campaign.
- 1e-2 is past the limit even with a free first node: the in-horizon force is then too slow for the
  balance corrections themselves.
- **User's eye at 1e-3: the steps are fine (not abrupt), but the arms sway during the walk and the
  steps got short.** Two separate effects of pricing the ground-reaction rate:
  - *Arms:* arm accelerations are nearly free in the cost (r.jointAccelerations 5e-6 per (rad/s²)²,
    arm posture 5), so once the force rate is priced the optimizer uses the arms as the flywheel for the
    fast momentum changes. The pelvis metrics are blind to that; `score_suite.py` now also reports
    `leg_acc`, `leg_jerk`, `arm_acc`, `arm_pp` from the 1 kHz joint states, and `compare_suite.py` guards
    `steady_leg_jerk_mean` and `steady_arm_acc_mean` (≤ 1.15×).
  - *Step length:* the step time is fixed by the gait law, so a shorter step means less pelvis travel
    per step — the robot walks below the command. The braking/propulsion force swing at each touchdown
    scales with step length, so a rate price favours lagging the command (hold `v_err`, `step_l` in the
    per-segment limbs table). The 1e-2 fall is the same effect past its limit.

Next (config only): `humanF_1e-3` + arms stiffened ×10 (`make_wrench_configs.sh … arms`) to take the
flywheel away, rescoring the existing runs first for `step_l` / hold `v_err` / limb numbers. For the step
length the candidates are a lower weight (5e-4) or a higher base-velocity weight; if neither keeps the
smoothness, the rate weight moves to a per-phase schedule (priced around the contact handover, free
mid-stance).

### Rescore with limb metrics (`tuning_results/limbs_rescore.txt`)

| hold | human step_l / v_err / leg_acc / arm_acc | humanW_1e-4 | humanF_1e-3 |
|---|---|---|---|
| 0.5 m/s | 0.29 / 0.12 / 19.0 / 3.57 | 0.29 / 0.13 / 17.8 / 4.22 | 0.26 / 0.16 / 15.3 / 4.94 |
| 0.8 m/s | 0.38 / 0.16 / 32.1 / 5.75 | 0.37 / 0.18 / 29.7 / 6.57 | 0.34 / 0.22 / 24.1 / 8.42 |

- Legs: 1e-3 is the smoothest in the legs too (leg_acc −20 %, leg_jerk −25 %): the steps are not abrupt.
- Step length: −10 % at every speed with the hold `v_err` up 0.04–0.06 — the robot walks below the
  command. The velocity error is cheap (30 (m/s)⁻²) next to the force rate, so lagging is the
  optimizer's preferred way to pay less rate.
- Arms: arm_pp unchanged (same ~12–16° envelope) but arm_acc +40 % — the arms shake, not swing. The arm
  posture is unreferenced (armSwing off), so the motion is pure angular-momentum bookkeeping.
- User at 1e-3: upper body leans slightly back, arms do not swing forward — the torso (waist posture
  weight 2) and the arms held back are the CoM-behind-the-feet posture of a robot lagging its command.

Next candidates (`make_wrench_configs.sh`, config only): features `arms,swing,waist` (stiff arm/waist
posture with a human gait-locked arm swing reference, ±11° at 0.5 m/s) and the same plus `vel`
(base velocity weight ×3) for the step length.
