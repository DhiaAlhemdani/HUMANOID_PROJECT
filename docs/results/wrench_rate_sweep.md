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
