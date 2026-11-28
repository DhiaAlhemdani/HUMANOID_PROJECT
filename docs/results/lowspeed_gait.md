# Low-speed gait: what the command space's slow end can and cannot look like (2026-11-25)

Config-only experiments on `speeds` + `omni`, scored with `score_suite.py`, videos judged by the owner.
All variants: walk band dead or repurposed, trot 0.40–1.0 m/s, run ≥ 0.95 (never reached).

| variant | pattern below 0.45 m/s | pattern 0.45–1.0 | steady jerk / acc_hf (both scripts) | transition jerk max | falls | owner's verdict on 0.2 m/s and −0.2 m/s |
|---|---|---|---|---|---|---|
| base | walk 0.3 s swing + 0.1 s DS, lift 8 cm | trot 0.5 s (≤0.75), run above | 58 / 1.29 | 164 | 0 | rough |
| trotall | trot 0.5 s, lift 9 cm | trot 0.5 s | 20.3 / 0.28 | 44 | 0 | pelvis quiet, **step length short** |
| trotlow | trot 0.5 s, lift 5 cm | same, lift 5 cm | **17.4 / 0.22** | **38** | 0 | same as trotall (lift height invisible) |
| slowtrot | trot 0.7 s, lift 5 cm | trot 0.5 s | 22.8 / 0.32 | 322 (fall) | **1** (0.6 → 0.2+lateral down-switch) | still short |
| stroll50 | 0.5 s swing + 0.5 s DS | trot 0.5 s, lift 5 | 27.3 / 0.49 | 44 | 0 | **pause between steps very bothering**, still short |
| stroll70 | 0.5 s swing + 0.7 s DS | trot 0.5 s, lift 5 | 25.4 / 0.45 | 40 | 0 | same; v_err 0.32 (did not even hold 0.2 m/s) |

Owner's within-trot ranking from the videos: 0.8 > 0.6 > 0.5 > 0.3 ≈ 0.2 ≈ −0.2 m/s. The pelvis-jerk
proxy ranks the opposite way (0.2: 17, 0.8: 53): it measures ride quality and is blind to the legs.
The visible criterion at low speed is **step length**, and step length = speed × step time:

| speed [m/s] | 0.5 s steps (trot) | 0.7 s | 1.0 s (stroll50) | 1.2 s (stroll70) | for a 0.40 m step |
|---|---|---|---|---|---|
| 0.2 | 0.10 m | 0.14 | 0.20 | 0.24 | 2.0 s per step |
| 0.3 | 0.15 | 0.21 | 0.30 | 0.36 | 1.3 s |
| 0.5 | 0.25 | 0.35 | — | — | 0.8 s |
| 0.8 | 0.40 | — | — | — | 0.5 s |

Long steps at low speed need long step times; 0.7 s of single support already cost a fall, and the
double-support way of lengthening the step (how humans stroll) is the pause the owner rejects and is
twice as rough (two extra contact switches per cycle). **Below ~0.4 m/s there is no contact pattern
that gives long steps without pauses; this is kinematics, not tuning.** The natural-looking range of
this robot is the trot at 0.5–0.8 m/s (0.25–0.40 m steps at 2 steps/s).

Kept: `trotlow` (trot everywhere, 5 cm lift) is the best configuration on every metric; the walk and
run templates are the roughness of the shipped ladder. Open decision: what a command below ~0.4 m/s
should do (see roadmap v2).
