# Command-space baseline (shipped controller, 2026-11-25)

`run_suite.sh base default` — five scripts, segment scores from `score_suite.py`; videos
`tuning_results/base__*/replay.mp4` judged by the owner.

## Steady segments by the gait the ladder chose

| gait | commands | jerk [m/s³] | acc_hf | pitch pp | leg τ rate [Nm/s] | owner's video verdict |
|---|---|---|---|---|---|---|
| stance | 0 | 0.5–2.7 | 0.00 | 0.1° | 3–6 | — |
| walk (0.3 s swing + 0.1 s DS) | 0.2, 0.3, −0.2, 0.3+turn 0.4, 0.2+vy 0.1 | 62–68 | 1.5–1.7 | 0.5–0.9° | 265–313 | not smooth |
| **trot** (0.5 s single supports) | 0.5, 0.6 | **28–36** | **0.33–0.50** | 0.9–1.3° | 306–318 | **smooth** (0.5–0.7 m/s in omni, speeds, stops) |
| run (flight) | 0.8 | 263 | 5.7 | 16° | 1014 | not smooth |

Transitions score like the gait they lead into (ramp to 0.8: 164 because the run is 263; ramps
inside the walk gait: 59–67 like the walk holds). Suite aggregates: speeds steady 75 / transition
max 164; turns 52 / 76; lateral 39 / 68; stops 15 / 44; omni 46 / 67 (jerk).

## Calibration of the proxy — the project target

The owner's eye and the jerk/acc_hf ranking agree segment for segment. **"Visibly smooth" =
the trot's level: jerk ≤ ~35 m/s³ and acc_hf ≤ ~0.5 per segment.** The target of roadmap v2 is
every segment of the suite — holds and transitions, all speeds, turns, lateral, stops — at or
below that level, with command tracking kept (`v_err` guard).

## What this says about the work so far and the next step

Phases 1–2.C optimised the trot at 0.5 m/s: the one gait in the command space that is smooth.
Below 0.45 m/s the ladder runs the walk template, which is twice the jerk and four times the
acc_hf of the trot (the gait sweep saw the same at 0.5); above 0.7 it runs a flight gait that
pitches 16° peak-to-peak. The command-space problem is therefore first of all *which contact
pattern runs at which speed*. First experiment: the trot everywhere (`trotall`: walk band dead,
trot 0.03–1.0 m/s, run ≥ 0.95) on `speeds` and `omni`.
