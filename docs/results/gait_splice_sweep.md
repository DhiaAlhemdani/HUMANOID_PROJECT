# Phase 1.5 — gait-transition splice sweep (`gait.phaseTransitionStanceTime`)

- **Date**: 2026-11-24
- **Branch**: `arena/47d87774-humanoid-project` (tooling `5be82ac`)
- **Method**: G1 probe at **0.6 m/s** (stand → ramp → 20 s hold → ramp → stop); the ramp crosses
  walk→trot on the way up and trot→walk on the way down (ladder: walk ≤0.45 m/s, trot engages
  at measured >0.40). Baseline = shipped config (`phaseTransitionStanceTime 0.0`) via
  `make_config`; candidates 0.05 / 0.10 / 0.15 / 0.20 s. Same build throughout.
  Tooling: `run_gaitsplice_sweep.sh`, `compare_gaitsplice.py`, `plot_tracking.py` `gait` section.
- **Gates**: any-of transition primaries ≤ 0.85× (worst change window [−0.5, +1.5] s:
  `acc_hf`, z peak-to-peak, roll max); guards: same transition count, non-improved primaries
  not worse, first-step asymmetry, steady-state set, survival.

## What the lever does (from the code)

`GaitScheduleUpdater` splices a new template in at the event before an `LF` phase;
`GaitSchedule::insertModeSequenceTemplate` adds `phaseTransitionStanceTime` of stance only when
the outgoing mode is not already stance. So walk→trot **deletes the walk's last double support**
and inserts the splice instead (shipped 0 → swing straight into swing); trot→walk inserts it after
a swing; stand↔gait changes are unaffected.

## Results — 0/4; every value is worse than no splice

Every run: survived, sequence `stance>walk>trot>walk>stance`, exactly 2 gait-to-gait transitions.

| run | splice [s] | trans. acc_hf | trans. z pp [mm] | trans. roll max [°] | 1st-step asym | steady acc_hf | jump@switch | solve [Hz] | verdict |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| **gs_base** | 0.00 | **1.761** | **25.2** | **1.07** | 0.00 | 1.404 | 0.140 | 67.4 | — |
| gs_t005 | 0.05 | 1.863 (+6 %) | 26.6 (+5 %) | 1.53 (**+43 %**) | 0.00 | 1.403 | 0.154 | — | guards-fail |
| gs_t010 | 0.10 | 2.108 (**+20 %**) | 26.5 (+5 %) | 1.45 (**+36 %**) | 0.00 | 1.370 | 0.195 (+39 %) | — | guards-fail |
| gs_t015 | 0.15 | 1.981 (+12 %) | 33.1 (**+31 %**) | 1.51 (**+41 %**) | 0.00 | 1.354 | 0.151 | 59.8 (−11 %) | guards-fail |
| gs_t020 | 0.20 | 1.834 (+4 %) | 33.6 (**+33 %**) | 1.58 (**+48 %**) | **0.99** | 1.371 | 0.185 (+32 %) | — | guards-fail |

Steady-state (hold, trot @ measured 0.50 m/s): acc_hf 1.35–1.40, roll 0.73–0.76°, vx 0.500–0.502
— the splice is invisible in steady state, as expected (it acts only at the two switches).

## Findings

1. **A stance pause at a gait change is the wrong primitive for this controller.** The lateral
   lurch at the switch grows at *every* splice length (+36…48 %), and the height excursion jumps
   once the pause reaches 0.15 s (+31 %). Mechanism: during the inserted stance both feet are
   planted while the base keeps its momentum (0.4–0.5 m/s), so the first step of the new template
   starts from a larger CoM offset and has to catch up — a bigger, more lateral first step rather
   than a smoother one. The shipped behaviour (swing straight into swing, no pause) keeps the
   stepping rhythm continuous and is the best of the five.
2. **Longer splices break the step rhythm**: at 0.2 s the first step after the change deviates
   99 % from the new template's step duration; at 0.1 and 0.2 s the per-solve command jump rises
   32–39 % (a schedule discontinuity the solver has to re-plan across).
3. **Compute**: 0.15 s cost 11 % of the solve rate (67.4 → 59.8 Hz); the others were flat.
4. **Trot is the smoothness problem, not the transition.** New information from running at
   0.6 m/s: steady `acc_hf` in trot is **1.40** vs 1.05–1.08 in walk, and the walk→trot change
   is itself a 25 mm height / 1.1° roll event even with the best setting. The G1's trot template
   (`[LF, RF]`, 0.5 s steps, no double support, swing height 0.09) is where the remaining bounce
   lives at ≥0.5 m/s.

## Decisions

- **Not shipped**: `phaseTransitionStanceTime` stays **0.0**. §1.5's "stance splice" item is
  closed negative; the roadmap's other §1.5 ideas (cost-based rung smoothing, step-timing
  symmetry check) are superseded by the finding that the pause itself is harmful — the
  `gait` diag section is the tool for any future transition work.
- **Noted for later (outside Phase 1 scope unless asked)**: trot-template smoothness at ≥0.5 m/s
  (swing time / height, or a `fast_walk`-with-double-support rung between walk and trot in the
  ladder) is the next largest measurable bounce source.
