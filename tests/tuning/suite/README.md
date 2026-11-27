# Command-space benchmark suite

Each file is a command schedule for `tune_probe.py` (`PROFILE=script SCRIPT=<file>`): knots
`[t, vx, vy, yaw_rate]` in m/s and rad/s on the script's own clock (after the 4 s settle), linearly
interpolated (1.5 s ramps between holds), held after the last knot, then the 5 s stop.
`run_suite.sh <tag> <config>` runs the default six (`height` is opt-in: `SCRIPTS=height`) and `score_suite.py` scores each by segment: every
constant-command hold (after 1.5 s of settling) is a *steady* segment, every ramp plus 2 s after it
a *transition*. Gaits engaged by the ladder along the way (walk / trot / run, stance) are part of
the test, not controlled.

| script | covers | length |
|---|---|---|
| speeds | 0.2 → 0.5 → 0.8 → 0.5 → 0.2 → 0 m/s: every gait rung up and down | 40 s |
| turns | turn in place, arcs at 0.4 m/s both ways | 46 s |
| lateral | side-steps both ways, diagonal | 39 s |
| stops | three start/stop cycles at 0.5 m/s | 31 s |
| omni | a joystick-like mixture incl. backward | 42 s |
| height | the GUI slider: 0.5 m/s walk with the commanded height stepped 0.785 -> 0.70 -> 0.785 while walking, then standing 0.785 -> 0.66 -> 0.785 (5th knot column = height; the controller's height slew shapes the step) | 32 s |
| backward | the speeds ladder in reverse: -0.2 -> -0.5 -> -0.8 -> -0.5 -> -0.2 -> 0 m/s (the human-gait arm / waist templates run in reverse, see Run 19) | 40 s |

Scripts without a 5th column command the pelvis height `HEIGHT` (default 0.785 m, the shipped `defaultBaseHeight`).
`run_height_ladder.sh <tag> <config> [heights...]` runs one script (default `speeds`) at several heights
(default 0.785 0.75 0.72 0.68) and compares them with the first as the baseline; `z_m` in the limbs table is
the pelvis height the robot actually walked at.
