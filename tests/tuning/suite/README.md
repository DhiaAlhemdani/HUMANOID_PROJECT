# Command-space benchmark suite

Each file is a command schedule for `tune_probe.py` (`PROFILE=script SCRIPT=<file>`): knots
`[t, vx, vy, yaw_rate]` in m/s and rad/s on the script's own clock (after the 4 s settle), linearly
interpolated (1.5 s ramps between holds), held after the last knot, then the 5 s stop.
`run_suite.sh <tag> <config>` runs all five and `score_suite.py` scores each by segment: every
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
