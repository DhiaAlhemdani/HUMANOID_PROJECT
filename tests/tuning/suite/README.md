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
| height_back | the `height` script at -0.5 m/s (slider moves while walking backward) | 32 s |
| crouch | a 10 cm crouch on command while walking: 0.3 m/s and 0.5 m/s, height 0.72 -> 0.62 -> 0.72 at each (the crouch-gait target; z_m says how much of it the walk delivers) | 40 s |
| backward | the speeds ladder in reverse: -0.2 -> -0.5 -> -0.8 -> -0.5 -> -0.2 -> 0 m/s (the human-gait arm / waist templates run in reverse, see Run 19) | 40 s |

Scripts without a 5th column command the pelvis height `HEIGHT` (default 0.785 m, the shipped `defaultBaseHeight`).
`TERRAIN=<module>` (default `none`, flat) runs the same scripts on a terrain, one rung of the ladder at a time:

| TERRAIN | what | source |
|---|---|---|
| `suite_mild`, `suite_rough` | 40 x 6 m strips of 0.5 m tiles; mild = 1-2 cm steps, tops to 5 cm; rough = 2-4 cm steps, tops to 10 cm; fade in from a 1 m clear patch | `config/terrains/uneven_suite_*_params.yaml` |
| `slope_5`, `slope_10` | 6 m ramp up, 2 m plateau, 6 m ramp down | `config/terrains/g1/slope_*.yaml` |
| `curb_4`, `curb_8` | three 3 m plateaus, six curbs up / down per direction | `config/terrains/g1/curb_*.yaml` |
| `stairs_3x10` | 3 x 10 cm steps up, landing, 3 down (blind) | `config/terrains/g1/stairs_3x10.yaml` |
| `course` | the combined scene (draft until the rungs are done) | `config/terrains/g1/course.yaml` |

Every course is mirrored at -x, so `speeds` (forward) and `backward` meet the same features; they are the relevant scripts
(6 m wide: `turns` / `lateral` are not meant for them). The controller gets no terrain map: it walks blind. `z_m` is the
absolute pelvis height, so on terrain it includes the ground under the robot.
`run_height_ladder.sh <tag> <config> [heights...]` runs one script (default `speeds`) at several heights
(default 0.785 0.75 0.72 0.68) and compares them with the first as the baseline; `z_m` in the limbs table is
the pelvis height the robot actually walked at.
