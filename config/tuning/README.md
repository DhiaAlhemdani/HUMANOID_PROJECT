# Tuning candidate configs (tracked)

Candidate controller configs for the runs in `docs/results/human_gait_template.md`, generated from
`config/g1/ros2_controllers.yaml` with `tests/tuning/make_config.py` and committed here because the
copies under `tuning_results/` kept disappearing between sessions. Pass the path to
`tests/tuning/run_suite.sh <tag> config/tuning/<name>.yaml [baseline]`.

| file | recipe (on top of the shipped G1 config) |
| --- | --- |
| `humanTEV.yaml` | `groundHeight.useEstimate true`, `swingTrajectory.touchDownVelocity -0.15` (run 22 D: first config to walk `suite_mild` both ways) |
| `humanT23d.yaml` | humanTEV + `contactSync.descentVelocity 0.05`, `blockedHeight 0.01`, `heightThreshold 0.01` (run 23 C: velocity-aware "landed") |
| `humanT24a.yaml` | humanT23d + `terrainSpeedCap.enabled true` (run 24) |
| `humanT24b.yaml` | humanTEV + `terrainSpeedCap.enabled true` (run 24) |

Regenerate, e.g.:

```bash
tests/tuning/make_config.py config/tuning/humanTEV.yaml ocs2.reference.groundHeight.useEstimate=true ocs2.gait.swingTrajectory.touchDownVelocity=-0.15
tests/tuning/make_config.py config/tuning/humanT23d.yaml --in config/tuning/humanTEV.yaml ocs2.gait.contactSync.descentVelocity=0.05 ocs2.gait.contactSync.blockedHeight=0.01 ocs2.gait.contactSync.heightThreshold=0.01
tests/tuning/make_config.py config/tuning/humanT24a.yaml --in config/tuning/humanT23d.yaml ocs2.reference.terrainSpeedCap.enabled=true
tests/tuning/make_config.py config/tuning/humanT24b.yaml --in config/tuning/humanTEV.yaml ocs2.reference.terrainSpeedCap.enabled=true
```
