# Tuning candidate configs (tracked)

Candidate controller configs for the runs in `docs/results/human_gait_template.md`, generated from
`config/g1/ros2_controllers.yaml` with `tests/tuning/make_config.py` and committed here because the
copies under `tuning_results/` kept disappearing between sessions. Pass the path to
`tests/tuning/run_suite.sh <tag> config/tuning/<name>.yaml [baseline]`.

| file | recipe (on top of the shipped G1 config) |
| --- | --- |
| `humanTEV.yaml` | `groundHeight.useEstimate true`, `swingTrajectory.touchDownVelocity -0.15` (run 22 D: first config to walk `suite_mild` both ways) |
| `humanT23d.yaml` | humanTEV + `contactSync.descentVelocity 0.05`, `blockedHeight 0.01`, `heightThreshold 0.01` (run 23 C: velocity-aware "landed") |
| `humanT24a.yaml` | humanT23d + `terrainSpeedCap.enabled true`, single tier `[0.01] -> [0.5]` (run 24 A: walks `suite_mild` both ways) |
| `humanT24b.yaml` | humanTEV + the same single-tier cap (run 24 A) |
| `humanT24c.yaml` | humanT23d + the two-tier cap `[0.01, 0.025] -> [0.5, 0.25]` (run 24 C, for `suite_rough`) |

Regenerate, e.g.:

```bash
tests/tuning/make_config.py config/tuning/humanTEV.yaml ocs2.reference.groundHeight.useEstimate=true ocs2.gait.swingTrajectory.touchDownVelocity=-0.15
tests/tuning/make_config.py config/tuning/humanT23d.yaml --in config/tuning/humanTEV.yaml ocs2.gait.contactSync.descentVelocity=0.05 ocs2.gait.contactSync.blockedHeight=0.01 ocs2.gait.contactSync.heightThreshold=0.01
tests/tuning/make_config.py config/tuning/humanT24a.yaml --in config/tuning/humanT23d.yaml ocs2.reference.terrainSpeedCap.enabled=true "ocs2.reference.terrainSpeedCap.stepKnots=[0.01]" "ocs2.reference.terrainSpeedCap.speedKnots=[0.5]"
tests/tuning/make_config.py config/tuning/humanT24b.yaml --in config/tuning/humanTEV.yaml ocs2.reference.terrainSpeedCap.enabled=true "ocs2.reference.terrainSpeedCap.stepKnots=[0.01]" "ocs2.reference.terrainSpeedCap.speedKnots=[0.5]"
tests/tuning/make_config.py config/tuning/humanT24c.yaml --in config/tuning/humanT23d.yaml ocs2.reference.terrainSpeedCap.enabled=true
```
