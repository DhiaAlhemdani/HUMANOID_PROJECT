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
| `humanT24c.yaml` | humanT23d + the two-tier cap `[0.01, 0.025] -> [0.5, 0.25]` (run 24 C, for `suite_rough`; `make_config` from a current G1 file now needs the knots set explicitly, the defaults moved to T24e's) |
| `humanT24d.yaml` | humanT24c with tier 2 earlier and slower: `[0.01, 0.02] -> [0.5, 0.2]` (run 24 D) |
| `humanT24e.yaml` | three tiers `[0.01, 0.02, 0.035] -> [0.5, 0.2, 0.1]` (run 24 D; **the terrain candidate**: `suite_mild` 0 falls, flat unchanged - now the G1 defaults of the block) |
| `humanT24f.yaml` | humanT24e + a smaller planned support polygon: `contactGeometry.rectangle [0.05, -0.05, 0.02, -0.02]` (from +-0.09 / +-0.03), WB-ID CoP clip 0.06 / 0.025 (run 24 E: edge landings) |

Regenerate, e.g.:

```bash
tests/tuning/make_config.py config/tuning/humanTEV.yaml ocs2.reference.groundHeight.useEstimate=true ocs2.gait.swingTrajectory.touchDownVelocity=-0.15
tests/tuning/make_config.py config/tuning/humanT23d.yaml --in config/tuning/humanTEV.yaml ocs2.gait.contactSync.descentVelocity=0.05 ocs2.gait.contactSync.blockedHeight=0.01 ocs2.gait.contactSync.heightThreshold=0.01
tests/tuning/make_config.py config/tuning/humanT24a.yaml --in config/tuning/humanT23d.yaml ocs2.reference.terrainSpeedCap.enabled=true "ocs2.reference.terrainSpeedCap.stepKnots=[0.01]" "ocs2.reference.terrainSpeedCap.speedKnots=[0.5]"
tests/tuning/make_config.py config/tuning/humanT24b.yaml --in config/tuning/humanTEV.yaml ocs2.reference.terrainSpeedCap.enabled=true "ocs2.reference.terrainSpeedCap.stepKnots=[0.01]" "ocs2.reference.terrainSpeedCap.speedKnots=[0.5]"
tests/tuning/make_config.py config/tuning/humanT24c.yaml --in config/tuning/humanT23d.yaml ocs2.reference.terrainSpeedCap.enabled=true
tests/tuning/make_config.py config/tuning/humanT24d.yaml --in config/tuning/humanT24c.yaml "ocs2.reference.terrainSpeedCap.stepKnots=[0.01, 0.02]" "ocs2.reference.terrainSpeedCap.speedKnots=[0.5, 0.2]"
tests/tuning/make_config.py config/tuning/humanT24e.yaml --in config/tuning/humanT24c.yaml "ocs2.reference.terrainSpeedCap.stepKnots=[0.01, 0.02, 0.035]" "ocs2.reference.terrainSpeedCap.speedKnots=[0.5, 0.2, 0.1]"
tests/tuning/make_config.py config/tuning/humanT24f.yaml --in config/tuning/humanT24e.yaml "ocs2.model.contactGeometry.rectangle=[0.05, -0.05, 0.02, -0.02]" control.wholeBody.soleHalfLength=0.06 control.wholeBody.soleHalfWidth=0.025
```
