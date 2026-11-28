# Tuning candidate configs (tracked)

Candidate controller configs for the runs in `docs/results/human_gait_template.md`, generated from
`config/g1/ros2_controllers.yaml` with `tests/tuning/make_config.py` and committed here because the
copies under `tuning_results/` kept disappearing between sessions. Pass the path to
`tests/tuning/run_suite.sh <tag> config/tuning/<name>.yaml [baseline]`.

| file | recipe (on top of the G1 config as shipped before run 27: humanUB4 + lift-off window + contactSync) |
| --- | --- |
| `humanTEV.yaml` | `groundHeight.useEstimate true`, `swingTrajectory.touchDownVelocity -0.15` (run 22 D: first config to walk `suite_mild` both ways) |
| `humanT23d.yaml` | humanTEV + `contactSync.descentVelocity 0.05`, `blockedHeight 0.01`, `heightThreshold 0.01` (run 23 C: velocity-aware "landed") |
| `humanT24a.yaml` | humanT23d + `terrainSpeedCap.enabled true`, single tier `[0.01] -> [0.5]` (run 24 A: walks `suite_mild` both ways) |
| `humanT24b.yaml` | humanTEV + the same single-tier cap (run 24 A) |
| `humanT24c.yaml` | humanT23d + the two-tier cap `[0.01, 0.025] -> [0.5, 0.25]` (run 24 C, for `suite_rough`; `make_config` from a current G1 file now needs the knots set explicitly, the defaults moved to T24e's) |
| `humanT24d.yaml` | humanT24c with tier 2 earlier and slower: `[0.01, 0.02] -> [0.5, 0.2]` (run 24 D) |
| `humanT24e.yaml` | three tiers `[0.01, 0.02, 0.035] -> [0.5, 0.2, 0.1]` (run 24 D; **the terrain candidate**: `suite_mild` 0 falls, flat unchanged - now the G1 defaults of the block). From run 25 A every file here also carries the slope tiers `slopeKnots [0.05, 0.12] -> [0.3, 0.2]` (the block's defaults). |
| `humanT25c.yaml` | humanT24e / T25a + `groundHeight.followSlope true` (run 25 C: on an incline the cap reads, per-foot support heights on the plane and the plane's normal for the foot orientation; generated from the current G1 file with the T24e keys set explicitly). The same file ran as humanT25c3-c6, humanT26 and humanT27: those are code revisions of the plane estimate, not config changes. **Shipped as the G1 config after run 27** (the file now differs from `config/g1/ros2_controllers.yaml` only by the suite's `diagnostics.trackingRate`) |
| `humanT28.yaml` | the shipped (run 27) G1 config + `contactSync.probeWindow 0.3`, `probeDescent 0.15` (run 28: a foot still descending at its planned touchdown is held, the gait slides with it and its target goes down at 0.15 m/s until it finds the ground - the first stride off a crest; run 28 B: the hold outlives the policy, the unused push slides back, no probe before a stand). **Retired after run 28 B: four falls in four runs - see docs/results/human_gait_template.md** |
| `humanT29.yaml` | the shipped (run 27) recipe (humanT25c) + `control.wholeBody.searchWindow 0.3` (run 29: the touchdown search - a foot the plan has just loaded whose measured sole still descends is kept a swing foot of the whole-body layer, driven down at 0.15 m/s until it stops; decided at 500 Hz, the plan untouched - the crest residual without the run 28 probe's latency; 29 B: the searching leg is free of the posture PD; 29 C: the sole keeps the plan's orientation while it searches; 29 D: the descent never brakes the swing and grows 1.5 m/s² - `searchDescentAcceleration`; 29 E: a sole that does not follow the push is landed. slope_5 passes all guards at 29 E, crest-down pitch 22 → 2; tiles fall in every search run) |
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
tests/tuning/make_config.py config/tuning/humanT25c.yaml ocs2.reference.groundHeight.useEstimate=true ocs2.gait.swingTrajectory.touchDownVelocity=-0.15 ocs2.gait.contactSync.descentVelocity=0.05 ocs2.gait.contactSync.blockedHeight=0.01 ocs2.gait.contactSync.heightThreshold=0.01 ocs2.reference.terrainSpeedCap.enabled=true ocs2.reference.groundHeight.followSlope=true
tests/tuning/make_config.py config/tuning/humanT28.yaml --in config/tuning/humanT25c.yaml ocs2.gait.contactSync.probeWindow=0.3 ocs2.gait.contactSync.probeDescent=0.15
tests/tuning/make_config.py config/tuning/humanT29.yaml --in config/tuning/humanT25c.yaml control.wholeBody.searchWindow=0.3
tests/tuning/make_config.py config/tuning/humanT24f.yaml --in config/tuning/humanT24e.yaml "ocs2.model.contactGeometry.rectangle=[0.05, -0.05, 0.02, -0.02]" control.wholeBody.soleHalfLength=0.06 control.wholeBody.soleHalfWidth=0.025
```
