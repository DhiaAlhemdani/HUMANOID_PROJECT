# Walking smoothness A/B validation

**Purpose:** Preserve the user's local MuJoCo comparison results for commit to GitHub after the test batch is evaluated. Do not claim an improvement until both versions have been compared under controlled conditions.

## Versions

- Baseline cubic swing trajectory: `e63dfb2` (parent of the minimum-jerk change).
- Minimum-jerk swing trajectory: `9795811`.
- Robot tested: G1 in MuJoCo.

## Test protocol

Run both versions in MuJoCo on the same flat scene, with the same robot model, gait configuration, command, height, MPC rate, and run duration. Use a slow initial command (0.25 m/s), record joints and controller tracking, and perform at least three runs per version. Use fresh terminals when switching ROS install overlays so one version does not shadow the other.

Suggested command for each run (change `--robot-name` and the prefix as needed):

```bash
python3 <repo>/tests/wb_id_walk_test.py \
  --robot-name g1 --speed 0.25 --hold 25 \
  --record-joints --record-tracking \
  --prefix /tmp/<variant>_<run>
```

Analyze the matching prefixes with:

```bash
python3 <repo>/tests/tuning/analyze_gait_regularity.py \
  /tmp/cubic_1 /tmp/cubic_2 /tmp/cubic_3 \
  /tmp/quintic_1 /tmp/quintic_2 /tmp/quintic_3 \
  --robot-name g1 --bins
```

## Results

| Variant | Run | Verdict / fall | Command (m/s) | Measured hold speed (m/s) | Base-height SD (m) | Roll RMS (rad) | Pitch SD (rad) | Step-period CV | Step-length CV | MPC rate (Hz) | Notes / artifact path |
|---|---:|---|---:|---:|---:|---:|---:|---:|---:|---|
| Cubic baseline | 1 | SUCCESS | 0.25 | 0.221 | 0.005 | 0.004 | 0.002 | 0.01 | 0.02 | 31.7 | DS 0.379; mode agreement 0.87; saturation 0; peak abs roll/pitch 0.010/0.024 rad |
| Cubic baseline | 2 | SUCCESS | 0.25 | 0.221 | 0.005 | 0.004 | 0.002 | 0.00 | 0.01 | 31.3 | DS 0.376; mode agreement 0.87; saturation 0; peak abs roll/pitch 0.010/0.024 rad |
| Cubic baseline | 3 | SUCCESS | 0.25 | 0.221 | 0.005 | 0.004 | 0.002 | 0.01 | 0.01 | 31.2 | DS 0.378; mode agreement 0.87; saturation 0; peak abs roll/pitch 0.009/0.023 rad |
| Quintic swing | 1 | SUCCESS | 0.25 | 0.219 | 0.005 | 0.003 | 0.002 | 0.02 | 0.03 | 58.7 | DS 0.434; mode agreement 0.81; saturation 0.010; peak abs roll/pitch 0.007/0.050 rad |
| Quintic swing | 2 | SUCCESS | 0.25 | 0.218 | 0.005 | 0.003 | 0.002 | 0.02 | 0.04 | 61.6 | DS 0.437; mode agreement 0.81; saturation 0.010; peak abs roll/pitch 0.006/0.046 rad |
| Quintic swing | 3 | SUCCESS | 0.25 | 0.218 | 0.005 | 0.003 | 0.001 | 0.02 | 0.03 | 60.8 | DS 0.434; mode agreement 0.81; saturation 0.011; peak abs roll/pitch 0.007/0.046 rad |

## Interpretation

- Prioritize no falls or new actuator saturation. Compare medians across repeats, not just one run.
- Check base-height variation, body roll/pitch, step-period and step-length variability, and the generated `*_gait.png` plots.
- A smoother planned vertical foot curve is not by itself evidence that whole-body walking improved. Record whether the visible gait changed and whether any metric regressed.
- Keep raw generated logs and replay files outside Git unless a specific artifact is requested; record the concise results and reproducible conditions here.

## Local A/B result (2026-11-23)

Three G1 MuJoCo runs per trajectory, command 0.25 m/s, 25 s hold. All six test verdicts were `SUCCESS`. Aggregates below are medians across runs:

| Metric | Cubic baseline | Quintic swing | Initial reading |
|---|---:|---:|---|
| Measured hold speed [m/s] | 0.221 | 0.218 | Similar |
| Pelvis height mean ± SD [m] | 0.747 ± 0.005 | 0.746 ± 0.005 | Essentially unchanged |
| Roll RMS [rad] | 0.004 | 0.003 | Tiny reduction |
| Pitch mean ± SD [rad] | +0.019 ± 0.002 | +0.012 ± 0.002 | Lower forward pitch; needs cautious interpretation |
| Step-period CV | 0.01 | 0.02 | Slightly more variable |
| Step-length CV | 0.01 | 0.03 | More variable |
| Double-support fraction | 0.378 | 0.434 | Increased |
| Planned/replayed mode agreement | 0.87 | 0.81 | Decreased |
| Actual MPC rate [Hz] | 31.3 | 60.8 | Large mismatch; comparison is confounded |
| Fall / balance verdict | 3/3 success | 3/3 success | Both survived |

The generated YAML files are identical, and both logs report a 100 Hz MPC target with the same `nThreads: 5`, `dt: 0.02`, and `sqpIteration: 1`. Actual solver performance nevertheless differed: the analyzer medians were 31.3 Hz cubic vs 60.8 Hz quintic; the supplied final timing lines show 32.50 Hz / 30.217 ms average advance (302.2% utilization) vs 47.98 Hz / 20.658 ms (206.6%). Both had zero solver exceptions; the cumulative torque-window overrun counters were 3301 and 1297 at those log points. This is an observed runtime difference, not a YAML target mismatch, and it affects the closed-loop gait results. The gait metrics remain mixed, so do not attribute the body-pitch change to the spline alone. A rate-controlled repeat or direct comparison of the planned foot-height signals is needed before claiming improved walking smoothness; do not retune or revert the default based on this A/B alone.

## Rate-controlled follow-up (2026-11-23)

A second three-run-per-variant G1 MuJoCo A/B used a 30 Hz MPC target, 0.25 m/s command, and 25 s hold. This diagnostic rate is intentionally low to match the baseline solver's measured capacity; it is not a recommended operating setting. All six runs reported `SUCCESS`.

| Metric | Cubic baseline | Quintic swing | Reading |
|---|---:|---:|---|
| Measured hold speed [m/s] | 0.220 | 0.219 | Matched |
| Pelvis height mean ± SD [m] | 0.748 ± 0.005 | 0.746 ± 0.005 | Essentially unchanged |
| Roll RMS [rad] | 0.004 | 0.003 | Small reduction |
| Pitch mean ± SD [rad] | +0.020 ± 0.002 | +0.014 ± 0.006 | Lower mean pitch, greater run/sample variability |
| Step-period CV | 0.01 | 0.01 | No material difference |
| Step-length CV | 0.02 | 0.02 | No material difference |
| Double-support fraction | 0.378 | 0.446 | Higher in quintic runs |
| Planned/replayed mode agreement | 0.87 | 0.80 | Lower in quintic runs |
| Actual MPC rate [Hz] | 28.9 | 29.9 | Closely matched |
| Ankle torque saturation fraction | 0 | 0.010 | Small new saturation in quintic runs |
| Fall / balance verdict | 3/3 success | 3/3 success | Both survived |

Per-run reports are in the `/tmp/cubic30_*` and `/tmp/quintic30_*` artifacts on the user's machine. The contact-mode difference may reflect earlier or longer physical foot contact, or geometric replay classification; the aggregate report cannot distinguish these. Review the generated `*_gait.png` plots and swing-foot clearance before deciding whether the vertical profile needs further tuning.

## Final assessment

The rate-matched diagnostic shows a small improvement in mean body pitch and roll, with similar speed, pelvis-height variation, and step regularity. It also shows higher double-support fraction, worse planned/replayed contact agreement, and about 1% ankle torque saturation with the quintic path. The uploaded gait plots visually corroborate the contact discrepancy: replayed contact occupies more of the swing intervals than the planned schedule, particularly for the quintic runs. Both variants show strong step-frequency roll/pitch ripple; quintic30_1 and quintic30_2 also show isolated pitch transients during the hold. The low-point traces show similar roughly 6 cm swing peaks, but these plots do not show the planned foot-height reference or its jerk, so they cannot establish that the quintic path itself is smoother. Overall, this is not yet a clear whole-body smoothness improvement. Because the matched test ran at only about 30 Hz, do not generalize it to the intended 100 Hz operating point. Next, inspect the tracking records around the pitch transients and quantify liftoff/touchdown timing error before changing the swing trajectory further.
