# Local validation log

This is the running record of tests and verification performed on the user's local machine during this project. Add an entry for **every test**, not only walking-smoothness tests. After a test batch has been evaluated, commit and push its concise results to the active project branch. Do not mark a test as passed unless the result was observed; record failures and environment limitations as well.

Keep raw output, bags, videos, and large replay files local unless a specific artifact is useful and small enough to check in. Record the command, conditions, concise outcome, and any local artifact path so results can be reproduced and reviewed.

## Test entries

| ID | Date (UTC) | Area / change | Commit tested | Robot / environment | Command or procedure | Result | Artifact / details |
|---|---|---|---|---|---|---|---|
| 001 | 2026-11-23 | Walking smoothness A/B | Baseline `e63dfb2`; quintic `9795811` | G1 MuJoCo, 0.25 m/s command, 25 s hold, three runs per variant | See [`walking_smoothness_ab_test.md`](walking_smoothness_ab_test.md) | All six run verdicts `SUCCESS`; measured comparison is mixed and confounded by actual MPC-rate mismatch | Summary metrics are in the linked report. Actual MPC median was 31.3 Hz cubic vs 60.8 Hz quintic; match solver rates before attributing changes to the spline. |
| 002 | 2026-11-23 | Walking smoothness result analyzer | Baseline `e63dfb2`; quintic `9795811`; analyzer fix `54323a0` | Ubuntu host, separate Python venv | `analyze_gait_regularity.py` over `/tmp/cubic_{1..3}` and `/tmp/quintic_{1..3}` | Initially blocked by missing `mujoco`; then a false-fall filter caused `IndexError`; completed after the fix | Analyzer had hardcoded a 0.7 m fall-height cutoff, above valid G1 crouch heights (minimum 0.637-0.641 m). Updated to use the test's recorded fall threshold and rerun succeeded. |
| 003 | 2026-11-23 | Rate-controlled walking smoothness A/B | Baseline `e63dfb2`; quintic `9795811`; analyzer fix `54323a0` | G1 MuJoCo, 30 Hz target, 0.25 m/s, 25 s hold, three runs per variant | `wb_id_walk_test.py --mpc-rate 30 --record-joints --record-tracking`; analyzer on `/tmp/cubic30_{1..3}` and `/tmp/quintic30_{1..3}` | All six `SUCCESS`; rates closely matched (28.9 vs 29.9 Hz); body pitch/roll slightly lower in quintic, but contact agreement worse and ~1% ankle saturation | Diagnostic 30 Hz only; report and interpretation in [`walking_smoothness_ab_test.md`](walking_smoothness_ab_test.md). Raw files remain in `/tmp`. |

## Detailed results

### 2026-11-23 — G1 cubic vs quintic swing A/B

- **Conditions:** MuJoCo, flat scene, G1, 0.25 m/s command, 25 s hold, three runs for each trajectory. The baseline was `e63dfb2`; the quintic version was `9795811`.
- **Verdicts:** all six runs reported `SUCCESS`; no falls. Actual forward speed was 0.221 m/s cubic and 0.218 m/s quintic.
- **Medians:** pelvis height 0.747 ± 0.005 m cubic vs 0.746 ± 0.005 m quintic; roll RMS 0.004 vs 0.003 rad; pitch mean ± SD +0.019 ± 0.002 vs +0.012 ± 0.002 rad; step-period CV 0.01 vs 0.02; step-length CV 0.01 vs 0.03; double-support fraction 0.378 vs 0.434; planned/replayed contact-mode agreement 0.87 vs 0.81.
- **Runtime difference:** the generated cubic/quintic YAML files were identical; both logged target=100 Hz, `nThreads: 5`, `dt: 0.02`, `sqpIteration: 1`. Actual MPC medians were 31.3 Hz vs 60.8 Hz. The provided final timing records showed 32.50 Hz / 30.217 ms average advance / 302.2% utilization (cubic) vs 47.98 Hz / 20.658 ms / 206.6% (quintic). Both had zero solver exceptions; cumulative torque-window overruns at those log points were 3301 vs 1297.
- **Interpretation:** the actual MPC-rate difference changes closed-loop timing, so the pitch/roll and contact metrics cannot isolate the spline's smoothness effect. Body pitch/roll decreased slightly, but step regularity and contact-mode agreement got slightly worse; pelvis-height variation was unchanged. No walking-smoothness improvement is claimed; a rate-controlled repeat or direct planned-foot-height comparison is needed.
- **Analyzer issues:** the first Python environment lacked `mujoco`; after dependencies were installed, the analyzer's fixed 0.7 m cutoff incorrectly treated valid G1 crouch heights as a fall. Analyzer fix `54323a0` now uses the recorded test fall threshold. Re-analysis produced the summaries above.
- **Raw artifacts:** retained locally under `/tmp/cubic_{1..3}` and `/tmp/quintic_{1..3}`; not checked into Git.

### 2026-11-23 — G1 rate-controlled cubic vs quintic follow-up

- **Conditions:** MuJoCo flat scene; G1; `--mpc-rate 30`; 0.25 m/s command; 25 s hold; three runs per version; joint and controller tracking recorded. This low rate was diagnostic only.
- **Verdicts and rates:** all six runs reported `SUCCESS`. Median measured MPC rates were 28.9 Hz cubic and 29.9 Hz quintic.
- **Medians:** measured speed 0.220 vs 0.219 m/s; pelvis height 0.748 ± 0.005 vs 0.746 ± 0.005 m; roll RMS 0.004 vs 0.003 rad; pitch +0.020 ± 0.002 vs +0.014 ± 0.006 rad; period CV 0.01 vs 0.01; step-length CV 0.02 vs 0.02; double-support fraction 0.378 vs 0.446; planned/replayed contact agreement 0.87 vs 0.80; ankle torque saturation 0 vs 0.010 median fraction.
- **Interpretation:** matched rate reduces the timing confound. Quintic slightly reduces mean pitch and roll but increases pitch variability, double support, contact-mode disagreement, and small ankle saturation; whole-body smoothness improvement remains unproven. Review swing-clearance/contact plots before changing the trajectory again. The 30 Hz diagnostic result is not validated for the intended 100 Hz operating point.
- **Raw artifacts:** `/tmp/cubic30_{1..3}` and `/tmp/quintic30_{1..3}` remain local; only summarized measurements are committed.
- **Uploaded plot review (qualitative):** cubic and quintic plots show similar broad planned/replayed contact offsets, with replayed stance occupying more of swing in several quintic traces. Both variants show step-frequency roll/pitch ripple. Low-point swing peaks are similar at roughly 0.06 m. Plots omit the planned foot-height reference and jerk, so they do not establish trajectory smoothness or premature touchdown. Uploaded images are local under `/tmp/arena-coding-uploads/*_gait.png`, not committed.
- **Supplemental user-supplied quintic summaries/replays:** maximum absolute hold pitch in the three 30 Hz-target quintic runs was 2.677° at replay-relative 28.48 s, 2.257° at 18.82 s, and 3.279° at 20.11 s. The three original-rate quintic summaries report 1.055°, 0.915°, and 0.920°; only quintic_1 replay was supplied, confirming 1.055° at 18.71 s. These quantify quintic-run peaks only; there are no cubic per-run peak values for a matched comparison, and the rate difference prevents causal attribution. The supplied 30 Hz quintic summaries show planned/replayed mode agreement 0.8028–0.8048 and geometric replay stance/swing means near 0.568/0.212 s. They lack event-by-event planned contact times; raw `_tracking.csv` files are still required for liftoff/touchdown offset calculations. Uploaded `.json`, `_gait.json`, and `_replay.json` files remain in `/tmp/arena-coding-uploads/`, not Git.

## Project-end checklist

- [ ] Add all local tests run during the project, including failures and inconclusive runs.
- [ ] Verify each entry identifies the code commit and test conditions.
- [ ] Include relevant concise metrics and conclusions; do not commit disposable build/cache files or large raw logs by default.
- [ ] Ensure every evaluated local test batch has been committed and pushed to the active project branch.
