# G1 walking courses

Each YAML here is one course for `description/mujoco/terrains/generate_course.py` (key `course`; feature kinds and
their fields are documented in the script). The suite runs them with `TERRAIN=<name>` (`tests/tuning/run_suite.sh`),
the module names are the file names. All courses lie along world +x from the spawn at the origin and are mirrored
at -x, so `speeds` (forward) and `backward` meet the same features. The MPC gets no terrain map; everything here
is walked blind.

| course | what | severity ladder |
|---|---|---|
| `slope_5`, `slope_10` | 6 m ramp up, 2 m plateau, 6 m ramp down (5 deg: 0.53 m rise; 10 deg: 1.06 m) | 5 -> 10 deg |
| `curb_4`, `curb_8` | three 3 m plateaus with 2 m gaps: six curbs up / down per direction | 4 -> 8 cm |
| `stairs_3x10` | three 10 cm x 0.30 m steps up, a 1.5 m landing, three down | the stairs rung (stair_climbing has its own planner; this is the blind walker) |
| `course` | the combination: mild field, curb, ramp up / plateau / down, rough field (DRAFT: composed from what passes the rungs above) | - |

The uneven strips (`suite_mild`, `suite_rough`) are generate_uneven_terrain.py fields, see `../uneven_suite_*_params.yaml`.

Regenerate after an edit (and `--check` in CI):

```bash
for c in slope_5 slope_10 curb_4 curb_8 stairs_3x10 course; do
  python3 description/mujoco/terrains/generate_course.py --params config/terrains/g1/$c.yaml --mjcf description/mujoco/terrains/g1_$c.xml
done
```
