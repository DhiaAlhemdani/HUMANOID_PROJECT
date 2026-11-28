"""Phase 1.5 contract: plot_tracking.py's `gait` section finds gait templates and transitions.

Runs the real plot_tracking.py on synthetic recordings from tests/tuning/synth_run.py (scripted
plan_mode sequences, a pelvis bump at each change). Needs numpy + matplotlib (skipped otherwise).
"""

import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
SYNTH = ROOT / "tests/tuning/synth_run.py"
PLOT = ROOT / "tests/tuning/plot_tracking.py"
HAVE = importlib.util.find_spec("numpy") is not None and importlib.util.find_spec("matplotlib") is not None


def run(tmp: Path, name: str, *synth_args: str) -> dict:
    d = tmp / name
    subprocess.run([sys.executable, str(SYNTH), str(d), *synth_args], check=True, capture_output=True)
    subprocess.run([sys.executable, str(PLOT), str(d), name], check=True, capture_output=True, cwd=tmp)
    return json.loads((d / "diag.json").read_text())


@unittest.skipUnless(HAVE, "numpy/matplotlib not installed")
class GaitTransitionMetrics(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        t = Path(cls.tmp.name)
        cls.bump = run(t, "bump", "--sequence", "walk:8,trot:8,walk:6", "--bump", "0.03")
        cls.flat = run(t, "flat", "--sequence", "walk:8,trot:8,walk:6", "--bump", "0.0")
        cls.splice = run(t, "splice", "--sequence", "walk:8,trot:8,walk:6", "--bump", "0.0", "--splice", "0.15")
        cls.walk = run(t, "walk", "--sequence", "walk:20", "--bump", "0.0")
        cls.stop = run(t, "stop", "--sequence", "walk:6,run:6,walk:6", "--bump", "0.01", "--splice", "1.5")

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def test_sequence_and_template_time(self):
        g = self.bump["gait"]
        self.assertEqual(g["sequence"], "stance>walk>trot>walk>stance")
        self.assertAlmostEqual(g["templates_s"]["trot"], 8.0, places=1)
        self.assertAlmostEqual(g["templates_s"]["walk"], 14.0, delta=0.4)

    def test_transition_times_and_direction(self):
        ch = self.bump["gait"]["changes"]
        self.assertEqual([(c["from"], c["to"]) for c in ch], [("walk", "trot"), ("trot", "walk")])
        self.assertAlmostEqual(ch[0]["t"], 15.0, places=2)   # start of the first trot step
        self.assertAlmostEqual(ch[1]["t"], 23.0, places=2)   # start of the first walk step
        self.assertEqual(self.bump["gait"]["changes_locomotion"], 2)

    def test_bump_is_measured_in_the_transition_window_only(self):
        gb, gf = self.bump["gait"], self.flat["gait"]
        self.assertGreater(gb["transition_z_pp_mm_max"], 25.0)   # 30 mm bump injected
        self.assertLess(gf["transition_z_pp_mm_max"], 6.0)
        self.assertGreater(gb["transition_roll_max_deg"], 2 * gf["transition_roll_max_deg"])  # 0.86 deg on a 0.57 ripple
        self.assertGreater(gb["transition_acc_hf_max"], 10 * gf["transition_acc_hf_max"])

    def test_splice_shows_up_as_the_gap_and_shifts_the_switch(self):
        ch = self.splice["gait"]["changes"]
        self.assertEqual(len(ch), 2)
        self.assertAlmostEqual(ch[0]["gap_s"], 0.1 + 0.15, places=2)  # walk DS kept + splice
        self.assertAlmostEqual(ch[1]["gap_s"], 0.15, places=2)
        self.assertAlmostEqual(ch[0]["t"], 15.15, places=2)
        self.assertAlmostEqual(ch[1]["t"], 23.3, places=2)
        self.assertEqual(self.splice["gait"]["templates_s"]["trot"], 8.0)

    def test_first_step_asymmetry_is_near_zero_for_regular_templates(self):
        self.assertLess(self.bump["gait"]["first_step_asym_max"], 0.05)

    def test_no_transition_and_stop_between_gaits(self):
        self.assertEqual(self.walk["gait"]["sequence"], "stance>walk>stance")
        self.assertEqual(self.walk["gait"]["changes_locomotion"], 0)
        self.assertIsNone(self.walk["gait"]["transition_acc_hf_max"])
        self.assertEqual(self.stop["gait"]["sequence"], "stance>walk>stance>run>stance>walk>stance")
        self.assertEqual(self.stop["gait"]["changes_locomotion"], 0)  # 1.5 s stops are not transitions


if __name__ == "__main__":
    unittest.main()
