"""The G1 course modules (config/terrains/g1/*.yaml -> description/mujoco/terrains/g1_*.xml) are current, load, and
have the surface their YAML describes, mirrored at -x. Run from tests/: python3 -m unittest test_g1_courses."""

import importlib.util
import math
from pathlib import Path
import unittest

import yaml
from mujoco_scene import MODULES, temporary_scene

try:
    import mujoco
    import numpy as np
except ImportError:
    mujoco = None

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "description/mujoco/terrains/generate_course.py"
spec = importlib.util.spec_from_file_location("generate_course", SCRIPT)
course = importlib.util.module_from_spec(spec)
spec.loader.exec_module(course)
COURSES = ("slope_5", "slope_10", "curb_4", "curb_8", "stairs_3x10", "course")


def load(name):
    params = ROOT / f"config/terrains/g1/{name}.yaml"
    config = yaml.safe_load(params.read_text())["course"]
    config.setdefault("source", str(params.relative_to(ROOT)))
    return config, params.parent


class G1CoursesTest(unittest.TestCase):
    def test_generated_files_are_current_and_registered(self):
        for name in COURSES:
            config, params_dir = load(name)
            text = course.uneven.xml_text(course.build_mjcf(config, params_dir))
            self.assertEqual((ROOT / f"description/mujoco/terrains/g1_{name}.xml").read_text(), text, name)
            self.assertEqual(MODULES[name], f"terrains/g1_{name}.xml")

    def test_invalid_courses_fail(self):
        config, params_dir = load("curb_4")
        for bad in ({"plateaus": [{"name": "a", "x": [2, 4], "top": .04}, {"name": "b", "x": [3, 5], "top": .04}]},
                    {"plateaus": [{"name": "spawn", "x": [-.2, .8], "top": .04}]},
                    {"plateaus": [{"name": "flat", "x": [2, 4], "top": 0.}]},
                    {"ramps": [{"name": "r", "x": [4, 2], "z": [0, .1]}], "plateaus": []},
                    {"stairs": [{"name": "s", "x0": 2., "riser": 0., "tread": .3, "count": 3}], "plateaus": []}):
            with self.assertRaises(ValueError, msg=str(bad)):
                course.build_mjcf({**config, **bad}, params_dir)
        config, params_dir = load("course")
        config["uneven_fields"][0]["edge_blend"] = 0.3
        with self.assertRaises(ValueError):
            course.build_mjcf(config, params_dir)

    @unittest.skipIf(mujoco is None, "mujoco not installed")
    def test_surface_heights_match_the_yaml_both_ways(self):
        expected = {
            "slope_5": [(4.5, 3 * math.tan(math.radians(5))), (8.5, 6 * math.tan(math.radians(5))), (12.5, 3 * math.tan(math.radians(5))), (1.0, 0.)],
            "slope_10": [(4.5, 3 * math.tan(math.radians(10))), (8.5, 6 * math.tan(math.radians(10))), (15.8, 0.)],
            "curb_4": [(3.5, .04), (6., 0.), (13.5, .04), (15.5, 0.)],
            "curb_8": [(3.5, .08), (6., 0.), (13.5, .08)],
            "stairs_3x10": [(2.15, .1), (2.45, .2), (2.75, .3), (3.6, .3), (4.55, .3), (4.85, .2), (5.15, .1), (5.45, 0.)],
            "course": [(7., .04), (11., .175), (14., .35), (17., .175), (0., 0.), (19.5, 0.)],
        }
        for name, points in expected.items():
            with temporary_scene(terrain=name) as scene:
                model = mujoco.MjModel.from_xml_path(str(scene))
                data = mujoco.MjData(model)
                # park the robot away from the course before casting rays
                data.qpos[:3] = [0., 50., 5.]
                mujoco.mj_forward(model, data)
                for x, z in points:
                    for sign in (1, -1):
                        geomid = np.zeros(1, dtype=np.int32)
                        dist = mujoco.mj_ray(model, data, np.array([sign * x, .15, 5.]), np.array([0., 0., -1.]), None, 1, -1, geomid)
                        self.assertAlmostEqual(5. - dist, z, places=3, msg=f"{name} x={sign * x}")
            with temporary_scene(terrain=name) as scene:
                model = mujoco.MjModel.from_xml_path(str(scene))
                # the uneven fields of the combined course stay within their tile style
                if name == "course":
                    tops = [model.geom_pos[g][2] + model.geom_size[g][2] for g in range(model.ngeom)
                            if model.geom(g).name.startswith("field_mild_")]
                    self.assertLessEqual(max(tops), .05 + 1e-9)
                    self.assertGreater(max(tops), .03)


if __name__ == "__main__":
    unittest.main()
