"""Phase 1.4 contract: the sole-model variant tool and the scene hook behave as documented.

Pure stdlib + ElementTree; MuJoCo compile checks are skipped when `mujoco` is absent.
"""

import importlib.util
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
G1 = ROOT / "description/mujoco/robots/g1/g1_29dof.xml"
MAKE_MODEL = ROOT / "tests/tuning/make_model.py"
sys.path.insert(0, str(ROOT / "tests"))
from mujoco_scene import write_scene  # noqa: E402

spec = importlib.util.spec_from_file_location("make_model", MAKE_MODEL)
make_model = importlib.util.module_from_spec(spec)
spec.loader.exec_module(make_model)

try:
    import mujoco
except ImportError:
    mujoco = None


def sole_geoms(xml_path: Path, side: str):
    root = ET.parse(xml_path).getroot()
    body = root.find(f".//body[@name='{side}_ankle_roll_link']")
    return [g for g in body.findall("geom") if g.get("type") != "mesh"]


class ShippedSoleFacts(unittest.TestCase):
    """The recon the sweep cases are built on; fails loudly if the model is edited."""

    def test_four_5mm_spheres_per_foot_at_documented_positions(self):
        for side in ("left", "right"):
            geoms = sole_geoms(G1, side)
            self.assertEqual(len(geoms), 4, side)
            pos = sorted(tuple(float(v) for v in g.get("pos").split()) for g in geoms)
            self.assertEqual(pos, [(-0.05, -0.025, -0.03), (-0.05, 0.025, -0.03),
                                   (0.12, -0.03, -0.03), (0.12, 0.03, -0.03)])
            self.assertTrue(all(g.get("size") == "0.005" for g in geoms))

    def test_floor_priority_makes_robot_solref_inert(self):
        floor = ET.parse(ROOT / "description/mujoco/common/backgrounds/background_day.xml").getroot()
        geom = floor.find(".//geom[@name='floor']")
        self.assertEqual(geom.get("priority"), "1")
        self.assertIsNone(geom.get("solref"))  # -> MuJoCo default (0.02, 1) governs sole-floor contact


class MakeModelBox(unittest.TestCase):
    def convert(self, *args):
        out = Path(self.tmp.name) / "g1_variant.xml"
        subprocess.run([sys.executable, str(MAKE_MODEL), str(out), "--sole", "box", *args],
                       check=True, capture_output=True, text=True)
        return out

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()

    def tearDown(self):
        self.tmp.cleanup()

    def test_sphere_passthrough_is_byte_identical(self):
        out = Path(self.tmp.name) / "same.xml"
        subprocess.run([sys.executable, str(MAKE_MODEL), str(out), "--sole", "spheres"], check=True)
        self.assertEqual(out.read_text(), G1.read_text())

    def test_box_hull_spheres_matches_footprint_and_height(self):
        out = self.convert()
        for side in ("left", "right"):
            geoms = sole_geoms(out, side)
            self.assertEqual(len(geoms), 1)
            box = geoms[0]
            self.assertEqual(box.get("type"), "box")
            self.assertEqual(box.get("name"), f"{side}_sole")
            hx, hy, hz = (float(v) for v in box.get("size").split())
            cx, cy, cz = (float(v) for v in box.get("pos").split())
            self.assertAlmostEqual(cx - hx, -0.05)
            self.assertAlmostEqual(cx + hx, 0.12)
            self.assertAlmostEqual(hy, 0.03)
            self.assertAlmostEqual(cy, 0.0)
            self.assertAlmostEqual(cz - hz, -0.035)  # same ground height as the sphere bottoms
        # Everything else is verbatim: only the 8 sphere lines went away, 2 box lines came in.
        before, after = G1.read_text().splitlines(), out.read_text().splitlines()
        self.assertEqual(len(before) - len(after), 6)
        self.assertEqual([l for l in before if 'size="0.005"' not in l],
                         [l for l in after if "_sole" not in l])

    def test_box_hull_rectangle_matches_mpc_rectangle(self):
        out = self.convert("--hull", "rectangle")
        box = sole_geoms(out, "left")[0]
        hx, hy, _ = (float(v) for v in box.get("size").split())
        cx, _, _ = (float(v) for v in box.get("pos").split())
        # contactGeometry: frameTranslation x 0.035, rectangle [0.09, -0.09, 0.03, -0.03]
        self.assertAlmostEqual(cx - hx, 0.035 - 0.09)
        self.assertAlmostEqual(cx + hx, 0.035 + 0.09)
        self.assertAlmostEqual(hy, 0.03)

    def test_explicit_box_and_validation(self):
        out = self.convert("--box", "-0.04", "0.10", "-0.02", "0.02", "--thickness", "0.004")
        box = sole_geoms(out, "right")[0]
        self.assertEqual(box.get("size"), "0.07 0.02 0.002")
        self.assertEqual(box.get("pos"), "0.03 0 -0.033")
        with self.assertRaises(subprocess.CalledProcessError):
            self.convert("--box", "0.1", "-0.1", "0", "0.01")

    def test_convert_refuses_unexpected_layout(self):
        text = G1.read_text().replace('pos="0.12 0.03 -0.03"', 'pos="0.12 0.03 -0.02"', 1)
        with self.assertRaises(ValueError):
            make_model.convert(text, make_model.HULLS["spheres"], 0.005)

    def test_scene_hook_swaps_only_the_robot_include(self):
        variant = self.convert()
        default_scene = write_scene(Path(self.tmp.name) / "default.xml")
        variant_scene = write_scene(Path(self.tmp.name) / "variant.xml", robot_model=variant)
        d, v = ET.parse(default_scene).getroot(), ET.parse(variant_scene).getroot()
        d_inc = [e.get("file") for e in d.findall("include")]
        v_inc = [e.get("file") for e in v.findall("include")]
        self.assertEqual(len(d_inc), len(v_inc))
        self.assertEqual(d_inc[:-1], v_inc[:-1])
        self.assertEqual(Path(d_inc[-1]), G1.resolve())
        self.assertEqual(Path(v_inc[-1]), variant.resolve())
        self.assertEqual(d.find("compiler").attrib, v.find("compiler").attrib)  # meshdir still absolute
        with self.assertRaises(FileNotFoundError):
            write_scene(Path(self.tmp.name) / "missing.xml", robot_model=Path(self.tmp.name) / "nope.xml")

    @unittest.skipIf(mujoco is None, "mujoco is not installed")
    def test_variant_compiles_and_rests_at_the_same_height(self):
        variant = self.convert()
        scene = write_scene(Path(self.tmp.name) / "scene.xml", robot_model=variant)
        model = mujoco.MjModel.from_xml_path(str(scene))
        for side in ("left", "right"):
            gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, f"{side}_sole")
            self.assertGreaterEqual(gid, 0)
            self.assertEqual(model.geom_type[gid], mujoco.mjtGeom.mjGEOM_BOX)
            self.assertAlmostEqual(float(model.geom_pos[gid][2] - model.geom_size[gid][2]), -0.035)


if __name__ == "__main__":
    unittest.main()
