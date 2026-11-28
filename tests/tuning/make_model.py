#!/usr/bin/env python3
"""Write a G1 MJCF variant with a different sole contact model (roadmap #1.4).

usage:
  make_model.py <out.xml> [--in <g1_29dof.xml>] --sole {spheres,box} [--hull {spheres,rectangle}]
                [--box XMIN XMAX YMIN YMAX] [--thickness M]

The shipped sole is four r=5 mm spheres per foot at x in {-0.05, 0.12}, y in {+-0.025 (heel),
+-0.03 (toe)}, z=-0.03 in the ankle-roll frame (bottoms at z=-0.035). `--sole box` replaces
the four spheres of each foot with ONE box whose bottom face sits at the same height:
  --hull spheres    (default) bounding box of the spheres:  x [-0.05, 0.12], y [-0.03, 0.03]
  --hull rectangle  the MPC CoP rectangle (contactGeometry.rectangle at frameTranslation x=0.035):
                    x [-0.055, 0.125], y [-0.03, 0.03]
  --box ...         explicit extents in the ankle-roll frame.
Everything else (meshes, joints, armature, solref, sensors) is kept verbatim: the edit is a
line-level text substitution and asserts exactly four sphere lines per foot were replaced.
The output is meant for a results dir and `run_case.sh` via MODEL=<out.xml>.
"""
from __future__ import annotations

import argparse
import pathlib
import re

DEFAULT_IN = pathlib.Path(__file__).resolve().parents[2] / "description/mujoco/robots/g1/g1_29dof.xml"
SPHERE = re.compile(r'^(\s*)<geom size="0\.005" pos="(-?[0-9.]+) (-?[0-9.]+) (-?[0-9.]+)" rgba="([^"]*)"/>\s*$')
HULLS = {"spheres": (-0.05, 0.12, -0.03, 0.03), "rectangle": (-0.055, 0.125, -0.03, 0.03)}
SOLE_BOTTOM_Z = -0.035  # sphere centre -0.03 minus radius 0.005


def box_line(indent: str, extents: tuple[float, float, float, float], thickness: float, rgba: str) -> str:
    xmin, xmax, ymin, ymax = extents
    cx, cy = (xmin + xmax) / 2, (ymin + ymax) / 2
    hx, hy, hz = (xmax - xmin) / 2, (ymax - ymin) / 2, thickness / 2
    cz = SOLE_BOTTOM_Z + hz
    return (f'{indent}<geom name="SIDE_sole" type="box" size="{hx:.4g} {hy:.4g} {hz:.4g}" '
            f'pos="{cx:.4g} {cy:.4g} {cz:.4g}" rgba="{rgba}"/>')


def convert(text: str, extents: tuple[float, float, float, float], thickness: float) -> str:
    """Replace each foot's four sphere lines with one box line; returns new text."""
    lines = text.splitlines(keepends=True)
    out: list[str] = []
    side = None
    replaced = {"left": 0, "right": 0}
    for line in lines:
        m_body = re.search(r'<body name="(left|right)_ankle_roll_link"', line)
        if m_body:
            side = m_body.group(1)
        m = SPHERE.match(line)
        if m and side is not None:
            z = float(m.group(4))
            if abs(z - (-0.03)) > 1e-9:
                raise ValueError(f"unexpected sole sphere height {z}: {line.strip()}")
            replaced[side] += 1
            if replaced[side] == 1:
                out.append(box_line(m.group(1), extents, thickness, m.group(5)).replace("SIDE", side) + "\n")
            continue
        out.append(line)
    if replaced != {"left": 4, "right": 4}:
        raise ValueError(f"expected 4 sole spheres per foot, found {replaced} — model layout changed?")
    return "".join(out)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("out")
    ap.add_argument("--in", dest="inp", default=str(DEFAULT_IN))
    ap.add_argument("--sole", choices=("spheres", "box"), required=True)
    ap.add_argument("--hull", choices=tuple(HULLS), default="spheres")
    ap.add_argument("--box", type=float, nargs=4, metavar=("XMIN", "XMAX", "YMIN", "YMAX"))
    ap.add_argument("--thickness", type=float, default=0.005, help="box height [m] (default 5 mm)")
    args = ap.parse_args()
    text = pathlib.Path(args.inp).read_text()
    if args.sole == "box":
        extents = tuple(args.box) if args.box else HULLS[args.hull]
        if not (extents[0] < extents[1] and extents[2] < extents[3]) or args.thickness <= 0:
            raise SystemExit("box extents must be ordered and thickness positive")
        text = convert(text, extents, args.thickness)
    pathlib.Path(args.out).write_text(text)
    print(args.out)


if __name__ == "__main__":
    main()
