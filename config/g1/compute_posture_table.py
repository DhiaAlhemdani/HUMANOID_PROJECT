#!/usr/bin/env python3
"""Symmetric flat-foot G1 leg postures as a function of pelvis height; prints the YAML for
ocs2.reference.postureHeights / postureJointStates, or writes a candidate controller config with them.

    pip install mujoco numpy          # offline tool; not needed at run time
    python3 config/g1/compute_posture_table.py [--min 0.60] [--max 0.79] [--step 0.01]
    python3 config/g1/compute_posture_table.py --write-config tuning_results/humanPT.yaml [--in config/g1/ros2_controllers.yaml]

For each pelvis height h the sole is flat on the floor and the leg is the symmetric bend hip = ankle = -k/2,
knee = k (the hip-pitch-to-knee segment and the knee-to-ankle segment tilt by the same angle fore and aft,
so the ankle stays under the hip and the sole stays level: the G1's hip-roll and knee body rotations cancel).
k is solved by bisection on the real MJCF kinematics (sole height = ankle-roll link height minus its
standing value 0.0361 m; straight leg = 0.793 m). Hip roll / yaw, ankle roll, waist and arms keep the
defaultJointState values. Right leg = left leg (the sagittal joints are not mirrored on the G1).

Why: the shipped defaultJointState is a 0.60 m crouch (hip -0.8, knee 1.6, ankle -0.8) while the height
reference is 0.785 m, so the posture cost (knee weight 0.1) and the height cost (weight 50) pull against
each other at every height and the compromise moves with the commanded height. With a posture table the
joint reference is the posture of the commanded height (TargetTrajectoriesCalculatorBase::getPostureForHeight),
and the only height-dependent term left in the reference is the height itself.
"""

import argparse
import pathlib
import re
import sys

HERE = pathlib.Path(__file__).resolve().parent
MJCF = HERE / ".." / ".." / "description" / "mujoco" / "robots" / "g1" / "g1_29dof.xml"
DEFAULT_CONFIG = HERE / "ros2_controllers.yaml"
WB = "humanoid_whole_body_inverse_dynamics_mpc_controller"
# MPC joint order (robot.jointNames in the G1 config): legs L/R [hp hr hy kn ap ar], waist [y r p], arms L/R [sp sr sy el].
LEG_SAGITTAL = {0: "hip", 3: "knee", 4: "ankle", 6: "hip", 9: "knee", 10: "ankle"}


class Model:
    def __init__(self):
        import mujoco  # noqa: PLC0415

        self.mujoco = mujoco
        self.m = mujoco.MjModel.from_xml_path(str(MJCF))
        self.d = mujoco.MjData(self.m)
        self.q = {n: self.m.joint(n).qposadr[0] for n in ("left_hip_pitch_joint", "left_knee_joint", "left_ankle_pitch_joint")}
        self.ankle_roll = self.m.body("left_ankle_roll_link").id
        self.standing_pelvis = 0.793
        self.sole_offset = self._ankle_roll_z(0.0) - 0.0  # ankle-roll height at a straight leg, pelvis at 0.793

    def _ankle_roll_z(self, knee):
        d = self.d
        d.qpos[:] = 0.0
        d.qpos[2] = self.standing_pelvis
        d.qpos[3] = 1.0
        d.qpos[self.q["left_hip_pitch_joint"]] = -0.5 * knee
        d.qpos[self.q["left_knee_joint"]] = knee
        d.qpos[self.q["left_ankle_pitch_joint"]] = -0.5 * knee
        self.mujoco.mj_kinematics(self.m, d)
        return float(d.xpos[self.ankle_roll][2])

    def pelvis_height(self, knee):
        """Pelvis height with the sole on the floor at this symmetric knee bend."""
        return self.standing_pelvis - (self._ankle_roll_z(knee) - self.sole_offset)

    def knee_for_height(self, height):
        lo, hi = 0.0, 2.6
        if height > self.pelvis_height(lo) + 1e-9 or height < self.pelvis_height(hi):
            raise ValueError(f"height {height:.3f} m outside the symmetric posture range "
                             f"[{self.pelvis_height(hi):.3f}, {self.pelvis_height(lo):.3f}]")
        for _ in range(60):
            mid = 0.5 * (lo + hi)
            if self.pelvis_height(mid) > height:
                lo = mid
            else:
                hi = mid
        return 0.5 * (lo + hi)


def read_default_joint_state(text):
    i = text.index("defaultJointState:")
    j = text.index("]", i)
    body = text[text.index("[", i) + 1:j]
    values = [float(v) for v in re.split(r"[\s,]+", body.strip()) if v]
    if len(values) != 23:
        raise ValueError(f"defaultJointState has {len(values)} entries, expected 23")
    return values


def posture(default, knee):
    q = list(default)
    for idx, part in LEG_SAGITTAL.items():
        q[idx] = knee if part == "knee" else -0.5 * knee
    return q


def table(model, default, heights):
    return [(h, posture(default, model.knee_for_height(h))) for h in heights]


def yaml_block(rows, indent="        "):
    lines = [f"{indent}postureHeights:",
             f"{indent}  [" + ", ".join(f"{h:.3f}" for h, _ in rows) + "]",
             f"{indent}postureJointStates:"]
    for n, (_, q) in enumerate(rows):
        # every entry a double literal: rcl YAML rejects a list mixing ints and doubles
        row = ", ".join(f"{v:.6f}" if i in LEG_SAGITTAL else repr(float(v)) for i, v in enumerate(q))
        lines.append(f"{indent}  {'[' if n == 0 else ' '}{row}{']' if n == len(rows) - 1 else ','}")
    return "\n".join(lines)


def write_config(src, dst, rows, min_h, max_h):
    text = src.read_text()
    if "postureHeights:" in text:
        raise ValueError(f"{src} already has a posture table")
    block = ("\n        # Posture reference by commanded height (config/g1/compute_posture_table.py): the symmetric flat-foot\n"
             "        # leg bend of each height, so the posture cost and the height cost agree. Covers [minBaseHeight,\n"
             "        # maxBaseHeight]; regenerate before widening that range.\n" + yaml_block(rows) + "\n")
    i = text.index("defaultJointState:")
    j = text.index("]", i) + 1
    j = text.index("\n", j) + 1
    text = text[:j] + block + text[j:]
    text = re.sub(r"(\n\s*minBaseHeight:\s*)[0-9.]+", rf"\g<1>{min_h:.3f}", text, count=1)
    text = re.sub(r"(\n\s*maxBaseHeight:\s*)[0-9.]+", rf"\g<1>{max_h:.3f}", text, count=1)
    dst.parent.mkdir(parents=True, exist_ok=True)
    dst.write_text(text)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--min", type=float, default=0.60, help="lowest table height [m] (the shipped crouch posture is 0.600)")
    ap.add_argument("--max", type=float, default=0.79, help="highest table height [m] (straight leg 0.793; keep a knee margin)")
    ap.add_argument("--step", type=float, default=0.01)
    ap.add_argument("--in", dest="src", type=pathlib.Path, default=DEFAULT_CONFIG, help="controller YAML to read defaultJointState from")
    ap.add_argument("--write-config", type=pathlib.Path, help="write <src> with the table inserted and min/maxBaseHeight set to the table ends")
    a = ap.parse_args()
    model = Model()
    default = read_default_joint_state(a.src.read_text())
    n = int((a.max - a.min) / a.step + 1e-9)
    heights = [round(a.min + i * a.step, 3) for i in range(n + 1)]
    if heights[-1] < a.max - 1e-9:
        heights.append(round(a.max, 3))  # the top end is always in the table, so maxBaseHeight is covered
    rows = table(model, default, heights)
    if a.write_config:
        write_config(a.src, a.write_config, rows, heights[0], heights[-1])
        print(f"wrote {a.write_config} ({len(rows)} heights {heights[0]:.3f}-{heights[-1]:.3f} m, "
              f"knee {rows[-1][1][3]:.3f}-{rows[0][1][3]:.3f} rad)")
    else:
        print(yaml_block(rows))
    print("# height -> knee [rad] (hip = ankle = -knee/2):", ", ".join(f"{h:.2f}:{q[3]:.3f}" for h, q in rows), file=sys.stderr)


if __name__ == "__main__":
    main()
