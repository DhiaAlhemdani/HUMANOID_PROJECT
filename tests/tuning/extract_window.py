#!/usr/bin/env python3
"""Cut a short window out of a case's data.npz (tracking stream + ground truth) into a small, shareable file.

usage: extract_window.py <case_dir> --at T_SIM [--before 1.5] [--after 0.3] [--format csv|npz] [--out PATH]

csv (default): one row per tracking sample with t in [T-before, T+after]; the columns are the tracking
names minus the velocity groups (cmd_qd, meas_v, plan_v) plus `gt/z`, `gt/roll`, `gt/pitch` (ground truth
interpolated to the sample times). ~1.5 MB for a 1.8 s window. npz: the same plus the raw gt rows.
"""
from __future__ import annotations

import argparse
import os

import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("case")
ap.add_argument("--at", type=float, required=True)
ap.add_argument("--before", type=float, default=1.5)
ap.add_argument("--after", type=float, default=0.3)
ap.add_argument("--out", default=None)
ap.add_argument("--format", choices=("csv", "npz"), default="csv")
a = ap.parse_args()

npz = os.path.join(a.case, "data.npz") if os.path.isdir(a.case) else a.case
case_dir = a.case if os.path.isdir(a.case) else os.path.dirname(a.case)
d = np.load(npz, allow_pickle=False)
TN = list(d["track_names"])
TR = d["track"]
t = TR[:, TN.index("t")]
lo, hi = a.at - a.before, a.at + a.after
w = (t >= lo) & (t <= hi)
gt = d["gt"]
g = gt[(gt[:, 0] >= lo - 0.05) & (gt[:, 0] <= hi + 0.05)]
tag = os.path.basename(case_dir.rstrip("/"))
if a.format == "csv":
    out = a.out or os.path.join(case_dir, f"window_{a.at:.2f}.csv")
    keep = [i for i, n in enumerate(TN) if not n.startswith(("cmd_qd/", "meas_v/", "plan_v/"))]
    tw = t[w]
    gcols = [np.interp(tw, gt[:, 0], gt[:, c]) for c in (4, 8, 9)]
    data = np.column_stack([TR[w][:, keep]] + gcols)
    header = ",".join([TN[i] for i in keep] + ["gt/z", "gt/roll", "gt/pitch"])
    with open(out, "w") as fh:
        fh.write(f"# {tag} window {lo:.2f}..{hi:.2f} s sim; fell={float(d['fell']) if 'fell' in d.files else 'nan'}\n")
        np.savetxt(fh, data, fmt="%.5g", delimiter=",", header=header, comments="")
    print(out, f"{w.sum()} rows x {data.shape[1]} cols, {os.path.getsize(out) / 1e6:.1f} MB")
    raise SystemExit
out = a.out or os.path.join(case_dir, f"window_{a.at:.2f}.npz")
np.savez_compressed(out, track_names=np.array(TN), track=TR[w].astype(np.float32), gt=g,
                    js_names=d["js_names"] if "js_names" in d.files else np.array([]),
                    fell=d["fell"] if "fell" in d.files else np.array(np.nan),
                    meta=np.array(f"{tag} window {lo:.2f}..{hi:.2f} s sim"))
print(out, f"{w.sum()} track rows, {len(g)} gt rows, {os.path.getsize(out) / 1e6:.1f} MB")
