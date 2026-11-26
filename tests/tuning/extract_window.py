#!/usr/bin/env python3
"""Cut a short window out of a case's data.npz (tracking stream + ground truth) into a small, shareable npz.

usage: extract_window.py <case_dir> --at T_SIM [--before 1.5] [--after 0.3] [--out <case>/window_<T>.npz]

The output keeps `track_names`, the `track` rows with t in [T-before, T+after], the `gt` rows in the same
sim-time range, `js_names`, and a `meta` string with the case name and the window. Typically ~1 MB
for a 2 s window at 500 Hz, against ~50 MB for the full run.
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
out = a.out or os.path.join(case_dir, f"window_{a.at:.2f}.npz")
np.savez_compressed(out, track_names=np.array(TN), track=TR[w].astype(np.float32), gt=g,
                    js_names=d["js_names"] if "js_names" in d.files else np.array([]),
                    fell=d["fell"] if "fell" in d.files else np.array(np.nan),
                    meta=np.array(f"{os.path.basename(case_dir.rstrip('/'))} window {lo:.2f}..{hi:.2f} s sim"))
print(out, f"{w.sum()} track rows, {len(g)} gt rows, {os.path.getsize(out) / 1e6:.1f} MB")
