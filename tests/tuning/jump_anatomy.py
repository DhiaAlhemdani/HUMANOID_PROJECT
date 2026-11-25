#!/usr/bin/env python3
"""Anatomy of legs.cmd_tau_jump_at_switch: is a change in the mean a shift of every policy switch
or a few outliers (one bad solve)?

usage: jump_anatomy.py <case dir> [more cases ...]
Per case: number of policy switches in the walk window, the per-switch leg-mean |delta cmd_tau|
distribution (mean = the metric, median, p90, p99, max), the share of the mean carried by the top
1 % of switches, and the mean split by proximity to a planned contact change (within 40 ms / not).
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from touchdown_trace import load  # noqa: E402


def anatomy(P):
    OUT, t, c, walk, feet = load(P)
    d = np.load(OUT("data.npz"), allow_pickle=False)
    names = list(d["track_names"])
    legs = [n.split("/", 1)[1] for n in names if n.startswith("cmd_tau/") and any(k in n for k in ("hip", "knee", "ankle"))]
    t0 = c("policy_t0")
    switch = np.zeros(len(t), bool)
    switch[1:] = np.diff(t0) != 0
    switch &= walk
    dtau = np.column_stack([np.abs(np.diff(c("cmd_tau/" + j), prepend=np.nan)) for j in legs])
    per = np.nanmean(dtau[switch], axis=1)
    per = per[np.isfinite(per)]
    if not len(per):
        print(f"{P}: no switches"); return
    contact = np.zeros(len(t), bool)
    for f in feet:
        fz = np.nan_to_num(c(f"plan_wrench/{f}/fz"), nan=0.0) > 1.0
        for k in np.flatnonzero(np.diff(fz.astype(int)) != 0) + 1:
            contact |= (t >= t[k] - 0.04) & (t <= t[k] + 0.04)
    near = np.nanmean(dtau[switch & contact], axis=1); far = np.nanmean(dtau[switch & ~contact], axis=1)
    top = np.sort(per)[::-1][:max(1, len(per) // 100)]
    name = P.rstrip("/").split("/")[-1]
    print(f"=== {name}: {len(per)} policy switches | mean {per.mean():.3f} (the metric) median {np.median(per):.3f} "
          f"p90 {np.percentile(per, 90):.3f} p99 {np.percentile(per, 99):.3f} max {per.max():.2f} Nm | "
          f"top 1 % of switches carry {100 * top.sum() / per.sum():.0f} % of the mean | "
          f"within 40 ms of a planned contact change: {np.nanmean(near):.3f} (n {np.isfinite(near).sum()}), "
          f"elsewhere {np.nanmean(far):.3f} (n {np.isfinite(far).sum()})")


if __name__ == "__main__":
    for P in sys.argv[1:]:
        anatomy(P)
