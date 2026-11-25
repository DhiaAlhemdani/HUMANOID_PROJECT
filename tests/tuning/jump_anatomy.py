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
    # Switches that land on the tick of a planned contact change count the plan's own switch step
    # (plot_tracking excludes them since the 2.C-timing close-out); report the mean without them.
    if "plan_mode" in names:
        _pm = np.nan_to_num(c("plan_mode"), nan=-1.0)
        on_event = np.zeros(len(t), bool); on_event[1:] = np.diff(_pm) != 0
        clean = np.nanmean(dtau[switch & ~on_event], axis=1); clean = clean[np.isfinite(clean)]
        cleanstr = f" | excluding switches on a contact-change tick ({int((switch & on_event).sum())}): mean {clean.mean():.3f} max {clean.max():.2f}"
    else:
        cleanstr = ""
    top = np.sort(per)[::-1][:max(1, len(per) // 100)]
    name = P.rstrip("/").split("/")[-1]
    # The largest jumps, one line each: when, which joint, how far from the nearest planned contact
    # change, and whether the re-solve flipped the planned contact mode at that very tick (the new
    # policy puts "now" on the other side of a contact event the old one had elsewhere).
    changes = []
    for f in feet:
        fz = np.nan_to_num(c(f"plan_wrench/{f}/fz"), nan=0.0) > 1.0
        for k in np.flatnonzero(np.diff(fz.astype(int)) != 0) + 1:
            changes.append((t[k], f, "td" if fz[k] else "lo"))
    changes.sort()
    ct = np.array([x[0] for x in changes]) if changes else np.array([np.inf])
    mode = c("plan_mode") if "plan_mode" in names else None
    sw_idx = np.flatnonzero(switch)
    vals = np.nanmean(dtau[sw_idx], axis=1)
    order = np.argsort(np.nan_to_num(vals, nan=-1))[::-1][:8]
    lines = []
    for o in order:
        i = sw_idx[o]
        jw = int(np.nanargmax(dtau[i]))
        n = int(np.argmin(np.abs(ct - t[i]))) if changes else -1
        where = f"{1e3 * (t[i] - changes[n][0]):+5.0f} ms from {changes[n][2]} {changes[n][1]}" if changes else "-"
        flip = "" if mode is None else (f" mode {int(mode[i - 1])}->{int(mode[i])}" if mode[i] != mode[i - 1] else f" mode {int(mode[i])}")
        lines.append(f"      t={t[i]:7.3f}  {vals[o]:6.2f} Nm  {legs[jw].replace('_joint', ''):18s} {dtau[i, jw]:6.1f} Nm  {where}{flip}")
    print(f"=== {name}: {len(per)} policy switches | mean {per.mean():.3f} (the metric) median {np.median(per):.3f} "
          f"p90 {np.percentile(per, 90):.3f} p99 {np.percentile(per, 99):.3f} max {per.max():.2f} Nm | "
          f"top 1 % of switches carry {100 * top.sum() / per.sum():.0f} % of the mean | "
          f"within 40 ms of a planned contact change: {np.nanmean(near):.3f} (n {np.isfinite(near).sum()}), "
          f"elsewhere {np.nanmean(far):.3f} (n {np.isfinite(far).sum()}){cleanstr}")
    print("    largest jumps: t | leg-mean | worst joint | distance to nearest planned contact change (td/lo) | plan mode at the tick")
    print("\n".join(lines))
    # Tick-by-tick trace of the largest one: which path steps -- the plan's torque (plan_tau), the
    # written command (cmd_tau), the contact-switch fade (wbc/contact_alpha) or the policy swap
    # (policy_t0) -- over +-12 ms, for the worst joint.
    if len(order) and np.isfinite(vals[order[0]]) and vals[order[0]] > 1.0:
        i = sw_idx[order[0]]; jw = int(np.nanargmax(dtau[i])); j = legs[jw]
        win = np.flatnonzero((t >= t[i] - 0.012) & (t <= t[i] + 0.012))
        alpha = c("wbc/contact_alpha") if "wbc/contact_alpha" in names else np.full(len(t), np.nan)
        ptau = c("plan_tau/" + j) if ("plan_tau/" + j) in names else np.full(len(t), np.nan)
        mtau = c("meas_tau/" + j) if ("meas_tau/" + j) in names else np.full(len(t), np.nan)
        print(f"    trace of the largest ({j.replace('_joint', '')}): ms | cmd_tau | plan_tau | meas_tau | fade alpha | plan_mode | policy_t0")
        for k in win:
            print(f"      {1e3 * (t[k] - t[i]):+5.0f}  {c('cmd_tau/' + j)[k]:7.1f}  {ptau[k]:7.1f}  {mtau[k]:7.1f}  {alpha[k]:5.2f}  "
                  f"{'' if mode is None else int(mode[k]):>3}  {t0[k]:.3f}")


if __name__ == "__main__":
    for P in sys.argv[1:]:
        anatomy(P)
