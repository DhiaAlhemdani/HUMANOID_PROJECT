#!/usr/bin/env python3
"""Aligned foot trajectories around every touchdown of a run: what the foot and the plan actually
did in the last 80 ms before the measured stop (roadmap 2.C).

usage: touchdown_trace.py <case dir | prefix> [more cases ...]

For each planned touchdown in the walk window (same detection as plot_tracking.py: plan wrench fz
0 -> >1 N, foot "stopped" at the first tick after its descent-speed minimum with vz >= -5 mm/s)
the measured and planned contact-frame height and vertical velocity are resampled onto a grid
relative to the stop, and the median over events is printed at 5 ms steps, plus:
  decel_onset   ms before the stop at which the measured vz first departs from the planned vz by
                more than 0.05 m/s (and stays), and the foot's height above its resting (stance)
                height at that instant: ~1-2 mm = the foot was already touching the ground
                (geometry / compliance), >5 mm = the controller slowed it in the air.
  t_e           planned event relative to the stop (= early_ms).
Writes <case>/touchdown_trace.png (all events thin, median thick) and prints one table per case so
the numbers can be pasted back without the figure.
"""
import os
import sys

import numpy as np

GRID_MS = np.arange(-80, 21, 1)          # 1 ms grid, [-80, +20] ms around the stop
PRINT_MS = list(range(-80, 21, 5))
TD_WIN = 0.06


def load(P):
    OUT = (lambda n: os.path.join(P, n)) if os.path.isdir(P) else (lambda n: P + "_" + n)
    d = np.load(OUT("data.npz"), allow_pickle=False)
    names = list(d["track_names"]); T = d["track"]
    col = {n: i for i, n in enumerate(names)}
    nan = np.full(len(T), np.nan)
    c = lambda n: T[:, col[n]] if n in col else nan
    t = c("t")
    walk = c("policy_valid") > 0
    if "phases" in d.files and "gt" in d.files and d["gt"].size:
        SETTLE, RAMP, HOLD, STOP, VX = d["phases"]; gt = d["gt"]
        w2s = lambda w: float(np.interp(w, gt[:, 1], gt[:, 0]))
        w = (t >= w2s(SETTLE + RAMP + 3.0)) & (t <= w2s(SETTLE + RAMP + HOLD)) & walk
        if w.sum() >= 200:
            walk = w
    feet = sorted({n.split("/")[1] for n in names if n.startswith("plan_foot/")})
    return OUT, t, c, walk, feet


def events(t, c, walk, feet):
    """[(foot, i_event, i_stop)] with the plot_tracking.py detection."""
    out = []
    for f in feet:
        fz = np.nan_to_num(c(f"plan_wrench/{f}/fz"), nan=0.0) > 1.0
        vz = c(f"obs_foot/{f}/vz")
        for k in np.flatnonzero(np.diff(fz.astype(int)) > 0) + 1:
            if not walk[k]:
                continue
            look = np.flatnonzero((t >= t[k] - 0.15) & (t <= t[k] + TD_WIN))
            if not len(look) or not np.isfinite(vz[look]).all():
                continue
            i_min = look[np.argmin(vz[look])]
            after = look[look >= i_min]
            stopped = after[vz[after] >= -5e-3]
            if len(stopped):
                out.append((f, k, stopped[0]))
    return out


def trace(P):
    OUT, t, c, walk, feet = load(P)
    evs = events(t, c, walk, feet)
    if not evs:
        print(f"{P}: no touchdown events"); return
    rows = {k: [] for k in ("obs_z", "plan_z", "obs_vz", "plan_vz")}
    early, onset_ms, onset_mm = [], [], []
    for f, k, i_stop in evs:
        tau = 1e3 * (t - t[i_stop])
        sel = (tau >= GRID_MS[0] - 2) & (tau <= GRID_MS[-1] + 2)
        if sel.sum() < 10:
            continue
        g = {}
        for key, col in (("obs_z", f"obs_foot/{f}/z"), ("plan_z", f"plan_foot/{f}/z"),
                         ("obs_vz", f"obs_foot/{f}/vz"), ("plan_vz", f"plan_foot/{f}/vz")):
            g[key] = np.interp(GRID_MS, tau[sel], c(col)[sel])
        mid = (t >= t[k] + 0.10) & (t <= t[k] + 0.20)
        rest = float(np.median(c(f"obs_foot/{f}/z")[mid])) if mid.any() else 0.0
        g["obs_z"] = 1e3 * (g["obs_z"] - rest); g["plan_z"] = 1e3 * (g["plan_z"] - rest)
        for key in rows:
            rows[key].append(g[key])
        early.append(1e3 * (t[k] - t[i_stop]))
        # deceleration onset: last grid time before the stop where |obs - plan| vz <= 0.05 m/s,
        # scanning backwards from the stop (so it is the start of the final departure, not a blip)
        dv = np.abs(g["obs_vz"] - g["plan_vz"])
        pre = GRID_MS <= 0
        bad = dv > 0.05
        j = None
        for i in np.flatnonzero(pre)[::-1]:
            if not bad[i]:
                break
            j = i
        if j is not None and GRID_MS[j] > GRID_MS[0]:
            onset_ms.append(-GRID_MS[j]); onset_mm.append(g["obs_z"][j])
    M = {k: np.array(v) for k, v in rows.items()}
    med = {k: np.median(v, axis=0) for k, v in M.items()}
    q = lambda v: (np.percentile(v, 25), np.percentile(v, 75)) if len(v) else (np.nan, np.nan)
    name = P.rstrip("/").split("/")[-1]
    print(f"=== {name}: {len(early)} touchdowns, t_e - stop = {np.median(early):.0f} ms "
          f"(p25/p75 {q(early)[0]:.0f}/{q(early)[1]:.0f}); heights relative to the foot's resting height")
    if onset_ms:
        print(f"    decel_onset: {np.median(onset_ms):.0f} ms before the stop (p25/p75 {q(onset_ms)[0]:.0f}/{q(onset_ms)[1]:.0f}), "
              f"foot {np.median(onset_mm):.1f} mm above rest (p25/p75 {q(onset_mm)[0]:.1f}/{q(onset_mm)[1]:.1f}); "
              f"{len(onset_ms)}/{len(early)} events had one inside the window")
    else:
        print("    decel_onset: none (measured vz stayed within 0.05 m/s of the plan up to the stop)")
    print("    ms   obs_z   plan_z  obs_vz  plan_vz   [mm, mm, m/s, m/s]  (0 = measured stop)")
    for ms in PRINT_MS:
        i = int(np.flatnonzero(GRID_MS == ms)[0])
        print(f"  {ms:4d}  {med['obs_z'][i]:6.2f}  {med['plan_z'][i]:6.2f}  {med['obs_vz'][i]:6.3f}  {med['plan_vz'][i]:6.3f}")
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(2, 1, figsize=(8, 7), sharex=True)
        for key, a, style in (("obs_z", ax[0], "C0"), ("plan_z", ax[0], "C1"), ("obs_vz", ax[1], "C0"), ("plan_vz", ax[1], "C1")):
            for row in M[key]:
                a.plot(GRID_MS, row, style, lw=0.4, alpha=0.25)
            a.plot(GRID_MS, med[key], style, lw=2, label=key)
        for a in ax:
            a.axvline(0, color="k", lw=0.8); a.axvline(np.median(early), color="r", lw=0.8, ls="--")
            a.grid(alpha=0.3); a.legend(loc="upper right")
        ax[0].set_ylabel("height above rest [mm]"); ax[0].set_ylim(-3, 25)
        ax[1].set_ylabel("vz [m/s]"); ax[1].set_xlabel("ms relative to the measured stop (red dashed: planned event)")
        ax[0].set_title(f"{name}: {len(early)} touchdowns, median thick")
        fig.tight_layout(); fig.savefig(OUT("touchdown_trace.png"), dpi=110); plt.close(fig)
    except Exception as e:  # noqa: BLE001 - the table is the deliverable; the figure is a bonus
        print(f"    (no figure: {e})")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__); sys.exit(1)
    for P in sys.argv[1:]:
        trace(P)
