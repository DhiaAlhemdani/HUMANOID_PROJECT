#!/usr/bin/env python3
"""Where in the stride does the pelvis jerk come from?

usage: jerk_attribution.py <case_dir> [<case_dir> ...] [--min-vx 0.4] [--win 0.04] [--bins 10]

For every steady hold of the script (same segments as score_suite.py) with |vx| >= --min-vx, the
0-15 Hz pelvis jerk magnitude is resampled at 500 Hz and attributed to events of the recorded plan
and the measured feet:
  td        measured touchdowns (obs_foot z crosses below 2 mm while the plan says that foot swings)
  sw_td     the planned contact switches into double support (LF/RF -> STANCE)
  sw_lo     the planned lift-offs (STANCE -> LF/RF)
For each event class the share of the segment's jerk energy that lies within +-win of the events is
compared with the share of time those windows cover: `conc` = energy share / time share (1 = the
jerk is spread evenly over the stride, 3 = three times denser around the event). `lead_ms` is the
measured touchdown minus the planned switch (positive = the foot is down before the plan says so).
The last columns bin the jerk energy over the stride phase from one measured touchdown to the next
(either foot), so the profile is readable without a plot: bin 0 is the touchdown itself.
"""
from __future__ import annotations

import argparse
import os
import sys

import numpy as np

ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument("cases", nargs="+")
ap.add_argument("--min-vx", type=float, default=0.4)
ap.add_argument("--win", type=float, default=0.04, help="half window around an event [s]")
ap.add_argument("--bins", type=int, default=10)
a = ap.parse_args()
FS = 500.0


def band_series(tt, x, lo=0.0, hi=15.0):
    """0-15 Hz component of x on a uniform FS grid (same filter as score_suite.band, series not RMS)."""
    tu = np.arange(tt[0], tt[-1], 1.0 / FS)
    xu = np.interp(tu, tt, x)
    xu = xu - np.polyval(np.polyfit(tu - tu[0], xu, 1), tu - tu[0])
    f = np.fft.rfftfreq(len(xu), 1.0 / FS)
    X = np.fft.rfft(xu)
    X[(f < lo) | (f > hi)] = 0
    return tu, np.fft.irfft(X, len(xu))


def side(name):
    n = name.lower()
    return "l" if ("left" in n or "_l_" in n or n.endswith("_l") or n.startswith("l_")) else "r"


def share(tu, e, events, win):
    """(energy share, time share) of the +-win windows around `events` in the series e(tu)."""
    if not len(events):
        return np.nan, np.nan
    m = np.zeros(len(tu), dtype=bool)
    for te in events:
        m |= np.abs(tu - te) <= win
    tot = e.sum()
    return (e[m].sum() / tot if tot > 0 else np.nan), m.mean()


hdr = (f"{'case':22s} {'segment':14s} {'n_td':>4s} {'lead_ms':>12s} {'jerk':>6s} | {'td':>11s} {'sw_td':>11s} {'sw_lo':>11s} | "
       f"phase bins (energy %)")
print(hdr)
print(f"{'':22s} {'':14s} {'':4s} {'mean sd':>12s} {'':6s} | {'E% conc':>11s} {'E% conc':>11s} {'E% conc':>11s} |")
for case in a.cases:
    npz = os.path.join(case, "data.npz") if os.path.isdir(case) else case
    tag = os.path.basename(case.rstrip("/"))
    d = np.load(npz, allow_pickle=False)
    TN = list(d["track_names"]) if "track_names" in d.files else []
    if not TN or not d["track"].size:
        print(f"{tag}: no tracking stream", file=sys.stderr)
        continue
    TR = d["track"]
    col = {n: i for i, n in enumerate(TN)}
    t = TR[:, col["t"]]
    o = np.argsort(t)
    TR, t = TR[o], t[o]
    keep = np.concatenate([[True], np.diff(t) > 1e-6])
    TR, t = TR[keep], t[keep]
    sig = lambda n: TR[:, col[n]] if n in col else None
    gt = d["gt"]
    gt = gt[np.argsort(gt[:, 0])]
    gt = gt[np.concatenate([[True], np.diff(gt[:, 0]) > 1e-6])]
    SETTLE = float(d["phases"][0])
    w2s = lambda w: float(np.interp(w, gt[:, 1], gt[:, 0]))
    K = d["script"] if "script" in d.files and d["script"].size else None
    if K is None:
        segs = [dict(t0=gt[0, 0], t1=gt[-1, 0], label="whole run", vx=np.nan)]
    else:
        segs = []
        for p, q in zip(K[:-1], K[1:]):
            if np.allclose(p[1:], q[1:]) and q[0] - p[0] >= 2.5 and abs(p[1]) >= a.min_vx:
                segs.append(dict(t0=w2s(SETTLE + p[0] + 1.5), t1=w2s(SETTLE + q[0]), vx=p[1],
                                 label=f"hold vx {p[1]:+.1f} w {p[3]:+.1f}"))
    mode = sig("plan_mode")
    mode = np.where(np.isfinite(mode), mode, -1.0) if mode is not None else None
    feet = sorted({n.split("/")[1] for n in TN if n.startswith("obs_foot/")})
    by_side = {side(f): f for f in feet}
    swinging = {1: "l", 2: "r"}  # RF (1) = right in contact, left swings

    for s in segs:
        g = gt[(gt[:, 0] >= s["t0"]) & (gt[:, 0] <= s["t1"])]
        if len(g) < 100:
            continue
        tt = g[:, 0]
        v = np.column_stack([np.gradient(g[:, k], tt) for k in (2, 3, 4)])
        acc = np.column_stack([np.gradient(v[:, k], tt) for k in range(3)])
        jrk = np.column_stack([np.gradient(acc[:, k], tt) for k in range(3)])
        tu, jx = band_series(tt, jrk[:, 0])
        _, jy = band_series(tt, jrk[:, 1])
        _, jz = band_series(tt, jrk[:, 2])
        e = jx ** 2 + jy ** 2 + jz ** 2
        jerk_rms = float(np.sqrt(e.mean()))

        # planned switches inside the segment
        sw_td, sw_lo, td, leads = [], [], [], []
        if mode is not None:
            m = (t >= s["t0"]) & (t <= s["t1"])
            idx = np.flatnonzero(m)
            for k in idx[1:]:
                m0, m1 = int(mode[k - 1]), int(mode[k])
                if m0 == m1:
                    continue
                if m0 in swinging and m1 == 3:
                    sw_td.append(t[k])
                    f = by_side.get(swinging[m0])
                    z = sig(f"obs_foot/{f}/z") if f else None
                    if z is not None:
                        k0 = max(0, k - int(round(0.25 / np.median(np.diff(t)))))
                        down = z[k0:k + 1] < 0.002
                        if down[-1]:
                            nz = np.flatnonzero(~down)
                            kc = k0 + (nz[-1] + 1 if nz.size else 0)
                        else:  # foot still up at the switch: first touchdown after it
                            after = np.flatnonzero(z[k:k + int(round(0.15 / np.median(np.diff(t))))] < 0.002)
                            kc = k + (after[0] if after.size else 0)
                        td.append(t[kc])
                        leads.append(1000 * (t[k] - t[kc]))
                elif m0 == 3 and m1 in swinging:
                    sw_lo.append(t[k])
        e_td, t_td = share(tu, e, td, a.win)
        e_std, t_std = share(tu, e, sw_td, a.win)
        e_slo, t_slo = share(tu, e, sw_lo, a.win)
        conc = lambda es, ts: es / ts if np.isfinite(es) and ts > 0 else np.nan
        # stride-phase profile between consecutive measured touchdowns (either foot)
        prof = np.zeros(a.bins)
        tds = np.sort(np.array(td))
        for t0, t1 in zip(tds[:-1], tds[1:]):
            if not 0.2 < t1 - t0 < 1.5:
                continue
            m = (tu >= t0) & (tu < t1)
            ph = (tu[m] - t0) / (t1 - t0)
            prof += np.bincount(np.minimum((ph * a.bins).astype(int), a.bins - 1), weights=e[m], minlength=a.bins)
        prof = 100 * prof / prof.sum() if prof.sum() > 0 else prof
        lead_s = f"{np.mean(leads):5.0f} {np.std(leads):4.0f}" if leads else f"{'-':>10s}"
        print(f"{tag:22s} {s['label']:14s} {len(td):4d} {lead_s:>12s} {jerk_rms:6.1f} | "
              f"{100 * e_td:4.0f} {conc(e_td, t_td):5.1f}  {100 * e_std:4.0f} {conc(e_std, t_std):5.1f}  "
              f"{100 * e_slo:4.0f} {conc(e_slo, t_slo):5.1f} | " + " ".join(f"{p:4.0f}" for p in prof))
print(f"\nwindows +-{1000 * a.win:.0f} ms; E% = share of the segment's 0-15 Hz jerk energy inside the windows, conc = E% / time share.")
