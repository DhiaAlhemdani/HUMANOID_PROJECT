#!/usr/bin/env python3
"""Lateral anatomy of a walk (2.D recon): what does the pelvis do sideways, what does the plan ask for,
and what would a linear inverted pendulum do between the same footholds?

    python3 tests/tuning/lateral_anatomy.py tuning_results/<case> [more cases]

Per case, over the steady-walk window:
  - the plan's lateral pelvis velocity reference (pelvis frame) -- expected 0: the base cost tracks
    vy to the command, and x/y position is a gauge, so no lateral sway is planned at all;
  - what the pelvis actually does: lateral velocity and position relative to the stance foot,
    binned over the single-support phase (0 = contact change, 1 = next one);
  - the LIPM prediction for the measured step width, support time and pelvis height: closest
    approach to the stance foot at mid-stance and lateral velocity at the switch;
  - what pays for the difference: stance ankle-roll and hip-roll torque, body roll and roll rate,
    and the 4-40 Hz band of the lateral pelvis acceleration (candidate metric `pelvis_lat_acc_hf`).
Columns missing from an older log print as nan.
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from touchdown_trace import load  # noqa: E402

G = 9.81
NBIN = 10


def band_t(tt, x, lo=4.0, hi=40.0, fs=500.0):
    if len(tt) < 100 or not np.isfinite(x).all():
        return float("nan")
    tu = np.arange(tt[0], tt[-1], 1.0 / fs)
    xu = np.interp(tu, tt, x)
    xu = xu - np.polyval(np.polyfit(tu - tu[0], xu, 1), tu - tu[0])
    f = np.fft.rfftfreq(len(xu), 1.0 / fs)
    X = np.fft.rfft(xu)
    X[(f < lo) | (f > hi)] = 0
    return float(np.sqrt(np.mean(np.fft.irfft(X, len(xu)) ** 2)))


def side(foot):
    return "left" if ("_l" in foot and "_r" not in foot.replace("_l", "")) or "left" in foot else "right"


def one(P):
    OUT, t, c, walk, feet = load(P)
    name = P.rstrip("/").split("/")[-1]
    names = set(np.load(OUT("data.npz"), allow_pickle=False)["track_names"])
    if len(feet) != 2:
        print(f"=== {name}: need two feet, have {feet}"); return
    yaw = c("obs_base/yaw")
    cy, sy = np.cos(yaw), np.sin(yaw)
    lat = lambda vx, vy: -sy * vx + cy * vy  # world -> pelvis-frame lateral (left positive)
    vy_obs = lat(c("obs_base/vx"), c("obs_base/vy"))
    vy_plan = lat(c("plan_base/vx"), c("plan_base/vy"))
    fz = {f: np.nan_to_num(c(f"plan_wrench/{f}/fz"), nan=0.0) > 1.0 for f in feet}
    ss = {f: fz[f] & ~fz[[g for g in feet if g != f][0]] for f in feet}  # single support on f
    changes = np.flatnonzero((np.diff(fz[feet[0]].astype(int)) != 0) | (np.diff(fz[feet[1]].astype(int)) != 0)) + 1
    changes = changes[walk[changes]]
    # Per single-support interval: phase, lateral position of pelvis relative to the stance foot
    # (pelvis frame, positive = away from the foot, i.e. toward the body centre line and beyond).
    bins = {k: [[] for _ in range(NBIN)] for k in ("vy", "yrel", "roll", "ank", "hip")}
    widths, y_sw, y_mid, vy_sw, T_ss = [], [], [], [], []
    for a, b in zip(changes[:-1], changes[1:]):
        f = next((g for g in feet if ss[g][a:b].all()), None)
        if f is None or b - a < 20:
            continue
        other = [g for g in feet if g != f][0]
        sgn = -1.0 if side(f) == "left" else 1.0  # away from a left foot is to the right
        yrel = sgn * lat(c("obs_base/x") - c(f"obs_foot/{f}/x"), c("obs_base/y") - c(f"obs_foot/{f}/y"))
        vy_a = sgn * vy_obs
        s = (np.arange(a, b) - a) / (b - a)
        ank = c(f"meas_tau/{side(f)}_ankle_roll_joint"); hip = c(f"meas_tau/{side(f)}_hip_roll_joint")
        for k in range(NBIN):
            m = np.arange(a, b)[(s >= k / NBIN) & (s < (k + 1) / NBIN)]
            if len(m):
                bins["vy"][k].append(np.nanmean(vy_a[m])); bins["yrel"][k].append(np.nanmean(yrel[m]))
                bins["roll"][k].append(np.nanmean(c("obs_base/roll")[m]))
                bins["ank"][k].append(np.sqrt(np.nanmean(ank[m] ** 2))); bins["hip"][k].append(np.sqrt(np.nanmean(hip[m] ** 2)))
        widths.append(abs(float(lat(c(f"obs_foot/{other}/x") - c(f"obs_foot/{f}/x"),
                                    c(f"obs_foot/{other}/y") - c(f"obs_foot/{f}/y"))[b - 1])))
        y_sw.append(float(np.nanmean(yrel[[a, b - 1]]))); y_mid.append(float(np.nanmin(yrel[a:b])))
        vy_sw.append(float(np.nanmax(np.abs(vy_a[[a, b - 1]])))); T_ss.append(float(t[b] - t[a]))
    if not T_ss:
        print(f"=== {name}: no single-support intervals in the walk window"); return
    z = float(np.nanmean(c("obs_base/z")[walk]))
    omega = np.sqrt(G / max(z, 0.1)); T = float(np.mean(T_ss)); h = omega * T / 2
    ysw = float(np.mean(y_sw)); ymid_pred = ysw / np.cosh(h); vysw_pred = ymid_pred * omega * np.sinh(h)
    w = walk
    print(f"=== {name}: {len(T_ss)} single-support intervals, T_ss {T:.3f} s, step width {np.mean(widths):.3f} m, "
          f"pelvis z {z:.3f} m (omega {omega:.2f} 1/s)")
    print(f"    plan lateral vy (pelvis frame): rms {np.sqrt(np.nanmean(vy_plan[w] ** 2)):.3f} m/s | "
          f"observed: rms {np.sqrt(np.nanmean(vy_obs[w] ** 2)):.3f}, at the switch {np.mean(vy_sw):.3f} m/s")
    print(f"    pelvis vs stance foot (lateral): at the switch {ysw:.3f} m, closest approach {np.mean(y_mid):.3f} m "
          f"-> sway {1e3 * (ysw - np.mean(y_mid)):.0f} mm per step")
    print(f"    LIPM between the same footholds: closest approach {ymid_pred:.3f} m (sway {1e3 * (ysw - ymid_pred):.0f} mm), "
          f"vy at the switch {vysw_pred:.3f} m/s")
    print(f"    cost of not swaying: stance ankle-roll tau rms {np.nanmean([np.mean(b) for b in bins['ank'] if b]):.2f} Nm, "
          f"hip-roll {np.nanmean([np.mean(b) for b in bins['hip'] if b]):.2f} Nm | roll rms {np.degrees(np.nanstd(c('obs_base/roll')[w])):.2f} deg, "
          f"roll-rate rms {np.degrees(np.sqrt(np.nanmean(c('obs_base/droll')[w] ** 2))):.1f} deg/s | "
          f"lateral acc hf {band_t(t[w][1:], np.diff(vy_obs[w]) / np.maximum(np.diff(t[w]), 1e-6)):.3f} "
          f"(vertical {band_t(t[w][1:], np.diff(c('obs_base/vz')[w]) / np.maximum(np.diff(t[w]), 1e-6)):.3f})")
    print("    phase  vy away [m/s]  y vs foot [m]  roll [deg]  ankle-roll [Nm]  hip-roll [Nm]")
    for k in range(NBIN):
        g = lambda key: np.mean(bins[key][k]) if bins[key][k] else np.nan
        print(f"     {(k + 0.5) / NBIN:4.2f}   {g('vy'):+8.3f}      {g('yrel'):7.3f}      {np.degrees(g('roll')):+6.2f}       {g('ank'):6.2f}         {g('hip'):6.2f}")


if __name__ == "__main__":
    for P in sys.argv[1:]:
        one(P)
