#!/usr/bin/env python3
"""Score a scripted run (PROFILE=script) by command segment: the command-space smoothness benchmark.

usage: score_suite.py <case_dir | out_prefix> [title]
writes <dir>/suite.json and <dir>/suite.png; prints SUITE and one line per segment.

Segments come from the script's knots (saved in data.npz by tune_probe.py / bag_to_npz.py):
  steady      a constant-command hold, scored from 1.5 s after the command settles to its end;
  transition  a ramp between two holds, scored from the ramp start to 2.0 s after its end
              (the gait ladder's splice lands here: a change starts at the first contact event after
              0.3 s of horizon, so the mechanical transient of a command change is 0.5-2.5 s later).
Per segment (ground truth at 100 Hz, ~/tracking at 500 Hz):
  acc_hf       pelvis vertical acceleration, 4-40 Hz band RMS [m/s^2]   (the legacy smoothness number)
  lat_acc_hf   pelvis lateral (pelvis-frame y) acceleration, 4-40 Hz band RMS
  jerk         pelvis 3-D jerk RMS, 0-15 Hz [m/s^3] -- what a rider feels and a viewer sees
  ang_jerk     pelvis angular acceleration RMS (roll/pitch rates differentiated), 0-15 Hz [rad/s^2]
  roll_pp / pitch_pp   peak-to-peak roll and pitch [deg]
  tau_rate     leg measured torque rate RMS, 0-40 Hz [Nm/s]: the hardware-side proxy of torque steps
  tau_hf       leg measured torque, 4-40 Hz band RMS [Nm]
  v_err        command tracking error RMS (|v_cmd - v_pelvis| planar, after the ramp) [m/s]
  gaits        templates seen (from plan_mode)
Aggregate (SUITE line / suite.json "agg"): mean over steady segments, max over transitions, for
jerk, acc_hf, tau_rate; falls disqualify (fell_at recorded).
"""
import json
import math
import os
import sys

import numpy as np

P = sys.argv[1]
TITLE = sys.argv[2] if len(sys.argv) > 2 else P.rstrip("/").split("/")[-1]
OUT = (lambda name: os.path.join(P, name)) if os.path.isdir(P) else (lambda name: P + "_" + name)
d = np.load(OUT("data.npz"), allow_pickle=False)
if "script" not in d.files or d["script"].size == 0:
    sys.exit(f"{P}: no script in data.npz (run with PROFILE=script)")
K = d["script"]                                   # [t, vx, vy, yaw] on the script clock
SETTLE = float(d["phases"][0])
gt = d["gt"].copy()                               # t_sim, t_wall, x, y, z, vx, vy, vz, roll, pitch, yaw
# The odometry twist's frame has differed between simulator builds (docs/h2_wb_id_mpc.md); derive the
# world-frame velocity from the pose instead, so every rotation below is well defined.
gt = gt[np.argsort(gt[:, 0])]
gt = gt[np.concatenate([[True], np.diff(gt[:, 0]) > 1e-6])]
for c in (5, 6, 7):
    gt[:, c] = np.gradient(gt[:, c - 3], gt[:, 0])
fell = float(d["fell"]) if "fell" in d.files else float("nan")
w2s = lambda w: float(np.interp(w, gt[:, 1], gt[:, 0]))   # probe wall clock -> sim time
TN = list(d["track_names"]) if "track_names" in d.files else []
TR = d["track"] if TN and d["track"].size else None
tc = lambda n: TR[:, TN.index(n)] if TR is not None and n in TN else None


def band(tt, x, lo, hi, fs=500.0):
    """RMS of x in [lo, hi] Hz after resampling at fs (lo = 0 keeps the mean out, not the trend)."""
    if len(tt) < 50 or not np.isfinite(x).all():
        return float("nan")
    tu = np.arange(tt[0], tt[-1], 1.0 / fs)
    xu = np.interp(tu, tt, x)
    xu = xu - np.polyval(np.polyfit(tu - tu[0], xu, 1), tu - tu[0])
    f = np.fft.rfftfreq(len(xu), 1.0 / fs)
    X = np.fft.rfft(xu)
    X[(f < lo) | (f > hi)] = 0
    return float(np.sqrt(np.mean(np.fft.irfft(X, len(xu)) ** 2)))


def deriv(tt, x):
    return np.gradient(x, tt)


# ---- segments on the script clock -------------------------------------------------------------
segs = []
for a, b in zip(K[:-1], K[1:]):
    if np.allclose(a[1:], b[1:]):
        if b[0] - a[0] >= 2.5:
            segs.append(dict(kind="steady", t0=a[0] + 1.5, t1=b[0], cmd=list(a[1:]), label=f"hold vx {a[1]:+.1f} vy {a[2]:+.1f} w {a[3]:+.1f}"))
    else:
        segs.append(dict(kind="transition", t0=a[0], t1=b[0] + 2.0, cmd=list(b[1:]),
                         label=f"ramp -> vx {b[1]:+.1f} vy {b[2]:+.1f} w {b[3]:+.1f}"))

# ---- gait names from plan_mode ---------------------------------------------------------------
MODE = {0: "FLY", 1: "RF", 2: "LF", 3: "STANCE"}
legs = [n.split("/", 1)[1] for n in TN if n.startswith("meas_tau/") and any(k in n for k in ("hip", "knee", "ankle"))]
jt = tc("t")
yaw_tr = tc("obs_base/yaw")

rows = []
for s in segs:
    s0, s1 = w2s(SETTLE + s["t0"]), w2s(SETTLE + s["t1"])
    if np.isfinite(fell) and fell < SETTLE + s["t1"]:
        s["fell"] = True
    g = gt[(gt[:, 0] >= s0) & (gt[:, 0] <= s1)]
    if len(g) < 50:
        continue
    tt = g[:, 0]
    acc = np.column_stack([deriv(tt, g[:, 5]), deriv(tt, g[:, 6]), deriv(tt, g[:, 7])])
    jerk = np.column_stack([deriv(tt, acc[:, k]) for k in range(3)])
    yaw = g[:, 10]
    lat_v = -np.sin(yaw) * g[:, 5] + np.cos(yaw) * g[:, 6]
    fwd_v = np.cos(yaw) * g[:, 5] + np.sin(yaw) * g[:, 6]
    ang_acc = np.column_stack([deriv(tt, deriv(tt, np.unwrap(g[:, 8]))), deriv(tt, deriv(tt, np.unwrap(g[:, 9])))])
    r = dict(kind=s["kind"], label=s["label"], t0=round(s0, 2), t1=round(s1, 2), fell=bool(s.get("fell", False)),
             acc_hf=band(tt, acc[:, 2], 4, 40), lat_acc_hf=band(tt, deriv(tt, lat_v), 4, 40),
             jerk=float(np.sqrt(sum(band(tt, jerk[:, k], 0, 15) ** 2 for k in range(3)))),
             ang_jerk=float(np.sqrt(sum(band(tt, ang_acc[:, k], 0, 15) ** 2 for k in range(2)))),
             roll_pp=math.degrees(float(np.ptp(g[:, 8]))), pitch_pp=math.degrees(float(np.ptp(g[:, 9]))),
             z_pp_mm=1000 * float(np.ptp(g[:, 4])))
    # command tracking after the ramp (steady: whole segment; transition: last 2 s)
    tail = tt >= (s0 if s["kind"] == "steady" else s1 - 2.0)
    cmd = s["cmd"]
    r["v_err"] = float(np.sqrt(np.mean((fwd_v[tail] - cmd[0]) ** 2 + (lat_v[tail] - cmd[1]) ** 2))) if tail.sum() > 5 else float("nan")
    yaw_rate = deriv(tt, np.unwrap(yaw))
    r["w_err"] = float(np.sqrt(np.mean((yaw_rate[tail] - cmd[2]) ** 2))) if tail.sum() > 5 else float("nan")
    if TR is not None and legs:
        m = (jt >= s0) & (jt <= s1)
        if m.sum() > 50:
            taus = np.column_stack([tc("meas_tau/" + j)[m] for j in legs])
            r["tau_rate"] = float(np.mean([band(jt[m], deriv(jt[m], taus[:, k]), 0, 40) for k in range(taus.shape[1])]))
            r["tau_hf"] = float(np.mean([band(jt[m], taus[:, k], 4, 40) for k in range(taus.shape[1])]))
            pm = tc("plan_mode")
            if pm is not None:
                seen = [MODE.get(int(v), str(int(v))) for v in np.unique(pm[m][np.isfinite(pm[m])])]
                r["modes"] = "/".join(seen)
    rows.append({k: (round(v, 4) if isinstance(v, float) else v) for k, v in r.items()})

steady = [r for r in rows if r["kind"] == "steady"]
trans = [r for r in rows if r["kind"] == "transition"]
agg = {"survived": bool(np.isnan(fell)), "fell_at": None if np.isnan(fell) else round(fell - SETTLE, 1),
       "n_steady": len(steady), "n_transition": len(trans)}
for key in ("jerk", "acc_hf", "lat_acc_hf", "tau_rate", "ang_jerk", "v_err"):
    sv = [r[key] for r in steady if np.isfinite(r.get(key, np.nan))]
    tv = [r[key] for r in trans if np.isfinite(r.get(key, np.nan))]
    agg[f"steady_{key}_mean"] = round(float(np.mean(sv)), 4) if sv else None
    agg[f"steady_{key}_max"] = round(float(np.max(sv)), 4) if sv else None
    agg[f"transition_{key}_max"] = round(float(np.max(tv)), 4) if tv else None
    agg[f"transition_{key}_mean"] = round(float(np.mean(tv)), 4) if tv else None
if trans:
    worst = max(trans, key=lambda r: r.get("jerk", -1))
    agg["worst_transition"] = worst["label"]

json.dump({"title": TITLE, "agg": agg, "segments": rows, "script": K.tolist()}, open(OUT("suite.json"), "w"), indent=1)

print(f"SUITE {TITLE} | survived={agg['survived']} fell_at={agg['fell_at']} | steady ({len(steady)}): jerk {agg['steady_jerk_mean']} "
      f"acc_hf {agg['steady_acc_hf_mean']} tau_rate {agg['steady_tau_rate_mean']} v_err {agg['steady_v_err_mean']} | "
      f"transitions ({len(trans)}): jerk max {agg['transition_jerk_max']} acc_hf max {agg['transition_acc_hf_max']} "
      f"tau_rate max {agg['transition_tau_rate_max']} | worst: {agg.get('worst_transition')}")
print("SEG segment                               kind        jerk  acc_hf lat_hf ang_jrk roll_pp pitch_pp  tau_rate  v_err  modes")
for r in rows:
    print(f"SEG {r['label']:36s} {r['kind']:10s} {r['jerk']:6.2f} {r['acc_hf']:6.3f} {r['lat_acc_hf']:6.3f} {r['ang_jerk']:7.2f} "
          f"{r['roll_pp']:6.2f} {r['pitch_pp']:7.2f} {r.get('tau_rate', float('nan')):9.1f} {r['v_err']:6.3f}  {r.get('modes', '')}"
          + ("  FELL" if r["fell"] else ""))

# ---- figure ------------------------------------------------------------------------------------
try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    ts = gt[:, 0] - w2s(SETTLE)
    fig, ax = plt.subplots(4, 1, figsize=(14, 10), sharex=True)
    ax[0].plot(K[:, 0], K[:, 1], "k-", label="vx cmd"); ax[0].plot(K[:, 0], K[:, 2], "b-", label="vy cmd"); ax[0].plot(K[:, 0], K[:, 3], "g-", label="yaw cmd")
    yaw = gt[:, 10]
    ax[0].plot(ts, np.cos(yaw) * gt[:, 5] + np.sin(yaw) * gt[:, 6], "k:", lw=0.8); ax[0].plot(ts, -np.sin(yaw) * gt[:, 5] + np.cos(yaw) * gt[:, 6], "b:", lw=0.8)
    ax[0].set_ylabel("command / pelvis [m/s, rad/s]"); ax[0].legend(loc="upper right", fontsize=8)
    acc = np.column_stack([deriv(gt[:, 0], gt[:, 5 + k]) for k in range(3)])
    ax[1].plot(ts, np.linalg.norm(np.column_stack([deriv(gt[:, 0], acc[:, k]) for k in range(3)]), axis=1), lw=0.6); ax[1].set_ylabel("pelvis |jerk| [m/s³]")
    ax[2].plot(ts, np.degrees(gt[:, 8]), label="roll"); ax[2].plot(ts, np.degrees(gt[:, 9]), label="pitch"); ax[2].set_ylabel("deg"); ax[2].legend(fontsize=8)
    if TR is not None and legs:
        tau = np.column_stack([tc("meas_tau/" + j) for j in legs])
        ax[3].plot(jt - w2s(SETTLE), np.sqrt(np.mean(np.column_stack([deriv(jt, tau[:, k]) for k in range(tau.shape[1])]) ** 2, axis=1)), lw=0.5)
    ax[3].set_ylabel("leg |dτ/dt| RMS [Nm/s]"); ax[3].set_xlabel("script time [s]")
    for s in segs:
        for a in ax:
            a.axvspan(s["t0"], s["t1"], color="orange" if s["kind"] == "transition" else "green", alpha=0.08)
    fig.suptitle(f"{TITLE}: command-space suite segments (green steady, orange transition)")
    fig.tight_layout(); fig.savefig(OUT("suite.png"), dpi=110)
except Exception as e:  # the numbers are the deliverable; the figure is best effort
    print(f"suite.png skipped: {e}")
