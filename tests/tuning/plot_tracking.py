#!/usr/bin/env python3
"""Plot and score MPC plan vs joint command vs measurement from a claude_tune_probe *_data.npz.

usage: plot_tracking.py <prefix> [title]
Writes <prefix>_{joints,base,spectra,ff_fb,solver}.png and <prefix>_diag.json and prints
a DIAG line. The question each number answers: is the problem already in what the MPC plans and
commands (MPC level), or does the measurement depart from a good command (joint servo level)?
"""
import json
import math
import os
import pathlib
import sys
import xml.etree.ElementTree as ET

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

P = sys.argv[1]
TITLE = sys.argv[2] if len(sys.argv) > 2 else P.rstrip("/").split("/")[-1]
# A case folder from run_case.sh (data.npz -> joints.png ...) or a file prefix (<p>_data.npz -> <p>_joints.png ...).
OUT = (lambda name: os.path.join(P, name)) if os.path.isdir(P) else (lambda name: P + "_" + name)
d = np.load(OUT("data.npz"), allow_pickle=False)
# ---------------- SUMMARY: ground truth + joint states at full rate ----------------
def band_t(tt, x, lo=4.0, hi=40.0, fs=500.0):
    """RMS of x restricted to [lo, hi] Hz after resampling (tt must be sampled well above 2*hi)."""
    if len(tt) < 100:
        return float("nan")
    tu = np.arange(tt[0], tt[-1], 1.0 / fs)
    xu = np.interp(tu, tt, x)
    xu = xu - np.polyval(np.polyfit(tu - tu[0], xu, 1), tu - tu[0])
    f = np.fft.rfftfreq(len(xu), 1.0 / fs)
    X = np.fft.rfft(xu)
    X[(f < lo) | (f > hi)] = 0
    return float(np.sqrt(np.mean(np.fft.irfft(X, len(xu)) ** 2)))


SETTLE, RAMP, HOLD, STOP, VX = d["phases"]
gt = d["gt"]
fell_wall = float(d["fell"]) if "fell" in d.files else float("nan")
w2s = lambda w: float(np.interp(w, gt[:, 1], gt[:, 0]))
w0, w1 = w2s(SETTLE + RAMP + 3.0), w2s(SETTLE + RAMP + HOLD)
# Joint signals: /joint_states header stamps only advance with the 100 Hz sim clock, so use the
# measured joints of ~/tracking (exact control-tick times, trackingRate) when it is there.
TN = list(d["track_names"]) if "track_names" in d.files else []
if TN and d["track"].size:
    TR = d["track"]
    jn = [n.split("/", 1)[1] for n in TN if n.startswith("meas_q/")]
    jt = TR[:, TN.index("t")]
    JQ = np.column_stack([TR[:, TN.index("meas_q/" + n)] for n in jn])
    JV = np.column_stack([TR[:, TN.index("meas_v/" + n)] for n in jn])
    JE = np.column_stack([TR[:, TN.index("meas_tau/" + n)] for n in jn])
else:
    jt, jn = d["js_t"], list(d["js_names"])
    JQ, JV, JE = d["js_q"], d["js_v"], d["js_e"]
o = np.argsort(jt); jt, JQ, JV, JE = jt[o], JQ[o], JV[o], JE[o]
jt, u = np.unique(jt, return_index=True); JQ, JV, JE = JQ[u], JV[u], JE[u]
G_ARMS = ["left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_elbow_joint",
          "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_elbow_joint"]
G_LEGS = [n for n in jn if any(k in n for k in ("hip", "knee", "ankle"))]
metrics = {"survived": bool(np.isnan(fell_wall)), "fell_at": None if np.isnan(fell_wall) else round(fell_wall - SETTLE, 1),
           "rates_hz": {"joints": round(len(jt) / (jt[-1] - jt[0]), 1), "ground_truth": round(len(np.unique(gt[:, 0])) / (gt[-1, 0] - gt[0, 0]), 1)}}
if np.isnan(fell_wall) or fell_wall > SETTLE + RAMP + HOLD:
    g = gt[(gt[:, 0] >= w0) & (gt[:, 0] <= w1)]
    m = (jt >= w0) & (jt <= w1)
    tt = g[:, 0]
    W = {"vx_mean": float((g[-1, 2] - g[0, 2]) / (tt[-1] - tt[0])),
         "vy_mean": float((g[-1, 3] - g[0, 3]) / (tt[-1] - tt[0])),
         "yaw_rate_mean": float(np.unwrap(g[:, 10])[-1] - np.unwrap(g[:, 10])[0]) / float(tt[-1] - tt[0]),
         "z_sd_mm": 1000 * float(np.std(g[:, 4] - np.polyval(np.polyfit(tt - tt[0], g[:, 4], 1), tt - tt[0]))),
         "vz_rms": float(np.sqrt(np.mean(g[:, 7] ** 2))),
         "roll_rms_deg": math.degrees(float(np.std(g[:, 8]))), "pitch_rms_deg": math.degrees(float(np.std(g[:, 9]))),
         "max_tilt_deg": math.degrees(float(np.max(np.hypot(g[:, 8], g[:, 9])))),
         "yaw_change_deg": math.degrees(float(g[-1, 10] - g[0, 10])),
         "pelvis_acc_hf": band_t(tt[1:], np.diff(g[:, 7]) / np.maximum(np.diff(tt), 1e-6)),
         "arm_vel_hf": float(np.mean([band_t(jt[m], JV[m, jn.index(n)]) for n in G_ARMS])),
         "arm_tau_hf": float(np.mean([band_t(jt[m], JE[m, jn.index(n)]) for n in G_ARMS])),
         "leg_vel_hf": float(np.mean([band_t(jt[m], JV[m, jn.index(n)]) for n in G_LEGS])),
         "leg_tau_hf": float(np.mean([band_t(jt[m], JE[m, jn.index(n)]) for n in G_LEGS])),
         "arm_swing_pp_deg": math.degrees(float(np.mean([np.ptp(JQ[m, jn.index(n)]) for n in ("left_shoulder_pitch_joint", "right_shoulder_pitch_joint")])))}
    metrics["walk"] = {k: round(v, 4) for k, v in W.items()}
json.dump(metrics, open(OUT("metrics.json"), "w"), indent=1)
Wm = metrics.get("walk", {})
nan = float("nan")
PROF = str(d["profile"]) if "profile" in d.files else "vx"
metrics["profile"] = PROF
print(f"SUMMARY {TITLE} [{PROF}] vy {Wm.get('vy_mean', nan):.3f} yaw_rate {Wm.get('yaw_rate_mean', nan):.3f} | survived={metrics['survived']} fell_at={metrics['fell_at']} | rates joints {metrics['rates_hz']['joints']} gt {metrics['rates_hz']['ground_truth']} Hz"
      f" | vx {Wm.get('vx_mean', nan):.3f} z_sd {Wm.get('z_sd_mm', nan):.1f}mm vz_rms {Wm.get('vz_rms', nan):.3f} roll {Wm.get('roll_rms_deg', nan):.2f} pitch {Wm.get('pitch_rms_deg', nan):.2f}"
      f" tilt_max {Wm.get('max_tilt_deg', nan):.1f} acc_hf {Wm.get('pelvis_acc_hf', nan):.2f} | arm vel_hf {Wm.get('arm_vel_hf', nan):.3f} tau_hf {Wm.get('arm_tau_hf', nan):.2f}"
      f" swing {Wm.get('arm_swing_pp_deg', nan):.1f} | leg vel_hf {Wm.get('leg_vel_hf', nan):.3f} tau_hf {Wm.get('leg_tau_hf', nan):.2f} | yaw {Wm.get('yaw_change_deg', nan):.1f}")

names = list(d["track_names"])
T = d["track"]
if T.size == 0 or not names:
    print("DIAG no tracking data")
    sys.exit(0)
col = {n: i for i, n in enumerate(names)}
c = lambda n: T[:, col[n]]
t = c("t")
loco = c("phase") == np.max(c("phase")[c("policy_valid") > 0]) if np.any(c("policy_valid") > 0) else np.zeros(len(t), bool)
# Walking window in sim time (w0, w1 above), restricted to ticks with a valid MPC policy.
walk = (t >= w0) & (t <= w1) & (c("policy_valid") > 0)
if walk.sum() < 200:
    walk = c("policy_valid") > 0
J = [n.split("/", 1)[1] for n in names if n.startswith("cmd_q/")]
LEGS = [j for j in J if any(k in j for k in ("hip", "knee", "ankle"))]
ARMS = [j for j in J if any(k in j for k in ("shoulder", "elbow"))]
WAIST = [j for j in J if "waist" in j]
FEET = sorted({n.split("/")[1] for n in names if n.startswith("plan_foot/")})


def resample(x, mask, fs=500.0):
    tt = t[mask]
    tu = np.arange(tt[0], tt[-1], 1.0 / fs)
    return tu, np.interp(tu, tt, x[mask])


def band(x, mask, lo=4.0, hi=40.0, fs=500.0):
    tu, xu = resample(x, mask, fs)
    xu = xu - np.polyval(np.polyfit(tu - tu[0], xu, 1), tu - tu[0])
    f = np.fft.rfftfreq(len(xu), 1 / fs)
    X = np.fft.rfft(xu)
    X[(f < lo) | (f > hi)] = 0
    return float(np.sqrt(np.mean(np.fft.irfft(X, len(xu)) ** 2)))


def psd(x, mask, fs=500.0, seg=1024):
    tu, xu = resample(x, mask, fs)
    xu = xu - xu.mean()
    n = min(seg, len(xu))
    win = np.hanning(n)
    segs = [xu[i:i + n] * win for i in range(0, len(xu) - n + 1, n // 2)]
    S = np.mean([np.abs(np.fft.rfft(s)) ** 2 for s in segs], axis=0) / (fs * (win ** 2).sum())
    return np.fft.rfftfreq(n, 1 / fs), S


diag = {"title": TITLE, "samples_walk": int(walk.sum())}
for gname, group in (("legs", LEGS), ("waist", WAIST), ("arms", ARMS)):
    e_q = np.array([c("meas_q/" + j)[walk] - c("cmd_q/" + j)[walk] for j in group])
    e_v = np.array([c("meas_v/" + j)[walk] - c("cmd_qd/" + j)[walk] for j in group])
    pd = {j: c("meas_tau/" + j) - c("cmd_tau/" + j) for j in group}
    diag[gname] = {
        "track_err_q_deg": round(math.degrees(float(np.sqrt(np.mean(e_q ** 2)))), 3),
        "track_err_v": round(float(np.sqrt(np.mean(e_v ** 2))), 4),
        # HF (4-40 Hz) content: in the MPC's own command vs in what the servo adds.
        "cmd_q_hf_deg": round(math.degrees(np.mean([band(c("cmd_q/" + j), walk) for j in group])), 4),
        "meas_q_hf_deg": round(math.degrees(np.mean([band(c("meas_q/" + j), walk) for j in group])), 4),
        "cmd_tau_hf": round(float(np.mean([band(c("cmd_tau/" + j), walk) for j in group])), 3),
        "pd_tau_hf": round(float(np.mean([band(pd[j], walk) for j in group])), 3),
        # Total applied torque HF: the servo-architecture-independent number (with the whole-body
        # layer the feedback moves from pd_tau into cmd_tau, so neither alone compares across configs).
        "meas_tau_hf": round(float(np.mean([band(c("meas_tau/" + j), walk) for j in group])), 3),
        "pd_tau_rms": round(float(np.mean([np.sqrt(np.mean(pd[j][walk] ** 2)) for j in group])), 3),
        "cmd_tau_rms": round(float(np.mean([np.sqrt(np.mean(c("cmd_tau/" + j)[walk] ** 2)) for j in group])), 3),
        # Peaks the averages hide (touchdown impacts): 99.9th percentile and max of |PD torque|.
        "pd_tau_p999": round(float(np.max([np.percentile(np.abs(pd[j][walk]), 99.9) for j in group])), 2),
        "pd_tau_max": {j.replace("_joint", ""): round(float(np.max(np.abs(pd[j][walk]))), 1)
                       for j in sorted(group, key=lambda j: -np.max(np.abs(pd[j][walk])))[:3]},
    }
# Actuator saturation (D2 reality check): the measured torque is the plant's actuator force,
# which MuJoCo clamps at actuatorfrcrange (= URDF effort). The ankles sit at 50 Nm, so
# legs.pd_tau_p999 pins at ~50.25 whenever >0.1 % of walk ticks saturate and says nothing
# about how often that happens — this does. Limits are read from the shipped URDF.
URDF = pathlib.Path(__file__).resolve().parents[2] / "description/urdf/robots/g1/g1_29dof.urdf"
try:
    _lim = {j.get("name"): float(j.find("limit").get("effort")) for j in ET.parse(URDF).getroot().iter("joint")
            if j.find("limit") is not None and j.find("limit").get("effort")}
except (OSError, ET.ParseError):
    _lim = {}
for gname, group in (("legs", LEGS), ("arms", ARMS)):
    sat = {j: float(np.mean(np.abs(c("meas_tau/" + j)[walk]) >= 0.98 * _lim[j])) for j in group if _lim.get(j)}
    if sat:
        worst = max(sat, key=sat.get)
        diag[gname]["tau_sat_pct"] = round(100.0 * sat[worst], 3)          # worst joint, % of walk ticks at >=98 % of its limit
        diag[gname]["tau_sat_pct_all"] = round(100.0 * float(np.mean(list(sat.values()))), 3)
        diag[gname]["tau_sat_worst"] = worst.replace("_joint", "")
        diag[gname]["tau_sat_limit"] = _lim[worst]
detr = lambda x, m: x[m] - np.polyval(np.polyfit(t[m] - t[m][0], x[m], 1), t[m] - t[m][0])
diag["base"] = {
    "plan_z_sd_mm": round(1000 * float(np.std(detr(c("plan_base/z"), walk))), 2),
    "obs_z_sd_mm": round(1000 * float(np.std(detr(c("obs_base/z"), walk))), 2),
    "plan_vz_rms": round(float(np.sqrt(np.mean(c("plan_base/vz")[walk] ** 2))), 4),
    "obs_vz_rms": round(float(np.sqrt(np.mean(c("obs_base/vz")[walk] ** 2))), 4),
    "z_track_err_mm": round(1000 * float(np.sqrt(np.mean((c("obs_base/z") - c("plan_base/z"))[walk] ** 2))), 2),
    "roll_track_err_deg": round(math.degrees(float(np.sqrt(np.mean((c("obs_base/roll") - c("plan_base/roll"))[walk] ** 2)))), 3),
    "pitch_track_err_deg": round(math.degrees(float(np.sqrt(np.mean((c("obs_base/pitch") - c("plan_base/pitch"))[walk] ** 2)))), 3),
}
for f in FEET:
    diag["foot_" + f] = {"z_track_err_mm": round(1000 * float(np.sqrt(np.mean((c(f"obs_foot/{f}/z") - c(f"plan_foot/{f}/z"))[walk] ** 2))), 2),
                         "xy_track_err_mm": round(1000 * float(np.sqrt(np.mean((c(f"obs_foot/{f}/x") - c(f"plan_foot/{f}/x"))[walk] ** 2
                                                                                + (c(f"obs_foot/{f}/y") - c(f"plan_foot/{f}/y"))[walk] ** 2))), 2)}
# Command jumps at policy switches: the MPC replans each update; a jump there is plan inconsistency.
t0 = c("policy_t0")
switch = np.zeros(len(t), bool)
switch[1:] = np.diff(t0) != 0
switch &= walk
dtau = {j: np.abs(np.diff(c("cmd_tau/" + j), prepend=np.nan)) for j in J}
for gname, group in (("legs", LEGS), ("arms", ARMS)):
    at = np.nanmean([dtau[j][switch] for j in group])
    off = np.nanmean([dtau[j][walk & ~switch] for j in group])
    diag[gname]["cmd_tau_jump_at_switch"] = round(float(at), 3)
    diag[gname]["cmd_tau_step_otherwise"] = round(float(off), 3)
diag["policy_switch_hz"] = round(float(switch.sum() / max(1e-6, t[walk][-1] - t[walk][0])), 1)


# ---------------- gait templates & transitions (roadmap #1.5) ----------------
# plan_mode per tick (FLY 0, RF 1, LF 2, STANCE 3). A swing phase is labelled by what follows it:
# a short STANCE (< DS_MAX, the walk's double support) -> walk, the other foot's swing -> trot,
# FLY -> run; long stances are the stance template. Changes between locomotion templates are
# where gait.phaseTransitionStanceTime acts; each is scored over [-0.5, +1.5] s around the first
# step of the new template. Whole-run (policy-valid) scope, not just the hold window.
DS_MAX = 0.25
_mode = np.where(c("policy_valid") > 0, np.nan_to_num(c("plan_mode"), nan=-1.0), -1.0).astype(int)
_cut = np.flatnonzero(np.diff(_mode) != 0) + 1
phases = []  # (mode, t_start, t_end)
for s_, e_ in zip(np.concatenate([[0], _cut]), np.concatenate([_cut, [len(_mode)]])):
    if _mode[s_] >= 0:
        phases.append((int(_mode[s_]), float(t[s_]), float(t[e_]) if e_ < len(t) else float(t[-1])))
segs: list = []  # [label, t_start, t_end, swing durations]


def _push(label, a, b, dur=None):
    if segs and segs[-1][0] == label:
        segs[-1][2] = b
        if dur is not None:
            segs[-1][3].append(dur)
    else:
        segs.append([label, a, b, [] if dur is None else [dur]])


def _last_loco():
    return next((s[0] for s in reversed(segs) if s[0] != "stance"), "walk")


def _kind(p):
    if p is None:
        return "none"
    if p[0] == 0:
        return "fly"
    if p[0] == 3:
        return "ds" if p[2] - p[1] < DS_MAX else "stance"
    return "swing"


_LAB = {"ds": "walk", "swing": "trot"}
items = []  # ("stance", a, b, None) | ("swing", a, b, label-or-None)
for i, (m, a, b) in enumerate(phases):
    if m == 3:
        if b - a >= DS_MAX:
            items.append(["stance", a, b, None])
        continue  # a short double support belongs to the walk around it
    if m == 0:
        continue  # flight belongs to the run around it
    prv, nxt = _kind(phases[i - 1] if i > 0 else None), _kind(phases[i + 1] if i + 1 < len(phases) else None)
    # Steps with the same kind of neighbour on both sides are unambiguous (flight on either
    # side = run; a stop on one side = the other side decides). A step between a double
    # support and another swing is a boundary step of a walk<->trot change and is resolved
    # below by its duration (walk swing 0.3 s vs trot 0.5 s), since the splice stance that
    # phaseTransitionStanceTime inserts looks exactly like a walk double support.
    if "fly" in (prv, nxt):
        label = "run"
    elif prv in _LAB and nxt in _LAB and prv != nxt:
        label = None
    elif prv in _LAB:
        label = _LAB[prv]
    elif nxt in _LAB:
        label = _LAB[nxt]
    else:
        label = "walk"
    items.append(["swing", a, b, label])
for k, it in enumerate(items):
    if it[0] != "swing" or it[3] is not None:
        continue
    dur = it[2] - it[1]
    best, best_err = None, None
    for direction in (-1, 1):
        j = k + direction
        while 0 <= j < len(items) and not (items[j][0] == "swing" and items[j][3] is not None):
            j += direction
        if 0 <= j < len(items):
            err = abs((items[j][2] - items[j][1]) - dur)
            if best is None or err < best_err - 1e-9 or (abs(err - best_err) <= 1e-9 and direction == 1):
                best, best_err = items[j][3], err
    it[3] = best or _last_loco()
for kind_, a, b, label in items:
    if kind_ == "stance":
        _push("stance", a, b)
    else:
        _push(label, a, b, b - a)

gtt = gt[:, 0]
_hold = gt[(gtt >= w0) & (gtt <= w1)]
_roll_ref = float(np.median(_hold[:, 8])) if len(_hold) else float(np.median(gt[:, 8]))
changes = []
loco = [k for k, s in enumerate(segs) if s[0] != "stance"]
for k_prev, k_next in zip(loco, loco[1:]):
    between = segs[k_prev + 1:k_next]
    if any(s[2] - s[1] >= 1.0 for s in between):
        continue  # a real stop in between, not a gait-to-gait transition
    prev, new = segs[k_prev], segs[k_next]
    tc = new[1]
    ch = {"t": round(tc, 3), "from": prev[0], "to": new[0], "gap_s": round(new[1] - prev[2], 3)}
    g = gt[(gtt >= tc - 0.5) & (gtt <= tc + 1.5)]
    if len(g) >= 100:
        tt = g[:, 0]
        ch["acc_hf"] = round(band_t(tt[1:], np.diff(g[:, 7]) / np.maximum(np.diff(tt), 1e-6)), 4)
        ch["z_pp_mm"] = round(1000 * float(np.ptp(g[:, 4] - np.polyval(np.polyfit(tt - tt[0], g[:, 4], 1), tt - tt[0]))), 2)
        ch["roll_max_deg"] = round(math.degrees(float(np.max(np.abs(g[:, 8] - _roll_ref)))), 3)
    d_new = new[3]
    if len(d_new) >= 5:
        ref = float(np.median(d_new[2:]))
        ch["first_step_asym"] = round(float(max(abs(x - ref) for x in d_new[:2]) / max(ref, 1e-6)), 3)
    changes.append(ch)
_tot = {}
for lab, a, b, _ in segs:
    _tot[lab] = round(_tot.get(lab, 0.0) + (b - a), 2)
_agg = lambda key: (round(max(ch[key] for ch in changes if key in ch), 4) if any(key in ch for ch in changes) else None)
diag["gait"] = {
    "sequence": ">".join(s[0] for s in segs),
    "templates_s": _tot,
    "changes_locomotion": len(changes),
    "changes": changes,
    "transition_acc_hf_max": _agg("acc_hf"),
    "transition_z_pp_mm_max": _agg("z_pp_mm"),
    "transition_roll_max_deg": _agg("roll_max_deg"),
    "first_step_asym_max": _agg("first_step_asym"),
}
# ---------------- command transients (roadmap #1.6 S-curve slew) ----------------
# The probe's command schedule is known (wall clock -> sim time via ground truth): vx/vy/yaw
# profiles ramp over RAMP s at SETTLE and SETTLE+RAMP+HOLD; `stops` additionally steps the command
# 0 <-> VX every 4 s / 3 s during the hold. Each event is scored over [-0.3, +2.0] s on the ground
# truth: HF vertical acceleration, height excursion, pitch/roll excursion from the hold median.
events = []
if PROF == "stops":
    k, tw = 0, SETTLE + RAMP
    while tw + 4.0 < SETTLE + RAMP + HOLD:
        events.append(("step_down", tw + 4.0))
        if tw + 7.0 < SETTLE + RAMP + HOLD:
            events.append(("step_up", tw + 7.0))
        tw += 7.0
else:
    events = [("ramp_up_start", SETTLE), ("ramp_up_end", SETTLE + RAMP),
              ("ramp_down_start", SETTLE + RAMP + HOLD), ("ramp_down_end", SETTLE + RAMP + HOLD + RAMP)]
_pitch_ref = float(np.median(_hold[:, 9])) if len(_hold) else float(np.median(gt[:, 9]))
cmd_events = []
for kind_, tw in events:
    if not np.isnan(fell_wall) and fell_wall < tw:
        break
    ts = w2s(tw)
    g = gt[(gtt >= ts - 0.3) & (gtt <= ts + 2.0)]
    ev = {"kind": kind_, "t": round(ts, 3)}
    if len(g) >= 100:
        tt = g[:, 0]
        ev["acc_hf"] = round(band_t(tt[1:], np.diff(g[:, 7]) / np.maximum(np.diff(tt), 1e-6)), 4)
        ev["z_pp_mm"] = round(1000 * float(np.ptp(g[:, 4] - np.polyval(np.polyfit(tt - tt[0], g[:, 4], 1), tt - tt[0]))), 2)
        ev["pitch_max_deg"] = round(math.degrees(float(np.max(np.abs(g[:, 9] - _pitch_ref)))), 3)
        ev["roll_max_deg"] = round(math.degrees(float(np.max(np.abs(g[:, 8] - _roll_ref)))), 3)
    cmd_events.append(ev)
_cagg = lambda key: (round(max(ev[key] for ev in cmd_events if key in ev), 4) if any(key in ev for ev in cmd_events) else None)
diag["command"] = {
    "profile": PROF,
    "events": cmd_events,
    "transient_acc_hf_max": _cagg("acc_hf"),
    "transient_z_pp_mm_max": _cagg("z_pp_mm"),
    "transient_pitch_max_deg": _cagg("pitch_max_deg"),
    "transient_roll_max_deg": _cagg("roll_max_deg"),
}
# ---------------- touchdown events (roadmap 2.C) ----------------
# Per planned touchdown in the walk window (plan wrench fz of a foot 0 -> >1 N): how far the
# measured robot departs from the plan right after impact. The plan has no impact model, so its
# base vertical velocity is continuous across touchdown while the robot's is not; the mismatch
# over the first 60 ms is what the servo/whole-body layer must then absorb.
TD_WIN = 0.06
_nan = np.full(len(t), np.nan)
_cs = lambda n: c(n) if n in col else _nan  # synthetic recordings carry a subset of the columns
_fz = {f: np.nan_to_num(_cs(f"plan_wrench/{f}/fz"), nan=0.0) > 1.0 for f in FEET}
_leg = lambda f: [j for j in LEGS if j.startswith("left_" if ("_l_" in f or "left" in f) else "right_")]
td_events = []
for f in FEET:
    idx = np.flatnonzero(np.diff(_fz[f].astype(int)) > 0) + 1
    for k in idx:
        if not walk[k]:
            continue
        w = (t >= t[k]) & (t <= t[k] + TD_WIN)
        pre = (t >= t[k] - 0.01) & (t < t[k])
        if w.sum() < 5 or pre.sum() < 2:
            continue
        legs_f = _leg(f)
        # Measured landing, from kinematics (G1 logs no contact sensors): the real landing speed is the
        # most negative foot vz in the 100 ms before the planned event; the foot "stopped" at the first
        # tick after that minimum where vz >= -5 mm/s (window [-150, +60] ms); early_ms > 0 = the
        # robot landed before the plan said so.
        look = np.flatnonzero((t >= t[k] - 0.15) & (t <= t[k] + TD_WIN))
        fz_obs = _cs(f"obs_foot/{f}/z")
        vz_obs = _cs(f"obs_foot/{f}/vz")
        early_ms = impact_vz = plan_vz_at_stop = z_err_descent = float("nan")
        if len(look) and np.isfinite(vz_obs[look]).all():
            i_min = look[np.argmin(vz_obs[look])]
            after = look[look >= i_min]
            stopped = after[vz_obs[after] >= -5e-3]
            if len(stopped):
                i_stop = stopped[0]
                early_ms = 1e3 * (t[k] - t[i_stop])
                # speed at impact: the foot's velocity 2-4 ms before it stopped (the stop itself is smeared
                # over the soft contact); and what the plan was doing at that instant
                b = (t >= t[i_stop] - 4e-3) & (t <= t[i_stop] - 2e-3)
                impact_vz = float(np.mean(vz_obs[b])) if b.any() else float("nan")
                plan_vz_at_stop = float(_cs(f"plan_foot/{f}/vz")[i_stop])
                # tracking during the descent, 30 ms before the stop: obs - plan foot height (< 0: foot low = lag)
                dsc = (t >= t[i_stop] - 0.032) & (t <= t[i_stop] - 0.028)
                z_err_descent = 1e3 * float(np.mean((fz_obs - _cs(f"plan_foot/{f}/z"))[dsc])) if dsc.any() else float("nan")
        # resting height of the contact frame in mid-stance vs the plan's (geometric offset if constant)
        mid = (t >= t[k] + 0.10) & (t <= t[k] + 0.20)
        ev = {"t": round(float(t[k]), 3), "foot": f,
              "early_ms": early_ms,
              "impact_vz_obs": impact_vz,
              "plan_vz_at_stop": plan_vz_at_stop,
              "foot_z_err_descent_mm": z_err_descent,
              "stance_z_obs_mm": 1e3 * float(np.median(fz_obs[mid])) if mid.any() else float("nan"),
              "stance_z_plan_mm": 1e3 * float(np.median(_cs(f"plan_foot/{f}/z")[mid])) if mid.any() else float("nan"),
              "foot_z_err_pre_mm": 1e3 * float(np.mean((fz_obs - _cs(f"plan_foot/{f}/z"))[pre])),
              "base_z_err_pre_mm": 1e3 * float(np.mean((_cs("obs_base/z") - _cs("plan_base/z"))[pre])),
              "base_vz_mismatch": float(np.sqrt(np.mean((_cs("obs_base/vz") - _cs("plan_base/vz"))[w] ** 2))),
              "base_vz_drop": float(_cs("obs_base/vz")[pre].mean() - _cs("obs_base/vz")[w].min()),
              "foot_vz_obs_pre": float(np.mean(_cs(f"obs_foot/{f}/vz")[pre])),
              "foot_vz_plan_pre": float(np.mean(_cs(f"plan_foot/{f}/vz")[pre])),
              "leg_tau_peak": float(max(np.max(np.abs(_cs("meas_tau/" + j)[w])) for j in legs_f)) if legs_f else float("nan"),
              "leg_cmd_minus_plan_peak": float(max(np.max(np.abs((_cs("cmd_tau/" + j) - _cs("plan_tau/" + j))[w]))
                                                   for j in legs_f)) if legs_f else float("nan")}
        td_events.append(ev)
def _tdm(key):
    vals = [ev[key] for ev in td_events if np.isfinite(ev[key])]
    return round(float(np.median(vals)), 4) if vals else None
diag["touchdown"] = {
    "count": len(td_events),
    "window_s": TD_WIN,
    "early_ms": _tdm("early_ms"),                        # planned event minus measured foot stop, median [ms] (>0: landed early)
    "impact_vz_obs": _tdm("impact_vz_obs"),              # measured foot vz just before it stopped [m/s]
    "plan_vz_at_stop": _tdm("plan_vz_at_stop"),          # planned foot vz at that instant
    "foot_z_err_descent_mm": _tdm("foot_z_err_descent_mm"),  # obs - plan foot height 30 ms before the stop
    "stance_z_obs_mm": _tdm("stance_z_obs_mm"),          # contact-frame height at rest (mid-stance) vs the plan's
    "stance_z_plan_mm": _tdm("stance_z_plan_mm"),
    "foot_z_err_pre_mm": _tdm("foot_z_err_pre_mm"),      # obs - plan foot height 10 ms before the event
    "base_z_err_pre_mm": _tdm("base_z_err_pre_mm"),
    "base_vz_mismatch": _tdm("base_vz_mismatch"),        # rms(obs - plan base vz) over the window, median over events [m/s]
    "base_vz_drop": _tdm("base_vz_drop"),                # measured base vz loss at impact, median [m/s]
    "foot_vz_obs_pre": _tdm("foot_vz_obs_pre"),          # measured landing speed (plan: swingTrajectory.touchDownVelocity)
    "foot_vz_plan_pre": _tdm("foot_vz_plan_pre"),
    "leg_tau_peak": _tdm("leg_tau_peak"),                # median over events of the landing leg's |tau| peak [Nm]
    "leg_cmd_minus_plan_peak": _tdm("leg_cmd_minus_plan_peak"),
}
json.dump(diag, open(OUT("diag.json"), "w"), indent=1)

# ---------------- figures ----------------
mid = t[walk][0] + 0.5 * (t[walk][-1] - t[walk][0])
zoom = walk & (t >= mid) & (t <= mid + 2.5)
show = ["left_hip_pitch_joint", "left_hip_roll_joint", "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
        "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_elbow_joint"]
fig, ax = plt.subplots(len(show), 2, figsize=(15, 2.0 * len(show)), sharex=True)
for i, j in enumerate(show):
    ax[i, 0].plot(t[zoom], np.degrees(c("plan_q/" + j)[zoom]), lw=1.2, label="MPC plan q")
    ax[i, 0].plot(t[zoom], np.degrees(c("meas_q/" + j)[zoom]), lw=1.0, label="measured q")
    ax[i, 0].set_ylabel(j.replace("_joint", "").replace("left_", "L ") + " [deg]", fontsize=8)
    ax[i, 1].plot(t[zoom], c("plan_tau/" + j)[zoom], lw=1.0, label="MPC tau")
    ax[i, 1].plot(t[zoom], c("cmd_tau/" + j)[zoom], lw=1.0, ls="--", label="tau_ff sent")
    ax[i, 1].plot(t[zoom], c("meas_tau/" + j)[zoom], lw=0.8, label="applied (ff + PD)")
    ax[i, 1].set_ylabel("[Nm]", fontsize=8)
ax[0, 0].legend(fontsize=7, loc="upper right"); ax[0, 1].legend(fontsize=7, loc="upper right")
ax[-1, 0].set_xlabel("sim time [s]"); ax[-1, 1].set_xlabel("sim time [s]")
fig.suptitle(f"{TITLE}: joint position (left) and torque (right), 2.5 s of walking")
fig.tight_layout(); fig.savefig(OUT("joints.png"), dpi=80); plt.close(fig)

fig, ax = plt.subplots(5, 1, figsize=(14, 12), sharex=True)
g = gt[(gt[:, 0] >= t[zoom][0]) & (gt[:, 0] <= t[zoom][-1])]
ax[0].plot(t[zoom], c("plan_base/z")[zoom], label="MPC plan"); ax[0].plot(t[zoom], c("obs_base/z")[zoom], label="estimate"); ax[0].plot(g[:, 0], g[:, 4], lw=0.8, label="ground truth")
ax[0].set_ylabel("pelvis z [m]"); ax[0].legend(fontsize=8)
ax[1].plot(t[zoom], c("plan_base/vz")[zoom], label="MPC plan"); ax[1].plot(t[zoom], c("obs_base/vz")[zoom], label="estimate"); ax[1].set_ylabel("pelvis vz [m/s]"); ax[1].legend(fontsize=8)
for a in ("roll", "pitch"):
    ax[2].plot(t[zoom], np.degrees(c(f"plan_base/{a}")[zoom]), label=f"plan {a}"); ax[2].plot(t[zoom], np.degrees(c(f"obs_base/{a}")[zoom]), ls="--", label=f"est {a}")
ax[2].set_ylabel("[deg]"); ax[2].legend(fontsize=8, ncol=4)
for f in FEET:
    ax[3].plot(t[zoom], c(f"plan_foot/{f}/z")[zoom], label=f"plan {f}"); ax[3].plot(t[zoom], c(f"obs_foot/{f}/z")[zoom], ls="--", label=f"est {f}")
    ax[4].plot(t[zoom], c(f"plan_wrench/{f}/fz")[zoom], label=f"plan fz {f}")
ax[3].set_ylabel("foot z [m]"); ax[3].legend(fontsize=8, ncol=4); ax[4].set_ylabel("planned fz [N]"); ax[4].legend(fontsize=8)
ax[4].set_xlabel("sim time [s]")
fig.suptitle(f"{TITLE}: base and feet, MPC plan vs estimate"); fig.tight_layout(); fig.savefig(OUT("base.png"), dpi=80); plt.close(fig)

fig, ax = plt.subplots(2, 3, figsize=(16, 8))
for k, (gname, group) in enumerate((("legs", LEGS), ("arms", ARMS), ("waist", WAIST))):
    S = lambda key: np.mean([psd(c(key + "/" + j), walk)[1] for j in group], axis=0)
    f = psd(c("cmd_tau/" + group[0]), walk)[0]
    pdS = np.mean([psd(c("meas_tau/" + j) - c("cmd_tau/" + j), walk)[1] for j in group], axis=0)
    ax[0, k].semilogy(f, S("cmd_tau"), label="tau_ff (MPC)"); ax[0, k].semilogy(f, pdS, label="PD torque (applied - ff)")
    ax[0, k].set_title(f"{gname}: torque PSD"); ax[0, k].set_xlim(0, 60); ax[0, k].legend(fontsize=8)
    ax[1, k].semilogy(f, S("cmd_q"), label="q command (MPC)"); ax[1, k].semilogy(f, S("meas_q"), label="q measured")
    ax[1, k].set_title(f"{gname}: position PSD"); ax[1, k].set_xlim(0, 60); ax[1, k].set_xlabel("Hz"); ax[1, k].legend(fontsize=8)
fig.suptitle(f"{TITLE}: where the high-frequency content is (walking window)"); fig.tight_layout(); fig.savefig(OUT("spectra.png"), dpi=80); plt.close(fig)

# ---- feedforward (MPC tau_ff) vs feedback (servo PD = applied - tau_ff) ----
fig, ax = plt.subplots(5, 1, figsize=(14, 13), sharex=False)
groups = (("legs", LEGS), ("waist", WAIST), ("arms", ARMS))
labels = [g for g, _ in groups]
ff_rms = [diag[g]["cmd_tau_rms"] for g in labels]; fb_rms = [diag[g]["pd_tau_rms"] for g in labels]
ff_hf = [diag[g]["cmd_tau_hf"] for g in labels]; fb_hf = [diag[g]["pd_tau_hf"] for g in labels]
xx = np.arange(len(labels))
ax[0].bar(xx - 0.3, ff_rms, 0.2, label="FF rms"); ax[0].bar(xx - 0.1, fb_rms, 0.2, label="FB rms")
ax[0].bar(xx + 0.1, ff_hf, 0.2, label="FF 4-40 Hz"); ax[0].bar(xx + 0.3, fb_hf, 0.2, label="FB 4-40 Hz")
ax[0].set_xticks(xx); ax[0].set_xticklabels(labels); ax[0].set_ylabel("[Nm], mean over joints"); ax[0].legend(fontsize=8, ncol=4)
for i, j in enumerate(["left_hip_roll_joint", "left_knee_joint", "left_ankle_roll_joint", "left_shoulder_pitch_joint"], start=1):
    ax[i].plot(t[zoom], c("cmd_tau/" + j)[zoom], label="FF (MPC tau_ff)")
    ax[i].plot(t[zoom], (c("meas_tau/" + j) - c("cmd_tau/" + j))[zoom], label="FB (servo PD)")
    ax[i].set_ylabel(j.replace("_joint", "").replace("left_", "L ") + " [Nm]", fontsize=8); ax[i].legend(fontsize=7, loc="upper right")
ax[-1].set_xlabel("sim time [s]")
fig.suptitle(f"{TITLE}: feedforward vs feedback torque"); fig.tight_layout(); fig.savefig(OUT("ff_fb.png"), dpi=80); plt.close(fig)

# ---- solver metrics (per solve) ----
if "solver" in d.files and d["solver"].size and len(d["solver_names"]):
    sn = list(d["solver_names"]); S = d["solver"]; sc = {n: i for i, n in enumerate(sn)}
    st = S[:, sc["t"]]
    sw = (st >= w0) & (st <= w1)
    if sw.sum() > 20:
        adv = S[sw, sc["advance_ms"]]
        diag["solver"] = {"solves": int(sw.sum()), "solve_hz": round(float(sw.sum() / (st[sw][-1] - st[sw][0])), 1),
                          "advance_ms_mean": round(float(adv.mean()), 2), "advance_ms_p95": round(float(np.percentile(adv, 95)), 2),
                          "advance_ms_max": round(float(adv.max()), 2), "eq_sse_mean": float(np.nanmean(S[sw, sc["eq_sse"]])),
                          "dyn_sse_max": float(np.nanmax(S[sw, sc["dyn_sse"]])), "cost_mean": round(float(np.nanmean(S[sw, sc["cost"]])), 3)}
        cost_cols = [n for n in sn if "/" in n and n.split("/")[0] in ("cost", "state_cost", "soft", "state_soft")]
        # Raw means, largest magnitude first. Soft constraints use relaxed log barriers, which are negative
        # well inside the feasible set, so a "share of the total" would be meaningless.
        means = {n: float(np.nanmean(S[sw, sc[n]])) for n in cost_cols}
        diag["solver"]["cost_terms_mean"] = {n: round(v, 3) for n, v in sorted(means.items(), key=lambda kv: -abs(kv[1]))[:12]}
        if "eq_sse_near" in sc:
            diag["solver"]["eq_sse_near_mean"] = float(np.nanmean(S[sw, sc["eq_sse_near"]]))
            near = [n for n in sn if n.startswith("eq_sse_near/")]
            diag["solver"]["eq_sse_near_by_term_mean"] = {n[12:]: float(np.nanmean(S[sw, sc[n]])) for n in sorted(near, key=lambda n: -np.nanmean(S[sw, sc[n]]))[:4]}
        eqc = [n for n in sn if n.startswith("eq_sse/")]
        diag["solver"]["eq_sse_by_term_mean"] = {n[7:]: float(np.nanmean(S[sw, sc[n]])) for n in sorted(eqc, key=lambda n: -np.nanmean(S[sw, sc[n]]))[:6]}
        fig, ax = plt.subplots(4, 1, figsize=(14, 12), sharex=True)
        ax[0].plot(st[sw], adv, lw=0.8); ax[0].axhline(10.0, color="r", ls="--", lw=0.8, label="10 ms (100 Hz)")
        ax[0].set_ylabel("solve time [ms]"); ax[0].legend(fontsize=8)
        ax[1].semilogy(st[sw], S[sw, sc["eq_sse"]], label="equality SSE"); ax[1].semilogy(st[sw], S[sw, sc["dyn_sse"]] + 1e-30, label="dynamics SSE")
        for n in sorted(eqc, key=lambda n: -np.nanmean(S[sw, sc[n]]))[:4]:
            ax[1].semilogy(st[sw], S[sw, sc[n]] + 1e-30, lw=0.6, label=n[7:])
        if "eq_sse_near" in sc:
            ax[1].semilogy(st[sw], S[sw, sc["eq_sse_near"]] + 1e-30, color="k", lw=1.0, label="equality SSE, first 3 nodes")
        ax[1].set_ylabel("constraint violation"); ax[1].legend(fontsize=7, ncol=3)
        ax[2].plot(st[sw], S[sw, sc["cost"]], label="total cost (horizon)"); ax[2].set_ylabel("cost"); ax[2].legend(fontsize=8)
        for n in list(diag["solver"]["cost_terms_mean"])[:8]:
            ax[3].plot(st[sw], S[sw, sc[n]], lw=0.8, label=n.split("/", 1)[1][:40])
        ax[3].set_ylabel("cost terms at node 0"); ax[3].set_xlabel("sim time [s]"); ax[3].legend(fontsize=7, ncol=2)
        fig.suptitle(f"{TITLE}: solver"); fig.tight_layout(); fig.savefig(OUT("solver.png"), dpi=80); plt.close(fig)
json.dump(diag, open(OUT("diag.json"), "w"), indent=1)

L, A, B = diag["legs"], diag["arms"], diag["base"]
print(f"DIAG {TITLE} | base z sd plan {B['plan_z_sd_mm']} / obs {B['obs_z_sd_mm']} mm, vz rms plan {B['plan_vz_rms']} / obs {B['obs_vz_rms']}"
      f" | legs PD peak p99.9 {L['pd_tau_p999']} Nm max {list(L['pd_tau_max'].items())[0]}"
      f" sat {L.get('tau_sat_pct', 'n/a')}% of ticks ({L.get('tau_sat_worst', '-')} @ {L.get('tau_sat_limit', '-')} Nm)"
      f" | legs q err {L['track_err_q_deg']}deg, q_hf cmd {L['cmd_q_hf_deg']} meas {L['meas_q_hf_deg']}, tau_hf ff {L['cmd_tau_hf']} pd {L['pd_tau_hf']}, jump@switch {L['cmd_tau_jump_at_switch']} vs {L['cmd_tau_step_otherwise']}"
      f" | arms q err {A['track_err_q_deg']}deg, q_hf cmd {A['cmd_q_hf_deg']} meas {A['meas_q_hf_deg']}, tau_hf ff {A['cmd_tau_hf']} pd {A['pd_tau_hf']}, jump@switch {A['cmd_tau_jump_at_switch']} vs {A['cmd_tau_step_otherwise']}"
      f" | switches {diag['policy_switch_hz']} Hz"
      f" | gait {diag['gait']['sequence']} {diag['gait']['templates_s']} transitions {diag['gait']['changes_locomotion']}"
      + (f" (acc_hf max {diag['gait']['transition_acc_hf_max']}, z pp {diag['gait']['transition_z_pp_mm_max']} mm,"
         f" roll max {diag['gait']['transition_roll_max_deg']} deg, 1st-step asym {diag['gait']['first_step_asym_max']})"
         if diag['gait']['changes_locomotion'] else "")
      + f" | cmd transients ({diag['command']['profile']}, {len(diag['command']['events'])} events) acc_hf max {diag['command']['transient_acc_hf_max']},"
        f" z pp {diag['command']['transient_z_pp_mm_max']} mm, pitch max {diag['command']['transient_pitch_max_deg']} deg"
      + (f" | solve {diag['solver']['advance_ms_mean']} ms mean, {diag['solver']['advance_ms_p95']} p95, {diag['solver']['solve_hz']} Hz, eq_sse {diag['solver']['eq_sse_mean']:.1e}"
         f" (first 3 nodes {diag['solver'].get('eq_sse_near_mean', float('nan')):.1e})" if "solver" in diag else ""))
