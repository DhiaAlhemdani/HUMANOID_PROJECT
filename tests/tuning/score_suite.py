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
  leg_acc      leg joint acceleration RMS, 0-15 Hz [rad/s^2], mean over hip/knee/ankle joints (measured positions):
               the limb-side abruptness the pelvis numbers do not see (a force-rate cost can move it there)
  leg_jerk     leg joint jerk RMS, 0-15 Hz [rad/s^3]
  arm_acc      arm joint acceleration RMS, 0-15 Hz [rad/s^2], shoulder/elbow joints
  arm_pp       shoulder-pitch swing amplitude, peak-to-peak [deg], mean of both arms (arm sway)
  tau_hf       leg measured torque, 4-40 Hz band RMS [Nm]
  v_err        command tracking error RMS (|v_cmd - v_pelvis| planar, after the ramp) [m/s]; the command is
               rounded like the controller does when <case>/config.yaml enables ocs2.gait.continuous
  z_m          mean pelvis height over the segment [m] (ground truth; against the commanded height: the
               walking height is a cost compromise, not a tracked setpoint)
  step_t/l     mean time between step starts [s] and pelvis travel per step [m] (plan_mode; the leg proxy:
               human-scaled targets 0.36 m at 0.45 m/s, 0.44 m at 0.8, 0.56 m at 1.15), ds = double-support fraction
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
K = d["script"]                                   # [t, vx, vy, yaw(, height)] on the script clock
HCMD = K[:, 4] if K.shape[1] == 5 else None       # commanded pelvis height per knot (height suite script)
K = K[:, :4]
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
# Solver-row telemetry (per solve): contactSync counters and the terrain speed cap, when the build has them.
SN = [str(n) for n in d["solver_names"]] if "solver_names" in d.files else []
SV = d["solver"] if SN and "solver" in d.files and d["solver"].size else None
sc = lambda n: SV[:, SN.index(n)] if SV is not None and n in SN else None


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
# A height change (5th column) with the velocity held is a transition too ("ramp h -> ..."): the slider moved.
segs = []
for i, (a, b) in enumerate(zip(K[:-1], K[1:])):
    ha, hb = (HCMD[i], HCMD[i + 1]) if HCMD is not None else (0.0, 0.0)
    htag = f" h {hb:.3f}" if HCMD is not None else ""
    if np.allclose(a[1:], b[1:]) and abs(hb - ha) < 1e-6:
        if b[0] - a[0] >= 2.5:
            segs.append(dict(kind="steady", t0=a[0] + 1.5, t1=b[0], cmd=list(a[1:]), label=f"hold vx {a[1]:+.1f} vy {a[2]:+.1f} w {a[3]:+.1f}{htag}"))
    elif np.allclose(a[1:], b[1:]):
        segs.append(dict(kind="transition", t0=a[0], t1=b[0] + 2.0, cmd=list(b[1:]),
                         label=f"ramp h -> {hb:.3f} at vx {b[1]:+.1f}"))
    else:
        segs.append(dict(kind="transition", t0=a[0], t1=b[0] + 2.0, cmd=list(b[1:]),
                         label=f"ramp -> vx {b[1]:+.1f} vy {b[2]:+.1f} w {b[3]:+.1f}{htag}"))

# ---- gait names from plan_mode ---------------------------------------------------------------
MODE = {0: "FLY", 1: "RF", 2: "LF", 3: "STANCE"}
legs = [n.split("/", 1)[1] for n in TN if n.startswith("meas_tau/") and any(k in n for k in ("hip", "knee", "ankle"))]
jt = tc("t")
yaw_tr = tc("obs_base/yaw")

# ---- limb motion from the measured joint positions (1 kHz) --------------------------------------
JS_T = d["js_t"] if "js_t" in d.files and d["js_t"].size else None
JS_Q = d["js_q"] if JS_T is not None else None
JS_N = list(d["js_names"]) if JS_T is not None else []
if JS_T is not None:
    o = np.argsort(JS_T)
    JS_T, JS_Q = JS_T[o], JS_Q[o]
    keep = np.concatenate([[True], np.diff(JS_T) > 1e-6])
    JS_T, JS_Q = JS_T[keep], JS_Q[keep]
LEG_J = [i for i, n in enumerate(JS_N) if any(k in n for k in ("hip", "knee", "ankle"))]
ARM_J = [i for i, n in enumerate(JS_N) if any(k in n for k in ("shoulder", "elbow"))]
SHOULDER_PITCH = [i for i, n in enumerate(JS_N) if "shoulder_pitch" in n]


def limb_metrics(s0, s1):
    """Joint-space acceleration / jerk RMS (0-15 Hz) over a segment; empty dict without joint states."""
    if JS_T is None:
        return {}
    m = (JS_T >= s0) & (JS_T <= s1)
    if m.sum() < 100:
        return {}
    t, q = JS_T[m], JS_Q[m]
    fs = 200.0
    tu = np.arange(t[0], t[-1], 1.0 / fs)
    out = {}
    for key, idx in (("leg", LEG_J), ("arm", ARM_J)):
        if not idx:
            continue
        accs, jerks = [], []
        for i in idx:
            qu = np.interp(tu, t, q[:, i])
            v = np.gradient(qu, tu)
            a = np.gradient(v, tu)
            accs.append(band(tu, a, 0, 15, fs))
            jerks.append(band(tu, np.gradient(a, tu), 0, 15, fs))
        out[f"{key}_acc"] = float(np.mean(accs))
        out[f"{key}_jerk"] = float(np.mean(jerks))
    if SHOULDER_PITCH:
        out["arm_pp"] = float(np.mean([math.degrees(np.ptp(q[:, i])) for i in SHOULDER_PITCH]))
    return out

# ---- human-likeness: phase-averaged joint curves against the normative human gait ---------------
# For each walking segment the measured hip / knee / ankle angles are binned over each leg's gait-cycle
# phase (the planned mode schedule time-warped onto the human cycle, see human_gait.py) and compared with
# the human curves at the segment's own step length (scale = step_l / leg length / 0.78). Reported:
#   hl_leg    RMS shape deviation of hip+knee+ankle (mean removed), both legs, over the cycle [deg]; 0 = the
#             human motion, the per-joint offsets (crouch) are in knee_ms
#   knee_ms   knee flexion at mid stance [deg] (humans ~5-10; the crouch indicator)
#   hip_rom   hip pitch range over the cycle [deg] (humans ~40 at 1.3 m/s, ~30 at 0.5)
#   knee_sw   peak swing knee flexion [deg] (humans ~60)
#   arm_amp   shoulder pitch swing amplitude [deg] (humans ~15-25 at 1.3 m/s, scaling with speed)
#   arm_ph    shoulder swing phase error against the human (arm back at the own leg's heel strike) [deg]
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
try:
    import human_gait as HG
except Exception as e:  # noqa: BLE001
    HG = None
    print(f"human_gait.py not importable ({e}); no human-likeness score")
HL_JOINTS = {}
if HG is not None and JS_T is not None:
    for leg, side in ((0, "left"), (1, "right")):
        for key, name in (("hip", f"{side}_hip_pitch_joint"), ("knee", f"{side}_knee_joint"), ("ankle", f"{side}_ankle_pitch_joint"),
                          ("shoulder", f"{side}_shoulder_pitch_joint")):
            if name in JS_N:
                HL_JOINTS[(leg, key)] = JS_N.index(name)
pm_series = tc("plan_mode")
SCHED = HG.schedule_from_plan_mode(jt, pm_series) if (HG is not None and pm_series is not None and jt is not None) else None


def human_likeness(s0, s1, step_l):
    """Human-likeness numbers for a segment; {} without joint states / schedule / a measured step length."""
    if SCHED is None or not HL_JOINTS or not np.isfinite(step_l):
        return {}
    m = (JS_T >= s0) & (JS_T <= s1)
    if m.sum() < 200:
        return {}
    t = JS_T[m][::5]  # 200 Hz is plenty for a 20-bin phase average
    q = JS_Q[m][::5]
    scale = min(max(step_l / HG.G1_LEG_LENGTH / HG.HUMAN_STEP_RATIO, 0.0), 1.2)
    ph_bins = (np.arange(20) + 0.5) / 20.0
    dev, knee_ms, hip_rom, knee_sw, arm_amp, arm_ph = [], [], [], [], [], []
    for leg in (0, 1):
        valid, ph, _ = HG.phase_series(SCHED[0], SCHED[1], leg, t)
        if valid.sum() < 100:
            continue
        hip, knee, ankle = HG.leg_template_rad(ph_bins, scale, knee_offset=0.0)  # offsets are the robot's choice, compare the motion
        ref = {"hip": -np.degrees(hip), "knee": np.degrees(knee), "ankle": -np.degrees(ankle)}  # back in human sign conventions
        meas = {}
        for key, sign in (("hip", -1.0), ("knee", 1.0), ("ankle", -1.0)):
            j = HL_JOINTS.get((leg, key))
            if j is None:
                continue
            meas[key] = HG.phase_average(ph[valid], sign * np.degrees(q[valid, j]))
        if len(meas) < 3:
            continue
        # the deviation is about the motion (mean removed); the stance offset (crouch) is knee_ms
        for key in ("hip", "knee", "ankle"):
            r, mm = ref[key], meas[key]
            ok = np.isfinite(mm)
            dev.append(np.mean(((mm[ok] - np.mean(mm[ok])) - (r[ok] - np.mean(r[ok]))) ** 2))
        knee_ms.append(float(meas["knee"][6]))       # 30-35 % of the cycle: mid stance
        hip_rom.append(float(np.nanmax(meas["hip"]) - np.nanmin(meas["hip"])))
        knee_sw.append(float(np.nanmax(meas["knee"][12:18])))   # 60-90 %: swing
        j = HL_JOINTS.get((leg, "shoulder"))
        if j is not None:
            sh = HG.phase_average(ph[valid], np.degrees(q[valid, j]))
            if np.isfinite(sh).sum() >= 10:
                c = np.cos(2 * np.pi * ph_bins)
                sn = np.sin(2 * np.pi * ph_bins)
                ok = np.isfinite(sh)
                a = 2 * np.nanmean((sh[ok] - np.nanmean(sh[ok])) * c[ok])
                b = 2 * np.nanmean((sh[ok] - np.nanmean(sh[ok])) * sn[ok])
                arm_amp.append(float(math.hypot(a, b)))          # first-harmonic amplitude
                if math.hypot(a, b) > 1.0:
                    arm_ph.append(float(abs(math.degrees(math.atan2(b, a)))))  # 0 = arm back at own heel strike (shoulder pitch +)
    if not dev:
        return {}
    out = {"hl_leg": float(np.sqrt(np.mean(dev))), "knee_ms": float(np.mean(knee_ms)), "hip_rom": float(np.mean(hip_rom)),
           "knee_sw": float(np.mean(knee_sw))}
    if arm_amp:
        out["arm_amp"] = float(np.mean(arm_amp))
        out["arm_ph"] = float(np.mean(arm_ph))
    return out

# ---- steps from plan_mode: a step starts when a single support begins -------------------------
# step_t = time to the next step start, step_l = planar pelvis displacement over it (the leg proxy: what the
# eye reads as walking is the step length, pelvis jerk does not see it), ds = planned double-support fraction.
pm_all = tc("plan_mode")
step_t0, step_len = np.array([]), np.array([])
if pm_all is not None and jt is not None:
    ok = np.isfinite(pm_all)
    pmv, pmt = pm_all[ok].astype(int), jt[ok]
    starts = pmt[1:][(np.isin(pmv[1:], (1, 2))) & (pmv[1:] != pmv[:-1])]
    if len(starts) > 1:
        px = np.interp(starts, gt[:, 0], gt[:, 2])
        py = np.interp(starts, gt[:, 0], gt[:, 3])
        step_t0 = starts[:-1]
        step_dur = np.diff(starts)
        step_len = np.hypot(np.diff(px), np.diff(py))

# ---- the command the controller tracks: the continuous law rounds slow commands ------------------
ROUND = None
cfg_path = OUT("config.yaml")
if os.path.isfile(cfg_path):
    try:
        import yaml
        c = yaml.safe_load(open(cfg_path))["humanoid_whole_body_inverse_dynamics_mpc_controller"]["ros__parameters"]["ocs2"]["gait"].get("continuous", {})
        if c.get("enabled") and float(c.get("minWalkSpeed", 0)) > 0:
            ROUND = (float(c.get("standBelow", 0)), float(c["minWalkSpeed"]))
    except Exception as e:  # noqa: BLE001
        print(f"config.yaml not parsed ({e}); v_err against the raw command")


def tracked_cmd(cmd):
    if ROUND is None:
        return cmd
    v = math.hypot(cmd[0], cmd[1])
    if v < ROUND[0]:
        return [0.0, 0.0, cmd[2]]
    if 0 < v < ROUND[1]:
        return [cmd[0] * ROUND[1] / v, cmd[1] * ROUND[1] / v, cmd[2]]
    return cmd

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
             z_pp_mm=1000 * float(np.ptp(g[:, 4])), z_m=float(np.mean(g[:, 4])))
    # command tracking after the ramp (steady: whole segment; transition: last 2 s)
    tail = tt >= (s0 if s["kind"] == "steady" else s1 - 2.0)
    cmd = tracked_cmd(s["cmd"])
    if len(step_t0):
        inseg = (step_t0 >= s0) & (step_t0 <= s1) & (step_dur < 3.0)
        if inseg.sum() >= 2:
            r["step_t"] = float(np.mean(step_dur[inseg]))
            r["step_l"] = float(np.mean(step_len[inseg]))
    if pm_all is not None:
        mm = (jt >= s0) & (jt <= s1) & np.isfinite(pm_all)
        if mm.sum() > 50:
            r["ds"] = float(np.mean(pm_all[mm] == 3))
    r["v_err"] = float(np.sqrt(np.mean((fwd_v[tail] - cmd[0]) ** 2 + (lat_v[tail] - cmd[1]) ** 2))) if tail.sum() > 5 else float("nan")
    r.update(limb_metrics(s0, s1))
    if s["kind"] == "steady" and abs(cmd[0]) > 0.05 and abs(cmd[1]) < 0.05 and abs(cmd[2]) < 0.05 and cmd[0] > 0:
        r.update(human_likeness(s0, s1, r.get("step_l", float("nan"))))
    yaw_rate = deriv(tt, np.unwrap(yaw))
    r["w_err"] = float(np.sqrt(np.mean((yaw_rate[tail] - cmd[2]) ** 2))) if tail.sum() > 5 else float("nan")
    cap = sc("terrain/cap")
    if cap is not None:
        st = sc("t")
        ms = (st >= s0) & (st <= s1) & np.isfinite(cap)
        if ms.sum() > 5:
            r["cap_frac"] = float(np.mean(cap[ms] > 0))  # share of solves with the terrain speed cap in force
            steps = sc("terrain/steps")[ms]
            r["terrain_steps"] = int(steps[-1] - steps[0]) if np.isfinite(steps).all() else 0
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
for key in ("jerk", "acc_hf", "lat_acc_hf", "tau_rate", "ang_jerk", "v_err", "z_m", "step_t", "step_l", "leg_acc", "leg_jerk", "arm_acc", "arm_pp",
            "hl_leg", "knee_ms", "hip_rom", "knee_sw", "arm_amp", "arm_ph"):
    sv = [r[key] for r in steady if np.isfinite(r.get(key, np.nan))]
    tv = [r[key] for r in trans if np.isfinite(r.get(key, np.nan))]
    agg[f"steady_{key}_mean"] = round(float(np.mean(sv)), 4) if sv else None
    agg[f"steady_{key}_max"] = round(float(np.max(sv)), 4) if sv else None
    agg[f"transition_{key}_max"] = round(float(np.max(tv)), 4) if tv else None
    agg[f"transition_{key}_mean"] = round(float(np.mean(tv)), 4) if tv else None
if trans:
    worst = max(trans, key=lambda r: r.get("jerk", -1))
    agg["worst_transition"] = worst["label"]
# Cumulative solver-row counters over the whole run (0 when the build has no telemetry).
def _counter(name):
    v = sc(name)
    if v is None:
        return None
    v = v[np.isfinite(v)]
    return int(v[-1] - v[0]) if v.size else 0
agg["contact_sync"] = {k: _counter(f"contact_sync/{k}") for k in ("advanced", "held", "blocked", "descending")}
agg["terrain_steps"] = _counter("terrain/steps")
cap_all = sc("terrain/cap")
agg["cap_frac"] = round(float(np.mean(cap_all[np.isfinite(cap_all)] > 0)), 4) if cap_all is not None and np.isfinite(cap_all).any() else None

json.dump({"title": TITLE, "agg": agg, "segments": rows, "script": d["script"].tolist()}, open(OUT("suite.json"), "w"), indent=1)

print(f"SUITE {TITLE} | survived={agg['survived']} fell_at={agg['fell_at']} | steady ({len(steady)}): jerk {agg['steady_jerk_mean']} "
      f"acc_hf {agg['steady_acc_hf_mean']} tau_rate {agg['steady_tau_rate_mean']} v_err {agg['steady_v_err_mean']} | "
      f"transitions ({len(trans)}): jerk max {agg['transition_jerk_max']} acc_hf max {agg['transition_acc_hf_max']} "
      f"tau_rate max {agg['transition_tau_rate_max']} | worst: {agg.get('worst_transition')}"
      + (" | contact_sync " + " ".join(f"{k} {v}" for k, v in agg["contact_sync"].items())
         if agg["contact_sync"]["advanced"] is not None else "")
      + (f" | terrain steps {agg['terrain_steps']} cap {agg['cap_frac']:.0%} of solves" if agg["cap_frac"] is not None else ""))
print("SEG segment                               kind        jerk  acc_hf lat_hf ang_jrk roll_pp pitch_pp  tau_rate  v_err   z_m step_t step_l   ds leg_acc leg_jrk arm_acc arm_pp  hl_leg knee_ms hip_rom knee_sw arm_amp arm_ph  modes")
for r in rows:
    print(f"SEG {r['label']:36s} {r['kind']:10s} {r['jerk']:6.2f} {r['acc_hf']:6.3f} {r['lat_acc_hf']:6.3f} {r['ang_jerk']:7.2f} "
          f"{r['roll_pp']:6.2f} {r['pitch_pp']:7.2f} {r.get('tau_rate', float('nan')):9.1f} {r['v_err']:6.3f} {r['z_m']:5.3f} "
          f"{r.get('step_t', float('nan')):6.2f} {r.get('step_l', float('nan')):6.2f} {r.get('ds', float('nan')):5.2f} "
          f"{r.get('leg_acc', float('nan')):7.2f} {r.get('leg_jerk', float('nan')):7.1f} {r.get('arm_acc', float('nan')):7.2f} "
          f"{r.get('arm_pp', float('nan')):6.1f} {r.get('hl_leg', float('nan')):7.1f} {r.get('knee_ms', float('nan')):7.1f} "
          f"{r.get('hip_rom', float('nan')):7.1f} {r.get('knee_sw', float('nan')):7.1f} {r.get('arm_amp', float('nan')):7.1f} "
          f"{r.get('arm_ph', float('nan')):6.0f}  {r.get('modes', '')}"
          + (f"  cap {r['cap_frac']:.0%}/{r['terrain_steps']}" if "cap_frac" in r else "")
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
