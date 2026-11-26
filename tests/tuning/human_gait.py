"""Normative human walking kinematics and the gait-cycle phase, shared by the human-likeness score.

Mirrors `SwitchedModelReferenceManager::getLegStridePhase` / `addHumanGaitTemplate` (C++): the same curve
tables, the same time-warp of the planned mode schedule onto the human cycle, the same robot joint
conventions. Keep the two in step.

Curves: sagittal joint angles of normal human walking every 5 % of one leg's gait cycle from its heel
strike (Winter-type normative data, degrees): hip and knee flexion positive, ankle dorsiflexion positive.
Toe-off of the leg at 60 %, the other leg's heel strike at 50 %.
"""
import math

import numpy as np

HIP_FLEX_DEG = np.array([30, 29, 26, 22, 17, 12, 7, 2, -3, -7, -10, -10, -6, 2, 12, 21, 28, 32, 33, 32, 30], float)
KNEE_FLEX_DEG = np.array([5, 10, 16, 18, 16, 13, 10, 7, 5, 6, 10, 19, 35, 52, 62, 62, 55, 42, 27, 13, 5], float)
ANKLE_DORSI_DEG = np.array([0, -4, -6, -3, 0, 3, 6, 8, 9, 10, 9, 4, -8, -17, -16, -10, -5, -2, 0, 1, 0], float)
CURVES = {"hip": HIP_FLEX_DEG, "knee": KNEE_FLEX_DEG, "ankle": ANKLE_DORSI_DEG}

HUMAN_STEP_RATIO = 0.78   # step length / leg length at the reference speed
REFERENCE_SPEED = 1.3     # m/s
G1_LEG_LENGTH = 0.65      # hip pitch to ankle [m]
# Shipped reference amplitudes (rad) at REFERENCE_SPEED / full scale
SHOULDER_AMPLITUDE = 0.35
ELBOW_AMPLITUDE = 0.15
WAIST_YAW_AMPLITUDE = 0.08
PELVIS_ROLL_AMPLITUDE = 0.05
HEIGHT_OSCILLATION = 0.015


def human_curve_deg(name, phase):
    """Linear interpolation of a normative curve at phase in [0, 1) (array ok)."""
    c = CURVES[name]
    p = np.asarray(phase, float) % 1.0
    x = p * 20.0
    i = np.minimum(x.astype(int), 19)
    a = x - i
    return (1.0 - a) * c[i] + a * c[i + 1]


def leg_template_rad(phase, scale, knee_offset=0.3, remove_hip_mean=True):
    """Robot (G1 convention) hip pitch, knee, ankle pitch reference for one leg at `phase` and `scale`."""
    d2r = math.pi / 180.0
    half = 0.5 * knee_offset
    hip_mean = float(np.mean(HIP_FLEX_DEG[:20])) if remove_hip_mean else 0.0
    ankle_mean = float(np.mean(ANKLE_DORSI_DEG[:20])) if remove_hip_mean else 0.0
    hip = -(scale * (human_curve_deg("hip", phase) - hip_mean) * d2r) - half
    knee = scale * human_curve_deg("knee", phase) * d2r + knee_offset
    ankle = -(scale * (human_curve_deg("ankle", phase) - ankle_mean) * d2r) - half
    return hip, knee, ankle


def contact(mode, leg):
    """Stance flag of leg 0 (left) / 1 (right) for mode numbers FLY 0, RF 1, LF 2, STANCE 3."""
    return {0: (False, False), 1: (False, True), 2: (True, False), 3: (True, True)}[int(mode)][leg]


def schedule_from_plan_mode(t, pm):
    """Event times and mode sequence from a sampled plan_mode timeline (NaN samples dropped)."""
    ok = np.isfinite(pm)
    t, pm = t[ok], pm[ok].astype(int)
    if len(pm) == 0:
        return np.array([]), np.array([])
    ch = np.flatnonzero(pm[1:] != pm[:-1]) + 1
    return t[ch], np.concatenate([[pm[0]], pm[ch]])


def leg_stride_phase(event_times, modes, leg, time):
    """(valid, phase, step_time) of one leg at `time` — the port of getLegStridePhase."""
    n = len(modes)
    if n < 2 or len(event_times) + 1 != n:
        return False, 0.0, 0.0
    other = 1 - leg

    def inc(i, l):
        return contact(modes[i], l)

    def change_time(i):
        return event_times[i - 1]

    def next_change(frm, l, to_contact):
        for i in range(frm + 1, n):
            if inc(i, l) == to_contact and inc(i - 1, l) != to_contact:
                return i
        return n

    k = int(np.searchsorted(event_times, time, side="right"))
    k = min(k, n - 1)
    latest = n
    for i in range(k, 0, -1):
        if inc(i, leg) != inc(i - 1, leg):
            latest = i
            break
    if latest == n:
        return False, 0.0, 0.0
    heel = n
    if inc(latest, leg):
        heel = latest
    else:
        for i in range(latest - 1, 0, -1):
            if inc(i, leg) and not inc(i - 1, leg):
                heel = i
                break
    if heel == n:
        td = next_change(latest, leg, True)
        if td >= n:
            return False, 0.0, 0.0
        t0, t1 = change_time(latest), change_time(td)
        if not t1 > t0:
            return False, 0.0, 0.0
        return True, min(max(0.6 + 0.4 * (time - t0) / (t1 - t0), 0.6), 1 - 1e-9), (t1 - t0) / 0.8
    hs = change_time(heel)
    i_to_other = next_change(heel, other, False)
    i_hs_other = next_change(i_to_other, other, True) if i_to_other < n else n
    i_to = next_change(heel, leg, False)
    i_hs_next = next_change(i_to, leg, True) if i_to < n else n
    step = float("nan")
    if i_hs_other < n:
        step = change_time(i_hs_other) - hs
    elif i_hs_next < n:
        step = 0.5 * (change_time(i_hs_next) - hs)
    elif i_to < n:
        step = (change_time(i_to) - hs) / 1.2
    if not step > 0:
        return False, 0.0, 0.0
    t_to_other = change_time(i_to_other) if i_to_other < n else hs + 0.2 * step
    t_hs_other = change_time(i_hs_other) if i_hs_other < n else hs + step
    if i_to >= n and i_hs_other < n and time >= t_hs_other:
        return False, 0.0, 0.0  # the other leg lands and this one never leaves the ground again: standing
    t_to = change_time(i_to) if i_to < n else hs + 1.2 * step
    t_hs_next = change_time(i_hs_next) if i_hs_next < n else hs + 2.0 * step
    if time < t_to_other and t_to_other > hs:
        ph = 0.10 * (time - hs) / (t_to_other - hs)
    elif time < t_hs_other and t_hs_other > t_to_other:
        ph = 0.10 + 0.40 * (time - t_to_other) / (t_hs_other - t_to_other)
    elif time < t_to and t_to > t_hs_other:
        ph = 0.50 + 0.10 * (time - t_hs_other) / (t_to - t_hs_other)
    elif t_hs_next > t_to:
        ph = 0.60 + 0.40 * (time - t_to) / (t_hs_next - t_to)
    else:
        return False, 0.0, 0.0
    return True, min(max(ph, 0.0), 1 - 1e-9), step


def phase_series(event_times, modes, leg, times):
    """Vectorised-by-loop phase for an array of times: (valid mask, phase array, step_time array)."""
    valid = np.zeros(len(times), bool)
    ph = np.zeros(len(times))
    st = np.zeros(len(times))
    for j, tt in enumerate(times):
        v, p, s = leg_stride_phase(event_times, modes, leg, tt)
        valid[j], ph[j], st[j] = v, p, s
    return valid, ph, st


def phase_average(phase, values, bins=20):
    """Mean of `values` in `bins` phase bins; NaN where empty."""
    idx = np.minimum((np.asarray(phase) * bins).astype(int), bins - 1)
    out = np.full(bins, np.nan)
    for b in range(bins):
        m = idx == b
        if m.any():
            out[b] = float(np.mean(values[m]))
    return out
