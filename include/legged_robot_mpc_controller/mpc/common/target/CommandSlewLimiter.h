/******************************************************************************
Jerk-limited (S-curve) slew limiter for one axis of the walking-velocity command.

The command path used to be: 5 Hz low-pass -> per-axis rate clamp. On a command step the clamp
dominates, so the reference became a linear ramp with an acceleration *step* at both ends (a jerk
impulse the MPC tracks as a pelvis pitch/height kick). With a jerk limit the rate itself slews, and
the approach to the target is planned so the rate reaches zero at the target (discrete-time
brake cap) — overshoot bounded by jerk*dt^2, no terminal step.

Header-only, no ocs2/Eigen dependency: tests/tuning/run_slew_microbench.py compiles it standalone.
******************************************************************************/

#pragma once

#include <algorithm>
#include <cmath>

namespace ocs2::humanoid {

/**
 * One step of the limiter.
 *
 * @param target       unlimited (filtered, scaled) command for this axis
 * @param previous     last output of this axis
 * @param previousRate [in/out] last output rate of this axis (per second); updated to the new rate
 * @param dt           step [s], >= 0 (0 returns `previous` unchanged)
 * @param rateLimit    max |rate| (e.g. m/s^2 for a velocity axis); <= 0 disables the axis entirely
 * @param jerkLimit    max |rate change| per second; <= 0 keeps the legacy pure rate clamp
 * @return the new output
 */
// SLEW_LIMITER_BEGIN
inline double slewLimitStep(double target, double previous, double& previousRate, double dt, double rateLimit,
                            double jerkLimit) {
  if (rateLimit <= 0.0) {
    previousRate = 0.0;
    return target;
  }
  if (dt <= 0.0) {
    return previous;
  }
  const double remaining = target - previous;
  if (jerkLimit <= 0.0) {
    const double out = std::clamp(target, previous - rateLimit * dt, previous + rateLimit * dt);
    previousRate = (out - previous) / dt;
    return out;
  }
  // Rate the axis would like: get there this step, but never faster than the rate limit and never
  // faster than can still be braked to zero at the target under the jerk limit. The brake cap is
  // the discrete-time one (stopping distance v^2/(2J) + v*dt/2 for Euler steps of dt), so the rate
  // is only ever changed by at most jerk*dt — no terminal clamp, no jerk spike at arrival; the
  // residual overshoot is bounded by jerk*dt^2 and is removed on the next steps.
  const double halfStep = 0.5 * jerkLimit * dt;
  const double brakeable = -halfStep + std::sqrt(halfStep * halfStep + 2.0 * jerkLimit * std::fabs(remaining));
  const double cap = std::max(0.0, std::min(rateLimit, brakeable));
  double rate = std::clamp(remaining / dt, -cap, cap);
  rate = std::clamp(rate, previousRate - jerkLimit * dt, previousRate + jerkLimit * dt);
  const double out = previous + rate * dt;
  previousRate = rate;
  return out;
}
// SLEW_LIMITER_END

}  // namespace ocs2::humanoid
