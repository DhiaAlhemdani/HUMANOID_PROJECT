#include "legged_robot_mpc_controller/mpc/common/swing_foot_planner/QuinticSpline.h"

#include <cmath>
#include <stdexcept>

namespace ocs2::humanoid {

QuinticSpline::QuinticSpline(Node start, Node end) : t0_(start.time), dt_(end.time - start.time), c0_(start.position) {
  if (!std::isfinite(start.time) || !std::isfinite(end.time) || !(dt_ > 0.0) ||
      !std::isfinite(start.position) || !std::isfinite(end.position) ||
      !std::isfinite(start.velocity) || !std::isfinite(end.velocity) ||
      !std::isfinite(start.acceleration) || !std::isfinite(end.acceleration)) {
    throw std::invalid_argument("QuinticSpline requires finite nodes with strictly increasing times");
  }

  const scalar_t c1 = start.velocity * dt_;
  const scalar_t c2 = 0.5 * start.acceleration * dt_ * dt_;
  const scalar_t positionResidual = end.position - c0_ - c1 - c2;
  const scalar_t velocityResidual = end.velocity * dt_ - c1 - 2.0 * c2;
  const scalar_t accelerationResidual = end.acceleration * dt_ * dt_ - 2.0 * c2;

  c1_ = c1;
  c2_ = c2;
  c3_ = 10.0 * positionResidual - 4.0 * velocityResidual + 0.5 * accelerationResidual;
  c4_ = -15.0 * positionResidual + 7.0 * velocityResidual - accelerationResidual;
  c5_ = 6.0 * positionResidual - 3.0 * velocityResidual + 0.5 * accelerationResidual;
}

scalar_t QuinticSpline::position(scalar_t time) const {
  const scalar_t s = normalizedTime(time);
  return c0_ + s * (c1_ + s * (c2_ + s * (c3_ + s * (c4_ + s * c5_))));
}

scalar_t QuinticSpline::velocity(scalar_t time) const {
  const scalar_t s = normalizedTime(time);
  return (c1_ + s * (2.0 * c2_ + s * (3.0 * c3_ + s * (4.0 * c4_ + s * 5.0 * c5_)))) / dt_;
}

scalar_t QuinticSpline::acceleration(scalar_t time) const {
  const scalar_t s = normalizedTime(time);
  return (2.0 * c2_ + s * (6.0 * c3_ + s * (12.0 * c4_ + s * 20.0 * c5_))) / (dt_ * dt_);
}

scalar_t QuinticSpline::normalizedTime(scalar_t time) const {
  return (time - t0_) / dt_;
}

}  // namespace ocs2::humanoid
