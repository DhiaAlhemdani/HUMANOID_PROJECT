#pragma once

#include "legged_robot_mpc_controller/mpc/common/swing_foot_planner/QuinticSpline.h"

namespace ocs2::humanoid {

/// Two-piece minimum-jerk swing-height profile with a stationary, C2-continuous apex.
class MinimumJerkSplineCpg {
 public:
  MinimumJerkSplineCpg(QuinticSpline::Node liftOff, scalar_t midHeight, QuinticSpline::Node touchDown);

  scalar_t position(scalar_t time) const;
  scalar_t velocity(scalar_t time) const;
  scalar_t acceleration(scalar_t time) const;

 private:
  scalar_t midTime_;
  QuinticSpline leftSpline_;
  QuinticSpline rightSpline_;
};

}  // namespace ocs2::humanoid
