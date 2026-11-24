#include "legged_robot_mpc_controller/mpc/common/swing_foot_planner/MinimumJerkSplineCpg.h"

namespace ocs2::humanoid {

MinimumJerkSplineCpg::MinimumJerkSplineCpg(
  QuinticSpline::Node liftOff, scalar_t midHeight, QuinticSpline::Node touchDown)
: midTime_((liftOff.time + touchDown.time) / 2.0),
  leftSpline_(liftOff, QuinticSpline::Node{midTime_, midHeight, 0.0, 0.0}),
  rightSpline_(QuinticSpline::Node{midTime_, midHeight, 0.0, 0.0}, touchDown) {}

scalar_t MinimumJerkSplineCpg::position(scalar_t time) const {
  return time < midTime_ ? leftSpline_.position(time) : rightSpline_.position(time);
}

scalar_t MinimumJerkSplineCpg::velocity(scalar_t time) const {
  return time < midTime_ ? leftSpline_.velocity(time) : rightSpline_.velocity(time);
}

scalar_t MinimumJerkSplineCpg::acceleration(scalar_t time) const {
  return time < midTime_ ? leftSpline_.acceleration(time) : rightSpline_.acceleration(time);
}

}  // namespace ocs2::humanoid
