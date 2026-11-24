#pragma once

#include <ocs2_core/Types.h>

namespace ocs2::humanoid {

/// Quintic Hermite segment with zero endpoint acceleration by default.
/// The boundary velocities remain configurable to preserve the existing
/// lift-off/touch-down behavior while making acceleration continuous.
class QuinticSpline {
 public:
  struct Node {
    scalar_t time;
    scalar_t position;
    scalar_t velocity;
    scalar_t acceleration = 0.0;
  };

  QuinticSpline(Node start, Node end);

  scalar_t position(scalar_t time) const;
  scalar_t velocity(scalar_t time) const;
  scalar_t acceleration(scalar_t time) const;

 private:
  scalar_t normalizedTime(scalar_t time) const;

  scalar_t t0_;
  scalar_t dt_;
  scalar_t c0_;
  scalar_t c1_;
  scalar_t c2_;
  scalar_t c3_;
  scalar_t c4_;
  scalar_t c5_;
};

}  // namespace ocs2::humanoid
