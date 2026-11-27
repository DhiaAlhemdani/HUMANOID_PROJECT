// Regression test for the incline foot references (SlopeFootReference.h): asserts, no framework.
// Built by the package tests (CMakeLists.txt) with assertions kept active.

#include <cassert>
#include <cmath>
#include <stdexcept>
#include <vector>

#include <legged_robot_mpc_controller/mpc/common/gait/SlopeFootReference.h>

int main() {
  using namespace ocs2;
  using namespace ocs2::humanoid;
  const auto near = [](double a, double b, double tol = 1e-9) { return std::abs(a - b) < tol; };

  // A walk schedule, period 0.72 s (0.47 s single support, 0.25 s double). Phase p spans
  // [events[p-1], events[p]]; left foot = 0, right foot = 1. Right lands at -0.25, left lifts at 0.0 and
  // lands at 0.47, right lifts at 0.72 and lands at 1.19, left lifts at 1.44 and lands at 1.91.
  const scalar_array_t events{-0.25, 0.0, 0.47, 0.72, 1.19, 1.44, 1.91};  // 8 phases
  const std::vector<bool> leftContact{true, true, false, true, true, true, false, true};
  const std::vector<bool> rightContact{false, true, true, true, false, true, true, true};
  const feet_array_t<std::vector<bool>> contacts{leftContact, rightContact};

  // 10 deg ramp (slope 0.176), walking up at 0.2 m/s; solve at t = 0.1, left foot in the air.
  SlopeFootReferenceInput in;
  in.initTime = 0.1;
  in.slope = 0.176;
  in.velocityAlong = 0.2;
  in.contacts = {false, true};
  in.soleAlong = {-0.05, 0.10};      // right (stance) is the leading foot
  in.soleHeights = {-0.0088, 0.0176};  // both on the plane z = 0.176 * along
  in.touchDownHeightOffset = -0.001;

  // Walking up: footfalls v x dt further along, heights slope x that.
  {
    const auto plan = planSlopeFootReference(in, events, contacts);
    assert(plan.strideVelocity == 0.2);
    assert(near(plan.anchorAlong, 0.10) && near(plan.anchorHeight, 0.0176));
    assert(near(plan.anchorTime, -0.25));  // the right foot's touchdown, from the schedule's past
    for (size_t foot = 0; foot < 2; ++foot) {
      assert(plan.liftOffHeights[foot].size() == 8 && plan.touchDownHeights[foot].size() == 8);
    }
    // Stance right foot: its own height everywhere before its next touchdown.
    for (size_t p = 0; p < 5; ++p) {
      assert(near(plan.liftOffHeights[1][p], 0.0176));
    }
    // Left touchdown at 0.47 (event 2): anchor + slope x v x (0.47 - (-0.25)).
    const double leftTd = 0.0176 + 0.176 * 0.2 * 0.72;
    assert(near(plan.touchDownHeights[0][2], leftTd - 0.001));
    for (size_t p = 3; p < 7; ++p) {
      assert(near(plan.liftOffHeights[0][p], leftTd));  // its stance after landing, and the next lift-off
    }
    assert(near(plan.liftOffHeights[0][7], 0.0176 + 0.176 * 0.2 * 2.16));  // the stance after its second landing
    // Left lift-off (in the air now): the previous footfall, at its lift-off time 0.0.
    const double leftLo = 0.0176 + 0.176 * 0.2 * 0.25;
    assert(near(plan.liftOffHeights[0][2], leftLo));
    // Right touchdown at 1.19 (event 4), then its stance; left again at 1.91 (event 6).
    const double rightTd = 0.0176 + 0.176 * 0.2 * 1.44;
    assert(near(plan.touchDownHeights[1][4], rightTd - 0.001));
    assert(near(plan.liftOffHeights[1][5], rightTd) && near(plan.liftOffHeights[1][7], rightTd));
    assert(near(plan.touchDownHeights[0][6], 0.0176 + 0.176 * 0.2 * 2.16 - 0.001));
    // Each landing is one stride x slope above the previous one of the other foot.
    assert(near(rightTd - leftTd, 0.176 * 0.2 * 0.72));
    // Past touchdowns are not re-planned (phase 0 is the right foot's past swing).
    assert(near(plan.touchDownHeights[1][0], 0.0176 - 0.001));
  }
  // Walking down the same ramp (heading unchanged, velocity negative): the leading foot is the rear one.
  {
    SlopeFootReferenceInput down = in;
    down.velocityAlong = -0.2;
    down.contacts = {true, true};
    const auto plan = planSlopeFootReference(down, events, contacts);
    assert(near(plan.anchorAlong, -0.05) && near(plan.anchorHeight, -0.0088));
    assert(near(plan.anchorTime, 0.47 - 0.72));  // the left foot's last touchdown is before the schedule: one period back
    assert(near(plan.touchDownHeights[0][2], -0.0088 - 0.176 * 0.2 * 0.72 - 0.001));
    // Both stance feet keep their own heights until they step.
    assert(near(plan.liftOffHeights[0][2], -0.0088) && near(plan.liftOffHeights[1][2], 0.0176));
  }
  // Standing (below standingSpeed): every touchdown at the feet midpoint height.
  {
    SlopeFootReferenceInput stand = in;
    stand.velocityAlong = 0.02;
    stand.contacts = {true, true};
    const auto plan = planSlopeFootReference(stand, events, contacts);
    assert(plan.strideVelocity == 0.0);
    assert(near(plan.anchorAlong, 0.025) && near(plan.anchorHeight, 0.0044));
    assert(near(plan.touchDownHeights[0][2], 0.0044 - 0.001) && near(plan.touchDownHeights[1][4], 0.0044 - 0.001));
    assert(near(plan.liftOffHeights[1][0], 0.0176));  // a stance foot still holds its own height
  }
  // Flat (slope 0): every height is the anchor's, i.e. the stance feet's own heights and the anchor for landings.
  {
    SlopeFootReferenceInput flat = in;
    flat.slope = 0.0;
    flat.soleHeights = {0.0, 0.0};
    const auto plan = planSlopeFootReference(flat, events, contacts);
    for (size_t foot = 0; foot < 2; ++foot) {
      for (size_t p = 0; p < 8; ++p) {
        assert(near(plan.liftOffHeights[foot][p], 0.0) && near(plan.touchDownHeights[foot][p], -0.001));
      }
    }
  }
  // Flight (no contact): anchored at the feet midpoint, nothing throws.
  {
    SlopeFootReferenceInput fly = in;
    fly.contacts = {false, false};
    const auto plan = planSlopeFootReference(fly, events, contacts);
    assert(near(plan.anchorAlong, 0.025));
  }
  // Malformed inputs throw.
  {
    bool threw = false;
    try {
      planSlopeFootReference(in, scalar_array_t{0.0}, contacts);
    } catch (const std::invalid_argument&) {
      threw = true;
    }
    assert(threw);
    threw = false;
    try {
      planSlopeFootReference(in, events, feet_array_t<std::vector<bool>>{leftContact, std::vector<bool>{true}});
    } catch (const std::invalid_argument&) {
      threw = true;
    }
    assert(threw);
  }
  // The plane normal: tilted against the rise, unit length, heading-aware.
  {
    const vector3_t n = slopePlaneNormal(0.176, 0.0);
    assert(near(n.norm(), 1.0) && n(0) < 0.0 && near(n(1), 0.0) && n(2) > 0.98);
    assert(near(std::atan2(-n(0), n(2)), std::atan(0.176)));  // pitched by the ramp angle
    const vector3_t ny = slopePlaneNormal(0.176, M_PI_2);
    assert(near(ny(0), 0.0) && ny(1) < 0.0);
    const vector3_t back = slopePlaneNormal(0.176, M_PI);  // facing down the same ramp: slope reads negative along the heading
    const vector3_t same = slopePlaneNormal(-0.176, 0.0);
    assert(near(back(0), same(0)) && near(back(2), same(2)));
    assert(near((slopePlaneNormal(0.0, 1.0) - vector3_t(0.0, 0.0, 1.0)).norm(), 0.0));
  }
  return 0;
}
