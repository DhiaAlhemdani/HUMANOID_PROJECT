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

  // 10 deg ramp (gradient 0.176 along x), walking up at 0.2 m/s; solve at t = 0.1, left foot in the air.
  // Feet 0.2 m apart sideways (left at +y): with the gradient along x the stance width adds no height.
  SlopeFootReferenceInput in;
  in.initTime = 0.1;
  in.gradient = vector2_t(0.176, 0.0);
  in.velocity = vector2_t(0.2, 0.0);
  in.contacts = {false, true};
  in.solePositions = {vector2_t(-0.05, 0.1), vector2_t(0.10, -0.1)};  // right (stance) is the leading foot
  in.soleHeights = {-0.0088, 0.0176};                                  // both on the plane z = 0.176 x
  in.touchDownHeightOffset = -0.001;

  // Walking up: footfalls v x dt further along, heights slope x that.
  {
    const auto plan = planSlopeFootReference(in, events, contacts);
    assert(plan.strideVelocity == vector2_t(0.2, 0.0));
    assert(near(plan.anchorPosition(0), 0.10) && near(plan.anchorHeight, 0.0176));
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
    // Left lift-off (in the air now): where it last touched down, one footfall before the anchor's
    // (-0.25 - 0.72, before the schedule): one stride x slope BELOW the anchor, not at its lift-off time.
    const double leftLo = 0.0176 - 0.176 * 0.2 * 0.72;
    assert(near(plan.liftOffHeights[0][2], leftLo));
    assert(leftLo < plan.anchorHeight);
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
  // The air foot's own last touchdown in the schedule wins over the one-period guess: solve at 1.0 (right
  // in the air since 0.72, its touchdown at -0.25 is in the schedule, the anchor is the left foot that landed at 0.47).
  {
    SlopeFootReferenceInput later = in;
    later.initTime = 1.0;
    later.contacts = {true, false};
    later.solePositions = {vector2_t(0.244, 0.1), vector2_t(0.10, -0.1)};
    later.soleHeights = {0.176 * 0.244, 0.0176};
    const auto plan = planSlopeFootReference(later, events, contacts);
    assert(near(plan.anchorTime, 0.47) && near(plan.anchorHeight, 0.176 * 0.244));
    // Right lift-off height: the plane at its touchdown time -0.25 = anchor - slope x v x 0.72.
    assert(near(plan.liftOffHeights[1][4], plan.anchorHeight - 0.176 * 0.2 * 0.72));
    assert(near(plan.touchDownHeights[1][4], plan.anchorHeight + 0.176 * 0.2 * 0.72 - 0.001));
  }
  // Walking down the same ramp (velocity reversed): the leading foot is the rear one.
  {
    SlopeFootReferenceInput down = in;
    down.velocity = vector2_t(-0.2, 0.0);
    down.contacts = {true, true};
    const auto plan = planSlopeFootReference(down, events, contacts);
    assert(near(plan.anchorPosition(0), -0.05) && near(plan.anchorHeight, -0.0088));
    assert(near(plan.anchorTime, 0.47 - 0.72));  // the left foot's last touchdown is before the schedule: one period back
    assert(near(plan.touchDownHeights[0][2], -0.0088 - 0.176 * 0.2 * 0.72 - 0.001));
    // Both stance feet keep their own heights until they step.
    assert(near(plan.liftOffHeights[0][2], -0.0088) && near(plan.liftOffHeights[1][2], 0.0176));
  }
  // Across the slope (run 26): the gradient along y, walking along x. The left track (y = +0.1) is 0.2 x
  // 0.176 = 3.5 cm above the right one; the footfall rule keeps each foot on its own track.
  {
    SlopeFootReferenceInput across = in;
    across.gradient = vector2_t(0.0, 0.176);
    across.soleHeights = {0.0176, -0.0176};  // left at y = +0.1, right at y = -0.1, both on z = 0.176 y
    const auto plan = planSlopeFootReference(across, events, contacts);
    assert(near(plan.anchorHeight, -0.0176));  // the right foot leads along x
    assert(near(plan.touchDownHeights[0][2], 0.0176 - 0.001));   // the left foot lands on its own track: 3.5 cm up
    assert(near(plan.touchDownHeights[1][4], -0.0176 - 0.001));  // the right foot on its own
    assert(near(plan.liftOffHeights[0][2], 0.0176));             // and the left lift-off is its own track too
  }
  // Standing (below standingSpeed): every touchdown at the plane where the foot is.
  {
    SlopeFootReferenceInput stand = in;
    stand.velocity = vector2_t(0.02, 0.0);
    stand.contacts = {true, true};
    const auto plan = planSlopeFootReference(stand, events, contacts);
    assert(plan.strideVelocity.norm() == 0.0);
    assert(near(plan.anchorPosition(0), 0.025) && near(plan.anchorHeight, 0.0044));
    assert(near(plan.touchDownHeights[0][2], -0.0088 - 0.001) && near(plan.touchDownHeights[1][4], 0.0176 - 0.001));
    assert(near(plan.liftOffHeights[1][0], 0.0176));  // a stance foot still holds its own height
  }
  // Turn in place on a cross-slope (run 26): the gradient along y, the feet side by side at y = +-0.1 about the
  // base, yaw rate 0.5 rad/s. A foot swung by the turn lands where the rotation takes it: after 90 deg both
  // feet are on the y = 0 line, at the plane's height there (0), not at their current +-1.76 cm.
  {
    SlopeFootReferenceInput turn;
    turn.initTime = 0.1;
    turn.gradient = vector2_t(0.0, 0.176);
    turn.velocity = vector2_t::Zero();
    turn.yawRate = M_PI_2 / (0.47 - 0.1);  // a quarter turn by the left foot's touchdown at 0.47
    turn.basePosition = vector2_t(0.0, 0.0);
    turn.contacts = {false, true};
    turn.solePositions = {vector2_t(0.0, 0.1), vector2_t(0.0, -0.1)};
    turn.soleHeights = {0.0176, -0.0176};
    const auto plan = planSlopeFootReference(turn, events, contacts);
    assert(near(plan.touchDownHeights[0][2], 0.0));                   // left lands at (-0.1, 0): plane height 0
    assert(near(plan.liftOffHeights[0][2], 0.0176));                  // its lift-off: the plane where it is
    assert(near(plan.liftOffHeights[1][0], -0.0176));                 // the stance right foot: its own height
    const double angle = turn.yawRate * (1.19 - 0.1);  // the right foot keeps turning until its touchdown at 1.19
    const double rightY = -0.1 * std::cos(angle);        // rotating (0, -0.1) by `angle`
    assert(near(plan.touchDownHeights[1][4], 0.176 * rightY));
    // Without a yaw rate the feet land where they are.
    turn.yawRate = 0.0;
    const auto still = planSlopeFootReference(turn, events, contacts);
    assert(near(still.touchDownHeights[0][2], 0.0176) && near(still.touchDownHeights[1][4], -0.0176));
  }
  // Flat (zero gradient): every height is the anchor's, i.e. the stance feet's own heights and the anchor for landings.
  {
    SlopeFootReferenceInput flat = in;
    flat.gradient.setZero();
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
    assert(near(plan.anchorPosition(0), 0.025));
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
  // The plane normal: tilted against the rise, unit length, in the world frame.
  {
    const vector3_t n = slopePlaneNormal(vector2_t(0.176, 0.0));
    assert(near(n.norm(), 1.0) && n(0) < 0.0 && near(n(1), 0.0) && n(2) > 0.98);
    assert(near(std::atan2(-n(0), n(2)), std::atan(0.176)));  // pitched by the ramp angle
    const vector3_t ny = slopePlaneNormal(vector2_t(0.0, 0.176));
    assert(near(ny(0), 0.0) && ny(1) < 0.0);
    assert(near((slopePlaneNormal(vector2_t::Zero()) - vector3_t(0.0, 0.0, 1.0)).norm(), 0.0));
  }
  return 0;
}
