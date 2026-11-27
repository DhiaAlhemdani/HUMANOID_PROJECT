// Regression test for the blind terrain speed cap (TerrainSpeedCap.h): asserts, no framework.
// Built by the package tests (CMakeLists.txt) with assertions kept active.

#include <cassert>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <legged_robot_mpc_controller/mpc/common/gait/TerrainSpeedCap.h>

int main() {
  using namespace ocs2;
  using namespace ocs2::humanoid;
  const auto near = [](double a, double b) { return std::abs(a - b) < 1e-9; };
  const contact_flag_t both{true, true};
  const contact_flag_t leftOnly{true, false};
  const contact_flag_t rightOnly{false, true};
  const feet_array_t<scalar_t> flat{0.0003, -0.0002};
  const feet_array_t<scalar_t> tile{0.0003, 0.0153};  // right foot on a +1.5 cm tile

  TerrainSpeedCapSettings s;
  s.enabled = true;
  s.stepKnots = {0.01, 0.025};
  s.speedKnots = {0.5, 0.25};
  s.holdSeconds = 3.0;
  validateTerrainSpeedCapSettings(s);

  // Disabled: never caps, whatever the feet report.
  {
    TerrainSpeedCapSettings off = s;
    off.enabled = false;
    TerrainSpeedCap cap(off);
    assert(std::isinf(cap.update(0.0, both, tile)) && !cap.active() && cap.stepCount() == 0);
  }
  // Flat ground: double support with both soles on the plane never trips it.
  {
    TerrainSpeedCap cap(s);
    for (int i = 0; i < 200; ++i) {
      const contact_flag_t c = (i % 20) < 5 ? both : ((i / 20) % 2 ? leftOnly : rightOnly);
      assert(std::isinf(cap.update(0.025 * i, c, flat)));
    }
    assert(!cap.active() && cap.stepCount() == 0 && std::isnan(cap.lastStepTime()));
  }
  // A single support with a large height difference (the swing foot in the air) is not a step.
  {
    TerrainSpeedCap cap(s);
    assert(std::isinf(cap.update(0.0, leftOnly, {0.0, 0.06})));
    assert(cap.stepCount() == 0);
  }
  // A tile step in double support caps the speed for holdSeconds, counted once per double support.
  {
    TerrainSpeedCap cap(s);
    assert(std::isinf(cap.update(1.00, rightOnly, {0.04, 0.0})));
    assert(near(cap.update(1.05, both, tile), 0.5) && cap.active() && cap.stepCount() == 1 && near(cap.lastStep(), 0.015));
    assert(near(cap.update(1.10, both, {0.0003, 0.0203}), 0.5) && cap.stepCount() == 1 && near(cap.lastStep(), 0.02));
    assert(near(cap.lastStepTime(), 1.10));
    // Back on flat strides: the cap holds for holdSeconds after the last step, then releases.
    assert(near(cap.update(1.20, leftOnly, flat), 0.5));
    assert(near(cap.update(4.09, both, flat), 0.5));
    assert(std::isinf(cap.update(4.11, both, flat)) && !cap.active());
    // The next double support with a step counts again.
    assert(std::isinf(cap.update(4.20, rightOnly, flat)));
    assert(near(cap.update(4.30, both, {0.0, -0.012}), 0.5) && cap.stepCount() == 2);
  }
  // A larger step engages the lower tier; the lower tier releases first, the first tier holds on.
  {
    TerrainSpeedCap cap(s);
    assert(near(cap.update(1.00, both, {0.0, 0.03}), 0.25) && cap.stepCount() == 1 && near(cap.capValue(), 0.25));
    assert(near(cap.update(1.50, leftOnly, flat), 0.25));
    assert(near(cap.update(2.00, both, tile), 0.25) && cap.stepCount() == 2);  // a 1.5 cm step refreshes tier 1 only
    assert(near(cap.update(3.99, both, flat), 0.25));
    assert(near(cap.update(4.01, both, flat), 0.5));  // tier 2 released at 1.00 + 3.0, tier 1 holds until 5.00
    assert(near(cap.update(4.99, both, flat), 0.5));
    assert(std::isinf(cap.update(5.01, both, flat)) && !cap.active());
  }
  // Below the threshold: no cap.
  {
    TerrainSpeedCap cap(s);
    assert(std::isinf(cap.update(0.0, both, {0.0, 0.009})) && cap.stepCount() == 0);
  }
  // Time going backwards (a new run) resets the memory.
  {
    TerrainSpeedCap cap(s);
    assert(near(cap.update(10.0, both, tile), 0.5));
    assert(std::isinf(cap.update(0.0, both, flat)) && cap.stepCount() == 0);
  }
  // Validation.
  {
    TerrainSpeedCapSettings bad = s;
    bad.speedKnots = {0.5, 0.0};
    bool threw = false;
    try {
      validateTerrainSpeedCapSettings(bad);
    } catch (const std::invalid_argument&) {
      threw = true;
    }
    assert(threw);
    bad = s;
    bad.stepKnots = {0.01};  // length mismatch
    threw = false;
    try {
      validateTerrainSpeedCapSettings(bad);
    } catch (const std::invalid_argument&) {
      threw = true;
    }
    assert(threw);
    bad = s;
    bad.speedKnots = {0.25, 0.5};  // a larger step must not raise the cap
    threw = false;
    try {
      validateTerrainSpeedCapSettings(bad);
    } catch (const std::invalid_argument&) {
      threw = true;
    }
    assert(threw);
    bad = s;
    bad.holdSeconds = -1.0;
    threw = false;
    try {
      TerrainSpeedCap cap(bad);
    } catch (const std::invalid_argument&) {
      threw = true;
    }
    assert(threw);
  }
  return 0;
}
