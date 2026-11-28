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
  // Incline: strides of 0.3 m up a 5 deg ramp (rise 0.026 m per stride). The raw step would be a tier-2
  // roughness step every stride; once three samples agree the ground is a plane, the roughness residual is
  // zero and the incline cap (tier 1 of slopeKnots, 0.05 -> 0.3 m/s) holds instead.
  TerrainSpeedCapSettings d;  // the defaults: [0.01, 0.02, 0.035] -> [0.5, 0.2, 0.1], slopes [0.05, 0.12] -> [0.3, 0.2]
  d.enabled = true;
  {
    TerrainSpeedCap cap(d);
    const scalar_t rise = 0.3 * std::tan(5.0 * M_PI / 180.0);  // 0.0262
    scalar_t t = 0.0, lastTouchdown = 0.0;
    scalar_t zBack = 0.0, dBack = 0.0;
    size_t lead = 0;  // foot in front
    for (int stride = 0; stride < 8; ++stride) {
      const size_t back = 1 - lead;
      const scalar_t zFront = zBack + rise, dFront = dBack + 0.3;
      feet_array_t<scalar_t> z, dd;
      z[lead] = zFront; z[back] = zBack; dd[lead] = dFront; dd[back] = dBack;
      contact_flag_t swing = lead == 0 ? contact_flag_t{false, true} : contact_flag_t{true, false};
      cap.update(t, swing, z, dd);  // single support (the leading foot in flight): no sample
      t += 0.5;
      lastTouchdown = t;
      const scalar_t c = cap.update(t, both, z, dd);  // touchdown: double support
      t += 0.2;
      if (stride < 2) {
        assert(near(c, 0.2) && !cap.slopeConsistent());  // not yet three samples: the raw 2.6 cm step reads as tier 2 roughness
      } else {
        assert(cap.slopeConsistent() && near(cap.slope(), rise / 0.3) && near(cap.lastStep(), rise));
      }
      zBack = zFront; dBack = dFront; lead = back;
    }
    // Roughness tiers released (last raw step at the second touchdown, > 3 s ago); the incline cap holds at 0.3.
    assert(near(cap.capValue(), 0.3) && cap.active() && cap.stepCount() == 2);
    // Standing still on the ramp (feet side by side along the heading) adds no samples and keeps the incline cap:
    // the ground has not changed. (A single-support pause, no double-support refresh, lets it expire.)
    assert(near(cap.update(lastTouchdown + 5.0, both, {0.1, 0.1}, {0.0, 0.0}), 0.3));
    assert(near(cap.update(lastTouchdown + 5.5, leftOnly, {0.1, 0.1}, {0.0, 0.0}), 0.3));
    assert(std::isinf(cap.update(lastTouchdown + 8.6, leftOnly, {0.1, 0.1}, {0.0, 0.0})));
    // Walking off the ramp onto the flat: the first full flat stride misses the plane by 2.6 cm and drops it at
    // once (the references go flat; a flat step is no roughness); three flat samples make the plane flat again.
    scalar_t tf = lastTouchdown + 9.0;
    for (int stride = 0; stride < 3; ++stride) {
      cap.update(tf, stride % 2 ? leftOnly : rightOnly, {0.2, 0.2}, {dBack + 0.3, dBack});
      cap.update(tf + 0.5, both, {0.2, 0.2}, {dBack + 0.3, dBack});
      if (stride == 0) {
        assert(!cap.slopeConsistent() && cap.stepCount() == 2);
      }
      tf += 0.7;
    }
    assert(cap.slopeConsistent() && std::abs(cap.slope()) < 1e-9);
    assert(std::isinf(cap.capValue()) && cap.stepCount() == 2);
  }
  // A 10 deg incline engages the second incline tier (0.12 -> 0.2 m/s); walking down it (negative slope) too.
  for (const scalar_t sign : {1.0, -1.0}) {
    TerrainSpeedCap cap(d);
    const scalar_t slope = sign * std::tan(10.0 * M_PI / 180.0);  // 0.176
    for (int stride = 0; stride < 7; ++stride) {  // the first two raw 5.3 cm steps trip roughness tier 3 (0.1) for 3 s
      const scalar_t t = 0.7 * stride;
      cap.update(t, leftOnly, {0.0, 0.0}, {0.0, 0.0});
      cap.update(t + 0.5, both, {0.0, 0.3 * slope}, {0.0, 0.3});
    }
    assert(cap.slopeConsistent() && near(cap.slope(), slope) && near(cap.capValue(), 0.2));
  }
  // The plane stands through noise and falls to a contradiction, both measured in height: a noisy sample on a
  // short base (the stop's closing step: 8 cm apart, 4 mm of sole noise = 0.05 of slope) must not zero the plane
  // - the references and the contact normal would flip to flat on the incline - while the first full stride onto
  // the plateau (a 5 cm miss) drops it at once; three agreeing ~0 samples then make the ground flat.
  {
    TerrainSpeedCap cap(d);
    const scalar_t slope = std::tan(10.0 * M_PI / 180.0);
    scalar_t t = 0.0;
    for (int stride = 0; stride < 4; ++stride, t += 0.7) {
      cap.update(t, leftOnly, {0.0, 0.0}, {0.0, 0.0});
      cap.update(t + 0.5, both, {0.0, 0.3 * slope}, {0.0, 0.3});
    }
    assert(cap.slopeConsistent() && near(cap.slope(), slope));
    cap.update(t, leftOnly, {0.0, 0.0}, {0.0, 0.0});
    cap.update(t + 0.5, both, {0.0, 0.08 * slope + 0.004}, {0.0, 0.08});  // the noisy short step: sample 0.226
    t += 0.7;
    assert(cap.slopeConsistent() && near(cap.slope(), slope));  // the plane is kept, not zeroed
    cap.update(t, leftOnly, {0.0, 0.0}, {0.0, 0.0});
    cap.update(t + 0.5, both, {0.0, 0.08 * slope - 0.004}, {0.0, 0.08});  // and the other way: sample 0.126
    t += 0.7;
    assert(cap.slopeConsistent() && near(cap.slope(), slope));
    cap.update(t, leftOnly, {0.0, 0.0}, {0.0, 0.0});
    cap.update(t + 0.5, both, {0.0, 0.08 * slope + 0.012}, {0.0, 0.08});  // 1.2 cm off on a short base: a contradiction
    t += 0.7;
    assert(!cap.slopeConsistent() && near(cap.slope(), slope));  // dropped (the incline cap keeps the slope for its hold)
    for (int stride = 0; stride < 3; ++stride, t += 0.7) {  // back on the ramp: three agreeing strides rebuild it
      cap.update(t, leftOnly, {0.0, 0.0}, {0.0, 0.0});
      cap.update(t + 0.5, both, {0.0, 0.3 * slope}, {0.0, 0.3});
    }
    assert(cap.slopeConsistent() && near(cap.slope(), slope));
    cap.update(t, leftOnly, {0.0, 0.0}, {0.0, 0.0});
    cap.update(t + 0.5, both, {0.0, 0.0}, {0.0, 0.3});  // the first full stride on the plateau: 5.3 cm off the plane
    t += 0.7;
    assert(!cap.slopeConsistent());
    for (int stride = 0; stride < 2; ++stride, t += 0.7) {  // two more flat samples: the ground is flat
      cap.update(t, leftOnly, {0.0, 0.0}, {0.0, 0.0});
      cap.update(t + 0.5, both, {0.0, 0.0}, {0.0, 0.3});
    }
    assert(cap.slopeConsistent() && std::abs(cap.slope()) < 1e-9);
  }
  // Tiles: steps of random sign do not agree, so they keep the raw step and the roughness tiers (no incline cap).
  {
    TerrainSpeedCap cap(d);
    const scalar_t steps[] = {0.015, -0.012, 0.016, -0.015};
    scalar_t t = 0.0;
    for (const scalar_t dz : steps) {
      cap.update(t, leftOnly, {0.0, 0.0}, {0.0, 0.0});
      assert(near(cap.update(t + 0.5, both, {0.0, dz}, {0.0, 0.3}), 0.5));  // tier 1 roughness, 0.5 m/s
      t += 0.7;
    }
    assert(!cap.slopeConsistent() && cap.stepCount() == 4);
  }
  // Flat ground with the heading projection supplied: slope 0, consistent, no cap.
  {
    TerrainSpeedCap cap(d);
    for (int stride = 0; stride < 4; ++stride) {
      cap.update(0.7 * stride, rightOnly, flat, {0.3, 0.0});
      assert(std::isinf(cap.update(0.7 * stride + 0.5, both, flat, {0.3, 0.0})));
    }
    assert(cap.slopeConsistent() && std::abs(cap.slope()) < 0.01 && cap.stepCount() == 0);
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
    bad.slopeSpeedKnots = {0.3};  // length mismatch with slopeKnots
    threw = false;
    try {
      validateTerrainSpeedCapSettings(bad);
    } catch (const std::invalid_argument&) {
      threw = true;
    }
    assert(threw);
    bad = s;
    bad.slopeKnots.clear();
    bad.slopeSpeedKnots.clear();
    validateTerrainSpeedCapSettings(bad);  // no incline cap is allowed
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
