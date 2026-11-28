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
      // A double support with one solve gives its sample as it ends (at the next lift-off). The first stride's
      // raw 2.6 cm step reads as tier 2 roughness (no plane yet); its sample is a provisional plane that
      // explains the next strides (no more trips); the fourth sample confirms it at the fifth lift-off.
      assert(near(c, stride < 5 ? 0.2 : 0.3) && cap.stepCount() == 1);  // the trip holds 3 s, then the incline cap
      if (stride == 0) {
        assert(!cap.slopeConsistent());
      } else {
        assert(cap.slopeConsistent() && near(cap.slopeAlongX(), rise / 0.3) && cap.slopeConfirmed() == (stride >= 4));
      }
      zBack = zFront; dBack = dFront; lead = back;
    }
    // Roughness tier released (the one raw step, > 3 s ago); the incline cap of the confirmed plane holds at 0.3.
    assert(near(cap.capValue(), 0.3) && cap.active() && cap.stepCount() == 1);
    // Standing still on the ramp (feet side by side along the heading) adds no samples and keeps the incline cap:
    // the ground has not changed. (A single-support pause, no double-support refresh, lets it expire.)
    assert(near(cap.update(lastTouchdown + 5.0, both, {0.1, 0.1}, {0.0, 0.0}), 0.3));
    assert(near(cap.update(lastTouchdown + 5.5, leftOnly, {0.1, 0.1}, {0.0, 0.0}), 0.3));
    assert(std::isinf(cap.update(lastTouchdown + 8.6, leftOnly, {0.1, 0.1}, {0.0, 0.0})));
    // Walking off the ramp onto the flat: the first full flat stride misses the plane by 2.6 cm. Read against
    // the confirmed plane while that double support lasts it is one roughness trip (tier 2); its sample
    // replaces the plane with a provisional flat one (the references go flat at once, flat steps are no
    // roughness) and four flat samples confirm it.
    scalar_t tf = lastTouchdown + 9.0;
    for (int stride = 0; stride < 5; ++stride) {
      cap.update(tf, stride % 2 ? leftOnly : rightOnly, {0.2, 0.2}, {dBack + 0.3, dBack});
      if (stride >= 1) {
        assert(cap.slopeConsistent() && std::abs(cap.slopeAlongX()) < 1e-9 && cap.slopeConfirmed() == (stride >= 4));
      }
      cap.update(tf + 0.5, both, {0.2, 0.2}, {dBack + 0.3, dBack});
      assert(cap.stepCount() == 2);
      if (stride == 0) {
        assert(near(cap.capValue(), 0.2));
      }
      tf += 0.7;
    }
    assert(cap.slopeConfirmed() && std::abs(cap.slopeAlongX()) < 1e-9);
    assert(std::isinf(cap.update(tf + 1.0, leftOnly, {0.2, 0.2}, {dBack + 0.3, dBack})) && cap.stepCount() == 2);
  }
  // A 10 deg incline engages the second incline tier (0.12 -> 0.2 m/s); walking down it (negative slope) too.
  for (const scalar_t sign : {1.0, -1.0}) {
    TerrainSpeedCap cap(d);
    const scalar_t slope = sign * std::tan(10.0 * M_PI / 180.0);  // 0.176
    for (int stride = 0; stride < 7; ++stride) {  // the first raw 5.3 cm step trips roughness tier 3 (0.1) for 3 s
      const scalar_t t = 0.7 * stride;
      cap.update(t, leftOnly, {0.0, 0.0}, {0.0, 0.0});
      cap.update(t + 0.5, both, {0.0, 0.3 * slope}, {0.0, 0.3});
    }
    assert(cap.slopeConfirmed() && near(cap.slopeAlongX(), slope) && near(cap.capValue(), 0.2) && cap.stepCount() == 1);
  }
  // The plane stands through noise and falls to a contradiction, both measured in height: a noisy sample on a
  // short base (the stop's closing step: 8 cm apart, 4 mm of sole noise = 0.05 of slope) must not zero the plane
  // - the references and the contact normal would flip to flat on the incline - while the first full stride onto
  // the plateau (a 5 cm miss) drops it at once; four agreeing ~0 samples then make the ground flat. Each
  // stride's sample is read at its lift-off (the end of the double support).
  {
    TerrainSpeedCap cap(d);
    const scalar_t slope = std::tan(10.0 * M_PI / 180.0);
    scalar_t t = 0.0;
    auto stride = [&](scalar_t dz, scalar_t dd) {
      cap.update(t, leftOnly, {0.0, 0.0}, {0.0, 0.0});
      cap.update(t + 0.5, both, {0.0, dz}, {0.0, dd});
      t += 0.7;
      cap.update(t, leftOnly, {0.0, 0.0}, {0.0, 0.0});  // lift-off: the sample is read
    };
    for (int k = 0; k < 4; ++k) {
      stride(0.3 * slope, 0.3);
    }
    assert(cap.slopeConfirmed() && near(cap.slopeAlongX(), slope));
    stride(0.08 * slope + 0.004, 0.08);  // the noisy short step: sample 0.226
    assert(cap.slopeConfirmed() && near(cap.slopeAlongX(), slope));  // the plane is kept, not zeroed
    stride(0.08 * slope - 0.004, 0.08);  // and the other way: sample 0.126
    assert(cap.slopeConfirmed() && near(cap.slopeAlongX(), slope));
    stride(0.08 * slope + 0.012, 0.08);  // 1.2 cm off on a short base: a contradiction
    // A short base lays no provisional plane (1.2 cm over 8 cm would be 0.15 of slope on top): the ground is
    // rough until a sample agrees with the plane the one before it laid (run 27). The first stride back on the
    // ramp contradicts the short step's pending plane too; the second agrees with the first's and the references
    // follow the true slope again; four agreeing strides confirm it.
    assert(cap.rough() && !cap.slopeConsistent() && cap.gradient().norm() < 1e-12);
    stride(0.3 * slope, 0.3);
    assert(cap.rough() && !cap.slopeConsistent() && cap.gradient().norm() < 1e-12);
    stride(0.3 * slope, 0.3);
    assert(!cap.rough() && cap.slopeConsistent() && !cap.slopeConfirmed() && near(cap.slopeAlongX(), slope));
    stride(0.3 * slope, 0.3);
    stride(0.3 * slope, 0.3);
    assert(cap.slopeConfirmed() && near(cap.slopeAlongX(), slope));
    // The rough stride's raw step (5.3 cm with no plane standing) holds for holdSeconds: a crest within 3 s of it
    // would get its plane from the second plateau sample, not the first. Two more strides and it is calm.
    stride(0.3 * slope, 0.3);
    stride(0.3 * slope, 0.3);
    stride(0.0, 0.3);  // the first full stride on the plateau: 5.3 cm off the plane
    assert(cap.slopeConsistent() && !cap.slopeConfirmed() && std::abs(cap.slopeAlongX()) < 1e-9);  // provisional flat at once
    stride(0.0, 0.3);
    stride(0.0, 0.3);
    assert(cap.slopeConsistent() && !cap.slopeConfirmed() && std::abs(cap.slopeAlongX()) < 1e-9);
    stride(0.0, 0.3);  // four flat samples: the ground is confirmed flat
    assert(cap.slopeConfirmed() && std::abs(cap.slopeAlongX()) < 1e-9);
  }
  // The sample is read when the double support ends, not when it begins: a landing foot that is still in the
  // air at the planned touchdown (planned above the ground by a kept plane) must not confirm that plane. It is
  // read earlier when the reading has settled (unchanged from the previous solve): a landed foot.
  {
    TerrainSpeedCap cap(d);
    const scalar_t slope = std::tan(5.0 * M_PI / 180.0);
    scalar_t t = 0.0;
    for (int k = 0; k < 4; ++k, t += 0.7) {
      cap.update(t, leftOnly, {0.0, 0.0}, {0.0, 0.0});
      cap.update(t + 0.5, both, {0.0, 0.3 * slope}, {0.0, 0.3});
    }
    cap.update(t, leftOnly, {0.0, 0.0}, {0.0, 0.0});
    assert(cap.slopeConfirmed() && near(cap.slopeAlongX(), slope));
    // Onto the plateau: at the first solves of the double support the landing foot hangs where the plane put it
    // (2.6 cm up) and descends; the descending readings are not samples.
    cap.update(t + 0.5, both, {0.0, 0.3 * slope}, {0.0, 0.3});
    cap.update(t + 0.525, both, {0.0, 0.7 * 0.3 * slope}, {0.0, 0.3});
    cap.update(t + 0.55, both, {0.0, 0.3 * 0.3 * slope}, {0.0, 0.3});
    assert(cap.slopeConfirmed());  // not read yet
    cap.update(t + 0.575, both, {0.0, 0.0}, {0.0, 0.3});
    cap.update(t + 0.6, both, {0.0, 0.0}, {0.0, 0.3});  // settled: the sample is the landed reading
    assert(cap.slopeConsistent() && !cap.slopeConfirmed() && std::abs(cap.slopeAlongX()) < 1e-9);
    cap.update(t + 0.65, both, {0.0, 0.0}, {0.0, 0.3});  // one sample per double support
    cap.update(t + 0.7, rightOnly, {0.0, 0.0}, {0.0, 0.3});
    assert(cap.slopeConsistent() && !cap.slopeConfirmed());
    // A double support that never settles (a single solve, or a foot still moving) gives its sample at the end.
    cap.update(t + 1.2, both, {0.0, 0.0}, {0.0, 0.3});
    cap.update(t + 1.225, both, {0.0, 0.003}, {0.0, 0.3});
    cap.update(t + 1.4, leftOnly, {0.0, 0.0}, {0.0, 0.3});
    cap.update(t + 1.9, both, {0.0, 0.0}, {0.0, 0.3});
    cap.update(t + 2.1, rightOnly, {0.0, 0.0}, {0.0, 0.3});
    assert(cap.slopeConsistent() && !cap.slopeConfirmed());  // three flat samples and the ramp's: not yet
    cap.update(t + 2.6, both, {0.0, 0.0}, {0.0, 0.3});
    cap.update(t + 2.8, leftOnly, {0.0, 0.0}, {0.0, 0.3});
    assert(cap.slopeConfirmed() && std::abs(cap.slopeAlongX()) < 0.011);  // samples 0, 0.01, 0, 0 agree within 0.03
  }
  // World-frame plane (run 26): confirmed by strides up a 10 deg ramp along x, it survives a turn in place -
  // the feet then straddle the slope sideways, which a plane in the heading frame read as a contradiction -
  // and strides across the slope afterwards refine only the component they see.
  {
    TerrainSpeedCap cap(d);
    const scalar_t slope = std::tan(10.0 * M_PI / 180.0);
    const auto ground = [&](const vector2_t& p) { return slope * p(0); };
    scalar_t t = 0.0;
    vector2_t heading(1.0, 0.0);
    const auto support = [&](const vector2_t& p0, const vector2_t& p1, bool left) {
      cap.update(t, left ? leftOnly : rightOnly, {ground(p0), ground(p1)}, feet_array_t<vector2_t>{p0, p1}, heading);
      cap.update(t + 0.5, both, {ground(p0), ground(p1)}, feet_array_t<vector2_t>{p0, p1}, heading);
      const scalar_t c = cap.update(t + 0.55, both, {ground(p0), ground(p1)}, feet_array_t<vector2_t>{p0, p1}, heading);
      t += 0.7;
      return c;
    };
    // Up the ramp: feet 0.2 m apart sideways, strides of 0.3, the lead foot alternating (left = foot 0 at +y).
    for (int stride = 0; stride < 4; ++stride) {
      const scalar_t x = 0.3 * stride;
      if (stride % 2 == 0) {
        support(vector2_t(x + 0.3, 0.1), vector2_t(x, -0.1), true);
      } else {
        support(vector2_t(x, 0.1), vector2_t(x + 0.3, -0.1), false);
      }
      if (stride == 0) {  // one diagonal sample: the provisional plane lies along the heading at the stride's slope
        assert(cap.slopeConsistent() && !cap.slopeConfirmed() && near(cap.gradient()(0), slope) && near(cap.gradient()(1), 0.0));
      }
    }
    assert(cap.slopeConfirmed() && near(cap.gradient()(0), slope) && near(cap.gradient()(1), 0.0));
    // Turn in place: the feet's separation rotates from along x to along y (0.2 m apart, on the plane).
    const vector2_t centre(1.2, 0.0);
    const size_t steps = cap.stepCount();
    for (int k = 1; k <= 6; ++k) {
      const scalar_t angle = k * M_PI / 12.0;  // 15 deg per step, 90 deg in all
      const vector2_t half = 0.1 * vector2_t(-std::sin(angle), std::cos(angle));
      heading = vector2_t(std::cos(angle), std::sin(angle));
      const scalar_t c = support(centre + half, centre - half, k % 2 == 0);
      assert(cap.slopeConfirmed() && near(cap.gradient()(0), slope) && near(cap.gradient()(1), 0.0));
      assert(near(c, k == 1 ? 0.1 : 0.2) && cap.stepCount() == steps);  // the incline cap holds (the first stride's trip until 3.55 s); no roughness trip
    }
    // Walking across the slope (along y): the strides, alternating left and right of the track, see the whole
    // plane again (x from the stance width, y from the stride) and confirm it unchanged.
    heading = vector2_t(0.0, 1.0);
    for (int stride = 0; stride < 4; ++stride) {
      const scalar_t y = 0.3 * stride;
      if (stride % 2 == 0) {
        support(centre + vector2_t(-0.1, y + 0.3), centre + vector2_t(0.1, y), true);
      } else {
        support(centre + vector2_t(-0.1, y), centre + vector2_t(0.1, y + 0.3), false);
      }
    }
    assert(cap.slopeConfirmed() && near(cap.gradient()(0), slope) && near(cap.gradient()(1), 0.0) && cap.stepCount() == steps);
    assert(near(cap.slopeAlong(vector2_t(0.0, 1.0)), 0.0) && near(cap.slopeAlong(vector2_t(1.0, 0.0)), slope));
    // Back the way it came (heading -x, foot 0 now at -y): the plane unchanged.
    heading = vector2_t(-1.0, 0.0);
    for (int stride = 0; stride < 3; ++stride) {
      const scalar_t x = 1.2 - 0.3 * stride;
      if (stride % 2 == 0) {
        support(vector2_t(x - 0.3, -0.1), vector2_t(x, 0.1), true);
      } else {
        support(vector2_t(x, -0.1), vector2_t(x - 0.3, 0.1), false);
      }
    }
    assert(cap.slopeConfirmed() && near(cap.gradient()(0), slope) && near(cap.gradient()(1), 0.0) && cap.stepCount() == steps);
    // Onto level ground: the first level stride misses the plane by 5.3 cm (one roughness trip) and lays a
    // provisional level plane along the heading.
    cap.update(t, leftOnly, {0.0, 0.0}, feet_array_t<vector2_t>{vector2_t(-0.3, -0.1), vector2_t(0.0, 0.1)}, heading);
    assert(near(cap.update(t + 0.5, both, {0.0, 0.0}, feet_array_t<vector2_t>{vector2_t(-0.3, -0.1), vector2_t(0.0, 0.1)}, heading), 0.1));
    cap.update(t + 0.55, both, {0.0, 0.0}, feet_array_t<vector2_t>{vector2_t(-0.3, -0.1), vector2_t(0.0, 0.1)}, heading);
    assert(cap.slopeConsistent() && !cap.slopeConfirmed() && cap.gradient().norm() < 1e-9 && cap.stepCount() == steps + 1);
  }
  // Provisional planes and roughness: a tile step on flat ground makes a provisional plane of 0.05, and the next
  // tile step (the other way) must still read as its own raw step, not as the residual from that plane.
  {
    TerrainSpeedCap cap(d);
    cap.update(0.0, leftOnly, flat, {0.3, 0.0});
    cap.update(0.5, both, flat, {0.3, 0.0});
    cap.update(0.7, rightOnly, flat, {0.3, 0.0});
    assert(cap.slopeConsistent() && !cap.slopeConfirmed() && std::abs(cap.slopeAlongX()) < 0.002);
    assert(near(cap.update(1.2, both, {0.015, 0.0}, {0.3, 0.0}), 0.5) && std::abs(cap.lastStep() - 0.015) < 0.001);  // tier 1
    cap.update(1.4, leftOnly, flat, {0.3, 0.0});
    assert(cap.slopeConsistent() && !cap.slopeConfirmed() && near(cap.slopeAlongX(), 0.05));
    assert(near(cap.update(1.9, both, {-0.015, 0.0}, {0.3, 0.0}), 0.5) && std::abs(cap.lastStep() - 0.015) < 0.001);  // not 3 cm
    assert(cap.stepCount() == 2);
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
    assert(!cap.slopeConfirmed() && cap.stepCount() == 4);
  }
  // Rough ground (run 27): the second contradiction in a row makes the ground rough - no plane at all, instead
  // of a new provisional plane per stride (on tiles the references tilted 3-4 deg a new way at every landing).
  // Rough ground ends when a sample agrees with the plane the one before it laid.
  {
    TerrainSpeedCap cap(d);
    scalar_t t = 0.0;
    auto stride = [&](scalar_t dz, scalar_t dd) {
      cap.update(t, leftOnly, {0.0, 0.0}, {0.0, 0.0});
      cap.update(t + 0.5, both, {0.0, dz}, {0.0, dd});
      t += 0.7;
      cap.update(t, leftOnly, {0.0, 0.0}, {0.0, 0.0});
    };
    stride(0.0, 0.3);  // flat
    assert(cap.slopeConsistent() && !cap.rough());
    stride(0.015, 0.3);  // onto a tile: contradicts flat, provisional 0.05
    assert(cap.slopeConsistent() && !cap.rough() && near(cap.slopeAlongX(), 0.05));
    stride(0.0, 0.3);  // along the tile: contradicts the 0.05 plane (1.5 cm off) - the second in a row
    assert(cap.rough() && !cap.slopeConsistent() && !cap.slopeConfirmed() && cap.gradient().norm() < 1e-12);
    stride(-0.015, 0.3);  // off the tile: still rough, the raw step is the roughness
    assert(cap.rough() && cap.gradient().norm() < 1e-12 && std::abs(cap.lastStep() - 0.015) < 1e-9);
    stride(0.016, 0.3);
    assert(cap.rough());
    // Onto a 10 deg ramp from the tiles: the first stride lays a pending plane, the second agrees with it and the
    // references follow (one stride later than from flat), the fourth confirms it and the incline cap applies.
    const scalar_t slope = std::tan(10.0 * M_PI / 180.0);
    stride(0.3 * slope, 0.3);
    assert(cap.rough() && cap.gradient().norm() < 1e-12);
    stride(0.3 * slope, 0.3);
    assert(!cap.rough() && cap.slopeConsistent() && !cap.slopeConfirmed() && near(cap.slopeAlongX(), slope));
    stride(0.3 * slope, 0.3);
    assert(!cap.rough() && cap.slopeConsistent() && !cap.slopeConfirmed() && near(cap.slopeAlongX(), slope));
    stride(0.3 * slope, 0.3);  // four ramp samples in the window
    assert(cap.slopeConfirmed() && near(cap.slopeAlongX(), slope));
    // The first two ramp strides read as raw 5.3 cm steps on rough ground (tier 3, 0.1 m/s for holdSeconds): the price
    // of entering a ramp from tiles; the incline cap (0.2) takes over when it releases.
    assert(near(cap.update(t + 0.5, both, {0.0, 0.3 * slope}, {0.0, 0.3}), 0.1) && cap.stepCount() == 5);
    cap.update(t + 0.7, leftOnly, {0.0, 0.0}, {0.0, 0.0});
    t += 0.7;
    // A crest is one contradiction, then agreement: ramp -> plateau -> down-ramp never goes rough, and the
    // down-ramp plane is provisional from its first stride (the slope_5 stops, run 25 F').
    stride(0.0, 0.3);  // onto the plateau
    assert(!cap.rough() && cap.slopeConsistent() && !cap.slopeConfirmed() && std::abs(cap.slopeAlongX()) < 1e-9);
    stride(0.0, 0.3);  // along the plateau: agrees
    assert(!cap.rough() && cap.slopeConsistent());
    stride(-0.3 * slope, 0.3);  // off the crest
    assert(!cap.rough() && cap.slopeConsistent() && !cap.slopeConfirmed() && near(cap.slopeAlongX(), -slope));
    stride(-0.3 * slope, 0.3);
    stride(-0.3 * slope, 0.3);
    assert(!cap.rough() && cap.slopeConsistent() && !cap.slopeConfirmed() && near(cap.slopeAlongX(), -slope));
    stride(-0.3 * slope, 0.3);
    assert(cap.slopeConfirmed() && near(cap.slopeAlongX(), -slope));
    // Four samples still confirm a plane straight out of rough ground (alternating leads on a cross-slope agree
    // with one plane without any two of them laying the same provisional one along the heading) - but at half
    // the tolerance: two tile samples on the same separation 1.5 cm apart in rise and the rest fit one plane
    // within 7.5 mm each (few points, two parameters), which must not end the rough ground.
    TerrainSpeedCap cap2(d);
    scalar_t t2 = 0.0;
    auto stride2 = [&](scalar_t dz, const vector2_t& run) {
      cap2.update(t2, leftOnly, {0.0, 0.0}, {vector2_t::Zero(), vector2_t::Zero()}, vector2_t(1.0, 0.0));
      cap2.update(t2 + 0.5, both, {dz, 0.0}, {run, vector2_t::Zero()}, vector2_t(1.0, 0.0));  // rise dz over run
      t2 += 0.7;
      cap2.update(t2, leftOnly, {0.0, 0.0}, {vector2_t::Zero(), vector2_t::Zero()}, vector2_t(1.0, 0.0));
    };
    stride2(0.0, vector2_t(0.3, 0.2));
    stride2(0.015, vector2_t(0.3, -0.2));
    stride2(-0.015, vector2_t(0.3, 0.2));
    assert(cap2.rough());
    const vector2_t g(0.0, 0.1);  // a pure cross-slope: rise = +-0.02 with the lead alternating, nothing along x
    stride2(g.dot(vector2_t(0.3, -0.2)), vector2_t(0.3, -0.2));
    // The one ambiguity a single sample cannot resolve: the last tile sample (-0.015 on this separation) and the
    // first cross-slope sample (-0.02) agree within the tolerance, so their plane (-0.05 along x) is applied -
    // with one contradiction already on the count.
    assert(!cap2.rough() && cap2.slopeConsistent() && !cap2.slopeConfirmed() && near(cap2.slopeAlongX(), -0.05));
    stride2(g.dot(vector2_t(0.3, 0.2)), vector2_t(0.3, 0.2));
    assert(cap2.rough());  // the next sample contradicts it: rough at once, no second provisional plane
    stride2(g.dot(vector2_t(0.3, -0.2)), vector2_t(0.3, -0.2));
    stride2(g.dot(vector2_t(0.3, 0.2)), vector2_t(0.3, 0.2));
    assert(cap2.rough());  // the window restarted at the contradiction: three cross-slope samples so far
    stride2(g.dot(vector2_t(0.3, -0.2)), vector2_t(0.3, -0.2));
    assert(!cap2.rough() && cap2.slopeConfirmed() && (cap2.gradient() - g).norm() < 1e-9);  // four agree exactly
  }
  // A stop on tiles (run 27): the closing step lands on a tile 1.5 cm up, 8 cm ahead of the other foot. On calm
  // flat ground that contradicts the flat plane, but a short base lays no provisional plane (it would be 0.19 -
  // run 26 M backward fell standing on such a plane): no plane, the raw step is roughness, and the stand's own
  // sample (feet side by side) lays none either.
  {
    TerrainSpeedCap cap(d);
    scalar_t t = 0.0;
    for (int k = 0; k < 5; ++k, t += 0.7) {
      cap.update(t, leftOnly, flat, feet_array_t<vector2_t>{vector2_t(0.3 * k + 0.3, 0.1), vector2_t(0.3 * k, -0.1)}, vector2_t(1.0, 0.0));
      cap.update(t + 0.5, both, flat, feet_array_t<vector2_t>{vector2_t(0.3 * k + 0.3, 0.1), vector2_t(0.3 * k, -0.1)}, vector2_t(1.0, 0.0));
    }
    assert(cap.slopeConfirmed() && cap.gradient().norm() < 0.002);
    cap.update(t, leftOnly, flat, feet_array_t<vector2_t>{vector2_t(1.8, 0.1), vector2_t(1.5, -0.1)}, vector2_t(1.0, 0.0));
    const feet_array_t<vector2_t> closing{vector2_t(1.58, 0.1), vector2_t(1.5, -0.1)};
    assert(near(cap.update(t + 0.5, both, {0.0153, 0.0003}, closing, vector2_t(1.0, 0.0)), 0.5));  // tier 1 roughness
    cap.update(t + 0.525, both, {0.0153, 0.0003}, closing, vector2_t(1.0, 0.0));  // settled: the sample is read
    assert(cap.rough() && !cap.slopeConsistent() && cap.gradient().norm() < 1e-12);
    for (int i = 1; i <= 40; ++i) {  // standing: one double support, no further sample, no plane
      cap.update(t + 0.525 + 0.025 * i, both, {0.0153, 0.0003}, closing, vector2_t(1.0, 0.0));
    }
    assert(cap.rough() && !cap.slopeConsistent() && cap.gradient().norm() < 1e-12);
  }
  // Flat ground with the heading projection supplied: slope 0, confirmed, no cap.
  {
    TerrainSpeedCap cap(d);
    for (int stride = 0; stride < 5; ++stride) {
      cap.update(0.7 * stride, rightOnly, flat, {0.3, 0.0});
      assert(std::isinf(cap.update(0.7 * stride + 0.5, both, flat, {0.3, 0.0})));
    }
    assert(cap.slopeConfirmed() && std::abs(cap.slopeAlongX()) < 0.01 && cap.stepCount() == 0);
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
