// Contact-synchronised touchdown (ContactSync.h): the rule on a human-gait schedule
// LF (right swings) | STANCE | RF (left swings) | STANCE, events at 0.0 / 0.3 / 0.4 / 0.7 / 0.8.
// Built with -UNDEBUG so the asserts stay active.
#include <algorithm>
#include <cassert>
#include <functional>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <legged_robot_mpc_controller/mpc/common/gait/ContactSync.h>
#include <legged_robot_mpc_controller/mpc/common/gait/GaitSchedule.h>
#include <legged_robot_mpc_controller/mpc/common/gait/MotionPhaseDefinition.h>

int main() {
  using namespace ocs2;
  using namespace ocs2::humanoid;
  constexpr scalar_t kNaN = std::numeric_limits<scalar_t>::quiet_NaN();
  const auto near = [](double a, double b) { return std::abs(a - b) < 1e-9; };
  // Height-only observations (velocities and plan unknown): the rule of runs 12-18.
  const auto heights = [](scalar_t left, scalar_t right) {
    feet_array_t<ContactSyncFootObservation> feet;
    feet[0].soleHeight = left;
    feet[1].soleHeight = right;
    return feet;
  };
  // modes[i + 1] starts at events[i]: STANCE until 0.0, LF (right swings) 0.0-0.3, STANCE 0.3-0.4,
  // RF (left swings) 0.4-0.7, STANCE 0.7-0.8, LF from 0.8.
  const auto makeSchedule = [] {
    return ModeSchedule({0.0, 0.3, 0.4, 0.7, 0.8}, {STANCE, LF, STANCE, RF, STANCE, LF});
  };
  ContactSyncSettings s;
  s.enabled = true;
  s.earlyWindow = 0.06;
  s.lateWindow = 0.0;
  s.heightThreshold = 0.002;
  validateContactSyncSettings(s);

  // Disabled: nothing moves even with the foot down inside the window.
  {
    ContactSyncSettings off = s;
    off.enabled = false;
    auto sch = makeSchedule();
    feet_array_t<ContactSyncHold> hold{};
    const auto r = synchroniseTouchdowns(off, 0.26, heights(0.0, 0.0005), sch, hold);
    assert(r.advancedFoot == -1 && r.heldFoot == -1 && near(sch.eventTimes[1], 0.3));
  }
  // Early: right foot (index 1) swinging in LF, down at 0.26, touchdown planned at 0.3 -> moved to the solve time.
  {
    auto sch = makeSchedule();
    feet_array_t<ContactSyncHold> hold{};
    const auto r = synchroniseTouchdowns(s, 0.26, heights(0.0, 0.0005), sch, hold);
    assert(r.advancedFoot == 1 && near(r.advancedBy, 0.041) && r.heldFoot == -1);
    assert(near(sch.eventTimes[1], 0.259) && near(sch.eventTimes[2], 0.4));  // 1 ms before the solve time  // the following events stay
    assert(sch.modeAtTime(0.26) == STANCE && sch.modeAtTime(0.2589) == LF);
    // Idempotent: the next solve a moment later sees the foot in contact and nothing to do.
    const auto again = synchroniseTouchdowns(s, 0.27, heights(0.0, 0.0003), sch, hold);
    assert(again.advancedFoot == -1 && again.heldFoot == -1 && near(sch.eventTimes[1], 0.259));
  }
  // Early: not yet landed (3 mm) -> nothing; landed but outside the window (lead 0.1) -> nothing.
  {
    auto sch = makeSchedule();
    feet_array_t<ContactSyncHold> hold{};
    assert(synchroniseTouchdowns(s, 0.26, heights(0.0, 0.003), sch, hold).advancedFoot == -1 && near(sch.eventTimes[1], 0.3));
    assert(synchroniseTouchdowns(s, 0.2, heights(0.0, 0.0), sch, hold).advancedFoot == -1 && near(sch.eventTimes[1], 0.3));
  }
  // Early: the stance foot being down never advances anything; a foot on the ground right after
  // lift-off (lead 0.29) is outside the window.
  {
    auto sch = makeSchedule();
    feet_array_t<ContactSyncHold> hold{};
    assert(synchroniseTouchdowns(s, 0.01, heights(0.0, 0.0), sch, hold).advancedFoot == -1 && near(sch.eventTimes[1], 0.3));
  }
  // Early: the current phase is never shortened below minPhaseDuration.
  {
    ContactSyncSettings wide = s;
    wide.earlyWindow = 0.5;
    wide.minPhaseDuration = 0.05;
    auto sch = makeSchedule();
    feet_array_t<ContactSyncHold> hold{};
    const auto r = synchroniseTouchdowns(wide, 0.01, heights(0.0, 0.0), sch, hold);
    assert(r.advancedFoot == 1 && near(sch.eventTimes[1], 0.05) && near(r.advancedBy, 0.25));
  }
  // Late half off (lateWindow 0): the foot in the air after the planned touchdown is left alone.
  {
    auto sch = makeSchedule();
    feet_array_t<ContactSyncHold> hold{};
    const auto r = synchroniseTouchdowns(s, 0.31, heights(0.0, 0.01), sch, hold);
    assert(r.heldFoot == -1 && near(sch.eventTimes[1], 0.3));
  }
  // Late: right foot still 10 mm up at 0.31 -> event pushed to 0.33 (holdStep); again at 0.335 -> 0.355;
  // the hold stops lateWindow after the ORIGINAL time (0.3 + 0.08 = 0.38) and the foot is loaded where it is.
  {
    ContactSyncSettings late = s;
    late.lateWindow = 0.08;
    auto sch = makeSchedule();
    feet_array_t<ContactSyncHold> hold{};
    auto r = synchroniseTouchdowns(late, 0.31, heights(0.0, 0.01), sch, hold);
    assert(r.heldFoot == 1 && near(sch.eventTimes[1], 0.33) && near(r.heldBy, 0.03) && near(hold[1].origin, 0.3));
    assert(sch.modeAtTime(0.31) == LF);  // the plan keeps swinging it down
    r = synchroniseTouchdowns(late, 0.335, heights(0.0, 0.008), sch, hold);
    assert(r.heldFoot == 1 && near(sch.eventTimes[1], 0.355) && near(hold[1].origin, 0.3));
    r = synchroniseTouchdowns(late, 0.36, heights(0.0, 0.006), sch, hold);
    assert(r.heldFoot == 1 && near(sch.eventTimes[1], 0.38));  // capped at original + lateWindow
    r = synchroniseTouchdowns(late, 0.385, heights(0.0, 0.005), sch, hold);
    assert(r.heldFoot == -1 && near(sch.eventTimes[1], 0.38) && std::isnan(hold[1].origin));
    // A held foot that lands is loaded at once through the early half (event moved to now).
    auto sch2 = makeSchedule();
    feet_array_t<ContactSyncHold> hold2{};
    synchroniseTouchdowns(late, 0.31, heights(0.0, 0.01), sch2, hold2);
    r = synchroniseTouchdowns(late, 0.32, heights(0.0, 0.001), sch2, hold2);
    assert(r.advancedFoot == 1 && near(sch2.eventTimes[1], 0.319) && std::isnan(hold2[1].origin));
  }
  // Late: the next event bounds the push (events stay strictly increasing).
  {
    ContactSyncSettings late = s;
    late.lateWindow = 0.5;
    late.minPhaseDuration = 0.02;
    auto sch = makeSchedule();
    feet_array_t<ContactSyncHold> hold{};
    const auto r = synchroniseTouchdowns(late, 0.39, heights(0.0, 0.01), sch, hold);
    assert(r.heldFoot == -1 && near(sch.eventTimes[1], 0.3));  // 0.4 - 0.02 = 0.38 < time: window used up here
    const auto r2 = synchroniseTouchdowns(late, 0.37, heights(0.0, 0.01), sch, hold);
    assert(r2.heldFoot == 1 && near(sch.eventTimes[1], 0.38) && sch.eventTimes[1] < sch.eventTimes[2]);
  }
  // GaitSchedule::updateEventTimes keeps the per-phase swing clearance and rejects bad input.
  {
    ModeSequenceTemplate tpl({0.0, 0.3, 0.4, 0.7, 0.8}, {LF, STANCE, RF, STANCE});
    tpl.swingHeight = 0.05;
    GaitSchedule gait(ModeSchedule({0.0}, {STANCE, STANCE}), tpl, 0.0);
    gait.insertModeSequenceTemplate(tpl, 0.0, 2.0);
    auto sch = gait.getModeSchedule(0.0, 2.0);
    const auto heights = gait.getSwingHeightSequence();
    auto moved = sch.eventTimes;
    moved[1] -= 0.04;
    gait.updateEventTimes(moved);
    assert(near(gait.getCurrentModeSchedule().eventTimes[1], moved[1]));
    assert(gait.getSwingHeightSequence() == heights);
    bool threw = false;
    try {
      auto bad = moved;
      bad[2] = bad[1];
      gait.updateEventTimes(bad);
    } catch (const std::runtime_error&) {
      threw = true;
    }
    assert(threw);
    threw = false;
    try {
      gait.updateEventTimes(scalar_array_t{0.0});
    } catch (const std::runtime_error&) {
      threw = true;
    }
    assert(threw);
  }
  // Validation: enabled with both windows 0 is a configuration error; negative values too.
  {
    ContactSyncSettings bad = s;
    bad.earlyWindow = 0.0;
    bool threw = false;
    try {
      validateContactSyncSettings(bad);
    } catch (const std::invalid_argument&) {
      threw = true;
    }
    assert(threw);
    bad = s;
    bad.heightThreshold = -1e-3;
    threw = false;
    try {
      validateContactSyncSettings(bad);
    } catch (const std::invalid_argument&) {
      threw = true;
    }
    assert(threw);
    // blockedHeight without descentVelocity cannot tell a stopped sole from a descending one.
    bad = s;
    bad.blockedHeight = 0.01;
    threw = false;
    try {
      validateContactSyncSettings(bad);
    } catch (const std::invalid_argument&) {
      threw = true;
    }
    assert(threw);
    bad.descentVelocity = 0.03;
    validateContactSyncSettings(bad);
  }
  // Run 23: velocity-aware "landed" for uneven ground. Off by default: height only, velocities ignored.
  {
    ContactSyncFootObservation f;
    f.soleHeight = 0.001;
    f.soleVelocity = -0.3;  // falling fast through the plane
    assert(contactSyncLanded(s, f));
    f.soleHeight = 0.02;
    f.soleVelocity = 0.0;
    f.plannedHeight = 0.001;
    f.plannedVelocity = -0.05;
    assert(!contactSyncLanded(s, f));
  }
  ContactSyncSettings terrain = s;
  terrain.descentVelocity = 0.03;
  terrain.blockedHeight = 0.01;
  validateContactSyncSettings(terrain);
  {
    // Lower tile: within the threshold of the plane but still descending at the touchdown speed -> in the air.
    ContactSyncFootObservation f;
    f.soleHeight = 0.0015;
    f.soleVelocity = -0.05;
    assert(!contactSyncLanded(terrain, f));
    f.soleVelocity = -0.02;  // slower than descentVelocity: settled on the plane
    assert(contactSyncLanded(terrain, f));
    f.soleVelocity = 0.0;
    assert(contactSyncLanded(terrain, f));
    f.soleVelocity = kNaN;  // velocity unknown: height rule
    assert(contactSyncLanded(terrain, f));
    // Higher tile: stopped 2 cm above the plane while the plan is 1 mm above it and descending -> landed.
    f.soleHeight = 0.02;
    f.soleVelocity = 0.0;
    f.plannedHeight = 0.001;
    f.plannedVelocity = -0.05;
    assert(contactSyncLanded(terrain, f));
    f.plannedVelocity = 0.0;  // plan finished (stance): still landed
    assert(contactSyncLanded(terrain, f));
    f.plannedVelocity = 0.1;  // plan still rising: a lagging foot near lift-off is not blocked
    assert(!contactSyncLanded(terrain, f));
    f.plannedVelocity = -0.05;
    f.soleVelocity = -0.1;  // still coming down: lagging, not blocked
    assert(!contactSyncLanded(terrain, f));
    f.soleVelocity = 0.0;
    f.plannedHeight = 0.015;  // only 5 mm above the plan: tracking error, not a tile
    assert(!contactSyncLanded(terrain, f));
    f.plannedHeight = kNaN;  // no plan yet (first solve): height rule only
    assert(!contactSyncLanded(terrain, f));
  }
  // Through the schedule: the blocked foot advances its touchdown inside the window, the falling one does not.
  {
    auto sch = makeSchedule();
    feet_array_t<ContactSyncHold> hold{};
    feet_array_t<ContactSyncFootObservation> feet;
    feet[0].soleHeight = 0.0;
    feet[0].soleVelocity = 0.0;
    feet[1].soleHeight = 0.02;
    feet[1].soleVelocity = 0.0;
    feet[1].plannedHeight = 0.002;
    feet[1].plannedVelocity = -0.05;
    const auto r = synchroniseTouchdowns(terrain, 0.26, feet, sch, hold);
    assert(r.advancedFoot == 1 && near(sch.eventTimes[1], 0.259));
    assert(r.blockedLandings == 1 && r.descendingRejections == 0);
    // Outside the window the decision is not consulted and not counted.
    auto sch2 = makeSchedule();
    const auto r2 = synchroniseTouchdowns(terrain, 0.2, feet, sch2, hold);
    assert(r2.advancedFoot == -1 && r2.blockedLandings == 0);
  }
  {
    auto sch = makeSchedule();
    feet_array_t<ContactSyncHold> hold{};
    feet_array_t<ContactSyncFootObservation> feet;
    feet[0].soleHeight = 0.0;
    feet[1].soleHeight = 0.001;
    feet[1].soleVelocity = -0.15;  // passing the plane towards a lower tile
    feet[1].plannedHeight = 0.002;
    feet[1].plannedVelocity = -0.15;
    const auto r = synchroniseTouchdowns(terrain, 0.26, feet, sch, hold);
    assert(r.advancedFoot == -1 && near(sch.eventTimes[1], 0.3));
    assert(r.descendingRejections == 1 && r.blockedLandings == 0);
    // With the late half on, the same foot past its planned touchdown is held while it still descends.
    ContactSyncSettings late = terrain;
    late.lateWindow = 0.1;
    const auto h = synchroniseTouchdowns(late, 0.31, feet, sch, hold);
    assert(h.heldFoot == 1 && near(sch.eventTimes[1], 0.33) && near(hold[1].origin, 0.3));
    assert(h.descendingRejections == 1);
    // Once it stops (landed on the lower tile, plane estimate not yet caught up) the hold is released.
    feet[1].soleVelocity = 0.0;
    const auto done = synchroniseTouchdowns(late, 0.33, feet, sch, hold);
    assert(done.heldFoot == -1 && std::isnan(hold[1].origin));
    assert(done.descendingRejections == 0 && done.blockedLandings == 0);
  }
  // Probe (run 28): the late half for a sole that is still descending only.
  {
    ContactSyncSettings probe = s;
    probe.heightThreshold = 0.01;
    probe.descentVelocity = 0.05;
    probe.blockedHeight = 0.01;
    probe.probeWindow = 0.3;
    probe.probeDescent = 0.15;
    validateContactSyncSettings(probe);
    // It needs the velocity terms, and probeDescent needs the window.
    {
      ContactSyncSettings bad = probe;
      bad.descentVelocity = 0.0;
      bad.blockedHeight = 0.0;
      bool threw = false;
      try {
        validateContactSyncSettings(bad);
      } catch (const std::invalid_argument&) {
        threw = true;
      }
      assert(threw);
      bad = probe;
      bad.probeWindow = 0.0;
      threw = false;
      try {
        validateContactSyncSettings(bad);
      } catch (const std::invalid_argument&) {
        threw = true;
      }
      assert(threw);
    }
    // The first stride off a crest: at the planned touchdown (0.3) the right foot is at the plane, still
    // going down at the swing's end velocity, the ground 2.6 cm lower. The event is pushed solve by solve.
    auto sch = makeSchedule();
    feet_array_t<ContactSyncHold> hold{};
    feet_array_t<ContactSyncFootObservation> feet;
    feet[0].soleHeight = 0.0;
    feet[1].soleHeight = -0.001;
    feet[1].soleVelocity = -0.11;
    feet[1].plannedHeight = -0.001;
    feet[1].plannedVelocity = -0.11;
    const auto first = synchroniseTouchdowns(probe, 0.31, feet, sch, hold);
    assert(first.heldFoot == 1 && first.probing && near(sch.eventTimes[1], 0.33) && near(hold[1].origin, 0.3));
    assert(near(first.heldBy, 0.03) && first.descendingRejections == 1);
    // The rest of the schedule slides with the touchdown: the double support keeps its 0.1 s.
    assert(near(sch.eventTimes[2], 0.43) && near(sch.eventTimes[3], 0.73) && near(sch.eventTimes[4], 0.83));
    feet[1].soleHeight = -0.012;  // below the plane, still descending: held on
    const auto second = synchroniseTouchdowns(probe, 0.33, feet, sch, hold);
    assert(second.heldFoot == 1 && second.probing && near(sch.eventTimes[1], 0.35) && near(second.heldBy, 0.05));
    assert(near(sch.eventTimes[2], 0.45) && near(sch.eventTimes[4], 0.85));
    // It touches the ramp and stops: landed (within the threshold of the lowered plan / on the plane), hold over.
    feet[1].soleHeight = -0.026;
    feet[1].soleVelocity = 0.0;
    feet[1].plannedHeight = -0.026;
    const auto landed = synchroniseTouchdowns(probe, 0.351, feet, sch, hold);
    assert(landed.heldFoot == -1 && landed.advancedFoot == -1 && !landed.probing && std::isnan(hold[1].origin));
    assert(near(sch.eventTimes[1], 0.35) && sch.modeAtTime(0.352) == STANCE);
    // Landing while the pushed event is still ahead (the usual case: the push stays ahead of the solve):
    // the early half pulls it to the solve time whatever its lead, and the push it did not use slides
    // back - the stride after the touchdown keeps its planned phases.
    {
      ContactSyncSettings slow = probe;
      slow.holdStep = 0.08;  // > earlyWindow 0.06, as the reference manager sets it on a slow solver
      auto sch5 = makeSchedule();
      feet_array_t<ContactSyncHold> hold5{};
      feet_array_t<ContactSyncFootObservation> f5 = feet;
      f5[1].soleHeight = -0.001;
      f5[1].soleVelocity = -0.11;
      f5[1].plannedHeight = -0.001;
      f5[1].plannedVelocity = -0.11;
      const auto h5 = synchroniseTouchdowns(slow, 0.31, f5, sch5, hold5);
      assert(h5.heldFoot == 1 && h5.probing && near(sch5.eventTimes[1], 0.39) && near(h5.heldBy, 0.09));
      assert(near(sch5.eventTimes[2], 0.49) && near(sch5.eventTimes[4], 0.89) && hold5[1].probing);
      // Still ahead at 0.32 (lead 0.07, beyond the early window - the held foot is watched all the same),
      // not landed: nothing moves.
      f5[1].soleHeight = -0.003;
      const auto a5 = synchroniseTouchdowns(slow, 0.32, f5, sch5, hold5);
      assert(a5.heldFoot == -1 && a5.advancedFoot == -1 && near(sch5.eventTimes[1], 0.39) && near(hold5[1].origin, 0.3));
      assert(a5.descendingRejections == 1);
      // Lands at 0.325 (lead 0.065 > earlyWindow): touchdown at 0.324, the unused 0.066 s of the push slides
      // back, the double support is 0.1 s again and the next swing 0.3 s.
      f5[1].soleHeight = -0.004;
      f5[1].soleVelocity = 0.0;
      f5[1].plannedHeight = -0.004;
      const auto l5 = synchroniseTouchdowns(slow, 0.325, f5, sch5, hold5);
      assert(l5.advancedFoot == 1 && l5.heldFoot == -1 && std::isnan(hold5[1].origin) && !hold5[1].probing);
      assert(near(sch5.eventTimes[1], 0.324) && near(sch5.eventTimes[2], 0.424) && near(sch5.eventTimes[3], 0.724) &&
             near(sch5.eventTimes[4], 0.824));
      // An unheld foot landing that far ahead of its event is left to the early window as before.
      auto sch5b = makeSchedule();
      feet_array_t<ContactSyncHold> hold5b{};
      const auto u5 = synchroniseTouchdowns(slow, 0.23, f5, sch5b, hold5b);
      assert(u5.advancedFoot == -1 && near(sch5b.eventTimes[1], 0.3));
      assert(std::is_sorted(sch5.eventTimes.begin(), sch5.eventTimes.end(), std::less_equal<scalar_t>()));
    }
    // A probe needs a stride to follow: the last touchdown before a stand is loaded where it is.
    {
      ModeSchedule stop({0.0, 0.3, 0.4, 1.4}, {STANCE, LF, STANCE, STANCE, STANCE});
      feet_array_t<ContactSyncHold> hold6{};
      const auto r6 = synchroniseTouchdowns(probe, 0.31, feet, stop, hold6);
      assert(r6.heldFoot == -1 && !r6.probing && near(stop.eventTimes[1], 0.3) && std::isnan(hold6[1].origin));
    }
    // A sole that has STOPPED in the air past its planned touchdown is not probed (lateWindow 0: loaded where
    // it is) - the run 23 B tile failure was a stopped sole held and lifted. 1.5 cm up, plan at the plane:
    // that is a blocked landing anyway; 5 mm up it is on the plane. Neither is held.
    {
      auto sch2 = makeSchedule();
      feet_array_t<ContactSyncHold> hold2{};
      feet_array_t<ContactSyncFootObservation> stopped;
      stopped[0].soleHeight = 0.0;
      stopped[1].soleHeight = 0.015;
      stopped[1].soleVelocity = 0.0;
      stopped[1].plannedHeight = -0.001;
      stopped[1].plannedVelocity = 0.0;
      const auto r = synchroniseTouchdowns(probe, 0.31, stopped, sch2, hold2);
      assert(r.heldFoot == -1 && !r.probing && near(sch2.eventTimes[1], 0.3));
      assert(r.blockedLandings == 0);  // not consulted: no window applies to a stopped sole
      stopped[1].soleHeight = 0.005;
      const auto r2 = synchroniseTouchdowns(probe, 0.31, stopped, sch2, hold2);
      assert(r2.heldFoot == -1 && near(sch2.eventTimes[1], 0.3));
      // Velocity unknown: no probe either (the height-only rule has nothing to say about descent).
      stopped[1].soleHeight = 0.02;
      stopped[1].soleVelocity = kNaN;
      const auto r3 = synchroniseTouchdowns(probe, 0.31, stopped, sch2, hold2);
      assert(r3.heldFoot == -1 && near(sch2.eventTimes[1], 0.3));
      // Still 2 cm ABOVE the plane and descending at the planned time: a late swing (a slow foot, a higher
      // tile under it), not a missing ground - loaded as before, not probed.
      stopped[1].soleVelocity = -0.11;
      stopped[1].plannedHeight = -0.001;
      stopped[1].plannedVelocity = 0.0;
      const auto r4 = synchroniseTouchdowns(probe, 0.31, stopped, sch2, hold2);
      assert(r4.heldFoot == -1 && !r4.probing && near(sch2.eventTimes[1], 0.3));
    }
    // The window caps the hold: a hole. From 0.3 the foot descends with nothing under it; the pushes stop at
    // 0.6 and the plan loads the foot where it is.
    {
      auto sch3 = makeSchedule();
      feet_array_t<ContactSyncHold> hold3{};
      feet_array_t<ContactSyncFootObservation> falling = feet;
      falling[1].soleVelocity = -0.11;
      falling[1].plannedVelocity = -0.11;
      scalar_t t = 0.31;
      ContactSyncResult r;
      for (; t < 0.7; t += 0.02) {
        falling[1].soleHeight = -0.001 - 0.11 * (t - 0.3);
        falling[1].plannedHeight = falling[1].soleHeight;
        r = synchroniseTouchdowns(probe, t, falling, sch3, hold3);
        if (r.heldFoot < 0) {
          break;
        }
        assert(sch3.eventTimes[1] <= 0.6 + 1e-9 && near(r.heldBy, sch3.eventTimes[1] - 0.3));
      }
      assert(t > 0.58 && near(sch3.eventTimes[1], 0.6) && std::isnan(hold3[1].origin));
      // The whole gait is 0.3 s later, phase lengths untouched.
      assert(near(sch3.eventTimes[2], 0.7) && near(sch3.eventTimes[3], 1.0) && near(sch3.eventTimes[4], 1.1));
    }
    // The late half proper (lateWindow, a sole merely not landed) still eats the double support instead.
    {
      ContactSyncSettings late = probe;
      late.probeWindow = 0.0;
      late.probeDescent = 0.0;
      late.lateWindow = 0.3;
      auto sch4 = makeSchedule();
      feet_array_t<ContactSyncHold> hold4{};
      feet_array_t<ContactSyncFootObservation> f4 = feet;
      f4[1].soleHeight = -0.001;
      f4[1].soleVelocity = -0.11;
      f4[1].plannedHeight = -0.001;
      f4[1].plannedVelocity = -0.11;
      for (scalar_t t = 0.31; t < 0.5; t += 0.02) {
        const auto r = synchroniseTouchdowns(late, t, f4, sch4, hold4);
        assert(!r.probing);
        if (r.heldFoot < 0) {
          break;
        }
      }
      assert(near(sch4.eventTimes[1], 0.38) && near(sch4.eventTimes[2], 0.4));
    }
  }
  return 0;
}
