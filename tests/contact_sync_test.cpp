// Contact-synchronised touchdown (ContactSync.h): the rule on a human-gait schedule
// LF (right swings) | STANCE | RF (left swings) | STANCE, events at 0.0 / 0.3 / 0.4 / 0.7 / 0.8.
// Built with -UNDEBUG so the asserts stay active.
#include <cassert>
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
    feet_array_t<scalar_t> hold{kNaN, kNaN};
    const auto r = synchroniseTouchdowns(off, 0.26, heights(0.0, 0.0005), sch, hold);
    assert(r.advancedFoot == -1 && r.heldFoot == -1 && near(sch.eventTimes[1], 0.3));
  }
  // Early: right foot (index 1) swinging in LF, down at 0.26, touchdown planned at 0.3 -> moved to the solve time.
  {
    auto sch = makeSchedule();
    feet_array_t<scalar_t> hold{kNaN, kNaN};
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
    feet_array_t<scalar_t> hold{kNaN, kNaN};
    assert(synchroniseTouchdowns(s, 0.26, heights(0.0, 0.003), sch, hold).advancedFoot == -1 && near(sch.eventTimes[1], 0.3));
    assert(synchroniseTouchdowns(s, 0.2, heights(0.0, 0.0), sch, hold).advancedFoot == -1 && near(sch.eventTimes[1], 0.3));
  }
  // Early: the stance foot being down never advances anything; a foot on the ground right after
  // lift-off (lead 0.29) is outside the window.
  {
    auto sch = makeSchedule();
    feet_array_t<scalar_t> hold{kNaN, kNaN};
    assert(synchroniseTouchdowns(s, 0.01, heights(0.0, 0.0), sch, hold).advancedFoot == -1 && near(sch.eventTimes[1], 0.3));
  }
  // Early: the current phase is never shortened below minPhaseDuration.
  {
    ContactSyncSettings wide = s;
    wide.earlyWindow = 0.5;
    wide.minPhaseDuration = 0.05;
    auto sch = makeSchedule();
    feet_array_t<scalar_t> hold{kNaN, kNaN};
    const auto r = synchroniseTouchdowns(wide, 0.01, heights(0.0, 0.0), sch, hold);
    assert(r.advancedFoot == 1 && near(sch.eventTimes[1], 0.05) && near(r.advancedBy, 0.25));
  }
  // Late half off (lateWindow 0): the foot in the air after the planned touchdown is left alone.
  {
    auto sch = makeSchedule();
    feet_array_t<scalar_t> hold{kNaN, kNaN};
    const auto r = synchroniseTouchdowns(s, 0.31, heights(0.0, 0.01), sch, hold);
    assert(r.heldFoot == -1 && near(sch.eventTimes[1], 0.3));
  }
  // Late: right foot still 10 mm up at 0.31 -> event pushed to 0.33 (holdStep); again at 0.335 -> 0.355;
  // the hold stops lateWindow after the ORIGINAL time (0.3 + 0.08 = 0.38) and the foot is loaded where it is.
  {
    ContactSyncSettings late = s;
    late.lateWindow = 0.08;
    auto sch = makeSchedule();
    feet_array_t<scalar_t> hold{kNaN, kNaN};
    auto r = synchroniseTouchdowns(late, 0.31, heights(0.0, 0.01), sch, hold);
    assert(r.heldFoot == 1 && near(sch.eventTimes[1], 0.33) && near(r.heldBy, 0.03) && near(hold[1], 0.3));
    assert(sch.modeAtTime(0.31) == LF);  // the plan keeps swinging it down
    r = synchroniseTouchdowns(late, 0.335, heights(0.0, 0.008), sch, hold);
    assert(r.heldFoot == 1 && near(sch.eventTimes[1], 0.355) && near(hold[1], 0.3));
    r = synchroniseTouchdowns(late, 0.36, heights(0.0, 0.006), sch, hold);
    assert(r.heldFoot == 1 && near(sch.eventTimes[1], 0.38));  // capped at original + lateWindow
    r = synchroniseTouchdowns(late, 0.385, heights(0.0, 0.005), sch, hold);
    assert(r.heldFoot == -1 && near(sch.eventTimes[1], 0.38) && std::isnan(hold[1]));
    // A held foot that lands is loaded at once through the early half (event moved to now).
    auto sch2 = makeSchedule();
    feet_array_t<scalar_t> hold2{kNaN, kNaN};
    synchroniseTouchdowns(late, 0.31, heights(0.0, 0.01), sch2, hold2);
    r = synchroniseTouchdowns(late, 0.32, heights(0.0, 0.001), sch2, hold2);
    assert(r.advancedFoot == 1 && near(sch2.eventTimes[1], 0.319) && std::isnan(hold2[1]));
  }
  // Late: the next event bounds the push (events stay strictly increasing).
  {
    ContactSyncSettings late = s;
    late.lateWindow = 0.5;
    late.minPhaseDuration = 0.02;
    auto sch = makeSchedule();
    feet_array_t<scalar_t> hold{kNaN, kNaN};
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
    feet_array_t<scalar_t> hold{kNaN, kNaN};
    feet_array_t<ContactSyncFootObservation> feet;
    feet[0].soleHeight = 0.0;
    feet[0].soleVelocity = 0.0;
    feet[1].soleHeight = 0.02;
    feet[1].soleVelocity = 0.0;
    feet[1].plannedHeight = 0.002;
    feet[1].plannedVelocity = -0.05;
    const auto r = synchroniseTouchdowns(terrain, 0.26, feet, sch, hold);
    assert(r.advancedFoot == 1 && near(sch.eventTimes[1], 0.259));
  }
  {
    auto sch = makeSchedule();
    feet_array_t<scalar_t> hold{kNaN, kNaN};
    feet_array_t<ContactSyncFootObservation> feet;
    feet[0].soleHeight = 0.0;
    feet[1].soleHeight = 0.001;
    feet[1].soleVelocity = -0.15;  // passing the plane towards a lower tile
    feet[1].plannedHeight = 0.002;
    feet[1].plannedVelocity = -0.15;
    const auto r = synchroniseTouchdowns(terrain, 0.26, feet, sch, hold);
    assert(r.advancedFoot == -1 && near(sch.eventTimes[1], 0.3));
    // With the late half on, the same foot past its planned touchdown is held while it still descends.
    ContactSyncSettings late = terrain;
    late.lateWindow = 0.1;
    const auto h = synchroniseTouchdowns(late, 0.31, feet, sch, hold);
    assert(h.heldFoot == 1 && near(sch.eventTimes[1], 0.33) && near(hold[1], 0.3));
    // Once it stops (landed on the lower tile, plane estimate not yet caught up) the hold is released.
    feet[1].soleVelocity = 0.0;
    const auto done = synchroniseTouchdowns(late, 0.33, feet, sch, hold);
    assert(done.heldFoot == -1 && std::isnan(hold[1]));
  }
  return 0;
}
