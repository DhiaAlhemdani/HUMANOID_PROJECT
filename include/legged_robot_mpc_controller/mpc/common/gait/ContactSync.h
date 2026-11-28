/******************************************************************************
Copyright (c) 2025, Manuel Yves Galliker. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#pragma once

#include <limits>
#include <ocs2_core/Types.h>
#include <ocs2_core/reference/ModeSchedule.h>

#include "legged_robot_mpc_controller/mpc/common/model/Types.h"

namespace ocs2::humanoid {

/**
 * Contact-synchronised touchdown: the plan loads a foot when it lands, not when the schedule
 * said it would.
 *
 * The swing foot reaches the ground 20-50 ms before its planned touchdown (the spline spends its
 * last 27 ms inside the final millimetre, the plan sags ~1-2 mm under the spline, the sole sits at
 * 0.3 mm). Until the planned event the foot stands there unloaded with the swing equality still
 * active, then the plan loads it 0 -> 120 N in one node (docs/results/human_gait_template.md
 * runs 12-17). Shaping the touchdown force does not help (runs 13 / 17: the body arrives with the
 * foot and needs the force the moment it lands); moving the event to the landing does.
 *
 * Rule, applied once per solve to the schedule the solve is built on:
 *  - early: a foot the schedule has swinging at `time`, whose next contact event is less than
 *    `earlyWindow` ahead, and whose measured sole height is at most `heightThreshold` above the
 *    support plane, has that event moved to `time` (the following events stay: the double
 *    support absorbs the lead).
 *  - late: a foot the schedule has in contact at `time`, whose contact event lies less than
 *    `lateWindow` behind, and whose sole is still above `heightThreshold`, has that event pushed to
 *    `time + holdStep` (the plan keeps swinging it down); the push stops at `lateWindow` after the
 *    originally planned time. 0 disables the late half.
 * No phase is made shorter than `minPhaseDuration`.
 *
 * "Landed" (contactSyncLanded): height alone is wrong on uneven ground both ways (run 22 D): a foot
 * standing on a tile 2 cm above the plane is never accepted and the swing equality drives it into
 * the tile; a foot passing the plane on its way to a tile 3 cm below is accepted in the air. With
 * the optional velocity terms (0 = off, the height-only rule of runs 12-18 is unchanged):
 *  - a sole within `heightThreshold` of the plane that is still descending faster than
 *    `descentVelocity` is NOT landed (it is passing the plane towards lower ground);
 *  - a sole at least `blockedHeight` above its planned swing height, while the plan is descending or
 *    at rest and the sole has stopped descending (slower than `descentVelocity`), IS landed (it is
 *    standing on higher ground than the plan assumed).
 *
 * Probe (run 28): the first stride down off a crest reaches its planned touchdown at the plane the
 * plateau laid, still descending, with the ground 1.5-2.6 cm lower (slope_5: the plan loaded the foot
 * in the air and the body pivoted onto it, pitch 22 deg). With `probeWindow` > 0 the planned touchdown
 * of a foot that is at the plane and still DESCENDING through it faster than `descentVelocity` (the
 * `Descending` landing) is held like the late half, up to `probeWindow` after the planned time - and
 * only while it descends: a stopped sole is landed (on the plane or blocked) or loaded where it is,
 * which is what made the general late half destructive on tiles (run 23 B: a sole pressed on a tile,
 * held and lifted again); a sole still above the plane is late, not probing, and is loaded as before.
 * Needs the velocity terms. The reference manager lowers the held foot's touchdown target by
 * `probeDescent` x the hold so far, so the swing keeps going down instead of hovering at the plane (a
 * re-stretched spline alone lifts the foot). A probe hold slides the rest of the schedule with the
 * touchdown (the step takes longer); when the foot lands, the part of the push it did not use slides
 * back, so the stride after a probed touchdown keeps its planned phases. The late half squeezes the
 * following double support instead. A probe needs a stride to follow the touchdown (the next phase after
 * its double support is a single support): the last touchdown before a stand is loaded where it is (run
 * 28 A: probing the stop on a ramp held the robot on one leg while it decelerated, pitch 31 deg).
 *
 * Run 28 A (tiles, two falls, plan_mode flickering swing/stance - the step clock read 0.4 s against a
 * 0.72 s gait): a held event must outlive the policy that carries it. The schedule a solve builds is
 * published when the solve ends and is used until the next one is, so an event `holdStep` = 20 ms past
 * the solve time is already in the past for a controller running at 50 Hz effective: the whole-body
 * side loads the foot the plan still swings. The reference manager raises holdStep to three measured
 * solve periods (capped), and a held foot that lands is synchronised whatever its lead.
 */
struct ContactSyncSettings {
  bool enabled = false;
  scalar_t earlyWindow = 0.06;       //!< [s] landings this long before the planned touchdown advance it
  scalar_t lateWindow = 0.0;         //!< [s] the planned touchdown is held up to this long while the foot is in the air
  scalar_t heightThreshold = 0.002;  //!< [m] sole height above the support plane that counts as landed
  scalar_t holdStep = 0.02;          //!< [s] a held touchdown is pushed this far past the solve time
  scalar_t minPhaseDuration = 0.02;  //!< [s] no phase is shortened below this
  scalar_t descentVelocity = 0.0;    //!< [m/s] a sole descending faster than this is in the air whatever its height; 0 = off
  scalar_t blockedHeight = 0.0;      //!< [m] a stopped sole this far above the descending plan is landed; 0 = off
  scalar_t probeWindow = 0.0;        //!< [s] the planned touchdown is held up to this long while the sole still descends; 0 = off
  scalar_t probeDescent = 0.0;       //!< [m/s] the held foot's touchdown target is lowered at this rate over the hold (reference manager)
};

/** What the rule sees of one foot. Velocities and the plan may be NaN (unknown): the height-only rule then applies. */
struct ContactSyncFootObservation {
  scalar_t soleHeight = 0.0;                                                  //!< [m] measured sole height above the support plane
  scalar_t soleVelocity = std::numeric_limits<scalar_t>::quiet_NaN();     //!< [m/s] measured vertical sole velocity (up positive)
  scalar_t plannedHeight = std::numeric_limits<scalar_t>::quiet_NaN();    //!< [m] planned swing height above the support plane at the solve time
  scalar_t plannedVelocity = std::numeric_limits<scalar_t>::quiet_NaN();  //!< [m/s] planned vertical swing velocity at the solve time
};

/** Outcome of the "landed" decision documented above. */
enum class ContactSyncLanding {
  InAir,       //!< above the threshold, not blocked
  OnPlane,     //!< within the threshold (and not descending through it)
  Blocked,     //!< stopped above the descending plan: landed on higher ground
  Descending,  //!< within the threshold but still descending: passing the plane towards lower ground
};

ContactSyncLanding contactSyncLanding(const ContactSyncSettings& settings, const ContactSyncFootObservation& foot);

/** The "landed" decision: OnPlane or Blocked. */
bool contactSyncLanded(const ContactSyncSettings& settings, const ContactSyncFootObservation& foot);

void validateContactSyncSettings(const ContactSyncSettings& settings);

/** Per-foot state of a held touchdown, kept by the caller between solves. */
struct ContactSyncHold {
  scalar_t origin = std::numeric_limits<scalar_t>::quiet_NaN();  //!< [s] originally planned touchdown time of the held event; NaN = not holding
  bool probing = false;                                           //!< the hold slid the schedule (a probe): the unused push slides back on landing
};

struct ContactSyncResult {
  int advancedFoot = -1;        //!< foot whose touchdown was moved to the solve time, -1 = none
  scalar_t advancedBy = 0.0;    //!< [s] how much earlier the touchdown now is
  int heldFoot = -1;            //!< foot whose touchdown was pushed back, -1 = none
  scalar_t heldBy = 0.0;        //!< [s] how far past the originally planned time it now is
  bool probing = false;         //!< the hold is the probe's (the sole still descending past its planned touchdown)
  //! Diagnostics of the velocity terms, counted only where the decision was consulted (a swing foot
  //! inside the early window; a just-switched foot in the late half).
  int blockedLandings = 0;      //!< feet accepted as landed by the blockedHeight rule this call
  int descendingRejections = 0; //!< feet within the threshold rejected by the descentVelocity rule this call
};

/**
 * Applies the rule to `schedule` in place.
 *
 * @param settings   the rule.
 * @param time       the solve's start time.
 * @param feet       per-foot measured sole height / velocity and planned swing height / velocity (see
 *                   ContactSyncFootObservation; NaN = unknown).
 * @param schedule   the mode schedule the solve is built on (eventTimes strictly increasing).
 * @param hold       per-foot state of a held event (ContactSyncHold); owned by the caller, updated here.
 */
ContactSyncResult synchroniseTouchdowns(const ContactSyncSettings& settings, scalar_t time,
                                        const feet_array_t<ContactSyncFootObservation>& feet, ModeSchedule& schedule,
                                        feet_array_t<ContactSyncHold>& hold);

}  // namespace ocs2::humanoid
