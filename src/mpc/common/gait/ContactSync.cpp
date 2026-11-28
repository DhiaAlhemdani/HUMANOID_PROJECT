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

#include "legged_robot_mpc_controller/mpc/common/gait/ContactSync.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "legged_robot_mpc_controller/mpc/common/gait/MotionPhaseDefinition.h"

namespace ocs2::humanoid {

void validateContactSyncSettings(const ContactSyncSettings& s) {
  if (!s.enabled) {
    return;
  }
  if (!(s.earlyWindow >= 0.0) || !(s.lateWindow >= 0.0) || !(s.heightThreshold >= 0.0) || !(s.holdStep > 0.0) ||
      !(s.minPhaseDuration >= 0.0)) {
    throw std::invalid_argument(
        "[ContactSync] earlyWindow, lateWindow, heightThreshold and minPhaseDuration must be >= 0 and holdStep > 0");
  }
  if (s.earlyWindow == 0.0 && s.lateWindow == 0.0 && s.probeWindow == 0.0) {
    throw std::invalid_argument("[ContactSync] enabled with every window 0: nothing to do; disable it instead");
  }
  if (!(s.descentVelocity >= 0.0) || !(s.blockedHeight >= 0.0) || !(s.probeWindow >= 0.0) || !(s.probeDescent >= 0.0)) {
    throw std::invalid_argument("[ContactSync] descentVelocity, blockedHeight, probeWindow and probeDescent must be >= 0");
  }
  if (s.blockedHeight > 0.0 && s.descentVelocity <= 0.0) {
    throw std::invalid_argument("[ContactSync] blockedHeight needs descentVelocity > 0 to tell a stopped sole from a descending one");
  }
  if (s.probeWindow > 0.0 && s.descentVelocity <= 0.0) {
    throw std::invalid_argument("[ContactSync] probeWindow needs descentVelocity > 0: it holds a sole that still descends");
  }
  if (s.probeDescent > 0.0 && s.probeWindow <= 0.0) {
    throw std::invalid_argument("[ContactSync] probeDescent lowers a probed foot's target: it needs probeWindow > 0");
  }
}

ContactSyncLanding contactSyncLanding(const ContactSyncSettings& s, const ContactSyncFootObservation& f) {
  const bool nearPlane = f.soleHeight <= s.heightThreshold;
  const bool velocityKnown = std::isfinite(f.soleVelocity);
  if (nearPlane) {
    // Passing the plane on the way down: the ground is lower than the plane assumed.
    const bool descending = s.descentVelocity > 0.0 && velocityKnown && f.soleVelocity < -s.descentVelocity;
    return descending ? ContactSyncLanding::Descending : ContactSyncLanding::OnPlane;
  }
  if (s.blockedHeight <= 0.0 || !velocityKnown || !std::isfinite(f.plannedHeight) || !std::isfinite(f.plannedVelocity)) {
    return ContactSyncLanding::InAir;
  }
  // Stopped above a descending (or finished) plan: the ground is higher than the plane assumed.
  const bool stopped = f.soleVelocity >= -s.descentVelocity;
  const bool planDescending = f.plannedVelocity <= 0.0;
  const bool blocked = stopped && planDescending && f.soleHeight - f.plannedHeight >= s.blockedHeight;
  return blocked ? ContactSyncLanding::Blocked : ContactSyncLanding::InAir;
}

bool contactSyncLanded(const ContactSyncSettings& s, const ContactSyncFootObservation& f) {
  const ContactSyncLanding landing = contactSyncLanding(s, f);
  return landing == ContactSyncLanding::OnPlane || landing == ContactSyncLanding::Blocked;
}

ContactSyncResult synchroniseTouchdowns(const ContactSyncSettings& settings, scalar_t time,
                                        const feet_array_t<ContactSyncFootObservation>& feet, ModeSchedule& schedule,
                                        feet_array_t<ContactSyncHold>& hold) {
  ContactSyncResult result;
  if (!settings.enabled) {
    return result;
  }
  auto& events = schedule.eventTimes;
  const auto& modes = schedule.modeSequence;
  if (events.empty() || modes.size() != events.size() + 1) {
    return result;
  }
  // modes[i + 1] starts at events[i]; the mode at `time` is modes[k] with k the first event after `time`.
  const size_t k = static_cast<size_t>(std::upper_bound(events.begin(), events.end(), time) - events.begin());
  const contact_flag_t now = modeNumber2StanceLeg(modes[k]);

  for (size_t foot = 0; foot < N_CONTACTS; ++foot) {
    const ContactSyncLanding landing = contactSyncLanding(settings, feet[foot]);
    const bool landed = landing == ContactSyncLanding::OnPlane || landing == ContactSyncLanding::Blocked;
    const auto count = [&] {
      result.blockedLandings += landing == ContactSyncLanding::Blocked;
      result.descendingRejections += landing == ContactSyncLanding::Descending;
    };
    if (!now[foot]) {
      // Early half: the foot swings and its touchdown is the next event.
      if (settings.earlyWindow <= 0.0 || k >= events.size() || !modeNumber2StanceLeg(modes[k + 1])[foot]) {
        continue;
      }
      // A held touchdown sits up to holdStep ahead of the solve time, which may exceed the early window:
      // a held foot that lands is synchronised whatever its lead.
      const bool held = std::isfinite(hold[foot].origin);
      const scalar_t lead = events[k] - time;
      if (lead <= 0.0 || (lead > settings.earlyWindow && !held)) {
        continue;
      }
      count();
      if (!landed) {
        continue;
      }
      // Just before the solve time: ocs2 resolves the mode at an event time as the mode before it,
      // so an event exactly at the solve time would leave the first node swinging.
      constexpr scalar_t kBeforeSolveTime = 1e-3;
      const scalar_t phaseStart = k > 0 ? events[k - 1] : -std::numeric_limits<scalar_t>::infinity();
      const scalar_t newTime = std::max(time - kBeforeSolveTime, phaseStart + settings.minPhaseDuration);
      if (newTime >= events[k]) {
        continue;
      }
      if (hold[foot].probing) {
        // The probe slid the rest of the schedule with the pushed touchdown; the push it did not use
        // slides back, so the stride after the touchdown keeps its planned phases.
        const scalar_t unused = events[k] - newTime;
        for (size_t i = k + 1; i < events.size(); ++i) {
          events[i] -= unused;
        }
      }
      events[k] = newTime;
      hold[foot] = ContactSyncHold{};
      result.advancedFoot = static_cast<int>(foot);
      result.advancedBy = lead - (newTime - time);
    } else {
      // Late half: the schedule has the foot down since events[k - 1] but it is still in the air. The
      // probe is the late half for a sole that has reached the plane the plan aimed at and is still going
      // down through it (Descending: the ground the plan assumed is not there); a sole still above the plane
      // is merely late and is loaded as before, a stopped sole gets the late window only.
      // A probe also needs a stride to follow: the phase after this touchdown's double support is a single
      // support. The last touchdown before a stand is loaded where it is (ContactSync.h).
      const bool descending = landing == ContactSyncLanding::Descending;
      const bool strideFollows = k + 1 < modes.size() && (modes[k + 1] == ModeNumber::LF || modes[k + 1] == ModeNumber::RF);
      const scalar_t window = std::max(settings.lateWindow, descending && strideFollows ? settings.probeWindow : 0.0);
      if (window <= 0.0 || k == 0 || modeNumber2StanceLeg(modes[k - 1])[foot]) {
        hold[foot] = ContactSyncHold{};
        continue;
      }
      const bool held = std::isfinite(hold[foot].origin);
      const scalar_t planned = held ? hold[foot].origin : events[k - 1];
      if (time - planned <= window) {
        count();
      }
      if (landed) {
        hold[foot] = ContactSyncHold{};
        continue;
      }
      const bool probing = descending && window > settings.lateWindow;
      scalar_t newTime = std::min(time + settings.holdStep, planned + window);
      if (!probing) {
        // The late half eats the double support, down to the minimum phase.
        newTime = std::min(newTime, events[k] - std::max(settings.minPhaseDuration, scalar_t(1e-3)));
      }
      if (newTime <= events[k - 1] || newTime <= time) {
        hold[foot] = ContactSyncHold{};  // the window is used up: the plan loads the foot where it is
        continue;
      }
      if (probing) {
        // A probed step simply takes longer: the rest of the schedule slides with its touchdown, the
        // double support and the other foot's swing keep their lengths.
        const scalar_t shift = newTime - events[k - 1];
        for (size_t i = k; i < events.size(); ++i) {
          events[i] += shift;
        }
      }
      events[k - 1] = newTime;
      hold[foot] = ContactSyncHold{planned, probing};
      result.heldFoot = static_cast<int>(foot);
      result.heldBy = newTime - planned;
      result.probing = probing;
    }
  }
  return result;
}

}  // namespace ocs2::humanoid
