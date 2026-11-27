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
  if (s.earlyWindow == 0.0 && s.lateWindow == 0.0) {
    throw std::invalid_argument("[ContactSync] enabled with both windows 0: nothing to do; disable it instead");
  }
  if (!(s.descentVelocity >= 0.0) || !(s.blockedHeight >= 0.0)) {
    throw std::invalid_argument("[ContactSync] descentVelocity and blockedHeight must be >= 0");
  }
  if (s.blockedHeight > 0.0 && s.descentVelocity <= 0.0) {
    throw std::invalid_argument("[ContactSync] blockedHeight needs descentVelocity > 0 to tell a stopped sole from a descending one");
  }
}

bool contactSyncLanded(const ContactSyncSettings& s, const ContactSyncFootObservation& f) {
  const bool nearPlane = f.soleHeight <= s.heightThreshold;
  const bool velocityKnown = std::isfinite(f.soleVelocity);
  if (nearPlane) {
    // Passing the plane on the way down: the ground is lower than the plane assumed.
    return !(s.descentVelocity > 0.0 && velocityKnown && f.soleVelocity < -s.descentVelocity);
  }
  if (s.blockedHeight <= 0.0 || !velocityKnown || !std::isfinite(f.plannedHeight) || !std::isfinite(f.plannedVelocity)) {
    return false;
  }
  // Stopped above a descending (or finished) plan: the ground is higher than the plane assumed.
  const bool stopped = f.soleVelocity >= -s.descentVelocity;
  const bool planDescending = f.plannedVelocity <= 0.0;
  return stopped && planDescending && f.soleHeight - f.plannedHeight >= s.blockedHeight;
}


ContactSyncResult synchroniseTouchdowns(const ContactSyncSettings& settings, scalar_t time,
                                        const feet_array_t<ContactSyncFootObservation>& feet, ModeSchedule& schedule,
                                        feet_array_t<scalar_t>& holdOrigin) {
  ContactSyncResult result;
  constexpr scalar_t kNaN = std::numeric_limits<scalar_t>::quiet_NaN();
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
    const bool landed = contactSyncLanded(settings, feet[foot]);
    if (!now[foot]) {
      // Early half: the foot swings and its touchdown is the next event.
      if (settings.earlyWindow <= 0.0 || k >= events.size() || !modeNumber2StanceLeg(modes[k + 1])[foot] || !landed) {
        continue;
      }
      const scalar_t lead = events[k] - time;
      if (lead <= 0.0 || lead > settings.earlyWindow) {
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
      events[k] = newTime;
      holdOrigin[foot] = kNaN;
      result.advancedFoot = static_cast<int>(foot);
      result.advancedBy = lead - (newTime - time);
    } else {
      // Late half: the schedule has the foot down since events[k - 1] but it is still in the air.
      if (settings.lateWindow <= 0.0 || k == 0 || modeNumber2StanceLeg(modes[k - 1])[foot] || landed) {
        holdOrigin[foot] = kNaN;
        continue;
      }
      const scalar_t planned = std::isnan(holdOrigin[foot]) ? events[k - 1] : holdOrigin[foot];
      scalar_t newTime = std::min(time + settings.holdStep, planned + settings.lateWindow);
      newTime = std::min(newTime, events[k] - std::max(settings.minPhaseDuration, scalar_t(1e-3)));
      if (newTime <= events[k - 1] || newTime <= time) {
        holdOrigin[foot] = kNaN;  // the window is used up: the plan loads the foot where it is
        continue;
      }
      events[k - 1] = newTime;
      holdOrigin[foot] = planned;
      result.heldFoot = static_cast<int>(foot);
      result.heldBy = newTime - planned;
    }
  }
  return result;
}

}  // namespace ocs2::humanoid
