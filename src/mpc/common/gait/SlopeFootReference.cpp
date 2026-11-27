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

#include "legged_robot_mpc_controller/mpc/common/gait/SlopeFootReference.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace ocs2::humanoid {

namespace {

// Event index e is a touchdown of the foot when phase e is a swing and phase e + 1 a stance.
bool isTouchDown(const std::vector<bool>& contacts, size_t e) {
  return e + 1 < contacts.size() && !contacts[e] && contacts[e + 1];
}

}  // namespace

SlopeFootReferencePlan planSlopeFootReference(const SlopeFootReferenceInput& input, const scalar_array_t& eventTimes,
                                              const feet_array_t<std::vector<bool>>& contactSequence) {
  const size_t numPhases = contactSequence[0].size();
  for (const auto& sequence : contactSequence) {
    if (sequence.size() != numPhases) {
      throw std::invalid_argument("[SlopeFootReference] contact sequences must have the same length");
    }
  }
  if (numPhases == 0 || eventTimes.size() + 1 != numPhases) {
    throw std::invalid_argument("[SlopeFootReference] event times must be one shorter than the phase sequence");
  }
  if (!std::isfinite(input.slope) || !std::isfinite(input.velocityAlong)) {
    throw std::invalid_argument("[SlopeFootReference] slope and velocity must be finite");
  }

  SlopeFootReferencePlan plan;
  const scalar_t v = std::abs(input.velocityAlong) < input.standingSpeed ? 0.0 : input.velocityAlong;
  plan.strideVelocity = v;

  // The anchor: the stance foot furthest along the travel direction (the one that landed last in a
  // walk); the stance feet's midpoint when standing or with no foot on the ground (flight).
  int anchorFoot = -1;
  scalar_t anchorAlong = 0.0, anchorHeight = 0.0;
  {
    size_t numContacts = 0;
    for (size_t foot = 0; foot < N_CONTACTS; ++foot) {
      if (input.contacts[foot]) {
        ++numContacts;
        anchorAlong += input.soleAlong[foot];
        anchorHeight += input.soleHeights[foot];
      }
    }
    if (numContacts == 0) {
      for (size_t foot = 0; foot < N_CONTACTS; ++foot) {
        anchorAlong += input.soleAlong[foot];
        anchorHeight += input.soleHeights[foot];
      }
      numContacts = N_CONTACTS;
    }
    anchorAlong /= static_cast<scalar_t>(numContacts);
    anchorHeight /= static_cast<scalar_t>(numContacts);
    if (v != 0.0) {
      const scalar_t direction = v > 0.0 ? 1.0 : -1.0;
      scalar_t best = -std::numeric_limits<scalar_t>::infinity();
      for (size_t foot = 0; foot < N_CONTACTS; ++foot) {
        if (input.contacts[foot] && direction * input.soleAlong[foot] > best) {
          best = direction * input.soleAlong[foot];
          anchorFoot = static_cast<int>(foot);
        }
      }
      if (anchorFoot >= 0) {
        anchorAlong = input.soleAlong[anchorFoot];
        anchorHeight = input.soleHeights[anchorFoot];
      }
    }
  }

  // Phase index at initTime; events before it are the schedule's past.
  const size_t current =
      static_cast<size_t>(std::upper_bound(eventTimes.begin(), eventTimes.end(), input.initTime) - eventTimes.begin());

  // Touchdown spacing, from the scheduled touchdowns of both feet (the fallback when fewer than two).
  scalar_array_t futureTouchDowns;
  for (size_t e = current; e < eventTimes.size(); ++e) {
    for (size_t foot = 0; foot < N_CONTACTS; ++foot) {
      if (isTouchDown(contactSequence[foot], e)) {
        futureTouchDowns.push_back(eventTimes[e]);
      }
    }
  }
  std::sort(futureTouchDowns.begin(), futureTouchDowns.end());
  const scalar_t period =
      futureTouchDowns.size() >= 2 ? std::max(futureTouchDowns[1] - futureTouchDowns[0], 1e-3) : input.fallbackStepPeriod;

  // The anchor's touchdown time: from the schedule when it is there, else one touchdown spacing
  // before the first future touchdown (mid-stance when none is scheduled).
  scalar_t anchorTime = input.initTime;
  {
    bool found = false;
    if (anchorFoot >= 0) {
      for (size_t e = current; e-- > 0;) {
        if (isTouchDown(contactSequence[anchorFoot], e)) {
          anchorTime = eventTimes[e];
          found = true;
          break;
        }
      }
    }
    if (!found) {
      anchorTime = futureTouchDowns.empty() ? input.initTime - 0.5 * period : futureTouchDowns.front() - period;
    }
  }
  plan.anchorAlong = anchorAlong;
  plan.anchorHeight = anchorHeight;
  plan.anchorTime = anchorTime;

  // Height of the plane at the footfall that happens at `time` (anchor + v x elapsed along the heading).
  const auto footfallHeight = [&](scalar_t time) { return anchorHeight + input.slope * v * (time - anchorTime); };

  for (size_t foot = 0; foot < N_CONTACTS; ++foot) {
    const auto& contacts = contactSequence[foot];
    scalar_t footHeight;
    if (input.contacts[foot]) {
      footHeight = input.soleHeights[foot];  // its own height: no pull towards a shared level
    } else {
      // In the air: its lift-off height is where it last touched down - the footfall before the anchor's,
      // i.e. the plane at that touchdown time (NOT at the lift-off time: the rule maps touchdown times to
      // positions, and this foot stood still from its touchdown to its lift-off). One period before the
      // anchor when its touchdown is not in the schedule.
      bool found = false;
      for (size_t e = current; e-- > 0;) {
        if (isTouchDown(contacts, e)) {
          footHeight = footfallHeight(eventTimes[e]);
          found = true;
          break;
        }
      }
      if (!found) {
        footHeight = footfallHeight(anchorTime - period);
      }
    }
    plan.liftOffHeights[foot].assign(numPhases, footHeight);
    plan.touchDownHeights[foot].assign(numPhases, footHeight + input.touchDownHeightOffset);
    for (size_t p = 0; p < numPhases; ++p) {
      plan.liftOffHeights[foot][p] = footHeight;
      plan.touchDownHeights[foot][p] = footHeight + input.touchDownHeightOffset;
      if (p >= current && p < eventTimes.size() && isTouchDown(contacts, p)) {
        const scalar_t touchDown = footfallHeight(eventTimes[p]);
        plan.touchDownHeights[foot][p] = touchDown + input.touchDownHeightOffset;
        footHeight = touchDown;  // the stance that follows
      }
    }
  }
  return plan;
}

vector3_t slopePlaneNormal(scalar_t slope, scalar_t yaw) {
  return vector3_t(-slope * std::cos(yaw), -slope * std::sin(yaw), 1.0).normalized();
}

}  // namespace ocs2::humanoid
