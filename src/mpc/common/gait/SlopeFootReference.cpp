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

#include <Eigen/Geometry>

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
  if (!input.gradient.allFinite() || !input.velocity.allFinite() || !std::isfinite(input.yawRate)) {
    throw std::invalid_argument("[SlopeFootReference] gradient, velocity and yaw rate must be finite");
  }

  SlopeFootReferencePlan plan;
  const bool standing = input.velocity.norm() < input.standingSpeed;
  const vector2_t v = standing ? vector2_t::Zero() : input.velocity;
  plan.strideVelocity = v;

  // The anchor: the stance foot furthest along the travel direction (the one that landed last in a
  // walk); the stance feet's midpoint when standing or with no foot on the ground (flight).
  int anchorFoot = -1;
  vector2_t anchorPosition = vector2_t::Zero();
  scalar_t anchorHeight = 0.0;
  {
    size_t numContacts = 0;
    for (size_t foot = 0; foot < N_CONTACTS; ++foot) {
      if (input.contacts[foot]) {
        ++numContacts;
        anchorPosition += input.solePositions[foot];
        anchorHeight += input.soleHeights[foot];
      }
    }
    if (numContacts == 0) {
      for (size_t foot = 0; foot < N_CONTACTS; ++foot) {
        anchorPosition += input.solePositions[foot];
        anchorHeight += input.soleHeights[foot];
      }
      numContacts = N_CONTACTS;
    }
    anchorPosition /= static_cast<scalar_t>(numContacts);
    anchorHeight /= static_cast<scalar_t>(numContacts);
    if (!standing) {
      const vector2_t direction = v.normalized();
      scalar_t best = -std::numeric_limits<scalar_t>::infinity();
      for (size_t foot = 0; foot < N_CONTACTS; ++foot) {
        if (input.contacts[foot] && direction.dot(input.solePositions[foot]) > best) {
          best = direction.dot(input.solePositions[foot]);
          anchorFoot = static_cast<int>(foot);
        }
      }
      if (anchorFoot >= 0) {
        anchorPosition = input.solePositions[anchorFoot];
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
  plan.anchorPosition = anchorPosition;
  plan.anchorHeight = anchorHeight;
  plan.anchorTime = anchorTime;

  // The plane, anchored.
  const auto plane = [&](const vector2_t& p) { return anchorHeight + input.gradient.dot(p - anchorPosition); };

  // Where foot `foot` falls at `time`. Walking: v x elapsed along the anchor's track, offset sideways by the
  // foot's own lateral position (the stance width; the anchor foot itself has none). Standing: where the foot
  // is, swung about the base by the yaw rate (a turn in place).
  const auto footfallPosition = [&](size_t foot, scalar_t time) {
    if (!standing) {
      const vector2_t direction = v.normalized();
      const vector2_t offset = input.solePositions[foot] - anchorPosition;
      const vector2_t lateral = static_cast<int>(foot) == anchorFoot ? vector2_t::Zero() : vector2_t(offset - offset.dot(direction) * direction);
      return vector2_t(anchorPosition + v * (time - anchorTime) + lateral);
    }
    const scalar_t angle = input.yawRate * (time - input.initTime);
    const Eigen::Rotation2D<scalar_t> rotation(angle);
    return vector2_t(input.basePosition + rotation * (input.solePositions[foot] - input.basePosition));
  };
  const auto footfallHeight = [&](size_t foot, scalar_t time) { return plane(footfallPosition(foot, time)); };

  for (size_t foot = 0; foot < N_CONTACTS; ++foot) {
    const auto& contacts = contactSequence[foot];
    scalar_t footHeight;
    if (input.contacts[foot]) {
      footHeight = input.soleHeights[foot];  // its own height: no pull towards a shared level
    } else if (standing) {
      footHeight = plane(input.solePositions[foot]);  // in the air, from about where it is
    } else {
      // In the air: its lift-off height is where it last touched down - the footfall before the anchor's,
      // i.e. the plane at that touchdown time (NOT at the lift-off time: the rule maps touchdown times to
      // positions, and this foot stood still from its touchdown to its lift-off). One period before the
      // anchor when its touchdown is not in the schedule.
      bool found = false;
      for (size_t e = current; e-- > 0;) {
        if (isTouchDown(contacts, e)) {
          footHeight = footfallHeight(foot, eventTimes[e]);
          found = true;
          break;
        }
      }
      if (!found) {
        footHeight = footfallHeight(foot, anchorTime - period);
      }
    }
    plan.liftOffHeights[foot].assign(numPhases, footHeight);
    plan.touchDownHeights[foot].assign(numPhases, footHeight + input.touchDownHeightOffset);
    for (size_t p = 0; p < numPhases; ++p) {
      plan.liftOffHeights[foot][p] = footHeight;
      plan.touchDownHeights[foot][p] = footHeight + input.touchDownHeightOffset;
      if (p >= current && p < eventTimes.size() && isTouchDown(contacts, p)) {
        const scalar_t touchDown = footfallHeight(foot, eventTimes[p]);
        plan.touchDownHeights[foot][p] = touchDown + input.touchDownHeightOffset;
        footHeight = touchDown;  // the stance that follows
      }
    }
  }
  return plan;
}

vector3_t slopePlaneNormal(const vector2_t& gradient) {
  return vector3_t(-gradient(0), -gradient(1), 1.0).normalized();
}

}  // namespace ocs2::humanoid
