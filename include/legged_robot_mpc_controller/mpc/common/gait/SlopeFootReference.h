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

#include <vector>

#include <ocs2_core/Types.h>

#include "legged_robot_mpc_controller/mpc/common/model/Types.h"

namespace ocs2::humanoid {

/**
 * Foot references on an incline the blind terrain cap has read (run 25 C of
 * docs/results/human_gait_template.md).
 *
 * The flat-ground swing update gives both feet one support height (the mean of the stance soles)
 * and every foot-orientation term the world vertical as the ground normal. On a plane that is wrong
 * in two ways that compound: the stance height constraint pulls the rear foot up and the front foot
 * down by half the sole-height difference (1.75 cm on a 10 deg ramp), and the orientation terms
 * (task-space foot cost, stance orientation feedback) hold the sole level, so it rests on its heel
 * or toe edge - the strides tables read `edge` 3.0 cm at every landing on 10 deg, and standing on two
 * edges is how slope_10 ends.
 *
 * Once TerrainSpeedCap's slope samples agree the ground is a plane, this plans the per-foot support
 * heights on it:
 *  - a stance foot's reference is its own measured sole height (no pull, whatever the plane fit);
 *  - the plan is anchored at the leading stance foot (the one furthest along the travel direction):
 *    footfalls of a periodic gait at speed v are v x (time since the anchor's touchdown) further
 *    along, so the n-th touchdown in the horizon is planned at anchor height + slope x that
 *    distance; the foot in the air gets its lift-off height from the same rule at its own last
 *    touchdown (one footfall before the anchor's);
 *  - below `standingSpeed` the anchor is the midpoint of the stance feet and every touchdown is
 *    planned at its height (a first step from standing lands next to the feet).
 * The ground normal for the orientation terms is the plane's: slopePlaneNormal().
 *
 * Pure: no solver types beyond the mode schedule's arrays, so it unit-tests with a bare compiler.
 */
struct SlopeFootReferenceInput {
  scalar_t initTime = 0.0;                       //!< [s] solve time (phase index = events before it)
  scalar_t slope = 0.0;                          //!< [-] rise / run along the heading (TerrainSpeedCap::slope())
  scalar_t velocityAlong = 0.0;                  //!< [m/s] base velocity projected on the heading, signed
  contact_flag_t contacts{false, false};         //!< contact flags at initTime
  feet_array_t<scalar_t> soleHeights{0.0, 0.0};  //!< [m] sole heights, world
  feet_array_t<scalar_t> soleAlong{0.0, 0.0};    //!< [m] sole positions projected on the heading
  scalar_t touchDownHeightOffset = 0.0;          //!< [m] added to every touchdown height (SwingTrajectoryPlanner::Config)
  scalar_t standingSpeed = 0.05;                 //!< [m/s] |velocityAlong| below this plans every touchdown at the feet midpoint
  scalar_t fallbackStepPeriod = 0.7;             //!< [s] touchdown spacing assumed when the schedule shows fewer than two
};

struct SlopeFootReferencePlan {
  feet_array_t<scalar_array_t> liftOffHeights;    //!< per foot, per phase (SwingTrajectoryPlanner::update layout)
  feet_array_t<scalar_array_t> touchDownHeights;  //!< per foot, per phase, touchDownHeightOffset included
  scalar_t anchorAlong = 0.0;                     //!< [m] where the footfall sequence is anchored
  scalar_t anchorHeight = 0.0;                    //!< [m]
  scalar_t anchorTime = 0.0;                      //!< [s] the anchor foot's touchdown (estimated when not in the schedule)
  scalar_t strideVelocity = 0.0;                  //!< [m/s] velocityAlong after the standing rule
};

/**
 * @param input           see SlopeFootReferenceInput.
 * @param eventTimes      mode schedule event times, size = phases - 1.
 * @param contactSequence per foot, the contact flag of each phase (modeNumber2StanceLeg over the mode sequence).
 */
SlopeFootReferencePlan planSlopeFootReference(const SlopeFootReferenceInput& input, const scalar_array_t& eventTimes,
                                              const feet_array_t<std::vector<bool>>& contactSequence);

/** Unit normal of the plane rising by `slope` per metre along the heading `yaw` (world frame). */
vector3_t slopePlaneNormal(scalar_t slope, scalar_t yaw);

}  // namespace ocs2::humanoid
