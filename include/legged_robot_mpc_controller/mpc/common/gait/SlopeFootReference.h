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
 * Once TerrainSpeedCap's slope samples agree the ground is a plane - a world-frame gradient (run 26) -
 * this plans the per-foot support heights on it:
 *  - a stance foot's reference is its own measured sole height (no pull, whatever the plane fit);
 *  - the plane is anchored at the leading stance foot (the one furthest along the travel direction):
 *    plane(p) = anchor height + gradient . (p - anchor position);
 *  - walking, the footfall at time t is v x (t - the anchor's touchdown) further along the track,
 *    offset sideways by the foot's own lateral position relative to the anchor's (the stance width:
 *    across a slope the two tracks differ in height); the foot in the air gets its lift-off height
 *    from the same rule at its own last touchdown (one footfall before the anchor's);
 *  - below `standingSpeed` (standing, a turn in place) the anchor is the stance feet's midpoint and
 *    each foot lands where it is, swung about the base by the yaw rate until its touchdown - on a
 *    cross-slope the two feet differ in height by the stance width x the gradient across.
 * The ground normal for the orientation terms is the plane's: slopePlaneNormal().
 *
 * Pure: no solver types beyond the mode schedule's arrays, so it unit-tests with a bare compiler.
 */
struct SlopeFootReferenceInput {
  scalar_t initTime = 0.0;                             //!< [s] solve time (phase index = events before it)
  vector2_t gradient = vector2_t::Zero();              //!< [-] rise per metre, world xy (TerrainSpeedCap::gradient())
  vector2_t velocity = vector2_t::Zero();              //!< [m/s] base planar velocity, world
  scalar_t yawRate = 0.0;                              //!< [rad/s] base yaw rate: where a foot lands when turning in place
  vector2_t basePosition = vector2_t::Zero();          //!< [m] base planar position, world (the turn centre)
  contact_flag_t contacts{false, false};               //!< contact flags at initTime
  feet_array_t<scalar_t> soleHeights{0.0, 0.0};        //!< [m] sole heights, world
  feet_array_t<vector2_t> solePositions{vector2_t::Zero(), vector2_t::Zero()};  //!< [m] sole planar positions, world
  scalar_t touchDownHeightOffset = 0.0;                //!< [m] added to every touchdown height (SwingTrajectoryPlanner::Config)
  scalar_t standingSpeed = 0.05;                       //!< [m/s] |velocity| below this plans every touchdown where the foot is
  scalar_t fallbackStepPeriod = 0.7;                   //!< [s] touchdown spacing assumed when the schedule shows fewer than two
};

struct SlopeFootReferencePlan {
  feet_array_t<scalar_array_t> liftOffHeights;    //!< per foot, per phase (SwingTrajectoryPlanner::update layout)
  feet_array_t<scalar_array_t> touchDownHeights;  //!< per foot, per phase, touchDownHeightOffset included
  vector2_t anchorPosition = vector2_t::Zero();   //!< [m] where the plane is anchored
  scalar_t anchorHeight = 0.0;                    //!< [m]
  scalar_t anchorTime = 0.0;                      //!< [s] the anchor foot's touchdown (estimated when not in the schedule)
  vector2_t strideVelocity = vector2_t::Zero();   //!< [m/s] velocity after the standing rule
};

/**
 * @param input           see SlopeFootReferenceInput.
 * @param eventTimes      mode schedule event times, size = phases - 1.
 * @param contactSequence per foot, the contact flag of each phase (modeNumber2StanceLeg over the mode sequence).
 */
SlopeFootReferencePlan planSlopeFootReference(const SlopeFootReferenceInput& input, const scalar_array_t& eventTimes,
                                              const feet_array_t<std::vector<bool>>& contactSequence);

/** Unit normal of the plane rising by `gradient` per metre (world frame). */
vector3_t slopePlaneNormal(const vector2_t& gradient);

}  // namespace ocs2::humanoid
