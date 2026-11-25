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
#include <ocs2_core/reference/ModeSchedule.h>

#include "legged_robot_mpc_controller/mpc/common/gait/ModeSequenceTemplate.h"

namespace ocs2::humanoid {

/**
 * Walking gait as a continuous function of the commanded speed (ocs2.gait.continuous).
 *
 * Instead of a ladder of fixed templates (walk / trot / run, each with its own cadence, switched
 * when the command crosses a band edge), the step time, the double-support time and the swing
 * clearance are interpolated between speed knots, and the gait schedule is re-tiled at every
 * solve from the next step boundary on. Nothing that has already begun is touched, so the
 * contact pattern changes phase-continuously as the command changes.
 *
 * The shipped knots are human gait regressions scaled to the robot's leg length
 * (docs/results/lowspeed_gait.md): the step length that results, speed x step time, is what a
 * person's eye reads as walking; a fixed cadence gives short shuffling steps at low speed.
 */
struct ContinuousGaitConfig {
  bool enabled = false;
  //! Increasing commanded speeds [m/s] the three laws below are tabulated at; values are clamped outside.
  std::vector<scalar_t> speedKnots;
  //! Time from one foot's lift-off to the other foot's lift-off [s] at each knot (= swing + double support).
  std::vector<scalar_t> stepTime;
  //! Double-support time between the two swings [s] at each knot; below minDoubleSupport none is planned.
  std::vector<scalar_t> doubleSupport;
  //! Swing clearance [m] at each knot.
  std::vector<scalar_t> swingHeight;
  //! A yaw-rate command counts as this lever arm [m] times the rate towards the gait speed (turning in place steps).
  scalar_t yawLever = 0.15;
  //! Double supports shorter than this [s] are dropped (two contact switches for nothing).
  scalar_t minDoubleSupport = 0.03;
  //! The schedule is re-tiled from the first step boundary later than initTime + spliceDelay [s].
  scalar_t spliceDelay = 0.1;
  //! Planar command magnitudes below this [m/s] are rounded to zero (stand) ...
  scalar_t standBelow = 0.0;
  //! ... and those between standBelow and this [m/s] are raised to it, direction kept: nobody walks slower.
  scalar_t minWalkSpeed = 0.0;
};

/** One evaluation of the law. */
struct ContinuousGaitPhases {
  scalar_t swing = 0.5;          //!< single-support (swing) time [s]
  scalar_t doubleSupport = 0.0;  //!< double-support time [s], 0 = alternating single supports
  scalar_t swingHeight = -1.0;   //!< [m], -1 inherits the planner's height
  scalar_t stepTime() const { return swing + doubleSupport; }
};

/** Validates the tables (sizes, monotone knots, positive phases). Throws std::invalid_argument. */
void validateContinuousGaitConfig(const ContinuousGaitConfig& config);

/** Gait speed the law is evaluated at: planar speed plus the yaw-rate lever term. */
scalar_t continuousGaitSpeed(const ContinuousGaitConfig& config, scalar_t vx, scalar_t vy, scalar_t yawRate);

/** Piecewise-linear evaluation at `speed`, clamped to the knot range. */
ContinuousGaitPhases evaluateContinuousGait(const ContinuousGaitConfig& config, scalar_t speed);

/**
 * One gait cycle for the phases, starting with the single support `nextSwing` (LF or RF = that foot in
 * stance, the other swinging): swing, double support (if any), the other swing, double support.
 * Period = 2 x stepTime.
 */
ModeSequenceTemplate continuousGaitTemplate(const ContinuousGaitPhases& phases, size_t nextSwing);

/**
 * Where the law splices into `schedule`: the start of the first single support later than `earliest`
 * (`nextSwing` is its mode). The phase before it - a double support of the previous law, or the rest
 * stance before the first step - keeps the duration the schedule had. Returns false when no single
 * support starts after `earliest` (no walking planned).
 */
bool continuousGaitSplicePoint(const ModeSchedule& schedule, scalar_t earliest, scalar_t& spliceTime, size_t& nextSwing);

}  // namespace ocs2::humanoid
