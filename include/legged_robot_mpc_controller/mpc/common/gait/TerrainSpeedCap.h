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

#include "legged_robot_mpc_controller/mpc/common/model/Types.h"

namespace ocs2::humanoid {

/**
 * Blind terrain speed cap: slow down while the ground is uneven.
 *
 * Run 23 (docs/results/human_gait_template.md): every configuration that walks the block tiles at
 * 0.2-0.5 m/s falls or slips at 0.8 m/s, within a stride or two of a 1-3 cm step, with the hip
 * pitches at their torque limit. The walk is blind, so the only terrain signal it has is what the
 * feet report: when both feet are in contact, the difference of their sole heights is the step the
 * last stride took (frame-independent: same instant, same estimator frame). A step of at least
 * `stepThreshold` marks the ground as uneven for `holdSeconds`; while that holds, the commanded
 * planar speed is capped at `maxSpeed` (the slew limiter of the velocity command turns the cap into
 * a ramp). Flat ground never trips it: both soles sit on the same plane within millimetres.
 */
struct TerrainSpeedCapSettings {
  bool enabled = false;
  scalar_t stepThreshold = 0.01;  //!< [m] sole-height difference in double support that counts as a terrain step
  scalar_t maxSpeed = 0.5;        //!< [m/s] planar speed cap while the ground counts as uneven
  scalar_t holdSeconds = 3.0;     //!< [s] the cap stays after the last terrain step (about 4-5 strides)
};

void validateTerrainSpeedCapSettings(const TerrainSpeedCapSettings& settings);

class TerrainSpeedCap {
 public:
  explicit TerrainSpeedCap(TerrainSpeedCapSettings settings = {});

  void setSettings(const TerrainSpeedCapSettings& settings);
  const TerrainSpeedCapSettings& getSettings() const { return settings_; }

  /**
   * One update per solve.
   * @param time        the solve's start time.
   * @param contacts    contact flags of the mode the schedule has at `time`.
   * @param soleHeights measured sole heights [m] (any common frame).
   * @return the speed cap in force: `maxSpeed` while the ground counts as uneven, +inf otherwise.
   */
  scalar_t update(scalar_t time, const contact_flag_t& contacts, const feet_array_t<scalar_t>& soleHeights);

  bool active() const { return active_; }
  scalar_t lastStep() const { return lastStep_; }          //!< [m] largest step of the last double support that tripped the cap
  scalar_t lastStepTime() const { return lastStepTime_; }  //!< [s] when it was seen (NaN = never)
  size_t stepCount() const { return stepCount_; }          //!< double supports that tripped the cap so far
  void reset();

 private:
  TerrainSpeedCapSettings settings_;
  bool inDoubleSupport_ = false;
  bool countedThisSupport_ = false;
  bool active_ = false;
  scalar_t lastStep_ = 0.0;
  scalar_t lastStepTime_ = std::numeric_limits<scalar_t>::quiet_NaN();
  scalar_t previousTime_ = std::numeric_limits<scalar_t>::quiet_NaN();
  size_t stepCount_ = 0;
};

}  // namespace ocs2::humanoid
