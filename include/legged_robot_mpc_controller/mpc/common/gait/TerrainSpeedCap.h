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

#include <array>
#include <limits>
#include <vector>
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
 * last stride took (frame-independent: same instant, same estimator frame). Tiers: a step of at least
 * `stepKnots[i]` marks the ground as uneven at tier i for `holdSeconds`; while any tier holds, the
 * commanded planar speed is capped at the smallest `speedKnots[i]` of the tiers that hold (the slew
 * limiter of the velocity command turns the cap into a ramp). Run 24: 1-2 cm steps are walked at
 * 0.5 m/s and lost at 0.8; 2-4 cm steps are walked at 0.2 and lost at 0.5; a 3.5 cm step says the
 * big edges are here (0.1). Note that a sole straddling an edge reads about half the edge height, so
 * the knots are set below the terrain steps they are meant to catch. Flat ground never trips
 * it: both soles sit on the same plane within millimetres.
 *
 * Slopes (run 24 F / 25): on an incline the two stance feet differ in height by stride x slope, so
 * the raw step reads a 5 deg ramp as roughness - and, worse, as roughness proportional to the
 * speed, which makes the cap release at a crawl and re-trip after the next long stride (the slope_5
 * forward fall). The signed step divided by the feet's separation along the heading is a slope
 * sample; when the last three agree within `slopeConsistency` the ground is a plane, the roughness
 * step becomes the residual from that plane, and the incline gets its own cap: `|slope| >=
 * slopeKnots[i]` holds `slopeSpeedKnots[i]`. Tile fields do not agree stride to stride (random
 * sign), so they keep the raw step and the roughness tiers. A standing plane falls only to a sample
 * that contradicts it in height - its rise off by more than max(slopeConsistency x run,
 * slopeHeightTolerance) - so the millimetres of sole noise on a short double support (a stop's
 * closing steps) keep the plane, while a full stride that misses it by centimetres (the plateau
 * after a ramp) drops it at once (run 25 C / 25 E); three agreeing samples build the next one. The
 * sample is read at the end of the double support, when the landed foot carries load: at its first
 * solve a foot planned above the ground is still in the air and would confirm the plane that put it
 * there (run 25 F).
 */
struct TerrainSpeedCapSettings {
  bool enabled = false;
  std::vector<scalar_t> stepKnots{0.01, 0.02, 0.035};  //!< [m] sole-height differences in double support that count as a terrain step, increasing
  std::vector<scalar_t> speedKnots{0.5, 0.2, 0.1};     //!< [m/s] planar speed cap of each tier, decreasing
  scalar_t holdSeconds = 3.0;                          //!< [s] a tier stays after its last terrain step (about 4-5 strides)
  std::vector<scalar_t> slopeKnots{0.05, 0.12};        //!< [-] |rise / run| along the heading that counts as an incline (0.05 ~ 3 deg, 0.12 ~ 7 deg), increasing
  std::vector<scalar_t> slopeSpeedKnots{0.3, 0.2};     //!< [m/s] planar speed cap on each incline tier, non-increasing
  scalar_t slopeMinSeparation = 0.08;                  //!< [m] feet closer than this along the heading give no slope sample (standing)
  scalar_t slopeConsistency = 0.03;                    //!< [-] spread of the last three slope samples below which the ground counts as a plane; also the per-run tolerance of a sample against a standing plane
  scalar_t slopeHeightTolerance = 0.01;                //!< [m] height tolerance floor of a sample against a standing plane (sole noise on a short base)
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
   * @param soleAlong   sole positions projected on the heading [m] (same frame); feeds the slope estimate.
   * @return the speed cap in force: the smallest `speedKnots` / `slopeSpeedKnots` of the tiers that hold, +inf otherwise.
   */
  scalar_t update(scalar_t time, const contact_flag_t& contacts, const feet_array_t<scalar_t>& soleHeights,
                  const feet_array_t<scalar_t>& soleAlong);
  scalar_t update(scalar_t time, const contact_flag_t& contacts, const feet_array_t<scalar_t>& soleHeights) {
    return update(time, contacts, soleHeights, feet_array_t<scalar_t>{0.0, 0.0});  // no slope samples
  }

  bool active() const { return active_; }
  scalar_t lastStep() const { return lastStep_; }          //!< [m] largest step of the last double support that tripped the cap
  scalar_t lastStepTime() const { return lastStepTime_; }  //!< [s] when a step was last seen (NaN = never)
  scalar_t capValue() const { return capValue_; }          //!< [m/s] the cap in force (+inf = none)
  scalar_t slope() const { return slope_; }                //!< [-] slope of the last plane three samples agreed on (0 until some did; kept for the incline cap's hold after a contradiction)
  bool slopeConsistent() const { return slopeConsistent_; }  //!< a plane stands: three samples agreed and no later sample contradicted it in height
  size_t stepCount() const { return stepCount_; }          //!< double supports that tripped the cap so far
  void reset();

 private:
  TerrainSpeedCapSettings settings_;
  bool inDoubleSupport_ = false;
  bool countedThisSupport_ = false;
  bool active_ = false;
  scalar_t lastStep_ = 0.0;
  scalar_t lastStepTime_ = std::numeric_limits<scalar_t>::quiet_NaN();
  scalar_t capValue_ = std::numeric_limits<scalar_t>::infinity();
  std::vector<scalar_t> tierTime_;  //!< [s] per tier: when a step of that size was last seen (NaN = never)
  bool fitsPlane(scalar_t slope, scalar_t rise, scalar_t run) const;
  void takeSlopeSample(scalar_t dz, scalar_t dd);
  std::array<scalar_t, 3> slopeSamples_{};  //!< [-] rise / run of the last three double supports
  scalar_t lastDsDz_ = std::numeric_limits<scalar_t>::quiet_NaN();  //!< [m] signed sole-height step at the last solve of the current double support
  scalar_t lastDsDd_ = std::numeric_limits<scalar_t>::quiet_NaN();  //!< [m] the feet's separation along the heading there
  size_t slopeSampleCount_ = 0;
  scalar_t slope_ = 0.0;
  bool slopeConsistent_ = false;
  scalar_t slopeTime_ = std::numeric_limits<scalar_t>::quiet_NaN();  //!< [s] last solve with a consistent incline sample
  scalar_t previousTime_ = std::numeric_limits<scalar_t>::quiet_NaN();
  size_t stepCount_ = 0;
};

}  // namespace ocs2::humanoid
