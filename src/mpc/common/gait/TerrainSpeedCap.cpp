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

#include "legged_robot_mpc_controller/mpc/common/gait/TerrainSpeedCap.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ocs2::humanoid {

void validateTerrainSpeedCapSettings(const TerrainSpeedCapSettings& s) {
  if (!s.enabled) {
    return;
  }
  if (!(s.holdSeconds > 0.0)) {
    throw std::invalid_argument("[TerrainSpeedCap] holdSeconds must be > 0");
  }
  if (s.stepKnots.empty() || s.stepKnots.size() != s.speedKnots.size()) {
    throw std::invalid_argument("[TerrainSpeedCap] stepKnots and speedKnots must have the same, non-zero length");
  }
  for (size_t i = 0; i < s.stepKnots.size(); ++i) {
    if (!(s.stepKnots[i] > 0.0) || !(s.speedKnots[i] > 0.0)) {
      throw std::invalid_argument("[TerrainSpeedCap] stepKnots and speedKnots must be > 0");
    }
    if (i > 0 && (!(s.stepKnots[i] > s.stepKnots[i - 1]) || !(s.speedKnots[i] <= s.speedKnots[i - 1]))) {
      throw std::invalid_argument("[TerrainSpeedCap] stepKnots must increase and speedKnots must not increase (a larger step, a lower cap)");
    }
  }
  if (s.slopeKnots.size() != s.slopeSpeedKnots.size()) {
    throw std::invalid_argument("[TerrainSpeedCap] slopeKnots and slopeSpeedKnots must have the same length (empty = no incline cap)");
  }
  for (size_t i = 0; i < s.slopeKnots.size(); ++i) {
    if (!(s.slopeKnots[i] > 0.0) || !(s.slopeSpeedKnots[i] > 0.0)) {
      throw std::invalid_argument("[TerrainSpeedCap] slopeKnots and slopeSpeedKnots must be > 0");
    }
    if (i > 0 && (!(s.slopeKnots[i] > s.slopeKnots[i - 1]) || !(s.slopeSpeedKnots[i] <= s.slopeSpeedKnots[i - 1]))) {
      throw std::invalid_argument("[TerrainSpeedCap] slopeKnots must increase and slopeSpeedKnots must not increase");
    }
  }
  if (!(s.slopeMinSeparation > 0.0) || !(s.slopeConsistency > 0.0)) {
    throw std::invalid_argument("[TerrainSpeedCap] slopeMinSeparation and slopeConsistency must be > 0");
  }
}

namespace {
scalar_t medianOf3(std::array<scalar_t, 3> v) {
  std::sort(v.begin(), v.end());
  return v[1];
}
}  // namespace

TerrainSpeedCap::TerrainSpeedCap(TerrainSpeedCapSettings settings) : settings_(std::move(settings)) {
  validateTerrainSpeedCapSettings(settings_);
}

void TerrainSpeedCap::setSettings(const TerrainSpeedCapSettings& settings) {
  validateTerrainSpeedCapSettings(settings);
  settings_ = settings;
  reset();
}

void TerrainSpeedCap::reset() {
  inDoubleSupport_ = false;
  countedThisSupport_ = false;
  active_ = false;
  lastStep_ = 0.0;
  lastStepTime_ = std::numeric_limits<scalar_t>::quiet_NaN();
  capValue_ = std::numeric_limits<scalar_t>::infinity();
  tierTime_.assign(settings_.stepKnots.size(), std::numeric_limits<scalar_t>::quiet_NaN());
  previousTime_ = std::numeric_limits<scalar_t>::quiet_NaN();
  stepCount_ = 0;
  slopeSamples_.fill(0.0);
  slopeSampleCount_ = 0;
  slope_ = 0.0;
  slopeConsistent_ = false;
  planeFresh_ = false;
  slopeTime_ = std::numeric_limits<scalar_t>::quiet_NaN();
}

scalar_t TerrainSpeedCap::update(scalar_t time, const contact_flag_t& contacts, const feet_array_t<scalar_t>& soleHeights,
                                 const feet_array_t<scalar_t>& soleAlong) {
  constexpr scalar_t kNoCap = std::numeric_limits<scalar_t>::infinity();
  if (!settings_.enabled) {
    active_ = false;
    capValue_ = kNoCap;
    return kNoCap;
  }
  if (tierTime_.size() != settings_.stepKnots.size()) {
    reset();
  }
  if (std::isfinite(previousTime_) && time < previousTime_) {
    reset();  // time went backwards: a new run
  }
  previousTime_ = time;

  const bool doubleSupport = contacts[0] && contacts[1];
  if (doubleSupport) {
    const scalar_t dz = soleHeights[0] - soleHeights[1];
    const scalar_t dd = soleAlong[0] - soleAlong[1];
    // One slope sample per double support, from the first solve in it (the landed foot is on the ground).
    if (!inDoubleSupport_ && std::isfinite(dz) && std::isfinite(dd) && std::abs(dd) >= settings_.slopeMinSeparation) {
      slopeSamples_[slopeSampleCount_ % slopeSamples_.size()] = dz / dd;
      ++slopeSampleCount_;
      if (slopeSampleCount_ >= slopeSamples_.size()) {
        const auto [lo, hi] = std::minmax_element(slopeSamples_.begin(), slopeSamples_.end());
        planeFresh_ = *hi - *lo <= settings_.slopeConsistency;
        if (planeFresh_) {
          slopeConsistent_ = true;
          slope_ = medianOf3(slopeSamples_);
        }
        // A triple that disagrees keeps the last plane the feet agreed on. One short or noisy double support
        // (a sample on an 8 cm base carries +-0.05 of sole-height noise) is not evidence of flat ground - flat
        // ground announces itself with three agreeing ~0 samples - and zeroing the plane mid-incline flipped
        // the foot references and the contact normal to flat exactly at the stop's short closing steps
        // (run 25 C, slope_10 stops).
      }
    }
    if (slopeConsistent_) {
      slopeTime_ = time;
    }
    // Roughness: the step out of the plane the feet agree on; the raw step while no plane exists. While the
    // current triple disagrees with a kept plane the ground may be either (the plateau after a ramp, a noisy
    // stride on the ramp): a step counts only if neither level ground nor the plane explains it.
    const scalar_t fromPlane = std::abs(dz - slope_ * dd);
    const scalar_t step = !slopeConsistent_ ? std::abs(dz) : planeFresh_ ? fromPlane : std::min(fromPlane, std::abs(dz));
    if (std::isfinite(step) && step >= settings_.stepKnots.front()) {
      if (!inDoubleSupport_ || !countedThisSupport_) {
        ++stepCount_;
        countedThisSupport_ = true;
        lastStep_ = step;
      } else {
        lastStep_ = std::max(lastStep_, step);
      }
      lastStepTime_ = time;
      for (size_t i = 0; i < settings_.stepKnots.size(); ++i) {
        if (step >= settings_.stepKnots[i]) {
          tierTime_[i] = time;
        }
      }
    }
  } else {
    countedThisSupport_ = false;
  }
  inDoubleSupport_ = doubleSupport;

  capValue_ = kNoCap;
  for (size_t i = 0; i < tierTime_.size(); ++i) {
    if (std::isfinite(tierTime_[i]) && time - tierTime_[i] < settings_.holdSeconds) {
      capValue_ = std::min(capValue_, settings_.speedKnots[i]);
    }
  }
  if (std::isfinite(slopeTime_) && time - slopeTime_ < settings_.holdSeconds) {
    for (size_t i = 0; i < settings_.slopeKnots.size(); ++i) {
      if (std::abs(slope_) >= settings_.slopeKnots[i]) {
        capValue_ = std::min(capValue_, settings_.slopeSpeedKnots[i]);
      }
    }
  }
  active_ = std::isfinite(capValue_);
  return capValue_;
}

}  // namespace ocs2::humanoid
