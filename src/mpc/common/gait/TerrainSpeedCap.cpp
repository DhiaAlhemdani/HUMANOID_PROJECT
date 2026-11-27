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
}

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
}

scalar_t TerrainSpeedCap::update(scalar_t time, const contact_flag_t& contacts, const feet_array_t<scalar_t>& soleHeights) {
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
    const scalar_t step = std::abs(soleHeights[0] - soleHeights[1]);
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
  active_ = std::isfinite(capValue_);
  return capValue_;
}

}  // namespace ocs2::humanoid
