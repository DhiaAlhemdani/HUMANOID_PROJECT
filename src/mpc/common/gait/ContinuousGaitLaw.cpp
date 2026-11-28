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

#include "legged_robot_mpc_controller/mpc/common/gait/ContinuousGaitLaw.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

#include "legged_robot_mpc_controller/mpc/common/gait/MotionPhaseDefinition.h"

namespace ocs2::humanoid {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void validateContinuousGaitConfig(const ContinuousGaitConfig& config) {
  const std::string where = "[ocs2.gait.continuous] ";
  const size_t n = config.speedKnots.size();
  if (n == 0) {
    throw std::invalid_argument(where + "speedKnots is empty");
  }
  if (config.stepTime.size() != n || config.doubleSupport.size() != n || config.swingHeight.size() != n) {
    throw std::invalid_argument(where + "stepTime, doubleSupport and swingHeight need one value per speed knot");
  }
  for (size_t i = 0; i < n; ++i) {
    if (i > 0 && !(config.speedKnots[i] > config.speedKnots[i - 1])) {
      throw std::invalid_argument(where + "speedKnots must increase");
    }
    if (!(config.stepTime[i] > 0.0) || config.doubleSupport[i] < 0.0 || !(config.stepTime[i] - config.doubleSupport[i] > 0.05)) {
      throw std::invalid_argument(where + "each knot needs stepTime > doubleSupport + 0.05 s and doubleSupport >= 0");
    }
  }
  if (config.yawLever < 0.0 || config.minDoubleSupport < 0.0 || config.spliceDelay < 0.0) {
    throw std::invalid_argument(where + "yawLever, minDoubleSupport and spliceDelay must be >= 0");
  }
  if (config.standBelow < 0.0 || config.minWalkSpeed < config.standBelow) {
    throw std::invalid_argument(where + "needs 0 <= standBelow <= minWalkSpeed");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t continuousGaitSpeed(const ContinuousGaitConfig& config, scalar_t vx, scalar_t vy, scalar_t yawRate) {
  return std::hypot(vx, vy) + config.yawLever * std::abs(yawRate);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
namespace {
scalar_t interpolate(const std::vector<scalar_t>& x, const std::vector<scalar_t>& y, scalar_t at) {
  if (at <= x.front()) return y.front();
  if (at >= x.back()) return y.back();
  const auto hi = std::upper_bound(x.begin(), x.end(), at);
  const size_t i = static_cast<size_t>(hi - x.begin());
  const scalar_t s = (at - x[i - 1]) / (x[i] - x[i - 1]);
  return y[i - 1] + s * (y[i] - y[i - 1]);
}
}  // namespace

ContinuousGaitPhases evaluateContinuousGait(const ContinuousGaitConfig& config, scalar_t speed) {
  ContinuousGaitPhases phases;
  const scalar_t stepTime = interpolate(config.speedKnots, config.stepTime, speed);
  scalar_t doubleSupport = interpolate(config.speedKnots, config.doubleSupport, speed);
  if (doubleSupport < config.minDoubleSupport) {
    doubleSupport = 0.0;
  }
  phases.doubleSupport = doubleSupport;
  phases.swing = stepTime - doubleSupport;
  phases.swingHeight = interpolate(config.speedKnots, config.swingHeight, speed);
  return phases;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ModeSequenceTemplate continuousGaitTemplate(const ContinuousGaitPhases& phases, size_t nextSwing) {
  const size_t first = (nextSwing == RF) ? RF : LF;
  const size_t other = (first == LF) ? RF : LF;
  std::vector<size_t> modes;
  std::vector<scalar_t> times{0.0};
  const auto push = [&](size_t mode, scalar_t duration) {
    modes.push_back(mode);
    times.push_back(times.back() + duration);
  };
  push(first, phases.swing);
  if (phases.doubleSupport > 0.0) push(STANCE, phases.doubleSupport);
  push(other, phases.swing);
  if (phases.doubleSupport > 0.0) push(STANCE, phases.doubleSupport);
  ModeSequenceTemplate gait(std::move(times), std::move(modes));
  gait.swingHeight = phases.swingHeight;
  return gait;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool continuousGaitSplicePoint(const ModeSchedule& schedule, scalar_t earliest, scalar_t& spliceTime, size_t& nextSwing) {
  const auto& eventTimes = schedule.eventTimes;
  const auto& modeSequence = schedule.modeSequence;
  // modeSequence[i + 1] starts at eventTimes[i]; the first single support starting after `earliest`.
  for (size_t i = 0; i < eventTimes.size() && i + 1 < modeSequence.size(); ++i) {
    if (eventTimes[i] <= earliest) continue;
    if (modeSequence[i + 1] == LF || modeSequence[i + 1] == RF) {
      spliceTime = eventTimes[i];
      nextSwing = modeSequence[i + 1];
      return true;
    }
  }
  return false;
}

}  // namespace ocs2::humanoid
