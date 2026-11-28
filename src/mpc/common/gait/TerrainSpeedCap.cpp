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

#include <Eigen/Eigenvalues>

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
  if (!(s.slopeMinSeparation > 0.0) || !(s.slopeConsistency > 0.0) || !(s.slopeHeightTolerance > 0.0)) {
    throw std::invalid_argument("[TerrainSpeedCap] slopeMinSeparation, slopeConsistency and slopeHeightTolerance must be > 0");
  }
}

namespace {
constexpr scalar_t kSpanning = 0.1;  // the separations span the plane when the smaller eigenvalue of sum(run run^T) is this share of the larger
constexpr int kRoughContradictions = 2;  // contradicted planes in a row that make the ground rough (no plane) instead of yet another plane
constexpr scalar_t kRoughShare = 0.5;    // share of the height tolerance a plane out of rough ground must fit within
constexpr scalar_t kProvisionalBase = 2.0;  // x slopeMinSeparation: the separation along the heading a single sample needs to lay a provisional plane (a full stride, 0.16 m: not a closing step, not the 0.14 m strides under the 0.2 m/s cap)
}  // namespace

void TerrainSpeedCap::takeSlopeSample(scalar_t time, const SlopeSample& sample, const vector2_t& heading) {
  slopeSamples_[slopeSampleCount_ % slopeSamples_.size()] = sample;
  ++slopeSampleCount_;
  // A standing plane falls only to a sample that contradicts it in height: the sole-height noise of a short
  // double support (the stop's closing steps, 8 cm apart, +-4 mm = 0.05 of slope) stays within the tolerance
  // and keeps the plane - zeroing it there flipped the references and the contact normal to flat mid-incline
  // (run 25 C, slope_10 stops) - while the first full stride onto the plateau misses the plane by stride x
  // slope (2-5 cm) and contradicts it at once (run 25 E: a plane kept for three more double supports planned
  // the feet into the air over the crest of slope_5). What replaces it is the contradicting sample itself, a
  // provisional plane: the references follow the ground from the first stride on it, not three strides later
  // (run 25 F': both slope_5 stops fell standing on the first 0.3 m of the down-ramp with level references,
  // the plane one agreeing sample short). The incline cap waits for a confirmed plane (four agreeing
  // samples), so a tile step does not become an incline. One sample fixes one component of the gradient:
  // along the heading when the feet have run along it (a stride: its separation is stride plus stance
  // width, and the rise belongs to the stride), else along the separation itself (a turn in place).
  //
  // Two contradictions in a row are rough ground, not a second new plane (run 27): on a tile field every
  // stride contradicts the plane the previous one laid, and the references tilted 3-4 deg a new way at each
  // landing (run 26 M: `terrain/slope` 0.01-0.06 on flat tiles, a stand with the feet on two tiles read as a
  // 0.06 plane, a fall at the stop). So is one contradiction on ground that was rough a moment ago: a single
  // sample lays a provisional plane only when no raw step (a step on rough ground) was seen within
  // holdSeconds before this double support - level ground onto a ramp, a ramp onto its plateau. A real
  // change of plane (the crest: ramp -> plateau -> ramp) is one contradiction, then agreement. Rough ground
  // has no plane: the references are level and the raw step is the roughness (the run 24 E behaviour). It
  // ends when a sample agrees with the plane the sample before it laid (the ramp after the tiles, the
  // down-ramp after the plateau step: one stride later than from calm ground; a stop's closing step counts)
  // - with one contradiction already on the count, so the next one is rough ground at once - or when the
  // samples confirm a plane. Agreeing samples work the count off one at a time.
  const auto provisional = [&](const SlopeSample& s) -> vector2_t {
    const scalar_t alongHeading = s.run.dot(heading);
    if (std::abs(alongHeading) >= settings_.slopeMinSeparation) {
      return s.rise / alongHeading * heading;
    }
    return s.rise / s.run.squaredNorm() * s.run;
  };
  const bool calmBefore = !(std::isfinite(rawStepBeforeSupport_) && time - rawStepBeforeSupport_ < settings_.holdSeconds);
  // And only a stride lays one: a short base (a stop's closing step 8 cm along the heading, or the feet side by
  // side) turns a 1.5 cm tile edge into a 0.19 plane and a stand on it (run 26 M backward: fell in the stand
  // after the stop); such a sample can still contradict, agree, confirm, and pair with the next one.
  const bool strideBase = std::abs(sample.run.dot(heading)) >= kProvisionalBase * settings_.slopeMinSeparation;
  bool contradicted = false;  // the ground changed under this sample: the older samples are not part of its plane
  if (rough_) {
    if (pendingSet_ && fitsPlane(pending_, sample, kRoughShare)) {
      rough_ = false;
      pendingSet_ = false;
      contradictions_ = kRoughContradictions - 1;
      slopeConsistent_ = true;
      slopeConfirmed_ = false;
      gradient_ = pending_;
    } else {
      pending_ = provisional(sample);
      pendingSet_ = true;
    }
  } else if (!slopeConsistent_) {  // the first sample
    slopeConsistent_ = true;
    slopeConfirmed_ = false;
    contradictions_ = 0;
    gradient_ = provisional(sample);
  } else if (fitsPlane(gradient_, sample)) {
    contradictions_ = std::max(0, contradictions_ - 1);
  } else if (++contradictions_ >= kRoughContradictions || !calmBefore || !strideBase) {
    contradicted = true;
    rough_ = true;
    slopeConsistent_ = false;
    slopeConfirmed_ = false;
    gradient_.setZero();
    pending_ = provisional(sample);
    pendingSet_ = true;
  } else {
    contradicted = true;
    slopeConfirmed_ = false;
    gradient_ = provisional(sample);
  }
  if (contradicted) {
    // The samples before the change are not part of the new plane: with two parameters, four samples and a
    // centimetre of tolerance the fit absorbs the first stride off a crest (1.4 cm over 0.26 m) into the
    // lateral component and keeps a nearly level "confirmed" plane on the down-ramp (-0.02 for -0.087), at
    // this sample or at the next agreeing one. The window restarts with this sample.
    slopeSamples_.fill(SlopeSample{});
    slopeSamples_[0] = sample;
    slopeSampleCount_ = 1;
    return;
  }
  // A plane is confirmed (or refreshed) by four samples that agree with one plane within slopeConsistency x
  // their run: short bases scatter in slope and never fake one (three left one residual degree of freedom
  // when walking straight, and a tile field passed it about every other stride; four leave two and cut
  // that to a few percent on the suite's tile field). Alternating strides span the plane (the lead
  // foot is left, then right of the track) and fit the whole gradient; separations along one line (strides
  // along one axis, a turn in place over a few degrees) fit only the component along it and keep the rest of
  // the standing plane, so strides across a slope do not flatten the plane the strides up it confirmed.
  if (slopeSampleCount_ >= slopeSamples_.size()) {
    Eigen::Matrix2d normal = Eigen::Matrix2d::Zero();
    vector2_t rhs = vector2_t::Zero();
    for (const auto& s : slopeSamples_) {
      normal += s.run * s.run.transpose();
      rhs += s.rise * s.run;
    }
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigen(normal);
    const vector2_t values = eigen.eigenvalues();  // ascending
    vector2_t fitted = gradient_;
    if (values(0) >= kSpanning * values(1)) {
      fitted = normal.ldlt().solve(rhs);
    } else {
      const vector2_t axis = eigen.eigenvectors().col(1);
      fitted += (rhs.dot(axis) / values(1) - gradient_.dot(axis)) * axis;
    }
    // Out of rough ground the fit must be twice as good: a few points nearly always fit the parameters within a
    // centimetre (two tile samples on the same separation 1.5 cm apart in rise plus the rest: 7.5 mm each).
    const scalar_t share = rough_ ? kRoughShare : 1.0;
    bool agree = true;
    for (const auto& s : slopeSamples_) {
      agree = agree && std::abs(s.rise - fitted.dot(s.run)) <= share * settings_.slopeConsistency * s.run.norm();
    }
    if (agree) {
      slopeConsistent_ = true;
      slopeConfirmed_ = true;
      rough_ = false;
      pendingSet_ = false;
      contradictions_ = 0;
      gradient_ = fitted;
    }
  }
}

bool TerrainSpeedCap::fitsPlane(const vector2_t& gradient, const SlopeSample& sample, scalar_t share) const {
  // In height, not in rise / run: a short base amplifies millimetres of sole noise into a slope "disagreement"
  // while a full stride on the wrong plane misses by centimetres. `share` scales the tolerance: a plane out of
  // rough ground must fit twice as well as a standing plane needs to be kept.
  const scalar_t tolerance = share * std::max(settings_.slopeConsistency * sample.run.norm(), settings_.slopeHeightTolerance);
  return std::abs(sample.rise - gradient.dot(sample.run)) <= tolerance;
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
  lastRawStepTime_ = std::numeric_limits<scalar_t>::quiet_NaN();
  rawStepBeforeSupport_ = std::numeric_limits<scalar_t>::quiet_NaN();
  capValue_ = std::numeric_limits<scalar_t>::infinity();
  tierTime_.assign(settings_.stepKnots.size(), std::numeric_limits<scalar_t>::quiet_NaN());
  previousTime_ = std::numeric_limits<scalar_t>::quiet_NaN();
  stepCount_ = 0;
  slopeSamples_.fill(SlopeSample{});
  slopeSampleCount_ = 0;
  lastDsDz_ = std::numeric_limits<scalar_t>::quiet_NaN();
  lastDsRun_.setZero();
  sampledThisSupport_ = false;
  gradient_.setZero();
  slopeConsistent_ = false;
  slopeConfirmed_ = false;
  rough_ = false;
  pendingSet_ = false;
  pending_.setZero();
  contradictions_ = 0;
  slopeTime_ = std::numeric_limits<scalar_t>::quiet_NaN();
}

scalar_t TerrainSpeedCap::update(scalar_t time, const contact_flag_t& contacts, const feet_array_t<scalar_t>& soleHeights,
                                 const feet_array_t<vector2_t>& solePositions, const vector2_t& heading) {
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
  // One slope sample per double support, read from the first *settled* solve in it (the feet's height
  // difference unchanged, within kSettled, from the previous solve), else from its last. At the first solve
  // the landing foot can still be in the air (the schedule says double support from the planned touchdown;
  // a foot planned above the ground - a plane kept over a crest - gets there late), and a sample read there
  // confirms the plane that put the foot in the air: on the slope_5 plateau the plane decayed over six
  // landings instead of dropping at the first flat stride (run 25 F, tdref +1.2..+2.7 cm). A foot that landed
  // early is down and loaded at the first solve, and that is the cleanest reading: by the end of the double
  // support the trailing foot is unloading: on 10 deg at 0.10 m strides the end-of-support readings built no
  // plane for 15 s (run 25 F', slope_10: speeds formed it only at the last 0.2 rung, backward never).
  if (!doubleSupport && inDoubleSupport_ && !sampledThisSupport_ && std::isfinite(lastDsDz_) && lastDsRun_.allFinite() &&
      lastDsRun_.norm() >= settings_.slopeMinSeparation) {
    takeSlopeSample(time, SlopeSample{lastDsDz_, lastDsRun_}, heading);
  }
  if (!doubleSupport) {
    sampledThisSupport_ = false;
    lastDsDz_ = std::numeric_limits<scalar_t>::quiet_NaN();
    lastDsRun_.setZero();
  }
  if (doubleSupport && !inDoubleSupport_) {
    rawStepBeforeSupport_ = lastRawStepTime_;  // what the slope samples of this double support see as "before"
  }
  if (doubleSupport) {
    const scalar_t dz = soleHeights[0] - soleHeights[1];
    const vector2_t run = solePositions[0] - solePositions[1];
    constexpr scalar_t kSettled = 0.0005;  // [m] per solve: a foot descending at the contactSync landing speed (0.05 m/s, ~1.2 mm at 42 Hz) moves more
    if (!sampledThisSupport_ && std::isfinite(dz) && std::isfinite(lastDsDz_) && std::abs(dz - lastDsDz_) <= kSettled) {
      sampledThisSupport_ = true;
      if (run.allFinite() && run.norm() >= settings_.slopeMinSeparation) {
        takeSlopeSample(time, SlopeSample{dz, run}, heading);
      }
    }
    lastDsDz_ = dz;
    lastDsRun_ = run;
    if (slopeConfirmed_) {
      slopeTime_ = time;
    }
    // Roughness: the step out of the plane the feet agree on; the raw step while no plane stands. Against a
    // provisional plane (one sample) a step counts only if neither level ground nor the plane explains it -
    // a tile step must not become a plane that doubles the next tile step's residual.
    const scalar_t fromPlane = run.allFinite() ? std::abs(dz - gradient_.dot(run)) : std::abs(dz);
    const scalar_t step = !slopeConsistent_ ? std::abs(dz) : slopeConfirmed_ ? fromPlane : std::min(fromPlane, std::abs(dz));
    if (std::isfinite(step) && step >= settings_.stepKnots.front()) {
      if (!inDoubleSupport_ || !countedThisSupport_) {
        ++stepCount_;
        countedThisSupport_ = true;
        lastStep_ = step;
      } else {
        lastStep_ = std::max(lastStep_, step);
      }
      lastStepTime_ = time;
      if (rough_) {
        lastRawStepTime_ = time;  // a raw step on rough ground
      }
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
      if (gradient_.norm() >= settings_.slopeKnots[i]) {
        capValue_ = std::min(capValue_, settings_.slopeSpeedKnots[i]);
      }
    }
  }
  active_ = std::isfinite(capValue_);
  return capValue_;
}

}  // namespace ocs2::humanoid
