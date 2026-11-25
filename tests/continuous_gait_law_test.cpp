// Regression test of the speed-continuous gait law (ocs2.gait.continuous): interpolation, template
// construction and the phase-continuous splice into a GaitSchedule. Built with -UNDEBUG (assert active).
#include <cassert>
#include <cmath>
#include <stdexcept>

#include <legged_robot_mpc_controller/mpc/common/gait/ContinuousGaitLaw.h>
#include <legged_robot_mpc_controller/mpc/common/gait/GaitSchedule.h>

int main() {
  using namespace ocs2;
  using namespace ocs2::humanoid;
  ContinuousGaitConfig cfg;
  cfg.enabled = true;
  cfg.speedKnots = {0.0, 0.45, 0.8, 1.15};
  cfg.stepTime = {0.72, 0.72, 0.55, 0.48};
  cfg.doubleSupport = {0.25, 0.25, 0.12, 0.09};
  cfg.swingHeight = {0.05, 0.05, 0.05, 0.06};
  cfg.minWalkSpeed = 0.4;
  cfg.standBelow = 0.15;
  validateContinuousGaitConfig(cfg);

  const auto near = [](double a, double b) { return std::abs(a - b) < 1e-9; };
  // Clamped below the first knot, interpolated between knots, clamped above the last.
  auto p = evaluateContinuousGait(cfg, 0.2);
  assert(near(p.stepTime(), 0.72) && near(p.doubleSupport, 0.25) && near(p.swing, 0.47));
  p = evaluateContinuousGait(cfg, 0.625);  // halfway 0.45 -> 0.8
  assert(near(p.stepTime(), 0.635) && near(p.doubleSupport, 0.185));
  p = evaluateContinuousGait(cfg, 2.0);
  assert(near(p.stepTime(), 0.48) && near(p.doubleSupport, 0.09) && near(p.swingHeight, 0.06));
  // The yaw lever turns a turn-in-place command into gait speed.
  assert(near(continuousGaitSpeed(cfg, 0.0, 0.0, 1.0), 0.15) && near(continuousGaitSpeed(cfg, 0.3, 0.4, 0.0), 0.5));
  // Short double supports are dropped: alternating single supports of the full step time.
  ContinuousGaitConfig trot = cfg;
  trot.doubleSupport = {0.02, 0.02, 0.0, 0.0};
  p = evaluateContinuousGait(trot, 0.5);
  assert(near(p.doubleSupport, 0.0) && near(p.swing, p.stepTime()));
  auto tpl = continuousGaitTemplate(p, RF);
  assert(tpl.modeSequence.size() == 2 && tpl.modeSequence[0] == RF && tpl.modeSequence[1] == LF);
  assert(near(tpl.switchingTimes.back(), 2 * p.stepTime()));
  // With double support: swing, DS, swing, DS; period 2 x stepTime.
  p = evaluateContinuousGait(cfg, 0.45);
  tpl = continuousGaitTemplate(p, LF);
  assert(tpl.modeSequence.size() == 4 && tpl.modeSequence[0] == LF && tpl.modeSequence[1] == STANCE &&
         tpl.modeSequence[2] == RF && tpl.modeSequence[3] == STANCE);
  assert(near(tpl.switchingTimes[1], 0.47) && near(tpl.switchingTimes[2], 0.72) && near(tpl.switchingTimes[4], 1.44));
  assert(near(tpl.swingHeight, 0.05));

  // Validation rejects inconsistent tables.
  ContinuousGaitConfig bad = cfg;
  bad.stepTime.pop_back();
  bool threw = false;
  try { validateContinuousGaitConfig(bad); } catch (const std::invalid_argument&) { threw = true; }
  assert(threw);

  // Splice into a trot schedule: LF 1.0-1.5, RF 1.5-2.0, LF 2.0-2.5 ... At t = 1.2 with a 0.1 s delay the
  // next single support starts at 1.5 (RF); the LF in progress is untouched.
  ModeSequenceTemplate stance({0.0, 0.5}, {STANCE});
  ModeSequenceTemplate trotTpl({0.0, 0.5, 1.0}, {LF, RF});
  GaitSchedule schedule(ModeSchedule({0.0, 0.5}, {STANCE, STANCE, STANCE}), stance, 0.0);
  schedule.insertModeSequenceTemplate(trotTpl, 1.0, 3.0);
  auto ms = schedule.getModeSchedule(1.2, 3.0);
  scalar_t spliceTime = 0.0;
  size_t nextSwing = STANCE;
  assert(continuousGaitSplicePoint(ms, 1.3, spliceTime, nextSwing));
  assert(near(spliceTime, 1.5) && nextSwing == RF);
  schedule.insertModeSequenceTemplate(continuousGaitTemplate(evaluateContinuousGait(cfg, 0.45), nextSwing), spliceTime, 4.0);
  ms = schedule.getModeSchedule(1.2, 4.0);
  // Before the splice the old plan; from 1.5 on: RF 0.47, STANCE 0.25, LF 0.47, STANCE 0.25 ...
  assert(ms.modeAtTime(1.3) == LF);
  assert(ms.modeAtTime(1.6) == RF && ms.modeAtTime(1.5 + 0.47 + 0.1) == STANCE && ms.modeAtTime(1.5 + 0.72 + 0.1) == LF);
  const auto& heights = schedule.getSwingHeightSequence();
  assert(heights.size() == ms.modeSequence.size());
  // Nothing single-support after a stance-only tail: no splice point.
  ModeSchedule onlyStance({5.0, 6.0}, {STANCE, STANCE, STANCE});
  assert(!continuousGaitSplicePoint(onlyStance, 4.0, spliceTime, nextSwing));
  return 0;
}
