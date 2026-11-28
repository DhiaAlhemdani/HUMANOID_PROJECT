#!/usr/bin/env python3
"""Static wiring checks for the blind terrain speed cap (reference.terrainSpeedCap).

The rule itself is covered by tests/terrain_speed_cap_test.cpp (asserts, built with the package tests).
These pin the parameter plumbing: schema defaults (off), the G1 block, the builder, the interface handing
the settings to the reference manager, the reference manager updating it on the flat-ground path, and the
motion manager applying the cap to the velocity command. Run from the tests/ directory.
"""

import unittest

import yaml

from wb_id_parameter_contract_test import NAME, ROOT, UniqueLoader

SCHEMA = yaml.load((ROOT / "src/controller/parameters.yaml").read_text(), Loader=UniqueLoader)[NAME]
PARAMS = yaml.load((ROOT / "config/g1/ros2_controllers.yaml").read_text(), Loader=UniqueLoader)[NAME]["ros__parameters"]
BUILDER = (ROOT / "src/controller/config_builder.cpp").read_text()
INTERFACE_H = (ROOT / "include/legged_robot_mpc_controller/mpc/wb_id/WbIdMpcInterface.h").read_text()
INTERFACE_CPP = (ROOT / "src/mpc/wb_id/WbIdMpcInterface.cpp").read_text()
REFMGR_CPP = (ROOT / "src/mpc/common/reference_manager/SwitchedModelReferenceManager.cpp").read_text()
MOTION_CPP = (ROOT / "src/mpc/common/reference_manager/ProceduralMpcMotionManager.cpp").read_text()
TARGET_CPP = (ROOT / "src/mpc/common/target/WalkingVelocityTarget.cpp").read_text()
CAP_H = (ROOT / "include/legged_robot_mpc_controller/mpc/common/gait/TerrainSpeedCap.h").read_text()
CAP_CPP = (ROOT / "src/mpc/common/gait/TerrainSpeedCap.cpp").read_text()
CONTROLLER_CPP = (ROOT / "src/controller/controller.cpp").read_text()
CMAKE = (ROOT / "CMakeLists.txt").read_text()

KEYS = ("enabled", "stepKnots", "speedKnots", "holdSeconds", "slopeKnots", "slopeSpeedKnots", "slopeMinSeparation", "slopeConsistency",
        "slopeHeightTolerance")


class TerrainSpeedCapContractTest(unittest.TestCase):
    def test_schema_defaults_off_and_match_the_struct(self):
        schema = SCHEMA["ocs2"]["reference"]["terrainSpeedCap"]
        self.assertEqual(tuple(schema), KEYS)
        self.assertFalse(schema["enabled"]["default_value"])
        for key, value in (("stepKnots", [0.01, 0.02, 0.035]), ("speedKnots", [0.5, 0.2, 0.1]), ("holdSeconds", 3.0),
                           ("slopeKnots", [0.05, 0.12]), ("slopeSpeedKnots", [0.3, 0.2]), ("slopeMinSeparation", 0.08),
                           ("slopeConsistency", 0.03), ("slopeHeightTolerance", 0.01)):
            self.assertEqual(schema[key]["default_value"], value, key)
            if isinstance(value, list):
                self.assertIn(f"std::vector<scalar_t> {key}{{{', '.join(str(v) for v in value)}}};", CAP_H, key)
            else:
                self.assertIn(f"scalar_t {key} = {value};", CAP_H, key)

    def test_g1_block_is_present_and_off(self):
        block = PARAMS["ocs2"]["reference"]["terrainSpeedCap"]
        self.assertEqual(tuple(block), KEYS)
        self.assertFalse(block["enabled"])
        for key in KEYS[1:]:
            self.assertEqual(block[key], SCHEMA["ocs2"]["reference"]["terrainSpeedCap"][key]["default_value"], key)

    def test_builder_copies_every_key_and_validates(self):
        for key in KEYS:
            self.assertIn(f"config.terrainSpeedCap.{key} = p.reference.terrainSpeedCap.{key};", BUILDER, key)
        self.assertIn("validateTerrainSpeedCapSettings(config.terrainSpeedCap)", BUILDER)

    def test_interface_hands_the_settings_to_the_reference_manager(self):
        self.assertIn("TerrainSpeedCapSettings terrainSpeedCap;", INTERFACE_H)
        self.assertIn("referenceManagerPtr_->setTerrainSpeedCapSettings(config_.terrainSpeedCap);", INTERFACE_CPP)

    def test_reference_manager_updates_it_on_the_flat_ground_path_after_contact_sync(self):
        start = REFMGR_CPP.index("void SwitchedModelReferenceManager::modifyReferences")
        body = REFMGR_CPP[start:start + 2500]
        sync = body.index("applyContactSync(initTime, initState, modeSchedule)")
        cap = body.index("updateTerrainSpeedCap(initTime, initState, modeSchedule)")
        self.assertLess(sync, cap, "the cap reads the schedule after the touchdown sync")
        self.assertIn("if (!stairPlan && !isTerrainWalkActive()) {\n    updateTerrainSpeedCap", body)

    def test_motion_manager_applies_the_cap_before_conditioning_the_command(self):
        start = MOTION_CPP.index("void ProceduralMpcMotionManager::preSolverRun")
        body = MOTION_CPP[start:]
        set_cap = body.index("walkingVelocityTarget_.setSpeedCap(switchedModelReferenceManagerPtr_->terrainSpeedCapValue())")
        evaluate = body.index("walkingVelocityTarget_.evaluate(initTime, finalTime, initState)")
        self.assertLess(set_cap, evaluate)
        # Applied to the scaled command, i.e. before the filter and slew limiter ramp it.
        scale = TARGET_CPP.index("WalkingVelocityCommand WalkingVelocityTarget::scaleCommand")
        self.assertIn("speedCap_.load()", TARGET_CPP[scale:TARGET_CPP.index("WalkingVelocityTarget::setMinimumWalkSpeed")])

    def test_plane_falls_only_to_a_contradiction_in_height(self):
        # Run 25 C: zeroing the plane whenever the latest three samples disagreed put the slope_10 stops into
        # the flat branch (level foot references, level contact normal) and all three fell. Run 25 E: keeping
        # it until three new samples agreed planned the feet into the air over the crest of slope_5 (the first
        # full stride on the plateau misses the plane by stride x slope). A standing plane must fall only to a
        # sample that contradicts it in height, with a floor for the sole noise of a short base.
        sample = CAP_CPP[CAP_CPP.index("TerrainSpeedCap::takeSlopeSample("):CAP_CPP.index("TerrainSpeedCap::fitsPlane(")]
        self.assertIn("if (!slopeConsistent_ || !fitsPlane(gradient_, sample)) {", sample)
        self.assertIn("if (agree) {\n      slopeConsistent_ = true;\n      slopeConfirmed_ = true;\n      gradient_ = fitted;", sample)
        self.assertIn("std::abs(s.rise - fitted.dot(s.run)) <= settings_.slopeConsistency * s.run.norm()", sample)
        self.assertIn("std::max(settings_.slopeConsistency * sample.run.norm(), settings_.slopeHeightTolerance)", CAP_CPP)
        self.assertIn("The plane stands through noise and falls to a contradiction", (ROOT / "tests/terrain_speed_cap_test.cpp").read_text())

    def test_contradiction_leaves_a_provisional_plane(self):
        # Run 25 F': with the plane dropped to nothing at the crest and three agreeing samples needed to build
        # the next one, both slope_5 stops landed on the first 0.3 m of the down-ramp with level references and
        # a level contact normal and fell standing there. The contradicting sample is itself the new
        # (provisional) plane: the references follow it from the first stride; the incline cap waits for a
        # confirmed plane (three agreeing samples), so a tile step is not an incline.
        sample = CAP_CPP[CAP_CPP.index("TerrainSpeedCap::takeSlopeSample("):CAP_CPP.index("TerrainSpeedCap::fitsPlane(")]
        update = CAP_CPP[CAP_CPP.index("TerrainSpeedCap::update("):]
        self.assertIn("if (!slopeConsistent_ || !fitsPlane(gradient_, sample)) {\n    slopeConsistent_ = true;\n    slopeConfirmed_ = false;", sample)
        self.assertIn("gradient_ = sample.rise / alongHeading * heading;", sample)
        self.assertIn("if (slopeConfirmed_) {\n      slopeTime_ = time;", update)
        self.assertIn("slopeConfirmed_ ? fromPlane : std::min(fromPlane, std::abs(dz))", update)
        self.assertIn("terrainSpeedCap_.slopeConsistent() ? terrainSpeedCap_.gradient() : vector2_t::Zero()", REFMGR_CPP)
        self.assertIn("Provisional planes and roughness", (ROOT / "tests/terrain_speed_cap_test.cpp").read_text())

    def test_sample_is_read_when_the_double_support_settles(self):
        # Run 25 F: at the first solve of the planned double support the landing foot can still be in the air
        # (planned above the ground by a kept plane), and a sample read there confirms the plane that put it
        # there - the plane decayed over six landings on the slope_5 plateau instead of dropping at the first
        # flat stride. Run 25 F': read at the end of the double support instead, the trailing foot is already
        # unloading and on 10 deg at 0.10 m strides no plane formed for 15 s. The sample is the first settled
        # reading in the double support (unchanged from the previous solve), else the last one as it ends.
        update = CAP_CPP[CAP_CPP.index("TerrainSpeedCap::update("):]
        self.assertIn("if (!doubleSupport && inDoubleSupport_ && !sampledThisSupport_ && std::isfinite(lastDsDz_) && lastDsRun_.allFinite() &&\n"
                      "      lastDsRun_.norm() >= settings_.slopeMinSeparation) {\n    takeSlopeSample(SlopeSample{lastDsDz_, lastDsRun_}, heading);", update)
        self.assertIn("std::abs(dz - lastDsDz_) <= kSettled) {\n      sampledThisSupport_ = true;", update)
        self.assertIn("lastDsDz_ = dz;\n    lastDsRun_ = run;", update)
        self.assertNotIn("slopeSamples_[slopeSampleCount_ % slopeSamples_.size()] =", update)
        self.assertIn("The sample is read when the double support ends, not when it begins",
                      (ROOT / "tests/terrain_speed_cap_test.cpp").read_text())

    def test_plane_is_a_world_frame_gradient(self):
        # Run 26: samples are rise over the planar separation vector; three spanning samples fit the whole
        # gradient, collinear ones only the component along their line; the incline cap reads its magnitude.
        self.assertIn("const feet_array_t<vector2_t>& solePositions, const vector2_t& heading);", CAP_H)
        self.assertIn("const vector2_t& gradient() const { return gradient_; }", CAP_H)
        self.assertIn("if (values(0) >= kSpanning * values(1)) {\n      fitted = normal.ldlt().solve(rhs);", CAP_CPP)
        self.assertIn("fitted += (rhs.dot(axis) / values(1) - gradient_.dot(axis)) * axis;", CAP_CPP)
        self.assertIn("if (gradient_.norm() >= settings_.slopeKnots[i]) {", CAP_CPP)
        self.assertIn("World-frame plane (run 26)", (ROOT / "tests/terrain_speed_cap_test.cpp").read_text())

    def test_telemetry_and_unit_test(self):
        for name in ("terrain/cap", "terrain/steps", "terrain/last_step", "terrain/slope"):
            self.assertIn(f'"{name}"', CONTROLLER_CPP, name)
        self.assertIn("terrain_speed_cap_test", CMAKE)
        self.assertTrue((ROOT / "tests/terrain_speed_cap_test.cpp").exists())


if __name__ == "__main__":
    unittest.main()
