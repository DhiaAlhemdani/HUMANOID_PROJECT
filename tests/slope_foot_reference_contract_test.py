#!/usr/bin/env python3
"""Static wiring checks for the incline foot references (reference.groundHeight.followSlope).

The planning rule itself is covered by tests/slope_foot_reference_test.cpp (asserts, built with the package
tests). These pin the plumbing: the schema key (off), the G1 block (off), the builder copying it and refusing
it without the height estimate and the cap, the interface handing it to the reference manager, the reference
manager publishing the plane's gradient for getContactNormal and taking the per-foot swing update on the
flat-ground path only when a slope is read, the telemetry column, and the tracked candidate config. Run from
the tests/ directory.
"""

import unittest

import yaml

from wb_id_parameter_contract_test import NAME, ROOT, UniqueLoader

SCHEMA = yaml.load((ROOT / "src/controller/parameters.yaml").read_text(), Loader=UniqueLoader)[NAME]
PARAMS = yaml.load((ROOT / "config/g1/ros2_controllers.yaml").read_text(), Loader=UniqueLoader)[NAME]["ros__parameters"]
CANDIDATE = yaml.load((ROOT / "config/tuning/humanT25c.yaml").read_text(), Loader=UniqueLoader)[NAME]["ros__parameters"]
BUILDER = (ROOT / "src/controller/config_builder.cpp").read_text()
INTERFACE_H = (ROOT / "include/legged_robot_mpc_controller/mpc/wb_id/WbIdMpcInterface.h").read_text()
INTERFACE_CPP = (ROOT / "src/mpc/wb_id/WbIdMpcInterface.cpp").read_text()
REFMGR_H = (ROOT / "include/legged_robot_mpc_controller/mpc/common/reference_manager/SwitchedModelReferenceManager.h").read_text()
REFMGR_CPP = (ROOT / "src/mpc/common/reference_manager/SwitchedModelReferenceManager.cpp").read_text()
FOOT_COST_CPP = (ROOT / "src/mpc/common/cost/EndEffectorFootCost.cpp").read_text()
STANCE_CPP = (ROOT / "src/mpc/wb_id/constraint/WbIdFootAccelerationConstraintCppAd.cpp").read_text()
CONE_CPP = (ROOT / "src/mpc/common/constraint/FrictionForceConeConstraint.cpp").read_text()
CONTROLLER_CPP = (ROOT / "src/controller/controller.cpp").read_text()
CMAKE = (ROOT / "CMakeLists.txt").read_text()


class SlopeFootReferenceContractTest(unittest.TestCase):
    def test_schema_key_defaults_off(self):
        block = SCHEMA["ocs2"]["reference"]["groundHeight"]
        self.assertEqual(tuple(block), ("useEstimate", "maxStepPerSolve", "followSlope"))
        self.assertEqual(block["followSlope"]["type"], "bool")
        self.assertFalse(block["followSlope"]["default_value"])

    def test_g1_off_and_candidate_on_with_its_prerequisites(self):
        self.assertFalse(PARAMS["ocs2"]["reference"]["groundHeight"]["followSlope"])
        ground = CANDIDATE["ocs2"]["reference"]["groundHeight"]
        self.assertTrue(ground["followSlope"])
        self.assertTrue(ground["useEstimate"])
        self.assertTrue(CANDIDATE["ocs2"]["reference"]["terrainSpeedCap"]["enabled"])
        # humanT25c = the humanT24e / T25a recipe plus the flag.
        base = yaml.load((ROOT / "config/tuning/humanT24e.yaml").read_text(), Loader=UniqueLoader)[NAME]["ros__parameters"]
        for section in ("gait", "reference", "constraints", "costs", "model"):
            candidate = dict(CANDIDATE["ocs2"][section])
            expected = dict(base["ocs2"][section])
            if section == "reference":
                candidate["groundHeight"] = dict(candidate["groundHeight"])
                candidate["groundHeight"].pop("followSlope")
            self.assertEqual(candidate, expected, section)

    def test_builder_copies_the_flag_and_refuses_it_without_its_inputs(self):
        self.assertIn("config.followSlope = p.reference.groundHeight.followSlope;", BUILDER)
        self.assertIn("if (config.followSlope && !(config.useTerrainHeightEstimate && config.terrainSpeedCap.enabled))", BUILDER)
        # The cap settings are copied before the check reads them.
        self.assertLess(BUILDER.index("config.terrainSpeedCap.enabled = p.reference.terrainSpeedCap.enabled;"),
                        BUILDER.index("config.followSlope = "))

    def test_interface_hands_it_to_the_reference_manager(self):
        self.assertIn("bool followSlope = false;", INTERFACE_H)
        self.assertIn("referenceManagerPtr_->setSlopeFootReference(config_.followSlope);", INTERFACE_CPP)

    def test_reference_manager_publishes_the_plane_for_the_orientation_terms(self):
        # The gradient is written where the slope is read, zero when the feature is off ...
        start = REFMGR_CPP.index("void SwitchedModelReferenceManager::updateTerrainSpeedCap")
        body = REFMGR_CPP[start:REFMGR_CPP.index("void SwitchedModelReferenceManager::updateSwingOnSlope")]
        self.assertIn("const scalar_t followed = slopeFootReference_ ? slope : 0.0;", body)
        self.assertIn("slopeGradientX_.store(followed * heading(0));", body)
        self.assertIn("slopeGradientY_.store(followed * heading(1));", body)
        # ... and getContactNormal turns it into the normal every orientation / cone term already reads.
        start = REFMGR_H.index("vector3_t getContactNormal(size_t foot, scalar_t time) const {")
        normal = REFMGR_H[start:start + 700]
        self.assertIn("return vector3_t(-gx, -gy, 1.0).normalized();", normal)
        self.assertIn("return vector3_t(0.0, 0.0, 1.0);", normal)
        for consumer, text in (("foot cost", FOOT_COST_CPP), ("stance constraint", STANCE_CPP), ("friction cone", CONE_CPP)):
            self.assertIn("referenceManagerPtr_->getContactNormal(", text, consumer)

    def test_flat_ground_path_takes_the_per_foot_update_only_on_a_read_slope(self):
        start = REFMGR_CPP.index("void SwitchedModelReferenceManager::modifyReferences")
        body = REFMGR_CPP[start:start + 9000]
        self.assertIn("if (slopeFootReference_ && useTerrainHeightEstimate_ && terrainSlope_.load() != 0.0 && wellFormed) {", body)
        self.assertIn("updateSwingOnSlope(initTime, initState, modeSchedule, swingHeights);", body)
        self.assertIn("swingTrajectoryPtr_->update(modeSchedule, terrainHeight, swingHeights);", body)
        # The base height reference is still adapted first (the per-foot plan replaces only the swing update).
        self.assertLess(body.index("adaptToCurrentGroundHeight(targetTrajectories, initState, initMode)"),
                        body.index("updateSwingOnSlope("))
        start = REFMGR_CPP.index("void SwitchedModelReferenceManager::updateSwingOnSlope")
        update = REFMGR_CPP[start:start + 3000]
        self.assertIn("planSlopeFootReference(input, modeSchedule.eventTimes, contactSequence)", update)
        self.assertIn("swingTrajectoryPtr_->update(modeSchedule, plan.liftOffHeights, plan.touchDownHeights, swingHeights);", update)
        self.assertIn("input.touchDownHeightOffset = swingTrajectoryPtr_->getConfig().touchDownHeightOffset;", update)

    def test_telemetry_and_unit_test(self):
        self.assertIn('"terrain/next_touchdown"', CONTROLLER_CPP)
        self.assertIn("put(reference_manager->slopeNextTouchDownHeight());", CONTROLLER_CPP)
        self.assertIn("slope_foot_reference_test", CMAKE)
        self.assertTrue((ROOT / "tests/slope_foot_reference_test.cpp").exists())


if __name__ == "__main__":
    unittest.main()
