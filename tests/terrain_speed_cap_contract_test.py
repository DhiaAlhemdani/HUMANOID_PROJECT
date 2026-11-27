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
CONTROLLER_CPP = (ROOT / "src/controller/controller.cpp").read_text()
CMAKE = (ROOT / "CMakeLists.txt").read_text()

KEYS = ("enabled", "stepKnots", "speedKnots", "holdSeconds")


class TerrainSpeedCapContractTest(unittest.TestCase):
    def test_schema_defaults_off_and_match_the_struct(self):
        schema = SCHEMA["ocs2"]["reference"]["terrainSpeedCap"]
        self.assertEqual(tuple(schema), KEYS)
        self.assertFalse(schema["enabled"]["default_value"])
        for key, value in (("stepKnots", [0.01, 0.02, 0.035]), ("speedKnots", [0.5, 0.2, 0.1]), ("holdSeconds", 3.0)):
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

    def test_telemetry_and_unit_test(self):
        for name in ("terrain/cap", "terrain/steps", "terrain/last_step"):
            self.assertIn(f'"{name}"', CONTROLLER_CPP, name)
        self.assertIn("terrain_speed_cap_test", CMAKE)
        self.assertTrue((ROOT / "tests/terrain_speed_cap_test.cpp").exists())


if __name__ == "__main__":
    unittest.main()
