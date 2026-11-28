#!/usr/bin/env python3
"""Static wiring checks for the contact-synchronised touchdown (gait.contactSync).

The rule itself is covered by tests/contact_sync_test.cpp (asserts, built with the package tests).
These pin the parameter plumbing: schema defaults (off), the G1 block, the builder, the interface
handing the settings to the reference manager, and where the reference manager applies them.
Run from the tests/ directory (imports wb_id_parameter_contract_test).
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
SYNC_H = (ROOT / "include/legged_robot_mpc_controller/mpc/common/gait/ContactSync.h").read_text()
CMAKE = (ROOT / "CMakeLists.txt").read_text()

KEYS = ("enabled", "earlyWindow", "lateWindow", "heightThreshold", "holdStep", "minPhaseDuration", "descentVelocity",
        "blockedHeight")


class ContactSyncContractTest(unittest.TestCase):
    def test_schema_defaults_off_and_match_the_struct(self):
        schema = SCHEMA["ocs2"]["gait"]["contactSync"]
        self.assertEqual(tuple(schema), KEYS)
        self.assertFalse(schema["enabled"]["default_value"])
        # The schema defaults are the C++ struct defaults, so a config that only sets `enabled` gets the documented rule.
        for key, value in (("earlyWindow", 0.06), ("lateWindow", 0.0), ("heightThreshold", 0.002), ("holdStep", 0.02),
                           ("minPhaseDuration", 0.02), ("descentVelocity", 0.0), ("blockedHeight", 0.0)):
            self.assertEqual(schema[key]["default_value"], value, key)
            self.assertIn(f"scalar_t {key} = {value};", SYNC_H.replace("0.060", "0.06"), key)

    def test_g1_ships_the_early_half_with_the_terrain_rule(self):
        # Run 18 (docs/results/human_gait_template.md): the early half is a suite-wide win; the late half
        # (holding the event while the foot is still up) cost balance at 0.8 m/s and stays off. Run 27
        # shipped the terrain recipe (run 23 C): the velocity-aware "landed" rule with a 1 cm threshold.
        block = PARAMS["ocs2"]["gait"]["contactSync"]
        self.assertEqual(tuple(block), KEYS)
        self.assertTrue(block["enabled"])
        self.assertEqual(block["lateWindow"], 0.0)
        shipped = {"heightThreshold": 0.01, "descentVelocity": 0.05, "blockedHeight": 0.01}
        for key in KEYS[1:]:
            self.assertEqual(block[key], shipped.get(key, SCHEMA["ocs2"]["gait"]["contactSync"][key]["default_value"]), key)

    def test_builder_copies_every_key_and_validates(self):
        for key in KEYS:
            self.assertIn(f"config.contactSync.{key} = p.gait.contactSync.{key};", BUILDER, key)
        self.assertIn("validateContactSyncSettings(config.contactSync)", BUILDER)

    def test_interface_hands_the_settings_to_the_reference_manager(self):
        self.assertIn("ContactSyncSettings contactSync;", INTERFACE_H)
        self.assertIn("referenceManagerPtr_->setContactSyncSettings(config_.contactSync);", INTERFACE_CPP)

    def test_reference_manager_applies_it_to_the_solve_schedule_on_flat_ground_only(self):
        start = REFMGR_CPP.index("void SwitchedModelReferenceManager::modifyReferences")
        body = REFMGR_CPP[start:start + 2000]
        fetch = body.index("gaitSchedulePtr_->getModeSchedule(")
        apply = body.index("applyContactSync(initTime, initState, modeSchedule)")
        self.assertLess(fetch, apply, "the sync must see the schedule the solve is built on")
        self.assertIn("contactSync_.enabled && !stairPlan && !isTerrainWalkActive()", body)
        # The moved event is written back so the next solve starts from it.
        self.assertIn("gaitSchedulePtr_->updateEventTimes(modeSchedule.eventTimes);", REFMGR_CPP)

    def test_rule_is_unit_tested(self):
        self.assertIn("contact_sync_test", CMAKE)
        self.assertTrue((ROOT / "tests/contact_sync_test.cpp").exists())


if __name__ == "__main__":
    unittest.main()
