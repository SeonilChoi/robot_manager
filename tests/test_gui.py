"""GUI test: config used by GUI exists and RobotManager initializes (no window)."""
import unittest
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parent.parent
CONFIG_PATH = PROJECT_ROOT / "config" / "robot_config.yaml"


class TestGui(unittest.TestCase):
    def test_config_exists(self):
        self.assertTrue(CONFIG_PATH.exists(), f"Config not found: {CONFIG_PATH}")

    def test_robot_manager_initializes_with_config(self):
        from robot_manager import RobotManager
        manager = RobotManager(str(CONFIG_PATH))
        manager.initialize()
        self.assertIsNotNone(manager._robot)


if __name__ == "__main__":
    unittest.main()
