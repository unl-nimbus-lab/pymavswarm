import sys

sys.path.append("..")

import unittest
from Mission import Mission
from Waypoint import Waypoint


class TestMission(unittest.TestCase):
    def test_add_waypoint(self) -> None:
        """
        Test adding a waypoint to the mission
        """
        # Create a new mission
        ms = Mission()

        # Create a new waypoint
        wp = Waypoint(0.0, 0.0, 0.0)

        # Add the waypoint
        ms.add_waypoint(wp)

        # Test to make sure that the waypoint was correctly added
        self.assertIn(wp, ms.waypoints)

        # Clear waypoints
        ms.clear_waypoints()

        return

    def test_remove_waypoint(self) -> None:
        """
        Test removing a waypoint from the mission
        """
        # Create a new mission
        ms = Mission()

        # Create a new waypoint
        wp = Waypoint(0.0, 0.0, 0.0)

        # Add the waypoint
        ms.add_waypoint(wp)

        # Remove the waypoint
        ms.remove_waypoint(wp)

        # Test to make sure that the waypoint was correctly removed
        self.assertNotIn(wp, ms.waypoints)

        # Clear waypoints
        ms.clear_waypoints()

        return

    def test_remove_waypoint_by_value(self) -> None:
        """
        Test removing a waypoint from the mission
        """
        # Create a new mission
        ms = Mission()

        # Create a new waypoint
        wp = Waypoint(0.0, 0.0, 0.0)

        # Add the waypoint
        ms.add_waypoint(wp)

        # Remove the waypoint
        ms.remove_waypoint_by_value(0.0, 0.0, 0.0)

        # Test to make sure that the waypoint was correctly removed
        self.assertNotIn(wp, ms.waypoints)

        # Clear waypoints
        ms.clear_waypoints()

        return

    
    def test_remove_waypoint_by_index(self) -> None:
        """
        Test removing a waypoint from the mission
        """
        # Create a new mission
        ms = Mission()

        # Create two new waypoints
        wp = Waypoint(0.0, 0.0, 0.0)
        wp_2 = Waypoint(0.0, 0.0, 0.0)

        # Add the waypoints
        ms.add_waypoint(wp)
        ms.add_waypoint(wp_2)

        self.assertEqual(ms.waypoints[0], wp)

        # Remove the waypoint
        ms.remove_waypoint_by_index(0)

        # Test to make sure that the waypoint was correctly removed
        self.assertNotIn(wp, ms.waypoints)
        self.assertEqual(ms.waypoints[0], wp_2)

        # Clear waypoints
        ms.clear_waypoints()

        return


if __name__ == "__main__":
    unittest.main()
