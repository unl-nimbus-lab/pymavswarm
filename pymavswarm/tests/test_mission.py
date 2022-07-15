# pymavswarm is an interface for swarm control and interaction
# Copyright (C) 2022  Evan Palmer

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

import unittest

from pymavswarm.mission import SwarmMission, Waypoint


class TestMission(unittest.TestCase):
    """Test the Mission class."""

    def test_add_waypoint(self) -> None:
        """Test adding a waypoint to the mission."""
        # Create a new mission
        mission = SwarmMission()

        # Create a new waypoint
        waypoint = Waypoint(0.0, 0.0, 0.0)

        # Add the waypoint
        mission.add_waypoint(waypoint)

        # Test to make sure that the waypoint was correctly added
        self.assertIn(waypoint, mission.waypoints)

        # Clear waypoints
        mission.clear_waypoints()

        return

    def test_remove_waypoint(self) -> None:
        """Test removing a waypoint from the mission."""
        # Create a new mission
        mission = SwarmMission()

        # Create a new waypoint
        waypoint = Waypoint(0.0, 0.0, 0.0)

        # Add the waypoint
        mission.add_waypoint(waypoint)

        # Remove the waypoint
        mission.remove_waypoint(waypoint)

        # Test to make sure that the waypoint was correctly removed
        self.assertNotIn(waypoint, mission.waypoints)

        # Clear waypoints
        mission.clear_waypoints()

        return

    def test_remove_waypoint_by_value(self) -> None:
        """Test removing a waypoint from the mission."""
        # Create a new mission
        mission = SwarmMission()

        # Create a new waypoint
        waypoint = Waypoint(0.0, 0.0, 0.0)

        # Add the waypoint
        mission.add_waypoint(waypoint)

        # Remove the waypoint
        mission.remove_waypoint_by_value(0.0, 0.0, 0.0)

        # Test to make sure that the waypoint was correctly removed
        self.assertNotIn(waypoint, mission.waypoints)

        # Clear waypoints
        mission.clear_waypoints()

        return

    def test_remove_waypoint_by_index(self) -> None:
        """Test removing a waypoint from the mission by index."""
        # Create a new mission
        mission = SwarmMission()

        # Create two new waypoints
        waypoint_1 = Waypoint(0.0, 0.0, 0.0)
        waypoint_2 = Waypoint(0.0, 0.0, 0.0)

        # Add the waypoints
        mission.add_waypoint(waypoint_1)
        mission.add_waypoint(waypoint_2)

        self.assertEqual(mission.waypoints[0], waypoint_1)

        # Remove the waypoint
        mission.remove_waypoint_by_index(0)

        # Test to make sure that the waypoint was correctly removed
        self.assertNotIn(waypoint_1, mission.waypoints)
        self.assertEqual(mission.waypoints[0], waypoint_2)

        # Clear waypoints
        mission.clear_waypoints()

        return


if __name__ == "__main__":
    unittest.main()
