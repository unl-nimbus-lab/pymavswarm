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

from pymavswarm import Agent
from pymavswarm.safety import HyperRectangle, Interval


class TestAgent(unittest.TestCase):
    """Test the Agent class."""

    def test_compute_reachable_set(self) -> None:
        """Test computing an agents reachable set."""
        agent = Agent(1, 1)

        agent.position.x = 2.0
        agent.position.y = 3.0
        agent.position.z = 5.0

        agent.velocity.x = 0.0
        agent.velocity.y = 2.0
        agent.velocity.z = 0.0

        agent.acceleration.x = 0.0
        agent.acceleration.y = 0.0
        agent.acceleration.z = 0.0

        reach_time = 2.0

        result_rect, _ = agent.compute_reachable_set(
            0, 0, reach_time, uses_lat_lon=False
        )

        expected_rect = HyperRectangle(
            [
                Interval(2.0, 2.0),  # x
                Interval(3.0, 7.0),  # y
                Interval(5.0, 5.0),  # z
                Interval(0.0, 0.0),  # vx
                Interval(2.0, 2.0),  # vy
                Interval(0.0, 0.0),  # vz
            ]
        )

        for dim in range(result_rect.dimensions):
            self.assertAlmostEqual(
                result_rect.intervals[dim].interval_min,
                expected_rect.intervals[dim].interval_min,
            )
            self.assertAlmostEqual(
                result_rect.intervals[dim].interval_max,
                expected_rect.intervals[dim].interval_max,
            )

        return
