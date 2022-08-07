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

from pymavswarm.safety import HyperRectangle, Interval, SafetyChecker


class TestSafetyChecker(unittest.TestCase):
    """Test the SafetyChecker class."""

    def test_make_neighborhood_rectangle_min_face(self) -> None:
        """Verify that a neighborhood rectangle is made properly."""
        original = HyperRectangle(
            [
                Interval(0.1, 0.3),  # x
                Interval(0.5, 0.6),  # y
                Interval(3.4, 5.6),  # z
                Interval(1.2, 1.4),  # vx
                Interval(2.3, 2.4),  # vy
                Interval(2.5, 2.7),  # vz
            ]
        )
        bloated = HyperRectangle(
            [
                Interval(0.1, 0.3),  # x
                Interval(0.5, 0.6),  # y
                Interval(3.4, 5.6),  # z
                Interval(1.2, 1.4),  # vx
                Interval(2.3, 2.4),  # vy
                Interval(2.5, 2.7),  # vz
            ]
        )

        face = 0
        neighborhood_width = 0.1

        expected = HyperRectangle(
            [
                Interval(0.1, 0.2),  # We expect the range to be reset to (0.1, 0.2)
                Interval(0.5, 0.6),
                Interval(3.4, 5.6),
                Interval(1.2, 1.4),
                Interval(2.3, 2.4),
                Interval(2.5, 2.7),
            ]
        )

        result = SafetyChecker.make_neighborhood_rectangle(
            original, bloated, face, neighborhood_width
        )

        for dim in range(result.dimensions):
            self.assertEqual(
                result.intervals[dim].interval_min,
                expected.intervals[dim].interval_min,
            )
            self.assertEqual(
                result.intervals[dim].interval_max,
                expected.intervals[dim].interval_max,
            )

        return

    def test_make_neighborhood_rectangle_max_face(self) -> None:
        """Verify that a neighborhood rectangle is made properly."""
        original = HyperRectangle(
            [
                Interval(0.1, 0.3),  # x
                Interval(0.5, 0.6),  # y
                Interval(3.4, 5.6),  # z
                Interval(1.2, 1.4),  # vx
                Interval(2.3, 2.4),  # vy
                Interval(2.5, 2.7),  # vz
            ]
        )
        bloated = HyperRectangle(
            [
                Interval(0.1, 0.3),  # x
                Interval(0.5, 0.6),  # y
                Interval(3.4, 5.6),  # z
                Interval(1.2, 1.4),  # vx
                Interval(2.3, 2.4),  # vy
                Interval(2.5, 2.7),  # vz
            ]
        )

        face = 1
        neighborhood_width = -0.1

        expected = HyperRectangle(
            [
                Interval(0.2, 0.3),  # We expect the range to be reset to (0.2, 0.3)
                Interval(0.5, 0.6),
                Interval(3.4, 5.6),
                Interval(1.2, 1.4),
                Interval(2.3, 2.4),
                Interval(2.5, 2.7),
            ]
        )

        result = SafetyChecker.make_neighborhood_rectangle(
            original, bloated, face, neighborhood_width
        )

        # Python experiences floating point errors too
        for dim in range(result.dimensions):
            self.assertAlmostEqual(
                result.intervals[dim].interval_min, expected.intervals[dim].interval_min
            )
            self.assertAlmostEqual(
                result.intervals[dim].interval_max, expected.intervals[dim].interval_max
            )

        return

    def test_compute_derivative_bounds_position(self) -> None:
        """Verify that the position derivative bounds are properly computed."""
        rect = HyperRectangle(
            [
                Interval(0.1, 0.3),  # x
                Interval(0.5, 0.6),  # y
                Interval(3.4, 5.6),  # z
                Interval(1.2, 1.4),  # vx
                Interval(2.3, 2.4),  # vy
                Interval(2.5, 2.7),  # vz
            ]
        )
        acceleration = (0.1, 0.1, 0.2)

        position_dimensions = [0, 1, 2]

        for dim in position_dimensions:
            min_der = SafetyChecker.compute_derivative_bounds(
                rect, dim * 2, acceleration
            )
            max_der = SafetyChecker.compute_derivative_bounds(
                rect, dim * 2 + 1, acceleration
            )

            # The derivative of the position should be equal to the velocity
            self.assertEqual(rect.intervals[dim + 3].interval_min, min_der)
            self.assertEqual(rect.intervals[dim + 3].interval_max, max_der)

        return

    def test_compute_derivative_bounds_velocity(self) -> None:
        """Verify that the velocity derivative bounds are properly computed."""
        rect = HyperRectangle(
            [
                Interval(0.1, 0.3),  # x
                Interval(0.5, 0.6),  # y
                Interval(3.4, 5.6),  # z
                Interval(1.2, 1.4),  # vx
                Interval(2.3, 2.4),  # vy
                Interval(2.5, 2.7),  # vz
            ]
        )
        acceleration = (0.1, 0.1, 0.2)

        velocity_dimensions = [3, 4, 5]

        for dim in velocity_dimensions:
            min_der = SafetyChecker.compute_derivative_bounds(
                rect, dim * 2, acceleration
            )
            max_der = SafetyChecker.compute_derivative_bounds(
                rect, dim * 2 + 1, acceleration
            )

            # The derivative of the velocity should be equal to the acceleration
            self.assertEqual(acceleration[dim - 3], min_der)
            self.assertEqual(acceleration[dim - 3], max_der)

        return

    def test_single_face_lift(self) -> None:
        """Ensure that the single face lift is correctly performed."""
        rect = HyperRectangle(
            [
                Interval(2.0, 2.0),  # x
                Interval(3.0, 3.0),  # y
                Interval(5.0, 5.0),  # z
                Interval(0.0, 0.0),  # vx
                Interval(2.0, 2.0),  # vy
                Interval(0.0, 0.0),  # vz
            ]
        )
        acceleration = (0.0, 0.0, 0.0)
        step_size = 0.01
        time_remaining = 2

        lifted_rect, _ = SafetyChecker.single_face_lift(
            rect, acceleration, step_size, time_remaining
        )

        expected_rect = HyperRectangle(
            [
                Interval(2.0, 2.0),  # x
                Interval(3.02, 3.02),  # y
                Interval(5.0, 5.0),  # z
                Interval(0.0, 0.0),  # vx
                Interval(2.0, 2.0),  # vy
                Interval(0.0, 0.0),  # vz
            ]
        )

        for dim in range(lifted_rect.dimensions):
            self.assertAlmostEqual(
                lifted_rect.intervals[dim].interval_min,
                expected_rect.intervals[dim].interval_min,
            )
            self.assertAlmostEqual(
                lifted_rect.intervals[dim].interval_max,
                expected_rect.intervals[dim].interval_max,
            )

        return

    def test_face_lifting_iterative_improvement(self) -> None:
        """Ensure that the iterative face lifting is correctly performed."""
        rect = HyperRectangle(
            [
                Interval(2.0, 2.0),  # x
                Interval(3.0, 3.0),  # y
                Interval(5.0, 5.0),  # z
                Interval(0.0, 0.0),  # vx
                Interval(2.0, 2.0),  # vy
                Interval(0.0, 0.0),  # vz
            ]
        )
        acceleration = (0.0, 0.0, 0.0)

        reach_time = 2.0

        lifted_rect, _ = SafetyChecker.face_lifting_iterative_improvement(
            rect, 500, acceleration, reach_time
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

        for dim in range(lifted_rect.dimensions):
            self.assertAlmostEqual(
                lifted_rect.intervals[dim].interval_min,
                expected_rect.intervals[dim].interval_min,
            )
            self.assertAlmostEqual(
                lifted_rect.intervals[dim].interval_max,
                expected_rect.intervals[dim].interval_max,
            )

        return

    def test_check_intersection_with_collision_trajectory(self) -> None:
        """Verify collisions are detected between agents on a collision trajectory."""
        agent_one = HyperRectangle(
            [
                Interval(2.0, 2.0),  # x
                Interval(3.0, 3.0),  # y
                Interval(5.0, 5.0),  # z
                Interval(0.0, 0.0),  # vx
                Interval(2.0, 2.0),  # vy
                Interval(0.0, 0.0),  # vz
            ]
        )
        agent_two = HyperRectangle(
            [
                Interval(4.0, 4.0),  # x
                Interval(7.0, 7.0),  # y
                Interval(5.0, 5.0),  # z
                Interval(-1.0, -1.0),  # vx
                Interval(0.0, 0.0),  # vy
                Interval(0.0, 0.0),  # vz
            ]
        )

        # Positive acceleration in the y direction
        agent_one_acceleration = (0.0, 0.0, 0.0)

        # Negative acceleration in the -x direction
        agent_two_acceleration = (0.0, 0.0, 0.0)

        time_since_boot_ms = 5000
        reach_time = 2.0

        agent_one_future_state, _ = SafetyChecker.face_lifting_iterative_improvement(
            agent_one, time_since_boot_ms, agent_one_acceleration, reach_time
        )
        agent_two_future_state, _ = SafetyChecker.face_lifting_iterative_improvement(
            agent_two, time_since_boot_ms, agent_two_acceleration, reach_time
        )

        self.assertTrue(
            agent_one_future_state.intersects(
                agent_two_future_state, dimensions=[0, 1, 2]
            )
        )

        return

    def test_check_no_collision(self) -> None:
        """Verify that no collisions are detected between non-colliding agents."""
        agent_one = HyperRectangle(
            [
                Interval(1.0, 1.0),  # x
                Interval(2.0, 2.0),  # y
                Interval(3.0, 3.0),  # z
                Interval(-4.0, -4.0),  # vx
                Interval(0.0, 0.0),  # vy
                Interval(0.0, 0.0),  # vz
            ]
        )
        agent_two = HyperRectangle(
            [
                Interval(3.0, 3.0),  # x
                Interval(2.0, 2.0),  # y
                Interval(3.0, 3.0),  # z
                Interval(4.0, 4.0),  # vx
                Interval(0.0, 0.0),  # vy
                Interval(0.0, 0.0),  # vz
            ]
        )
        acceleration = (0.1, 0.1, 0.1)
        time_since_boot_ms = 5000
        reach_time = 2.0

        agent_one_future_state, _ = SafetyChecker.face_lifting_iterative_improvement(
            agent_one, time_since_boot_ms, acceleration, reach_time
        )
        agent_two_future_state, _ = SafetyChecker.face_lifting_iterative_improvement(
            agent_two, time_since_boot_ms, acceleration, reach_time
        )

        self.assertFalse(
            agent_one_future_state.intersects(
                agent_two_future_state, dimensions=[0, 1, 2]
            )
        )

        return


if __name__ == "__main__":
    unittest.main()
