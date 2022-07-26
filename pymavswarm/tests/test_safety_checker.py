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

        # Sometimes Python experiences floating point errors
        for dim in range(result.dimensions):
            self.assertAlmostEqual(
                result.intervals[dim].interval_min,
                expected.intervals[dim].interval_min,
                delta=0.000000001,
            )
            self.assertAlmostEqual(
                result.intervals[dim].interval_max,
                expected.intervals[dim].interval_max,
                delta=0.000000001,
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
                Interval(0.1, 0.3),  # x
                Interval(0.5, 0.6),  # y
                Interval(3.4, 5.6),  # z
                Interval(1.2, 1.4),  # vx
                Interval(2.3, 2.4),  # vy
                Interval(2.5, 2.7),  # vz
            ]
        )
        acceleration = (0.3, 0.4, 0.5)
        step_size = 0.01
        time_remaining = 2

        new_rect, time_to_elapse = SafetyChecker.single_face_lift(
            rect, acceleration, step_size, time_remaining
        )

        return

    def test_face_lifting_iterative_improvement(self) -> None:
        """Verify that the face lifting algorithm is correctly implemented."""
        pass

    def test_check_collision(self) -> None:
        """Verify that collisions are detected between collided agents."""
        agent_one = HyperRectangle(
            [
                Interval(0.1, 0.3),  # x
                Interval(0.5, 0.6),  # y
                Interval(3.4, 5.6),  # z
                Interval(1.2, 1.4),  # vx
                Interval(2.3, 2.4),  # vy
                Interval(2.5, 2.7),  # vz
            ]
        )
        agent_two = HyperRectangle(
            [
                Interval(1.1, 1.3),  # x
                Interval(1.5, 1.6),  # y
                Interval(4.4, 5.6),  # z
                Interval(1.2, 1.4),  # vx
                Interval(2.3, 2.4),  # vy
                Interval(2.5, 2.7),  # vz
            ]
        )
        acceleration = (0.3, 0.4, 0.5)
        time_since_boot_ms = 5000

        agent_one_future_state, _ = SafetyChecker.face_lifting_iterative_improvement(
            agent_one, time_since_boot_ms, acceleration
        )
        agent_two_future_state, _ = SafetyChecker.face_lifting_iterative_improvement(
            agent_two, time_since_boot_ms, acceleration
        )

        self.assertTrue(
            SafetyChecker.check_collision(
                agent_one_future_state, agent_two_future_state, 2
            )
        )

        return

    def test_check_no_collision(self) -> None:
        """Verify that no collisions are detected between distant agents."""
        agent_one = HyperRectangle(
            [
                Interval(0.1, 0.3),  # x
                Interval(0.5, 0.6),  # y
                Interval(3.4, 5.6),  # z
                Interval(1.2, 1.4),  # vx
                Interval(2.3, 2.4),  # vy
                Interval(2.5, 2.7),  # vz
            ]
        )
        agent_two = HyperRectangle(
            [
                Interval(4.0, 6.6),  # x
                Interval(7.5, 9.6),  # y
                Interval(4.4, 5.6),  # z
                Interval(1.2, 1.4),  # vx
                Interval(2.3, 2.4),  # vy
                Interval(2.5, 2.7),  # vz
            ]
        )
        acceleration = (0.3, 0.4, 0.5)
        time_since_boot_ms = 5000

        agent_one_future_state, _ = SafetyChecker.face_lifting_iterative_improvement(
            agent_one, time_since_boot_ms, acceleration
        )
        agent_two_future_state, _ = SafetyChecker.face_lifting_iterative_improvement(
            agent_two, time_since_boot_ms, acceleration
        )

        self.assertFalse(
            SafetyChecker.check_collision(
                agent_one_future_state, agent_two_future_state, 2
            )
        )

        return


if __name__ == "__main__":
    unittest.main()
