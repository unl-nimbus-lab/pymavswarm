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

from pymavswarm.safety import HyperRectangle, Interval


class TestHyperRectangle(unittest.TestCase):
    """Test the HyperRectangle class."""

    def test_dimensions(self) -> None:
        """Verify that the dimensions are correctly computed."""
        intervals = [Interval(0, 1), Interval(0, 1)]
        rect = HyperRectangle(intervals)

        # Ensure that the dimensions are equal to the length of the intervals list
        self.assertEqual(len(intervals), rect.dimensions)

        return

    def test_faces(self) -> None:
        """Verify that the number of faces is computed properly."""
        intervals = [Interval(0, 1), Interval(0, 1)]
        rect = HyperRectangle(intervals)

        # Ensure that the number of faces is equal to 2x the length of the intervals
        # list
        self.assertEqual(2 * len(intervals), rect.faces)

        return

    def test_max_width(self) -> None:
        """Ensure that the maximum interval width is properly computed."""
        max_interval_range = (0, 5)
        min_interval_range = (0, 1)

        # Compute the width of the interval
        max_interval_width = max_interval_range[1] - max_interval_range[0]

        intervals = [Interval(*max_interval_range), Interval(*min_interval_range)]
        rect = HyperRectangle(intervals)

        # Verify that the max width property is equivalent to the max width
        # pre-determined
        self.assertEqual(max_interval_width, rect.max_width)

        return

    def test_invalid_contains(self) -> None:
        """Verify that an error is raised when rectangle dimensions don't match up."""
        outside = HyperRectangle([Interval(0, 1), Interval(1, 2)])
        inside = HyperRectangle([Interval(0, 1)])

        self.assertRaises(ValueError, outside.contains, inside)

        return

    def test_contains(self) -> None:
        """Verify that rectangles properly determine when another is contained."""
        outside = HyperRectangle([Interval(0, 5), Interval(3, 10)])
        inside = HyperRectangle([Interval(1, 4), Interval(4, 7)])

        self.assertTrue(outside.contains(inside))

        return

    def test_does_not_contain(self) -> None:
        """Verify that rectangles properly determine when another is not contained."""
        outside = HyperRectangle([Interval(0, 5), Interval(3, 10)])

        # The inside rectangle has an interval that is smaller than the outside
        # rectangle
        inside = HyperRectangle([Interval(-1, 4), Interval(4, 7)])

        self.assertFalse(outside.contains(inside))

        return

    def test_invalid_convex_hull(self) -> None:
        """Verify that an error is raised when rectangle dimensions don't match."""
        outside = HyperRectangle([Interval(0, 5), Interval(3, 10)])
        inside = HyperRectangle([Interval(1, 4)])

        self.assertRaises(ValueError, outside.convex_hull, inside)

        return

    def test_convex_hull(self) -> None:
        """Verify that the convex hull is computed properly."""
        rect_one = HyperRectangle([Interval(0, 5), Interval(3, 10)])
        rect_two = HyperRectangle([Interval(-1, 4), Interval(4, 12)])

        # Create a hyperrectangle with the expected values
        expected_hull = HyperRectangle([Interval(-1, 5), Interval(3, 12)])

        # Compute the convex hull
        resulting_hull = rect_one.convex_hull(rect_two, in_place=False)

        for dim in range(resulting_hull.dimensions):  # type: ignore
            self.assertEqual(
                resulting_hull.intervals[dim].interval_min,  # type: ignore
                expected_hull.intervals[dim].interval_min,
            )
            self.assertEqual(
                resulting_hull.intervals[dim].interval_max,  # type: ignore
                expected_hull.intervals[dim].interval_max,
            )

        return

    def test_convex_hull_in_place(self) -> None:
        """Verify that the convex hull is computed properly in place."""
        rect_one = HyperRectangle([Interval(0, 5), Interval(3, 10)])
        rect_two = HyperRectangle([Interval(-1, 4), Interval(4, 12)])

        # Create a hyperrectangle with the expected values
        expected_hull = HyperRectangle([Interval(-1, 5), Interval(3, 12)])

        # Compute the convex hull
        rect_one.convex_hull(rect_two, in_place=True)

        for dim in range(rect_one.dimensions):  # type: ignore
            self.assertEqual(
                rect_one.intervals[dim].interval_min,  # type: ignore
                expected_hull.intervals[dim].interval_min,
            )
            self.assertEqual(
                rect_one.intervals[dim].interval_max,  # type: ignore
                expected_hull.intervals[dim].interval_max,
            )

        return


if __name__ == "__main__":
    unittest.main()
