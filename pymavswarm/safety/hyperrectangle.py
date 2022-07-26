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

from __future__ import annotations

from copy import deepcopy

from pymavswarm.safety.interval import Interval


class HyperRectangle:
    """
    Orthotope used for computing reachable drone states.

    This implementation has been inspired by the following source:
    https://github.com/verivital/rtreach/blob/master/src/geometry.c
    """

    def __init__(self, intervals: list[Interval]) -> None:
        """
        Create a new hyperrectangle.

        :param intervals: intervals that should be used to construct the hyperrectangle
            faces
        :type intervals: list[Interval]
        """
        if len(intervals) <= 0:
            raise ValueError(
                "Invalid HyperRectangle dimensions provided. Ensure that the "
                "dimensions are non-negative."
            )

        # Construct the dimensions for the hyper-rectangle
        self.__intervals = intervals
        self.__dimensions = len(self.__intervals)
        self.__faces = 2 * len(self.__intervals)

        return

    @property
    def faces(self) -> int:
        """
        Get the total number of faces in the hyperrectangle.

        :return: number of faces
        :rtype: int
        """
        return self.__faces

    @property
    def dimensions(self) -> int:
        """
        Get the hyperrectangle dimensions.

        :return: dimensions
        :rtype: int
        """
        return self.__dimensions

    @property
    def intervals(self) -> list[Interval]:
        """
        Get the hyperrectangle intervals.

        :return: intervals
        :rtype: list[Interval]
        """
        return self.__intervals

    @property
    def max_width(self) -> float:
        """
        Get the maximum interval width.

        :return: maximum distance between the intervals
        :rtype: float
        """
        return max(
            [
                interval.interval_max - interval.interval_min
                for interval in self.__intervals
            ]
        )

    def contains(self, inside_rectangle: HyperRectangle) -> bool:
        """
        Determine if this hyperrectangle contains the given hyperrectangle.

        :param inside_rectangle: hyperrectangle to check for containment
        :type inside_rectangle: HyperRectangle
        :return: inside_rectangle is contained by this hyperrectangle
        :rtype: bool
        """
        if self.__dimensions != inside_rectangle.dimensions:
            raise ValueError(
                "The provided rectangle does not have the same dimensions as this "
                f"rectangle. Expected dimensions: {self.__dimensions}, but got "
                f"{inside_rectangle.dimensions}"
            )

        for dim in range(self.__dimensions):
            if (
                inside_rectangle.intervals[dim].interval_min
                < self.__intervals[dim].interval_min
                or inside_rectangle.intervals[dim].interval_max
                > self.__intervals[dim].interval_max
            ):
                return False

        return True

    def convex_hull(
        self, contained: HyperRectangle, in_place: bool = True
    ) -> HyperRectangle | None:
        """
        Compute the convex hull of this rectangle and the contained rectangle.

        :param contained: rectangle to compute the convex hull with
        :type contained: HyperRectangle
        :param in_place: compute the hull in place, defaults to True
        :type in_place: bool, optional
        :raises ValueError: dimensions of the containing rectangle don't match the
            dimensions of this rectangle
        :return: convex hull if not computed in place
        :rtype: HyperRectangle | None
        """
        if self.__dimensions != contained.dimensions:
            raise ValueError(
                f"The dimensions of the provided rectangle {contained.dimensions} is "
                f"not equal to the current dimensions {self.__dimensions}"
            )

        if not in_place:
            result_rect = deepcopy(self)

        for dim in range(self.__dimensions):
            if (
                contained.intervals[dim].interval_min
                < self.__intervals[dim].interval_min
            ):
                if in_place:
                    self.__intervals[dim].interval_min = contained.intervals[
                        dim
                    ].interval_min
                else:
                    result_rect.intervals[dim].interval_min = contained.intervals[
                        dim
                    ].interval_min

            if (
                contained.intervals[dim].interval_max
                > self.__intervals[dim].interval_max
            ):
                if in_place:
                    self.__intervals[dim].interval_max = contained.intervals[
                        dim
                    ].interval_max
                else:
                    result_rect.intervals[dim].interval_max = contained.intervals[
                        dim
                    ].interval_max

        if not in_place:
            return result_rect

        return None
