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

from pymavswarm.safety import Interval


class HyperRectangle:
    """
    Orthotope used for computing reachable drone states.

    This implementation has been inspired by the following source:
    https://github.com/verivital/rtreach/blob/master/src/geometry.c
    """

    def __init__(self, dimensions: int) -> None:
        """
        Create a new hyperrectangle.

        :param dimensions: number of dimensions that the hyperrectangle should exist in
        :type dimensions: int
        """
        if dimensions <= 0:
            raise ValueError(
                "Invalid HyperRectangle dimensions provided. Ensure that the "
                "dimensions are non-negative."
            )

        # Construct the dimensions for the hyper-rectangle
        self.__intervals: list[Interval] = [
            Interval(0, float("inf")) for _ in range(dimensions)
        ]
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
        Get the maximum face width.

        :return: maximum distance between the face intervals
        :rtype: float
        """
        return max([face.interval_max - face.interval_min for face in self.__intervals])

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
