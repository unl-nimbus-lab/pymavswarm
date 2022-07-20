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


class Interval:
    def __init__(self, interval_min: float, interval_max: float) -> None:
        self.__interval_min = interval_min
        self.__interval_max = interval_max

        return

    @property
    def interval_min(self) -> float:
        self.__interval_min

    @interval_min.setter
    def interval_min(self, min_value: float) -> None:
        self.__interval_min = min_value
        return

    @property
    def interval_max(self) -> float:
        self.__interval_max

    @interval_max.setter
    def interval_max(self, max_value: float) -> None:
        self.__interval_max = max_value
        return


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
        # Construct the dimensions for the hyper-rectangle
        self.__dimensions: list[Interval] = [
            Interval(0, float("inf")) for _ in range(dimensions)
        ]

        return

    @property
    def num_faces(self) -> int:
        """
        Get the total number of faces in the hyperrectangle.

        :return: number of faces
        :rtype: int
        """
        return 2 * len(self.__dimensions)

    @property
    def dimensions(self) -> list[Interval]:
        """
        Get the hyperrectangle dimensions.

        :return: intervals
        :rtype: list[Interval]
        """
        return self.__dimensions

    @property
    def max_width(self) -> float:
        """
        Get the maximum face width.

        :return: maximum distance between the face intervals
        :rtype: float
        """
        return max(
            [face.interval_max - face.interval_min for face in self.__dimensions]
        )

    def contains(self, rect: HyperRectangle) -> bool:
        """
        Determine if another hyperrectangle is contained within this hyperrectangle.

        :param rect: hyperrectangle to check for containment
        :type rect: HyperRectangle
        :return: rect is contained by this hyperrectangle
        :rtype: bool
        """
        for dim in range(len(self.__dimensions)):
            if (
                rect.dimensions[dim].interval_min < self.__dimensions[dim].interval_min
                or rect.dimensions[dim].interval_max
                > self.__dimensions[dim].interval_max
            ):
                return False

        return True
