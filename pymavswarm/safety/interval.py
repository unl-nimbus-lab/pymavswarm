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
    """Interval used for a HyperRectangle face."""

    def __init__(self, interval_min: float, interval_max: float) -> None:
        """
        Create a new Interval.

        :param interval_min: minimum interval value
        :type interval_min: float
        :param interval_max: maximum interval value
        :type interval_max: float
        """
        if interval_min > interval_max:
            raise ValueError(
                "Ensure that the interval minimum is smaller than the interval maximum."
            )

        self.__interval_min = interval_min
        self.__interval_max = interval_max

        return

    @property
    def interval_min(self) -> float:
        """
        Minimum value in the interval.

        :return: interval minimum
        :rtype: float
        """
        return self.__interval_min

    @interval_min.setter
    def interval_min(self, min_value: float) -> None:
        """
        Set the interval minimum.

        :param min_value: new minimum value
        :type min_value: float
        """
        self.__interval_min = min_value

        return

    @property
    def interval_max(self) -> float:
        """
        Maximum interval value.

        :return: interval maximum
        :rtype: float
        """
        return self.__interval_max

    @interval_max.setter
    def interval_max(self, max_value: float) -> None:
        """
        Set the interval maximum.

        :param max_value: new interval maximum
        :type max_value: float
        """
        self.__interval_max = max_value

        return

    def __str__(self) -> str:
        """
        Print an interval in a human-readable format.

        :return: interval
        :rtype: str
        """
        return f"Interval: ({self.__interval_min}, {self.__interval_max})"
