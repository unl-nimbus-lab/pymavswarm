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

import logging

import pymavswarm.utils as swarm_utils
from pymavswarm.mission.waypoint import Waypoint


class SwarmMission:
    """Set of waypoints that should be executed sequentially by an agent."""

    def __init__(
        self,
        waypoints: list[Waypoint] | None = None,
        log_level: int = logging.INFO,
    ) -> None:
        """
        Create a new mission.

        :param waypoints: The set of waypoints to be executed, defaults to None
        :type waypoints: list[Waypoint] | None, optional
        :param log_level: The desired debugging level, defaults to logging.INFO
        :type debug: int, optional
        """
        self.__logger = swarm_utils.init_logger(__name__, log_level=log_level)
        self.__waypoints = waypoints

        return

    @property
    def waypoints(self) -> list[Waypoint]:
        """
        Set of waypoints included in the mission.

        :return: mission waypoints
        :rtype: List[Waypoint]
        """
        if self.__waypoints is None:
            return []

        return self.__waypoints

    def add_waypoint(self, waypoint: Waypoint, index: int | None = None) -> bool:
        """
        Add a waypoint to the mission.

        :param waypoint: new waypoint to add
        :type waypoint: Waypoint
        :param index: index that the waypoint should be placed at in the mission
            sequence, defaults to None
        :type index: int | None, optional
        :return: whether or not the waypoint was inserted successfully
        :rtype: bool
        """
        if self.__waypoints is None:
            self.__waypoints = []

        if index is not None:
            try:
                self.__waypoints.insert(index, waypoint)
            except IndexError:
                self.__logger.exception(
                    "An attempt was made to insert a waypoint at an invalid index. "
                    f"Mission size: {len(self.__waypoints)}. Index: {index}"
                )
                return False
        else:
            self.__waypoints.append(waypoint)

        return True

    def remove_waypoint(self, waypoint: Waypoint) -> bool:
        """
        Remove the provided waypoint from the list of waypoints.

        :param waypoint: waypoint to remove
        :type waypoint: Waypoint
        :return: flag indicating whether the waypoint was successfully removed
        :rtype: bool
        """
        if self.__waypoints is None:
            return False

        if waypoint in self.__waypoints:
            self.__waypoints.remove(waypoint)

        return True

    def remove_waypoint_by_index(self, index: int) -> bool:
        """
        Remove the waypoint at the given index.

        :param index: index of the waypoint in the mission that should be removed
        :type index: int
        :return: whether or not the waypoint was removed successfully
        :rtype: bool
        """
        if self.__waypoints is None:
            return False

        try:
            self.__waypoints.pop(index)
        except IndexError:
            return False

        return True

    def remove_waypoint_by_value(
        self, latitude: float, longitude: float, altitude: float
    ) -> bool:
        """
        Remove all waypoints that are located at the provided geographic location.

        :param latitude: waypoint's latitude
        :type latitude: float
        :param longitude: waypoint's longitude
        :type longitude: float
        :param altitude: waypoint's altitude
        :type altitude: float
        :return: whether or not the waypoint was removed properly
        :rtype: bool
        """
        if self.__waypoints is None:
            return False

        for waypoint in self.__waypoints:
            if (
                waypoint.latitude == latitude
                and waypoint.longitude == longitude
                and waypoint.altitude == altitude
            ):
                self.__waypoints.remove(waypoint)

        return True

    def clear_waypoints(self) -> None:
        """Remove all waypoints from the mission."""
        if self.__waypoints is not None:
            self.__waypoints.clear()

        return
