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

import logging
from typing import Optional

import pymavswarm.utils as swarm_utils
from pymavswarm.mission import Waypoint


class Mission:
    """
    A set of waypoints that should be executed sequentially by an agent
    """

    def __init__(self, waypoints: list = [], log_level: int = logging.INFO) -> None:
        """
        :param waypoints: The set of waypoints to be executed, defaults to []
        :type waypoints: list, optional

        :param log_level: The desired debugging level, defaults to logging.INFO
        :type debug: int, optional
        """
        self.__logger = swarm_utils.init_logger("mission", log_level=log_level)
        self.__waypoints = waypoints

        return

    @property
    def waypoints(self) -> list:
        """
        The set of waypoints included in the mission

        :rtype: list
        """
        return self.__waypoints

    def add_waypoint(self, waypoint: Waypoint, index: Optional[int] = None) -> bool:
        """
        Add a waypoint to the mission

        :param waypoint: The new waypoint to add
        :type waypoint: Waypoint

        :param index: The index that the waypoint should be placed at in the mission
            sequence, defaults to None
        :type index: Optional[int], optional

        :return: Indication of whether the waypoint was inserted successfully
        :rtype: bool
        """
        if index is not None:
            try:
                self.__waypoints.insert(index)
            except IndexError:
                self.__logger.exception(
                    "An attempt was made to insert a waypoint at "
                    f"an invalid index. Mission size: {len(self.__waypoints)}. "
                    f"Index: {index}"
                )
                return False
        else:
            self.__waypoints.append(waypoint)

        return True

    def remove_waypoint(self, waypoint: Waypoint) -> bool:
        """
        Remove the provided waypoint from the list of waypoints

        :param waypoint: Waypoint to remove
        :type waypoint: Waypoint

        :return: Flag indicating whether the waypoint was successfully removed
        :rtype: bool
        """
        if waypoint in self.__waypoints:
            self.__waypoints.remove(waypoint)

        return True

    def remove_waypoint_by_index(self, index: int) -> bool:
        """
        Remove the waypoint at the given index

        :param index: The index of the waypoint in the mission that should be removed
        :type index: int

        :return: Indication of whether the waypoint was removed successfully
        :rtype: bool
        """
        try:
            self.__waypoints.pop(index)
        except IndexError:
            self.__logger.exception(
                f"An attempt was made to remove a waypoint at an invalid index. "
                f"Mission size: {len(self.__waypoints)}. Index: {index}"
            )
            return False

        return True

    def remove_waypoint_by_value(
        self, latitude: float, longitude: float, altitude: float
    ) -> bool:
        """
        Remove all waypoints that have the specified latitude, longitude, and altitude
        values

        :param latitude: Waypoint's latitude
        :type latitude: float

        :param longitude: Waypoint's longitude
        :type longitude: float

        :param altitude: Waypoint's altitude
        :type altitude: float

        :return: Indication of whether the waypoint was removed properly
        :rtype: bool
        """
        for waypoint in self.__waypoints:
            if (
                waypoint.latitude == latitude
                and waypoint.longitude == longitude
                and waypoint.altitude == altitude
            ):
                self.__waypoints.remove(waypoint)

        return True

    def clear_waypoints(self) -> None:
        """
        Remove all waypoints from the mission
        """
        self.__waypoints.clear()

        return
