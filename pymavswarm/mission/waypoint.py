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


class Waypoint:
    """Waypoint that an agent should fly to."""

    def __init__(
        self,
        latitude: float,
        longitude: float,
        altitude: float,
        airspeed: float | None = None,
        groundspeed: float | None = None,
    ) -> None:
        """
        Create a waypoint.

        :param latitude: waypoint's latitude
        :type latitude: float
        :param longitude: waypoint's longitude
        :type longitude: float
        :param altitude: waypoint's altitude
        :type altitude: float
        :param airspeed: target airspeed of the agent when passing through the waypoint,
            defaults to None
        :type airspeed: float | None, optional
        :param groundspeed: target groundspeed of the agent when passing through the
            waypoint, defaults to None
        :type groundspeed: float | None, optional
        """
        self.__latitude = latitude
        self.__longitude = longitude
        self.__altitude = altitude
        self.__airspeed = airspeed
        self.__groundspeed = groundspeed

        return

    @property
    def latitude(self) -> float:
        """
        Latitude of the waypoint.

        :return: waypoint latitude
        :rtype: float
        """
        return self.__latitude

    @property
    def longitude(self) -> float:
        """
        Longitude of the waypoint.

        :return: waypoint longitude
        :rtype: float
        """
        return self.__longitude

    @property
    def altitude(self) -> float:
        """
        Altitude of the waypoint.

        :return: waypoint altitude
        :rtype: float
        """
        return self.__altitude

    @property
    def airspeed(self) -> float | None:
        """
        Airspeed that the agent should have going through the waypoint.

        :return: agent airspeed at waypoint
        :rtype: float | None
        """
        return self.__airspeed

    @property
    def groundspeed(self) -> float | None:
        """
        Groundspeed that the agent should have when going through the waypoint.

        :return: agent groundspeed at waypoint
        :rtype: float | None
        """
        return self.__groundspeed
