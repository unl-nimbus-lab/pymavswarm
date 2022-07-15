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

from pymavswarm.state.state import State


class Location(State):
    """Agent GPS location."""

    def __init__(
        self,
        latitude: float = 0.0,
        longitude: float = 0.0,
        altitude: float = 0.0,
        optional_context_props: dict | None = None,
    ) -> None:
        """
        Create a new location object.

        :param latitude: latitude [WGS84, EGM96 ellipsoid], defaults to 0.0
        :type latitude: float, optional
        :param longitude: longitude [WGS84, EGM96 ellipsoid], defaults to 0.0
        :type longitude: float, optional
        :param altitude: altitude [MSL]. Positive for up. Note that virtually all GPS
            modules provide the MSL altitude in addition to the WGS84 altitude,
            defaults to 0.0
        :type altitude: float, optional
        :param optional_context_props: properties to add to the location context,
            defaults to None
        :type optional_context_props: Optional[dict], optional
        """
        super().__init__(optional_context_props)

        self.__latitude = latitude
        self.__longitude = longitude
        self.__altitude = altitude

        return

    @property
    def latitude(self) -> float:
        """
        Latitude [WGS84, EGM96 ellipsoid].

        :return: latitude
        :rtype: float
        """
        return self.__latitude

    @latitude.setter
    def latitude(self, lat: float) -> None:
        """
        Set the latitude.

        :param lat: latitude [degE7]
        :type lat: float
        """
        prev_latitude = self.__latitude
        self.__latitude = lat

        # Signal state change event
        if self.__latitude != prev_latitude:
            self.state_changed_event.notify(context=self.context)

        return

    @property
    def longitude(self) -> float:
        """
        Longitude [WGS84, EGM96 ellipsoid].

        :return: longitude
        :rtype: float
        """
        return self.__longitude

    @longitude.setter
    def longitude(self, lon: float) -> None:
        """
        Set the longitude.

        :param lon: longitude [degE7]
        :type lon: float
        """
        prev_longitude = self.__longitude
        self.__longitude = lon

        # Signal state change event
        if self.__longitude != prev_longitude:
            self.state_changed_event.notify(context=self.context)

        return

    @property
    def altitude(self) -> float:
        """
        Altitude [MSL].

        Positive for up.

        :return: altitude
        :rtype: float
        """
        return self.__altitude

    @altitude.setter
    def altitude(self, alt: float) -> None:
        """
        Set the altitude.

        :param alt: altitude [MSL, mm]
        :type alt: float
        """
        prev_altitude = self.__altitude
        self.__altitude = alt

        # Signal state change event
        if self.__altitude != prev_altitude:
            self.state_changed_event.notify(context=self.context)

        return

    @property
    def context(self) -> dict:
        """
        Location context.

        :return: context
        :rtype: dict
        """
        context = super().context

        context["altitude"] = self.__altitude
        context["latitude"] = self.__latitude
        context["longitude"] = self.__longitude

        return context

    def __str__(self) -> str:
        """
        Print location in a human-readable format.

        :return: location
        :rtype: str
        """
        return f"Location: {self.context}"
