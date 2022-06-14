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

from pymavswarm.state.state import State


class Location(State):
    """
    GPS location
    """

    def __init__(
        self,
        latitude: float = 0.0,
        longitude: float = 0.0,
        altitude: float = 0.0,
        optional_context_props: dict = {},
    ) -> None:
        """
        :param latitude: Latitude (WGS84, EGM96 ellipsoid), defaults to 0.0
        :type latitude: float, optional

        :param longitude: Longitude (WGS84, EGM96 ellipsoid), defaults to 0.0
        :type longitude: float, optional

        :param altitude: Altitude (MSL). Positive for up. Note that virtually all GPS
            modules provide the MSL altitude in addition to the WGS84 altitude,
            defaults to 0.0
        :type altitude: float, optional

        :param optional_context_props: Optional properties to add to the context
        :type optional_context_props: dict, optional
        """
        super().__init__()

        self.__latitude = latitude
        self.__longitude = longitude
        self.__altitude = altitude
        self.__optional_context_props = optional_context_props

        return

    @property
    def latitude(self) -> float:
        """
        Latitude (WGS84, EGM96 ellipsoid)

        :rtype: float
        """
        return self.__latitude

    @latitude.setter
    def latitude(self, lat: float) -> None:
        """
        latitude setter

        :param lat: Latitude (degE7)
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
        Longitude (WGS84, EGM96 ellipsoid)

        :rtype: float
        """
        return self.__longitude

    @longitude.setter
    def longitude(self, lon: float) -> None:
        """
        longitude setter

        :param lon: Longitude (degE7)
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
        Altitude (MSL). Positive for up.

        :rtype: float
        """
        return self.__altitude

    @altitude.setter
    def altitude(self, alt: float) -> None:
        """
        altitude setter

        :param alt: Altitude (MSL, mm)
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
        Get the current state as a dictionary for callbacks

        :return: Properties of interested associated with the GPS information
        :rtype: dict
        """
        context = {
            "latitude": self.__latitude,
            "longitude": self.__longitude,
            "altitude": self.__altitude,
        }
        context.update(self.__optional_context_props)

        return context
