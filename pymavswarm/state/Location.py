from .State import State


class Location(State):
    """
    GPS location
    """

    def __init__(
        self, latitude: float = 0.0, longitude: float = 0.0, altitude: float = 0.0
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
        """
        self.__latitude = latitude
        self.__longitude = longitude
        self.__altitude = altitude

        return

    def get_current_state(self) -> dict:
        """
        Get the current state as a dictionary for callbacks

        :return: Properties of interested associated with the GPS information
        :rtype: dict
        """
        return {
            "latitude": self.latitude,
            "longitude": self.longitude,
            "altitude": self.altitude,
        }

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
        self.__latitude = lat

        for cb in self.callbacks:
            cb(self.get_current_state())

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
        self.__longitude = lon

        for cb in self.callbacks:
            cb(self.get_current_state())

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
        self.__altitude = alt

        for cb in self.callbacks:
            cb(self.get_current_state())

        return
