from typing import Optional


class Waypoint:
    """
    Waypoint that an agent should fly to.
    """

    def __init__(
        self,
        latitude: float,
        longitude: float,
        altitude: float,
        airspeed: Optional[float] = None,
        groundspeed: Optional[float] = None,
    ) -> None:
        """
        Constructor.

        :param latitude: The waypoint's latitude
        :type latitude: float

        :param longitude: The waypoint's longitude
        :type longitude: float

        :param altitude: The waypoint's altitude
        :type altitude: float

        :param airspeed: Target airspeed of the agent when passing through the waypoint,
            defaults to None
        :type airspeed: Optional[float], optional

        :param groundspeed: Target groundspeed of the agent when passing through the
            waypoint, defaults to None
        :type groundspeed: Optional[float], optional
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

        :rtype: float
        """
        return self.__latitude

    @property
    def longitude(self) -> float:
        """
        Longitude of the waypoint.

        :rtype: float
        """
        return self.__longitude

    @property
    def altitude(self) -> float:
        """
        Altitude of the waypoint.

        :rtype: float
        """
        return self.__altitude

    @property
    def airspeed(self) -> Optional[float]:
        """
        Airspeed that the agent should have going through the waypoint.

        :rtype: Optional[float]
        """
        return self.__airspeed

    @property
    def groundspeed(self) -> Optional[float]:
        """
        Groundspeed that the agent should have when going through the waypoint.

        :rtype: Optional[float]
        """
        return self.__groundspeed
