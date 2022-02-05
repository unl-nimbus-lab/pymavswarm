class Location:
    def __init__(self, latitude: float=0.0, 
                 longitude: float=0.0, 
                 altitude: float=0.0) -> None:
        """
        MAVLink Specification
        latitude  : lat : degE7 : Latitude (WGS84, EGM96 ellipsoid)
        longitude : lon : degE7 : Longitude (WGS84, EGM96 ellipsoid)
        altitude  : alt : mm    : Altitude (MSL). Positive for up. Note that virtually all GPS modules 
                                  provide the MSL altitude in addition to the WGS84 altitude.
        """
        self.latitude = latitude
        self.longitude = longitude
        self.altitude = altitude

        return