from .State import State



class Location(State):
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
        self.__latitude = latitude
        self.__longitude = longitude
        self.__altitude = altitude

        return


    def get_current_state(self) -> dict:
        return {
            'latitude': self.latitude,
            'longitude': self.longitude,
            'altitude': self.altitude
        }


    @property
    def latitude(self) -> float:
        return self.__latitude


    @latitude.setter
    def latitude(self, lat: float) -> None:
        self.__latitude = lat

        for cb in self.callbacks:
            cb(self.get_current_state())
            
        return

    
    @property
    def longitude(self) -> float:
        return self.__longitude

    
    @longitude.setter
    def longitude(self, lon: float) -> None:
        self.__longitude = lon

        for cb in self.callbacks:
            cb(self.get_current_state())
            
        return


    @property
    def altitude(self) -> float:
        return self.__altitude

    
    @altitude.setter
    def altitude(self, alt: float) -> None:
        self.__altitude = alt

        for cb in self.callbacks:
            cb(self.get_current_state())
            
        return