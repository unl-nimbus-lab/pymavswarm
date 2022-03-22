class Waypoint:
    """
    Waypoint defines a waypoint that an agent should fly to. The airspeed (m/s)
    and ground speed (m/s) may also be set
    """
    def __init__(self, latitude: float, 
                 longitude: float, 
                 altitude: float, 
                 air_speed: float=None, 
                 ground_speed: float=None) -> None:
        self.latitude = latitude
        self.longitude = longitude
        self.altitude = altitude
        self.air_speed = air_speed
        self.ground_speed = ground_speed

        return