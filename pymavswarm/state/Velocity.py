class Velocity:
    def __init__(self, 
                 vx: float=0.0, 
                 vy: float=0.0, 
                 vz: float=0.0) -> None:
        """
        MAVLink Specification
        vx : cm/s : Ground X Speed (Latitude, positive north)
        vy : cm/s : Ground Y Speed (Longitude, positive east)
        vz : cm/s : Ground Z Speed (Altitude, positive down)
        """
        self.vx = vx
        self.vy = vy
        self.vz = vz