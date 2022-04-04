from .State import State



class Velocity(State):
    """
    MAVLink Specification
    vx : cm/s : Ground X Speed (Latitude, positive north)
    vy : cm/s : Ground Y Speed (Longitude, positive east)
    vz : cm/s : Ground Z Speed (Altitude, positive down)
    """
    def __init__(self, vx: float=0.0, 
                 vy: float=0.0, 
                 vz: float=0.0) -> None:
        self.__vx = vx
        self.__vy = vy
        self.__vz = vz

        return

    def get_current_state(self) -> dict:
        return {
            'vx': self.vx,
            'vy': self.vy,
            'vz': self.vz
        }
    
    @property
    def vx(self) -> float:
        return self.__vx
    

    @vx.setter
    def vx(self, vel: float) -> None:
        self.__vx = vel

        for cb in self.callbacks:
            cb(self.get_current_state())

        return

    @property
    def vy(self) -> float:
        return self.__vy
    

    @vy.setter
    def vy(self, vel: float) -> None:
        self.__vy = vel

        for cb in self.callbacks:
            cb(self.get_current_state())

        return

    @property
    def vz(self) -> float:
        return self.__vz
    

    @vz.setter
    def vz(self, vel: float) -> None:
        self.__vz = vel

        for cb in self.callbacks:
            cb(self.get_current_state())

        return