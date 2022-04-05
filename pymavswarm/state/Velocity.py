from .State import State


class Velocity(State):
    """
    Velocity state
    """

    def __init__(self, vx: float = 0.0, vy: float = 0.0, vz: float = 0.0) -> None:
        """
        :param vx: Ground X Speed (Latitude, positive north), defaults to 0.0
        :type vx: float, optional

        :param vy: Ground Y Speed (Longitude, positive east), defaults to 0.0
        :type vy: float, optional

        :param vz: Ground Z Speed (Altitude, positive down), defaults to 0.0
        :type vz: float, optional
        """
        self.__vx = vx
        self.__vy = vy
        self.__vz = vz

        return

    def get_current_state(self) -> dict:
        """
        Get the current state as a dictionary for callbacks

        :return: Properties of interested associated with the velocity
        :rtype: dict
        """
        return {"vx": self.vx, "vy": self.vy, "vz": self.vz}

    @property
    def vx(self) -> float:
        """
        Ground X Speed (Latitude, positive north)

        :rtype: float
        """
        return self.__vx

    @vx.setter
    def vx(self, vel: float) -> None:
        """
        vx setter

        :param vel: X speed (cm/s)
        :type vel: float
        """
        self.__vx = vel

        for cb in self.callbacks:
            cb(self.get_current_state())

        return

    @property
    def vy(self) -> float:
        """
        Ground Y Speed (Longitude, positive east)

        :rtype: float
        """
        return self.__vy

    @vy.setter
    def vy(self, vel: float) -> None:
        """
        vy setter

        :param vel: Y speed (cm/s)
        :type vel: float
        """
        self.__vy = vel

        for cb in self.callbacks:
            cb(self.get_current_state())

        return

    @property
    def vz(self) -> float:
        """
        Ground Z Speed (Altitude, positive down)

        :rtype: float
        """
        return self.__vz

    @vz.setter
    def vz(self, vel: float) -> None:
        """
        vz setter

        :param vel: Z speed (cm/s)
        :type vel: float
        """
        self.__vz = vel

        for cb in self.callbacks:
            cb(self.get_current_state())

        return
