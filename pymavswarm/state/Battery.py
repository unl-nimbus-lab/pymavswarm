from .State import State


class Battery(State):
    """
    Agent battery state
    """

    def __init__(
        self,
        voltage: float = 0.0,
        current: float = 0.0,
        level: float = 0.0,
        callbacks: list = [],
    ) -> None:
        """
        :param voltage: Battery voltage (mV), UINT16_MAX: Voltage not sent by autopilot,
            defaults to 0.0
        :type voltage: float, optional

        :param current: Battery current (cA), -1: Current not sent by autopilot,
            defaults to 0.0
        :type current: float, optional

        :param level: Battery energy remaining (%), -1: Battery remaining energy not
            sent by autopilot, defaults to 0.0
        :type level: float, optional

        :param callbacks: State change observers, defaults to []
        :type callbacks: list, optional
        """
        super().__init__(callbacks)

        self.__voltage = voltage
        self.__current = current
        self.__level = level

        return

    def get_current_state(self) -> dict:
        """
        Get the current state as a dictionary for callbacks

        :return: Properties of interest associated with the battery state
        :rtype: dict
        """
        return {"voltage": self.voltage, "current": self.current, "level": self.level}

    @property
    def voltage(self) -> float:
        """
        Battery voltage (mV), UINT16_MAX: Voltage not sent by autopilot

        :rtype: float
        """
        return self.__voltage

    @voltage.setter
    def voltage(self, voltage: float) -> None:
        """
        voltage setter

        :param voltage: Battery voltage (mV)
        :type voltage: float
        """
        self.__voltage = voltage

        for cb in self.callbacks:
            cb(self.get_current_state())

        return

    @property
    def current(self) -> float:
        """
        Battery current (cA), -1: Current not sent by autopilot

        :rtype: float
        """
        return self.__current

    @current.setter
    def current(self, current: float) -> None:
        """
        current setter

        :param current: Battery current (cA)
        :type current: float
        """
        if current == -1:
            self.__current = current
        else:
            self.__current = current / 100.0

        for cb in self.callbacks:
            cb(self.get_current_state())

        return

    @property
    def level(self) -> float:
        """
        Battery energy remaining (%), -1: Battery remaining energy not
        sent by autopilot, defaults to 0.0

        :rtype: float
        """
        return self.__level

    @level.setter
    def level(self, level: float) -> None:
        """
        level setter

        :param level: Battery level (%)
        :type level: float
        """
        self.__level = level

        for cb in self.callbacks:
            cb(self.get_current_state())

        return
