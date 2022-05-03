from pymavswarm.state.State import State


class Battery(State):
    """
    Agent battery state
    """

    def __init__(
        self, voltage: float = 0.0, current: float = 0.0, level: float = 0.0
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
        """
        super().__init__()

        self.__voltage = voltage
        self.__current = current
        self.__level = level

        return

    @property
    def context(self) -> dict:
        """
        Get the current state as a dictionary for callbacks

        :return: Properties of interest associated with the battery state
        :rtype: dict
        """
        return {
            "voltage": self.__voltage,
            "current": self.__current,
            "level": self.__level,
        }

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

        # Signal state change event
        self.state_changed_event.notify(context=self.context)

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

        # Signal state change event
        self.state_changed_event.notify(context=self.context)

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

        # Signal state change event
        self.state_changed_event.notify(context=self.context)

        return
