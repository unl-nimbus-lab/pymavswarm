from pymavswarm.state.state import State


class Telemetry(State):
    """
    Telemetry state information
    """

    def __init__(
        self, drop_rate: float = 0.0, optional_context_props: dict = {}
    ) -> None:
        """
        :param drop_rate: Communication drop rate, (UART, I2C, SPI, CAN), dropped
            packets on all links (packets that were corrupted on reception on the MAV),
            defaults to 0.0
        :type drop_rate: float, optional

        :param optional_context_props: Optional properties to add to the context
        :type optional_context_props: dict, optional
        """
        super().__init__()

        self.__drop_rate = drop_rate
        self.__optional_context_props = optional_context_props

        return

    @property
    def drop_rate(self) -> float:
        """
        Communication drop rate, (UART, I2C, SPI, CAN), dropped packets on all links
        (packets that were corrupted on reception on the MAV)

        :rtype: float
        """
        return self.__drop_rate

    @drop_rate.setter
    def drop_rate(self, rate: float) -> None:
        """
        drop_rate setter

        :param rate: Drop rate (c%)
        :type rate: float
        """
        prev_drop_rate = self.__drop_rate
        self.__drop_rate = rate

        # Signal state change event
        if self.__drop_rate != prev_drop_rate:
            self.state_changed_event.notify(context=self.context)

        return

    @property
    def context(self) -> dict:
        """
        Get the current state as a dictionary for callbacks

        :return: Properties of interested associated with the telemetry
        :rtype: dict
        """
        context = {"drop_rate": self.__drop_rate}
        context.update(self.__optional_context_props)

        return context
