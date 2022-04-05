from .State import State


class Telemetry(State):
    """
    Telemetry state information
    """

    def __init__(self, drop_rate: float = 0.0) -> None:
        """
        :param drop_rate: Communication drop rate, (UART, I2C, SPI, CAN), dropped
            packets on all links (packets that were corrupted on reception on the MAV),
            defaults to 0.0
        :type drop_rate: float, optional
        """
        self.__drop_rate = drop_rate

        return

    def get_current_state(self) -> dict:
        """
        Get the current state as a dictionary for callbacks

        :return: Properties of interested associated with the telemetry
        :rtype: dict
        """
        return {"drop_rate": self.drop_rate}

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
        self.__drop_rate = rate

        for cb in self.callbacks:
            cb(self.get_current_state())

        return
