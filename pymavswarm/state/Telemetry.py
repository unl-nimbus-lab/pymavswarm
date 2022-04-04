from .State import State



class Telemetry(State):
    """
    MAVLink Specification
    drop_rate   : drop_rate_comm : c% : Communication drop rate, (UART, I2C, SPI, CAN), dropped packets on all links 
                                        (packets that were corrupted on reception on the MAV)
    """
    def __init__(self, drop_rate: float=0.0) -> None:
        self.__drop_rate = drop_rate

        return


    def get_current_state(self) -> dict:
        return {
            'drop_rate': self.drop_rate
        }

    
    @property
    def drop_rate(self) -> float:
        return self.__drop_rate

    
    @drop_rate.setter
    def drop_rate(self, rate: float) -> None:
        self.__drop_rate = rate

        for cb in self.callbacks:
            cb(self.get_current_state())

        return