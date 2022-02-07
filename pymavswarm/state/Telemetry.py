class Telemetry:
    def __init__(self, drop_rate: float=0.0, 
                 comm_errors=None) -> None:
        """
        MAVLink Specification
        drop_rate   : drop_rate_comm : c% : Communication drop rate, (UART, I2C, SPI, CAN), dropped packets on all links 
                                            (packets that were corrupted on reception on the MAV)
        comm_errors : errors_comm    :    : Communication errors (UART, I2C, SPI, CAN), dropped packets on 
                                            all links (packets that were corrupted on reception on the MAV)
        """
        self.drop_rate = drop_rate
        self.comm_errors = comm_errors

        return