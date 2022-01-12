from .AgentMsg import AgentMsg



class FlightSpeedMsg(AgentMsg):
    """
    FlightSpeedMsg represents messages that should change the flight speed of an agent.
    This may be either ground speed or air speed in m/s.
    """
    def __init__(self, speed: float, msg_type: str, sys_id: int, comp_id: int, check_ack: bool, msg_timeout: float=5.0) -> None:
        super().__init__(msg_type, sys_id, comp_id, check_ack, msg_timeout=msg_timeout)
        self.speed = speed

        return
