from .AgentMsg import AgentMsg
from .MsgMap import FlightSpeedCommand


class FlightSpeedMsg(AgentMsg):
    """
    FlightSpeedMsg represents messages that should change the flight speed of an agent.
    This may be either ground speed or air speed in m/s.
    """
    def __init__(self, speed: float, msg_type: str, target_system: int, target_comp: int, retry: bool, msg_timeout: float=5.0) -> None:
        assert msg_type in FlightSpeedCommand, f'Attempted to create an invalid flight speed command: {msg_type}'

        super().__init__(msg_type, target_system, target_comp, retry, msg_timeout=msg_timeout)
        self.speed = speed

        return
