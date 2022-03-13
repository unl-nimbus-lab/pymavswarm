from .AgentMsg import AgentMsg
from utils import SupportedCommands


class FlightSpeedMsg(AgentMsg):
    """
    FlightSpeedMsg represents messages that should change the flight speed of an agent.
    This may be either ground speed or air speed in m/s.
    """
    def __init__(self, speed: float, speed_type: int, target_system: int, target_comp: int, retry: bool, msg_timeout: float=5.0) -> None:
        supported_speed_types = SupportedCommands().flight_speed_commands

        if speed_type not in supported_speed_types:
            raise TypeError(f'{speed_type} is not a supported speed command type. Supported commands include: {supported_speed_types}')

        super().__init__('FLIGHT_SPEED', target_system, target_comp, retry, msg_timeout=msg_timeout)
        self.speed = speed
        self.speed_type = speed_type

        return
