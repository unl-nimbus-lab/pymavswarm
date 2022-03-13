from .AgentMsg import AgentMsg
from utils import SupportedCommands



class FlightModeMsg(AgentMsg):
    """
    FlightModeMsg represents messages that should result in a flight mode change on an agent.
    """
    def __init__(self, flight_mode: str, target_system: int, target_comp: int, retry: bool, msg_timeout: float=5.0) -> None:
        supported_flight_modes = SupportedCommands().flight_modes.get_supported_flight_modes()

        if flight_mode not in supported_flight_modes:
            raise TypeError(f'{flight_mode} is not a supported flight mode. Supported flight modes include: {supported_flight_modes}')

        super().__init__('FLIGHT_MODE', target_system, target_comp, retry, msg_timeout=msg_timeout)
        self.flight_mode = flight_mode

        return