from .AgentMsg import AgentMsg



class FlightModeMsg(AgentMsg):
    """
    FlightModeMsg represents messages that should result in a flight mode change on an agent.
    """
    def __init__(self, msg_type: str, target_system: int, target_comp: int, retry: bool, msg_timeout: float=5.0) -> None:
        super().__init__(msg_type, target_system, target_comp, retry, msg_timeout=msg_timeout)

        return