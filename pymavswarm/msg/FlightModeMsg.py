from .AgentMsg import AgentMsg



class FlightModeMsg(AgentMsg):
    """
    FlightModeMsg represents messages that should result in a flight mode change on an agent.
    """
    def __init__(self, msg_type: str, sys_id: int, comp_id: int, check_ack: bool) -> None:
        super().__init__(msg_type, sys_id, comp_id, check_ack)