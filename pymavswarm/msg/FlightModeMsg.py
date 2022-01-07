from .AgentMsg import AgentMsg



class FlightModeMsg(AgentMsg):
    """
    FlightModeMsg represents messages that should result in a flight mode change on an agent.
    The message types that should be used with this type include:
        - stabilize
        - acro
        - alt_hold
        - auto
        - loiter
        - rtl
        - land
        - throw
        - systemid
        - guided
    """
    def __init__(self, msg_type: str, sys_id: int, comp_id: int, check_ack: bool) -> None:
        super().__init__(msg_type, sys_id, comp_id, check_ack)