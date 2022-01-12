from .AgentMsg import AgentMsg



class HRLMsg(AgentMsg):
    """
    HRLMsg class represents messages that indicate that an HRL command should be executed.
    The message types that should be used with this type include:
        - start_path_execution
        - stop_path_execution
    """
    def __init__(self, msg_type: str, sys_id: int, comp_id: int, retry: bool, msg_timeout: float=5.0) -> None:
        super().__init__(msg_type, sys_id, comp_id, retry, msg_timeout=msg_timeout)

        return