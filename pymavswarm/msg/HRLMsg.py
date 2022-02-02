from .AgentMsg import AgentMsg
from .MsgMap import HRLCommands



class HRLMsg(AgentMsg):
    """
    HRLMsg class represents messages that indicate that an HRL command should be executed.
    The message types that should be used with this type include:
        - start_path_execution
        - stop_path_execution
    """
    def __init__(self, msg_type: str, target_system: int, target_comp: int, retry: bool, msg_timeout: float=5.0) -> None:
        assert msg_type in HRLCommands, f'Attempted to generate an invalid HRL Command message: {msg_type}'

        super().__init__(msg_type, target_system, target_comp, retry, msg_timeout=msg_timeout)

        return