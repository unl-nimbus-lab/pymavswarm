from .AgentMsg import AgentMsg



class SystemCommandMsg(AgentMsg):
    """
    SystemCommandMsg class represents messages that perform some system-level operation on
    an agent such as arming or reboot.
    """
    def __init__(self, msg_type: str, sys_id: int, comp_id: int, check_ack: bool, msg_timeout: float=5.0) -> None:
        super().__init__(msg_type, sys_id, comp_id, check_ack, msg_timeout=msg_timeout)

        return