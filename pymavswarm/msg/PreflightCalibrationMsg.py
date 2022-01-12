from .AgentMsg import AgentMsg



class PreflightCalibrationMsg(AgentMsg):
    """
    PreflightCalibrationMsg represents messages that perform pre-flight calibrations on a selected
    agent
    """
    def __init__(self, msg_type: str, sys_id: int, comp_id: int, retry: bool, msg_timeout: float=5.0) -> None:
        super().__init__(msg_type, sys_id, comp_id, retry, msg_timeout=msg_timeout)
        
        return