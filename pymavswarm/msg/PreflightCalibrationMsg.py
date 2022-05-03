from .AgentMsg import AgentMsg
from .MsgMap import PreflightCalibrationCommands



class PreflightCalibrationMsg(AgentMsg):
    """
    PreflightCalibrationMsg represents messages that perform pre-flight calibrations on a selected
    agent
    """
    def __init__(self, msg_type: str, 
                 target_system: int, 
                 target_comp: int, 
                 retry: bool, 
                 msg_timeout: float=5.0, 
                 ack_timeout: float=1.0,
                 state_timeout: float=5.0,
                 state_delay: float=3.0) -> None:
        super().__init__(msg_type, 
                         target_system, 
                         target_comp, 
                         retry,
                         msg_timeout=msg_timeout,
                         ack_timeout=ack_timeout,
                         state_timeout=state_timeout,
                         state_delay=state_delay)
        
        return