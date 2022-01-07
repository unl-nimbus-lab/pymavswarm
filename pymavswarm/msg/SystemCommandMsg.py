from .AgentMsg import AgentMsg



class SystemCommandMsg(AgentMsg):
    """
    SystemCommandMsg class represents messages that perform some system-level operation on
    an agent such as arming or pre-flight calibration. The message types that should be used
    with this type include:
        - arm
        - disarm
        - accel_cal
        - accel_cal_simple
        - ahrs_trim
    """
    def __init__(self, msg_type: str, sys_id: int, comp_id: int, check_ack: bool) -> None:
        super().__init__(msg_type, sys_id, comp_id, check_ack)