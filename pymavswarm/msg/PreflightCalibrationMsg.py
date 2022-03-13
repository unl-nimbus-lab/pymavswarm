from typing import Type
from .AgentMsg import AgentMsg
from utils import SupportedCommands



class PreflightCalibrationMsg(AgentMsg):
    """
    PreflightCalibrationMsg represents messages that perform pre-flight calibrations on a selected
    agent
    """
    def __init__(self, calibration_type: int, target_system: int, target_comp: int, retry: bool, msg_timeout: float=5.0) -> None:
        supported_calibration_types = SupportedCommands().preflight_calibration_commands.get_supported_preflight_calibration_commands()

        if calibration_type not in supported_calibration_types:
            raise TypeError(f'{calibration_type} is not a supported calibration type. Supported pre-flight calibration types include {supported_calibration_types}')

        super().__init__(calibration_type, target_system, target_comp, retry, msg_timeout=msg_timeout)
        
        return