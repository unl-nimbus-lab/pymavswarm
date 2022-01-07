from enum import Enum

class CommandTypes(Enum):
    """
    Enum class used to encompass the command types that may be sent by pymavswarm
    """
    hrl_command = 0
    arming_command = 1
    flight_mode_command = 2
    preflight_calibration_command = 3