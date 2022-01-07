from enum import Enum

class FlightModes(Enum):
    """
    Enum class used to enable easy retrieval of the command used by a connection to 
    send a respective flight mode message
    """
    stabilize = 'stabilize'
    acro = 'acro'
    alt_hold = 'althold'
    auto = 'auto'
    loiter = 'loiter'
    rtl = 'rtl'
    land = 'land'
    throw = 'throw'
    systemid = 'systemid'
    guided = 'guided'


class SystemCommands(Enum):
    """
    Enum class used to enable easy retrieval of the command used by a connection to 
    send a respective system command message
    """
    arm = 'arm'
    disarm = 'disarm'
    accel_cal = 'accelcal'
    accel_cal_simple = 'accelcalsimple'
    ahrs_trim = 'ahrstrim'


class HRLCommands(Enum):
    """
    Enum class used to enable easy retrieval of the command used to send HRL commands
    """
    start_path_execution = 'startpath'
    stop_path_execution = 'stoppath'


class FlightSpeedCommand(Enum):
    """
    Enum class used to enable retrieval of the command used to send flight speed change requests
    """
    air_speed = 'airspeed'
    ground_speed = 'groundspeed'


class MsgMap(Enum):
    """
    Helper class used to provide easy retrieval of the commands used by pymavswarm to send
    messages
    """
    flight_modes = FlightModes
    system_commands = SystemCommands
    hrl_commands = HRLCommands
