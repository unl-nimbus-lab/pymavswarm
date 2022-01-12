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
    reboot = 'reboot'
    shutdown = 'shutdown'
    kill = 'kill'


class PreflightCalibrationCommands(Enum):
    """
    Enum class used to enable sending preflight calibration commands
    """
    gyro_cal = 'gyrocal'
    magnetometer_cal = 'magnetometercal'
    ground_pressure_cal = 'groundpressurecal'
    airspeed_cal = 'airspeedcal'
    barometer_temp_cal = 'barotempcal'
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
    climb_speed = 'climbspeed'
    descent_speed = 'descentspeed'


class MissionCommand(Enum):
    """
    Enum class used to retrieve mission commands
    """
    simple_waypoint = 'simplewaypoint'
    waypoint = 'waypoint'
    simple_takeoff = 'simpletakeoff'
    takeoff = 'takeoff'


class MsgMap:
    """
    Helper class used to provide easy retrieval of the commands used by pymavswarm to send
    messages
    """
    def __init__(self) -> None:
        self.flight_modes = FlightModes
        self.system_commands = SystemCommands
        self.preflight_calibration_commands = PreflightCalibrationCommands
        self.hrl_commands = HRLCommands
        self.flight_speed_commands = FlightSpeedCommand
        self.mission_commands = MissionCommand

        return
