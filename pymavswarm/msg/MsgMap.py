from dataclasses import dataclass


@dataclass
class FlightModes:
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


@dataclass
class SystemCommands:
    """
    Enum class used to enable easy retrieval of the command used by a connection to 
    send a respective system command message
    """
    arm = 'arm'
    disarm = 'disarm'
    reboot = 'reboot'
    shutdown = 'shutdown'
    kill = 'kill'


@dataclass
class PreflightCalibrationCommands:
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


@dataclass
class HRLCommands:
    """
    Enum class used to enable easy retrieval of the command used to send HRL commands
    """
    start_path_execution = 'startpath'
    reset_path_execution = 'resetpath'
    stop_path_execution = 'stoppath'
    start_live_execution = 'startlive'


@dataclass
class FlightSpeedCommand:
    """
    Enum class used to enable retrieval of the command used to send flight speed change requests
    """
    air_speed = 'airspeed'
    ground_speed = 'groundspeed'
    climb_speed = 'climbspeed'
    descent_speed = 'descentspeed'


@dataclass
class MissionCommand:
    """
    Enum class used to retrieve mission commands
    """
    simple_waypoint = 'simplewaypoint'
    waypoint = 'waypoint'
    simple_takeoff = 'simpletakeoff'
    takeoff = 'takeoff'
    reset_home_position_to_current = 'resethomecurrent'
    reset_home_position = 'resethome'


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
