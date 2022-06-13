from typing import List


class FlightModes:
    """
    Supported flight modes.
    """

    stabilize = "STABILIZE"
    acro = "ACRO"
    alt_hold = "ALT_HOLD"
    auto = "AUTO"
    loiter = "LOITER"
    rtl = "RTL"
    land = "LAND"
    throw = "THROW"
    systemid = "SYSTEMID"
    guided = "GUIDED"

    @staticmethod
    def get_supported_types() -> List[str]:
        """
        Get the supported flight modes.

        :return: supported flight modes
        :rtype: _type_
        """
        return [
            FlightModes.stabilize,
            FlightModes.acro,
            FlightModes.alt_hold,
            FlightModes.auto,
            FlightModes.loiter,
            FlightModes.rtl,
            FlightModes.land,
            FlightModes.throw,
            FlightModes.systemid,
            FlightModes.guided,
        ]


class SystemCommands:
    """
    Supported system-level commands.
    """

    arm = "ARM"
    disarm = "DISARM"
    reboot = "REBOOT"
    shutdown = "SHUTDOWN"
    kill = "KILL"

    @staticmethod
    def get_supported_types() -> List[str]:
        """
        Get the supported system commands

        :return: supported system commands
        :rtype: List[str]
        """
        return [
            SystemCommands.arm,
            SystemCommands.disarm,
            SystemCommands.reboot,
            SystemCommands.shutdown,
            SystemCommands.kill,
        ]


class PreflightCalibrationCommands:
    """
    Supported pre-flight calibration commands
    """

    gyro_cal = "GYROSCOPE_CALIBRATION"
    magnetometer_cal = "MAGNETOMETER_CALIBRATION"
    ground_pressure_cal = "GROUND_PRESSURE_CALIBRATION"
    airspeed_cal = "AIRSPEED_CALIBRATION"
    barometer_temp_cal = "BAROMETER_TEMPERATURE_CALIBRATION"
    accel_cal = "ACCELEROMETER_CALIBRATION"
    accel_cal_simple = "SIMPLE_ACCELEROMETER_CALIBRATION"
    ahrs_trim = "AHRS_TRIM"

    @staticmethod
    def get_supported_types() -> List[str]:
        """
        Get the supported pre-flight calibration commands.

        :return: supported pre-flight calibration commands
        :rtype: List[str]
        """
        return [
            PreflightCalibrationCommands.gyro_cal,
            PreflightCalibrationCommands.magnetometer_cal,
            PreflightCalibrationCommands.ground_pressure_cal,
            PreflightCalibrationCommands.airspeed_cal,
            PreflightCalibrationCommands.barometer_temp_cal,
            PreflightCalibrationCommands.accel_cal,
            PreflightCalibrationCommands.accel_cal_simple,
            PreflightCalibrationCommands.ahrs_trim,
        ]


class HRLCommands:
    """
    Supported HRL commands.
    """

    start_path_execution = 0
    reset_path_execution = 1
    stop_path_execution = 2
    start_live_execution = 3

    @staticmethod
    def get_supported_types() -> List[int]:
        """
        Get the supported HRL commands.

        :return: supported HRL commands
        :rtype: List[int]
        """
        return [
            HRLCommands.start_path_execution,
            HRLCommands.reset_path_execution,
            HRLCommands.stop_path_execution,
            HRLCommands.start_live_execution,
        ]


class FlightSpeedCommand:
    """
    Supported flight speed configuration commands.
    """

    air_speed = 0
    ground_speed = 1
    climb_speed = 2
    descent_speed = 3

    @staticmethod
    def get_supported_types() -> List[int]:
        """
        Get the supported flight speed configuration commands.

        :return: supported flight speed configuration commands
        :rtype: List[int]
        """
        return [
            FlightSpeedCommand.air_speed,
            FlightSpeedCommand.ground_speed,
            FlightSpeedCommand.climb_speed,
            FlightSpeedCommand.descent_speed,
        ]


class MissionCommand:
    """
    Supported mission commands.
    """

    simple_waypoint = "SIMPLE_WAYPOINT"
    waypoint = "WAYPOINT"
    simple_takeoff = "SIMPLE_TAKEOFF"
    takeoff = "TAKEOFF"
    full_takeoff = "FULL_TAKEOFF"
    full_simple_takeoff = "SIMPLE_FULL_TAKEOFF"
    reset_home_position_to_current = "RESET_HOME_TO_CURRENT"
    reset_home_position = "RESET_HOME"
    get_home_position = "GET_HOME_POSITION"

    @staticmethod
    def get_supported_types() -> List[int]:
        """
        Get the supported mission commands.

        :return: supported mission commands
        :rtype: List[int]
        """
        return [
            MissionCommand.simple_waypoint,
            MissionCommand.waypoint,
            MissionCommand.simple_takeoff,
            MissionCommand.takeoff,
            MissionCommand.full_takeoff,
            MissionCommand.full_simple_takeoff,
            MissionCommand.reset_home_position,
            MissionCommand.reset_home_position_to_current,
            MissionCommand.get_home_position,
        ]


class SupportedMessages:
    """
    Helper class used to provide easy retrieval of the commands used by pymavswarm to
    send messages.
    """

    flight_modes = FlightModes
    system_commands = SystemCommands
    preflight_calibration_commands = PreflightCalibrationCommands
    hrl_commands = HRLCommands
    flight_speed_commands = FlightSpeedCommand
    mission_commands = MissionCommand
