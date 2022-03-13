class SupportedFlightModes:
    """
    Class containing the supported flight modes
    """
    def __init__(self) -> None:
        self.stabilize = 'STABILIZE'
        self.acro = 'ACRO'
        self.alt_hold = 'ALT_HOLD'
        self.auto = 'AUTO'
        self.loiter = 'LOITER'
        self.rtl = 'RTL'
        self.land = 'LAND'
        self.throw = 'THROW'
        self.systemid = 'SYSTEMID'
        self.guided = 'GUIDED'


    def get_supported_flight_modes(self) -> list:
        return [
            self.stabilize,
            self.acro,
            self.alt_hold,
            self.auto,
            self.loiter,
            self.rtl,
            self.land,
            self.throw,
            self.systemid,
            self.guided
        ]



class SupportedSystemCommands:
    """
    Class containing all supported system commands
    """
    def __init__(self) -> None:
        self.arm = 'ARM'
        self.disarm = 'DISARM'
        self.reboot = 'REBOOT'
        self.shutdown = 'SHUTDOWN'
        self.kill = 'KILL'

    
    def get_supported_system_commands(self) -> list:
        return [
            self.arm,
            self.disarm,
            self.reboot,
            self.shutdown,
            self.kill
        ]



class SupportedPreflightCalibrationCommands:
    """
    Class containing all suppported preflight calibration commands
    """
    def __init__(self) -> None:
        self.gyro_cal = 'GYROSCOPE_CALIBRATION'
        self.magnetometer_cal = 'MAGNETOMETER_CALIBRATION'
        self.ground_pressure_cal = 'GROUND_PRESSURE_CALIBRATION'
        self.airspeed_cal = 'AIRSPEED_CALIBRATION'
        self.barometer_temp_cal = 'BAROMETER_TEMPERATURE_CALIBRATION'
        self.accel_cal = 'ACCELEROMETER_CALIBRATION'
        self.accel_cal_simple = 'SIMPLE_ACCELEROMETER_CALIBRATION'
        self.ahrs_trim = 'AHRS_TRIM'

    
    def get_supported_preflight_calibration_commands(self) -> list:
        return [
            self.gyro_cal,
            self.magnetometer_cal,
            self.ground_pressure_cal,
            self.airspeed_cal,
            self.barometer_temp_cal,
            self.accel_cal,
            self.accel_cal_simple,
            self.ahrs_trim        
        ]



class SupportedHRLCommands:
    """
    Class containing all supported HRL commands
    """
    def __init__(self) -> None:
        self.start_path_execution = 0
        self.reset_path_execution = 1
        self.stop_path_execution = 2
        self.start_live_execution = 3


    def get_supported_hrl_commands(self) -> list:
        return [
            self.start_path_execution,
            self.reset_path_execution,
            self.stop_path_execution,
            self.start_live_execution
        ]



class SupportedFlightSpeedCommands:
    """
    Class containing all supported flight speed commands
    """
    def __init__(self) -> None:
        self.air_speed = 0
        self.ground_speed = 1
        self.climb_speed = 2
        self.descent_speed = 3

    
    def get_supported_flight_speed_commands(self) -> list:
        return [
            self.air_speed,
            self.ground_speed,
            self.climb_speed,
            self.descent_speed
        ]



class SupportedMissionCommands:
    """
    Class containing all supported mission commands
    """
    def __init__(self) -> None:
        self.waypoint = 'WAYPOINT'
        self.takeoff = 'TAKEOFF'
        self.reset_home_position = 'RESET_HOME_POSITION'

    
    def get_supported_mission_commands(self) -> list:
        return [
            self.waypoint,
            self.takeoff,
            self.reset_home_position
        ]



class SupportedCommands:
    """
    Helper class used to provide easy retrieval of the commands used by pymavswarm to send
    messages
    """
    def __init__(self) -> None:
        self.flight_modes = SupportedFlightModes()
        self.system_commands = SupportedSystemCommands()
        self.preflight_calibration_commands = SupportedPreflightCalibrationCommands()
        self.hrl_commands = SupportedHRLCommands()
        self.flight_speed_commands = SupportedFlightSpeedCommands()
        self.mission_commands = SupportedMissionCommands()

        return
