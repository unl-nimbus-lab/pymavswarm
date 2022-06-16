# pymavswarm is an interface for swarm control and interaction
# Copyright (C) 2022  Evan Palmer

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

"""Supported standard message types."""

from typing import List


class FlightModes:
    """Supported flight modes."""

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
        :rtype: List[str]
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
    """Supported system-level commands."""

    arm = "ARM"
    disarm = "DISARM"
    reboot = "REBOOT"
    shutdown = "SHUTDOWN"
    kill = "KILL"

    @staticmethod
    def get_supported_types() -> List[str]:
        """
        Get the supported system commands.

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
    """Supported pre-flight calibration commands."""

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


class FlightSpeedCommand:
    """Supported flight speed configuration commands."""

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
    """Supported mission commands."""

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
    def get_supported_types() -> List[str]:
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


class SupportedCommands:
    """
    Supported command types.

    Provides an interface for accessing the command types that can be sent to agents.
    """

    flight_modes = FlightModes
    system_commands = SystemCommands
    preflight_calibration_commands = PreflightCalibrationCommands
    flight_speed_commands = FlightSpeedCommand
    mission_commands = MissionCommand
