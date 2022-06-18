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

# type: ignore[no-redef]
# pylint: disable=function-redefined,unused-argument

import logging
import time
from copy import deepcopy
from typing import Any, Callable, Dict, List, Optional, Tuple

from pymavlink import mavutil

import pymavswarm.messages as swarm_messages
import pymavswarm.utils as swarm_utils
from pymavswarm import Connection
from pymavswarm.messages import SupportedCommands as supported_messages
from pymavswarm.messages import responses


class Senders:
    """Methods responsible for sending MAVLink messages."""

    # Message types
    ARM = "ARM"
    DISARM = "DISARM"
    KILL = "KILL"
    REBOOT = "REBOOT"
    SHUTDOWN = "SHUTDOWN"
    ACCELEROMETER_CALIBRATION = "ACCELEROMETER_CALIBRATION"
    SIMPLE_ACCELEROMETER_CALIBRATION = "SIMPLE_ACCELEROMETER_CALIBRATION"
    AHRS_TRIM = "AHRS_TRIM"
    GYROSCOPE_CALIBRATION = "GYROSCOPE_CALIBRATION"
    MAGNETOMETER_CALIBRATION = "MAGNETOMETER_CALIBRATION"
    GROUND_PRESSURE_CALIBRATION = "GROUND_PRESSURE_CALIBRATION"
    AIRSPEED_CALIBRATION = "AIRSPEED_CALIBRATION"
    BAROMETER_TEMPERATURE_CALIBRATION = "BAROMETER_TEMPERATURE_CALIBRATION"
    FLIGHT_MODE = "FLIGHT_MODE"
    FLIGHT_SPEED = "FLIGHT_SPEED"
    SIMPLE_TAKEOFF = "SIMPLE_TAKEOFF"
    TAKEOFF = "TAKEOFF"
    SIMPLE_FULL_TAKEOFF = "SIMPLE_FULL_TAKEOFF"
    FULL_TAKEOFF = "FULL_TAKEOFF"
    SIMPLE_WAYPOINT = "SIMPLE_WAYPOINT"
    WAYPOINT = "WAYPOINT"
    GET_HOME_POSITION = "GET_HOME_POSITION"
    RESET_HOME_TO_CURRENT = "RESET_HOME_TO_CURRENT"
    RESET_HOME = "RESET_HOME"

    def __init__(
        self, logger_name: str = "senders", log_level: int = logging.INFO
    ) -> None:
        """
        Create a new senders object.

        :param logger_name: logger name, defaults to "senders"
        :type logger_name: str, optional

        :param log_level: logging level, defaults to logging.INFO
        :type log_level: int, optional
        """
        self.__logger = swarm_utils.init_logger(logger_name, log_level=log_level)
        self.__senders: Dict[str, List[Callable]] = {}

        @self.__send_message(Senders.ARM)
        @self.__timer()
        def sender(
            message: swarm_messages.SystemCommandMessage,
            connection: Connection,
            function_idx: int = 0,
        ) -> Tuple[bool, Tuple[int, str]]:
            """
            Arm an agent.

            :param message: arming message
            :type message: SystemCommandMessage

            :param connection: MAVLink connection
            :type connection: Connection

            :param function_idx: index of the method in the message type function
                handler list, defaults to 0
            :type function_idx: int, optional

            :return: message send success/fail, message response
            :rtype: bool
            """
            connection.mavlink_connection.mav.command_long_send(
                message.target_system,
                message.target_comp,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                1,
                0,
                0,
                0,
                0,
                0,
                0,
            )

            # Construct a method to use for verifying state change
            def verify_state_changed(message, connection: Connection):
                ack = True
                start_time = time.time()

                while not connection.agents[
                    (message.target_system, message.target_comp)
                ].armed.value:
                    if time.time() - start_time >= message.state_timeout:
                        ack = False
                        break

                return ack

            ack, response = self.__get_message_response(
                message,
                connection,
                function_idx,
                verify_state_changed,
            )

            return ack, response

        @self.__send_message(Senders.DISARM)
        @self.__timer()
        def sender(
            message: swarm_messages.SystemCommandMessage,
            connection: Connection,
            function_idx: int = 0,
        ) -> Tuple[bool, Tuple[int, str]]:
            """
            Disarm an agent.

            :param message: disarm message
            :type message: SystemCommandMessage

            :param connection: MAVLink connection
            :type connection: Connection

            :param function_idx: index of the method in the message type function
                handler list, defaults to 0
            :type function_idx: int, optional

            :return: message send success/fail
            :rtype: bool
            """
            connection.mavlink_connection.mav.command_long_send(
                message.target_system,
                message.target_comp,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
            )

            def verify_state_changed(message, connection: Connection):
                ack = True
                start_time = time.time()

                while connection.agents[
                    (message.target_system, message.target_comp)
                ].armed.value:
                    if time.time() - start_time >= message.state_timeout:
                        ack = False
                        break
                return ack

            ack, response = self.__get_message_response(
                message,
                connection,
                function_idx,
                verify_state_changed,
            )

            return ack, response

        @self.__send_message(Senders.KILL)
        @self.__timer()
        def sender(
            message: swarm_messages.SystemCommandMessage,
            connection: Connection,
            function_idx: int = 0,
        ) -> Tuple[bool, Tuple[int, str]]:
            """
            Force disarm an agent.

            :param message: kill message
            :type message: SystemCommandMessage

            :param connection: MAVLink connection
            :type connection: Connection

            :param function_idx: index of the method in the message type function
                handler list, defaults to 0
            :type function_idx: int, optional

            :return: message send success/fail
            :rtype: bool
            """
            connection.mavlink_connection.mav.command_long_send(
                message.target_system,
                message.target_comp,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                0,
                21196,
                0,
                0,
                0,
                0,
                0,
            )

            ack, response = self.__get_message_response(
                message,
                connection,
                function_idx,
            )

            return ack, response

        @self.__send_message(Senders.REBOOT)
        @self.__timer()
        def sender(
            message: swarm_messages.SystemCommandMessage,
            connection: Connection,
            function_idx: int = 0,
        ) -> Tuple[bool, Tuple[int, str]]:
            """
            Reboot an agent.

            :param message: reboot message
            :type message: SystemCommandMessage

            :param connection: MAVLink connection
            :type connection: Connection

            :param function_idx: index of the method in the message type function
                handler list, defaults to 0
            :type function_idx: int, optional

            :return: message send success/fail
            :rtype: bool
            """
            connection.mavlink_connection.mav.command_long_send(
                message.target_system,
                message.target_comp,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                1,
                0,
                0,
                0,
                0,
                0,
                0,
            )

            ack, response = self.__get_message_response(
                message,
                connection,
                function_idx,
            )

            return ack, response

        @self.__send_message(Senders.SHUTDOWN)
        @self.__timer()
        def sender(
            message: swarm_messages.SystemCommandMessage,
            connection: Connection,
            function_idx: int = 0,
        ) -> Tuple[bool, Tuple[int, str]]:
            """
            Shutdown an agent.

            :param message: shutdown message
            :type message: SystemCommandMessage

            :param connection: MAVLink connection
            :type connection: Connection

            :param function_idx: index of the method in the message type function
                handler list, defaults to 0
            :type function_idx: int, optional

            :return: message send success/fail
            :rtype: bool
            """
            connection.mavlink_connection.mav.command_long_send(
                message.target_system,
                message.target_comp,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                2,
                0,
                0,
                0,
                0,
                0,
                0,
            )

            ack, response = self.__get_message_response(
                message,
                connection,
                function_idx,
            )

            return ack, response

        @self.__send_message(Senders.ACCELEROMETER_CALIBRATION)
        @self.__timer()
        def sender(
            message: swarm_messages.PreflightCalibrationMessage,
            connection: Connection,
            function_idx: int = 0,
        ) -> Tuple[bool, Tuple[int, str]]:
            """
            Perform a full accelerometer calibration on the selected agent.

            :param message: calibration message
            :type message: PreflightCalibrationMessage

            :param connection: MAVLink connection
            :type connection: Connection

            :param function_idx: index of the method in the message type function
                handler list, defaults to 0
            :type function_idx: int, optional

            :return: message send success/fail
            :rtype: bool
            """
            connection.mavlink_connection.mav.command_long_send(
                message.target_system,
                message.target_comp,
                mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
                0,
                0,
                0,
                0,
                0,
                1,
                0,
                0,
            )

            ack, response = self.__get_message_response(
                message,
                connection,
                function_idx,
            )

            return ack, response

        @self.__send_message(Senders.SIMPLE_ACCELEROMETER_CALIBRATION)
        @self.__timer()
        def sender(
            message: swarm_messages.PreflightCalibrationMessage,
            connection: Connection,
            function_idx: int = 0,
        ) -> Tuple[bool, Tuple[int, str]]:
            """
            Perform a simple accelerometer calibration on the selected agent.

            :param message: calibration message
            :type message: PreflightCalibrationMessage

            :param connection: MAVLink connection
            :type connection: Connection

            :param function_idx: index of the method in the message type function
                handler list, defaults to 0
            :type function_idx: int, optional

            :return: message send success/fail
            :rtype: bool
            """
            connection.mavlink_connection.mav.command_long_send(
                message.target_system,
                message.target_comp,
                mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
                0,
                0,
                0,
                0,
                0,
                4,
                0,
                0,
            )

            ack, response = self.__get_message_response(
                message,
                connection,
                function_idx,
            )

            return ack, response

        @self.__send_message(Senders.AHRS_TRIM)
        @self.__timer()
        def sender(
            message: swarm_messages.PreflightCalibrationMessage,
            connection: Connection,
            function_idx: int = 0,
        ) -> Tuple[bool, Tuple[int, str]]:
            """
            Perform an AHRS trim on the selected agent.

            :param message: calibration message
            :type message: PreflightCalibrationMessage

            :param connection: MAVLink connection
            :type connection: Connection

            :param function_idx: index of the method in the message type function
                handler list, defaults to 0
            :type function_idx: int, optional

            :return: message send success/fail
            :rtype: bool
            """
            connection.mavlink_connection.mav.command_long_send(
                message.target_system,
                message.target_comp,
                mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
                0,
                0,
                0,
                0,
                0,
                2,
                0,
                0,
            )

            ack, response = self.__get_message_response(
                message,
                connection,
                function_idx,
            )

            return ack, response

        @self.__send_message(Senders.GYROSCOPE_CALIBRATION)
        @self.__timer()
        def sender(
            message: swarm_messages.PreflightCalibrationMessage,
            connection: Connection,
            function_idx: int = 0,
        ) -> Tuple[bool, Tuple[int, str]]:
            """
            Perform a gyroscope calibration on the selected agent.

            :param message: calibration message
            :type message: PreflightCalibrationMessage

            :param connection: MAVLink connection
            :type connection: Connection

            :param function_idx: index of the method in the message type function
                handler list, defaults to 0
            :type function_idx: int, optional

            :return: message send success/fail
            :rtype: bool
            """
            connection.mavlink_connection.mav.command_long_send(
                message.target_system,
                message.target_comp,
                mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
                0,
                1,
                0,
                0,
                0,
                0,
                0,
                0,
            )

            ack, response = self.__get_message_response(
                message,
                connection,
                function_idx,
            )

            return ack, response

        @self.__send_message(Senders.MAGNETOMETER_CALIBRATION)
        @self.__timer()
        def sender(
            message: swarm_messages.PreflightCalibrationMessage,
            connection: Connection,
            function_idx: int = 0,
        ) -> Tuple[bool, Tuple[int, str]]:
            """
            Perform a magnetometer calibration on the selected agent.

            :param message: calibration message
            :type message: PreflightCalibrationMessage

            :param connection: MAVLink connection
            :type connection: Connection

            :param function_idx: index of the method in the message type function
                handler list, defaults to 0
            :type function_idx: int, optional

            :return: message send success/fail
            :rtype: bool
            """
            connection.mavlink_connection.mav.command_long_send(
                message.target_system,
                message.target_comp,
                mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
                0,
                0,
                1,
                0,
                0,
                0,
                0,
                0,
            )

            ack, response = self.__get_message_response(
                message,
                connection,
                function_idx,
            )

            return ack, response

        @self.__send_message(Senders.GROUND_PRESSURE_CALIBRATION)
        @self.__timer()
        def sender(
            message: swarm_messages.PreflightCalibrationMessage,
            connection: Connection,
            function_idx: int = 0,
        ) -> Tuple[bool, Tuple[int, str]]:
            """
            Perform a ground pressure calibration on the selected agent.

            :param message: calibration message
            :type message: PreflightCalibrationMessage

            :param connection: MAVLink connection
            :type connection: Connection

            :param function_idx: index of the method in the message type function
                handler list, defaults to 0
            :type function_idx: int, optional

            :return: message send success/fail
            :rtype: bool
            """
            connection.mavlink_connection.mav.command_long_send(
                message.target_system,
                message.target_comp,
                mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
                0,
                0,
                0,
                3,
                0,
                0,
                0,
                0,
            )

            ack, response = self.__get_message_response(
                message,
                connection,
                function_idx,
            )

            return ack, response

        @self.__send_message(Senders.AIRSPEED_CALIBRATION)
        @self.__timer()
        def sender(
            message: swarm_messages.PreflightCalibrationMessage,
            connection: Connection,
            function_idx: int = 0,
        ) -> Tuple[bool, Tuple[int, str]]:
            """
            Perform airspeed calibration on the selected agent.

            :param message: calibration message
            :type message: PreflightCalibrationMessage

            :param connection: MAVLink connection
            :type connection: Connection

            :param function_idx: index of the method in the message type function
                handler list, defaults to 0
            :type function_idx: int, optional

            :return: message send success/fail
            :rtype: bool
            """
            connection.mavlink_connection.mav.command_long_send(
                message.target_system,
                message.target_comp,
                mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
                0,
                0,
                0,
                0,
                0,
                0,
                2,
                0,
            )

            ack, response = self.__get_message_response(
                message,
                connection,
                function_idx,
            )

            return ack, response

        @self.__send_message(Senders.BAROMETER_TEMPERATURE_CALIBRATION)
        @self.__timer()
        def sender(
            message: swarm_messages.PreflightCalibrationMessage,
            connection: Connection,
            function_idx: int = 0,
        ) -> Tuple[bool, Tuple[int, str]]:
            """
            Perform a barometer temperature calibration on the selected agent.

            :param message: calibration message
            :type message: PreflightCalibrationMessage

            :param connection: MAVLink connection
            :type connection: Connection

            :param function_idx: index of the method in the message type function
                handler list, defaults to 0
            :type function_idx: int, optional

            :return: message send success/fail
            :rtype: bool
            """
            connection.mavlink_connection.mav.command_long_send(
                message.target_system,
                message.target_comp,
                mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                3,
            )

            ack, response = self.__get_message_response(
                message,
                connection,
                function_idx,
            )

            return ack, response

        @self.__send_message(Senders.FLIGHT_MODE)
        @self.__timer()
        def sender(
            message: swarm_messages.FlightModeMessage,
            connection: Connection,
            function_idx: int = 0,
        ) -> Tuple[bool, Tuple[int, str]]:
            """
            Set the flight mode of an agent.

            :param message: flight mode message
            :type message: FlightModeMessage

            :param connection: MAVLink connection
            :type connection: Connection

            :param function_idx: index of the method in the message type function
                handler list, defaults to 0
            :type function_idx: int, optional

            :return: message send success/fail
            :rtype: bool
            """
            # Reset target
            connection.mavlink_connection.target_system = message.target_system
            connection.mavlink_connection.target_component = message.target_comp

            # Verify that the flight mode is supported by the agent
            # Note that we need to perform this check here because it is dependent on
            # the connection's mode mapping
            if message.flight_mode not in connection.mavlink_connection.mode_mapping():
                return False, responses.INVALID_PROPERTIES

            # Send flight mode
            connection.mavlink_connection.set_mode(
                connection.mavlink_connection.mode_mapping()[message.flight_mode]
            )

            # Construct a method to use for verifying the state change
            def verify_state_changed(message, connection: Connection):
                ack = True
                start_time = time.time()

                while (
                    connection.agents[
                        (message.target_system, message.target_comp)
                    ].flight_mode.value
                    != message.flight_mode
                ):
                    if time.time() - start_time >= message.state_timeout:
                        ack = False
                        break

                return ack

            ack, response = self.__get_message_response(
                message, connection, function_idx, verify_state_changed
            )

            return ack, response

        @self.__send_message(Senders.FLIGHT_SPEED)
        @self.__timer()
        def sender(
            message: swarm_messages.FlightSpeedMessage,
            connection: Connection,
            function_idx: int = 0,
        ) -> Tuple[bool, Tuple[int, str]]:
            """
            Set an agent's flight speed.

            :param message: speed message
            :type message: FlightSpeedMessage

            :param connection: MAVLink connection
            :type connection: Connection

            :param function_idx: index of the method in the message type function
                handler list, defaults to 0
            :type function_idx: int, optional

            :return: message send success/fail
            :rtype: bool
            """
            connection.mavlink_connection.mav.command_long_send(
                message.target_system,
                message.target_comp,
                mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
                0,
                message.speed_type,
                message.speed,
                -1,
                0,
                0,
                0,
                0,
            )

            ack, response = self.__get_message_response(
                message,
                connection,
                function_idx,
            )

            return ack, response

        @self.__send_message(Senders.SIMPLE_TAKEOFF)
        @self.__timer()
        def sender(
            message: swarm_messages.TakeoffMessage,
            connection: Connection,
            function_idx: int = 0,
        ) -> Tuple[bool, Tuple[int, str]]:
            """
            Execute a simple takeoff command (not a full sequence).

            Perform a simple takeoff command (just takeoff to a set altitude)
            Note that acknowledgement of this command does not indicate that the
            altitude was reached, but rather that the system will attempt to reach
            the specified altitude

            :param message: takeoff message
            :type message: TakeoffMessage

            :param connection: MAVLink connection
            :type connection: Connection

            :param function_idx: index of the method in the message type function
                handler list, defaults to 0
            :type function_idx: int, optional

            :return: message send success/fail
            :rtype: bool
            """
            connection.mavlink_connection.mav.command_long_send(
                message.target_system,
                message.target_comp,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                message.altitude,
            )
            ack, response = self.__get_message_response(
                message,
                connection,
                function_idx,
            )

            return ack, response

        @self.__send_message(Senders.TAKEOFF)
        @self.__timer()
        def sender(
            message: swarm_messages.TakeoffMessage,
            connection: Connection,
            function_idx: int = 0,
        ) -> Tuple[bool, Tuple[int, str]]:
            """
            Execute a takeoff command (not a full sequence).

            Perform a takeoff command (use lat, lon, and alt)
            Note that acknowledgement of this command does not indicate that the
            altitude was reached, but rather that the system will attempt to reach
            the specified altitude

            :param message: takeoff message
            :type message: TakeoffMessage

            :param connection: MAVLink connection
            :type connection: Connection

            :param function_idx: index of the method in the message type function
                handler list, defaults to 0
            :type function_idx: int, optional

            :return: message send success/fail
            :rtype: bool
            """
            connection.mavlink_connection.mav.command_long_send(
                message.target_system,
                message.target_comp,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                0,
                0,
                0,
                0,
                0,
                message.latitude,
                message.longitude,
                message.altitude,
            )
            ack, response = self.__get_message_response(
                message,
                connection,
                function_idx,
            )

            return ack, response

        @self.__send_message(Senders.SIMPLE_FULL_TAKEOFF)
        @self.__timer()
        def sender(
            message: swarm_messages.TakeoffMessage,
            connection: Connection,
            function_idx: int = 0,
        ) -> Tuple[bool, Tuple[int, str]]:
            """
            Execute a simple full takeoff command sequence.

            Command used to signal execution of a full simple takeoff sequence:
                1. Switch to GUIDED mode
                2. Arm
                3. Takeoff

            :param message: takeoff sequence message
            :type message: TakeoffMessage

            :param connection: MAVLink connection
            :type connection: Connection

            :param function_idx: index of the method in the message type function
                handler list, defaults to 0
            :type function_idx: int, optional

            :return: message send success/fail
            :rtype: bool
            """
            # Create a new guided mode
            guided_message = swarm_messages.FlightModeMessage(
                supported_messages.flight_modes.guided,
                message.target_system,
                message.target_comp,
                message.retry,
                message.message_timeout,
                ack_timeout=message.ack_timeout,
                state_timeout=message.state_timeout,
                state_delay=message.state_delay,
            )

            # Attempt to switch to GUIDED mode
            if not self.__send_sequence_message(guided_message, connection):
                return False, responses.SEQUENCE_STAGE_FAILURE

            # Create a new arming message to send
            arm_message = swarm_messages.SystemCommandMessage(
                supported_messages.system_commands.arm,
                message.target_system,
                message.target_comp,
                message.retry,
                message.message_timeout,
                ack_timeout=message.ack_timeout,
                state_timeout=message.state_timeout,
                state_delay=message.state_delay,
            )

            # Attempt to arm the system
            if not self.__send_sequence_message(arm_message, connection):
                return False, responses.SEQUENCE_STAGE_FAILURE

            # Give the agent a chance to fully arm
            time.sleep(message.state_delay)

            # Reset the message type to be a simple takeoff command
            message.message_type = supported_messages.mission_commands.simple_takeoff

            # Attempt to perform takeoff
            if not self.__send_sequence_message(message, connection):
                return False, responses.SEQUENCE_STAGE_FAILURE

            return True, responses.SUCCESS

        @self.__send_message(Senders.FULL_TAKEOFF)
        @self.__timer()
        def sender(
            message: swarm_messages.TakeoffMessage,
            connection: Connection,
            function_idx: int = 0,
        ) -> Tuple[bool, Tuple[int, str]]:
            """
            Execute a full takeoff sequence.

            Command used to signal execution of a full takeoff sequence:
                1. Switch to GUIDED mode
                2. Arm
                3. Takeoff

            :param message: takeoff message
            :type message: TakeoffMessage

            :param connection: MAVLink connection
            :type connection: Connection

            :param function_idx: index of the method in the message type function
                handler list, defaults to 0
            :type function_idx: int, optional

            :return: message send success/fail
            :rtype: bool
            """
            # Create a new guided mode
            guided_message = swarm_messages.FlightModeMessage(
                supported_messages.flight_modes.guided,
                message.target_system,
                message.target_comp,
                message.retry,
                message.message_timeout,
                ack_timeout=message.ack_timeout,
                state_timeout=message.state_timeout,
                state_delay=message.state_delay,
            )

            # Attempt to switch to GUIDED mode
            if not self.__send_sequence_message(guided_message, connection):
                return False, responses.SEQUENCE_STAGE_FAILURE

            # Create a new arming message to send
            arm_message = swarm_messages.SystemCommandMessage(
                supported_messages.system_commands.arm,
                message.target_system,
                message.target_comp,
                message.retry,
                message.message_timeout,
                ack_timeout=message.ack_timeout,
                state_timeout=message.state_timeout,
                state_delay=message.state_delay,
            )

            # Attempt to arm the system
            if not self.__send_sequence_message(arm_message, connection):
                return False, responses.SEQUENCE_STAGE_FAILURE

            # Give the agent a chance to fully arm
            time.sleep(message.state_delay)

            # Reset the message type to be a simple takeoff command
            message.message_type = supported_messages.mission_commands.takeoff

            # Attempt to perform takeoff
            if not self.__send_sequence_message(message, connection):
                return False, responses.SEQUENCE_STAGE_FAILURE

            return True, responses.SUCCESS

        @self.__send_message(Senders.SIMPLE_WAYPOINT)
        @self.__timer()
        def sender(
            message: swarm_messages.WaypointMessage,
            connection: Connection,
            function_idx: int = 0,
        ) -> Tuple[bool, Tuple[int, str]]:
            """
            Perform a simple waypoint command (just lat, lon, and alt).

            Acknowledgement of this command does not indicate that the
            waypoint was reached, but rather that the system will attempt to reach
            the specified waypoint.

            :param message: waypoint message
            :type message: WaypointMessage

            :param connection: MAVLink connection
            :type connection: Connection

            :param function_idx: index of the method in the message type function
                handler list, defaults to 0
            :type function_idx: int, optional

            :return: message send success/fail
            :rtype: bool
            """
            connection.mavlink_connection.mav.mission_item_send(
                message.target_system,
                message.target_comp,
                0,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0,
                0,
                0,
                0,
                0,
                0,
                message.latitude,
                message.longitude,
                message.altitude,
            )

            ack, response = self.__get_message_response(
                message,
                connection,
                function_idx,
            )

            return ack, response

        @self.__send_message(Senders.WAYPOINT)
        @self.__timer()
        def sender(
            message: swarm_messages.WaypointMessage,
            connection: Connection,
            function_idx: int = 0,
        ) -> Tuple[bool, Tuple[int, str]]:
            """
            Perform a waypoint navigation command.

            Acknowledgement of this command does not indicate that the
            waypoint was reached, but rather that the system will attempt to reach
            the specified waypoint.

            :param message: waypoint message
            :type message: WaypointMessage

            :param connection: MAVLink connection
            :type connection: Connection

            :param function_idx: index of the method in the message type function
                handler list, defaults to 0
            :type function_idx: int, optional

            :return: message send success/fail
            :rtype: bool
            """
            connection.mavlink_connection.mav.mission_item_send(
                message.target_system,
                message.target_comp,
                0,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0,
                0,
                message.hold,
                message.accept_radius,
                message.pass_radius,
                message.yaw,
                message.latitude,
                message.longitude,
                message.altitude,
            )

            ack, response = self.__get_message_response(
                message,
                connection,
                function_idx,
            )

            return ack, response

        @self.__send_message(Senders.GET_HOME_POSITION)
        @self.__timer()
        def sender(
            message: swarm_messages.AgentCommand,
            connection: Connection,
            function_idx: int = 0,
        ) -> Tuple[bool, Tuple[int, str]]:
            """
            Get the current home position of an agent.

            :param message: home position request message
            :type message: AgentMessage

            :param function_idx: index of the method in the message type function
                handler list, defaults to 0
            :type function_idx: int, optional

            :return: message send success/fail
            :rtype: bool
            """
            connection.mavlink_connection.mav.command_long_send(
                message.target_system,
                message.target_comp,
                mavutil.mavlink.MAV_CMD_GET_HOME_POSITION,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
            )

            ack, response = self.__get_message_response(
                message,
                connection,
                function_idx,
            )

            return ack, response

        @self.__send_message(Senders.RESET_HOME_TO_CURRENT)
        @self.__timer()
        def sender(
            message: swarm_messages.HomePositionMessage,
            connection: Connection,
            function_idx: int = 0,
        ) -> Tuple[bool, Tuple[int, str]]:
            """
            Reset the saved home position of an agent to the current position.

            Validation of this command can take a while. To ensure that the system
            has sufficient time to verify that the home position was properly reset, it
            may be necessary to extend the timeout periods.

            :param message: reset home position message
            :type message: HomePositionMessage

            :param connection: MAVLink connection
            :type connection: Connection

            :param function_idx: index of the method in the message type function
                handler list, defaults to 0
            :type function_idx: int, optional

            :return: message send success/fail
            :rtype: bool
            """
            agent_id = (message.target_system, message.target_comp)

            if agent_id in connection.agents:
                current_home_pos = connection.agents[agent_id].home_position

            # Set the home position to the current location
            connection.mavlink_connection.mav.command_long_send(
                message.target_system,
                message.target_comp,
                mavutil.mavlink.MAV_CMD_DO_SET_HOME,
                0,
                1,
                0,
                0,
                0,
                0,
                0,
                0,
            )

            def verify_state_changed(message, connection: Connection):
                ack = True
                start_time = time.time()

                while vars(connection.agents[agent_id].home_position) == vars(
                    current_home_pos
                ):
                    # Signal an update state command
                    update_home_state_message = swarm_messages.AgentCommand(
                        supported_messages.mission_commands.get_home_position,
                        message.target_system,
                        message.target_comp,
                        message.retry,
                        message.message_timeout,
                        message.ack_timeout,
                        message.state_timeout,
                        message.state_delay,
                    )

                    # Get the updated home position
                    self.__send_sequence_message(update_home_state_message, connection)

                    if time.time() - start_time >= message.state_timeout:
                        ack = False
                        break

                return ack

            ack, response = self.__get_message_response(
                message, connection, function_idx, verify_state_changed
            )

            return ack, response

        @self.__send_message(Senders.RESET_HOME)
        @self.__timer()
        def sender(
            message: swarm_messages.HomePositionMessage,
            connection: Connection,
            function_idx: int = 0,
        ) -> Tuple[bool, Tuple[int, str]]:
            """
            Reset the saved home position of an agent to the desired position.

            :param message: reset home position message
            :type message: HomePositionMessage

            :param connection: MAVLink connection
            :type connection: Connection

            :param function_idx: index of the method in the message type function
                handler list, defaults to 0
            :type function_idx: int, optional

            :return: message send success/fail
            :rtype: bool
            """
            connection.mavlink_connection.mav.command_long_send(
                message.target_system,
                message.target_comp,
                mavutil.mavlink.MAV_CMD_DO_SET_HOME,
                0,
                0,
                0,
                0,
                0,
                message.latitude,
                message.longitude,
                message.altitude,
            )

            def verify_state_changed(message, connection: Connection):
                agent_id = (message.target_system, message.target_comp)
                ack = True
                start_time = time.time()

                while (
                    connection.agents[agent_id].home_position.latitude
                    != message.latitude
                    and connection.agents[agent_id].home_position.longitude
                    != message.longitude
                    and connection.agents[agent_id].home_position.altitude
                    != message.altitude
                ):
                    # Signal an update state command
                    update_home_state_message = swarm_messages.AgentCommand(
                        supported_messages.mission_commands.get_home_position,
                        message.target_system,
                        message.target_comp,
                        message.retry,
                        message.message_timeout,
                        message.ack_timeout,
                        message.state_timeout,
                        message.state_delay,
                    )

                    # Get the updated home position
                    self.__send_sequence_message(update_home_state_message, connection)

                    if time.time() - start_time >= message.state_timeout:
                        ack = False
                        break

                return ack

            ack, response = self.__get_message_response(
                message, connection, function_idx, verify_state_changed
            )

            return ack, response

    @property
    def senders(self) -> dict:
        """
        Get the methods responsible for sending messages.

        :return: list of senders
        :rtype: dict
        """
        return self.__senders

    def __get_message_response(
        self,
        message: Any,
        connection: Connection,
        function_idx: int,
        state_verification_function: Optional[Callable] = None,
    ) -> Tuple[bool, Tuple[int, str]]:
        """
        Verify the result of a message and retry sending the message, if desired.

        :param message: message whose response should be captured
        :type message: Any

        :param connection: MAVLink connection
        :type connection: Connection

        :param function_idx: index of the function to use when sending the message
        :type function_idx: int

        :param state_verification_function: function used to verify that the target
            state was properly modified, defaults to None
        :type state_verification_function: Optional[Callable], optional

        :return: message acknowledgement, message response
        :rtype: Tuple[bool, Tuple[int, str]]
        """
        ack = False
        response = responses.ACK_FAILURE

        if swarm_utils.ack_message(
            "COMMAND_ACK", connection, timeout=message.ack_timeout
        ):
            ack = True
            response = responses.SUCCESS

            if (
                message.target_system,
                message.target_comp,
            ) in connection.agents and state_verification_function is not None:
                ack = state_verification_function(message, connection)

                if not ack:
                    response = responses.STATE_VALIDATION_FAILURE
        else:
            response = responses.ACK_FAILURE

        if message.retry and not ack:
            ack, response = self.__retry_message_send(
                deepcopy(message),
                connection,
                self.__senders[message.message_type][function_idx],
            )

        return ack, response

    def __retry_message_send(
        self,
        message: Any,
        connection: Connection,
        function: Callable,
    ) -> Tuple[bool, Tuple[int, str]]:
        """
        Retry a message send until success/timeout.

        :param message: message to retry sending
        :type message: pymavswarm message

        :param function: function to call using the message
        :type function: Callable

        :return: message acknowledged, message response
        :rtype: Tuple[bool, Tuple[int, str]]
        """
        ack = False
        start_time = time.time()

        # Don't let the message come back here and create an infinite loop
        message.retry = False

        while time.time() - start_time <= message.message_timeout:
            ack, response = function(self, message, connection)

            if ack:
                break

        return ack, response

    def __send_sequence_message(self, message: Any, connection: Connection) -> bool:
        """
        Send a sequence message.

        Helper function used to handle calling all of the handlers for a message.
        This method is used by sequence commands (such as the full takeoff
        command) to provide indication of a function execution result.

        :param message: The message to send
        :type message: Any

        :return: Indicates whether all of the message senders for a given message
            successfully sent their respective message
        :rtype: bool
        """
        if message.message_type not in self.__senders:
            return False

        for function_idx, function in enumerate(self.__senders[message.message_type]):
            try:
                success, _ = function(
                    self,
                    message,
                    connection,
                    function_idx=function_idx,
                )

                if not success:
                    return False

            except Exception:
                return False

        return True

    def __send_message(self, message: str) -> Callable:
        """
        Create a sender for a mavlink message.

        :param message: The message type to connect to the sender
        :type message: Union[list, str]

        :return: decorator
        :rtype: Callable
        """

        def decorator(function: Callable):
            if message not in self.__senders:
                self.__senders[message] = []

            if function not in self.__senders[message]:
                self.__senders[message].append(function)

        return decorator

    def __timer(self) -> Callable:
        """
        Log the time that a sender takes to complete. Used for debugging purposes.

        :return: decorator
        :rtype: Callable
        """

        def decorator(function: Callable) -> Callable:
            def wrapper(*args):
                start_t = time.time()
                response = function(*args)
                self.__logger.debug(
                    f"Time taken to execute function: {time.time() - start_t}s"
                )
                return response

            return wrapper

        return decorator
