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
from typing import Any, Tuple

from pymavlink import mavutil

import pymavswarm.messages as swarm_messages
import pymavswarm.state as swarm_state
from pymavswarm import Connection
from pymavswarm.handlers.senders import Senders
from pymavswarm.messages import responses


class MessageSenders(Senders):
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
    TAKEOFF = "TAKEOFF"
    TAKEOFF_SEQUENCE = "TAKEOFF_SEQUENCE"
    WAYPOINT = "WAYPOINT"
    GET_HOME_POSITION = "GET_HOME_POSITION"
    RESET_HOME_POSITION = "RESET_HOME_POSITION"
    RESET_HOME_TO_CURRENT = "RESET_HOME_TO_CURRENT"
    READ_PARAMETER = "READ_PARAMETER"
    SET_PARAMETER = "SET_PARAMETER"

    def __init__(
        self, logger_name: str = "message-senders", log_level: int = logging.INFO
    ) -> None:
        """
        Create a new message senders object.

        :param logger_name: logger name, defaults to "message-senders"
        :type logger_name: str, optional

        :param log_level: logging level, defaults to logging.INFO
        :type log_level: int, optional
        """
        super().__init__(logger_name, log_level)

        @self._send_message(MessageSenders.ARM)
        @self._timer()
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
            :rtype: Tuple[bool, Tuple[int, str]]
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

            ack, response, _ = self._get_message_response(
                message,
                connection,
                function_idx,
                verify_state_changed,
            )

            return ack, response

        @self._send_message(MessageSenders.DISARM)
        @self._timer()
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

            :return: message send success/fail, message response
            :rtype: Tuple[bool, Tuple[int, str]]
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

            def verify_state_changed(message: Any, connection: Connection):
                ack = True
                start_time = time.time()

                while connection.agents[
                    (message.target_system, message.target_comp)
                ].armed.value:
                    if time.time() - start_time >= message.state_timeout:
                        ack = False
                        break
                return ack

            ack, response, _ = self._get_message_response(
                message,
                connection,
                function_idx,
                verify_state_changed,
            )

            return ack, response

        @self._send_message(MessageSenders.KILL)
        @self._timer()
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

            :return: message send success/fail, message response
            :rtype: Tuple[bool, Tuple[int, str]]
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

            ack, response, _ = self._get_message_response(
                message,
                connection,
                function_idx,
            )

            return ack, response

        @self._send_message(MessageSenders.REBOOT)
        @self._timer()
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

            :return: message send success/fail, message response
            :rtype: Tuple[bool, Tuple[int, str]]
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

            ack, response, _ = self._get_message_response(
                message,
                connection,
                function_idx,
            )

            return ack, response

        @self._send_message(MessageSenders.SHUTDOWN)
        @self._timer()
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

            :return: message send success/fail, message response
            :rtype: Tuple[bool, Tuple[int, str]]
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

            ack, response, _ = self._get_message_response(
                message,
                connection,
                function_idx,
            )

            return ack, response

        @self._send_message(MessageSenders.ACCELEROMETER_CALIBRATION)
        @self._timer()
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

            :return: message send success/fail, message response
            :rtype: Tuple[bool, Tuple[int, str]]
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

            ack, response, _ = self._get_message_response(
                message,
                connection,
                function_idx,
            )

            return ack, response

        @self._send_message(MessageSenders.SIMPLE_ACCELEROMETER_CALIBRATION)
        @self._timer()
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

            :return: message send success/fail, message response
            :rtype: Tuple[bool, Tuple[int, str]]
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

            ack, response, _ = self._get_message_response(
                message,
                connection,
                function_idx,
            )

            return ack, response

        @self._send_message(MessageSenders.AHRS_TRIM)
        @self._timer()
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

            :return: message send success/fail, message response
            :rtype: Tuple[bool, Tuple[int, str]]
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

            ack, response, _ = self._get_message_response(
                message,
                connection,
                function_idx,
            )

            return ack, response

        @self._send_message(MessageSenders.GYROSCOPE_CALIBRATION)
        @self._timer()
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

            :return: message send success/fail, message response
            :rtype: Tuple[bool, Tuple[int, str]]
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

            ack, response, _ = self._get_message_response(
                message,
                connection,
                function_idx,
            )

            return ack, response

        @self._send_message(MessageSenders.MAGNETOMETER_CALIBRATION)
        @self._timer()
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

            :return: message send success/fail, message response
            :rtype: Tuple[bool, Tuple[int, str]]
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

            ack, response, _ = self._get_message_response(
                message,
                connection,
                function_idx,
            )

            return ack, response

        @self._send_message(MessageSenders.GROUND_PRESSURE_CALIBRATION)
        @self._timer()
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

            :return: message send success/fail, message response
            :rtype: Tuple[bool, Tuple[int, str]]
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

            ack, response, _ = self._get_message_response(
                message,
                connection,
                function_idx,
            )

            return ack, response

        @self._send_message(MessageSenders.AIRSPEED_CALIBRATION)
        @self._timer()
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

            :return: message send success/fail, message response
            :rtype: Tuple[bool, Tuple[int, str]]
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

            ack, response, _ = self._get_message_response(
                message,
                connection,
                function_idx,
            )

            return ack, response

        @self._send_message(MessageSenders.BAROMETER_TEMPERATURE_CALIBRATION)
        @self._timer()
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

            :return: message send success/fail, message response
            :rtype: Tuple[bool, Tuple[int, str]]
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

            ack, response, _ = self._get_message_response(
                message,
                connection,
                function_idx,
            )

            return ack, response

        @self._send_message(MessageSenders.FLIGHT_MODE)
        @self._timer()
        def sender(
            message: swarm_messages.FlightModeCommand,
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

            :return: message send success/fail, message response
            :rtype: Tuple[bool, Tuple[int, str]]
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
            def verify_state_changed(message: Any, connection: Connection):
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

            ack, response, _ = self._get_message_response(
                message, connection, function_idx, verify_state_changed
            )

            return ack, response

        @self._send_message(MessageSenders.FLIGHT_SPEED)
        @self._timer()
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

            :return: message send success/fail, message response
            :rtype: Tuple[bool, Tuple[int, str]]
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

            ack, response, _ = self._get_message_response(
                message,
                connection,
                function_idx,
            )

            return ack, response

        @self._send_message(MessageSenders.TAKEOFF)
        @self._timer()
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

            :return: message send success/fail, message response
            :rtype: Tuple[bool, Tuple[int, str]]
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
            ack, response, _ = self._get_message_response(
                message,
                connection,
                function_idx,
            )

            return ack, response

        @self._send_message(MessageSenders.TAKEOFF_SEQUENCE)
        @self._timer()
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

            :return: message send success/fail, message response
            :rtype: Tuple[bool, Tuple[int, str]]
            """
            # Create a new guided mode
            guided_message = swarm_messages.FlightModeCommand(
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
            if not self._send_sequence_message(guided_message, connection):
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
            if not self._send_sequence_message(arm_message, connection):
                return False, responses.SEQUENCE_STAGE_FAILURE

            # Give the agent a chance to fully arm
            time.sleep(message.state_delay)

            # Reset the message type to be a simple takeoff command
            message.message_type = supported_messages.mission_commands.takeoff

            # Attempt to perform takeoff
            if not self._send_sequence_message(message, connection):
                return False, responses.SEQUENCE_STAGE_FAILURE

            return True, responses.SUCCESS

        @self._send_message(MessageSenders.WAYPOINT)
        @self._timer()
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

            :return: message send success/fail, message response
            :rtype: Tuple[bool, Tuple[int, str]]
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

            ack, response, _ = self._get_message_response(
                message,
                connection,
                function_idx,
            )

            return ack, response

        @self._send_message(MessageSenders.GET_HOME_POSITION)
        @self._timer()
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

            :return: message send success/fail, message response
            :rtype: Tuple[bool, Tuple[int, str]]
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

            ack, response, _ = self._get_message_response(
                message,
                connection,
                function_idx,
            )

            return ack, response

        @self._send_message(MessageSenders.RESET_HOME_TO_CURRENT)
        @self._timer()
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

            :return: message send success/fail, message response
            :rtype: Tuple[bool, Tuple[int, str]]
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

            def verify_state_changed(message: Any, connection: Connection):
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
                    self._send_sequence_message(update_home_state_message, connection)

                    if time.time() - start_time >= message.state_timeout:
                        ack = False
                        break

                return ack

            ack, response, _ = self._get_message_response(
                message, connection, function_idx, verify_state_changed
            )

            return ack, response

        @self._send_message(MessageSenders.RESET_HOME_POSITION)
        @self._timer()
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

            :return: message send success/fail, message response
            :rtype: Tuple[bool, Tuple[int, str]]
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
                    self._send_sequence_message(update_home_state_message, connection)

                    if time.time() - start_time >= message.state_timeout:
                        ack = False
                        break

                return ack

            ack, response, _ = self._get_message_response(
                message, connection, function_idx, verify_state_changed
            )

            return ack, response

        @self._send_message(MessageSenders.READ_PARAMETER)
        @self._timer()
        def sender(
            message: swarm_messages.Parameter,
            connection: Connection,
            function_idx: int = 0,
        ) -> Tuple[bool, Tuple[int, str]]:
            """
            Read a parameter value from the target agent.

            The parameter read can be interpretted as a request.

            :param message: parameter to read
            :type message: Parameter

            :param connection: MAVLink connection
            :type connection: Connection

            :return: parameter read success, parameter read response
            :rtype: Tuple[bool, Tuple[int, str]]
            """
            try:
                connection.mavlink_connection.mav.param_request_read_send(
                    message.target_system,
                    message.target_component,
                    str.encode(message.parameter_id),
                    -1,
                )
            except Exception:
                return False, responses.PARAM_READ_FAILURE

            ack, response, ack_msg = self._get_message_response(
                message, connection, function_idx, ack_packet_type="PARAM_VALUE"
            )

            if ack:
                read_param = swarm_state.ReadParameter(
                    ack_msg["param_id"],
                    ack_msg["param_value"],
                    ack_msg["param_type"],
                    ack_msg["param_index"],
                    ack_msg["param_count"],
                )

                connection.agents[
                    (message.target_system, message.target_component)
                ].last_params_read.append(read_param)

            return ack, response

        @self._send_message(MessageSenders.SET_PARAMETER)
        @self._timer()
        def sender(
            message: swarm_messages.Parameter,
            connection: Connection,
            function_idx: int = 0,
        ) -> Tuple[bool, Tuple[int, str]]:
            """
            Set a parameter on the target agent.

            :param message: parameter to read
            :type message: Parameter

            :param connection: MAVLink connection
            :type connection: Connection

            :return: parameter read success, parameter read response
            :rtype: Tuple[bool, Tuple[int, str]]
            """
            try:
                # NOTE: In the current state, we only support float parameter value types
                #       Additional types may be added in the future
                connection.mav.param_set_send(
                    message.target_system,
                    message.target_component,
                    str.encode(message.parameter_id),
                    message.parameter_value,
                    9,
                )
            except Exception:
                return False, responses.PARAM_READ_FAILURE

            ack, response, _ = self._get_message_response(
                message, connection, function_idx, ack_packet_type="PARAM_VALUE"
            )

            return ack, response
