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

import logging
import math
import time
from typing import Any, Callable, Tuple, Union

from pymavlink import mavutil

import pymavswarm.messages as swarm_messages
import pymavswarm.utils as swarm_utils
from pymavswarm import Connection
from pymavswarm.messages import SupportedMessages as supported_messages
from pymavswarm.messages import responses


class Senders:
    def __init__(
        self, logger_name: str = "senders", log_level: int = logging.INFO
    ) -> None:
        self.__logger = swarm_utils.init_logger(logger_name, log_level=log_level)
        self.__senders = {}

        @self.__send_message("ARM")
        @self.__timer()
        def sender(
            message: swarm_messages.SystemCommandMessage,
            connection: Connection,
            function_id: int = 0,
            agent_exists: bool = False,
        ) -> bool:
            """
            Arm an agent

            :param message: Arming message
            :type message: SystemCommandMessage

            :param function_id: The index of the method in the message type function
                handler list, defaults to 0
            :type function_id: int, optional

            :param agent_exists: Flag indicating whether the agent that the message
                is intended for exists in the network, defaults to False
            :type agent_exists: bool, optional

            :return: Indicates whether the message was sent successfully
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
            ack = False
            message_code = responses.ACK_FAILURE

            if self.__ack_message(
                "COMMAND_ACK", connection, timeout=message.ack_timeout
            ):
                self.__logger.info(
                    "Successfully acknowledged reception of the arm command sent to "
                    f"Agent ({message.target_system}, {message.target_comp})"
                )
                ack = True
                message_code = responses.SUCCESS

                if agent_exists:
                    start_time = time.time()

                    while not connection.agents[
                        (message.target_system, message.target_comp)
                    ].armed.value:
                        if time.time() - start_time >= message.state_timeout:
                            ack = False
                            break
                    if ack:
                        self.__logger.info(
                            f"Successfully verified that Agent ({message.target_system}, "
                            f"{message.target_comp}) switched to the armed state"
                        )
                    else:
                        self.__logger.error(
                            f"Failed to verify that Agent ({message.target_system}, "
                            f"{message.target_comp}) switched to the armed state"
                        )
                        message_code = responses.STATE_VALIDATION_FAILURE
            else:
                self.__logger.error(
                    "Failed to acknowledge reception of the arm command sent to Agent "
                    f"({message.target_system}, {message.target_comp})"
                )
                message_code = responses.ACK_FAILURE

            if message.retry and not ack:
                if self.__retry_message_send(
                    message,
                    self.__senders[message.message_type][function_id],
                    agent_exists,
                ):
                    ack = True
                    message_code = responses.SUCCESS

            message.response = message_code
            message.message_result_event.notify(context=message.context)

            return ack

        @self.__send_message("DISARM")
        @self.__timer()
        def sender(
            message: swarm_messages.SystemCommandMessage,
            connection: Connection,
            function_id: int = 0,
            agent_exists: bool = False,
        ) -> bool:
            """
            Disarm an agent

            :param message: Disarm message
            :type message: SystemCommandMessage

            :param function_id: The index of the method in the message type function
                handler list, defaults to 0
            :type function_id: int, optional

            :param agent_exists: Flag indicating whether the agent that the message
                is intended for exists in the network, defaults to False
            :type agent_exists: bool, optional

            :return: Indicates whether or not the message was successfully sent
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
            ack = False
            message_code = responses.ACK_FAILURE

            if self.__ack_message(
                "COMMAND_ACK", connection, timeout=message.ack_timeout
            ):
                self.__logger.info(
                    "Successfully acknowledged reception of the disarm command sent "
                    f"to Agent ({message.target_system}, {message.target_comp})"
                )
                ack = True
                message_code = responses.SUCCESS

                if agent_exists:
                    start_time = time.time()

                    while connection.agents[
                        (message.target_system, message.target_comp)
                    ].armed.value:
                        if time.time() - start_time >= message.state_timeout:
                            ack = False
                            break
                    if ack:
                        self.__logger.info(
                            f"Successfully verified that Agent ({message.target_system}, "
                            f"{message.target_comp}) switched to the disarmed state"
                        )
                    else:
                        self.__logger.error(
                            f"Failed to verify that Agent ({message.target_system}, "
                            f"{message.target_comp}) switched to the disarmed state"
                        )
                        message_code = responses.STATE_VALIDATION_FAILURE
            else:
                self.__logger.error(
                    "Failed to acknowledge reception of the disarm command sent to "
                    f"Agent ({message.target_system}, {message.target_comp})"
                )
                message_code = responses.ACK_FAILURE

            if message.retry and not ack:
                if self.__retry_message_send(
                    message,
                    self.__senders[message.message_type][function_id],
                    agent_exists,
                ):
                    ack = True
                    message_code = responses.SUCCESS

            message.response = message_code
            message.message_result_event.notify(context=message.context)

            return ack

        @self.__send_message("KILL")
        @self.__timer()
        def sender(
            message: swarm_messages.SystemCommandMessage,
            connection: Connection,
            function_id: int = 0,
            agent_exists: bool = False,
        ) -> bool:
            """
            Force disarm an agent

            :param message: Kill message
            :type message: SystemCommandMessage

            :param function_id: The index of the method in the message type function
                handler list, defaults to 0
            :type function_id: int, optional

            :param agent_exists: Flag indicating whether the agent that the message
                is intended for exists in the network, defaults to False
            :type agent_exists: bool, optional

            :return: Indicates whether or not the message was successfully sent
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
            ack = False
            message_code = responses.ACK_FAILURE

            if self.__ack_message(
                "COMMAND_ACK", connection, timeout=message.ack_timeout
            ):
                self.__logger.info(
                    f"Successfully acknowledged reception of the kill command sent to "
                    f"Agent ({message.target_system}, {message.target_comp})"
                )
                ack = True
                message_code = responses.SUCCESS

                if agent_exists:
                    self.__logger.info(
                        "The system does not support verification of state changes for "
                        "the kill command."
                    )
            else:
                self.__logger.error(
                    "Failed to acknowledge reception of the kill command sent to "
                    f"Agent ({message.target_system}, {message.target_comp})"
                )
                message_code = responses.ACK_FAILURE

            if message.retry and not ack:
                if self.__retry_message_send(
                    message,
                    self.__senders[message.message_type][function_id],
                    agent_exists,
                ):
                    ack = True
                    message_code = responses.SUCCESS

            message.response = message_code
            message.message_result_event.notify(context=message.context)

            return ack

        @self.__send_message("REBOOT")
        @self.__timer()
        def sender(
            message: swarm_messages.SystemCommandMessage,
            connection: Connection,
            function_id: int = 0,
            agent_exists: bool = False,
        ) -> bool:
            """
            Reboot an agent

            :param message: Reboot message
            :type message: SystemCommandMessage

            :param function_id: The index of the method in the message type function
                handler list, defaults to 0
            :type function_id: int, optional

            :param agent_exists: Flag indicating whether the agent that the message
                is intended for exists in the network, defaults to False
            :type agent_exists: bool, optional

            :return: Indicates whether or not the message was successfully sent
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
            ack = False
            message_code = responses.ACK_FAILURE

            if self.__ack_message(
                "COMMAND_ACK", connection, timeout=message.ack_timeout
            ):
                self.__logger.info(
                    "Successfully acknowledged reception of the reboot command sent "
                    f"to Agent ({message.target_system}, {message.target_comp})"
                )
                ack = True
                message_code = responses.SUCCESS

                if agent_exists:
                    self.__logger.info(
                        "The system does not support verification of state changes for "
                        "the reboot command."
                    )
            else:
                self.__logger.error(
                    "Failed to acknowledge reception of the reboot command sent to "
                    f"Agent ({message.target_system}, {message.target_comp})"
                )
                message_code = responses.ACK_FAILURE

            if message.retry and not ack:
                if self.__retry_message_send(
                    message,
                    self.__senders[message.message_type][function_id],
                    agent_exists,
                ):
                    ack = True
                    message_code = responses.SUCCESS

            message.response = message_code
            message.message_result_event.notify(context=message.context)

            return ack

        @self.__send_message("SHUTDOWN")
        @self.__timer()
        def sender(
            message: swarm_messages.SystemCommandMessage,
            connection: Connection,
            function_id: int = 0,
            agent_exists: bool = False,
        ) -> bool:
            """
            Shutdown an agent

            :param message: Shutdown message
            :type message: SystemCommandMessage

            :param function_id: The index of the method in the message type function handler
                list, defaults to 0
            :type function_id: int, optional

            :param agent_exists: Flag indicating whether the agent that the message
                is intended for exists in the network, defaults to False
            :type agent_exists: bool, optional

            :return: Indicates whether or not the message was successfully sent
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
            ack = False
            message_code = responses.ACK_FAILURE

            if self.__ack_message(
                "COMMAND_ACK", connection, timeout=message.ack_timeout
            ):
                self.__logger.info(
                    "Successfully acknowledged reception of the shutdown command sent "
                    f"to Agent ({message.target_system}, {message.target_comp})"
                )
                ack = True
                message_code = responses.SUCCESS

                if agent_exists:
                    self.__logger.info(
                        "The system does not support verification of state changes for "
                        "the shutdown command."
                    )
            else:
                self.__logger.error(
                    "Failed to acknowledge reception of the shutdown command sent to "
                    f"Agent ({message.target_system}, {message.target_comp})"
                )
                message_code = responses.ACK_FAILURE

            if message.retry and not ack:
                if self.__retry_message_send(
                    message,
                    self.__senders[message.message_type][function_id],
                    agent_exists,
                ):
                    ack = True
                    message_code = responses.SUCCESS

            message.response = message_code
            message.message_result_event.notify(context=message.context)

            return ack

        @self.__send_message("ACCELEROMETER_CALIBRATION")
        @self.__timer()
        def sender(
            message: swarm_messages.PreflightCalibrationMessage,
            connection: Connection,
            function_id: int = 0,
            agent_exists: bool = False,
        ) -> bool:
            """
            Perform a full accelerometer calibration on the selected agent

            :param message: Calibration message
            :type message: PreflightCalibrationMessage

            :param function_id: The index of the method in the message type function
                handler list, defaults to 0
            :type function_id: int, optional

            :param agent_exists: Flag indicating whether the agent that the message
                is intended for exists in the network, defaults to False
            :type agent_exists: bool, optional

            :return: Indicates whether or not the message was successfully sent
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
            ack = False
            message_code = responses.ACK_FAILURE

            if self.__ack_message(
                "COMMAND_ACK", connection, timeout=message.ack_timeout
            ):
                self.__logger.info(
                    "Successfully acknowledged reception of the accelerometer "
                    f"calibration command sent to Agent ({message.target_system}, "
                    f"{message.target_comp})"
                )
                ack = True
                message_code = responses.SUCCESS

                if agent_exists:
                    self.__logger.info(
                        "The system does not support verification of state changes for "
                        "the accelerometer calibration command."
                    )
            else:
                self.__logger.error(
                    "Failed to acknowledge reception of the accelerometer calibration "
                    f"command sent to Agent ({message.target_system}, {message.target_comp})"
                )
                message_code = responses.ACK_FAILURE

            if message.retry and not ack:
                if self.__retry_message_send(
                    message,
                    self.__senders[message.message_type][function_id],
                    agent_exists,
                ):
                    ack = True
                    message_code = responses.SUCCESS

            message.response = message_code
            message.message_result_event.notify(context=message.context)

            return ack

        @self.__send_message("SIMPLE_ACCELEROMETER_CALIBRATION")
        @self.__timer()
        def sender(
            message: swarm_messages.PreflightCalibrationMessage,
            connection: Connection,
            function_id: int = 0,
            agent_exists: bool = False,
        ) -> bool:
            """
            Perform a simple accelerometer calibration on the selected agent

            :param message: Calibration message
            :type message: PreflightCalibrationMessage

            :param function_id: The index of the method in the message type function
                handler list, defaults to 0
            :type function_id: int, optional

            :param agent_exists: Flag indicating whether the agent that the message
                is intended for exists in the network, defaults to False
            :type agent_exists: bool, optional

            :return: Indicates whether or not the message was successfully sent
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
            ack = False
            message_code = responses.ACK_FAILURE

            if self.__ack_message(
                "COMMAND_ACK", connection, timeout=message.ack_timeout
            ):
                self.__logger.info(
                    "Successfully acknowledged reception of the simple accelerometer "
                    f"calibration command sent to Agent ({message.target_system}, "
                    f"{message.target_comp})"
                )
                ack = True
                message_code = responses.SUCCESS

                if agent_exists:
                    self.__logger.info(
                        "The system does not support verification of state changes for "
                        "the simple accelerometer calibration command."
                    )
            else:
                self.__logger.error(
                    "Failed to acknowledge reception of the simple accelerometer "
                    f"calibration command sent to Agent ({message.target_system}, "
                    f"{message.target_comp})"
                )
                message_code = responses.ACK_FAILURE

            if message.retry and not ack:
                if self.__retry_message_send(
                    message,
                    self.__senders[message.message_type][function_id],
                    agent_exists,
                ):
                    ack = True
                    message_code = responses.SUCCESS

            message.response = message_code
            message.message_result_event.notify(context=message.context)

            return ack

        @self.__send_message("AHRS_TRIM")
        @self.__timer()
        def sender(
            message: swarm_messages.PreflightCalibrationMessage,
            connection: Connection,
            function_id: int = 0,
            agent_exists: bool = False,
        ) -> bool:
            """
            Perform an AHRS trim on the selected agent

            :param message: Calibration message
            :type message: PreflightCalibrationMessage

            :param function_id: The index of the method in the message type function handler
                list, defaults to 0
            :type function_id: int, optional

            :param agent_exists: Flag indicating whether the agent that the message
                is intended for exists in the network, defaults to False
            :type agent_exists: bool, optional

            :return: Indicates whether or not the message was successfully sent
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
            ack = False
            message_code = responses.ACK_FAILURE

            if self.__ack_message(
                "COMMAND_ACK", connection, timeout=message.ack_timeout
            ):
                self.__logger.info(
                    f"Successfully acknowledged reception of the AHRS trim command "
                    f"sent to Agent ({message.target_system}, {message.target_comp})"
                )
                ack = True
                message_code = responses.SUCCESS

                if agent_exists:
                    self.__logger.info(
                        "The system does not support verification of state changes for "
                        "the AHRS trim command."
                    )
            else:
                self.__logger.error(
                    "Failed to acknowledge reception of the AHRS trim command sent to "
                    f"Agent ({message.target_system}, {message.target_comp})"
                )
                message_code = responses.ACK_FAILURE

            if message.retry and not ack:
                if self.__retry_message_send(
                    message,
                    self.__senders[message.message_type][function_id],
                    agent_exists,
                ):
                    ack = True
                    message_code = responses.SUCCESS

            message.response = message_code
            message.message_result_event.notify(context=message.context)

            return ack

        @self.__send_message("GYROSCOPE_CALIBRATION")
        @self.__timer()
        def sender(
            message: swarm_messages.PreflightCalibrationMessage,
            connection: Connection,
            function_id: int = 0,
            agent_exists: bool = False,
        ) -> bool:
            """
            Perform a gyroscope calibration on the selected agent

            :param message: Calibration message
            :type message: PreflightCalibrationMessage

            :param function_id: The index of the method in the message type function handler
                list, defaults to 0
            :type function_id: int, optional

            :param agent_exists: Flag indicating whether the agent that the message
                is intended for exists in the network, defaults to False
            :type agent_exists: bool, optional

            :return: Indicates whether or not the message was successfully sent
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
            ack = False
            message_code = responses.ACK_FAILURE

            if self.__ack_message(
                "COMMAND_ACK", connection, timeout=message.ack_timeout
            ):
                self.__logger.info(
                    "Successfully acknowledged reception of the gyroscope calibration "
                    f"command sent to Agent ({message.target_system}, "
                    f"{message.target_comp})"
                )
                ack = True
                message_code = responses.SUCCESS

                if agent_exists:
                    self.__logger.info(
                        "The system does not support verification of state changes for "
                        "the gyroscope calibration command."
                    )
            else:
                self.__logger.error(
                    "Failed to acknowledge reception of the gyroscope calibration "
                    f"command sent to Agent ({message.target_system}, "
                    f"{message.target_comp})"
                )
                message_code = responses.ACK_FAILURE

            if message.retry and not ack:
                if self.__retry_message_send(
                    message,
                    self.__senders[message.message_type][function_id],
                    agent_exists,
                ):
                    ack = True
                    message_code = responses.SUCCESS

            message.response = message_code
            message.message_result_event.notify(context=message.context)

            return ack

        @self.__send_message("MAGNETOMETER_CALIBRATION")
        @self.__timer()
        def sender(
            message: swarm_messages.PreflightCalibrationMessage,
            connection: Connection,
            function_id: int = 0,
            agent_exists: bool = False,
        ) -> bool:
            """
            Perform a magnetometer calibration on the selected agent

            :param message: Calibration message
            :type message: PreflightCalibrationMessage

            :param function_id: The index of the method in the message type function handler
                list, defaults to 0
            :type function_id: int, optional

            :param agent_exists: Flag indicating whether the agent that the message
                is intended for exists in the network, defaults to False
            :type agent_exists: bool, optional

            :return: Indicates whether or not the message was successfully sent
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
            ack = False
            message_code = responses.ACK_FAILURE

            if self.__ack_message(
                "COMMAND_ACK", connection, timeout=message.ack_timeout
            ):
                self.__logger.info(
                    "Successfully acknowledged reception of the magnetometer "
                    f"calibration command sent to Agent ({message.target_system}, "
                    f"{message.target_comp})"
                )
                ack = True
                message_code = responses.SUCCESS

                if agent_exists:
                    self.__logger.info(
                        "The system does not support verification of state changes for "
                        "the magnetometer calibration command."
                    )
            else:
                self.__logger.error(
                    "Failed to acknowledge reception of the magnetometer calibration "
                    "command sent to Agent ({message.target_system}, {message.target_comp})"
                )
                message_code = responses.ACK_FAILURE

            if message.retry and not ack:
                if self.__retry_message_send(
                    message,
                    self.__senders[message.message_type][function_id],
                    agent_exists,
                ):
                    ack = True
                    message_code = responses.SUCCESS

            message.response = message_code
            message.message_result_event.notify(context=message.context)

            return ack

        @self.__send_message("GROUND_PRESSURE_CALIBRATION")
        @self.__timer()
        def sender(
            message: swarm_messages.PreflightCalibrationMessage,
            connection: Connection,
            function_id: int = 0,
            agent_exists: bool = False,
        ) -> bool:
            """
            Perform a ground pressure calibration on the selected agent

            :param message: Calibration message
            :type message: PreflightCalibrationMessage

            :param function_id: The index of the method in the message type function handler
                list, defaults to 0
            :type function_id: int, optional

            :param agent_exists: Flag indicating whether the agent that the message
                is intended for exists in the network, defaults to False
            :type agent_exists: bool, optional

            :return: Indicates whether or not the message was successfully sent
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
            ack = False
            message_code = responses.ACK_FAILURE

            if self.__ack_message(
                "COMMAND_ACK", connection, timeout=message.ack_timeout
            ):
                self.__logger.info(
                    "Successfully acknowledged reception of the ground pressure "
                    f"calibration command sent to Agent ({message.target_system}, "
                    f"{message.target_comp})"
                )
                ack = True
                message_code = responses.SUCCESS

                if agent_exists:
                    self.__logger.info(
                        "The system does not support verification of state changes for "
                        "the ground pressure calibration command."
                    )
            else:
                self.__logger.error(
                    "Failed to acknowledge reception of the ground pressure "
                    f"calibration command sent to Agent ({message.target_system}, "
                    f"{message.target_comp})"
                )
                message_code = responses.ACK_FAILURE

            if message.retry and not ack:
                if self.__retry_message_send(
                    message,
                    self.__senders[message.message_type][function_id],
                    agent_exists,
                ):
                    ack = True
                    message_code = responses.SUCCESS

            message.response = message_code
            message.message_result_event.notify(context=message.context)

            return ack

        @self.__send_message("AIRSPEED_CALIBRATION")
        @self.__timer()
        def sender(
            message: swarm_messages.PreflightCalibrationMessage,
            connection: Connection,
            function_id: int = 0,
            agent_exists: bool = False,
        ) -> bool:
            """
            Perform airspeed calibration on the selected agent

            :param message: Calibration message
            :type message: PreflightCalibrationMessage

            :param function_id: The index of the method in the message type function handler
                list, defaults to 0
            :type function_id: int, optional

            :param agent_exists: Flag indicating whether the agent that the message
                is intended for exists in the network, defaults to False
            :type agent_exists: bool, optional

            :return: Indicates whether or not the message was successfully sent
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
            ack = False
            message_code = responses.ACK_FAILURE

            if self.__ack_message(
                "COMMAND_ACK", connection, timeout=message.ack_timeout
            ):
                self.__logger.info(
                    "Successfully acknowledged reception of the airspeed calibration "
                    f"command sent to Agent ({message.target_system}, "
                    f"{message.target_comp})"
                )
                ack = True
                message_code = responses.SUCCESS

                if agent_exists:
                    self.__logger.info(
                        "The system does not support verification of state changes for "
                        "the airspeed calibration command."
                    )
            else:
                self.__logger.error(
                    "Failed to acknowledge reception of the airspeed calibration "
                    f"command sent to Agent ({message.target_system}, "
                    f"{message.target_comp})"
                )
                message_code = responses.ACK_FAILURE

            if message.retry and not ack:
                if self.__retry_message_send(
                    message,
                    self.__senders[message.message_type][function_id],
                    agent_exists,
                ):
                    ack = True
                    message_code = responses.SUCCESS

            message.response = message_code
            message.message_result_event.notify(context=message.context)

            return ack

        @self.__send_message("BAROMETER_TEMPERATURE_CALIBRATION")
        @self.__timer()
        def sender(
            message: swarm_messages.PreflightCalibrationMessage,
            connection: Connection,
            function_id: int = 0,
            agent_exists: bool = False,
        ) -> bool:
            """
            Perform a barometer temperature calibration on the selected agent

            :param message: Calibration message
            :type message: PreflightCalibrationMessage

            :param function_id: The index of the method in the message type function handler
                list, defaults to 0
            :type function_id: int, optional

            :param agent_exists: Flag indicating whether the agent that the message
                is intended for exists in the network, defaults to False
            :type agent_exists: bool, optional

            :return: Indicates whether or not the message was successfully sent
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
            ack = False
            message_code = responses.ACK_FAILURE

            if self.__ack_message(
                "COMMAND_ACK", connection, timeout=message.ack_timeout
            ):
                self.__logger.info(
                    "Successfully acknowledged reception of the barometer "
                    f"temperature calibration command sent to Agent "
                    f"({message.target_system}, {message.target_comp})"
                )
                ack = True
                message_code = responses.SUCCESS

                if agent_exists:
                    self.__logger.info(
                        "The system does not support verification of state changes for "
                        "the barometer temperature calibration command."
                    )
            else:
                self.__logger.error(
                    "Failed to acknowledge reception of the barometer temperature "
                    f"calibration command sent to Agent ({message.target_system}, "
                    f"{message.target_comp})"
                )
                message_code = responses.ACK_FAILURE

            if message.retry and not ack:
                if self.__retry_message_send(
                    message,
                    self.__senders[message.message_type][function_id],
                    agent_exists,
                ):
                    ack = True
                    message_code = responses.SUCCESS

            message.response = message_code
            message.message_result_event.notify(context=message.context)

            return ack

        @self.__send_message("FLIGHT_MODE")
        @self.__timer()
        def sender(
            message: swarm_messages.FlightModeMessage,
            connection: Connection,
            function_id: int = 0,
            agent_exists: bool = False,
        ) -> bool:
            """
            Set the flight mode of an agent.

            :param message: Flight mode message
            :type message: FlightModeMessage

            :param function_id: The index of the method in the message type function
                handler list, defaults to 0
            :type function_id: int, optional

            :param agent_exists: Flag indicating whether the agent that the message
                is intended for exists in the network, defaults to False
            :type agent_exists: bool, optional

            :return: Indicates whether or not the message was successfully sent
            :rtype: bool
            """
            # Reset target
            connection.mavlink_connection.target_system = message.target_system
            connection.mavlink_connection.target_component = message.target_comp

            # Verify that the flight mode is supported by the agent
            if message.flight_mode not in connection.mavlink_connection.mode_mapping():
                self.__logger.error(
                    f"The desired flight mode, {message.flight_mode}, is not a supported "
                    "flight mode. Supported flight modes include: "
                    f"{connection.mavlink_connection.mode_mapping().keys()}"
                )
                message.response = responses.INVALID_PROPERTIES
                message.message_result_event.notify(context=message.context)

                return False

            # Send flight mode
            connection.mavlink_connection.set_mode(
                connection.mavlink_connection.mode_mapping()[message.flight_mode]
            )

            ack = False
            message_code = responses.ACK_FAILURE

            if self.__ack_message(
                "COMMAND_ACK", connection, timeout=message.ack_timeout
            ):
                self.__logger.info(
                    "Successfully acknowledged reception of the flight mode "
                    f"{message.flight_mode} command sent to Agent ({message.target_system}, "
                    f"{message.target_comp})"
                )
                ack = True
                message_code = responses.SUCCESS

                if agent_exists:
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
                    if ack:
                        self.__logger.info(
                            f"Successfully verified that Agent ({message.target_system}, "
                            f"{message.target_comp}) switched to the {message.flight_mode} "
                            "flight mode"
                        )
                    else:
                        self.__logger.error(
                            f"Failed to verify that Agent ({message.target_system}, "
                            f"{message.target_comp}) switched to the {message.flight_mode} "
                            "flight mode"
                        )
                        message_code = responses.STATE_VALIDATION_FAILURE
            else:
                self.__logger.error(
                    "Failed to acknowledge reception of the flight mode "
                    f"{message.flight_mode} command sent to Agent ({message.target_system}, "
                    f"{message.target_comp})"
                )
                message_code = responses.ACK_FAILURE

            if message.retry and not ack:
                if self.__retry_message_send(
                    message,
                    self.__senders[message.message_type][function_id],
                    agent_exists,
                ):
                    ack = True
                    message_code = responses.SUCCESS

            message.response = message_code
            message.message_result_event.notify(context=message.context)

            return ack

        @self.__send_message("FLIGHT_SPEED")
        @self.__timer()
        def sender(
            message: swarm_messages.FlightSpeedMessage,
            connection: Connection,
            function_id: int = 0,
            agent_exists: bool = False,
        ) -> bool:
            """
            Set an agent's flight speed.

            :param message: Speed message
            :type message: FlightSpeedMessage

            :param function_id: The index of the method in the message type function
                handler list, defaults to 0
            :type function_id: int, optional

            :param agent_exists: Flag indicating whether the agent that the message
                is intended for exists in the network, defaults to False
            :type agent_exists: bool, optional

            :return: Indicates whether or not the message was successfully sent
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
            ack = False
            message_code = responses.ACK_FAILURE

            if self.__ack_message(
                "COMMAND_ACK", connection, timeout=message.ack_timeout
            ):
                self.__logger.info(
                    "Successfully acknowledged reception of the flight speed command "
                    f"{message.speed_type} sent to Agent ({message.target_system}, "
                    f"{message.target_comp})"
                )
                ack = True
                message_code = responses.SUCCESS

                if agent_exists:
                    self.__logger.info(
                        "The system does not support verification of state changes for "
                        "the flight speed commands."
                    )
            else:
                self.__logger.error(
                    "Failed to acknowledge reception of the flight speed command "
                    f"{message.speed_type} sent to Agent ({message.target_system}, "
                    f"{message.target_comp})"
                )
                message_code = responses.ACK_FAILURE

            if message.retry and not ack:
                if self.__retry_message_send(
                    message,
                    self.__senders[message.message_type][function_id],
                    agent_exists,
                ):
                    ack = True
                    message_code = responses.SUCCESS

            message.response = message_code
            message.message_result_event.notify(context=message.context)

            return ack

        @self.__send_message("SIMPLE_TAKEOFF")
        @self.__timer()
        def sender(
            message: swarm_messages.TakeoffMessage,
            connection: Connection,
            function_id: int = 0,
            agent_exists: bool = False,
        ) -> bool:
            """
            Perform a simple takeoff command (just takeoff to a set altitude)
            Note that acknowledgement of this command does not indicate that the
            altitude was reached, but rather that the system will attempt to reach
            the specified altitude

            :param message: Takeoff message
            :type message: TakeoffMessage

            :param function_id: The index of the method in the message type function handler
                list, defaults to 0
            :type function_id: int, optional

            :param agent_exists: Flag indicating whether the agent that the message
                is intended for exists in the network, defaults to False
            :type agent_exists: bool, optional

            :return: Indicates whether or not the message was successfully sent
            :rtype: bool
            """
            if (
                message.altitude < 0
                or math.isinf(message.altitude)
                or math.isnan(message.altitude)
            ):
                self.__logger.exception(
                    f"An invalid takeoff altitude was provided ({message.altitude}). "
                    "Please send a valid takeoff altitude"
                )
                message.response = responses.INVALID_PROPERTIES
                message.message_result_event.notify(context=message.context)
                return False

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
            ack = False
            message_code = responses.ACK_FAILURE

            if self.__ack_message(
                "COMMAND_ACK", connection, timeout=message.ack_timeout
            ):
                self.__logger.info(
                    "Successfully acknowledged reception of the simple takeoff "
                    f"command sent to Agent ({message.target_system}, "
                    f"{message.target_comp})"
                )
                ack = True
                message_code = responses.SUCCESS

                if agent_exists:
                    self.__logger.info(
                        "The system does not support verification of state changes for "
                        "the simple takeoff command."
                    )
            else:
                self.__logger.error(
                    "Failed to acknowledge reception of the simple takeoff command "
                    f"sent to Agent ({message.target_system}, {message.target_comp})"
                )
                message_code = responses.ACK_FAILURE

            if message.retry and not ack:
                if self.__retry_message_send(
                    message,
                    self.__senders[message.message_type][function_id],
                    agent_exists,
                ):
                    ack = True
                    message_code = responses.SUCCESS

            message.response = message_code
            message.message_result_event.notify(context=message.context)

            return ack

        @self.__send_message("TAKEOFF")
        @self.__timer()
        def sender(
            message: swarm_messages.TakeoffMessage,
            connection: Connection,
            function_id: int = 0,
            agent_exists: bool = False,
        ) -> bool:
            """
            Perform a takeoff command (use lat, lon, and alt)
            Note that acknowledgement of this command does not indicate that the
            altitude was reached, but rather that the system will attempt to reach
            the specified altitude

            :param message: Speed message
            :type message: TakeoffMessage

            :param function_id: The index of the method in the message type function handler
                list, defaults to 0
            :type function_id: int, optional

            :param agent_exists: Flag indicating whether the agent that the message
                is intended for exists in the network, defaults to False
            :type agent_exists: bool, optional

            :return: Indicates whether or not the message was successfully sent
            :rtype: bool
            """
            if (
                message.altitude < 0
                or math.isinf(message.altitude)
                or math.isnan(message.altitude)
            ):
                self.__logger.exception(
                    f"An invalid takeoff altitude was provided ({message.altitude}). "
                    "Please send a valid takeoff altitude"
                )
                message.response = responses.INVALID_PROPERTIES
                message.message_result_event.notify(context=message.context)
                return False

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
            ack = False
            message_code = responses.ACK_FAILURE

            if self.__ack_message(
                "COMMAND_ACK", connection, timeout=message.ack_timeout
            ):
                self.__logger.info(
                    "Successfully acknowledged reception of the takeoff command sent "
                    f"to Agent ({message.target_system}, {message.target_comp})"
                )
                ack = True
                message_code = responses.SUCCESS

                if agent_exists:
                    self.__logger.info(
                        "The system does not support verification of state changes for "
                        "the takeoff command."
                    )
            else:
                self.__logger.error(
                    f"Failed to acknowledge reception of the takeoff command sent to "
                    f"Agent ({message.target_system}, {message.target_comp})"
                )
                message_code = responses.ACK_FAILURE

            if message.retry and not ack:
                if self.__retry_message_send(
                    message,
                    self.__senders[message.message_type][function_id],
                    agent_exists,
                ):
                    ack = True
                    message_code = responses.SUCCESS

            message.response = message_code
            message.message_result_event.notify(context=message.context)

            return ack

        @self.__send_message("SIMPLE_FULL_TAKEOFF")
        @self.__timer()
        def sender(
            message: swarm_messages.TakeoffMessage,
            connection: Connection,
            function_id: int = 0,
            agent_exists: bool = False,
        ) -> bool:
            """
            Command used to signal execution of a full simple takeoff sequence:
                1. Switch to GUIDED mode
                2. Arm
                3. Takeoff

            :param message: Speed message
            :type message: TakeoffMessage

            :param function_id: The index of the method in the message type function handler
                list, defaults to 0
            :type function_id: int, optional

            :param agent_exists: Flag indicating whether the agent that the message
                is intended for exists in the network, defaults to False
            :type agent_exists: bool, optional

            :return: Indicates whether or not the message was successfully sent
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
            if not self.__send_seq_message(guided_message, agent_exists):
                self.__logger.error(
                    "Failed to acknowledge reception of the guided command stage "
                    "within the simple full takeoff command sent to Agent "
                    f"({message.target_system}, {message.target_comp}). Full takeoff "
                    "sequence failed."
                )
                message.response = responses.SEQUENCE_STAGE_FAILURE
                message.message_result_event.notify(context=message.context)
                return False

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
            if not self.__send_seq_message(arm_message, agent_exists):
                self.__logger.error(
                    "Failed to acknowledge reception of the arm command stage within "
                    "the simple full takeoff command sent to Agent "
                    f"({message.target_system}, {message.target_comp}). Full takeoff "
                    "sequence failed."
                )
                message.response = responses.SEQUENCE_STAGE_FAILURE
                message.message_result_event.notify(context=message.context)
                return False

            # Give the agent a chance to fully arm
            time.sleep(message.state_delay)

            # Reset the message type to be a simple takeoff command
            message.message_type = supported_messages.mission_commands.simple_takeoff

            # Attempt to perform takeoff
            if not self.__send_seq_message(message, agent_exists):
                self.__logger.error(
                    "Failed to acknowledge reception of the takeoff command stage "
                    "within the simple full takeoff command sent to Agent "
                    f"({message.target_system}, {message.target_comp}). Full takeoff "
                    "sequence failed."
                )
                message.response = responses.SEQUENCE_STAGE_FAILURE
                message.message_result_event.notify(context=message.context)
                return False

            self.__logger.info(
                "Successfully sent and acknowledged all steps in the simple full "
                "takeoff command sequence."
            )

            message.context.update({"response": responses.SUCCESS})
            message.message_result_event.notify(context=message.context)

            return True

        @self.__send_message("FULL_TAKEOFF")
        @self.__timer()
        def sender(
            message: swarm_messages.TakeoffMessage,
            connection: Connection,
            function_id: int = 0,
            agent_exists: bool = False,
        ) -> bool:
            """
            Command used to signal execution of a full takeoff sequence:
                1. Switch to GUIDED mode
                2. Arm
                3. Takeoff

            :param message: Speed message
            :type message: TakeoffMessage

            :param function_id: The index of the method in the message type function handler
                list, defaults to 0
            :type function_id: int, optional

            :param agent_exists: Flag indicating whether the agent that the message
                is intended for exists in the network, defaults to False
            :type agent_exists: bool, optional

            :return: Indicates whether or not the message was successfully sent
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
            if not self.__send_seq_message(guided_message, agent_exists):
                self.__logger.error(
                    "Failed to acknowledge reception of the guided command stage "
                    "within the full takeoff command sent to Agent "
                    f"({message.target_system}, {message.target_comp}). Full takeoff "
                    "sequence failed."
                )
                message.response = responses.SEQUENCE_STAGE_FAILURE
                message.message_result_event.notify(context=message.context)
                return False

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
            if not self.__send_seq_message(arm_message, agent_exists):
                self.__logger.error(
                    "Failed to acknowledge reception of the arm command stage within "
                    f"the full takeoff command sent to Agent ({message.target_system}, "
                    f"{message.target_comp}). Full takeoff sequence failed."
                )
                message.response = responses.SEQUENCE_STAGE_FAILURE
                message.message_result_event.notify(context=message.context)
                return False

            # Give the agent a chance to fully arm
            time.sleep(message.state_delay)

            # Reset the message type to be a simple takeoff command
            message.message_type = supported_messages.mission_commands.takeoff

            # Attempt to perform takeoff
            if not self.__send_seq_message(message, agent_exists):
                self.__logger.error(
                    "Failed to acknowledge reception of the takeoff command stage "
                    "within the full takeoff command sent to Agent "
                    f"({message.target_system}, {message.target_comp}). Full takeoff sequence "
                    "failed."
                )
                message.response = responses.SEQUENCE_STAGE_FAILURE
                message.message_result_event.notify(context=message.context)
                return False

            self.__logger.info(
                "Successfully sent and acknowledged all steps in the full takeoff "
                "command sequence."
            )
            message.response = responses.SUCCESS
            message.message_result_event.notify(context=message.context)

            return True

        @self.__send_message("SIMPLE_WAYPOINT")
        @self.__timer()
        def sender(
            message: swarm_messages.WaypointMessage,
            connection: Connection,
            function_id: int = 0,
            agent_exists: bool = False,
        ) -> bool:
            """
            Perform a simple waypoint command (just lat, lon, and alt)

            NOTE: Acknowledgement of this command does not indicate that the
            waypoint was reached, but rather that the system will attempt to reach
            the specified waypoint

            :param message: Waypoint message
            :type message: WaypointMessage

            :param function_id: The index of the method in the message type function handler
                list, defaults to 0
            :type function_id: int, optional

            :param agent_exists: Flag indicating whether the agent that the message
                is intended for exists in the network, defaults to False
            :type agent_exists: bool, optional

            :return: Indicates whether or not the message was successfully sent
            :rtype: bool
            """
            if (
                message.altitude < 0
                or math.isinf(message.altitude)
                or math.isnan(message.altitude)
            ):
                self.__logger.exception(
                    f"An invalid takeoff altitude was provided ({message.altitude}). "
                    "Please send a valid waypoint altitude"
                )
                message.response = responses.INVALID_PROPERTIES
                message.message_result_event.notify(context=message.context)
                return False

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
            ack = False
            message_code = responses.ACK_FAILURE

            if self.__ack_message(
                "COMMAND_ACK", connection, timeout=message.ack_timeout
            ):
                self.__logger.info(
                    "Successfully acknowledged reception of the simple waypoint "
                    f"command sent to Agent ({message.target_system}, "
                    f"{message.target_comp})"
                )
                ack = True
                message_code = responses.SUCCESS

                if agent_exists:
                    self.__logger.info(
                        "The system does not support verification of state changes for "
                        "the simple waypoint command."
                    )
            else:
                self.__logger.error(
                    f"Failed to acknowledge reception of the simple waypoint command "
                    f"sent to Agent ({message.target_system}, {message.target_comp})"
                )
                message_code = responses.ACK_FAILURE

            if message.retry and not ack:
                if self.__retry_message_send(
                    message,
                    self.__senders[message.message_type][function_id],
                    agent_exists,
                ):
                    ack = True
                    message_code = responses.SUCCESS

            message.response = message_code
            message.message_result_event.notify(context=message.context)

            return ack

        @self.__send_message("WAYPOINT")
        @self.__timer()
        def sender(
            message: swarm_messages.WaypointMessage,
            connection: Connection,
            function_id: int = 0,
            agent_exists: bool = False,
        ) -> bool:
            """
            Perform a waypoint navigation command

            NOTE: Acknowledgement of this command does not indicate that the
            waypoint was reached, but rather that the system will attempt to reach
            the specified waypoint

            :param message: Waypoint message
            :type message: WaypointMessage

            :param function_id: The index of the method in the message type function handler
                list, defaults to 0
            :type function_id: int, optional

            :param agent_exists: Flag indicating whether the agent that the message
                is intended for exists in the network, defaults to False
            :type agent_exists: bool, optional

            :return: Indicates whether or not the message was successfully sent
            :rtype: bool
            """
            if (
                message.altitude < 0
                or math.isinf(message.altitude)
                or math.isnan(message.altitude)
            ):
                self.__logger.exception(
                    f"An invalid takeoff altitude was provided ({message.altitude}). "
                    "Please send a valid waypoint altitude"
                )
                message.response = responses.INVALID_PROPERTIES
                message.message_result_event.notify(context=message.context)
                return False

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
            ack = False
            message_code = responses.ACK_FAILURE

            if self.__ack_message(
                "COMMAND_ACK", connection, timeout=message.ack_timeout
            ):
                self.__logger.info(
                    "Successfully acknowledged reception of the waypoint command "
                    f"sent to Agent ({message.target_system}, {message.target_comp})"
                )
                ack = True
                message_code = responses.SUCCESS

                if agent_exists:
                    self.__logger.info(
                        "The system does not support verification of state changes for "
                        "the waypoint command."
                    )
            else:
                self.__logger.error(
                    "Failed to acknowledge reception of the waypoint command sent to "
                    f"Agent ({message.target_system}, {message.target_comp})"
                )
                message_code = responses.ACK_FAILURE

            if message.retry and not ack:
                if self.__retry_message_send(
                    message,
                    self.__senders[message.message_type][function_id],
                    agent_exists,
                ):
                    ack = True
                    message_code = responses.SUCCESS

            message.response = message_code
            message.message_result_event.notify(context=message.context)

            return ack

        @self.__send_message("GET_HOME_POSITION")
        @self.__timer()
        def sender(
            message: swarm_messages.AgentMessage,
            connection: Connection,
            function_id: int = 0,
            agent_exists: bool = False,
        ) -> bool:
            """
            Get the current home position of an agent

            :param message: Get home position message
            :type message: AgentMessage

            :param function_id: The index of the method in the message type function handler
                list, defaults to 0
            :type function_id: int, optional

            :param agent_exists: Flag indicating whether the agent that the message
                is intended for exists in the network, defaults to False
            :type agent_exists: bool, optional

            :return: Indicates whether or not the message was successfully sent
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
            ack = False
            message_code = responses.ACK_FAILURE

            if self.__ack_message(
                "COMMAND_ACK", connection, timeout=message.ack_timeout
            ):
                self.__logger.info(
                    "Successfully acknowledged reception of the get home position "
                    f"command sent to Agent ({message.target_system}, "
                    f"{message.target_comp})"
                )
                ack = True
                message_code = responses.SUCCESS

                if agent_exists:
                    self.__logger.info(
                        "The system does not support verification of state changes for "
                        "the get home position command."
                    )
            else:
                self.__logger.error(
                    "Failed to acknowledge reception of the get home position command "
                    f"sent to Agent ({message.target_system}, {message.target_comp})"
                )
                message_code = responses.ACK_FAILURE

            if message.retry and not ack:
                if self.__retry_message_send(
                    message,
                    self.__senders[message.message_type][function_id],
                    agent_exists,
                ):
                    ack = True
                    message_code = responses.SUCCESS

            message.response = message_code
            message.message_result_event.notify(context=message.context)

            return ack

        @self.__send_message("RESET_HOME_TO_CURRENT")
        @self.__timer()
        def sender(
            message: swarm_messages.HomePositionMessage,
            connection: Connection,
            function_id: int = 0,
            agent_exists: bool = False,
        ) -> bool:
            """
            Reset the saved home position of an agent to the current position

            NOTE: Validation of this command can take a while. To ensure that the system
            has sufficient time to verify that the home position was properly reset, it
            may be necessary to extend the timeout periods

            :param message: Reset home position message
            :type message: HomePositionMessage

            :param function_id: The index of the method in the message type function handler
                list, defaults to 0
            :type function_id: int, optional

            :param agent_exists: Flag indicating whether the agent that the message
                is intended for exists in the network, defaults to False
            :type agent_exists: bool, optional

            :return: Indicates whether or not the message was successfully sent
            :rtype: bool
            """
            agent_id = (message.target_system, message.target_comp)

            if agent_exists:
                current_home_pos = connection.agents[agent_id].home_position

                self.__logger.info(
                    "The initial home position prior to home location reset is as "
                    f"follows: Latitude: {current_home_pos.latitude}, Longitude: "
                    f"{current_home_pos.longitude}, Altitude: "
                    f"{current_home_pos.altitude}"
                )

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

            ack = False
            message_code = responses.ACK_FAILURE

            if self.__ack_message(
                "COMMAND_ACK", connection, timeout=message.ack_timeout
            ):
                self.__logger.info(
                    "Successfully acknowledged reception of the reset home position "
                    f"to current position command sent to Agent "
                    f"({message.target_system}, {message.target_comp})"
                )
                ack = True
                message_code = responses.SUCCESS

                if agent_exists:
                    start_time = time.time()

                    while vars(connection.agents[agent_id].home_position) == vars(
                        current_home_pos
                    ):
                        # Signal an update state command
                        update_home_state_message = swarm_messages.AgentMessage(
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
                        self.__send_seq_message(update_home_state_message, agent_exists)

                        if time.time() - start_time >= message.state_timeout:
                            ack = False
                            break
                    if ack:
                        self.__logger.info(
                            "Successfully the reset the home position of Agent "
                            f"({message.target_system}, {message.target_comp}) to: "
                            "Latitude: "
                            f"{connection.agents[agent_id].home_position.latitude}, "
                            "Longitude: "
                            f"{connection.agents[agent_id].home_position.longitude}, "
                            f"Altitude: "
                            f"{connection.agents[agent_id].home_position.altitude}"
                        )
                    else:
                        self.__logger.error(
                            "Failed to reset the home position of Agent "
                            f"({message.target_system}, {message.target_comp}) or the current "
                            "location of the agent has not changed since last updating "
                            "the home position."
                        )
                        message_code = responses.STATE_VALIDATION_FAILURE
            else:
                self.__logger.error(
                    "Failed to acknowledge reception of the reset home position to "
                    f"current position command sent to Agent ({message.target_system}, "
                    f"{message.target_comp})"
                )
                message_code = responses.ACK_FAILURE

            if message.retry and not ack:
                if self.__retry_message_send(
                    message,
                    self.__senders[message.message_type][function_id],
                    agent_exists,
                ):
                    ack = True
                    message_code = responses.SUCCESS

            message.response = message_code
            message.message_result_event.notify(context=message.context)

            return ack

        @self.__send_message("RESET_HOME")
        @self.__timer()
        def sender(
            message: swarm_messages.HomePositionMessage,
            connection: Connection,
            function_id: int = 0,
            agent_exists: bool = False,
        ) -> bool:
            """
            Reset the saved home position of an agent to the desired position

            :param message: Reset home position message
            :type message: HomePositionMessage

            :param function_id: The index of the method in the message type function handler
                list, defaults to 0
            :type function_id: int, optional

            :param agent_exists: Flag indicating whether the agent that the message
                is intended for exists in the network, defaults to False
            :type agent_exists: bool, optional

            :return: Indicates whether or not the message was successfully sent
            :rtype: bool
            """
            if (
                message.longitude is None
                or message.latitude is None
                or message.altitude is None
            ):
                self.__logger.exception(
                    "Cannot reset the home location to the given location unless the "
                    "latitude, longitude, and altitude are all provided."
                )
                message.response = responses.INVALID_PROPERTIES
                message.message_result_event.notify(context=message.context)
                return False

            if message.altitude < 0.0 or message.altitude > 150.0:
                self.__logger.exception(
                    "An invalid home position altitude was provided "
                    f"({message.altitude}). Please set a valid altitude"
                )
                message.response = responses.INVALID_PROPERTIES
                message.message_result_event.notify(context=message.context)
                return False

            agent_id = (message.target_system, message.target_comp)

            if agent_exists:
                current_home_pos = connection.agents[agent_id]

                self.__logger.info(
                    "The initial home position prior to home location reset is "
                    f"as follows: Latitude: {current_home_pos.latitude}, Longitude: "
                    f"{current_home_pos.longitude}, Altitude: "
                    f"{current_home_pos.altitude}"
                )

            # Set the home position to the desired location
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

            ack = False
            message_code = responses.ACK_FAILURE

            if self.__ack_message(
                "COMMAND_ACK", connection, timeout=message.ack_timeout
            ):
                self.__logger.info(
                    "Successfully acknowledged reception of the reset home position "
                    f"command sent to Agent ({message.target_system}, "
                    f"{message.target_comp})"
                )
                ack = True
                message_code = responses.SUCCESS

                if agent_exists:
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
                        update_home_state_message = swarm_messages.AgentMessage(
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
                        self.__send_seq_message(update_home_state_message, agent_exists)

                        if time.time() - start_time >= message.state_timeout:
                            ack = False
                            break
                    if ack:
                        self.__logger.info(
                            "Successfully the reset the home position of Agent "
                            f"({message.target_system}, {message.target_comp}) to: "
                            "Latitude: "
                            f"{connection.agents[agent_id].home_position.latitude}, "
                            "Longitude: "
                            f"{connection.agents[agent_id].home_position.longitude}, "
                            f"Altitude: "
                            f"{connection.agents[agent_id].home_position.altitude}"
                        )
                    else:
                        self.__logger.error(
                            "Failed to reset the home position of Agent "
                            f"({message.target_system}, {message.target_comp})"
                        )
                        message_code = responses.STATE_VALIDATION_FAILURE
            else:
                self.__logger.error(
                    "Failed to acknowledge reception of the reset home position "
                    f"command sent to Agent ({message.target_system}, "
                    f"{message.target_comp})"
                )
                message_code = responses.ACK_FAILURE

            if message.retry and not ack:
                if self.__retry_message_send(
                    message,
                    self.__senders[message.message_type][function_id],
                    agent_exists,
                ):
                    ack = True
                    message_code = responses.SUCCESS

            message.response = message_code
            message.message_result_event.notify(context=message.context)

            return ack

        return

    @property
    def senders(self) -> dict:
        """
        Methods responsible for handling message sending.

        :rtype: dict
        """
        return self.__senders

    def __retry_message_send(
        self, message: Any, function: Callable, agent_exists: bool
    ) -> bool:
        """
        Retry a message send until the an acknowledgement is received or a timeout
        occurs

        :param message: The message to retry sending
        :type message: Any

        :param function: The function to call using the message
        :type function: function

        :return: Indicate whether the retry was successful
        :rtype: bool
        """
        ack = False
        start_time = time.time()

        # Don't let the message come back here and create an infinite loop
        message.retry = False

        while time.time() - start_time <= message.message_timeout:
            # Reattempt the message send
            if function(self, message, agent_exists=agent_exists):
                ack = True
                break

        return ack

    def __send_seq_message(self, message: Any, agent_exists: bool) -> bool:
        """
        Helper function used to handle calling all of the message handlers.
        This method is used by the sequence commands such as the full takeoff
        command to provide indication of a function execution result.

        NOTE: THIS IS USED. DO NOT DELETE

        :param message: The message to send
        :type message: Any

        :param agent_exists: Flag indicating whether the given agent exists in the
            network
        :type agent_exists: bool

        :return: Indicates whether all of the message senders for a given message
            successfully sent their respective message
        :rtype: bool
        """
        # Helper function used to send the desired command
        if message.message_type in self.__senders:
            for function_id, function in enumerate(
                self.__senders[message.message_type]
            ):
                try:
                    if not function(
                        self,
                        message,
                        function_id=function_id,
                        agent_exists=agent_exists,
                    ):
                        return False
                except Exception:
                    pass

        return True

    def __send_message(self, message: Union[list, str]) -> Callable:
        """
        Decorator used to create a sender for a mavlink message

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
        Decorator used to log the time that a sender takes to complete. Used for
        debugging purposes.

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
