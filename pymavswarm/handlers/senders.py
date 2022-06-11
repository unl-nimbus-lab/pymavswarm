import logging
import math
import time
from typing import Any, Callable, Tuple, Union

from pymavlink import mavutil

import pymavswarm.msg as swarm_msgs
from pymavswarm import Connection
from pymavswarm.msg import SupportedMsgs as supported_msgs
from pymavswarm.msg import responses


class Senders:
    def __init__(
        self, logger_name: str = "senders", log_level: int = logging.INFO
    ) -> None:
        self.__logger = self.__init_logger(logger_name, log_level=log_level)
        self.__senders = {}

        @self.__send_message("ARM")
        @self.__timer()
        def sender(
            self,
            msg: swarm_msgs.SystemCommandMsg,
            connection: Connection,
            function_id: int = 0,
            device_exists: bool = False,
        ) -> bool:
            """
            Arm an agent

            :param msg: Arming message
            :type msg: SystemCommandMsg

            :param function_id: The index of the method in the message type function
                handler list, defaults to 0
            :type function_id: int, optional

            :param device_exists: Flag indicating whether the device that the message
                is intended for exists in the network, defaults to False
            :type device_exists: bool, optional

            :return: Indicates whether the message was sent successfully
            :rtype: bool
            """
            connection.mavlink_connection.mav.command_long_send(
                msg.target_system,
                msg.target_comp,
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
            msg_code = responses.ACK_FAILURE

            if self.__ack_msg("COMMAND_ACK", timeout=msg.ack_timeout):
                self.logger.info(
                    "Successfully acknowledged reception of the arm command sent to "
                    f"Agent ({msg.target_system}, {msg.target_comp})"
                )
                ack = True
                msg_code = responses.SUCCESS

                if device_exists:
                    start_time = time.time()

                    while not self.__devices[
                        (msg.target_system, msg.target_comp)
                    ].armed.value:
                        if time.time() - start_time >= msg.state_timeout:
                            ack = False
                            break
                    if ack:
                        self.logger.info(
                            f"Successfully verified that Agent ({msg.target_system}, "
                            f"{msg.target_comp}) switched to the armed state"
                        )
                    else:
                        self.logger.error(
                            f"Failed to verify that Agent ({msg.target_system}, "
                            f"{msg.target_comp}) switched to the armed state"
                        )
                        msg_code = responses.STATE_VALIDATION_FAILURE
            else:
                self.logger.error(
                    "Failed to acknowledge reception of the arm command sent to Agent "
                    f"({msg.target_system}, {msg.target_comp})"
                )
                msg_code = responses.ACK_FAILURE

            if msg.retry and not ack:
                if self.__retry_msg_send(
                    msg,
                    self.__message_senders[msg.msg_type][function_id],
                    device_exists,
                ):
                    ack = True
                    msg_code = responses.SUCCESS

            msg.response = msg_code
            msg.message_result_event.notify(context=msg.context)

            return ack

        @self.__send_message("DISARM")
        @self.__timer()
        def sender(
            self,
            msg: swarm_msgs.SystemCommandMsg,
            connection: Connection,
            function_id: int = 0,
            device_exists: bool = False,
        ) -> bool:
            """
            Disarm an agent

            :param msg: Disarm message
            :type msg: SystemCommandMsg

            :param function_id: The index of the method in the message type function handler
                list, defaults to 0
            :type function_id: int, optional

            :param device_exists: Flag indicating whether the device that the message
                is intended for exists in the network, defaults to False
            :type device_exists: bool, optional

            :return: Indicates whether or not the message was successfully sent
            :rtype: bool
            """
            connection.mavlink_connection.mav.command_long_send(
                msg.target_system,
                msg.target_comp,
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
            msg_code = responses.ACK_FAILURE

            if self.__ack_msg("COMMAND_ACK", timeout=msg.ack_timeout):
                self.logger.info(
                    "Successfully acknowledged reception of the disarm command sent "
                    f"to Agent ({msg.target_system}, {msg.target_comp})"
                )
                ack = True
                msg_code = responses.SUCCESS

                if device_exists:
                    start_time = time.time()

                    while self.__devices[
                        (msg.target_system, msg.target_comp)
                    ].armed.value:
                        if time.time() - start_time >= msg.state_timeout:
                            ack = False
                            break
                    if ack:
                        self.logger.info(
                            f"Successfully verified that Agent ({msg.target_system}, "
                            f"{msg.target_comp}) switched to the disarmed state"
                        )
                    else:
                        self.logger.error(
                            f"Failed to verify that Agent ({msg.target_system}, "
                            f"{msg.target_comp}) switched to the disarmed state"
                        )
                        msg_code = responses.STATE_VALIDATION_FAILURE
            else:
                self.logger.error(
                    "Failed to acknowledge reception of the disarm command sent to "
                    f"Agent ({msg.target_system}, {msg.target_comp})"
                )
                msg_code = responses.ACK_FAILURE

            if msg.retry and not ack:
                if self.__retry_msg_send(
                    msg,
                    self.__message_senders[msg.msg_type][function_id],
                    device_exists,
                ):
                    ack = True
                    msg_code = responses.SUCCESS

            msg.response = msg_code
            msg.message_result_event.notify(context=msg.context)

            return ack

        @self.__send_message("KILL")
        @self.__timer()
        def sender(
            self,
            msg: swarm_msgs.SystemCommandMsg,
            connection: Connection,
            function_id: int = 0,
            device_exists: bool = False,
        ) -> bool:
            """
            Force disarm an agent

            :param msg: Kill message
            :type msg: SystemCommandMsg

            :param function_id: The index of the method in the message type function handler
                list, defaults to 0
            :type function_id: int, optional

            :param device_exists: Flag indicating whether the device that the message
                is intended for exists in the network, defaults to False
            :type device_exists: bool, optional

            :return: Indicates whether or not the message was successfully sent
            :rtype: bool
            """
            connection.mavlink_connection.mav.command_long_send(
                msg.target_system,
                msg.target_comp,
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
            msg_code = responses.ACK_FAILURE

            if self.__ack_msg("COMMAND_ACK", timeout=msg.ack_timeout):
                self.logger.info(
                    f"Successfully acknowledged reception of the kill command sent to "
                    f"Agent ({msg.target_system}, {msg.target_comp})"
                )
                ack = True
                msg_code = responses.SUCCESS

                if device_exists:
                    self.logger.info(
                        "The system does not support verification of state changes for "
                        "the kill command."
                    )
            else:
                self.logger.error(
                    "Failed to acknowledge reception of the kill command sent to "
                    f"Agent ({msg.target_system}, {msg.target_comp})"
                )
                msg_code = responses.ACK_FAILURE

            if msg.retry and not ack:
                if self.__retry_msg_send(
                    msg,
                    self.__message_senders[msg.msg_type][function_id],
                    device_exists,
                ):
                    ack = True
                    msg_code = responses.SUCCESS

            msg.response = msg_code
            msg.message_result_event.notify(context=msg.context)

            return ack

        @self.__send_message("REBOOT")
        @self.__timer()
        def sender(
            self,
            msg: swarm_msgs.SystemCommandMsg,
            connection: Connection,
            function_id: int = 0,
            device_exists: bool = False,
        ) -> bool:
            """
            Reboot an agent

            :param msg: Reboot message
            :type msg: SystemCommandMsg

            :param function_id: The index of the method in the message type function handler
                list, defaults to 0
            :type function_id: int, optional

            :param device_exists: Flag indicating whether the device that the message
                is intended for exists in the network, defaults to False
            :type device_exists: bool, optional

            :return: Indicates whether or not the message was successfully sent
            :rtype: bool
            """
            connection.mavlink_connection.mav.command_long_send(
                msg.target_system,
                msg.target_comp,
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
            msg_code = responses.ACK_FAILURE

            if self.__ack_msg("COMMAND_ACK", timeout=msg.ack_timeout):
                self.logger.info(
                    "Successfully acknowledged reception of the reboot command sent "
                    f"to Agent ({msg.target_system}, {msg.target_comp})"
                )
                ack = True
                msg_code = responses.SUCCESS

                if device_exists:
                    self.logger.info(
                        "The system does not support verification of state changes for "
                        "the reboot command."
                    )
            else:
                self.logger.error(
                    "Failed to acknowledge reception of the reboot command sent to "
                    f"Agent ({msg.target_system}, {msg.target_comp})"
                )
                msg_code = responses.ACK_FAILURE

            if msg.retry and not ack:
                if self.__retry_msg_send(
                    msg,
                    self.__message_senders[msg.msg_type][function_id],
                    device_exists,
                ):
                    ack = True
                    msg_code = responses.SUCCESS

            msg.response = msg_code
            msg.message_result_event.notify(context=msg.context)

            return ack

        @self.__send_message("SHUTDOWN")
        @self.__timer()
        def sender(
            self,
            msg: swarm_msgs.SystemCommandMsg,
            connection: Connection,
            function_id: int = 0,
            device_exists: bool = False,
        ) -> bool:
            """
            Shutdown an agent

            :param msg: Shutdown message
            :type msg: SystemCommandMsg

            :param function_id: The index of the method in the message type function handler
                list, defaults to 0
            :type function_id: int, optional

            :param device_exists: Flag indicating whether the device that the message
                is intended for exists in the network, defaults to False
            :type device_exists: bool, optional

            :return: Indicates whether or not the message was successfully sent
            :rtype: bool
            """
            connection.mavlink_connection.mav.command_long_send(
                msg.target_system,
                msg.target_comp,
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
            msg_code = responses.ACK_FAILURE

            if self.__ack_msg("COMMAND_ACK", timeout=msg.ack_timeout):
                self.logger.info(
                    "Successfully acknowledged reception of the shutdown command sent "
                    f"to Agent ({msg.target_system}, {msg.target_comp})"
                )
                ack = True
                msg_code = responses.SUCCESS

                if device_exists:
                    self.logger.info(
                        "The system does not support verification of state changes for "
                        "the shutdown command."
                    )
            else:
                self.logger.error(
                    "Failed to acknowledge reception of the shutdown command sent to "
                    f"Agent ({msg.target_system}, {msg.target_comp})"
                )
                msg_code = responses.ACK_FAILURE

            if msg.retry and not ack:
                if self.__retry_msg_send(
                    msg,
                    self.__message_senders[msg.msg_type][function_id],
                    device_exists,
                ):
                    ack = True
                    msg_code = responses.SUCCESS

            msg.response = msg_code
            msg.message_result_event.notify(context=msg.context)

            return ack

        @self.__send_message("ACCELEROMETER_CALIBRATION")
        @self.__timer()
        def sender(
            self,
            msg: swarm_msgs.PreflightCalibrationMsg,
            connection: Connection,
            function_id: int = 0,
            device_exists: bool = False,
        ) -> bool:
            """
            Perform a full accelerometer calibration on the selected agent

            :param msg: Calibration message
            :type msg: PreflightCalibrationMsg

            :param function_id: The index of the method in the message type function handler
                list, defaults to 0
            :type function_id: int, optional

            :param device_exists: Flag indicating whether the device that the message
                is intended for exists in the network, defaults to False
            :type device_exists: bool, optional

            :return: Indicates whether or not the message was successfully sent
            :rtype: bool
            """
            connection.mavlink_connection.mav.command_long_send(
                msg.target_system,
                msg.target_comp,
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
            msg_code = responses.ACK_FAILURE

            if self.__ack_msg("COMMAND_ACK", timeout=msg.ack_timeout):
                self.logger.info(
                    "Successfully acknowledged reception of the accelerometer "
                    f"calibration command sent to Agent ({msg.target_system}, "
                    f"{msg.target_comp})"
                )
                ack = True
                msg_code = responses.SUCCESS

                if device_exists:
                    self.logger.info(
                        "The system does not support verification of state changes for "
                        "the accelerometer calibration command."
                    )
            else:
                self.logger.error(
                    "Failed to acknowledge reception of the accelerometer calibration "
                    f"command sent to Agent ({msg.target_system}, {msg.target_comp})"
                )
                msg_code = responses.ACK_FAILURE

            if msg.retry and not ack:
                if self.__retry_msg_send(
                    msg,
                    self.__message_senders[msg.msg_type][function_id],
                    device_exists,
                ):
                    ack = True
                    msg_code = responses.SUCCESS

            msg.response = msg_code
            msg.message_result_event.notify(context=msg.context)

            return ack

        @self.__send_message("SIMPLE_ACCELEROMETER_CALIBRATION")
        @self.__timer()
        def sender(
            self,
            msg: swarm_msgs.PreflightCalibrationMsg,
            connection: Connection,
            function_id: int = 0,
            device_exists: bool = False,
        ) -> bool:
            """
            Perform a simple accelerometer calibration on the selected agent

            :param msg: Calibration message
            :type msg: PreflightCalibrationMsg

            :param function_id: The index of the method in the message type function
                handler list, defaults to 0
            :type function_id: int, optional

            :param device_exists: Flag indicating whether the device that the message
                is intended for exists in the network, defaults to False
            :type device_exists: bool, optional

            :return: Indicates whether or not the message was successfully sent
            :rtype: bool
            """
            connection.mavlink_connection.mav.command_long_send(
                msg.target_system,
                msg.target_comp,
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
            msg_code = responses.ACK_FAILURE

            if self.__ack_msg("COMMAND_ACK", timeout=msg.ack_timeout):
                self.logger.info(
                    "Successfully acknowledged reception of the simple accelerometer "
                    f"calibration command sent to Agent ({msg.target_system}, "
                    f"{msg.target_comp})"
                )
                ack = True
                msg_code = responses.SUCCESS

                if device_exists:
                    self.logger.info(
                        "The system does not support verification of state changes for "
                        "the simple accelerometer calibration command."
                    )
            else:
                self.logger.error(
                    "Failed to acknowledge reception of the simple accelerometer "
                    f"calibration command sent to Agent ({msg.target_system}, "
                    f"{msg.target_comp})"
                )
                msg_code = responses.ACK_FAILURE

            if msg.retry and not ack:
                if self.__retry_msg_send(
                    msg,
                    self.__message_senders[msg.msg_type][function_id],
                    device_exists,
                ):
                    ack = True
                    msg_code = responses.SUCCESS

            msg.response = msg_code
            msg.message_result_event.notify(context=msg.context)

            return ack

        @self.__send_message("AHRS_TRIM")
        @self.__timer()
        def sender(
            self,
            msg: swarm_msgs.PreflightCalibrationMsg,
            connection: Connection,
            function_id: int = 0,
            device_exists: bool = False,
        ) -> bool:
            """
            Perform an AHRS trim on the selected agent

            :param msg: Calibration message
            :type msg: PreflightCalibrationMsg

            :param function_id: The index of the method in the message type function handler
                list, defaults to 0
            :type function_id: int, optional

            :param device_exists: Flag indicating whether the device that the message
                is intended for exists in the network, defaults to False
            :type device_exists: bool, optional

            :return: Indicates whether or not the message was successfully sent
            :rtype: bool
            """
            connection.mavlink_connection.mav.command_long_send(
                msg.target_system,
                msg.target_comp,
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
            msg_code = responses.ACK_FAILURE

            if self.__ack_msg("COMMAND_ACK", timeout=msg.ack_timeout):
                self.logger.info(
                    f"Successfully acknowledged reception of the AHRS trim command "
                    f"sent to Agent ({msg.target_system}, {msg.target_comp})"
                )
                ack = True
                msg_code = responses.SUCCESS

                if device_exists:
                    self.logger.info(
                        "The system does not support verification of state changes for "
                        "the AHRS trim command."
                    )
            else:
                self.logger.error(
                    "Failed to acknowledge reception of the AHRS trim command sent to "
                    f"Agent ({msg.target_system}, {msg.target_comp})"
                )
                msg_code = responses.ACK_FAILURE

            if msg.retry and not ack:
                if self.__retry_msg_send(
                    msg,
                    self.__message_senders[msg.msg_type][function_id],
                    device_exists,
                ):
                    ack = True
                    msg_code = responses.SUCCESS

            msg.response = msg_code
            msg.message_result_event.notify(context=msg.context)

            return ack

        @self.__send_message("GYROSCOPE_CALIBRATION")
        @self.__timer()
        def sender(
            self,
            msg: swarm_msgs.PreflightCalibrationMsg,
            connection: Connection,
            function_id: int = 0,
            device_exists: bool = False,
        ) -> bool:
            """
            Perform a gyroscope calibration on the selected agent

            :param msg: Calibration message
            :type msg: PreflightCalibrationMsg

            :param function_id: The index of the method in the message type function handler
                list, defaults to 0
            :type function_id: int, optional

            :param device_exists: Flag indicating whether the device that the message
                is intended for exists in the network, defaults to False
            :type device_exists: bool, optional

            :return: Indicates whether or not the message was successfully sent
            :rtype: bool
            """
            connection.mavlink_connection.mav.command_long_send(
                msg.target_system,
                msg.target_comp,
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
            msg_code = responses.ACK_FAILURE

            if self.__ack_msg("COMMAND_ACK", timeout=msg.ack_timeout):
                self.logger.info(
                    "Successfully acknowledged reception of the gyroscope calibration "
                    f"command sent to Agent ({msg.target_system}, "
                    f"{msg.target_comp})"
                )
                ack = True
                msg_code = responses.SUCCESS

                if device_exists:
                    self.logger.info(
                        "The system does not support verification of state changes for "
                        "the gyroscope calibration command."
                    )
            else:
                self.logger.error(
                    "Failed to acknowledge reception of the gyroscope calibration "
                    f"command sent to Agent ({msg.target_system}, "
                    f"{msg.target_comp})"
                )
                msg_code = responses.ACK_FAILURE

            if msg.retry and not ack:
                if self.__retry_msg_send(
                    msg,
                    self.__message_senders[msg.msg_type][function_id],
                    device_exists,
                ):
                    ack = True
                    msg_code = responses.SUCCESS

            msg.response = msg_code
            msg.message_result_event.notify(context=msg.context)

            return ack

        @self.__send_message("MAGNETOMETER_CALIBRATION")
        @self.__timer()
        def sender(
            self,
            msg: swarm_msgs.PreflightCalibrationMsg,
            connection: Connection,
            function_id: int = 0,
            device_exists: bool = False,
        ) -> bool:
            """
            Perform a magnetometer calibration on the selected agent

            :param msg: Calibration message
            :type msg: PreflightCalibrationMsg

            :param function_id: The index of the method in the message type function handler
                list, defaults to 0
            :type function_id: int, optional

            :param device_exists: Flag indicating whether the device that the message
                is intended for exists in the network, defaults to False
            :type device_exists: bool, optional

            :return: Indicates whether or not the message was successfully sent
            :rtype: bool
            """
            connection.mavlink_connection.mav.command_long_send(
                msg.target_system,
                msg.target_comp,
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
            msg_code = responses.ACK_FAILURE

            if self.__ack_msg("COMMAND_ACK", timeout=msg.ack_timeout):
                self.logger.info(
                    "Successfully acknowledged reception of the magnetometer "
                    f"calibration command sent to Agent ({msg.target_system}, "
                    f"{msg.target_comp})"
                )
                ack = True
                msg_code = responses.SUCCESS

                if device_exists:
                    self.logger.info(
                        "The system does not support verification of state changes for "
                        "the magnetometer calibration command."
                    )
            else:
                self.logger.error(
                    "Failed to acknowledge reception of the magnetometer calibration "
                    "command sent to Agent ({msg.target_system}, {msg.target_comp})"
                )
                msg_code = responses.ACK_FAILURE

            if msg.retry and not ack:
                if self.__retry_msg_send(
                    msg,
                    self.__message_senders[msg.msg_type][function_id],
                    device_exists,
                ):
                    ack = True
                    msg_code = responses.SUCCESS

            msg.response = msg_code
            msg.message_result_event.notify(context=msg.context)

            return ack

        @self.__send_message("GROUND_PRESSURE_CALIBRATION")
        @self.__timer()
        def sender(
            self,
            msg: swarm_msgs.PreflightCalibrationMsg,
            connection: Connection,
            function_id: int = 0,
            device_exists: bool = False,
        ) -> bool:
            """
            Perform a ground pressure calibration on the selected agent

            :param msg: Calibration message
            :type msg: PreflightCalibrationMsg

            :param function_id: The index of the method in the message type function handler
                list, defaults to 0
            :type function_id: int, optional

            :param device_exists: Flag indicating whether the device that the message
                is intended for exists in the network, defaults to False
            :type device_exists: bool, optional

            :return: Indicates whether or not the message was successfully sent
            :rtype: bool
            """
            connection.mavlink_connection.mav.command_long_send(
                msg.target_system,
                msg.target_comp,
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
            msg_code = responses.ACK_FAILURE

            if self.__ack_msg("COMMAND_ACK", timeout=msg.ack_timeout):
                self.logger.info(
                    "Successfully acknowledged reception of the ground pressure "
                    f"calibration command sent to Agent ({msg.target_system}, "
                    f"{msg.target_comp})"
                )
                ack = True
                msg_code = responses.SUCCESS

                if device_exists:
                    self.logger.info(
                        "The system does not support verification of state changes for "
                        "the ground pressure calibration command."
                    )
            else:
                self.logger.error(
                    "Failed to acknowledge reception of the ground pressure "
                    f"calibration command sent to Agent ({msg.target_system}, "
                    f"{msg.target_comp})"
                )
                msg_code = responses.ACK_FAILURE

            if msg.retry and not ack:
                if self.__retry_msg_send(
                    msg,
                    self.__message_senders[msg.msg_type][function_id],
                    device_exists,
                ):
                    ack = True
                    msg_code = responses.SUCCESS

            msg.response = msg_code
            msg.message_result_event.notify(context=msg.context)

            return ack

        @self.__send_message("AIRSPEED_CALIBRATION")
        @self.__timer()
        def sender(
            self,
            msg: swarm_msgs.PreflightCalibrationMsg,
            connection: Connection,
            function_id: int = 0,
            device_exists: bool = False,
        ) -> bool:
            """
            Perform airspeed calibration on the selected agent

            :param msg: Calibration message
            :type msg: PreflightCalibrationMsg

            :param function_id: The index of the method in the message type function handler
                list, defaults to 0
            :type function_id: int, optional

            :param device_exists: Flag indicating whether the device that the message
                is intended for exists in the network, defaults to False
            :type device_exists: bool, optional

            :return: Indicates whether or not the message was successfully sent
            :rtype: bool
            """
            connection.mavlink_connection.mav.command_long_send(
                msg.target_system,
                msg.target_comp,
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
            msg_code = responses.ACK_FAILURE

            if self.__ack_msg("COMMAND_ACK", timeout=msg.ack_timeout):
                self.logger.info(
                    "Successfully acknowledged reception of the airspeed calibration "
                    f"command sent to Agent ({msg.target_system}, "
                    f"{msg.target_comp})"
                )
                ack = True
                msg_code = responses.SUCCESS

                if device_exists:
                    self.logger.info(
                        "The system does not support verification of state changes for "
                        "the airspeed calibration command."
                    )
            else:
                self.logger.error(
                    "Failed to acknowledge reception of the airspeed calibration "
                    f"command sent to Agent ({msg.target_system}, "
                    f"{msg.target_comp})"
                )
                msg_code = responses.ACK_FAILURE

            if msg.retry and not ack:
                if self.__retry_msg_send(
                    msg,
                    self.__message_senders[msg.msg_type][function_id],
                    device_exists,
                ):
                    ack = True
                    msg_code = responses.SUCCESS

            msg.response = msg_code
            msg.message_result_event.notify(context=msg.context)

            return ack

        @self.__send_message("BAROMETER_TEMPERATURE_CALIBRATION")
        @self.__timer()
        def sender(
            self,
            msg: swarm_msgs.PreflightCalibrationMsg,
            connection: Connection,
            function_id: int = 0,
            device_exists: bool = False,
        ) -> bool:
            """
            Perform a barometer temperature calibration on the selected agent

            :param msg: Calibration message
            :type msg: PreflightCalibrationMsg

            :param function_id: The index of the method in the message type function handler
                list, defaults to 0
            :type function_id: int, optional

            :param device_exists: Flag indicating whether the device that the message
                is intended for exists in the network, defaults to False
            :type device_exists: bool, optional

            :return: Indicates whether or not the message was successfully sent
            :rtype: bool
            """
            connection.mavlink_connection.mav.command_long_send(
                msg.target_system,
                msg.target_comp,
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
            msg_code = responses.ACK_FAILURE

            if self.__ack_msg("COMMAND_ACK", timeout=msg.ack_timeout):
                self.logger.info(
                    "Successfully acknowledged reception of the barometer "
                    f"temperature calibration command sent to Agent "
                    f"({msg.target_system}, {msg.target_comp})"
                )
                ack = True
                msg_code = responses.SUCCESS

                if device_exists:
                    self.logger.info(
                        "The system does not support verification of state changes for "
                        "the barometer temperature calibration command."
                    )
            else:
                self.logger.error(
                    "Failed to acknowledge reception of the barometer temperature "
                    f"calibration command sent to Agent ({msg.target_system}, "
                    f"{msg.target_comp})"
                )
                msg_code = responses.ACK_FAILURE

            if msg.retry and not ack:
                if self.__retry_msg_send(
                    msg,
                    self.__message_senders[msg.msg_type][function_id],
                    device_exists,
                ):
                    ack = True
                    msg_code = responses.SUCCESS

            msg.response = msg_code
            msg.message_result_event.notify(context=msg.context)

            return ack

        @self.__send_message("FLIGHT_MODE")
        @self.__timer()
        def sender(
            self,
            msg: swarm_msgs.FlightModeMsg,
            connection: Connection,
            function_id: int = 0,
            device_exists: bool = False,
        ) -> bool:
            """
            Set the flight mode of an agent.

            :param msg: Flight mode message
            :type msg: FlightModeMsg

            :param function_id: The index of the method in the message type function
                handler list, defaults to 0
            :type function_id: int, optional

            :param device_exists: Flag indicating whether the device that the message
                is intended for exists in the network, defaults to False
            :type device_exists: bool, optional

            :return: Indicates whether or not the message was successfully sent
            :rtype: bool
            """
            # Reset target
            connection.mavlink_connection.target_system = msg.target_system
            connection.mavlink_connection.target_component = msg.target_comp

            # Verify that the flight mode is supported by the agent
            if msg.flight_mode not in connection.mavlink_connection.mode_mapping():
                self.logger.error(
                    f"The desired flight mode, {msg.flight_mode}, is not a supported "
                    "flight mode. Supported flight modes include: "
                    f"{connection.mavlink_connection.mode_mapping().keys()}"
                )
                msg.response = responses.INVALID_PROPERTIES
                msg.message_result_event.notify(context=msg.context)

                return False

            # Send flight mode
            connection.mavlink_connection.set_mode(
                connection.mavlink_connection.mode_mapping()[msg.flight_mode]
            )

            ack = False
            msg_code = responses.ACK_FAILURE

            if self.__ack_msg("COMMAND_ACK", timeout=msg.ack_timeout):
                self.logger.info(
                    "Successfully acknowledged reception of the flight mode "
                    f"{msg.flight_mode} command sent to Agent ({msg.target_system}, "
                    f"{msg.target_comp})"
                )
                ack = True
                msg_code = responses.SUCCESS

                if device_exists:
                    start_time = time.time()

                    while (
                        self.__devices[
                            (msg.target_system, msg.target_comp)
                        ].flight_mode.value
                        != msg.flight_mode
                    ):
                        if time.time() - start_time >= msg.state_timeout:
                            ack = False
                            break
                    if ack:
                        self.logger.info(
                            f"Successfully verified that Agent ({msg.target_system}, "
                            f"{msg.target_comp}) switched to the {msg.flight_mode} "
                            "flight mode"
                        )
                    else:
                        self.logger.error(
                            f"Failed to verify that Agent ({msg.target_system}, "
                            f"{msg.target_comp}) switched to the {msg.flight_mode} "
                            "flight mode"
                        )
                        msg_code = responses.STATE_VALIDATION_FAILURE
            else:
                self.logger.error(
                    "Failed to acknowledge reception of the flight mode "
                    f"{msg.flight_mode} command sent to Agent ({msg.target_system}, "
                    f"{msg.target_comp})"
                )
                msg_code = responses.ACK_FAILURE

            if msg.retry and not ack:
                if self.__retry_msg_send(
                    msg,
                    self.__message_senders[msg.msg_type][function_id],
                    device_exists,
                ):
                    ack = True
                    msg_code = responses.SUCCESS

            msg.response = msg_code
            msg.message_result_event.notify(context=msg.context)

            return ack

        @self.__send_message("FLIGHT_SPEED")
        @self.__timer()
        def sender(
            self,
            msg: swarm_msgs.FlightSpeedMsg,
            connection: Connection,
            function_id: int = 0,
            device_exists: bool = False,
        ) -> bool:
            """
            Set an agent's flight speed.

            :param msg: Speed message
            :type msg: FlightSpeedMsg

            :param function_id: The index of the method in the message type function
                handler list, defaults to 0
            :type function_id: int, optional

            :param device_exists: Flag indicating whether the device that the message
                is intended for exists in the network, defaults to False
            :type device_exists: bool, optional

            :return: Indicates whether or not the message was successfully sent
            :rtype: bool
            """
            connection.mavlink_connection.mav.command_long_send(
                msg.target_system,
                msg.target_comp,
                mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
                0,
                msg.speed_type,
                msg.speed,
                -1,
                0,
                0,
                0,
                0,
            )
            ack = False
            msg_code = responses.ACK_FAILURE

            if self.__ack_msg("COMMAND_ACK", timeout=msg.ack_timeout):
                self.logger.info(
                    "Successfully acknowledged reception of the flight speed command "
                    f"{msg.speed_type} sent to Agent ({msg.target_system}, "
                    f"{msg.target_comp})"
                )
                ack = True
                msg_code = responses.SUCCESS

                if device_exists:
                    self.logger.info(
                        "The system does not support verification of state changes for "
                        "the flight speed commands."
                    )
            else:
                self.logger.error(
                    "Failed to acknowledge reception of the flight speed command "
                    f"{msg.speed_type} sent to Agent ({msg.target_system}, "
                    f"{msg.target_comp})"
                )
                msg_code = responses.ACK_FAILURE

            if msg.retry and not ack:
                if self.__retry_msg_send(
                    msg,
                    self.__message_senders[msg.msg_type][function_id],
                    device_exists,
                ):
                    ack = True
                    msg_code = responses.SUCCESS

            msg.response = msg_code
            msg.message_result_event.notify(context=msg.context)

            return ack

        @self.__send_message("simpletakeoff")
        @self.__timer()
        def sender(
            self,
            msg: swarm_msgs.TakeoffMsg,
            connection: Connection,
            function_id: int = 0,
            device_exists: bool = False,
        ) -> bool:
            """
            Perform a simple takeoff command (just takeoff to a set altitude)
            Note that acknowledgement of this command does not indicate that the
            altitude was reached, but rather that the system will attempt to reach
            the specified altitude

            :param msg: Takeoff message
            :type msg: TakeoffMsg

            :param function_id: The index of the method in the message type function handler
                list, defaults to 0
            :type function_id: int, optional

            :param device_exists: Flag indicating whether the device that the message
                is intended for exists in the network, defaults to False
            :type device_exists: bool, optional

            :return: Indicates whether or not the message was successfully sent
            :rtype: bool
            """
            if msg.altitude < 0 or math.isinf(msg.altitude) or math.isnan(msg.altitude):
                self.logger.exception(
                    f"An invalid takeoff altitude was provided ({msg.altitude}). "
                    "Please send a valid takeoff altitude"
                )
                msg.response = responses.INVALID_PROPERTIES
                msg.message_result_event.notify(context=msg.context)
                return False

            connection.mavlink_connection.mav.command_long_send(
                msg.target_system,
                msg.target_comp,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                msg.altitude,
            )
            ack = False
            msg_code = responses.ACK_FAILURE

            if self.__ack_msg("COMMAND_ACK", timeout=msg.ack_timeout):
                self.logger.info(
                    "Successfully acknowledged reception of the simple takeoff "
                    f"command sent to Agent ({msg.target_system}, "
                    f"{msg.target_comp})"
                )
                ack = True
                msg_code = responses.SUCCESS

                if device_exists:
                    self.logger.info(
                        "The system does not support verification of state changes for "
                        "the simple takeoff command."
                    )
            else:
                self.logger.error(
                    "Failed to acknowledge reception of the simple takeoff command "
                    f"sent to Agent ({msg.target_system}, {msg.target_comp})"
                )
                msg_code = responses.ACK_FAILURE

            if msg.retry and not ack:
                if self.__retry_msg_send(
                    msg,
                    self.__message_senders[msg.msg_type][function_id],
                    device_exists,
                ):
                    ack = True
                    msg_code = responses.SUCCESS

            msg.response = msg_code
            msg.message_result_event.notify(context=msg.context)

            return ack

        @self.__send_message("takeoff")
        @self.__timer()
        def sender(
            self,
            msg: swarm_msgs.TakeoffMsg,
            connection: Connection,
            function_id: int = 0,
            device_exists: bool = False,
        ) -> bool:
            """
            Perform a takeoff command (use lat, lon, and alt)
            Note that acknowledgement of this command does not indicate that the
            altitude was reached, but rather that the system will attempt to reach
            the specified altitude

            :param msg: Speed message
            :type msg: TakeoffMsg

            :param function_id: The index of the method in the message type function handler
                list, defaults to 0
            :type function_id: int, optional

            :param device_exists: Flag indicating whether the device that the message
                is intended for exists in the network, defaults to False
            :type device_exists: bool, optional

            :return: Indicates whether or not the message was successfully sent
            :rtype: bool
            """
            if msg.altitude < 0 or math.isinf(msg.altitude) or math.isnan(msg.altitude):
                self.logger.exception(
                    f"An invalid takeoff altitude was provided ({msg.altitude}). "
                    "Please send a valid takeoff altitude"
                )
                msg.response = responses.INVALID_PROPERTIES
                msg.message_result_event.notify(context=msg.context)
                return False

            connection.mavlink_connection.mav.command_long_send(
                msg.target_system,
                msg.target_comp,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                0,
                0,
                0,
                0,
                0,
                msg.latitude,
                msg.longitude,
                msg.altitude,
            )
            ack = False
            msg_code = responses.ACK_FAILURE

            if self.__ack_msg("COMMAND_ACK", timeout=msg.ack_timeout):
                self.logger.info(
                    "Successfully acknowledged reception of the takeoff command sent "
                    f"to Agent ({msg.target_system}, {msg.target_comp})"
                )
                ack = True
                msg_code = responses.SUCCESS

                if device_exists:
                    self.logger.info(
                        "The system does not support verification of state changes for "
                        "the takeoff command."
                    )
            else:
                self.logger.error(
                    f"Failed to acknowledge reception of the takeoff command sent to "
                    f"Agent ({msg.target_system}, {msg.target_comp})"
                )
                msg_code = responses.ACK_FAILURE

            if msg.retry and not ack:
                if self.__retry_msg_send(
                    msg,
                    self.__message_senders[msg.msg_type][function_id],
                    device_exists,
                ):
                    ack = True
                    msg_code = responses.SUCCESS

            msg.response = msg_code
            msg.message_result_event.notify(context=msg.context)

            return ack

        @self.__send_message("simplefulltakeoff")
        @self.__timer()
        def sender(
            self,
            msg: swarm_msgs.TakeoffMsg,
            connection: Connection,
            function_id: int = 0,
            device_exists: bool = False,
        ) -> bool:
            """
            Command used to signal execution of a full simple takeoff sequence:
                1. Switch to GUIDED mode
                2. Arm
                3. Takeoff

            :param msg: Speed message
            :type msg: TakeoffMsg

            :param function_id: The index of the method in the message type function handler
                list, defaults to 0
            :type function_id: int, optional

            :param device_exists: Flag indicating whether the device that the message
                is intended for exists in the network, defaults to False
            :type device_exists: bool, optional

            :return: Indicates whether or not the message was successfully sent
            :rtype: bool
            """
            # Create a new guided mode
            guided_msg = swarm_msgs.FlightModeMsg(
                supported_msgs.flight_modes.guided,
                msg.target_system,
                msg.target_comp,
                msg.retry,
                msg.msg_timeout,
                ack_timeout=msg.ack_timeout,
                state_timeout=msg.state_timeout,
                state_delay=msg.state_delay,
            )

            # Attempt to switch to GUIDED mode
            if not self.__send_seq_msg(guided_msg, device_exists):
                self.logger.error(
                    "Failed to acknowledge reception of the guided command stage "
                    "within the simple full takeoff command sent to Agent "
                    f"({msg.target_system}, {msg.target_comp}). Full takeoff "
                    "sequence failed."
                )
                msg.response = responses.SEQUENCE_STAGE_FAILURE
                msg.message_result_event.notify(context=msg.context)
                return False

            # Create a new arming message to send
            arm_msg = swarm_msgs.SystemCommandMsg(
                supported_msgs.system_commands.arm,
                msg.target_system,
                msg.target_comp,
                msg.retry,
                msg.msg_timeout,
                ack_timeout=msg.ack_timeout,
                state_timeout=msg.state_timeout,
                state_delay=msg.state_delay,
            )

            # Attempt to arm the system
            if not self.__send_seq_msg(arm_msg, device_exists):
                self.logger.error(
                    "Failed to acknowledge reception of the arm command stage within "
                    "the simple full takeoff command sent to Agent "
                    f"({msg.target_system}, {msg.target_comp}). Full takeoff "
                    "sequence failed."
                )
                msg.response = responses.SEQUENCE_STAGE_FAILURE
                msg.message_result_event.notify(context=msg.context)
                return False

            # Give the agent a chance to fully arm
            time.sleep(msg.state_delay)

            # Reset the message type to be a simple takeoff command
            msg.msg_type = supported_msgs.mission_commands.simple_takeoff

            # Attempt to perform takeoff
            if not self.__send_seq_msg(msg, device_exists):
                self.logger.error(
                    "Failed to acknowledge reception of the takeoff command stage "
                    "within the simple full takeoff command sent to Agent "
                    f"({msg.target_system}, {msg.target_comp}). Full takeoff "
                    "sequence failed."
                )
                msg.response = responses.SEQUENCE_STAGE_FAILURE
                msg.message_result_event.notify(context=msg.context)
                return False

            self.logger.info(
                "Successfully sent and acknowledged all steps in the simple full "
                "takeoff command sequence."
            )

            msg.context.update({"response": responses.SUCCESS})
            msg.message_result_event.notify(context=msg.context)

            return True

        @self.__send_message("fulltakeoff")
        @self.__timer()
        def sender(
            self,
            msg: swarm_msgs.TakeoffMsg,
            connection: Connection,
            function_id: int = 0,
            device_exists: bool = False,
        ) -> bool:
            """
            Command used to signal execution of a full takeoff sequence:
                1. Switch to GUIDED mode
                2. Arm
                3. Takeoff

            :param msg: Speed message
            :type msg: TakeoffMsg

            :param function_id: The index of the method in the message type function handler
                list, defaults to 0
            :type function_id: int, optional

            :param device_exists: Flag indicating whether the device that the message
                is intended for exists in the network, defaults to False
            :type device_exists: bool, optional

            :return: Indicates whether or not the message was successfully sent
            :rtype: bool
            """
            # Create a new guided mode
            guided_msg = swarm_msgs.FlightModeMsg(
                supported_msgs.flight_modes.guided,
                msg.target_system,
                msg.target_comp,
                msg.retry,
                msg.msg_timeout,
                ack_timeout=msg.ack_timeout,
                state_timeout=msg.state_timeout,
                state_delay=msg.state_delay,
            )

            # Attempt to switch to GUIDED mode
            if not self.__send_seq_msg(guided_msg, device_exists):
                self.logger.error(
                    "Failed to acknowledge reception of the guided command stage "
                    "within the full takeoff command sent to Agent "
                    f"({msg.target_system}, {msg.target_comp}). Full takeoff "
                    "sequence failed."
                )
                msg.response = responses.SEQUENCE_STAGE_FAILURE
                msg.message_result_event.notify(context=msg.context)
                return False

            # Create a new arming message to send
            arm_msg = swarm_msgs.SystemCommandMsg(
                supported_msgs.system_commands.arm,
                msg.target_system,
                msg.target_comp,
                msg.retry,
                msg.msg_timeout,
                ack_timeout=msg.ack_timeout,
                state_timeout=msg.state_timeout,
                state_delay=msg.state_delay,
            )

            # Attempt to arm the system
            if not self.__send_seq_msg(arm_msg, device_exists):
                self.logger.error(
                    "Failed to acknowledge reception of the arm command stage within "
                    f"the full takeoff command sent to Agent ({msg.target_system}, "
                    f"{msg.target_comp}). Full takeoff sequence failed."
                )
                msg.response = responses.SEQUENCE_STAGE_FAILURE
                msg.message_result_event.notify(context=msg.context)
                return False

            # Give the agent a chance to fully arm
            time.sleep(msg.state_delay)

            # Reset the message type to be a simple takeoff command
            msg.msg_type = supported_msgs.mission_commands.takeoff

            # Attempt to perform takeoff
            if not self.__send_seq_msg(msg, device_exists):
                self.logger.error(
                    "Failed to acknowledge reception of the takeoff command stage "
                    "within the full takeoff command sent to Agent "
                    f"({msg.target_system}, {msg.target_comp}). Full takeoff sequence "
                    "failed."
                )
                msg.response = responses.SEQUENCE_STAGE_FAILURE
                msg.message_result_event.notify(context=msg.context)
                return False

            self.logger.info(
                "Successfully sent and acknowledged all steps in the full takeoff "
                "command sequence."
            )
            msg.response = responses.SUCCESS
            msg.message_result_event.notify(context=msg.context)

            return True

        @self.__send_message("simplewaypoint")
        @self.__timer()
        def sender(
            self,
            msg: swarm_msgs.WaypointMsg,
            connection: Connection,
            function_id: int = 0,
            device_exists: bool = False,
        ) -> bool:
            """
            Perform a simple waypoint command (just lat, lon, and alt)

            NOTE: Acknowledgement of this command does not indicate that the
            waypoint was reached, but rather that the system will attempt to reach
            the specified waypoint

            :param msg: Waypoint message
            :type msg: WaypointMsg

            :param function_id: The index of the method in the message type function handler
                list, defaults to 0
            :type function_id: int, optional

            :param device_exists: Flag indicating whether the device that the message
                is intended for exists in the network, defaults to False
            :type device_exists: bool, optional

            :return: Indicates whether or not the message was successfully sent
            :rtype: bool
            """
            if msg.altitude < 0 or math.isinf(msg.altitude) or math.isnan(msg.altitude):
                self.logger.exception(
                    f"An invalid takeoff altitude was provided ({msg.altitude}). "
                    "Please send a valid waypoint altitude"
                )
                msg.response = responses.INVALID_PROPERTIES
                msg.message_result_event.notify(context=msg.context)
                return False

            connection.mavlink_connection.mav.mission_item_send(
                msg.target_system,
                msg.target_comp,
                0,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0,
                0,
                0,
                0,
                0,
                0,
                msg.latitude,
                msg.longitude,
                msg.altitude,
            )
            ack = False
            msg_code = responses.ACK_FAILURE

            if self.__ack_msg("COMMAND_ACK", timeout=msg.ack_timeout):
                self.logger.info(
                    "Successfully acknowledged reception of the simple waypoint "
                    f"command sent to Agent ({msg.target_system}, "
                    f"{msg.target_comp})"
                )
                ack = True
                msg_code = responses.SUCCESS

                if device_exists:
                    self.logger.info(
                        "The system does not support verification of state changes for "
                        "the simple waypoint command."
                    )
            else:
                self.logger.error(
                    f"Failed to acknowledge reception of the simple waypoint command "
                    f"sent to Agent ({msg.target_system}, {msg.target_comp})"
                )
                msg_code = responses.ACK_FAILURE

            if msg.retry and not ack:
                if self.__retry_msg_send(
                    msg,
                    self.__message_senders[msg.msg_type][function_id],
                    device_exists,
                ):
                    ack = True
                    msg_code = responses.SUCCESS

            msg.response = msg_code
            msg.message_result_event.notify(context=msg.context)

            return ack

        @self.__send_message("waypoint")
        @self.__timer()
        def sender(
            self,
            msg: swarm_msgs.WaypointMsg,
            connection: Connection,
            function_id: int = 0,
            device_exists: bool = False,
        ) -> bool:
            """
            Perform a waypoint navigation command

            NOTE: Acknowledgement of this command does not indicate that the
            waypoint was reached, but rather that the system will attempt to reach
            the specified waypoint

            :param msg: Waypoint message
            :type msg: WaypointMsg

            :param function_id: The index of the method in the message type function handler
                list, defaults to 0
            :type function_id: int, optional

            :param device_exists: Flag indicating whether the device that the message
                is intended for exists in the network, defaults to False
            :type device_exists: bool, optional

            :return: Indicates whether or not the message was successfully sent
            :rtype: bool
            """
            if msg.altitude < 0 or math.isinf(msg.altitude) or math.isnan(msg.altitude):
                self.logger.exception(
                    f"An invalid takeoff altitude was provided ({msg.altitude}). "
                    "Please send a valid waypoint altitude"
                )
                msg.response = responses.INVALID_PROPERTIES
                msg.message_result_event.notify(context=msg.context)
                return False

            connection.mavlink_connection.mav.mission_item_send(
                msg.target_system,
                msg.target_comp,
                0,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0,
                0,
                msg.hold,
                msg.accept_radius,
                msg.pass_radius,
                msg.yaw,
                msg.latitude,
                msg.longitude,
                msg.altitude,
            )
            ack = False
            msg_code = responses.ACK_FAILURE

            if self.__ack_msg("COMMAND_ACK", timeout=msg.ack_timeout):
                self.logger.info(
                    "Successfully acknowledged reception of the waypoint command "
                    f"sent to Agent ({msg.target_system}, {msg.target_comp})"
                )
                ack = True
                msg_code = responses.SUCCESS

                if device_exists:
                    self.logger.info(
                        "The system does not support verification of state changes for "
                        "the waypoint command."
                    )
            else:
                self.logger.error(
                    "Failed to acknowledge reception of the waypoint command sent to "
                    f"Agent ({msg.target_system}, {msg.target_comp})"
                )
                msg_code = responses.ACK_FAILURE

            if msg.retry and not ack:
                if self.__retry_msg_send(
                    msg,
                    self.__message_senders[msg.msg_type][function_id],
                    device_exists,
                ):
                    ack = True
                    msg_code = responses.SUCCESS

            msg.response = msg_code
            msg.message_result_event.notify(context=msg.context)

            return ack

        @self.__send_message("gethomeposition")
        @self.__timer()
        def sender(
            self,
            msg: swarm_msgs.AgentMsg,
            connection: Connection,
            function_id: int = 0,
            device_exists: bool = False,
        ) -> bool:
            """
            Get the current home position of an agent

            :param msg: Get home position message
            :type msg: AgentMsg

            :param function_id: The index of the method in the message type function handler
                list, defaults to 0
            :type function_id: int, optional

            :param device_exists: Flag indicating whether the device that the message
                is intended for exists in the network, defaults to False
            :type device_exists: bool, optional

            :return: Indicates whether or not the message was successfully sent
            :rtype: bool
            """
            connection.mavlink_connection.mav.command_long_send(
                msg.target_system,
                msg.target_comp,
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
            msg_code = responses.ACK_FAILURE

            if self.__ack_msg("COMMAND_ACK", timeout=msg.ack_timeout):
                self.logger.info(
                    "Successfully acknowledged reception of the get home position "
                    f"command sent to Agent ({msg.target_system}, "
                    f"{msg.target_comp})"
                )
                ack = True
                msg_code = responses.SUCCESS

                if device_exists:
                    self.logger.info(
                        "The system does not support verification of state changes for "
                        "the get home position command."
                    )
            else:
                self.logger.error(
                    "Failed to acknowledge reception of the get home position command "
                    f"sent to Agent ({msg.target_system}, {msg.target_comp})"
                )
                msg_code = responses.ACK_FAILURE

            if msg.retry and not ack:
                if self.__retry_msg_send(
                    msg,
                    self.__message_senders[msg.msg_type][function_id],
                    device_exists,
                ):
                    ack = True
                    msg_code = responses.SUCCESS

            msg.response = msg_code
            msg.message_result_event.notify(context=msg.context)

            return ack

        @self.__send_message("resethomecurrent")
        @self.__timer()
        def sender(
            self,
            msg: swarm_msgs.HomePositionMsg,
            connection: Connection,
            function_id: int = 0,
            device_exists: bool = False,
        ) -> bool:
            """
            Reset the saved home position of an agent to the current position

            NOTE: Validation of this command can take a while. To ensure that the system
            has sufficient time to verify that the home position was properly reset, it
            may be necessary to extend the timeout periods

            :param msg: Reset home position message
            :type msg: HomePositionMsg

            :param function_id: The index of the method in the message type function handler
                list, defaults to 0
            :type function_id: int, optional

            :param device_exists: Flag indicating whether the device that the message
                is intended for exists in the network, defaults to False
            :type device_exists: bool, optional

            :return: Indicates whether or not the message was successfully sent
            :rtype: bool
            """
            device_id = (msg.target_system, msg.target_comp)

            if device_exists:
                current_home_pos = self.__devices[device_id].home_position

                self.logger.info(
                    "The initial home position prior to home location reset is as "
                    f"follows: Latitude: {current_home_pos.latitude}, Longitude: "
                    f"{current_home_pos.longitude}, Altitude: "
                    f"{current_home_pos.altitude}"
                )

            # Set the home position to the current location
            connection.mavlink_connection.mav.command_long_send(
                msg.target_system,
                msg.target_comp,
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
            msg_code = responses.ACK_FAILURE

            if self.__ack_msg("COMMAND_ACK", timeout=msg.ack_timeout):
                self.logger.info(
                    "Successfully acknowledged reception of the reset home position "
                    f"to current position command sent to Agent "
                    f"({msg.target_system}, {msg.target_comp})"
                )
                ack = True
                msg_code = responses.SUCCESS

                if device_exists:
                    start_time = time.time()

                    while vars(self.__devices[device_id].home_position) == vars(
                        current_home_pos
                    ):
                        # Signal an update state command
                        update_home_state_msg = swarm_msgs.AgentMsg(
                            supported_msgs.mission_commands.get_home_position,
                            msg.target_system,
                            msg.target_comp,
                            msg.retry,
                            msg.msg_timeout,
                            msg.ack_timeout,
                            msg.state_timeout,
                            msg.state_delay,
                        )

                        # Get the updated home position
                        self.__send_seq_msg(update_home_state_msg, device_exists)

                        if time.time() - start_time >= msg.state_timeout:
                            ack = False
                            break
                    if ack:
                        self.logger.info(
                            "Successfully the reset the home position of Agent "
                            f"({msg.target_system}, {msg.target_comp}) to: "
                            "Latitude: "
                            f"{self.__devices[device_id].home_position.latitude}, "
                            "Longitude: "
                            f"{self.__devices[device_id].home_position.longitude}, "
                            f"Altitude: "
                            f"{self.__devices[device_id].home_position.altitude}"
                        )
                    else:
                        self.logger.error(
                            "Failed to reset the home position of Agent "
                            f"({msg.target_system}, {msg.target_comp}) or the current "
                            "location of the agent has not changed since last updating "
                            "the home position."
                        )
                        msg_code = responses.STATE_VALIDATION_FAILURE
            else:
                self.logger.error(
                    "Failed to acknowledge reception of the reset home position to "
                    f"current position command sent to Agent ({msg.target_system}, "
                    f"{msg.target_comp})"
                )
                msg_code = responses.ACK_FAILURE

            if msg.retry and not ack:
                if self.__retry_msg_send(
                    msg,
                    self.__message_senders[msg.msg_type][function_id],
                    device_exists,
                ):
                    ack = True
                    msg_code = responses.SUCCESS

            msg.response = msg_code
            msg.message_result_event.notify(context=msg.context)

            return ack

        @self.__send_message("resethome")
        @self.__timer()
        def sender(
            self,
            msg: swarm_msgs.HomePositionMsg,
            connection: Connection,
            function_id: int = 0,
            device_exists: bool = False,
        ) -> bool:
            """
            Reset the saved home position of an agent to the desired position

            :param msg: Reset home position message
            :type msg: HomePositionMsg

            :param function_id: The index of the method in the message type function handler
                list, defaults to 0
            :type function_id: int, optional

            :param device_exists: Flag indicating whether the device that the message
                is intended for exists in the network, defaults to False
            :type device_exists: bool, optional

            :return: Indicates whether or not the message was successfully sent
            :rtype: bool
            """
            if msg.longitude is None or msg.latitude is None or msg.altitude is None:
                self.logger.exception(
                    "Cannot reset the home location to the given location unless the "
                    "latitude, longitude, and altitude are all provided."
                )
                msg.response = responses.INVALID_PROPERTIES
                msg.message_result_event.notify(context=msg.context)
                return False

            if msg.altitude < 0.0 or msg.altitude > 150.0:
                self.logger.exception(
                    "An invalid home position altitude was provided "
                    f"({msg.altitude}). Please set a valid altitude"
                )
                msg.response = responses.INVALID_PROPERTIES
                msg.message_result_event.notify(context=msg.context)
                return False

            device_id = (msg.target_system, msg.target_comp)

            if device_exists:
                current_home_pos = self.__devices[device_id]

                self.logger.info(
                    "The initial home position prior to home location reset is "
                    f"as follows: Latitude: {current_home_pos.latitude}, Longitude: "
                    f"{current_home_pos.longitude}, Altitude: "
                    f"{current_home_pos.altitude}"
                )

            # Set the home position to the desired location
            connection.mavlink_connection.mav.command_long_send(
                msg.target_system,
                msg.target_comp,
                mavutil.mavlink.MAV_CMD_DO_SET_HOME,
                0,
                0,
                0,
                0,
                0,
                msg.latitude,
                msg.longitude,
                msg.altitude,
            )

            ack = False
            msg_code = responses.ACK_FAILURE

            if self.__ack_msg("COMMAND_ACK", timeout=msg.ack_timeout):
                self.logger.info(
                    "Successfully acknowledged reception of the reset home position "
                    f"command sent to Agent ({msg.target_system}, "
                    f"{msg.target_comp})"
                )
                ack = True
                msg_code = responses.SUCCESS

                if device_exists:
                    start_time = time.time()

                    while (
                        self.__devices[device_id].home_position.latitude != msg.latitude
                        and self.__devices[device_id].home_position.longitude
                        != msg.longitude
                        and self.__devices[device_id].home_position.altitude
                        != msg.altitude
                    ):
                        # Signal an update state command
                        update_home_state_msg = swarm_msgs.AgentMsg(
                            supported_msgs.mission_commands.get_home_position,
                            msg.target_system,
                            msg.target_comp,
                            msg.retry,
                            msg.msg_timeout,
                            msg.ack_timeout,
                            msg.state_timeout,
                            msg.state_delay,
                        )

                        # Get the updated home position
                        self.__send_seq_msg(update_home_state_msg, device_exists)

                        if time.time() - start_time >= msg.state_timeout:
                            ack = False
                            break
                    if ack:
                        self.logger.info(
                            "Successfully the reset the home position of Agent "
                            f"({msg.target_system}, {msg.target_comp}) to: "
                            "Latitude: "
                            f"{self.__devices[device_id].home_position.latitude}, "
                            "Longitude: "
                            f"{self.__devices[device_id].home_position.longitude}, "
                            f"Altitude: "
                            f"{self.__devices[device_id].home_position.altitude}"
                        )
                    else:
                        self.logger.error(
                            "Failed to reset the home position of Agent "
                            f"({msg.target_system}, {msg.target_comp})"
                        )
                        msg_code = responses.STATE_VALIDATION_FAILURE
            else:
                self.logger.error(
                    "Failed to acknowledge reception of the reset home position "
                    f"command sent to Agent ({msg.target_system}, "
                    f"{msg.target_comp})"
                )
                msg_code = responses.ACK_FAILURE

            if msg.retry and not ack:
                if self.__retry_msg_send(
                    msg,
                    self.__message_senders[msg.msg_type][function_id],
                    device_exists,
                ):
                    ack = True
                    msg_code = responses.SUCCESS

            msg.response = msg_code
            msg.message_result_event.notify(context=msg.context)

            return ack

        return

    @property
    def senders(self) -> dict:
        """
        Methods responsible for handling message sending.

        :rtype: dict
        """
        return self.__senders

    def __init_logger(self, name: str, log_level: int = logging.INFO) -> logging.Logger:
        """
        Initialize the logger with the desired debug levels

        :param name: The name of the logger
        :type name: str

        :param log_level: The log level to display, defaults to logging.INFO
        :type log_level: int, optional

        :return: A newly configured logger
        :rtype: logging.Logger
        """
        logging.basicConfig()
        logger = logging.getLogger(name)
        logger.setLevel(log_level)
        return logger

    def __retry_msg_send(
        self, msg: Any, function: Callable, device_exists: bool
    ) -> bool:
        """
        Retry a message send until the an acknowledgement is received or a timeout
        occurs

        :param msg: The message to retry sending
        :type msg: Any

        :param function: The function to call using the message
        :type function: function

        :return: Indicate whether the retry was successful
        :rtype: bool
        """
        ack = False
        start_time = time.time()

        # Don't let the message come back here and create an infinite loop
        msg.retry = False

        while time.time() - start_time <= msg.msg_timeout:
            # Reattempt the message send
            if function(self, msg, device_exists=device_exists):
                ack = True
                break

        return ack

    def __ack_msg(
        self, msg_type: str, connection: Connection, timeout=1.0
    ) -> Tuple[bool, Any]:
        """
        Helper method used to ensure that a distributed msg is acknowledged

        :param msg_type: The type of message that should be acknowledged
        :type msg_type: str

        :param timeout: The acceptable time period before the acknowledgement is
            considered timed out, defaults to 1.0
        :type timeout: float, optional

        :return: _description_
        :rtype: Tuple[bool, Any]
        """
        # Attempt to acquire the mutex
        if not connection.read_message_mutex.acquire(timeout=1.0):
            return False

        # Flag indicating whether the message was acknowledged
        ack_success = False

        # Start acknowledgement timer
        start_t = time.time()

        while time.time() - start_t < timeout:
            # Read a new message
            try:
                ack_msg = connection.mavlink_connection.recv_match(
                    type=msg_type, blocking=False
                )
                ack_msg = ack_msg.to_dict()

                if ack_msg["mavpackettype"] == msg_type:
                    ack_success = True
                    break
            except mavutil.mavlink.MAVError:
                self.__logger.debug("An error occurred on MAVLink message reception")
            except AttributeError:
                # Catch errors with converting the message to a dict
                pass
            except Exception:
                # Log any other unexpected exception
                self.__logger.exception(
                    "Exception while receiving message: ", exc_info=False
                )

        # Continue reading status messages
        connection.read_message_mutex.release()

        return ack_success, ack_msg

    def __send_seq_msg(self, msg: Any, device_exists: bool) -> bool:
        """
        Helper function used to handle calling all of the message handlers.
        This method is used by the sequence commands such as the full takeoff
        command to provide indication of a function execution result.

        NOTE: THIS IS USED. DO NOT DELETE

        :param msg: The message to send
        :type msg: Any

        :param device_exists: Flag indicating whether the given device exists in the
            network
        :type device_exists: bool

        :return: Indicates whether all of the message senders for a given message
            successfully sent their respective message
        :rtype: bool
        """
        # Helper function used to send the desired command
        if msg.msg_type in self.__senders:
            for function_id, function in enumerate(self.__senders[msg.msg_type]):
                try:
                    if not function(
                        self, msg, function_id=function_id, device_exists=device_exists
                    ):
                        return False
                except Exception:
                    pass

        return True

    def __send_message(self, msg: Union[list, str]) -> Callable:
        """
        Decorator used to create a sender for a mavlink message

        :param msg: The message type to connect to the sender
        :type msg: Union[list, str]

        :return: decorator
        :rtype: Callable
        """

        def decorator(function: Callable):
            if msg not in self.__senders:
                self.__senders[msg] = []

            if function not in self.__senders[msg]:
                self.__senders[msg].append(function)

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
