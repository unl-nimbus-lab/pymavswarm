import math
import time
import atexit
import logging
import monotonic
import threading
from pymavswarm.msg import *
from pymavlink import mavutil
from pymavswarm.state import *
from pymavswarm.agent import Agent
from pymavswarm.event import Event
from pymavswarm.msg import responses
from pymavswarm.param import Parameter
from typing import Any, Callable, Tuple, Union
from pymavlink.dialects.v10 import ardupilotmega


class Connection:
    """
    Handles all interaction with the network and the MAVLink master device.
    """

    def __init__(
        self,
        port: str,
        baud: int,
        source_system: int = 255,
        source_component: int = 0,
        agent_timeout: float = 30.0,
        log_level: int = logging.INFO,
    ) -> None:
        """
        :param port: The port over which a connection should be established
        :type port: str

        :param baud: The baudrate that a connection should be established with
        :type baud: int

        :param source_system: The system ID of the connection, defaults to 255
        :type source_system: int, optional

        :param source_component: The component ID of the connection, defaults to 0
        :type source_component: int, optional

        :param agent_timeout: The amount of time allowable between agent heartbeats
            before an agent is considered timed out, defaults to 30.0
        :type agent_timeout: float, optional

        :param log_level: The log level of the connection logger, defaults to
            logging.INFO
        :type debug: int, optional

        :raises TimeoutError: The system was unable to establish a connection
        """

        self.logger = self.__init_logger("connection", log_level=log_level)

        # Create a new mavlink connection
        self.master = mavutil.mavlink_connection(
            port,
            baud=baud,
            source_system=source_system,
            source_component=source_component,
            autoreconnect=True,
        )

        # Ensure that a connection has been successfully established
        # Integrate a 2 second timeout
        resp = self.master.wait_heartbeat(timeout=2)

        if resp is None:
            raise TimeoutError(
                "The system was unable to establish a connection with the specified"
                "device within the timeout period"
            )

        # Class variables
        self.__connected = True
        self.__devices = {}
        self.__device_list_changed = Event()

        # Message Handlers
        self.__message_listeners = {}
        self.__message_senders = {}

        # Mutexes
        self.__read_msg_mutex = threading.Lock()
        self.__send_msg_mutex = threading.Lock()

        # Register the exit callback
        atexit.register(self.disconnect)

        # Threads
        self.__heartbeat_t = threading.Thread(target=self.__heartbeat)
        self.__heartbeat_t.daemon = True

        self.__incoming_msg_t = threading.Thread(target=self.__incoming_msg_handler)
        self.__incoming_msg_t.daemon = True

        """
        Message Listeners
        """

        @self.on_message(["HEARTBEAT"])
        def listener(self, msg: Any) -> None:
            """
            Register new devices or update the timeout status of existing agents

            :param msg: Incoming MAVLink message
            :type msg: Any
            """
            # Make sure that the message isn't from a GCS
            if msg.get_type() == mavutil.mavlink.MAV_TYPE_GCS:
                return

            sys_id = msg.get_srcSystem()
            comp_id = msg.get_srcComponent()

            device_id = (sys_id, comp_id)

            # Create a new device assigned the respective sysid:compid pair
            device = Agent(sys_id, comp_id, timeout_period=agent_timeout)

            # If the device hasn't been seen before, save it
            if device_id not in self.__devices:
                self.__devices[device_id] = device
                self.__device_list_changed.notify(agent=device)
            else:
                # The connection has been restored
                if self.__devices[device_id].timeout.value:
                    self.logger.info(
                        f"Connection to device {sys_id}:{comp_id} has been restored"
                    )

            # Update the last heartbeat variable
            self.__devices[device_id].last_heartbeat.value = monotonic.monotonic()

            self.__devices[device_id].timeout.value = False

            return

        @self.on_message(["HEARTBEAT"])
        def listener(self, msg: Any) -> None:
            """
            Handle general device information contained within a heartbeat

            :param msg: Incoming MAVLink message
            :type msg: Any
            """
            # Ignore messages sent by a GCS
            if msg.type == mavutil.mavlink.MAV_TYPE_GCS:
                return

            # Get the system ID and component ID
            sys_id = msg.get_srcSystem()
            comp_id = msg.get_srcComponent()

            # Create a new tuple key
            device_tuple = (sys_id, comp_id)

            # Let the heartbeat implementation handle this
            if not device_tuple in self.__devices:
                return

            self.__devices[device_tuple].armed.value = (
                msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            ) != 0

            self.__devices[device_tuple].system_status.value = msg.system_status
            self.__devices[device_tuple].vehicle_type.value = msg.type

            # Update the last heartbeat
            self.__devices[device_tuple].last_heartbeat.value = monotonic.monotonic()

            try:
                # NOTE: We assume that ArduPilot will be used
                self.__devices[
                    device_tuple
                ].flight_mode.value = mavutil.mode_mapping_bynumber(msg.type)[
                    msg.custom_mode
                ]
            except Exception:
                # We received an invalid message
                pass

            return

        @self.on_message(["GLOBAL_POSITION_INT"])
        def listener(self, msg: Any) -> None:
            """
            Handle the a GPS position message

            :param msg: Incoming MAVLink message
            :type msg: Any
            """
            # Get the system ID and component ID
            sys_id = msg.get_srcSystem()
            comp_id = msg.get_srcComponent()

            # Create a new tuple key
            device_tuple = (sys_id, comp_id)

            # Let the heartbeat implementation handle this
            if not device_tuple in self.__devices:
                return

            # Update the device velocity
            if self.__devices[device_tuple].velocity is None:
                velocity = Velocity(msg.vx / 100, msg.vy / 100, msg.vz / 100)
                self.__devices[device_tuple].velocity = velocity
            else:
                self.__devices[device_tuple].velocity.vx = msg.vx / 100
                self.__devices[device_tuple].velocity.vy = msg.vy / 100
                self.__devices[device_tuple].velocity.vz = msg.vz / 100

            # Update the device location
            if self.__devices[device_tuple].location is None:
                loc = Location(
                    msg.lat / 1.0e7, msg.lon / 1.0e7, msg.relative_alt / 1000
                )
                self.__devices[device_tuple].location = loc
            else:
                self.__devices[device_tuple].location.latitude = msg.lat / 1.0e7
                self.__devices[device_tuple].location.longitude = msg.lon / 1.0e7
                self.__devices[device_tuple].location.altitude = msg.relative_alt / 1000

            return

        @self.on_message(["ATTITUDE"])
        def listener(self, msg: Any) -> None:
            """
            Handle an agent attitude message

            :param msg: Incoming MAVLink message
            :type msg: Any
            """
            # Get the system ID and component ID
            sys_id = msg.get_srcSystem()
            comp_id = msg.get_srcComponent()

            # Create a new tuple key
            device_tuple = (sys_id, comp_id)

            # Let the heartbeat implementation handle this
            if not device_tuple in self.__devices:
                return

            # Update the respective devices attitude
            if self.__devices[device_tuple].attitude is None:
                att = Attitude(
                    msg.pitch,
                    msg.yaw,
                    msg.roll,
                    msg.pitchspeed,
                    msg.yawspeed,
                    msg.rollspeed,
                )
                self.__devices[device_tuple].attitude = att
            else:
                self.__devices[device_tuple].attitude.pitch = msg.pitch
                self.__devices[device_tuple].attitude.roll = msg.roll
                self.__devices[device_tuple].attitude.yaw = msg.yaw
                self.__devices[device_tuple].attitude.pitch_speed = msg.pitchspeed
                self.__devices[device_tuple].attitude.roll_speed = msg.rollspeed
                self.__devices[device_tuple].attitude.yaw_speed = msg.yawspeed

            return

        @self.on_message(["SYS_STATUS"])
        def listener(self, msg: Any) -> None:
            """
            Handle the system status message containing battery state

            :param msg: Incoming MAVLink message
            :type msg: Any
            """
            # Get the system ID and component ID
            sys_id = msg.get_srcSystem()
            comp_id = msg.get_srcComponent()

            # Create a new tuple key
            device_tuple = (sys_id, comp_id)

            # Let the heartbeat implementation handle this
            if not device_tuple in self.__devices:
                return

            # Update the battery information
            if self.__devices[device_tuple].battery is None:
                batt = Battery(
                    msg.voltage_battery, msg.current_battery, msg.battery_remaining
                )
                self.__devices[device_tuple].battery = batt
            else:
                self.__devices[device_tuple].battery.voltage = msg.voltage_battery
                self.__devices[device_tuple].battery.current = msg.current_battery
                self.__devices[device_tuple].battery.level = msg.battery_remaining

            return

        @self.on_message(["GPS_RAW_INT"])
        def listener(self, msg: Any) -> None:
            """
            Handle the GPS status information

            :param msg: Incoming MAVLink message
            :type msg: Any
            """
            # Get the system ID and component ID
            sys_id = msg.get_srcSystem()
            comp_id = msg.get_srcComponent()

            # Create a new tuple key
            device_tuple = (sys_id, comp_id)

            # Let the heartbeat implementation handle this
            if not device_tuple in self.__devices:
                return

            # Read the GPS status information
            if self.__devices[device_tuple].gps_info is None:
                info = GPSInfo(msg.eph, msg.epv, msg.fix_type, msg.satellites_visible)
                self.__devices[device_tuple].gps_info = info
            else:
                self.__devices[device_tuple].gps_info.eph = msg.eph
                self.__devices[device_tuple].gps_info.epv = msg.epv
                self.__devices[device_tuple].gps_info.fix_type = msg.fix_type
                self.__devices[
                    device_tuple
                ].gps_info.satellites_visible = msg.satellites_visible

            return

        @self.on_message(["EKF_STATUS_REPORT"])
        def listener(self, msg: Any) -> None:
            """
            Handle an EKF status message

            :param msg: Incoming MAVLink message
            :type msg: Any
            """
            # Get the system ID and component ID
            sys_id = msg.get_srcSystem()
            comp_id = msg.get_srcComponent()

            # Create a new tuple key
            device_tuple = (sys_id, comp_id)

            # Let the heartbeat implementation handle this
            if not device_tuple in self.__devices:
                return

            # Read the EKF Status information
            if self.__devices[device_tuple].ekf is None:
                ekf = EKFStatus(
                    msg.velocity_variance,
                    msg.pos_horiz_variance,
                    msg.pos_vert_variance,
                    msg.compass_variance,
                    msg.terrain_alt_variance,
                    (msg.flags & ardupilotmega.EKF_POS_HORIZ_ABS) > 0,
                    (msg.flags & ardupilotmega.EKF_CONST_POS_MODE) > 0,
                    (msg.flags & ardupilotmega.EKF_PRED_POS_HORIZ_ABS) > 0,
                )
                self.__devices[device_tuple].ekf = ekf
            else:
                # Read variance properties
                self.__devices[
                    device_tuple
                ].ekf.velocity_variance = msg.velocity_variance
                self.__devices[
                    device_tuple
                ].ekf.pos_horiz_variance = msg.pos_horiz_variance
                self.__devices[
                    device_tuple
                ].ekf.pos_vert_variance = msg.pos_vert_variance
                self.__devices[device_tuple].ekf.compass_variance = msg.compass_variance
                self.__devices[
                    device_tuple
                ].ekf.terrain_alt_variance = msg.terrain_alt_variance

                # Read flags
                self.__devices[device_tuple].ekf.pos_horiz_abs = (
                    msg.flags & ardupilotmega.EKF_POS_HORIZ_ABS
                ) > 0
                self.__devices[device_tuple].ekf.const_pos_mode = (
                    msg.flags & ardupilotmega.EKF_CONST_POS_MODE
                ) > 0
                self.__devices[device_tuple].ekf.pred_pos_horiz_abs = (
                    msg.flags & ardupilotmega.EKF_PRED_POS_HORIZ_ABS
                ) > 0

            return

        @self.on_message(["ATTITUDE"])
        def listener(self, msg: Any) -> None:
            """
            Handle an agent attitude message

            :param msg: Incoming MAVLink message
            :type msg: Any
            """
            # Get the system ID and component ID
            sys_id = msg.get_srcSystem()
            comp_id = msg.get_srcComponent()

            # Create a new tuple key
            device_tuple = (sys_id, comp_id)

            # Let the heartbeat implementation handle this
            if not device_tuple in self.__devices:
                return

            # Update the respective devices attitude
            if self.__devices[device_tuple].attitude is None:
                att = Attitude(
                    msg.pitch,
                    msg.yaw,
                    msg.roll,
                    msg.pitchspeed,
                    msg.yawspeed,
                    msg.rollspeed,
                )
                self.__devices[device_tuple].attitude = att
            else:
                self.__devices[device_tuple].attitude.pitch = msg.pitch
                self.__devices[device_tuple].attitude.roll = msg.roll
                self.__devices[device_tuple].attitude.yaw = msg.yaw
                self.__devices[device_tuple].attitude.pitch_speed = msg.pitchspeed
                self.__devices[device_tuple].attitude.roll_speed = msg.rollspeed
                self.__devices[device_tuple].attitude.yaw_speed = msg.yawspeed

            return

        @self.on_message(["HOME_POSITION"])
        def listener(self, msg: Any) -> None:
            """
            Handle the home position message

            NOTE: The altitude is provided in MSL, NOT AGL

            :param msg: Incoming MAVLink message
            :type msg: Any
            """
            # Get the system ID and component ID
            sys_id = msg.get_srcSystem()
            comp_id = msg.get_srcComponent()

            # Create a new tuple key
            device_tuple = (sys_id, comp_id)

            # Let the heartbeat implementation handle this
            if not device_tuple in self.__devices:
                return

            # Update the device home location
            if self.__devices[device_tuple].home_position is None:
                loc = Location(
                    msg.latitude / 1.0e7, msg.longitude / 1.0e7, msg.altitude / 1000
                )
                self.__devices[device_tuple].home_position = loc
            else:
                self.__devices[device_tuple].home_position.latitude = (
                    msg.latitude / 1.0e7
                )
                self.__devices[device_tuple].home_position.longitude = (
                    msg.longitude / 1.0e7
                )
                self.__devices[device_tuple].home_position.altitude = (
                    msg.altitude / 1000
                )

            return

        @self.send_message(["ARM"])
        def sender(
            self,
            msg: SystemCommandMsg,
            function_id: int = 0,
            device_exists: bool = False,
        ) -> bool:
            """
            Arm an agent

            :param msg: Arming message
            :type msg: SystemCommandMsg

            :param function_id: The index of the method in the message type function handler
                list, defaults to 0
            :type function_id: int, optional

            :param device_exists: Flag indicating whether the device that the message
                is intended for exists in the network, defaults to False
            :type device_exists: bool, optional

            :return: Indicates whether the message was sent successfully
            :rtype: bool
            """
            self.master.mav.command_long_send(
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

        @self.send_message(["DISARM"])
        def sender(
            self,
            msg: SystemCommandMsg,
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
            self.master.mav.command_long_send(
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

        @self.send_message(["KILL"])
        def sender(
            self,
            msg: SystemCommandMsg,
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
            self.master.mav.command_long_send(
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

        @self.send_message(["REBOOT"])
        def sender(
            self,
            msg: SystemCommandMsg,
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
            self.master.mav.command_long_send(
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

        @self.send_message(["SHUTDOWN"])
        def sender(
            self,
            msg: SystemCommandMsg,
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
            self.master.mav.command_long_send(
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

        @self.send_message(["ACCELEROMETER_CALIBRATION"])
        def sender(
            self,
            msg: PreflightCalibrationMsg,
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
            self.master.mav.command_long_send(
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

        @self.send_message(["SIMPLE_ACCELEROMETER_CALIBRATION"])
        def sender(
            self,
            msg: PreflightCalibrationMsg,
            function_id: int = 0,
            device_exists: bool = False,
        ) -> bool:
            """
            Perform a simple accelerometer calibration on the selected agent

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
            self.master.mav.command_long_send(
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

        @self.send_message(["AHRS_TRIM"])
        def sender(
            self,
            msg: PreflightCalibrationMsg,
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
            self.master.mav.command_long_send(
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

        @self.send_message(["GYROSCOPE_CALIBRATION"])
        def sender(
            self,
            msg: PreflightCalibrationMsg,
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
            self.master.mav.command_long_send(
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

        @self.send_message(["MAGNETOMETER_CALIBRATION"])
        def sender(
            self,
            msg: PreflightCalibrationMsg,
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
            self.master.mav.command_long_send(
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

        @self.send_message(["GROUND_PRESSURE_CALIBRATION"])
        def sender(
            self,
            msg: PreflightCalibrationMsg,
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
            self.master.mav.command_long_send(
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

        @self.send_message(["AIRSPEED_CALIBRATION"])
        def sender(
            self,
            msg: PreflightCalibrationMsg,
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
            self.master.mav.command_long_send(
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

        @self.send_message(["BAROMETER_TEMPERATURE_CALIBRATION"])
        def sender(
            self,
            msg: PreflightCalibrationMsg,
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
            self.master.mav.command_long_send(
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

        @self.send_message(["FLIGHT_MODE"])
        def sender(
            self, msg: FlightModeMsg, function_id: int = 0, device_exists: bool = False
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
            self.master.target_system = msg.target_system
            self.master.target_component = msg.target_comp

            # Verify that the flight mode is supported by the agent
            if msg.flight_mode not in self.master.mode_mapping():
                self.logger.error(
                    f"The desired flight mode, {msg.flight_mode}, is not a supported "
                    "flight mode. Supported flight modes include: "
                    f"{self.master.mode_mapping().keys()}"
                )
                msg.response = responses.INVALID_PROPERTIES
                msg.message_result_event.notify(context=msg.context)

                return False

            # Send flight mode
            self.master.set_mode(self.master.mode_mapping()[msg.flight_mode])

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

        @self.send_message(["HRL_COMMAND"])
        def sender(
            self, msg: HRLMsg, function_id: int = 0, device_exists: bool = False
        ) -> bool:
            """
            Send an HRL command to the swarm.

            :param msg: HRL message
            :type msg: HRLMsg

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
            self.master.target_system = msg.target_system
            self.master.target_component = msg.target_comp

            # Send flight mode
            self.master.mav.named_value_int_send(
                int(time.time()), str.encode("hrl-state-arg"), msg.hrl_command
            )

            ack = False
            msg_code = responses.ACK_FAILURE

            if self.__ack_msg("COMMAND_ACK", timeout=msg.ack_timeout):
                self.logger.info(
                    "Successfully acknowledged reception of the HRL command "
                    f"{msg.hrl_command} sent to Agent ({msg.target_system}, "
                    f"{msg.target_comp})"
                )
                ack = True
                msg_code = responses.SUCCESS

                if device_exists:
                    start_time = time.time()

                    while (
                        not self.__devices[
                            (msg.target_system, msg.target_comp)
                        ].hrl_state.value
                        != msg.hrl_command
                    ):
                        if time.time() - start_time >= msg.state_timeout:
                            ack = False
                            break
                    if ack:
                        self.logger.info(
                            f"Successfully verified that Agent ({msg.target_system}, "
                            f"{msg.target_comp}) executed HRL command {msg.hrl_command}"
                        )
                    else:
                        self.logger.error(
                            f"Failed to verify that Agent ({msg.target_system}, "
                            f"{msg.target_comp}) executed HRL command {msg.hrl_command}"
                        )
                        msg_code = responses.STATE_VALIDATION_FAILURE
            else:
                self.logger.error(
                    "Failed to acknowledge reception of the HRL command "
                    f"{msg.hrl_command} sent to Agent ({msg.target_system}, "
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

        @self.send_message(["FLIGHT_SPEED"])
        def sender(
            self, msg: FlightSpeedMsg, function_id: int = 0, device_exists: bool = False
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
            self.master.mav.command_long_send(
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

        @self.send_message(["simpletakeoff"])
        def sender(
            self, msg: TakeoffMsg, function_id: int = 0, device_exists: bool = False
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

            self.master.mav.command_long_send(
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

        @self.send_message(["takeoff"])
        def sender(
            self, msg: TakeoffMsg, function_id: int = 0, device_exists: bool = False
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

            self.master.mav.command_long_send(
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

        @self.send_message(["simplefulltakeoff"])
        def sender(
            self, msg: TakeoffMsg, function_id: int = 0, device_exists: bool = False
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
            guided_msg = FlightModeMsg(
                SupportedMsgs.flight_modes.guided,
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
            arm_msg = SystemCommandMsg(
                SupportedMsgs.system_commands.arm,
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
            msg.msg_type = SupportedMsgs.mission_commands.simple_takeoff

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

        @self.send_message(["fulltakeoff"])
        def sender(
            self, msg: TakeoffMsg, function_id: int = 0, device_exists: bool = False
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
            guided_msg = FlightModeMsg(
                SupportedMsgs.flight_modes.guided,
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
            arm_msg = SystemCommandMsg(
                SupportedMsgs.system_commands.arm,
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
            msg.msg_type = SupportedMsgs.mission_commands.takeoff

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

        @self.send_message(["simplewaypoint"])
        def sender(
            self, msg: WaypointMsg, function_id: int = 0, device_exists: bool = False
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

            self.master.mav.mission_item_send(
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

        @self.send_message(["waypoint"])
        def sender(
            self, msg: WaypointMsg, function_id: int = 0, device_exists: bool = False
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

            self.master.mav.mission_item_send(
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

        @self.send_message(["gethomeposition"])
        def sender(
            self, msg: AgentMsg, function_id: int = 0, device_exists: bool = False
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
            self.master.mav.command_long_send(
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

        @self.send_message(["resethomecurrent"])
        def sender(
            self,
            msg: HomePositionMsg,
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
            self.master.mav.command_long_send(
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
                        update_home_state_msg = AgentMsg(
                            SupportedMsgs.mission_commands.get_home_position,
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

        @self.send_message(["resethome"])
        def sender(
            self,
            msg: HomePositionMsg,
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
            self.master.mav.command_long_send(
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
                        update_home_state_msg = AgentMsg(
                            SupportedMsgs.mission_commands.get_home_position,
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

    @property
    def connected(self) -> bool:
        """
        Flag indicating whether the system currently has a connection

        :rtype: bool
        """
        return self.connected

    @property
    def devices(self) -> dict:
        """
        The current set of recognized devices

        The dict keys are (system ID, component ID) tuples

        :rtype: dict
        """
        return self.__devices

    @property
    def device_list_changed(self) -> Event:
        """
        Event indicating that the list of devices had a new device added or removed

        :rtype: Event
        """
        return self.__device_list_changed

    def on_message(self, msg: Union[list, str]):
        """
        Decorator used to create a listener for a mavlink message
        This implementation has been inspired by the following source:
            * Project: Dronekit
            * Repository: dronekit
            * URL: https://github.com/dronekit/dronekit-python

        :param msg: The type of message to watch for
        :type msg: Union[list, str]
        """

        def decorator(function: Callable):
            if isinstance(msg, list):
                for sub_msg in msg:
                    self.add_message_listener(sub_msg, function)
            else:
                self.add_message_listener(msg, function)

        return decorator

    def add_message_listener(self, msg: str, function: Callable) -> None:
        """
        Add a new function to the dictionary of message listeners
        This implementation has been inspired by the following source:
            * Project: Dronekit
            * Repository: dronekit
            * URL: https://github.com/dronekit/dronekit-python

        :param msg: The message type to bind the function to
        :type msg: str
        :param function: The function to connect
        :type function: function
        """
        if msg not in self.__message_listeners:
            self.__message_listeners[msg] = []

        if function not in self.__message_listeners[msg]:
            self.__message_listeners[msg].append(function)

        return

    def send_message(self, msg: Union[list, str]):
        """
        Decorator used to create a sender for a mavlink message

        :param msg: The message type to connect to the sender
        :type msg: Union[list, str]
        """

        def decorator(function: Callable):
            if isinstance(msg, list):
                for sub_msg in msg:
                    self.add_message_sender(sub_msg, function)
            else:
                self.add_message_sender(msg, function)

        return decorator

    def add_message_sender(self, msg: str, function: Callable) -> None:
        """
        Add a new function to the dictionary of message senders

        :param msg: The message type to connect to the sender
        :type msg: str

        :param function: The function to connect
        :type function: function
        """
        if msg not in self.__message_senders:
            self.__message_senders[msg] = []

        if function not in self.__message_senders[msg]:
            self.__message_senders[msg].append(function)

        return

    def __heartbeat(self) -> None:
        """
        Function used to sent a heartbeat to the network indicating that the GCS
        is still operating
        """
        while self.__connected:
            self.master.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0,
                0,
                0,
            )

            # Send a heartbeat every 2 seconds
            time.sleep(2)

        return

    def __incoming_msg_handler(self) -> None:
        """
        Handle incoming messages and distribute them to their respective handlers
        """
        while self.__connected:
            # Update the timeout flag for each device
            for device in self.__devices.values():
                if device.last_heartbeat.value is not None:
                    device.timeout.value = (
                        monotonic.monotonic() - device.last_heartbeat.value
                    ) >= device.timeout_period.value

            # Read a new message
            try:
                if not self.__read_msg_mutex.acquire(timeout=1.0):
                    msg = None
                else:
                    msg = self.master.recv_msg()
                    self.__read_msg_mutex.release()
            except mavutil.mavlink.MAVError:
                self.logger.debug("An error occurred on MAVLink message reception")
                msg = None
            except Exception:
                # Log any other unexpected exception
                self.logger.exception(
                    "Exception while receiving message: ", exc_info=True
                )
                msg = None

            if not msg:
                continue

            # Apply the respective message handler(s)
            if msg.get_type() in self.__message_listeners:
                for function in self.__message_listeners[msg.get_type()]:
                    try:
                        function(self, msg)
                    except Exception:
                        self.logger.exception(
                            f"Exception in message handler for {msg.get_type()}",
                            exc_info=True,
                        )

        return

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

    def __retry_param_send(self, param: Any, function: Callable) -> bool:
        """
        Retry a parameter send until the an acknowledgement is received or a timeout
        occurs

        :param param: The parameter to retry sending
        :type msg: Any

        :param function: The function to call using the message
        :type function: function

        :return: Indicate whether the retry was successful
        :rtype: bool
        """
        ack = False
        start_time = time.time()

        # Don't let the message come back here and create an infinite loop
        param.retry = False

        while time.time() - start_time <= param.msg_timeout:
            # Reattempt the message send
            if function(param):
                ack = True
                break

        return ack

    def __ack_msg(self, msg_type: str, timeout=1.0) -> Tuple[bool, Any]:
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
        if not self.__read_msg_mutex.acquire(timeout=1.0):
            return False

        # Flag indicating whether the message was acknowledged
        ack_success = False

        # Start acknowledgement timer
        start_t = time.time()

        while time.time() - start_t < timeout:
            # Read a new message
            try:
                ack_msg = self.master.recv_match(type=msg_type, blocking=False)
                ack_msg = ack_msg.to_dict()

                if ack_msg["mavpackettype"] == msg_type:
                    ack_success = True
                    break
            except mavutil.mavlink.MAVError:
                self.logger.debug("An error occurred on MAVLink message reception")
            except AttributeError:
                # Catch errors with converting the message to a dict
                pass
            except Exception:
                # Log any other unexpected exception
                self.logger.exception(
                    "Exception while receiving message: ", exc_info=False
                )

        # Continue reading status messages
        self.__read_msg_mutex.release()

        return ack_success, ack_msg

    def send_msg_handler(self, msg: Any) -> None:
        """
        Public method that is accessed by the mavswarm interface to signal the handler
        to complete message sending

        :param msg: The message to send
        :type msg: Any
        """
        # Make sure that a connection is established before attempting to send a message
        if self.__connected:
            handler_t = threading.Thread(target=self.__send_msg_handler, args=(msg,))

            # Send the message
            handler_t.start()

        return

    def send_msg_package_handler(self, package: MsgPackage) -> None:
        """
        Public method that is accessed by the mavswarm interface to signal the handler
        to send a message package

        :param package: Package of messages to send
        :type package: MsgPackage
        """
        # Make sure that a connection is established before attempting to send a message
        if self.__connected:
            handler_t = threading.Thread(
                target=self.__send_msg_package_handler, args=(package,)
            )

            # Send the message
            handler_t.start()

        return

    def __send_msg_package_handler(self, package: MsgPackage) -> None:
        """
        Helper function used to send each message in a package and to
        manage each message's success/failure

        :param package: Package of messages to send
        :type package: MsgPackage
        """

        def get_msg_success(msg: Any) -> list:
            msg_results = self.__send_msg_handler(msg)

            # Overall message success is a failure if any of the functions fails
            msg_success = True
            for result in msg_results:
                if not result[2]:
                    msg_success = False

            return msg_success

        # Initial attempt at sending each message in the package
        for msg in package.msgs:
            if not get_msg_success(msg):
                package.msgs_failed.append(msg)
            else:
                package.msgs_succeeded.append(msg)

        # Retry sending the failed messages
        if package.retry and len(package.msgs_failed) > 0:
            for _ in range(package.max_retry_attempts):
                # Only retry sending the failed messages
                if len(package.msgs_failed) > 0:
                    for msg in package.msgs_failed:
                        if get_msg_success(msg):
                            package.msgs_failed.remove(msg)
                            package.msgs_succeeded.append(msg)
                else:
                    break

        if len(package.msgs_failed) > 0:
            package.response = responses.SUCCESS
            package.package_result_event.notify(context=package.context)
        else:
            package.response = responses.PACKAGE_FAILURE
            package.package_result_event.notify(context=package.context)

        return

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
        if msg.msg_type in self.__message_senders:
            for function_id, function in enumerate(
                self.__message_senders[msg.msg_type]
            ):
                try:
                    if not function(
                        self, msg, function_id=function_id, device_exists=device_exists
                    ):
                        return False
                except Exception:
                    pass

        return True

    def __send_msg_handler(self, msg: Any) -> list:
        """
        Handle sending messages to the agents on the network

        :param msg: The message to send
        :type msg: Any
        """
        # List of the results for each function called by the message
        msg_function_results = []

        try:
            # Send the message if there is a message sender for it
            if msg.msg_type in self.__message_senders:
                device_exists = False

                if (msg.target_system, msg.target_comp) in self.__devices:
                    device_exists = True
                else:
                    self.logger.info(
                        "The current set of registered devices does not include Agent "
                        f"({msg.target_system, msg.target_comp}). The provided message "
                        "will still be sent; however, the system may not be able to "
                        "confirm reception of the message."
                    )

                for function_id, function in enumerate(
                    self.__message_senders[msg.msg_type]
                ):
                    # Prevent multiple sends from occurring at once
                    self.__send_msg_mutex.acquire()

                    # Result of the particular function called
                    function_result = False

                    # Execute the command
                    try:
                        function_result = function(
                            self,
                            msg,
                            function_id=function_id,
                            device_exists=device_exists,
                        )
                    except Exception:
                        self.logger.exception(
                            f"Exception in message sender for {msg.msg_type}",
                            exc_info=True,
                        )
                    finally:
                        msg_function_results.append(
                            (msg.msg_type, function_id, function_result)
                        )
                        self.__send_msg_mutex.release()
        except Exception:
            self.logger.exception(
                "An error occurred while attempting to send the provided message",
                exc_info=True,
            )

        return msg_function_results

    def set_param_handler(self, param: Parameter) -> None:
        """
        Set the value of a parameter on a given agent

        :param param: The parameter to set
        :type param: Parameter
        """
        # Make sure that a connection is established before attempting to set a param
        if self.__connected:
            handler_t = threading.Thread(target=self.__set_param_handler, args=(param,))

            # Set the parameter
            handler_t.start()

        return

    def __set_param_handler(self, param: Parameter) -> None:
        """
        Handle setting parameters on an agent in the network

        :param param: The parameter to set
        :type param: Parameter
        """
        # Prevent multiple sends from occurring at once
        self.__send_msg_mutex.acquire()

        try:
            self.__set_param(param)
        except Exception:
            self.logger.exception(
                "An error occurred while attempting to send the provided message",
                exc_info=True,
            )
        finally:
            self.__send_msg_mutex.release()

        return

    def __set_param(self, param: Parameter) -> bool:
        """
        Set the value of a parameter.

        NOTE: This sets the parameter value in RAM and not to EEPROM. Therefore, on
        reboot, the parameters will be reset to their default values

        :param param: The parameter to set
        :type param: Parameter

        :return: Indicates whether the parameter was successfully set
        :rtype: bool
        """
        try:
            # NOTE: In the current state, we only support float parameter value types
            #       Additional types may be added in the future
            self.master.mav.param_set_send(
                param.sys_id,
                param.comp_id,
                str.encode(param.param_id),
                param.param_value,
                9,
            )
        except Exception:
            self.logger.error(
                f"An error occurred while attempting to set {param.param_id} to "
                f"{param.param_value}",
                exc_info=True,
            )
            return False

        ack = False

        if self.__ack_msg("PARAM_VALUE", timeout=param.ack_timeout)[0]:
            ack = True
        else:
            if param.retry:
                if self.__retry_param_send(param, self.__set_param):
                    ack = True

        if ack:
            self.logger.info(
                f"Successfully set {param.param_id} to {param.param_value} on "
                f"Agent ({param.sys_id}, {param.comp_id})"
            )
        else:
            self.logger.error(
                f"Failed to set {param.param_id} to {param.param_value} on Agent "
                f"({param.sys_id}, {param.comp_id})"
            )

        return ack

    def read_param_handler(self, param: Parameter) -> None:
        """
        Read the value of a parameter

        :param param: The parameter to read
        :type param: Parameter
        """
        # Make sure that a connection is established before attempting to set a param
        if self.__connected:
            handler_t = threading.Thread(
                target=self.__read_param_handler, args=(param,)
            )

            # Send the message
            handler_t.start()

        return

    def __read_param_handler(self, param: Parameter) -> None:
        """
        Handler responsible for reading requested parameters.

        NOTE: This thread is primarily responsible for handling read requests and
        verifying that a read was accomplished on the message listener thread. The
        agent state itself is updated on the message listener thread

        :param param: The parameter to read
        :type param: Parameter
        """
        # Prevent multiple reads from occurring at once
        self.__send_msg_mutex.acquire()

        try:
            self.__read_param(param)
        except Exception:
            self.logger.exception(
                "An error occurred while attempting to send the provided message"
            )
        finally:
            self.__send_msg_mutex.release()

        return

    def __read_param(self, param: Parameter) -> bool:
        """
        Read a desired parameter value

        :param param: The parameter to read
        :type param: Parameter

        :return: Indicates whether the parameter was successfully read
        :rtype: bool
        """
        try:
            self.master.mav.param_request_read_send(
                param.sys_id, param.comp_id, str.encode(param.param_id), -1
            )
        except Exception:
            self.logger.exception(
                f"An exception occurred while attempting to read {param.param_id} "
                f"from Agent ({param.sys_id}, {param.comp_id})",
                exc_info=True,
            )
            return False

        ack = False

        ack, msg = self.__ack_msg("PARAM_VALUE", timeout=param.ack_timeout)

        if ack:
            read_param = ReadParameter(
                msg["param_id"],
                msg["param_value"],
                msg["param_type"],
                msg["param_index"],
                msg["param_count"],
            )

            self.__devices[(param.sys_id, param.comp_id)].last_params_read.append(
                read_param
            )
        else:
            if param.retry:
                if self.__retry_param_send(param, self.__read_param):
                    ack = True

        if ack:
            self.logger.info(
                f"Successfully read {param.param_id} from Agent ({param.sys_id}, "
                f"{param.comp_id}). Value: {msg}"
            )
        else:
            self.logger.error(
                f"Failed to read {param.param_id} from Agent ({param.sys_id}, "
                f"{param.comp_id})"
            )

        return ack

    def start_connection(self) -> None:
        """
        Helper method used to start non-mavlink related connection processes
        """
        self.__start_t()

        return

    def __start_t(self) -> None:
        """
        Start all threads available to the connection for message reception,
        message sending, and heartbeat handling
        """
        self.__heartbeat_t.start()
        self.__incoming_msg_t.start()

        return

    def __stop_t(self) -> None:
        """
        Join all threads
        """
        if self.__heartbeat_t is not None:
            self.__heartbeat_t.join()

        if self.__incoming_msg_t is not None:
            self.__incoming_msg_t.join()

        return

    def disconnect(self) -> None:
        """
        Close the connection and disconnect all threads
        """
        self.__connected = False
        self.__stop_t()
        self.__devices.clear()
        self.__device_list_changed.listeners.clear()

        if self.master is not None:
            self.master.close()

        return
