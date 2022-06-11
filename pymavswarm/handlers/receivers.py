import functools
import logging
import threading
from typing import Any, Callable, Union

import monotonic
from pymavlink import mavutil
from pymavlink.dialects.v10 import ardupilotmega

import pymavswarm.state as swarm_state
from pymavswarm import Agent, Connection


class Receivers:
    """
    Collection of methods responsible for processing incoming messages.
    """

    def __init__(
        self, logger_name: str = "receivers", log_level: int = logging.INFO
    ) -> None:
        """
        Constructor.

        :param logger_name: _description_, defaults to "receivers"
        :type logger_name: str, optional

        :param log_level: _description_, defaults to logging.INFO
        :type log_level: int, optional
        """
        self.__logger = self.__init_logger(logger_name, log_level=log_level)
        self.__receivers = {}

        @self.receive_message("HEARTBEAT")
        def listener(msg: Any, connection: Connection) -> None:
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

            # If the device hasn't been seen before, save it
            if device_id not in connection.devices:
                # Create and save a new device
                device = Agent(sys_id, comp_id, timeout_period=connection.agent_timeout)
                connection.devices[device_id] = device

                # Notify handlers that the devices changed
                connection.device_list_changed.notify(agent=device)
            else:
                # The connection has been restored
                if connection.devices[device_id].timeout.value:
                    self.__logger.info(
                        f"Connection to device {sys_id}:{comp_id} has been restored"
                    )

            # Update the last heartbeat variable
            connection.devices[device_id].last_heartbeat.value = monotonic.monotonic()
            connection.devices[device_id].timeout.value = False

            return

        @self.receive_message("HEARTBEAT")
        def listener(msg: Any, connection: Connection) -> None:
            """
            Handle general device information contained within a heartbeat

            :param msg: Incoming MAVLink message
            :type msg: Any
            """
            # Ignore messages sent by a GCS
            if msg.type == mavutil.mavlink.MAV_TYPE_GCS:
                return

            device_id = (msg.get_srcSystem(), msg.get_srcComponent())

            if not device_id in connection.devices:
                return

            connection.devices[device_id].armed.value = (
                msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            ) != 0

            connection.devices[device_id].system_status.value = msg.system_status
            connection.devices[device_id].vehicle_type.value = msg.type

            # Update the last heartbeat
            connection.devices[device_id].last_heartbeat.value = monotonic.monotonic()

            try:
                # NOTE: We assume that ArduPilot will be used
                connection.devices[
                    device_id
                ].flight_mode.value = mavutil.mode_mapping_bynumber(msg.type)[
                    msg.custom_mode
                ]
            except Exception:
                # We received an invalid message
                pass

            return

        @self.receive_message("GLOBAL_POSITION_INT")
        def listener(msg: Any, connection: Connection) -> None:
            """
            Handle the a GPS position message

            :param msg: Incoming MAVLink message
            :type msg: Any
            """
            device_id = (msg.get_srcSystem(), msg.get_srcComponent())

            if not device_id in connection.devices:
                return

            # Update the device velocity
            if connection.devices[device_id].velocity is None:
                velocity = swarm_state.Velocity(
                    msg.vx / 100, msg.vy / 100, msg.vz / 100
                )
                connection.devices[device_id].velocity = velocity
            else:
                connection.devices[device_id].velocity.vx = msg.vx / 100
                connection.devices[device_id].velocity.vy = msg.vy / 100
                connection.devices[device_id].velocity.vz = msg.vz / 100

            # Update the device location
            if connection.devices[device_id].location is None:
                loc = swarm_state.Location(
                    msg.lat / 1.0e7, msg.lon / 1.0e7, msg.relative_alt / 1000
                )
                connection.devices[device_id].location = loc
            else:
                connection.devices[device_id].location.latitude = msg.lat / 1.0e7
                connection.devices[device_id].location.longitude = msg.lon / 1.0e7
                connection.devices[device_id].location.altitude = (
                    msg.relative_alt / 1000
                )

            return

        @self.receive_message("ATTITUDE")
        def listener(msg: Any, connection: Connection) -> None:
            """
            Handle an agent attitude message

            :param msg: Incoming MAVLink message
            :type msg: Any
            """
            device_id = (msg.get_srcSystem(), msg.get_srcComponent())

            if not device_id in connection.devices:
                return

            # Update the respective devices attitude
            if connection.devices[device_id].attitude is None:
                att = swarm_state.Attitude(
                    msg.pitch,
                    msg.yaw,
                    msg.roll,
                    msg.pitchspeed,
                    msg.yawspeed,
                    msg.rollspeed,
                )
                connection.devices[device_id].attitude = att
            else:
                connection.devices[device_id].attitude.pitch = msg.pitch
                connection.devices[device_id].attitude.roll = msg.roll
                connection.devices[device_id].attitude.yaw = msg.yaw
                connection.devices[device_id].attitude.pitch_speed = msg.pitchspeed
                connection.devices[device_id].attitude.roll_speed = msg.rollspeed
                connection.devices[device_id].attitude.yaw_speed = msg.yawspeed

            return

        @self.receive_message("SYS_STATUS")
        def listener(msg: Any, connection: Connection) -> None:
            """
            Handle the system status message containing battery state

            :param msg: Incoming MAVLink message
            :type msg: Any
            """
            device_id = (msg.get_srcSystem(), msg.get_srcComponent())

            if not device_id in connection.devices:
                return

            # Update the battery information
            if connection.devices[device_id].battery is None:
                batt = swarm_state.Battery(
                    msg.voltage_battery, msg.current_battery, msg.battery_remaining
                )
                connection.devices[device_id].battery = batt
            else:
                connection.devices[device_id].battery.voltage = msg.voltage_battery
                connection.devices[device_id].battery.current = msg.current_battery
                connection.devices[device_id].battery.level = msg.battery_remaining

            return

        @self.receive_message("GPS_RAW_INT")
        def listener(msg: Any, connection: Connection) -> None:
            """
            Handle the GPS status information

            :param msg: Incoming MAVLink message
            :type msg: Any
            """
            device_id = (msg.get_srcSystem(), msg.get_srcComponent())

            if not device_id in connection.devices:
                return

            # Read the GPS status information
            if connection.devices[device_id].gps_info is None:
                info = swarm_state.GPSInfo(
                    msg.eph, msg.epv, msg.fix_type, msg.satellites_visible
                )
                connection.devices[device_id].gps_info = info
            else:
                connection.devices[device_id].gps_info.eph = msg.eph
                connection.devices[device_id].gps_info.epv = msg.epv
                connection.devices[device_id].gps_info.fix_type = msg.fix_type
                connection.devices[
                    device_id
                ].gps_info.satellites_visible = msg.satellites_visible

            return

        @self.receive_message("EKF_STATUS_REPORT")
        def listener(msg: Any, connection: Connection) -> None:
            """
            Handle an EKF status message

            :param msg: Incoming MAVLink message
            :type msg: Any
            """
            device_id = (msg.get_srcSystem(), msg.get_srcComponent())

            if not device_id in connection.devices:
                return

            # Read the EKF Status information
            if connection.devices[device_id].ekf is None:
                ekf = swarm_state.EKFStatus(
                    msg.velocity_variance,
                    msg.pos_horiz_variance,
                    msg.pos_vert_variance,
                    msg.compass_variance,
                    msg.terrain_alt_variance,
                    (msg.flags & ardupilotmega.EKF_POS_HORIZ_ABS) > 0,
                    (msg.flags & ardupilotmega.EKF_CONST_POS_MODE) > 0,
                    (msg.flags & ardupilotmega.EKF_PRED_POS_HORIZ_ABS) > 0,
                )
                connection.devices[device_id].ekf = ekf
            else:
                # Read variance properties
                connection.devices[
                    device_id
                ].ekf.velocity_variance = msg.velocity_variance
                connection.devices[
                    device_id
                ].ekf.pos_horiz_variance = msg.pos_horiz_variance
                connection.devices[
                    device_id
                ].ekf.pos_vert_variance = msg.pos_vert_variance
                connection.devices[
                    device_id
                ].ekf.compass_variance = msg.compass_variance
                connection.devices[
                    device_id
                ].ekf.terrain_alt_variance = msg.terrain_alt_variance

                # Read flags
                connection.devices[device_id].ekf.pos_horiz_abs = (
                    msg.flags & ardupilotmega.EKF_POS_HORIZ_ABS
                ) > 0
                connection.devices[device_id].ekf.const_pos_mode = (
                    msg.flags & ardupilotmega.EKF_CONST_POS_MODE
                ) > 0
                connection.devices[device_id].ekf.pred_pos_horiz_abs = (
                    msg.flags & ardupilotmega.EKF_PRED_POS_HORIZ_ABS
                ) > 0

            return

        @self.receive_message("ATTITUDE")
        def listener(msg: Any, connection: Connection) -> None:
            """
            Handle an agent attitude message

            :param msg: Incoming MAVLink message
            :type msg: Any
            """
            device_id = (msg.get_srcSystem(), msg.get_srcComponent())

            if not device_id in connection.devices:
                return

            # Update the respective device's attitude
            if connection.devices[device_id].attitude is None:
                att = swarm_state.Attitude(
                    msg.pitch,
                    msg.yaw,
                    msg.roll,
                    msg.pitchspeed,
                    msg.yawspeed,
                    msg.rollspeed,
                )
                connection.devices[device_id].attitude = att
            else:
                connection.devices[device_id].attitude.pitch = msg.pitch
                connection.devices[device_id].attitude.roll = msg.roll
                connection.devices[device_id].attitude.yaw = msg.yaw
                connection.devices[device_id].attitude.pitch_speed = msg.pitchspeed
                connection.devices[device_id].attitude.roll_speed = msg.rollspeed
                connection.devices[device_id].attitude.yaw_speed = msg.yawspeed

            return

        @self.receive_message("HOME_POSITION")
        def listener(msg: Any, connection: Connection) -> None:
            """
            Handle the home position message

            NOTE: The altitude is provided in MSL, NOT AGL

            :param msg: Incoming MAVLink message
            :type msg: Any
            """
            device_id = (msg.get_srcSystem(), msg.get_srcComponent())

            # Let the heartbeat implementation handle this
            if not device_id in connection.devices:
                return

            # Update the device home location
            if connection.devices[device_id].home_position is None:
                loc = swarm_state.Location(
                    msg.latitude / 1.0e7, msg.longitude / 1.0e7, msg.altitude / 1000
                )
                connection.devices[device_id].home_position = loc
            else:
                connection.devices[device_id].home_position.latitude = (
                    msg.latitude / 1.0e7
                )
                connection.devices[device_id].home_position.longitude = (
                    msg.longitude / 1.0e7
                )
                connection.devices[device_id].home_position.altitude = (
                    msg.altitude / 1000
                )

            return

        return

    @property
    def receivers(self) -> dict:
        """
        Methods used to handle incoming messages.

        :rtype: dict
        """
        return self.__receivers

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

    def receive_message(self, msg: Union[list, str]) -> Callable:
        """
        Decorator used to create a listener for a mavlink message
        This implementation has been inspired by the following source:
            * Project: Dronekit
            * Repository: dronekit
            * URL: https://github.com/dronekit/dronekit-python

        :param msg: The type of message to watch for
        :type msg: Union[list, str]

        :return: decorator
        :rtype: Callable
        """

        def decorator(function: Callable):
            if msg not in self.__receivers:
                self.__receivers[msg] = []

            if function not in self.__receivers[msg]:
                self.__receivers[msg].append(function)

        return decorator

    def synchronized(self, lock: threading.RLock) -> Callable:
        """
        Decorator used to wrap a method with a mutex lock and unlock

        :param lock: lock to use
        :type lock: threading.RLock

        :return: decorator
        :rtype: Callable
        """

        def decorator(function: Callable):
            @functools.wraps(function)
            def synchronized_function(*args, **kwargs):
                with lock:
                    return function(*args, **kwargs)

            return synchronized_function

        return decorator
