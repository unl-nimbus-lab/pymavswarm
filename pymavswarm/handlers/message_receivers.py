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
# pylint: disable=function-redefined

import logging
from typing import Any

import monotonic
from pymavlink import mavutil
from pymavlink.dialects.v10 import ardupilotmega

import pymavswarm.state as swarm_state
from pymavswarm import Agent, Connection
from pymavswarm.handlers.receivers import Receivers


class MessageReceivers(Receivers):
    """Collection of methods responsible for processing incoming messages."""

    def __init__(
        self, logger_name: str = "message-receivers", log_level: int = logging.INFO
    ) -> None:
        """
        Make a new receivers object.

        :param logger_name: logger name, defaults to "message-receivers"
        :type logger_name: str, optional

        :param log_level: log level, defaults to logging.INFO
        :type log_level: int, optional
        """
        super().__init__(logger_name, log_level)

        @self._receive_message("HEARTBEAT")
        @self._timer()
        def listener(message: Any, connection: Connection) -> None:
            """
            Register new agents or update the timeout status of existing agents.

            :param message: Incoming MAVLink message
            :type message: Any

            :param connection: MAVLink connection
            :type connection: Connection
            """
            # Make sure that the message isn't from a GCS
            if message.get_type() == mavutil.mavlink.MAV_TYPE_GCS:
                return

            sys_id = message.get_srcSystem()
            comp_id = message.get_srcComponent()

            agent_id = (sys_id, comp_id)

            # If the agent hasn't been seen before, save it
            if agent_id not in connection.agents:
                # Create and save a new agent
                agent = Agent(sys_id, comp_id, timeout_period=connection.agent_timeout)
                connection.agents[agent_id] = agent

                # Notify handlers that the agents changed
                connection.agent_list_changed.notify(agent=agent)
            else:
                # The connection has been restored
                if connection.agents[agent_id].timeout.value:
                    self.__logger.info(
                        f"Connection to agent {sys_id}:{comp_id} has been restored"
                    )

            # Update the last heartbeat variable
            connection.agents[agent_id].last_heartbeat.value = monotonic.monotonic()
            connection.agents[agent_id].timeout.value = False

            return

        @self._receive_message("HEARTBEAT")
        @self._timer()
        def listener(message: Any, connection: Connection) -> None:
            """
            Handle general agent information contained within a heartbeat.

            :param message: Incoming MAVLink message
            :type message: Any

            :param connection: MAVLink connection
            :type connection: Connection
            """
            # Ignore messages sent by a GCS
            if message.type == mavutil.mavlink.MAV_TYPE_GCS:
                return

            agent_id = (message.get_srcSystem(), message.get_srcComponent())

            if not agent_id in connection.agents:
                return

            connection.agents[agent_id].armed.value = (
                message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            ) != 0

            connection.agents[agent_id].system_status.value = message.system_status
            connection.agents[agent_id].vehicle_type.value = message.type

            # Update the last heartbeat
            connection.agents[agent_id].last_heartbeat.value = monotonic.monotonic()

            try:
                # WARNING: We are currently assuming that ardupilot will be used
                # this will be changed in a future version
                connection.agents[
                    agent_id
                ].flight_mode.value = mavutil.mode_mapping_bynumber(message.type)[
                    message.custom_mode
                ]
            except Exception:
                # We received an invalid message
                pass

            return

        @self._receive_message("GLOBAL_POSITION_INT")
        @self._timer()
        def listener(message: Any, connection: Connection) -> None:
            """
            Handle the a GPS position message.

            :param message: Incoming MAVLink message
            :type message: Any

            :param connection: MAVLink connection
            :type connection: Connection
            """
            agent_id = (message.get_srcSystem(), message.get_srcComponent())

            if not agent_id in connection.agents:
                return

            # Update the agent velocity
            if connection.agents[agent_id].velocity is None:
                velocity = swarm_state.Velocity(
                    message.vx / 100, message.vy / 100, message.vz / 100
                )
                connection.agents[agent_id].velocity = velocity
            else:
                connection.agents[agent_id].velocity.vx = message.vx / 100
                connection.agents[agent_id].velocity.vy = message.vy / 100
                connection.agents[agent_id].velocity.vz = message.vz / 100

            # Update the agent location
            if connection.agents[agent_id].location is None:
                loc = swarm_state.Location(
                    message.lat / 1.0e7,
                    message.lon / 1.0e7,
                    message.relative_alt / 1000,
                )
                connection.agents[agent_id].location = loc
            else:
                connection.agents[agent_id].location.latitude = message.lat / 1.0e7
                connection.agents[agent_id].location.longitude = message.lon / 1.0e7
                connection.agents[agent_id].location.altitude = (
                    message.relative_alt / 1000
                )

            return

        @self._receive_message("ATTITUDE")
        @self._timer()
        def listener(message: Any, connection: Connection) -> None:
            """
            Handle an agent attitude message.

            :param message: Incoming MAVLink message
            :type message: Any

            :param connection: MAVLink connection
            :type connection: Connection
            """
            agent_id = (message.get_srcSystem(), message.get_srcComponent())

            if not agent_id in connection.agents:
                return

            # Update the respective agents attitude
            if connection.agents[agent_id].attitude is None:
                att = swarm_state.Attitude(
                    message.pitch,
                    message.yaw,
                    message.roll,
                    message.pitchspeed,
                    message.yawspeed,
                    message.rollspeed,
                )
                connection.agents[agent_id].attitude = att
            else:
                connection.agents[agent_id].attitude.pitch = message.pitch
                connection.agents[agent_id].attitude.roll = message.roll
                connection.agents[agent_id].attitude.yaw = message.yaw
                connection.agents[agent_id].attitude.pitch_speed = message.pitchspeed
                connection.agents[agent_id].attitude.roll_speed = message.rollspeed
                connection.agents[agent_id].attitude.yaw_speed = message.yawspeed

            return

        @self._receive_message("SYS_STATUS")
        @self._timer()
        def listener(message: Any, connection: Connection) -> None:
            """
            Handle the system status message containing battery state.

            :param message: Incoming MAVLink message
            :type message: Any

            :param connection: MAVLink connection
            :type connection: Connection
            """
            agent_id = (message.get_srcSystem(), message.get_srcComponent())

            if not agent_id in connection.agents:
                return

            # Update the battery information
            if connection.agents[agent_id].battery is None:
                batt = swarm_state.Battery(
                    message.voltage_battery,
                    message.current_battery,
                    message.battery_remaining,
                )
                connection.agents[agent_id].battery = batt
            else:
                connection.agents[agent_id].battery.voltage = message.voltage_battery
                connection.agents[agent_id].battery.current = message.current_battery
                connection.agents[agent_id].battery.level = message.battery_remaining

            return

        @self._receive_message("GPS_RAW_INT")
        @self._timer()
        def listener(message: Any, connection: Connection) -> None:
            """
            Handle the GPS status information.

            :param message: Incoming MAVLink message
            :type message: Any

            :param connection: MAVLink connection
            :type connection: Connection
            """
            agent_id = (message.get_srcSystem(), message.get_srcComponent())

            if not agent_id in connection.agents:
                return

            # Read the GPS status information
            if connection.agents[agent_id].gps_info is None:
                info = swarm_state.GPSInfo(
                    message.eph,
                    message.epv,
                    message.fix_type,
                    message.satellites_visible,
                )
                connection.agents[agent_id].gps_info = info
            else:
                connection.agents[agent_id].gps_info.eph = message.eph
                connection.agents[agent_id].gps_info.epv = message.epv
                connection.agents[agent_id].gps_info.fix_type = message.fix_type
                connection.agents[
                    agent_id
                ].gps_info.satellites_visible = message.satellites_visible

            return

        @self._receive_message("EKF_STATUS_REPORT")
        @self._timer()
        def listener(message: Any, connection: Connection) -> None:
            """
            Handle an EKF status message.

            :param message: Incoming MAVLink message
            :type message: Any

            :param connection: MAVLink connection
            :type connection: Connection
            """
            agent_id = (message.get_srcSystem(), message.get_srcComponent())

            if not agent_id in connection.agents:
                return

            # Read the EKF Status information
            if connection.agents[agent_id].ekf is None:
                ekf = swarm_state.EKFStatus(
                    message.velocity_variance,
                    message.pos_horiz_variance,
                    message.pos_vert_variance,
                    message.compass_variance,
                    message.terrain_alt_variance,
                    (message.flags & ardupilotmega.EKF_POS_HORIZ_ABS) > 0,
                    (message.flags & ardupilotmega.EKF_CONST_POS_MODE) > 0,
                    (message.flags & ardupilotmega.EKF_PRED_POS_HORIZ_ABS) > 0,
                )
                connection.agents[agent_id].ekf = ekf
            else:
                # Read variance properties
                connection.agents[
                    agent_id
                ].ekf.velocity_variance = message.velocity_variance
                connection.agents[
                    agent_id
                ].ekf.pos_horiz_variance = message.pos_horiz_variance
                connection.agents[
                    agent_id
                ].ekf.pos_vert_variance = message.pos_vert_variance
                connection.agents[
                    agent_id
                ].ekf.compass_variance = message.compass_variance
                connection.agents[
                    agent_id
                ].ekf.terrain_alt_variance = message.terrain_alt_variance

                # Read flags
                connection.agents[agent_id].ekf.pos_horiz_abs = (
                    message.flags & ardupilotmega.EKF_POS_HORIZ_ABS
                ) > 0
                connection.agents[agent_id].ekf.const_pos_mode = (
                    message.flags & ardupilotmega.EKF_CONST_POS_MODE
                ) > 0
                connection.agents[agent_id].ekf.pred_pos_horiz_abs = (
                    message.flags & ardupilotmega.EKF_PRED_POS_HORIZ_ABS
                ) > 0

            return

        @self._receive_message("ATTITUDE")
        @self._timer()
        def listener(message: Any, connection: Connection) -> None:
            """
            Handle an agent attitude message.

            :param message: Incoming MAVLink message
            :type message: Any

            :param connection: MAVLink connection
            :type connection: Connection
            """
            agent_id = (message.get_srcSystem(), message.get_srcComponent())

            if not agent_id in connection.agents:
                return

            # Update the respective agent's attitude
            if connection.agents[agent_id].attitude is None:
                att = swarm_state.Attitude(
                    message.pitch,
                    message.yaw,
                    message.roll,
                    message.pitchspeed,
                    message.yawspeed,
                    message.rollspeed,
                )
                connection.agents[agent_id].attitude = att
            else:
                connection.agents[agent_id].attitude.pitch = message.pitch
                connection.agents[agent_id].attitude.roll = message.roll
                connection.agents[agent_id].attitude.yaw = message.yaw
                connection.agents[agent_id].attitude.pitch_speed = message.pitchspeed
                connection.agents[agent_id].attitude.roll_speed = message.rollspeed
                connection.agents[agent_id].attitude.yaw_speed = message.yawspeed

            return

        @self._receive_message("HOME_POSITION")
        @self._timer()
        def listener(message: Any, connection: Connection) -> None:
            """
            Handle the home position message.

            The altitude is provided in MSL, NOT AGL

            :param message: Incoming MAVLink message
            :type message: Any

            :param connection: MAVLink connection
            :type connection: Connection
            """
            agent_id = (message.get_srcSystem(), message.get_srcComponent())

            # Let the heartbeat implementation handle this
            if not agent_id in connection.agents:
                return

            # Update the agent home location
            if connection.agents[agent_id].home_position is None:
                loc = swarm_state.Location(
                    message.latitude / 1.0e7,
                    message.longitude / 1.0e7,
                    message.altitude / 1000,
                )
                connection.agents[agent_id].home_position = loc
            else:
                connection.agents[agent_id].home_position.latitude = (
                    message.latitude / 1.0e7
                )
                connection.agents[agent_id].home_position.longitude = (
                    message.longitude / 1.0e7
                )
                connection.agents[agent_id].home_position.altitude = (
                    message.altitude / 1000
                )

            return

        return
