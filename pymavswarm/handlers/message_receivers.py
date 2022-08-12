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

from __future__ import annotations

import logging
from copy import deepcopy
from typing import Any

import monotonic
from pymavlink import mavutil
from pymavlink.dialects.v10 import ardupilotmega as mavlink1

from pymavswarm.agent import Agent
from pymavswarm.handlers.receivers import Receivers
from pymavswarm.types import AgentID


class MessageReceivers(Receivers):
    """Collection of methods responsible for processing incoming messages."""

    def __init__(self, log_level: int = logging.INFO) -> None:
        """
        Make a new receivers object.

        :param log_level: log level, defaults to logging.INFO
        :type log_level: int, optional
        """
        super().__init__(__name__, log_level)

        @self._receive_message("HEARTBEAT")
        def listener(message: Any, agents: dict[AgentID, Agent]) -> None:
            """
            Register new agents or update the timeout status of existing agents.

            :param message: Incoming MAVLink message
            :type message: Any
            :param agents: agents in the swarm
            :type agents: dict[AgentID, Agent]
            """
            # Make sure that the message isn't from a GCS
            if message.get_type() == mavutil.mavlink.MAV_TYPE_GCS:
                return agents

            sys_id = message.get_srcSystem()
            comp_id = message.get_srcComponent()

            agent_id = (sys_id, comp_id)

            # If the agent hasn't been seen before, save it
            if agent_id not in agents:
                # Create and save a new agent
                agent = Agent(sys_id, comp_id)
                agents[agent_id] = agent

            else:
                # The connection has been restored
                if agents[agent_id].timeout.value:
                    self._logger.info(
                        f"Connection to agent {sys_id}:{comp_id} has been restored"
                    )

            # Update the last heartbeat variable
            agents[agent_id].last_heartbeat.value = monotonic.monotonic()
            agents[agent_id].timeout.value = False

            return

        @self._receive_message("HEARTBEAT")
        def listener(message: Any, agents: dict[AgentID, Agent]) -> None:
            """
            Handle general agent information contained within a heartbeat.

            :param message: Incoming MAVLink message
            :type message: Any
            :param agents: agents in the swarm
            :type agents: dict[AgentID, Agent]
            """
            # Ignore messages sent by a GCS
            if message.type == mavutil.mavlink.MAV_TYPE_GCS:
                return agents

            agent_id = (message.get_srcSystem(), message.get_srcComponent())

            if agent_id not in agents:
                return agents

            agents[agent_id].armed.value = (
                message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            ) != 0

            agents[agent_id].system_status.value = message.system_status
            agents[agent_id].vehicle_type.value = message.type

            # Update the last heartbeat
            agents[agent_id].last_heartbeat.value = monotonic.monotonic()

            try:
                # WARNING: We are currently assuming that ardupilot will be used
                # this will be changed in a future version
                agents[agent_id].mode.value = mavutil.mode_mapping_bynumber(
                    message.type
                )[message.custom_mode]
            except Exception:
                self._logger.debug("An invalid heartbeat message was received")

            return

        @self._receive_message("GLOBAL_POSITION_INT")
        def listener(message: Any, agents: dict[AgentID, Agent]) -> None:
            """
            Handle a GPS position message using the global frame.

            :param message: Incoming MAVLink message
            :type message: Any
            :param agents: agents in the swarm
            :type agents: dict[AgentID, Agent]
            """
            agent_id = (message.get_srcSystem(), message.get_srcComponent())

            if agent_id not in agents:
                return agents

            # Store the previous velocity
            prev_velocity = deepcopy(agents[agent_id].velocity)

            # Update the agent velocity
            agents[agent_id].velocity.global_frame.x = message.vx / 100
            agents[agent_id].velocity.global_frame.y = message.vy / 100
            agents[agent_id].velocity.global_frame.z = message.vz / 100

            # Compute the corrected timestamp in the global clock
            timestamp = message.time_boot_ms - agents[agent_id].clock_offset.value

            # Calculate the acceleration if there has been more than one velocity
            # reading
            if (
                prev_velocity.global_frame.timestamp != 0.0
                and (timestamp - prev_velocity.global_frame.timestamp) > 0.0
            ):
                agents[agent_id].acceleration.global_frame.x = (
                    agents[agent_id].velocity.global_frame.x
                    - prev_velocity.global_frame.x
                ) / (timestamp - prev_velocity.global_frame.timestamp)
                agents[agent_id].acceleration.global_frame.y = (
                    agents[agent_id].velocity.global_frame.y
                    - prev_velocity.global_frame.y
                ) / (timestamp - prev_velocity.global_frame.timestamp)
                agents[agent_id].acceleration.global_frame.z = (
                    agents[agent_id].velocity.global_frame.z
                    - prev_velocity.global_frame.z
                ) / (timestamp - prev_velocity.global_frame.timestamp)

            # Update the agent global location
            agents[agent_id].position.global_frame.x = message.lat / 1.0e7
            agents[agent_id].position.global_frame.y = message.lon / 1.0e7
            agents[agent_id].position.global_frame.z = message.alt / 1000

            # Update the timestamps using the current clock offset
            agents[agent_id].velocity.global_frame.timestamp = timestamp
            agents[agent_id].acceleration.global_frame.timestamp = timestamp
            agents[agent_id].position.global_frame.timestamp = timestamp

            return

        @self._receive_message("GLOBAL_POSITION_INT")
        def listener(message: Any, agents: dict[AgentID, Agent]) -> None:
            """
            Handle a GPS position message using the global relative frame.

            :param message: Incoming MAVLink message
            :type message: Any
            :param agents: agents in the swarm
            :type agents: dict[AgentID, Agent]
            """
            agent_id = (message.get_srcSystem(), message.get_srcComponent())

            if agent_id not in agents:
                return agents

            # Store the previous velocity
            prev_velocity = deepcopy(agents[agent_id].velocity)

            # Update the agent velocity
            agents[agent_id].velocity.global_relative_frame.x = message.vx / 100
            agents[agent_id].velocity.global_relative_frame.y = message.vy / 100
            agents[agent_id].velocity.global_relative_frame.z = message.vz / 100

            # Compute the corrected timestamp in the global clock
            timestamp = message.time_boot_ms - agents[agent_id].clock_offset.value

            # Calculate the acceleration if there has been more than one velocity
            # reading and the time difference is greater than zero
            if (
                prev_velocity.global_relative_frame.timestamp != 0.0
                and (timestamp - prev_velocity.global_relative_frame.timestamp) > 0.0
            ):
                agents[agent_id].acceleration.global_relative_frame.x = (
                    agents[agent_id].velocity.global_relative_frame.x
                    - prev_velocity.global_relative_frame.x
                ) / (timestamp - prev_velocity.global_relative_frame.timestamp)
                agents[agent_id].acceleration.global_relative_frame.y = (
                    agents[agent_id].velocity.global_relative_frame.y
                    - prev_velocity.global_relative_frame.y
                ) / (timestamp - prev_velocity.global_relative_frame.timestamp)
                agents[agent_id].acceleration.global_relative_frame.z = (
                    agents[agent_id].velocity.global_relative_frame.z
                    - prev_velocity.global_relative_frame.z
                ) / (timestamp - prev_velocity.global_relative_frame.timestamp)

            # Update the agent global location
            agents[agent_id].position.global_relative_frame.x = message.lat / 1.0e7
            agents[agent_id].position.global_relative_frame.y = message.lon / 1.0e7
            agents[agent_id].position.global_relative_frame.z = (
                message.relative_alt / 1000
            )

            # Update the timestamps using the current clock offset
            agents[agent_id].velocity.global_relative_frame.timestamp = timestamp
            agents[agent_id].acceleration.global_relative_frame.timestamp = timestamp
            agents[agent_id].position.global_relative_frame.timestamp = timestamp

            return

        @self._receive_message("LOCAL_POSITION_NED")
        def listener(message: Any, agents: dict[AgentID, Agent]) -> None:
            """
            Handle an agent local position message.

            :param message: Incoming MAVLink message
            :type message: Any
            :param agents: agents in the swarm
            :type agents: dict[AgentID, Agent]
            """
            agent_id = (message.get_srcSystem(), message.get_srcComponent())

            if agent_id not in agents:
                return agents

            # Store the previous velocity
            prev_velocity = deepcopy(agents[agent_id].velocity)

            # Update the agent velocity
            agents[agent_id].velocity.local_frame.x = message.vx
            agents[agent_id].velocity.local_frame.y = message.vy
            agents[agent_id].velocity.local_frame.z = message.vz

            # Compute the corrected timestamp in the global clock
            timestamp = message.time_boot_ms - agents[agent_id].clock_offset.value

            # Calculate the acceleration if there has been more than one velocity
            # reading
            if (
                prev_velocity.local_frame.timestamp != 0.0
                and (timestamp - prev_velocity.local_frame.timestamp) > 0.0
            ):
                agents[agent_id].acceleration.local_frame.x = (
                    agents[agent_id].velocity.local_frame.x
                    - prev_velocity.local_frame.x
                ) / (timestamp - prev_velocity.local_frame.timestamp)
                agents[agent_id].acceleration.local_frame.y = (
                    agents[agent_id].velocity.local_frame.y
                    - prev_velocity.local_frame.y
                ) / (timestamp - prev_velocity.local_frame.timestamp)
                agents[agent_id].acceleration.local_frame.z = (
                    agents[agent_id].velocity.local_frame.z
                    - prev_velocity.local_frame.z
                ) / (timestamp - prev_velocity.local_frame.timestamp)

            # Update the agent local location
            agents[agent_id].position.local_frame.x = message.x
            agents[agent_id].position.local_frame.y = message.y
            agents[agent_id].position.local_frame.z = message.z

            # Update the timestamps using the current clock offset
            agents[agent_id].velocity.local_frame.timestamp = timestamp
            agents[agent_id].acceleration.local_frame.timestamp = timestamp
            agents[agent_id].position.local_frame.timestamp = timestamp

            return

        @self._receive_message("ATTITUDE")
        def listener(message: Any, agents: dict[AgentID, Agent]) -> None:
            """
            Handle an agent attitude message.

            :param message: Incoming MAVLink message
            :type message: Any
            :param agents: agents in the swarm
            :type agents: dict[AgentID, Agent]
            """
            agent_id = (message.get_srcSystem(), message.get_srcComponent())

            if agent_id not in agents:
                return agents

            # Update the respective agents attitude
            agents[agent_id].attitude.pitch = message.pitch
            agents[agent_id].attitude.roll = message.roll
            agents[agent_id].attitude.yaw = message.yaw
            agents[agent_id].attitude.pitch_speed = message.pitchspeed
            agents[agent_id].attitude.roll_speed = message.rollspeed
            agents[agent_id].attitude.yaw_speed = message.yawspeed

            return

        @self._receive_message("SYS_STATUS")
        def listener(message: Any, agents: dict[AgentID, Agent]) -> None:
            """
            Handle the system status message containing battery state.

            :param message: Incoming MAVLink message
            :type message: Any
            :param agents: agents in the swarm
            :type agents: dict[AgentID, Agent]
            """
            agent_id = (message.get_srcSystem(), message.get_srcComponent())

            if agent_id not in agents:
                return agents

            # Update the battery information
            agents[agent_id].battery.voltage = message.voltage_battery
            agents[agent_id].battery.current = message.current_battery
            agents[agent_id].battery.level = message.battery_remaining

            return

        @self._receive_message("GPS_RAW_INT")
        def listener(message: Any, agents: dict[AgentID, Agent]) -> None:
            """
            Handle the GPS status information.

            :param message: Incoming MAVLink message
            :type message: Any
            :param agents: agents in the swarm
            :type agents: dict[AgentID, Agent]
            """
            agent_id = (message.get_srcSystem(), message.get_srcComponent())

            if agent_id not in agents:
                return agents

            # Read the GPS status information
            agents[agent_id].gps_info.eph = message.eph
            agents[agent_id].gps_info.epv = message.epv
            agents[agent_id].gps_info.fix_type = message.fix_type
            agents[agent_id].gps_info.satellites_visible = message.satellites_visible

            return

        @self._receive_message("EKF_STATUS_REPORT")
        def listener(message: Any, agents: dict[AgentID, Agent]) -> None:
            """
            Handle an EKF status message.

            :param message: Incoming MAVLink message
            :type message: Any
            :param agents: agents in the swarm
            :type agents: dict[AgentID, Agent]
            """
            agent_id = (message.get_srcSystem(), message.get_srcComponent())

            if agent_id not in agents:
                return agents

            # Read the EKF Status information
            agents[agent_id].ekf.velocity_variance = message.velocity_variance
            agents[agent_id].ekf.pos_horiz_variance = message.pos_horiz_variance
            agents[agent_id].ekf.pos_vert_variance = message.pos_vert_variance
            agents[agent_id].ekf.compass_variance = message.compass_variance
            agents[agent_id].ekf.terrain_alt_variance = message.terrain_alt_variance

            # Read flags
            agents[agent_id].ekf.pos_horiz_abs = (
                message.flags & mavlink1.EKF_POS_HORIZ_ABS
            ) > 0
            agents[agent_id].ekf.const_pos_mode = (
                message.flags & mavlink1.EKF_CONST_POS_MODE
            ) > 0
            agents[agent_id].ekf.pred_pos_horiz_abs = (
                message.flags & mavlink1.EKF_PRED_POS_HORIZ_ABS
            ) > 0

            return

        @self._receive_message("ATTITUDE")
        def listener(message: Any, agents: dict[AgentID, Agent]) -> None:
            """
            Handle an agent attitude message.

            :param message: Incoming MAVLink message
            :type message: Any
            :param agents: agents in the swarm
            :type agents: dict[AgentID, Agent]
            """
            agent_id = (message.get_srcSystem(), message.get_srcComponent())

            if agent_id not in agents:
                return agents

            # Update the respective agent's attitude
            agents[agent_id].attitude.pitch = message.pitch
            agents[agent_id].attitude.roll = message.roll
            agents[agent_id].attitude.yaw = message.yaw
            agents[agent_id].attitude.pitch_speed = message.pitchspeed
            agents[agent_id].attitude.roll_speed = message.rollspeed
            agents[agent_id].attitude.yaw_speed = message.yawspeed

            return

        @self._receive_message("HOME_POSITION")
        def listener(message: Any, agents: dict[AgentID, Agent]) -> None:
            """
            Handle the home position message.

            The altitude is provided in MSL, NOT AGL

            :param message: Incoming MAVLink message
            :type message: Any
            :param agents: agents in the swarm
            :type agents: dict[AgentID, Agent]
            """
            agent_id = (message.get_srcSystem(), message.get_srcComponent())

            # Let the heartbeat implementation handle this
            if agent_id not in agents:
                return agents

            # Update the agent home location
            agents[agent_id].home_position.x = message.latitude / 1.0e7
            agents[agent_id].home_position.y = message.longitude / 1.0e7
            agents[agent_id].home_position.z = message.altitude / 1000

            return

        return
