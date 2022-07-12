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

from typing import Any, Tuple

import monotonic

import pymavswarm.state as swarm_state
from pymavswarm.mission import SwarmMission
from pymavswarm.state.generic import Generic

# Define a type alias for the agent ID
# Order should be (system ID, component ID)
AgentID = Tuple[int, int]


class Agent:
    """
    Swarm agent.

    Agent represents and stores the state of an agent in the network. The
    agent's state is updated as new MAVLink messages are received from the
    associated message.
    """

    def __init__(
        self,
        system_id: int,
        component_id: int,
        timeout_period: float = 30.0,
        max_params_stored: int = 5,
    ) -> None:
        """
        Construct an agent.

        :param sys_id: The system ID of the agent
        :type sys_id: int
        :param comp_id: The component ID of the agent
        :type comp_id: int
        :param timeout_period: The timeout period of the agent, defaults to 30.0
        :type timeout_period: float, optional
        :param max_params_stored: The maximum number of parameters that should be
            stored by an agent at once (implemented using a circular buffer), defaults
            to 5
        :type max_params_stored: int, optional
        """
        # Immutable
        self.__system_id = system_id
        self.__component_id = component_id

        # We create additional context properties here so that listeners of the
        # respective property events have information about the parent agent
        context_props = {"sys_id": system_id, "comp_id": component_id}

        # Mutable
        self.__attitude = swarm_state.Attitude(optional_context_props=context_props)
        self.__battery = swarm_state.Battery(optional_context_props=context_props)
        self.__docker_info = swarm_state.DockerInfo(
            optional_context_props=context_props
        )
        self.__gps_info = swarm_state.GPSInfo(optional_context_props=context_props)
        self.__location = swarm_state.Location(optional_context_props=context_props)
        self.__ekf = swarm_state.EKFStatus(optional_context_props=context_props)
        self.__telemetry = swarm_state.Telemetry(optional_context_props=context_props)
        self.__velocity = swarm_state.Velocity(optional_context_props=context_props)
        self.__armed = swarm_state.Generic(
            "armed", False, optional_context_props=context_props
        )
        self.__mode = swarm_state.Generic(
            "mode", None, optional_context_props=context_props
        )
        self.__system_status = swarm_state.Generic(
            "system_status", None, optional_context_props=context_props
        )
        self.__vehicle_type = swarm_state.Generic(
            "vehicle_type", None, optional_context_props=context_props
        )
        self.__last_heartbeat = swarm_state.Generic(
            "last_heartbeat",
            monotonic.monotonic(),
            optional_context_props=context_props,
        )
        self.__timeout_period = swarm_state.Generic(
            "timeout_period", timeout_period, optional_context_props=context_props
        )
        self.__timeout = swarm_state.Generic(
            "timeout", False, optional_context_props=context_props
        )
        self.__current_waypoint = swarm_state.Generic(
            "current_waypoint", 0, optional_context_props=context_props
        )
        self.__mission = SwarmMission()
        self.__last_params_read = swarm_state.ParameterList(
            max_length=max_params_stored, optional_context_props=context_props
        )
        self.__home_position = swarm_state.Location(
            optional_context_props=context_props
        )
        self.__hrl_state = swarm_state.Generic(
            "hrl_state", None, optional_context_props=context_props
        )

        return

    @property
    def system_id(self) -> int:
        """
        System ID of the agent.

        :return: system ID
        :rtype: int
        """
        return self.__system_id

    @property
    def component_id(self) -> int:
        """
        Component ID of the agent.

        :return: component ID
        :rtype: int
        """
        return self.__component_id

    @property
    def attitude(self) -> swarm_state.Attitude:
        """
        Attitude of the agent.

        :return: agent's attitude
        :rtype: Attitude
        """
        return self.__attitude

    @property
    def battery(self) -> swarm_state.Battery:
        """
        Battery state of the agent.

        :return: agent battery state
        :rtype: Battery
        """
        return self.__battery

    @property
    def docker_info(self) -> swarm_state.DockerInfo:
        """
        Deployed docker image info.

        :return: agent docker information, if any
        :rtype: DockerInfo
        """
        return self.__docker_info

    @property
    def gps_info(self) -> swarm_state.GPSInfo:
        """
        Agent GPS information.

        :return: agent GPS information
        :rtype: GPSInfo
        """
        return self.__gps_info

    @property
    def location(self) -> swarm_state.Location:
        """
        Location of an agent.

        :return: agent location
        :rtype: Location
        """
        return self.__location

    @property
    def ekf(self) -> swarm_state.EKFStatus:
        """
        EKF status of the agent.

        :return: agent EKF
        :rtype: EKFStatus
        """
        return self.__ekf

    @property
    def telemetry(self) -> swarm_state.Telemetry:
        """
        Telemetry status information.

        :return: telemetry information
        :rtype: Telemetry
        """
        return self.__telemetry

    @property
    def velocity(self) -> swarm_state.Velocity:
        """
        Velocity of the agent.

        return: agent velocity
        :rtype: Velocity
        """
        return self.__velocity

    @property
    def armed(self) -> Generic:
        """
        Arm state of an agent.

        :return: agent's arm state
        :rtype: bool
        """
        return self.__armed

    @property
    def mode(self) -> Generic:
        """
        Flight mode that an agent is operating in.

        :return: agent's flight mode
        :rtype: str
        """
        return self.__mode

    @property
    def system_status(self) -> Generic:
        """
        System-level status of an agent.

        :return: agent system status
        :rtype: str
        """ ""
        return self.__system_status

    @property
    def vehicle_type(self) -> Generic:
        """
        Type of vehicle that the agent exists as.

        :return: agent vehicle type
        :rtype: str
        """
        return self.__vehicle_type

    @property
    def last_heartbeat(self) -> Any:
        """
        Timestamp of the last heartbeat message received.

        :return: last heartbeat timestamp
        :rtype: Any
        """
        return self.__last_heartbeat

    @property
    def timeout_period(self) -> Generic:
        """
        Max time between heartbeat messages.

        :return: max spacing between heartbeat messages
        :rtype: float
        """
        return self.__timeout_period

    @property
    def timeout(self) -> Generic:
        """
        Timeout state of the agent.

        :return: whether or not the agent is timed out
        :rtype: bool
        """
        return self.__timeout

    @property
    def current_waypoint(self) -> Generic:
        """
        Index of the current waypoint that an agent is maneuvering to.

        :return: current target waypoint
        :rtype: int
        """
        return self.__current_waypoint

    @property
    def mission(self) -> SwarmMission:
        """
        Mission being completed by an agent.

        :return: agent's mission
        :rtype: Mission
        """
        return self.__mission

    @property
    def last_params_read(self) -> swarm_state.ParameterList:
        """
        Circle buffer containing the most recent parameters read and their values.

        :return: parameters read from the agent
        :rtype: deque
        """
        return self.__last_params_read

    @property
    def home_position(self) -> swarm_state.Location:
        """
        Home position of the agent.

        :return: agent's home position
        :rtype: Location
        """
        return self.__home_position

    @property
    def hrl_state(self) -> Generic:
        """
        HRL state that the agent is in.

        :rtype: str
        """
        return self.__hrl_state

    def __str__(self) -> str:
        """
        Print agent information in a human-readable format.

        :return: agent information
        :rtype: str
        """
        context = {"system_id": self.__system_id, "component_id": self.__component_id}
        return f"Agent: {context}"
