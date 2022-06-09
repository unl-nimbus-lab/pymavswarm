from typing import Any, List, Optional

import monotonic

import pymavswarm.state as swarm_state
from pymavswarm.event import Event
from pymavswarm.mission import Mission


class Agent:
    """
    Agent represents and stores the state of an agent in the network. The
    agent's state is updated as new MAVLink messages are received from the
    associated message
    """

    def __init__(
        self,
        sys_id: int,
        comp_id: int,
        name: Optional[str] = None,
        timeout_period: float = 30.0,
        max_params_stored: int = 5,
    ) -> None:
        """
        :param sys_id: The system ID of the agent
        :type sys_id: int

        :param comp_id: The component ID of the agent
        :type comp_id: int

        :param name: The name assigned to the agent, defaults to None
        :type name: Optional[str], optional

        :param timeout_period: The timeout period of the agent, defaults to 30.0
        :type timeout_period: float, optional

        :param max_params_stored: The maximum number of parameters that should be
            stored by an agent at once (implemented using a circular buffer), defaults
            to 5
        :type max_params_stored: int, optional
        """
        # Immutable
        self.__sys_id = sys_id
        self.__comp_id = comp_id
        self.__name = name

        """
        We create additional context properties here so that listeners of the
        respective property events have information about the parent agent
        """
        context_props = {"sys_id": sys_id, "comp_id": comp_id, "name": name}

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
            False, "armed", optional_context_props=context_props
        )
        self.__flight_mode = swarm_state.Generic(
            "None", "flight_mode", optional_context_props=context_props
        )
        self.__system_status = swarm_state.Generic(
            "None", "system_status", optional_context_props=context_props
        )
        self.__vehicle_type = swarm_state.Generic(
            "None", "vehicle_type", optional_context_props=context_props
        )
        self.__last_heartbeat = swarm_state.Generic(
            monotonic.monotonic(),
            "last_heartbeat",
            optional_context_props=context_props,
        )
        self.__timeout_period = swarm_state.Generic(
            timeout_period, "timeout_period", optional_context_props=context_props
        )
        self.__timeout = swarm_state.Generic(
            False, "timeout", optional_context_props=context_props
        )
        self.__current_waypoint = swarm_state.Generic(
            0, "current_waypoint", optional_context_props=context_props
        )
        self.__mission = Mission()
        self.__last_params_read = swarm_state.ParameterList(
            max_length=max_params_stored, optional_context_props=context_props
        )
        self.__home_position = swarm_state.Location(
            optional_context_props=context_props
        )
        self.__hrl_state = swarm_state.Generic(
            "None", "hrl_state", optional_context_props=context_props
        )
        self.__custom_events = []

        return

    @property
    def sys_id(self) -> int:
        """
        The system ID of the agent

        :rtype: int
        """
        return self.__sys_id

    @property
    def comp_id(self) -> int:
        """
        The component ID of the agent

        :rtype: int
        """
        return self.__comp_id

    @property
    def name(self) -> str:
        """
        The assigned name of an agent. Used to assign more readable names to
        agents

        :rtype: str
        """
        return self.__name

    @property
    def attitude(self) -> swarm_state.Attitude:
        """
        The current attitude of the agent

        :rtype: Attitude
        """
        return self.__attitude

    @property
    def battery(self) -> swarm_state.Battery:
        """
        The current battery state of the agent

        :rtype: Battery
        """
        return self.__battery

    @property
    def docker_info(self) -> swarm_state.DockerInfo:
        """
        Information regarding deployed docker image

        :rtype: DockerInfo
        """
        return self.__docker_info

    @property
    def gps_info(self) -> swarm_state.GPSInfo:
        """
        Information regarding an agent's GPS

        :rtype: GPSInfo
        """
        return self.__gps_info

    @property
    def location(self) -> swarm_state.Location:
        """
        Current location of an agent

        :rtype: Location
        """
        return self.__location

    @property
    def ekf(self) -> swarm_state.EKFStatus:
        """
        EKF status of the agent

        :rtype: EKFStatus
        """
        return self.__ekf

    @property
    def telemetry(self) -> swarm_state.Telemetry:
        """
        Telemetry status information

        :rtype: Telemetry
        """
        return self.__telemetry

    @property
    def velocity(self) -> swarm_state.Velocity:
        """
        Current velocity of the agent

        :rtype: Velocity
        """
        return self.__velocity

    @property
    def armed(self) -> bool:
        """
        Flag indicating whether the agent is currently armed

        :rtype: bool
        """
        return self.__armed

    @property
    def flight_mode(self) -> str:
        """
        Current flight mode that an agent is operating in

        :rtype: str
        """
        return self.__flight_mode

    @property
    def system_status(self) -> str:
        """
        System-level status of an agent

        :rtype: str
        """ ""
        return self.__system_status

    @property
    def vehicle_type(self) -> str:
        """
        Type of vehicle that the agent exists as

        :rtype: str
        """
        return self.__vehicle_type

    @property
    def last_heartbeat(self) -> Any:
        """
        Timestamp of the last heartbeat message received

        :return: _description_
        :rtype: Any
        """
        return self.__last_heartbeat

    @property
    def timeout_period(self) -> float:
        """
        The amount of time allowable between heartbeat messages before an agent is
        considered timed out

        :rtype: float
        """
        return self.__timeout_period

    @property
    def timeout(self) -> bool:
        """
        The current timeout state of the agent

        :rtype: bool
        """
        return self.__timeout

    @property
    def current_waypoint(self) -> int:
        """
        Index of the current waypoint that an agent is maneuvering to

        :rtype: int
        """
        return self.__current_waypoint

    @property
    def mission(self) -> Mission:
        """
        Current mission being completed by an agent

        :rtype: Mission
        """
        return self.__mission

    @property
    def last_params_read(self) -> swarm_state.ParameterList:
        """
        Circle buffer containing the most recent parameters read and their values

        :rtype: deque
        """
        return self.__last_params_read

    @property
    def home_position(self) -> swarm_state.Location:
        """
        Home position of the agent

        :rtype: Location
        """
        return self.__home_position

    @property
    def hrl_state(self) -> str:
        """
        The current HRL state that the agent is in

        :rtype: str
        """
        return self.__hrl_state

    @property
    def custom_events(self) -> List[Event]:
        """
        A configurable list that can be used to enable non-default custom events

        :rtype: List[Event]
        """
        return self.__custom_events

    @custom_events.setter
    def custom_events(self, events: List[Event]) -> None:
        """
        custom_events setter

        :param events: The list of custom events
        :type events: List[Event]
        """
        self.__custom_events = events
        return
