import monotonic
from collections import deque
from pymavswarm.state import *
from pymavswarm.event import Event
from pymavswarm.mission import Mission
from typing import List, Optional, Any


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
        self.__attitude = Attitude(optional_context_props=context_props)
        self.__battery = Battery(optional_context_props=context_props)
        self.__docker_info = DockerInfo(optional_context_props=context_props)
        self.__gps_info = GPSInfo(optional_context_props=context_props)
        self.__location = Location(optional_context_props=context_props)
        self.__ekf = EKFStatus(optional_context_props=context_props)
        self.__telemetry = Telemetry(optional_context_props=context_props)
        self.__velocity = Velocity(optional_context_props=context_props)
        self.__armed = Common(False, "armed", optional_context_props=context_props)
        self.__flight_mode = Common(
            "None", "flight_mode", optional_context_props=context_props
        )
        self.__system_status = Common(
            "None", "system_status", optional_context_props=context_props
        )
        self.__vehicle_type = Common(
            "None", "vehicle_type", optional_context_props=context_props
        )
        self.__last_heartbeat = Common(
            monotonic.monotonic(),
            "last_heartbeat",
            optional_context_props=context_props,
        )
        self.__timeout_period = Common(
            timeout_period, "timeout_period", optional_context_props=context_props
        )
        self.__timeout = Common(False, "timeout", optional_context_props=context_props)
        self.__current_waypoint = Common(
            0, "current_waypoint", optional_context_props=context_props
        )
        self.__mission = Mission()
        self.__last_params_read = deque(maxlen=max_params_stored)
        self.__home_position = Location(optional_context_props=context_props)
        self.__hrl_state = Common(
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
        return self.__sys_id.value

    @property
    def comp_id(self) -> int:
        """
        The component ID of the agent

        :rtype: int
        """
        return self.__comp_id.value

    @property
    def name(self) -> str:
        """
        The assigned name of an agent. Used to assign more readable names to
        agents

        :rtype: str
        """
        return self.__name.value

    @property
    def attitude(self) -> Attitude:
        """
        The current attitude of the agent

        :rtype: Attitude
        """
        return self.__attitude

    @attitude.setter
    def attitude(self, att: Attitude) -> None:
        """
        attitude setter

        :param att: Current attitude of an agent
        :type att: Attitude
        """
        self.__attitude = att
        return

    @property
    def battery(self) -> Battery:
        """
        The current battery state of the agent

        :rtype: Battery
        """
        return self.__battery

    @battery.setter
    def battery(self, batt: Battery) -> None:
        """
        battery setter

        :param batt: Battery state
        :type batt: Battery
        """
        self.__battery = batt
        return

    @property
    def docker_info(self) -> DockerInfo:
        """
        Information regarding deployed docker image

        :rtype: DockerInfo
        """
        return self.__docker_info

    @docker_info.setter
    def docker_info(self, info: DockerInfo) -> None:
        """
        docker_info setter

        :param info: Docker image information
        :type info: DockerInfo
        """
        self.__docker_info = info
        return

    @property
    def gps_info(self) -> GPSInfo:
        """
        Information regarding an agent's GPS

        :rtype: GPSInfo
        """
        return self.__gps_info

    @gps_info.setter
    def gps_info(self, info: GPSInfo) -> None:
        """
        gps_info setter

        :param info: GPS information
        :type info: GPSInfo
        """
        self.__gps_info = info
        return

    @property
    def location(self) -> Location:
        """
        Current location of an agent

        :rtype: Location
        """
        return self.__location

    @location.setter
    def location(self, loc: Location) -> None:
        """
        location setter

        :param loc: Current location
        :type loc: Location
        """
        self.__location = loc
        return

    @property
    def ekf(self) -> EKFStatus:
        """
        EKF status of the agent

        :rtype: EKFStatus
        """
        return self.__ekf

    @ekf.setter
    def ekf(self, status: EKFStatus) -> None:
        """
        ekf setter

        :param status: Current EKF status
        :type status: EKFStatus
        """
        self.__ekf = status
        return

    @property
    def telemetry(self) -> Telemetry:
        """
        Telemetry status information

        :rtype: Telemetry
        """
        return self.__telemetry

    @telemetry.setter
    def telemetry(self, telem: Telemetry) -> None:
        """
        telemetry setter

        :param telem: Telemetry status
        :type telem: Telemetry
        """
        self.__telemetry = telem
        return

    @property
    def velocity(self) -> Velocity:
        """
        Current velocity of the agent

        :rtype: Velocity
        """
        return self.__velocity

    @velocity.setter
    def velocity(self, vel: Velocity) -> None:
        """
        velocity setter

        :param vel: Current velocity of an agent
        :type vel: Velocity
        """
        self.__velocity = vel
        return

    @property
    def armed(self) -> bool:
        """
        Flag indicating whether the agent is currently armed

        :rtype: bool
        """
        return self.__armed.value

    @armed.setter
    def armed(self, state: bool) -> None:
        """
        armed setter

        :param state: State flag
        :type state: bool
        """
        self.__armed.value = state
        return

    @property
    def flight_mode(self) -> str:
        """
        Current flight mode that an agent is operating in

        :rtype: str
        """
        return self.__flight_mode.value

    @flight_mode.setter
    def flight_mode(self, mode: str) -> None:
        """
        flight_mode setter

        :param mode: Flight mode
        :type mode: str
        """
        self.__flight_mode.value = mode
        return

    @property
    def system_status(self) -> str:
        """
        System-level status of an agent

        :rtype: str
        """ ""
        return self.__system_status.value

    @system_status.setter
    def system_status(self, status: str) -> None:
        """
        system_status setter

        :param status: System status
        :type status: str
        """
        self.__system_status.value = status
        return

    @property
    def vehicle_type(self) -> str:
        """
        Type of vehicle that the agent exists as

        :rtype: str
        """
        return self.__vehicle_type.value

    @vehicle_type.setter
    def vehicle_type(self, vehicle: str) -> None:
        """
        vehicle_type setter

        :param vehicle: Vehicle type
        :type vehicle: str
        """
        self.__vehicle_type.value = vehicle
        return

    @property
    def last_heartbeat(self) -> Any:
        """
        Timestamp of the last heartbeat message received

        :return: _description_
        :rtype: Any
        """
        return self.__last_heartbeat.value

    @last_heartbeat.setter
    def last_heartbeat(self, heartbeat: Any) -> None:
        """
        last_heartbeat setter

        :param heartbeat: Timestamp of last heartbeat received
        :type heartbeat: Any
        """
        self.__last_heartbeat.value = heartbeat
        return

    @property
    def timeout_period(self) -> float:
        """
        The amount of time allowable between heartbeat messages before an agent is
        considered timed out

        :rtype: float
        """
        return self.__timeout_period.value

    @timeout_period.setter
    def timeout_period(self, period: float) -> None:
        """
        timeout_period setter

        :param period: Maximum allowable time between heartbeat events (s)
        :type period: float
        """
        self.__timeout_period.value = period
        return

    @property
    def timeout(self) -> bool:
        """
        The current timeout state of the agent

        :rtype: bool
        """
        return self.__timeout.value

    @timeout.setter
    def timeout(self, status: bool) -> None:
        """
        timeout setter

        :param status: Timeout flag
        :type status: bool
        """
        self.__timeout.value = status
        return

    @property
    def current_waypoint(self) -> int:
        """
        Index of the current waypoint that an agent is maneuvering to

        :rtype: int
        """
        return self.__current_waypoint.value

    @current_waypoint.setter
    def current_waypoint(self, waypoint: int) -> None:
        """
        current_waypoint setter

        :param waypoint: Waypoint index
        :type waypoint: int
        """
        self.__current_waypoint.value = waypoint
        return

    @property
    def mission(self) -> Mission:
        """
        Current mission being completed by an agent

        :rtype: Mission
        """
        return self.__mission

    @mission.setter
    def mission(self, mission: Mission) -> None:
        """
        mission setter

        :param mission: Current mission to be completed by an agent
        :type mission: Mission
        """
        self.__mission = mission
        return

    @property
    def last_params_read(self) -> deque:
        """
        Circle buffer containing the most recent parameters read and their values

        :rtype: deque
        """
        return self.__last_params_read

    @last_params_read.setter
    def last_params_read(self, params: deque) -> None:
        """
        last_params_read setter

        :param params: Circle buffer with last read params
        :type params: deque
        """
        self.__last_params_read = params
        return

    @property
    def home_position(self) -> Location:
        """
        Home position of the agent

        :rtype: Location
        """
        return self.__home_position

    @home_position.setter
    def home_position(self, location: Location) -> None:
        """
        home_position setter

        :param location: Location of the home position
        :type location: Location
        """
        self.__home_position = location
        return

    @property
    def hrl_state(self) -> str:
        """
        The current HRL state that the agent is in

        :rtype: str
        """
        return self.__hrl_state.value

    @hrl_state.setter
    def hrl_state(self, state: str) -> None:
        """
        hrl_state setter

        :param state: The HRL state of the agent
        :type state: str
        """
        self.__hrl_state.value = state
        return

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
