import monotonic
from .state import *
from .mission import Mission
from collections import deque
from typing import Optional, Any



class Agent:
    """
    Agent represents and stores the state of an agent in the network. The
    agent's state is updated as new MAVLink messages are received from the
    associated message
    Params:
        - sys_id            : int   : The system ID of the agent
        - comp_id           : int   : The componenet ID of the agent
        - name              : str   : The name assigned to the agent
        - timeout_period    : float : The timeout period of the agent
        - max_params_stored : int   : The maximum number of parameters that should be stored by an agent at once (implemented using a circular buffer)
    """
    def __init__(self, sys_id: int, 
                 comp_id: int, 
                 name: Optional[str]=None, 
                 timeout_period: float=30.0, 
                 max_params_stored: int=5) -> None:
        self.__sys_id = sys_id
        self.__comp_id = comp_id
        self.__name = name
        self.__attitude = Attitude()
        self.__battery = Battery()
        self.__docker_info = DockerInfo()
        self.__gps_info = GPSInfo()
        self.__location = Location()
        self.__ekf = EKFStatus()
        self.__telemetry = Telemetry()
        self.__velocity = Velocity()
        self.__armed = False
        self.__flight_mode = 'None'
        self.__system_status = 'None'
        self.__vehicle_type = 'None'
        self.__last_heartbeat = monotonic.monotonic()
        self.__timeout_period = timeout_period
        self.__timeout = False
        self.__current_waypoint = 0
        self.__mission = Mission()
        self.__last_params_read = deque(maxlen=max_params_stored)
        self.__home_position = Location()
        self.__hrl_state = 'None'

        return


    @property
    def sys_id(self) -> int:
        return self.__sys_id


    @sys_id.setter
    def sys_id(self, id: int) -> None:
        self.__sys_id = id
        return


    @property
    def comp_id(self) -> int:
        return self.__comp_id


    @comp_id.setter
    def comp_id(self, id: int) -> None:
        self.__comp_id = id
        return

    
    @property
    def name(self) -> str:
        return self.__name

    
    @sys_id.setter
    def name(self, name: str) -> None:
        self.__name = name
        return


    @property
    def attitude(self) -> Attitude:
        return self.__attitude

    
    @attitude.setter
    def attitude(self, att: Attitude) -> None:
        self.__attitude = att
        return


    @property
    def battery(self) -> Battery:
        return self.__battery

    
    @battery.setter
    def battery(self, batt: Battery) -> None:
        self.__battery = batt
        return


    @property
    def docker_info(self) -> DockerInfo:
        return self.__docker_info

    
    @docker_info.setter
    def docker_info(self, info: DockerInfo) -> None:
        self.__docker_info = info
        return
    

    @property
    def gps_info(self) -> GPSInfo:
        return self.__gps_info

    
    @gps_info.setter
    def gps_info(self, info: GPSInfo) -> None:
        self.__gps_info = info
        return


    @property
    def location(self) -> Location:
        return self.__location

    
    @location.setter
    def location(self, loc: Location) -> None:
        self.__location = loc
        return


    @property
    def ekf(self) -> EKFStatus:
        return self.__ekf


    @ekf.setter
    def ekf(self, status: EKFStatus) -> None:
        self.__ekf = status
        return


    @property
    def telemetry(self) -> Telemetry:
        return self.__telemetry


    @telemetry.setter
    def telemetry(self, telem: Telemetry) -> None:
        self.__telemetry = telem
        return


    @property
    def velocity(self) -> Velocity:
        return self.__velocity

    
    @velocity.setter
    def velocity(self, vel: Velocity) -> None:
        self.__velocity = vel
        return


    @property
    def armed(self) -> bool:
        return self.__armed


    @armed.setter
    def armed(self, state: bool) -> None:
        self.__armed = state
        return


    @property
    def flight_mode(self) -> str:
        return self.__flight_mode


    @flight_mode.setter
    def flight_mode(self, mode: str) -> None:
        self.__flight_mode = mode
        return


    @property
    def system_status(self) -> str:
        return self.__system_status


    @system_status.setter
    def system_status(self, status: str) -> None:
        self.__system_status = status
        return


    @property
    def vehicle_type(self) -> str:
        return self.__vehicle_type

    
    @vehicle_type.setter
    def vehicle_type(self, vehicle: str) -> None:
        self.__vehicle_type = vehicle
        return


    @property
    def last_heartbeat(self) -> Any:
        return self.__last_heartbeat

    
    @last_heartbeat.setter
    def last_heartbeat(self, heartbeat: Any) -> None:
        self.__last_heartbeat = heartbeat
        return


    @property
    def timeout_period(self) -> float:
        return self.__timeout_period


    @timeout_period.setter
    def timeout_period(self, period: float) -> None:
        self.__timeout_period = period
        return


    @property
    def timeout(self) -> bool:
        return self.__timeout


    @timeout.setter
    def timeout(self, status: bool) -> None:
        self.__timeout = status 
        return


    @property
    def current_waypoint(self) -> int:
        return self.__current_waypoint


    @current_waypoint.setter
    def current_waypoint(self, waypoint: int) -> None:
        self.__current_waypoint = waypoint
        return


    @property
    def mission(self) -> Mission:
        return self.__mission


    @mission.setter
    def mission(self, mission: Mission) -> None:
        self.__mission = mission
        return


    @property
    def last_params_read(self) -> deque:
        return self.__last_params_read


    @last_params_read.setter
    def last_params_read(self, params: deque) -> None:
        self.__last_params_read = params
        return


    @property
    def home_position(self) -> Location:
        return self.__home_position


    @home_position.setter
    def home_position(self, location: Location) -> None:
        self.__home_position = location
        return


    @property
    def hrl_state(self) -> str:
        return self.__hrl_state


    @hrl_state.setter
    def hrl_state(self, state: str) -> None:
        self.__hrl_state = state
        return