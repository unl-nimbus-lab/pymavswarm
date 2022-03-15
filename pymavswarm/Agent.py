import monotonic
from .state import *
from .mission import Mission
from .param import Parameter
from typing import Optional
from collections import deque



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
        self.sys_id: int = sys_id
        self.comp_id: int = comp_id
        self.name: Optional[str] = name
        self.attitude: Attitude = Attitude()
        self.battery: Battery = Battery()
        self.docker_info: DockerInfo = DockerInfo()
        self.gps_info: GPSInfo = GPSInfo()
        self.location: Location = Location()
        self.ekf: EKFStatus = EKFStatus()
        self.telemetry: Telemetry = Telemetry()
        self.velocity: Velocity = Velocity()
        self.armed: bool = False
        self.flight_mode: str = 'None'
        self.system_status = 'None'
        self.vehicle_type = 'None'
        self.last_heartbeat: int = monotonic.monotonic()
        self.timeout_period: float = timeout_period
        self.timeout: bool = False
        self.current_waypoint: int = 0
        self.mission: Mission = Mission()
        self.last_params_read = deque(maxlen=max_params_stored)
        self.home_position = Location()

        return