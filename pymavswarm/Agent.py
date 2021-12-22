import monotonic
from .state import *


class Agent:
    """
    Agent represents and stores the state of an agent in the network. The
    agent's state is updated as new MAVLink messages are received from the
    associated message
    """
    def __init__(self, sys_id, comp_id, timeout_period=30) -> None:
        self.sys_id: int = sys_id
        self.comp_id: int = comp_id
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
        self.timeout_period: int = timeout_period
        self.timeout: bool = False