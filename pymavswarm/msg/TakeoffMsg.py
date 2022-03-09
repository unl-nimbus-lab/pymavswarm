from typing import Optional
from .AgentMsg import AgentMsg
from .MsgMap import MissionCommand


class TakeoffMsg(AgentMsg):
    """
    Message used to indicate that an agent should takeoff to a certain location/altitude
    Params:
        altitude : m : The altitude that an agent should takeoff to
        lon      :   : Longitude - this is optional
        lat      :   : Latitude - this is optional
    """
    def __init__(self, msg_type: str, 
                 target_system: int, 
                 target_comp: int, 
                 retry: bool, 
                 altitude: float=3.0,
                 lat: Optional[float]=None,
                 lon: Optional[float]=None,
                 msg_timeout: float=5.0, 
                 ack_timeout: float=1.0) -> None:                 
        super().__init__(msg_type, target_system, target_comp, retry, msg_timeout=msg_timeout, ack_timeout=ack_timeout)
        self.altitude = altitude
        self.lat = lat
        self.lon = lon

        return