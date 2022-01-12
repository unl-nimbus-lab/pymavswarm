from .AgentMsg import AgentMsg
from typing import Optional


class TakeoffMsg(AgentMsg):
    """
    Message used to indicate that an agent should takeoff to a certain location/altitude
    Params:
        altitude [float] : m : The altitude that an agent should takeoff to
        lon      [float] :   : Longitude - this is optional
        lat      [float] :   : Latitude - this is optional
    """
    def __init__(self, altitude: float,
                 msg_type: str, 
                 target_id: int, 
                 target_comp: int, 
                 retry: bool, 
                 lat: Optional[float]=None,
                 lon: Optional[float]=None,
                 msg_timeout: float=5.0, 
                 ack_timeout: float=1.0) -> None:
        super().__init__(msg_type, target_id, target_comp, retry, msg_timeout=msg_timeout, ack_timeout=ack_timeout)
        self.altitude = altitude
        self.lat = lat
        self.lon = lon

        return