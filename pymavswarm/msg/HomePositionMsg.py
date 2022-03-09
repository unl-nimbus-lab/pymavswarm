from typing import Optional
from .AgentMsg import AgentMsg


class HomePositionMsg(AgentMsg):
    """
    HomePositionMsg signals a home position reset. The home position can be reset to the current location
    or set to a specific location. Note that if the home location is being reset to the current position,
    then the location is not required. If the home location is being set to a specific location, all
    components must be set. 
    Params:
        lat  :   : Longitude
        long :   : Longitude
        alt  : m : Altitude
    """
    def __init__(self, msg_type: str, 
                 target_system: int, 
                 target_comp: int, 
                 retry: bool,
                 lat: Optional[float]=None,
                 lon: Optional[float]=None,
                 alt: Optional[float]=None,
                 msg_timeout: float=5.0) -> None:
        super().__init__(msg_type, target_system, target_comp, retry, msg_timeout=msg_timeout)
        self.altitude = alt
        self.lat = lat
        self.lon = lon

        return