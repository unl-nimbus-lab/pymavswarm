from .AgentMsg import AgentMsg



class TakeoffMsg(AgentMsg):
    """
    Message used to indicate that an agent should takeoff to a certain location/altitude
    Params:
        alt : m : The altitude that an agent should takeoff to
        lon :   : Longitude - this is optional
        lat :   : Latitude - this is optional
    """
    def __init__(self, alt: float, 
                 target_system: int, 
                 target_comp: int, 
                 retry: bool, 
                 lat: float=0,
                 lon: float=0,
                 msg_timeout: float=5.0, 
                 ack_timeout: float=1.0) -> None:                 
        super().__init__('TAKEOFF', target_system, target_comp, retry, msg_timeout=msg_timeout, ack_timeout=ack_timeout)
        self.alt = alt
        self.lat = lat
        self.lon = lon

        return