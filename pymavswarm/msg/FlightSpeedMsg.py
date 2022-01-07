from .AgentMsg import AgentMsg



class FlightSpeedMsg(AgentMsg):
    """
    FlightSpeedMsg represents messages that should change the flight speed of an agent.
    This may be either ground speed or air speed in m/s. The message types that should be used with this type 
    include:
        - air_speed
        - ground_speed
    """
    def __init__(self, speed: float, msg_type: str, sys_id: int, comp_id: int, check_ack: bool) -> None:
        super().__init__(msg_type, sys_id, comp_id, check_ack)
        
        self.speed = speed