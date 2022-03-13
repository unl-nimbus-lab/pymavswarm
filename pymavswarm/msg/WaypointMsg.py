from .AgentMsg import AgentMsg



class WaypointMsg(AgentMsg):
    """
    WaypointMsg represents a desired waypoint for an agent to fly to. Based on the MAV_CMD_NAV_WAYPOINT message
    Params:
        hold [float]  : s   : Time to stay at waypoint for rotary wing (ignored by fixed wing)
        accept_radius : m   : If the sphere with this radius is hit, the waypoint counts as reached
        pass_radius   : m   : 0 to pass through the WP, if > 0 radius to pass by WP. Positive value for 
                              clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.
        yaw           : deg : Desired yaw angle at waypoint (rotary wing). NaN to use the current system yaw heading mode 
                              (e.g. yaw towards next waypoint, yaw to home, etc.).
        lat           :     : Latitude
        lon           :     : Longitude
        alt           :     : Altitude
    """
    def __init__(self, lat: float,
                 lon: float,
                 alt: float,
                 target_system: int, 
                 target_comp: int, 
                 retry: bool,
                 hold: float=0,
                 accept_radius: float=0,
                 pass_radius: float=0,
                 yaw: float=0, 
                 msg_timeout: float=5.0, 
                 ack_timeout: float=1.0) -> None:
        super().__init__('WAYPOINT', target_system, target_comp, retry, msg_timeout=msg_timeout, ack_timeout=ack_timeout)
        self.hold = hold
        self.accept_radius = accept_radius
        self.pass_radius = pass_radius
        self.yaw = yaw
        self.lat = lat
        self.lon = lon
        self.alt = alt

        return

