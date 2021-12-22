class Attitude:
    def __init__(self, 
                 pitch: float=0.0, 
                 yaw: float=0.0, 
                 roll: float=0.0, 
                 pitch_speed: float=0.0, 
                 yaw_speed: float=0.0, 
                 roll_speed: float=0.0) -> None:
        """
        MAVLink Specifications
        pitch       : pitch      : rad   : Pitch angle (-pi..+pi)
        yaw         : yaw        : rad   : Yaw angle (-pi..+pi)
        roll        : roll       : rad   : Roll angle (-pi..+pi)
        pitch_speed : pitchspeed : rad/s : Pitch angular speed
        yaw_speed   : yawspeed   : rad/s : Yaw angular speed
        roll_speed  : rollspeed  : rad/s : Roll angular speed
        """
        self.pitch: float = pitch
        self.yaw: float = yaw
        self.roll: float = roll
        self.pitch_speed: float = pitch_speed
        self.yaw_speed: float = yaw_speed
        self.roll_speed: float = roll_speed