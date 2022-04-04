from .State import State



class Attitude(State):
    """
    MAVLink Specifications
    pitch       : pitch      : rad   : Pitch angle (-pi..+pi)
    yaw         : yaw        : rad   : Yaw angle (-pi..+pi)
    roll        : roll       : rad   : Roll angle (-pi..+pi)
    pitch_speed : pitchspeed : rad/s : Pitch angular speed
    yaw_speed   : yawspeed   : rad/s : Yaw angular speed
    roll_speed  : rollspeed  : rad/s : Roll angular speed
    """
    def __init__(self, pitch: float=0.0, 
                 yaw: float=0.0, 
                 roll: float=0.0, 
                 pitch_speed: float=0.0, 
                 yaw_speed: float=0.0, 
                 roll_speed: float=0.0,
                 callbacks: list=[]) -> None:
        super().__init__(callbacks)

        self.__pitch = pitch
        self.__yaw = yaw
        self.__roll = roll
        self.__pitch_speed = pitch_speed
        self.__yaw_speed = yaw_speed
        self.__roll_speed = roll_speed

        return


    def get_current_state(self) -> dict:
        """
        Get the current state as a dictionary for callbacks
        """
        return {
            'roll': self.roll,
            'pitch': self.pitch,
            'yaw': self.yaw,
            'roll_speed': self.roll_speed,
            'pitch_speed': self.pitch_speed,
            'yaw_speed': self.yaw_speed     
        }

    
    @property
    def pitch(self) -> float:
        return self.__pitch

    
    @pitch.setter
    def pitch(self, angle: float) -> None:
        self.__pitch = angle

        for cb in self.callbacks:
            cb(self.get_current_state())

        return


    @property
    def yaw(self) -> float:
        return self.__yaw

    
    @yaw.setter
    def yaw(self, angle: float) -> None:
        self.__yaw = angle

        for cb in self.callbacks:
            cb(self.get_current_state())

        return


    @property
    def roll(self) -> float:
        return self.__roll


    @roll.setter
    def roll(self, angle: float) -> None:
        self.__roll = angle

        for cb in self.callbacks:
            cb(self.get_current_state())

        return

    
    @property
    def pitch_speed(self) -> float:
        return self.__pitch_speed


    @pitch_speed.setter
    def pitch_speed(self, rate: float) -> None:
        self.__pitch_speed = rate

        for cb in self.callbacks:
            cb(self.get_current_state())

        return

    
    @property
    def roll_speed(self) -> float:
        return self.__roll_speed

    
    @roll_speed.setter
    def roll_speed(self, rate: float) -> None:
        self.__roll_speed = rate

        for cb in self.callbacks:
            cb(self.get_current_state())
        
        return

    
    @property
    def yaw_speed(self) -> float:
        return self.__yaw_speed

    
    @yaw_speed.setter
    def yaw_speed(self, rate: float) -> None:
        self.__yaw_speed = rate

        for cb in self.callbacks:
            cb(self.get_current_state())

        return
