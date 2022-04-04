from .State import State



class EKFStatus(State):
    """
    EKF Flags indicating EKF status (True if healthy)
    """
    def __init__(self, velocity_variance: float=0.0, 
                 pos_horiz_variance: float=0.0, 
                 pos_vert_variance: float=0.0, 
                 compass_variance: float=0.0, 
                 terrain_alt_variance: float=0.0, 
                 pos_horiz_abs: float=0.0, 
                 const_pos_mode: float=0.0, 
                 pred_pos_horiz_abs: float=0.0,
                 callbacks: list=[]) -> None:
        """
        MAVLink Specification
        pos_horiz_abs        : Flags, EKF's Horizontal position (absolute) estimate is good.
        const_pos_mode       : Flags, EKF is in constant position mode and does not know 
                               it's absolute or relative position.
        pred_pos_horiz_abs   : EKF's predicted horizontal position (absolute) estimate is good.
        pos_horiz_abs        : Flags, Horizontal position (absolute) estimate is good.
        velocity_variance    : Velocity variance.
        pos_horiz_variance   : Horizontal Position variance.
        pos_vert_variance    : Vertical Position variance.
        compass_variance     : Compass variance.
        terrain_alt_variance : Terrain Altitude variance.
        """
        super().__init__(callbacks)

        self.__velocity_variance = velocity_variance
        self.__pos_horiz_variance = pos_horiz_variance
        self.__pos_vert_variance = pos_vert_variance
        self.__compass_variance = compass_variance
        self.__terrain_alt_variance = terrain_alt_variance
        self.__pos_horiz_abs = pos_horiz_abs
        self.__const_pos_mode = const_pos_mode
        self.__pred_pos_horiz_abs = pred_pos_horiz_abs

        return


    def get_current_state(self) -> dict:
        return {
            'velocity_variance': self.velocity_variance,
            'pos_horiz_variance': self.pos_horiz_variance,
            'pos_vert_variance': self.pos_vert_variance,
            'compass_variance': self.compass_variance,
            'terrain_alt_variance': self.terrain_alt_variance,
            'pos_horiz_abs': self.pos_horiz_abs,
            'const_pos_mode': self.const_pos_mode,
            'pred_pos_horiz_abs': self.pred_pos_horiz_abs
        }

    
    @property
    def velocity_variance(self) -> float:
        return self.__velocity_variance

    
    @velocity_variance.setter
    def velocity_variance(self, variance: float) -> None:
        self.__velocity_variance = variance

        for cb in self.callbacks:
            cb(self.get_current_state())

        return


    @property
    def pos_horiz_variance(self) -> float:
        return self.__pos_horiz_variance

    
    @pos_horiz_variance.setter
    def pos_horiz_variance(self, variance: float) -> None:
        self.__pos_horiz_variance = variance

        for cb in self.callbacks:
            cb(self.get_current_state())

        return

    
    @property
    def pos_vert_variance(self) -> float:
        return self.__pos_vert_variance

    
    @pos_vert_variance.setter
    def pos_vert_variance(self, variance: float) -> None:
        self.__pos_vert_variance = variance

        for cb in self.callbacks:
            cb(self.get_current_state())

        return

    
    @property
    def compass_variance(self) -> float:
        return self.__compass_variance


    @compass_variance.setter
    def compass_variance(self, variance: float) -> None:
        self.__compass_variance = variance

        for cb in self.callbacks:
            cb(self.get_current_state())

        return

    
    @property
    def terrain_alt_variance(self) -> float:
        return self.__terrain_alt_variance

    
    @terrain_alt_variance.setter
    def terrain_alt_variance(self, variance: float) -> None:
        self.terrain_alt_variance = variance

        for cb in self.callbacks:
            cb(self.get_current_state())

        return

    
    @property
    def pos_horiz_abs(self) -> float:
        return self.__pos_horiz_abs

    
    @pos_horiz_abs.setter
    def pos_horiz_abs(self, flags: float) -> None:
        self.__pos_horiz_abs = flags

        for cb in self.callbacks:
            cb(self.get_current_state())

        return

    
    @property
    def const_pos_mode(self) -> float:
        return self.__const_pos_mode

    
    @const_pos_mode.setter
    def const_pos_mode(self, flags: float) -> None:
        self.__const_pos_mode = flags

        for cb in self.callbacks:
            cb(self.get_current_state())

        return


    @property
    def pred_pos_horiz_abs(self) -> float:
        return self.__pred_pos_horiz_abs


    @pred_pos_horiz_abs.setter
    def pred_pos_horiz_abs(self, pred: float) -> None:
        self.__pred_pos_horiz_abs = pred

        for cb in self.callbacks:
            cb(self.get_current_state())

        return