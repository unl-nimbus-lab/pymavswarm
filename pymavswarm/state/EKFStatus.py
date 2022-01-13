class EKFStatus:
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
                 pred_pos_horiz_abs: float=0.0) -> None:
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
        self.velocity_variance = velocity_variance
        self.pos_horiz_variance = pos_horiz_variance
        self.pos_vert_variance = pos_vert_variance
        self.compass_variance = compass_variance
        self.terrain_alt_variance = terrain_alt_variance
        self.pos_horiz_abs = pos_horiz_abs
        self.const_pos_mode = const_pos_mode
        self.pred_pos_horiz_abs = pred_pos_horiz_abs

        return