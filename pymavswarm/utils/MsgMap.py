class FlightModes:
    """
    Helper class used to enable easy retrieval of the command used by a connection to 
    send a respective flight mode message
    """
    def __init__(self) -> None:
        self.stabilize = 'stabilize'
        self.acro = 'acro'
        self.alt_hold = 'althold'
        self.auto = 'auto'
        self.loiter = 'loiter'
        self.rtl = 'rtl'
        self.land = 'land'
        self.throw = 'throw'
        self.systemid = 'systemid'
        self.guided = 'guided'


class SystemCommands:
    """
    Helper class used to enable easy retrieval of the command used by a connection to 
    send a respective system command message
    """
    def __init__(self) -> None:
        self.arm = 'arm'
        self.disarm = 'disarm'
        self.accel_cal = 'accelcal'
        self.accel_cal_simple = 'accelcalsimple'
        self.ahrs_trim = 'ahrstrim'
        self.start_ros = 'startros'
        self.stop_ros = 'stopros'
        self.start_path_execution = 'startpath'
        self.stop_path_execution = 'stoppath'


class MsgMap:
    """
    Helper class used to provide easy retrieval of the commands used by mavswarm to send
    messages
    """
    def __init__(self) -> None:
        self.flight_modes = FlightModes()
        self.system_commands = SystemCommands()
