from .State import State



class Battery(State):
    """
    MAVLink Specifications
    voltage : voltage_battery   : mV : Battery voltage, UINT16_MAX: Voltage not sent by autopilot
    current : current_battery   : cA : Battery current, -1: Current not sent by autopilot
    level   : battery_remaining : %  : Battery energy remaining, -1: Battery remaining energy not sent by autopilot
    """
    def __init__(self, voltage: float=0.0, 
                 current: float=0.0, 
                 level: float=0.0,
                 callbacks: list=[]) -> None:
        super().__init__(callbacks)
        
        self.__voltage = voltage
        self.__current = current
        self.__level = level

        return

    
    def get_current_state(self) -> dict:
        return {
            'voltage': self.voltage,
            'current': self.current,
            'level': self.level
        }

    
    @property
    def voltage(self) -> float:
        return self.__voltage

    
    @voltage.setter
    def voltage(self, voltage: float) -> None:
        self.__voltage = voltage

        for cb in self.callbacks:
            cb(self.get_current_state())

        return

    
    @property
    def current(self) -> float:
        return self.__current

    
    @current.setter
    def current(self, current: float) -> None:
        if current == -1:
            self.__current = current
        else:
            self.__current = current / 100.0

        for cb in self.callbacks:
            cb(self.get_current_state())

        return


    @property
    def level(self) -> float:
        return self.__level


    @level.setter
    def level(self, level: float) -> None:
        self.__level = level

        for cb in self.callbacks:
            cb(self.get_current_state())

        return