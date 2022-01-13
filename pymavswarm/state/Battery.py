class Battery:
    def __init__(self, voltage: float=0.0, 
                 current: float=0.0, 
                 level: float=0.0) -> None:
        """
        MAVLink Specifications
        voltage : voltage_battery   : mV : Battery voltage, UINT16_MAX: Voltage not sent by autopilot
        current : current_battery   : cA : Battery current, -1: Current not sent by autopilot
        level   : battery_remaining : %  : Battery energy remaining, -1: Battery remaining energy not sent by autopilot
        """
        self.voltage = voltage
        
        if current == -1:
            self.current = None
        else:
            self.current = current / 100.0
        
        if level == -1:
            self.level = None
        else:
            self.level = level