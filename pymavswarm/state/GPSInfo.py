from .State import State



class GPSInfo(State):
    """
    MAVLink Specification
    eph                : GPS HDOP horizontal dilution of position (unitless * 100). If unknown, set to: UINT16_MAX
    epv                : GPS VDOP vertical dilution of position (unitless * 100). If unknown, set to: UINT16_MAX
    fix_type           : GPS fix type (GPS_FIX_TYPE)
    satellites_visible : Number of satellites visible. If unknown, set to UINT8_MAX
    """
    def __init__(self, eph: float=0.0, 
                 epv: float=0.0, 
                 fix_type: int=0, 
                 satellites_visible: int=0) -> None:
        self.__eph = eph
        self.__epv = epv
        self.__fix_type = fix_type
        self.__satellites_visible = satellites_visible

        return


    def get_current_state(self) -> dict:
        return {
            'eph': self.eph,
            'epv': self.epv,
            'fix_type': self.fix_type,
            'satellites_visible': self.satellites_visible
        }

    
    @property
    def eph(self) -> float:
        return self.__eph

    
    @eph.setter
    def eph(self, dilution: float) -> None:
        self.__eph = dilution

        for cb in self.callbacks:
            cb(self.get_current_state())

        return


    @property
    def epv(self) -> float:
        return self.__epv


    @epv.setter
    def epv(self, dilution) -> None:
        self.__epv = dilution

        for cb in self.callbacks:
            cb(self.get_current_state())

        return


    @property
    def fix_type(self) -> int:
        return self.__fix_type

    

    @fix_type.setter
    def fix_type(self, gps_fix_type: int) -> None:
        self.__fix_type = gps_fix_type

        for cb in self.callbacks:
            cb(self.get_current_state())
            
        return


    @property
    def satellites_visible(self) -> int:
        return self.__satellites_visible

    
    @satellites_visible.setter
    def satellites_visible(self, satellites: int) -> None:
        self.__satellites_visible = satellites

        for cb in self.callbacks:
            cb(self.get_current_state())
            
        return