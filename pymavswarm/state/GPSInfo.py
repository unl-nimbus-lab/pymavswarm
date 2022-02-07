class GPSInfo:
    def __init__(self, eph: float=0.0, 
                 epv: float=0.0, 
                 fix_type: int=0, 
                 satellites_visible: int=0) -> None:
        """
        MAVLink Specification
        eph                : GPS HDOP horizontal dilution of position (unitless * 100). If unknown, set to: UINT16_MAX
        epv                : GPS VDOP vertical dilution of position (unitless * 100). If unknown, set to: UINT16_MAX
        fix_type           : GPS fix type (GPS_FIX_TYPE)
        satellites_visible : Number of satellites visible. If unknown, set to UINT8_MAX
        """
        self.eph = eph
        self.epv = epv
        self.fix_type = fix_type
        self.satellites_visible = satellites_visible

        return