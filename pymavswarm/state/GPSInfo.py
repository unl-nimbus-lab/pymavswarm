from .State import State


class GPSInfo(State):
    """
    GPS information associated with an agent
    """

    def __init__(
        self,
        eph: float = 0.0,
        epv: float = 0.0,
        fix_type: int = 0,
        satellites_visible: int = 0,
    ) -> None:
        """
        :param eph: GPS HDOP horizontal dilution of position (unitless * 100).
            If unknown, set to: UINT16_MAX, defaults to 0.0
        :type eph: float, optional

        :param epv: GPS VDOP vertical dilution of position (unitless * 100). If
            unknown, set to: UINT16_MAX, defaults to 0.0
        :type epv: float, optional

        :param fix_type: GPS fix type (GPS_FIX_TYPE), defaults to 0
        :type fix_type: int, optional

        :param satellites_visible: Number of satellites visible. If unknown, set to
            UINT8_MAX, defaults to 0
        :type satellites_visible: int, optional
        """
        self.__eph = eph
        self.__epv = epv
        self.__fix_type = fix_type
        self.__satellites_visible = satellites_visible

        return

    def get_current_state(self) -> dict:
        """
        Get the current state as a dictionary for callbacks

        :return: Properties of interested associated with the GPS information
        :rtype: dict
        """
        return {
            "eph": self.eph,
            "epv": self.epv,
            "fix_type": self.fix_type,
            "satellites_visible": self.satellites_visible,
        }

    @property
    def eph(self) -> float:
        """
        GPS HDOP horizontal dilution of position (unitless * 100)

        :rtype: float
        """
        return self.__eph

    @eph.setter
    def eph(self, dilution: float) -> None:
        """
        eph setter

        :param dilution: If unknown, set to: UINT16_MAX
        :type dilution: float
        """
        self.__eph = dilution

        for cb in self.callbacks:
            cb(self.get_current_state())

        return

    @property
    def epv(self) -> float:
        """
        GPS VDOP vertical dilution of position (unitless * 100)

        :rtype: float
        """
        return self.__epv

    @epv.setter
    def epv(self, dilution: float) -> None:
        """
        epv setter

        :param dilution: If unknown, set to: UINT16_MAX
        :type dilution: float
        """
        self.__epv = dilution

        for cb in self.callbacks:
            cb(self.get_current_state())

        return

    @property
    def fix_type(self) -> int:
        """
        GPS fix type (GPS_FIX_TYPE)

        :rtype: int
        """
        return self.__fix_type

    @fix_type.setter
    def fix_type(self, gps_fix_type: int) -> None:
        """
        fix_type setter

        :param gps_fix_type: GPS_FIX_TYPE
        :type gps_fix_type: int
        """
        self.__fix_type = gps_fix_type

        for cb in self.callbacks:
            cb(self.get_current_state())

        return

    @property
    def satellites_visible(self) -> int:
        """
        Number of satellites visible

        :rtype: int
        """
        return self.__satellites_visible

    @satellites_visible.setter
    def satellites_visible(self, satellites: int) -> None:
        """
        satellites_visible setter

        :param satellites: If unknown, set to UINT8_MAX
        :type satellites: int
        """
        self.__satellites_visible = satellites

        for cb in self.callbacks:
            cb(self.get_current_state())

        return
