# pymavswarm is an interface for swarm control and interaction
# Copyright (C) 2022  Evan Palmer

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

from pymavswarm.state.state import State


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
        optional_context_props: dict = {},
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

        :param optional_context_props: Optional properties to add to the context
        :type optional_context_props: dict, optional
        """
        super().__init__()

        self.__eph = eph
        self.__epv = epv
        self.__fix_type = fix_type
        self.__satellites_visible = satellites_visible
        self.__optional_context_props = optional_context_props

        return

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
        prev_eph = self.__eph
        self.__eph = dilution

        # Signal state change event
        if self.__eph != prev_eph:
            self.state_changed_event.notify(context=self.context)

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
        prev_epv = self.__epv
        self.__epv = dilution

        # Signal state change event
        if self.__epv != prev_epv:
            self.state_changed_event.notify(context=self.context)

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
        prev_fix_type = self.__fix_type
        self.__fix_type = gps_fix_type

        # Signal state change event
        if self.__fix_type != prev_fix_type:
            self.state_changed_event.notify(context=self.context)

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
        prev_satellites_visible = self.__satellites_visible
        self.__satellites_visible = satellites

        # Signal state change event
        if self.__satellites_visible != prev_satellites_visible:
            self.state_changed_event.notify(context=self.context)

        return

    @property
    def context(self) -> dict:
        """
        Get the current state as a dictionary for callbacks

        :return: Properties of interested associated with the GPS information
        :rtype: dict
        """
        context = {
            "eph": self.__eph,
            "epv": self.__epv,
            "fix_type": self.__fix_type,
            "satellites_visible": self.__satellites_visible,
        }
        context.update(self.__optional_context_props)

        return context
