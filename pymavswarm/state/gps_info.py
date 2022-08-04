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

from __future__ import annotations

from pymavswarm.state.state import State


class GPSInfo(State):
    """GPS information associated with an agent."""

    def __init__(
        self,
        eph: float,
        epv: float,
        fix_type: int,
        satellites_visible: int,
        optional_context_props: dict | None = None,
    ) -> None:
        """
        Create a GPSInfo object.

        :param eph: GPS HDOP horizontal dilution of position (unitless * 100).
            If unknown, set to: UINT16_MAX
        :type eph: float
        :param epv: GPS VDOP vertical dilution of position (unitless * 100). If
            unknown, set to: UINT16_MAX
        :type epv: float
        :param fix_type: GPS fix type (GPS_FIX_TYPE)
        :type fix_type: int
        :param satellites_visible: number of satellites visible. If unknown, set to
            UINT8_MAX
        :type satellites_visible: int
        :param optional_context_props: properties to add to the context, defaults to
            None
        :type optional_context_props: dict | None, optional
        """
        super().__init__(optional_context_props)

        self.__eph = eph
        self.__epv = epv
        self.__fix_type = fix_type
        self.__satellites_visible = satellites_visible

        return

    @property
    def eph(self) -> float:
        """
        GPS HDOP horizontal dilution of position (unitless * 100).

        :return: dilution
        :rtype: float
        """
        return self.__eph

    @eph.setter
    def eph(self, dilution: float) -> None:
        """
        Set the EPH.

        If unknown, set to: UINT16_MAX.

        :param dilution: dilution
        :type dilution: float
        """
        prev_eph = self.__eph
        self.__eph = dilution

        # Signal state change event
        if self.__eph != prev_eph:
            self.state_changed_event.notify(**self.context)

        return

    @property
    def epv(self) -> float:
        """
        GPS VDOP vertical dilution of position (unitless * 100).

        If unknown, set to: UINT16_MAX.

        :return: dilution
        :rtype: float
        """
        return self.__epv

    @epv.setter
    def epv(self, dilution: float) -> None:
        """
        Set the EPV.

        :param dilution: dilution
        :type dilution: float
        """
        prev_epv = self.__epv
        self.__epv = dilution

        # Signal state change event
        if self.__epv != prev_epv:
            self.state_changed_event.notify(**self.context)

        return

    @property
    def fix_type(self) -> int:
        """
        GPS fix type.

        (GPS_FIX_TYPE)

        :return: fix type
        :rtype: int
        """
        return self.__fix_type

    @fix_type.setter
    def fix_type(self, gps_fix_type: int) -> None:
        """
        Set the GPS fix type.

        :param gps_fix_type: fix type
        :type gps_fix_type: int
        """
        prev_fix_type = self.__fix_type
        self.__fix_type = gps_fix_type

        # Signal state change event
        if self.__fix_type != prev_fix_type:
            self.state_changed_event.notify(**self.context)

        return

    @property
    def satellites_visible(self) -> int:
        """
        Total number of satellites visible.

        :return: satellites visible
        :rtype: int
        """
        return self.__satellites_visible

    @satellites_visible.setter
    def satellites_visible(self, satellites: int) -> None:
        """
        Set the number of satellites visible.

        If unknown, set to UINT8_MAX.

        :param satellites: number of satellites
        :type satellites: int
        """
        prev_satellites_visible = self.__satellites_visible
        self.__satellites_visible = satellites

        # Signal state change event
        if self.__satellites_visible != prev_satellites_visible:
            self.state_changed_event.notify(**self.context)

        return

    @property
    def context(self) -> dict:
        """
        GPS info context.

        :return: context
        :rtype: dict
        """
        context = super().context

        context["eph"] = self.__eph
        context["epv"] = self.__epv
        context["fix_type"] = self.__fix_type
        context["satellites_visible"] = self.__satellites_visible

        return context

    def __str__(self) -> str:
        """
        Print GPS information in a human-readable format.

        :return: GPS information
        :rtype: str
        """
        return f"GPSInfo: {self.context}"
