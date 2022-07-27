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


class Battery(State):
    """Agent battery state."""

    def __init__(
        self,
        voltage: float,
        current: float,
        level: float,
        optional_context_props: dict | None = None,
    ) -> None:
        """
        Create a battery state instance.

        :param voltage: battery voltage [mV], UINT16_MAX: voltage not sent by autopilot
        :type voltage: float
        :param current: battery current [cA], -1: current not sent by autopilot
        :type current: float
        :param level: battery energy remaining [%], -1: battery remaining energy not
            sent by autopilot
        :type level: float
        :param optional_context_props: optional properties to add to the battery
            context, defaults to None
        :type optional_context_props: dict | None, optional
        """
        super().__init__(optional_context_props)

        self.__voltage = voltage
        self.__current = current
        self.__level = level

        return

    @property
    def voltage(self) -> float:
        """
        Battery voltage [mV].

        UINT16_MAX: Voltage not sent by autopilot.

        :return: voltage
        :rtype: float
        """
        return self.__voltage

    @voltage.setter
    def voltage(self, voltage: float) -> None:
        """
        Set the voltage.

        :param voltage: battery voltage [mV]
        :type voltage: float
        """
        prev_voltage = self.__voltage
        self.__voltage = voltage

        # Signal state change event
        if self.__voltage != prev_voltage:
            self.state_changed_event.notify(**self.context)

        return

    @property
    def current(self) -> float:
        """
        Battery current [cA].

        -1: Current not sent by autopilot.

        :return: current
        :rtype: float
        """
        return self.__current

    @current.setter
    def current(self, current: float) -> None:
        """
        Set the battery current.

        :param current: battery current [cA]
        :type current: float
        """
        prev_current = self.__current
        if current == -1:
            self.__current = current
        else:
            self.__current = current / 100.0

        # Signal state change event
        if self.__current != prev_current:
            self.state_changed_event.notify(**self.context)

        return

    @property
    def level(self) -> float:
        """
        Battery energy remaining [%].

        -1: Battery remaining energy not sent by autopilot.

        :rtype: float
        """
        return self.__level

    @level.setter
    def level(self, level: float) -> None:
        """
        Set the battery level.

        :param level: battery level [%]
        :type level: float
        """
        prev_level = self.__level
        self.__level = level

        # Signal state change event
        if self.__level != prev_level:
            self.state_changed_event.notify(**self.context)

        return

    @property
    def context(self) -> dict:
        """
        Battery context.

        :return: context
        :rtype: dict
        """
        context = super().context

        context["voltage"] = self.__voltage
        context["current"] = self.__current
        context["level"] = self.__level

        return context

    def __str__(self) -> str:
        """
        Print battery information in a human-readable format.

        :return: battery
        :rtype: str
        """
        return f"Battery: {self.context}"
