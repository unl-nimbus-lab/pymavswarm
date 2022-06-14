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


class Battery(State):
    """
    Agent battery state
    """

    def __init__(
        self,
        voltage: float = 0.0,
        current: float = 0.0,
        level: float = 0.0,
        optional_context_props: dict = {},
    ) -> None:
        """
        :param voltage: Battery voltage (mV), UINT16_MAX: Voltage not sent by autopilot,
            defaults to 0.0
        :type voltage: float, optional

        :param current: Battery current (cA), -1: Current not sent by autopilot,
            defaults to 0.0
        :type current: float, optional

        :param level: Battery energy remaining (%), -1: Battery remaining energy not
            sent by autopilot, defaults to 0.0
        :type level: float, optional

        :param optional_context_props: Optional properties to add to the context
        :type optional_context_props: dict, optional
        """
        super().__init__()

        self.__voltage = voltage
        self.__current = current
        self.__level = level
        self.__optional_context_props = optional_context_props

        return

    @property
    def voltage(self) -> float:
        """
        Battery voltage (mV), UINT16_MAX: Voltage not sent by autopilot

        :rtype: float
        """
        return self.__voltage

    @voltage.setter
    def voltage(self, voltage: float) -> None:
        """
        voltage setter

        :param voltage: Battery voltage (mV)
        :type voltage: float
        """
        prev_voltage = self.__voltage
        self.__voltage = voltage

        # Signal state change event
        if self.__voltage != prev_voltage:
            self.state_changed_event.notify(context=self.context)

        return

    @property
    def current(self) -> float:
        """
        Battery current (cA), -1: Current not sent by autopilot

        :rtype: float
        """
        return self.__current

    @current.setter
    def current(self, current: float) -> None:
        """
        current setter

        :param current: Battery current (cA)
        :type current: float
        """
        prev_current = self.__current
        if current == -1:
            self.__current = current
        else:
            self.__current = current / 100.0

        # Signal state change event
        if self.__current != prev_current:
            self.state_changed_event.notify(context=self.context)

        return

    @property
    def level(self) -> float:
        """
        Battery energy remaining (%), -1: Battery remaining energy not
        sent by autopilot, defaults to 0.0

        :rtype: float
        """
        return self.__level

    @level.setter
    def level(self, level: float) -> None:
        """
        level setter

        :param level: Battery level (%)
        :type level: float
        """
        prev_level = self.__level
        self.__level = level

        # Signal state change event
        if self.__level != prev_level:
            self.state_changed_event.notify(context=self.context)

        return

    @property
    def context(self) -> dict:
        """
        Get the current state as a dictionary for callbacks

        :return: Properties of interest associated with the battery state
        :rtype: dict
        """
        context = {
            "voltage": self.__voltage,
            "current": self.__current,
            "level": self.__level,
        }
        context.update(self.__optional_context_props)

        return context
