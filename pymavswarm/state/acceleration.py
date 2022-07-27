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


class Acceleration(State):
    """Acceleration state."""

    def __init__(
        self,
        ax: float,
        ay: float,
        az: float,
        optional_context_props: dict | None = None,
    ) -> None:
        """
        Create a new acceleration object.

        :param ax: ground x acceleration [Latitude, positive north]
        :type ax: float
        :param ay: ground y acceleration [Longitude, positive east]
        :type ay: float
        :param az: ground z acceleration [Altitude, positive down]
        :type az: float
        :param optional_context_props: optional properties to add to the context
        :type optional_context_props: dict, optional
        """
        super().__init__(optional_context_props)

        self.__ax = ax
        self.__ay = ay
        self.__az = az

        return

    @property
    def acceleration_x(self) -> float:
        """
        Ground X acceleration [Latitude, positive north].

        :return: acceleration x component
        :rtype: float
        """
        return self.__ax

    @acceleration_x.setter
    def acceleration_x(self, accel: float) -> None:
        """
        Set the acceleration's x component.

        :param accel: x acceleration [m/s^s]
        :type accel: float
        """
        prev_ax = self.__ax
        self.__ax = accel

        # Signal state change event
        if self.__ax != prev_ax:
            self.state_changed_event.notify(**self.context)

        return

    @property
    def acceleration_y(self) -> float:
        """
        Ground Y acceleration [Longitude, positive east].

        :return: acceleration y component
        :rtype: float
        """
        return self.__ay

    @acceleration_y.setter
    def acceleration_y(self, accel: float) -> None:
        """
        Set the acceleration's y component.

        :param accel: y acceleration [m/s^2]
        :type accel: float
        """
        prev_ay = self.__ay
        self.__ay = accel

        # Signal state change event
        if self.__ay != prev_ay:
            self.state_changed_event.notify(**self.context)

        return

    @property
    def acceleration_z(self) -> float:
        """
        Ground Z acceleration [Altitude, positive down].

        :return: acceleration z component
        :rtype: float
        """
        return self.__az

    @acceleration_z.setter
    def acceleration_z(self, accel: float) -> None:
        """
        Set the acceleration's z component.

        :param accel: z acceleration [m/s^2]
        :type accel: float
        """
        prev_az = self.__az
        self.__az = accel

        # Signal state change event
        if self.__az != prev_az:
            self.state_changed_event.notify(**self.context)

        return

    @property
    def context(self) -> dict:
        """
        Acceleration context.

        :return: context
        :rtype: dict
        """
        context = super().context

        context["ax"] = self.__ax
        context["ay"] = self.__ay
        context["az"] = self.__az

        return context

    def __str__(self) -> str:
        """
        Print acceleration information in a human-readable format.

        :return: acceleration information
        :rtype: str
        """
        return f"Acceleration: {self.context}"
