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


class Velocity(State):
    """Velocity state."""

    def __init__(
        self,
        vx: float = 0.0,
        vy: float = 0.0,
        vz: float = 0.0,
        optional_context_props: dict | None = None,
    ) -> None:
        """
        Create a new velocity object.

        :param vx: ground x speed [Latitude, positive north], defaults to 0.0
        :type vx: float, optional
        :param vy: ground y speed [Longitude, positive east], defaults to 0.0
        :type vy: float, optional
        :param vz: ground z speed [Altitude, positive down], defaults to 0.0
        :type vz: float, optional
        :param optional_context_props: optional properties to add to the context
        :type optional_context_props: dict, optional
        """
        super().__init__(optional_context_props)

        self.__vx = vx
        self.__vy = vy
        self.__vz = vz

        return

    @property
    def velocity_x(self) -> float:
        """
        Ground X Speed [Latitude, positive north].

        :return: velocity x component
        :rtype: float
        """
        return self.__vx

    @velocity_x.setter
    def velocity_x(self, vel: float) -> None:
        """
        Set the velocity's x component.

        :param vel: x speed [cm/s]
        :type vel: float
        """
        prev_vx = self.__vx
        self.__vx = vel

        # Signal state change event
        if self.__vx != prev_vx:
            self.state_changed_event.notify(context=self.context)

        return

    @property
    def velocity_y(self) -> float:
        """
        Ground Y Speed [Longitude, positive east].

        :return: velocity y component
        :rtype: float
        """
        return self.__vy

    @velocity_y.setter
    def velocity_y(self, vel: float) -> None:
        """
        Set the velocity's y component.

        :param vel: y speed [cm/s]
        :type vel: float
        """
        prev_vy = self.__vy
        self.__vy = vel

        # Signal state change event
        if self.__vy != prev_vy:
            self.state_changed_event.notify(context=self.context)

        return

    @property
    def velocity_z(self) -> float:
        """
        Ground Z Speed [Altitude, positive down].

        :return: velocity z component
        :rtype: float
        """
        return self.__vz

    @velocity_z.setter
    def velocity_z(self, vel: float) -> None:
        """
        Set the velocity's z component.

        :param vel: z speed [cm/s]
        :type vel: float
        """
        prev_vz = self.__vz
        self.__vz = vel

        # Signal state change event
        if self.__vz != prev_vz:
            self.state_changed_event.notify(context=self.context)

        return

    @property
    def context(self) -> dict:
        """
        Velocity context.

        :return: context
        :rtype: dict
        """
        context = super().context

        context["vx"] = self.__vx
        context["vy"] = self.__vy
        context["vz"] = self.__vz

        return context

    def __str__(self) -> str:
        """
        Print velocity information in a human-readable format.

        :return: velocity information
        :rtype: str
        """
        return f"Velocity: {self.context}"
