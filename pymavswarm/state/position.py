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


class Position(State):
    """Agent position."""

    def __init__(
        self,
        x: float,
        y: float,
        z: float,
        optional_context_props: dict | None = None,
    ) -> None:
        """
        Create a new location object.

        :param x: x position; this will typicall be latitude [WGS84, EGM96 ellipsoid]
        :type x: float
        :param y: y position; this will typically be longitude [WGS84, EGM96 ellipsoid]
        :type y: float
        :param z: z position; this will typically be altitude
        :type altitude: float
        :param optional_context_props: properties to add to the location context,
            defaults to None
        :type optional_context_props: dict | None, optional
        """
        super().__init__(optional_context_props)

        self.__x = x
        self.__y = y
        self.__z = z

        return

    @property
    def x(self) -> float:
        """
        x position; this will typically be latitude [WGS84, EGM96 ellipsoid].

        :return: x position
        :rtype: float
        """
        return self.__x

    @x.setter
    def x(self, pos: float) -> None:
        """
        Set the x position.

        :param pos: x position in the target frame
        :type pos: float
        """
        prev_pos = self.__x
        self.__x = pos

        # Signal state change event
        if self.__x != prev_pos:
            self.state_changed_event.notify(**self.context)

        return

    @property
    def y(self) -> float:
        """
        y position; this will typically be longitude [WGS84, EGM96 ellipsoid].

        :return: y position
        :rtype: float
        """
        return self.__y

    @y.setter
    def y(self, pos: float) -> None:
        """
        Set the y position.

        :param pos: y position in the target frame
        :type pos: float
        """
        prev_pos = self.__y
        self.__y = pos

        # Signal state change event
        if self.__y != prev_pos:
            self.state_changed_event.notify(**self.context)

        return

    @property
    def z(self) -> float:
        """
        z position; this will typically be altitude [MSL].

        Positive for up.

        :return: z position
        :rtype: float
        """
        return self.__z

    @z.setter
    def z(self, pos: float) -> None:
        """
        Set the z position.

        :param alt: z position in the target frame
        :type alt: float
        """
        prev_pos = self.__z
        self.__z = pos

        # Signal state change event
        if self.__z != prev_pos:
            self.state_changed_event.notify(**self.context)

        return

    @property
    def context(self) -> dict:
        """
        Position context.

        :return: context
        :rtype: dict
        """
        context = super().context

        context["x"] = self.__x
        context["y"] = self.__y
        context["z"] = self.__z

        return context

    def __str__(self) -> str:
        """
        Print position in a human-readable format.

        :return: position
        :rtype: str
        """
        return f"Position: {self.context}"
