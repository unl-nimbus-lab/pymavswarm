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


class Vector(State):
    """Vector in 3D space."""

    def __init__(
        self,
        x: float,
        y: float,
        z: float,
        frame: int,
        timestamp: float,
        optional_context_props: dict | None = None,
    ) -> None:
        """
        Create a new vector object.

        :param x: x component
        :type x: float
        :param y: y component
        :type y: float
        :param z: z component
        :type altitude: float
        :param frame: coordinate frame that the vector exists in
        :type frame: int
        :param timestamp: timestamp that the state was last updated
        :type timestamp: float
        :param optional_context_props: properties to add to the location context,
            defaults to None
        :type optional_context_props: dict | None, optional
        """
        super().__init__(optional_context_props)

        self.__x = x
        self.__y = y
        self.__z = z
        self.__frame = frame
        self.__timestamp = timestamp

        return

    @property
    def x(self) -> float:
        """
        x component.

        :return: x component
        :rtype: float
        """
        return self.__x

    @x.setter
    def x(self, value: float) -> None:
        """
        Set the x component.

        :param value: x component in the target frame
        :type value: float
        """
        self.__x = value

        return

    @property
    def y(self) -> float:
        """
        y component.

        :return: y component
        :rtype: float
        """
        return self.__y

    @y.setter
    def y(self, value: float) -> None:
        """
        Set the y component.

        :param value: y component in the target frame
        :type value: float
        """
        self.__y = value

        return

    @property
    def z(self) -> float:
        """
        z component.

        Valueitive for up.

        :return: z component
        :rtype: float
        """
        return self.__z

    @z.setter
    def z(self, value: float) -> None:
        """
        Set the z component.

        :param alt: z component in the target frame
        :type alt: float
        """
        self.__z = value

        return

    @property
    def frame(self) -> int:
        """
        Coordinate frame that the vector is in.

        :return: frame
        :rtype: int
        """
        return self.__frame

    @property
    def timestamp(self) -> float:
        """
        Timestamp that the vector was last updated.

        :return: timestamp
        :rtype: float
        """
        return self.__timestamp

    @timestamp.setter
    def timestamp(self, stamp: float) -> None:
        """
        Set the time that the vector was last updated.

        :param stamp: _description_
        :type stamp: float
        :return: _description_
        :rtype: _type_
        """
        prev_stamp = self.__timestamp
        self.__timestamp = stamp

        # Signal state change event
        if self.__timestamp != prev_stamp:
            self.state_changed_event.notify(**self.context)

        return

    @property
    def context(self) -> dict:
        """
        Vector context.

        :return: context
        :rtype: dict
        """
        context = super().context

        context["x"] = self.__x
        context["y"] = self.__y
        context["z"] = self.__z
        context["frame"] = self.__frame
        context["timestamp"] = self.__timestamp

        return context

    def __str__(self) -> str:
        """
        Print vector in a human-readable format.

        :return: vector
        :rtype: str
        """
        return f"Vector: {self.context}"
