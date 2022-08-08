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

        :param x: x position; this will typically be latitude [WGS84, EGM96 ellipsoid]
        :type x: float
        :param y: y position; this will typically be longitude [WGS84, EGM96 ellipsoid]
        :type y: float
        :param z: z position; this will typically be altitude
        :type altitude: float
        :param frame: coordinate frame that the vector exists in
        :type frame: int
        :param timestamp: timestamp that the message was initialized
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
        x position.

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
        y position.

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
        z position.

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

        :return: position
        :rtype: str
        """
        return f"Vector: {self.context}"
