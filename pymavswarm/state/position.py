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

from pymavlink import mavutil

from pymavswarm.state.state import State
from pymavswarm.state.vector import Vector


class Position(State):
    """Agent position."""

    def __init__(self, optional_context_props: dict | None = None) -> None:
        """
        Create a new location object.

        :param optional_context_props: properties to add to the location context,
            defaults to None
        :type optional_context_props: dict | None, optional
        """
        super().__init__(optional_context_props)

        self.__local = Vector(0.0, 0.0, 0.0, mavutil.mavlink.MAV_FRAME_LOCAL_NED, 0.0)
        self.__global = Vector(0.0, 0.0, 0.0, mavutil.mavlink.MAV_FRAME_GLOBAL, 0.0)
        self.__global_relative = Vector(
            0.0, 0.0, 0.0, mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT, 0.0
        )

        return

    @property
    def local_frame(self) -> Vector:
        """
        Position in the local frame (MAV_FRAME_LOCAL_NED).

        :return: local position
        :rtype: Vector
        """
        return self.__local

    @property
    def global_frame(self) -> Vector:
        """
        Position in the global frame (MAV_FRAME_GLOBAL).

        :return: global position
        :rtype: Vector
        """
        return self.__global

    @property
    def global_relative_frame(self) -> Vector:
        """
        Position in the global relative frame (MAV_FRAME_GLOBAL_TERRAIN_ALT).

        :return: global relative position
        :rtype: Vector
        """
        return self.__global_relative

    @property
    def context(self) -> dict:
        """
        Position context.

        :return: context
        :rtype: dict
        """
        context = super().context

        context["global_frame"] = self.__global
        context["global_relative_frame"] = self.__global_relative
        context["local_frame"] = self.__local

        return context

    def __str__(self) -> str:
        """
        Print position in a human-readable format.

        :return: position
        :rtype: str
        """
        return f"Position: {self.context}"
