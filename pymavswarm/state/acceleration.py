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


class Acceleration(State):
    """Acceleration state."""

    def __init__(self, optional_context_props: dict | None = None) -> None:
        """
        Create a new acceleration object.

        :param optional_context_props: optional properties to add to the context
        :type optional_context_props: dict, optional
        """
        super().__init__(optional_context_props)

        self.__local = Vector(
            0.0,
            0.0,
            0.0,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0.0,
            optional_context_props=optional_context_props,
        )
        self.__global = Vector(
            0.0,
            0.0,
            0.0,
            mavutil.mavlink.MAV_FRAME_GLOBAL,
            0.0,
            optional_context_props=optional_context_props,
        )
        self.__global_relative = Vector(
            0.0,
            0.0,
            0.0,
            mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT,
            0.0,
            optional_context_props=optional_context_props,
        )

        return

    @property
    def local_frame(self) -> Vector:
        """
        Acceleration in the local frame (MAV_FRAME_LOCAL_NED).

        :return: local acceleration
        :rtype: Vector
        """
        return self.__local

    @property
    def global_frame(self) -> Vector:
        """
        Acceleration in the global frame (MAV_FRAME_GLOBAL).

        :return: global acceleration
        :rtype: Vector
        """
        return self.__global

    @property
    def global_relative_frame(self) -> Vector:
        """
        Acceleration in the global relative frame (MAV_FRAME_GLOBAL_TERRAIN_ALT).

        :return: global relative acceleration
        :rtype: Vector
        """
        return self.__global_relative

    @property
    def context(self) -> dict:
        """
        Acceleration context.

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
        Print acceleration in a human-readable format.

        :return: acceleration
        :rtype: str
        """
        return f"Acceleration: {self.context}"
