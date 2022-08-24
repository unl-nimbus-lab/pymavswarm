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


class Attitude(State):
    """Attitude of an agent."""

    def __init__(
        self,
        pitch: float,
        yaw: float,
        roll: float,
        pitch_speed: float,
        yaw_speed: float,
        roll_speed: float,
        optional_context_props: dict | None = None,
    ) -> None:
        """
        Create a new attitude state instance.

        :param pitch: pitch angle [-pi..+pi]
        :type pitch: float
        :param yaw: yaw angle [-pi..+pi]
        :type yaw: float
        :param roll: roll angle [-pi..+pi]
        :type roll: float
        :param pitch_speed: pitch angular speed [rad/s]
        :type pitch_speed: float
        :param yaw_speed: yaw angular speed [rad/s]
        :type yaw_speed: float
        :param roll_speed: roll angular speed [rad/s]
        :type roll_speed: float
        :param optional_context_props: optional properties to add to the context,
            defaults to None
        :type optional_context_props: dict | None, optional
        """
        super().__init__(optional_context_props)

        self.__pitch = pitch
        self.__yaw = yaw
        self.__roll = roll
        self.__pitch_speed = pitch_speed
        self.__yaw_speed = yaw_speed
        self.__roll_speed = roll_speed

        return

    @property
    def pitch(self) -> float:
        """
        Pitch angle [-pi..+pi].

        :return: pitch
        :rtype: float
        """
        return self.__pitch

    @pitch.setter
    def pitch(self, angle: float) -> None:
        """
        Set the pitch.

        :param angle: pitch angle [rad]
        :type angle: float
        """
        prev_pitch = self.__pitch
        self.__pitch = angle

        # Signal state change event
        if self.__pitch != prev_pitch:
            self.state_changed_event.notify(**self.context)

        return

    @property
    def yaw(self) -> float:
        """
        Yaw angle [-pi..+pi].

        :return: yaw
        :rtype: float
        """
        return self.__yaw

    @yaw.setter
    def yaw(self, angle: float) -> None:
        """
        Set the yaw.

        :param angle: yaw angle [rad]
        :type angle: float
        """
        prev_yaw = self.__yaw
        self.__yaw = angle

        # Signal state change event
        if self.__yaw != prev_yaw:
            self.state_changed_event.notify(**self.context)

        return

    @property
    def roll(self) -> float:
        """
        Roll angle [-pi..+pi].

        :return: roll
        :rtype: float
        """
        return self.__roll

    @roll.setter
    def roll(self, angle: float) -> None:
        """
        Set the roll.

        :param angle: roll angle [rad]
        :type angle: float
        """
        prev_roll = self.__roll
        self.__roll = angle

        # Signal state change event
        if self.__roll != prev_roll:
            self.state_changed_event.notify(**self.context)

        return

    @property
    def pitch_speed(self) -> float:
        """
        Pitch angular speed [rad/s].

        :return: pitch speed
        :rtype: float
        """
        return self.__pitch_speed

    @pitch_speed.setter
    def pitch_speed(self, rate: float) -> None:
        """
        Set the pitch speed.

        :param rate: pitch speed [rad/s]
        :type rate: float
        """
        prev_pitch_speed = self.__pitch_speed
        self.__pitch_speed = rate

        # Signal state change event
        if self.__pitch_speed != prev_pitch_speed:
            self.state_changed_event.notify(**self.context)

        return

    @property
    def roll_speed(self) -> float:
        """
        Roll angular speed [rad/s].

        :return: roll speed
        :rtype: float
        """
        return self.__roll_speed

    @roll_speed.setter
    def roll_speed(self, rate: float) -> None:
        """
        Set the roll speed.

        :param rate: roll speed [rad/s]
        :type rate: float
        """
        prev_roll_speed = self.__roll_speed
        self.__roll_speed = rate

        # Signal state change event
        if self.__roll_speed != prev_roll_speed:
            self.state_changed_event.notify(**self.context)

        return

    @property
    def yaw_speed(self) -> float:
        """
        Yaw angular speed [rad/s].

        :return: yaw speed
        :rtype: float
        """
        return self.__yaw_speed

    @yaw_speed.setter
    def yaw_speed(self, rate: float) -> None:
        """
        Set the yaw speed.

        :param rate: yaw speed [rad/s]
        :type rate: float
        """
        prev_yaw_speed = self.__yaw_speed
        self.__yaw_speed = rate

        # Signal state change event
        if self.__yaw_speed != prev_yaw_speed:
            self.state_changed_event.notify(**self.context)

        return

    @property
    def context(self) -> dict:
        """
        Attitude context.

        :return: context
        :rtype: dict
        """
        context = super().context

        context["roll"] = self.__roll
        context["pitch"] = self.__pitch
        context["yaw"] = self.__yaw
        context["roll_speed"] = self.__roll_speed
        context["pitch_speed"] = self.__pitch_speed
        context["yaw_speed"] = self.__yaw_speed

        return context

    def __str__(self) -> str:
        """
        Print attitude information in a human-readable format.

        :return: attitude
        :rtype: str
        """
        return f"Attitude: {self.context}"
