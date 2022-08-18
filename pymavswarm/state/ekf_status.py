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


class EKFStatus(State):
    """Agent EKF status."""

    def __init__(
        self,
        velocity_variance: float,
        pos_horiz_variance: float,
        pos_vert_variance: float,
        compass_variance: float,
        terrain_alt_variance: float,
        pos_horiz_abs: float,
        const_pos_mode: float,
        pred_pos_horiz_abs: float,
        optional_context_props: dict | None = None,
    ) -> None:
        """
        Create an EKF status.

        :param velocity_variance: velocity variance
        :type velocity_variance: float
        :param pos_horiz_variance: horizontal position variance
        :type pos_horiz_variance: float
        :param pos_vert_variance: vertical position variance
        :type pos_vert_variance: float
        :param compass_variance: compass variance
        :type compass_variance: float
        :param terrain_alt_variance: terrain altitude variance
        :type terrain_alt_variance: float
        :param pos_horiz_abs: flags, EKF's Horizontal position (absolute) estimate is
            good
        :type pos_horiz_abs: float
        :param const_pos_mode: flags, EKF is in constant position mode and does not
            know  it's absolute or relative position
        :type const_pos_mode: float
        :param pred_pos_horiz_abs: EKF's predicted horizontal position (absolute)
            estimate is good
        :type pred_pos_horiz_abs: float
        :param optional_context_props: optional properties to add to the EKF status
            context, defaults to None
        :type optional_context_props: dict | None, optional
        """
        super().__init__(optional_context_props)

        self.__velocity_variance = velocity_variance
        self.__pos_horiz_variance = pos_horiz_variance
        self.__pos_vert_variance = pos_vert_variance
        self.__compass_variance = compass_variance
        self.__terrain_alt_variance = terrain_alt_variance
        self.__pos_horiz_abs = pos_horiz_abs
        self.__const_pos_mode = const_pos_mode
        self.__pred_pos_horiz_abs = pred_pos_horiz_abs

        return

    @property
    def velocity_variance(self) -> float:
        """
        Velocity variance.

        :return: variance
        :rtype: float
        """
        return self.__velocity_variance

    @velocity_variance.setter
    def velocity_variance(self, variance: float) -> None:
        """
        Set the velocity variance.

        :param variance: velocity variance
        :type variance: float
        """
        prev_velocity_variance = self.__velocity_variance
        self.__velocity_variance = variance

        # Signal state change event
        if self.__velocity_variance != prev_velocity_variance:
            self.state_changed_event.notify(**self.context)

        return

    @property
    def pos_horiz_variance(self) -> float:
        """
        Horizontal position variance.

        :return: position variance
        :rtype: float
        """
        return self.__pos_horiz_variance

    @pos_horiz_variance.setter
    def pos_horiz_variance(self, variance: float) -> None:
        """
        Set the horizontal position variance.

        :param variance: horizontal position variance
        :type variance: float
        """
        prev_pov_horiz_variance = self.__pos_horiz_variance
        self.__pos_horiz_variance = variance

        # Signal state change event
        if self.__pos_horiz_variance != prev_pov_horiz_variance:
            self.state_changed_event.notify(**self.context)

        return

    @property
    def pos_vert_variance(self) -> float:
        """
        Vertical position variance.

        :return: position variance
        :rtype: float
        """
        return self.__pos_vert_variance

    @pos_vert_variance.setter
    def pos_vert_variance(self, variance: float) -> None:
        """
        Set the vertical position variance.

        :param variance: vertical position variance
        :type variance: float
        """
        prev_pos_vert_variance = self.__pos_vert_variance
        self.__pos_vert_variance = variance

        # Signal state change event
        if self.__pos_vert_variance != prev_pos_vert_variance:
            self.state_changed_event.notify(**self.context)

        return

    @property
    def compass_variance(self) -> float:
        """
        Compass variance.

        :return: variance
        :rtype: float
        """
        return self.__compass_variance

    @compass_variance.setter
    def compass_variance(self, variance: float) -> None:
        """
        Set the compass variance.

        :param variance: compass variance
        :type variance: float
        """
        prev_compass_variance = self.__compass_variance
        self.__compass_variance = variance

        # Signal state change event
        if self.__compass_variance != prev_compass_variance:
            self.state_changed_event.notify(**self.context)

        return

    @property
    def terrain_alt_variance(self) -> float:
        """
        Terrain altitude variance.

        :return: variance
        :rtype: float
        """
        return self.__terrain_alt_variance

    @terrain_alt_variance.setter
    def terrain_alt_variance(self, variance: float) -> None:
        """
        Set the terrain altitude variance.

        :param variance: terrain altitude variance
        :type variance: float
        """
        prev_terrain_alt_variance = self.__terrain_alt_variance
        self.__terrain_alt_variance = variance

        # Signal state change event
        if self.__terrain_alt_variance != prev_terrain_alt_variance:
            self.state_changed_event.notify(**self.context)

        return

    @property
    def pos_horiz_abs(self) -> float:
        """
        EKF Flags.

        EKF's Horizontal position (absolute) estimate is good.

        :return: flags
        :rtype: float
        """
        return self.__pos_horiz_abs

    @pos_horiz_abs.setter
    def pos_horiz_abs(self, flags: float) -> None:
        """
        Set the horizontal position flags.

        :param flags: flags
        :type flags: float
        """
        prev_pos_horiz_abs = self.__pos_horiz_abs
        self.__pos_horiz_abs = flags

        # Signal state change event
        if self.__pos_horiz_abs != prev_pos_horiz_abs:
            self.state_changed_event.notify(**self.context)

        return

    @property
    def const_pos_mode(self) -> float:
        """
        EKF Flags.

        EKF is in constant position mode and does not know it's absolute or
        relative position.

        :return: flags
        :rtype: float
        """
        return self.__const_pos_mode

    @const_pos_mode.setter
    def const_pos_mode(self, flags: float) -> None:
        """
        Set the constant position mode flags.

        :param flags: flags
        :type flags: float
        """
        prev_const_pos_mode = self.__const_pos_mode
        self.__const_pos_mode = flags

        # Signal state change event
        if self.__const_pos_mode != prev_const_pos_mode:
            self.state_changed_event.notify(**self.context)

        return

    @property
    def pred_pos_horiz_abs(self) -> float:
        """
        EKF flags.

        EKF's predicted horizontal position (absolute) estimate is good.

        :return: flags
        :rtype: float
        """
        return self.__pred_pos_horiz_abs

    @pred_pos_horiz_abs.setter
    def pred_pos_horiz_abs(self, pred: float) -> None:
        """
        Set the predicted horizontal position flags.

        :param pred: flags
        :type pred: float
        """
        prev_pred_pos_horiz_abs = self.__pred_pos_horiz_abs
        self.__pred_pos_horiz_abs = pred

        # Signal state change event
        if self.__pred_pos_horiz_abs != prev_pred_pos_horiz_abs:
            self.state_changed_event.notify(**self.context)

        return

    @property
    def context(self) -> dict:
        """
        EKF status context.

        :return: context
        :rtype: dict
        """
        context = super().context

        context["velocity_variance"] = self.__velocity_variance
        context["pos_horiz_variance"] = self.__pos_horiz_variance
        context["pos_vert_variance"] = self.__pos_vert_variance
        context["compass_variance"] = self.__compass_variance
        context["terrain_alt_variance"] = self.__terrain_alt_variance
        context["pos_horiz_abs"] = self.__pos_horiz_abs
        context["const_pos_mode"] = self.__const_pos_mode
        context["pred_pos_horiz_abs"] = self.__pred_pos_horiz_abs

        return context

    def __str__(self) -> str:
        """
        Print EKF status information in a human-readable format.

        :return: EKF status
        :rtype: str
        """
        return f"EKFStatus: {self.context}"
