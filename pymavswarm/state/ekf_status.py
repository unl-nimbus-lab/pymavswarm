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


class EKFStatus(State):
    """
    EKF Flags indicating EKF status (True if healthy)
    """

    def __init__(
        self,
        velocity_variance: float = 0.0,
        pos_horiz_variance: float = 0.0,
        pos_vert_variance: float = 0.0,
        compass_variance: float = 0.0,
        terrain_alt_variance: float = 0.0,
        pos_horiz_abs: float = 0.0,
        const_pos_mode: float = 0.0,
        pred_pos_horiz_abs: float = 0.0,
        optional_context_props: dict = {},
    ) -> None:
        """
        :param velocity_variance: Velocity variance, defaults to 0.0
        :type velocity_variance: float, optional

        :param pos_horiz_variance: Horizontal position variance, defaults to 0.0
        :type pos_horiz_variance: float, optional

        :param pos_vert_variance: Vertical position variance, defaults to 0.0
        :type pos_vert_variance: float, optional

        :param compass_variance: Compass variance, defaults to 0.0
        :type compass_variance: float, optional

        :param terrain_alt_variance: Terrain altitude variance, defaults to 0.0
        :type terrain_alt_variance: float, optional

        :param pos_horiz_abs: Flags, EKF's Horizontal position (absolute) estimate is
            good, defaults to 0.0
        :type pos_horiz_abs: float, optional

        :param const_pos_mode: Flags, EKF is in constant position mode and does not
            know  it's absolute or relative position, defaults to 0.0
        :type const_pos_mode: float, optional

        :param pred_pos_horiz_abs: EKF's predicted horizontal position (absolute)
            estimate is good, defaults to 0.0
        :type pred_pos_horiz_abs: float, optional

        :param optional_context_props: Optional properties to add to the context
        :type optional_context_props: dict, optional
        """
        super().__init__()

        self.__velocity_variance = velocity_variance
        self.__pos_horiz_variance = pos_horiz_variance
        self.__pos_vert_variance = pos_vert_variance
        self.__compass_variance = compass_variance
        self.__terrain_alt_variance = terrain_alt_variance
        self.__pos_horiz_abs = pos_horiz_abs
        self.__const_pos_mode = const_pos_mode
        self.__pred_pos_horiz_abs = pred_pos_horiz_abs
        self.__optional_context_props = optional_context_props

        return

    @property
    def velocity_variance(self) -> float:
        """
        Velocity variance

        :rtype: float
        """
        return self.__velocity_variance

    @velocity_variance.setter
    def velocity_variance(self, variance: float) -> None:
        """
        velocity_variance setter

        :param variance: Velocity variance
        :type variance: float
        """
        prev_velocity_variance = self.__velocity_variance
        self.__velocity_variance = variance

        # Signal state change event
        if self.__velocity_variance != prev_velocity_variance:
            self.state_changed_event.notify(context=self.context)

        return

    @property
    def pos_horiz_variance(self) -> float:
        """
        Horizontal position variance

        :rtype: float
        """
        return self.__pos_horiz_variance

    @pos_horiz_variance.setter
    def pos_horiz_variance(self, variance: float) -> None:
        """
        pos_horiz_variance setter

        :param variance: Horizontal position variance
        :type variance: float
        """
        prev_pov_horiz_variance = self.__pos_horiz_variance
        self.__pos_horiz_variance = variance

        # Signal state change event
        if self.__pos_horiz_variance != prev_pov_horiz_variance:
            self.state_changed_event.notify(context=self.context)

        return

    @property
    def pos_vert_variance(self) -> float:
        """
        Vertical position variance

        :rtype: float
        """
        return self.__pos_vert_variance

    @pos_vert_variance.setter
    def pos_vert_variance(self, variance: float) -> None:
        """
        pos_vert_variance setter

        :param variance: Vertical position variance
        :type variance: float
        """
        prev_pos_vert_variance = self.__pos_vert_variance
        self.__pos_vert_variance = variance

        # Signal state change event
        if self.__pos_vert_variance != prev_pos_vert_variance:
            self.state_changed_event.notify(context=self.context)

        return

    @property
    def compass_variance(self) -> float:
        """
        Compass variance

        :rtype: float
        """
        return self.__compass_variance

    @compass_variance.setter
    def compass_variance(self, variance: float) -> None:
        """
        compass_variance setter

        :param variance: Compass variance
        :type variance: float
        """
        prev_compass_variance = self.__compass_variance
        self.__compass_variance = variance

        # Signal state change event
        if self.__compass_variance != prev_compass_variance:
            self.state_changed_event.notify(context=self.context)

        return

    @property
    def terrain_alt_variance(self) -> float:
        """
        Terrain altitude variance

        :rtype: float
        """
        return self.__terrain_alt_variance

    @terrain_alt_variance.setter
    def terrain_alt_variance(self, variance: float) -> None:
        """
        terrain_alt_variance setter

        :param variance: Terrain altitude variance
        :type variance: float
        """
        prev_terrain_alt_variance = self.__terrain_alt_variance
        self.__terrain_alt_variance = variance

        # Signal state change event
        if self.__terrain_alt_variance != prev_terrain_alt_variance:
            self.state_changed_event.notify(context=self.context)

        return

    @property
    def pos_horiz_abs(self) -> float:
        """
        Flags, EKF's Horizontal position (absolute) estimate is good

        :rtype: float
        """
        return self.__pos_horiz_abs

    @pos_horiz_abs.setter
    def pos_horiz_abs(self, flags: float) -> None:
        """
        pos_horiz_abs setter

        :param flags: EKF flags
        :type flags: float
        """
        prev_pos_horiz_abs = self.__pos_horiz_abs
        self.__pos_horiz_abs = flags

        # Signal state change event
        if self.__pos_horiz_abs != prev_pos_horiz_abs:
            self.state_changed_event.notify(context=self.context)

        return

    @property
    def const_pos_mode(self) -> float:
        """
        Flags, EKF is in constant position mode and does not know it's absolute or
        relative position

        :rtype: float
        """
        return self.__const_pos_mode

    @const_pos_mode.setter
    def const_pos_mode(self, flags: float) -> None:
        """
        const_pos_mode setter

        :param flags: EKF flags
        :type flags: float
        """
        prev_const_pos_mode = self.__const_pos_mode
        self.__const_pos_mode = flags

        # Signal state change event
        if self.__const_pos_mode != prev_const_pos_mode:
            self.state_changed_event.notify(context=self.context)

        return

    @property
    def pred_pos_horiz_abs(self) -> float:
        """
        EKF's predicted horizontal position (absolute) estimate is good

        :rtype: float
        """
        return self.__pred_pos_horiz_abs

    @pred_pos_horiz_abs.setter
    def pred_pos_horiz_abs(self, pred: float) -> None:
        """
        pred_pos_horiz_abs setter

        :param pred: Predicted position
        :type pred: float
        """
        prev_pred_pos_horiz_abs = self.__pred_pos_horiz_abs
        self.__pred_pos_horiz_abs = pred

        # Signal state change event
        if self.__pred_pos_horiz_abs != prev_pred_pos_horiz_abs:
            self.state_changed_event.notify(context=self.context)

        return

    @property
    def context(self) -> dict:
        """
        Get the current state as a dictionary for callbacks

        :return: Properties of interested associated with the EKF status
        :rtype: dict
        """
        context = {
            "velocity_variance": self.__velocity_variance,
            "pos_horiz_variance": self.__pos_horiz_variance,
            "pos_vert_variance": self.__pos_vert_variance,
            "compass_variance": self.__compass_variance,
            "terrain_alt_variance": self.__terrain_alt_variance,
            "pos_horiz_abs": self.__pos_horiz_abs,
            "const_pos_mode": self.__const_pos_mode,
            "pred_pos_horiz_abs": self.__pred_pos_horiz_abs,
        }
        context.update(self.__optional_context_props)

        return context
