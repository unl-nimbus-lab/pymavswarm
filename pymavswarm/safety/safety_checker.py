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

import time
from copy import deepcopy

import numpy as np

from pymavswarm.safety.hyperrectangle import HyperRectangle
from pymavswarm.safety.interval import Interval


class SafetyChecker:
    """
    Real-time reachability analysis for collision prevention.

    This implementation is inspired by the following source:

    Tran, H. D., Nguyen, L. V., Musau, P., Xiang, W., & Johnson, T. T. (2019, June).
    Decentralized real-time safety verification for distributed cyber-physical systems.
    In International Conference on Formal Techniques for Distributed Objects,
    Components, and Systems (pp. 261-277). Springer, Cham.
    """

    @staticmethod
    def make_neighborhood_rectangle(
        original: HyperRectangle,
        bloated: HyperRectangle,
        face: int,
        neighborhood_width: float,
    ) -> HyperRectangle:
        """
        Create a face's neighborhood given a target width.

        :param original: initial rectangle to
        :type original: HyperRectangle
        :param bloated: rectangle that should be used to create the neighborhood
        :type bloated: HyperRectangle
        :param face: face who's neighborhood should be created
        :type face: int
        :param neighborhood_width: width of the neighborhood
        :type neighborhood_width: float
        :return: neighborhood hyperrectangle
        :rtype: HyperRectangle
        """
        result = deepcopy(bloated)

        dim = int(face / 2)

        if face % 2 == 0:
            result.intervals[dim].interval_min = original.intervals[dim].interval_min
            result.intervals[dim].interval_max = original.intervals[dim].interval_min
        else:
            result.intervals[dim].interval_min = original.intervals[dim].interval_max
            result.intervals[dim].interval_max = original.intervals[dim].interval_max

        if neighborhood_width < 0:
            result.intervals[dim].interval_min += neighborhood_width
        else:
            result.intervals[dim].interval_max += neighborhood_width

        return result

    @staticmethod
    def compute_derivative_bounds(
        rect: HyperRectangle, face: int, acceleration: tuple[float, float, float]
    ) -> float:
        """
        Compute position bounds using simplified quadrotor motion dynamics.

        The dynamics of the system are modeled as a double integrator where:

        - q_{double_overdot} = u(t)
        - y = q(t)

        with the state vector x(t) = [q, q_{overdot}], where q is the drone position
        (x, y, z) and q_{overdot} is the velocity of the agent (vx, vy, vz) [m/s].

        :param rect: rectangle that should be used to compute the face derivative
        :type rect: HyperRectangle
        :param face: face who's derivative should be computed
        :type face: int
        :param acceleration: control signal; current acceleration of the agent [m/s^2]
        :type acceleration: tuple[float, float, float]
        :return: derivative of the current face
        :rtype: float
        """
        dimension = int(face / 2)

        # If the dimension is position, use the velocity as the bounds
        if dimension == 0:
            # x' = v_x
            bounds = Interval(
                rect.intervals[3].interval_min, rect.intervals[3].interval_max
            )
        elif dimension == 1:
            # y' = v_y
            bounds = Interval(
                rect.intervals[4].interval_min, rect.intervals[4].interval_max
            )
        elif dimension == 2:
            # z' = v_z
            bounds = Interval(
                rect.intervals[5].interval_min, rect.intervals[5].interval_max
            )

        # If the dimension is velocity, return the control signal (acceleration)
        elif dimension == 3:
            # v_x' = a_x
            bounds = Interval(acceleration[0], acceleration[0])
        elif dimension == 4:
            # v_y' = a_y
            bounds = Interval(acceleration[1], acceleration[1])
        elif dimension == 5:
            # v_z' = v_z
            bounds = Interval(acceleration[2], acceleration[2])

        return bounds.interval_min if face % 2 == 0 else bounds.interval_max

    @staticmethod
    def single_face_lift(
        rect: HyperRectangle,
        acceleration: tuple[float, float, float],
        step_size: float,
        time_remaining: float,
    ) -> tuple[HyperRectangle, float]:
        """
        Perform a single face lifting operation.

        :param rect: hyperrectangle that the face lifting operation should be applied to
        :type rect: HyperRectangle
        :param acceleration: current acceleration of the agent [m/s^2]
        :type acceleration: tuple[float, float, float]
        :param step_size: scale to apply to the derivative
        :type step_size: float
        :param time_remaining: amount of time left to perform the face lift
        :type time_remaining: float
        :raises RuntimeError: minimum neighborhood cross time is less than half of the
            step size
        :raises RuntimeError: lifted rectangle is outside of the bloated rectangle
        :return: lifted rectangle, elapsed reach time
        :rtype: tuple[HyperRectangle, float]
        """
        bloated_rect = deepcopy(rect)
        neighborhood_widths = np.zeros(rect.faces)
        derivatives = np.zeros(rect.faces)

        recompute = True

        while recompute:
            recompute = False
            min_neighbor_cross_time = float("inf")

            for face in range(rect.faces):
                # Get the target dimension
                dimension = int(face / 2)

                # Determine whether the face is a minimum face or a maximum face
                is_min = face % 2 == 0

                # Compute the candidate neighborhood
                neighbor_rect = SafetyChecker.make_neighborhood_rectangle(
                    rect, bloated_rect, face, neighborhood_widths[face]
                )

                # Compute the face derivative
                derivative = SafetyChecker.compute_derivative_bounds(
                    neighbor_rect, face, acceleration
                )

                # Calculate the face widths
                prev_neighborhood_width = neighborhood_widths[face]
                updated_neighborhood_width = derivative * step_size

                # Check if the face width grew outward
                prev_grew_outward = (is_min and prev_neighborhood_width < 0) or (
                    not is_min and prev_neighborhood_width > 0
                )
                grew_outward = (is_min and updated_neighborhood_width < 0) or (
                    not is_min and updated_neighborhood_width > 0
                )

                # Prevent flipping from outward face to inward face
                if not grew_outward and prev_grew_outward:
                    updated_neighborhood_width = 0
                    derivative = 0

                # Recompute if flipping from inward to outward or if the derivative
                # doubled
                if (not prev_grew_outward and grew_outward) or abs(
                    updated_neighborhood_width
                ) > 2 * abs(prev_neighborhood_width):
                    recompute = True

                # Adjust the bloated rectangle
                if recompute:
                    neighborhood_widths[face] = updated_neighborhood_width

                    if is_min and updated_neighborhood_width < 0:
                        bloated_rect.intervals[dimension].interval_min = (
                            rect.intervals[dimension].interval_min
                            + updated_neighborhood_width
                        )
                    elif not is_min and updated_neighborhood_width > 0:
                        bloated_rect.intervals[dimension].interval_max = (
                            rect.intervals[dimension].interval_max
                            + updated_neighborhood_width
                        )
                else:
                    if (derivative < 0 and prev_neighborhood_width > 0) or (
                        derivative > 0 and prev_neighborhood_width < 0
                    ):
                        derivative = 0

                    if derivative != 0:
                        cross_time = prev_neighborhood_width / derivative

                        if cross_time < min_neighbor_cross_time:
                            min_neighbor_cross_time = cross_time

                    derivatives[face] = derivative

        if min_neighbor_cross_time * 2 < step_size:
            raise RuntimeError(
                "The minimum neighborhood cross time is less than half of the step size"
            )

        time_to_elapse = min_neighbor_cross_time

        if time_remaining < time_to_elapse:
            time_to_elapse = time_remaining

        # Make a copy of the rectangle to prevent accidental modifications to the
        # original
        rect = deepcopy(rect)

        # Perform the face lift
        for dim in range(rect.dimensions):
            rect.intervals[dim].interval_min += derivatives[2 * dim] * time_to_elapse
            rect.intervals[dim].interval_max += (
                derivatives[2 * dim + 1] * time_to_elapse
            )

        if not bloated_rect.contains(rect):
            raise RuntimeError("Lifted rectangle is outside of the bloated rectangle")

        return rect, time_to_elapse

    @staticmethod
    def face_lifting_iterative_improvement(
        rect: HyperRectangle,
        start_time: float,
        acceleration: tuple[float, float, float],
        reach_time: float,
        initial_step_size: float = 0.01,
        timeout: float = 0.01,
        min_step_size: float = 0.0000001,
    ) -> tuple[HyperRectangle, float]:
        """
        Perform iterative face lifting to compute the reachable state.

        :param rect: initial state of the agent that sent the position message
        :type rect: HyperRectangle
        :param start_time: estimated time since boot of the agent that sent the
            message in the global clock [ms]
        :type start_time: float
        :param acceleration: acceleration [m/s^2] of the agent that sent the position
            message
        :type acceleration: tuple[float, float, float]
        :param reach_time: maximum reach time [s], defaults to 2.0
        :type reach_time: float, optional
        :param initial_step_size: step size to start the algorithm with, defaults to
            0.01
        :type initial_step_size: float, optional
        :param timeout: maximum time to execute the face lifting algorithm [s],
            defaults to 0.01
        :type timeout: float, optional
        :param min_step_size: minimum step size to allow before automatically
            cancelling the algorithm's execution, defaults to 0.0000001
        :type min_step_size: float, optional
        :return: reachable set, time in the global clock that the algorithm has
            computed forward to
        :rtype: tuple[HyperRectangle, float]
        """
        # Begin with the initial step size, this will decrease each iteration to
        # improve the accuracy of the result
        step_size = initial_step_size

        # Initialize the return values
        hull = deepcopy(rect)
        end_time = start_time

        start_t = time.time()

        # Improve the accuracy of the reachable set iteratively until timeout
        while time.time() - start_t < timeout:
            # Exit early if the step size shrinks too much
            if step_size < min_step_size:
                break

            reach_time_remaining = reach_time
            reach_time_advance = 0.0

            while reach_time_remaining > 0:
                rect, reach_time_elapsed = SafetyChecker.single_face_lift(
                    rect, acceleration, step_size, reach_time_remaining
                )

                hull.convex_hull(rect, in_place=True)

                reach_time_advance += reach_time_elapsed
                end_time = start_time + 1000 * reach_time_advance

                reach_time_remaining -= reach_time_elapsed

            step_size /= 2.0

        return hull, end_time
