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
    """Real-time reachability analysis for collision prevention."""

    def __init__(self) -> None:
        """Create a new SafetyChecker."""
        return

    def __make_neighborhood_rectangle(
        self,
        original: HyperRectangle,
        bloated: HyperRectangle,
        face: int,
        neighborhood_width: float,
    ) -> HyperRectangle:
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

    def __compute_derivative_bounds(
        self, rect: HyperRectangle, face: int, acceleration: tuple[float, float, float]
    ) -> float:
        """
        Compute position bounds using simplified quadrotor motion dynamics.

        The dynamics of the system are modeled as a double integrator where:

        - q_{double_overdot} = u(t)
        - y = q(t)

        with the state vector x(t) = [q, q_{overdot}], where q is the drone position
        (x, y, z) and q_{overdot} is the velocity of the agent (vx, vy, vz).

        :param rect: _description_
        :type rect: HyperRectangle
        :param face: _description_
        :type face: int
        :param acceleration: _description_
        :type acceleration: tuple[float, float, float]
        :return: _description_
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
            bounds = Interval(0, acceleration[0])
        elif dimension == 4:
            # v_y' = a_y
            bounds = Interval(0, acceleration[1])
        elif dimension == 5:
            # v_z' = v_z
            bounds = Interval(0, acceleration[2])

        return bounds.interval_min if face % 2 == 0 else bounds.interval_max

    def __single_face_lift(
        self,
        rect: HyperRectangle,
        acceleration: tuple[float, float, float],
        step_size: float,
        time_remaining: float,
    ) -> tuple[HyperRectangle, float]:
        bloated_rect = deepcopy(rect)
        recompute = True
        neighborhood_widths = np.zeros(rect.faces)
        derivatives: list[float] = []

        while recompute:
            recompute = False
            min_neighbor_cross_time = float("inf")

            for face in range(rect.faces):
                dimension = int(face / 2)
                is_min = face % 2 == 0

                # Compute the candidate neighborhood
                neighbor_rect = self.__make_neighborhood_rectangle(
                    rect, bloated_rect, face, neighborhood_widths[face]
                )

                # Compute the face derivative
                derivative = self.__compute_derivative_bounds(
                    neighbor_rect, face, acceleration
                )

                # Calculate the face widths
                prev_neighbor_width = neighborhood_widths[face]
                updated_neighbor_width = derivative * step_size

                # Check if the face width grew outward
                prev_grew_outward = (is_min and prev_neighbor_width < 0) or (
                    not is_min and prev_neighbor_width > 0
                )
                grew_outward = (is_min and updated_neighbor_width < 0) or (
                    not is_min and updated_neighbor_width > 0
                )

                # Prevent flipping from outward face to inward face
                if not grew_outward and prev_grew_outward:
                    updated_neighbor_width = 0
                    derivative = 0

                # Recompute if flipping from inward to outward or if the derivative
                # doubled
                if (not prev_grew_outward and grew_outward) or abs(
                    updated_neighbor_width
                ) > 2 * abs(prev_neighbor_width):
                    recompute = True

                # Adjust the bloated rectangle
                if recompute:
                    neighborhood_widths[face] = updated_neighbor_width

                    if is_min and updated_neighbor_width < 0:
                        bloated_rect.intervals[dimension].interval_min = (
                            rect.intervals[dimension].interval_min
                            + updated_neighbor_width
                        )
                    elif not is_min and updated_neighbor_width > 0:
                        bloated_rect.intervals[dimension].interval_max = (
                            rect.intervals[dimension].interval_max
                            + updated_neighbor_width
                        )
                else:
                    if (derivative < 0 and prev_neighbor_width > 0) or (
                        derivative > 0 and prev_neighbor_width < 0
                    ):
                        derivative = 0

                    if derivative != 0:
                        cross_time = prev_neighbor_width / derivative

                        if cross_time < min_neighbor_cross_time:
                            min_neighbor_cross_time = cross_time

                    derivatives.append(derivative)

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
            rect.intervals[dim].interval_min += (
                derivatives[2 * dimension] * time_to_elapse
            )
            rect.intervals[dim].interval_max += (
                derivatives[2 * dimension + 1] * time_to_elapse
            )

        if not bloated_rect.contains(rect):
            raise RuntimeError("Lifted rectangle is outside of the bloated rectangle")

        return rect, time_to_elapse

    def face_lifting_iterative_improvement(
        self,
        rect: HyperRectangle,
        local_start_time: float,
        acceleration: tuple[float, float, float],
        initial_step_size: float = 0.01,
        reach_time: float = 2.0,
        timeout: float = 0.01,
        min_step_size: float = 0.0000001,
    ) -> tuple[HyperRectangle, float]:
        """
        Perform the face lifting algorithm to compute the reachable set.

        :param rect: current state of the agent that sent the position message
        :type rect: HyperRectangle
        :param local_start_time: time since boot of the agent that sent the message in
            the local clock [ms]
        :type local_start_time: float
        :param acceleration: acceleration of the agent that sent the position message
        :type acceleration: tuple[float, float, float]
        :param initial_step_size: step size to start the algorithm with, defaults to
            0.01
        :type initial_step_size: float, optional
        :param reach_time: maximum reach time [s], defaults to 2.0
        :type reach_time: float, optional
        :param timeout: maximum time to execute the face lifting algorithm [s],
            defaults to 0.01
        :type timeout: float, optional
        :param min_step_size: minimum step size to allow before automatically
            cancelling the algorithm's execution, defaults to 0.0000001
        :type min_step_size: float, optional
        :return: reachable set, time in the local clock that the algorithm has
            computed forward
        :rtype: tuple[HyperRectangle, float]
        """
        # Begin with the initial step size, this will decrease each iteration to
        # improve the accuracy of the result
        step_size = initial_step_size

        # Initialize the return values
        hull = deepcopy(rect)
        end_time = local_start_time

        start_t = time.time()

        # Improve the accuracy of the reachable set iteratively until timeout
        while time.time() - start_t < timeout:
            # Exit early if the step size shrinks too much
            if step_size < min_step_size:
                break

            reach_time_remaining = reach_time
            reach_time_advance = 0.0

            while reach_time_remaining > 0:
                rect_result, reach_time_elapsed = self.__single_face_lift(
                    rect, acceleration, step_size, reach_time_remaining
                )

                updated_hull = hull.convex_hull(rect_result, in_place=False)

                if updated_hull is not None:
                    hull = updated_hull

                reach_time_advance += reach_time_elapsed
                end_time = local_start_time + 1000 * reach_time_advance

                reach_time_remaining -= reach_time_elapsed

            step_size /= 2.0

        return hull, end_time

    def check_collision(
        self,
        current_state: HyperRectangle,
        sender_state: HyperRectangle,
        allowable_distance: float,
    ) -> bool:
        """
        Check for a potential collision between the two agents.

        :param current_state: state of the agent that received the message
        :type current_state: HyperRectangle
        :param sender_state: state of the agent that sent the position message
        :type sender_state: HyperRectangle
        :param allowable_distance: minimum allowable distance between agents
        :type allowable_distance: float
        :return: potential collision detected
        :rtype: bool
        """
        current_min_pos = np.array(
            [
                current_state.intervals[0].interval_min,
                current_state.intervals[1].interval_min,
                current_state.intervals[2].interval_min,
            ]
        )

        current_max_pos = np.array(
            [
                current_state.intervals[0].interval_max,
                current_state.intervals[1].interval_max,
                current_state.intervals[2].interval_max,
            ]
        )

        sender_min_pos = np.array(
            [
                sender_state.intervals[0].interval_min,
                sender_state.intervals[1].interval_min,
                sender_state.intervals[2].interval_min,
            ]
        )

        sender_max_pos = np.array(
            [
                sender_state.intervals[0].interval_max,
                sender_state.intervals[1].interval_max,
                sender_state.intervals[2].interval_max,
            ]
        )

        # Calculate the distances between the intervals
        distances = [
            np.linalg.norm(sender_min_pos - current_min_pos),
            np.linalg.norm(sender_min_pos - current_max_pos),
            np.linalg.norm(sender_max_pos - current_min_pos),
            np.linalg.norm(sender_max_pos - current_max_pos),
        ]

        return min(distances) < allowable_distance  # type: ignore
