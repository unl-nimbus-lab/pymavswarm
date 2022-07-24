from copy import deepcopy

import numpy as np

from pymavswarm.safety import HyperRectangle, Interval


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
        neighbor_width: float,
    ) -> HyperRectangle:
        result = deepcopy(bloated)
        original = deepcopy(original)

        dim = int(face / 2)

        if face % 2 == 0:
            result.intervals[dim].interval_min = original.intervals[dim].interval_min
            result.intervals[dim].interval_max = original.intervals[dim].interval_min
        else:
            result.intervals[dim].interval_min = original.intervals[dim].interval_max
            result.intervals[dim].interval_max = original.intervals[dim].interval_max

        if neighbor_width < 0:
            result.intervals[dim].interval_min += neighbor_width
        else:
            result.intervals[dim].interval_max += neighbor_width

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
        step_size: int,
        time_remaining: float,
    ) -> tuple[HyperRectangle, float]:
        bloated_rect = deepcopy(rect)
        recompute = True
        neighbor_widths = np.zeros(rect.faces)
        derivatives: list[float] = []

        while recompute:
            recompute = False
            min_neighbor_cross_time = float("inf")

            for face in range(rect.faces):
                dimension = int(face / 2)
                is_min = face % 2 == 0

                # Compute the candidate neighborhood
                neighbor_rect = self.__make_neighborhood_rectangle(
                    rect, bloated_rect, face, neighbor_widths[face]
                )

                # Compute the face derivative
                derivative = self.__compute_derivative_bounds(
                    neighbor_rect, face, acceleration
                )

                # Calculate the face widths
                prev_neighbor_width = neighbor_widths[face]
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
                    neighbor_widths[face] = updated_neighbor_width

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