from copy import deepcopy

import numpy as np

from pymavswarm.safety import HyperRectangle


class SafetyChecker:
    def __init__(self) -> None:
        pass

    def __make_neighborhood_rect(
        self,
        original: HyperRectangle,
        bloated: HyperRectangle,
        face: int,
        neighbor_width: float,
    ) -> HyperRectangle:
        result = deepcopy(bloated)

        face = int(face / 2)

        if face % 2 == 0:
            result.dimensions[face].interval_min = original.dimensions[
                face
            ].interval_min
            result.dimensions[face].interval_max = original.dimensions[
                face
            ].interval_max
        else:
            result.dimensions[face].interval_min = original.dimensions[
                face
            ].interval_max
            result.dimensions[face].interval_max = original.dimensions[
                face
            ].interval_max

        if neighbor_width < 0:
            result.dimensions[face].interval_min += neighbor_width
        else:
            result.dimensions[face].interval_max += neighbor_width

        return result

    def __single_face_lift(self, rect: HyperRectangle, step: int, timeout: float):
        bloated_rect = deepcopy(rect)
        recompute = True
        neighbor_widths = np.zeros(rect.num_faces)

        while recompute:
            recompute = False

            for face in range(rect.num_faces):
                dimension = int(face / 2)

                neighborhood_rect = deepcopy(bloated_rect)

                # Construct a candidate neighborhood
                if face % 2 == 0:
                    neighborhood_rect.dimensions[
                        dimension
                    ].interval_min = rect.dimensions[dimension].interval_min
                    neighborhood_rect.dimensions[
                        dimension
                    ].interval_max = rect.dimensions[dimension].interval_max
                else:
                    neighborhood_rect.dimensions[
                        dimension
                    ].interval_min = rect.dimensions[dimension].interval_max
                    neighborhood_rect.dimensions[
                        dimension
                    ].interval_max = rect.dimensions[dimension].interval_max

                neighbor_width = neighbor_widths[face]

                if neighbor_width < 0:
                    neighborhood_rect.dimensions[face].interval_min += neighbor_width
                else:
                    neighborhood_rect.dimensions[face].interval_max += neighbor_width

                # TODO: Implement derivative computation
                derivative = 0

                prev_width = neighbor_width[face]
                new_neighbor_width = derivative * step

                grew_outward = (face % 2 == 0 and new_neighbor_width < 0) or (
                    face % 2 != 0 and new_neighbor_width > 0
                )
                prev_grew_outward = (face % 2 == 0 and prev_width < 0) or (
                    face % 2 != 0 and prev_width > 0
                )

                if not grew_outward and prev_grew_outward:
                    new_neighbor_width = 0
                    derivative = 0

                if not prev_grew_outward and grew_outward:
                    recompute = True
