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

import os
from functools import partial

import matplotlib.pyplot as plt
import numpy as np


def normalize_value(num: float, range_min: float, range_max: float) -> float:
    """
    Normalize a value to the range [0, 1].

    :param num: number to normalize
    :type num: float
    :param range_min: minimum value of the number
    :type range_min: float
    :param range_max: maximum value of the number
    :type range_max: float
    :return: normalized value
    :rtype: float
    """
    return (num - range_min) / (range_max - range_min)


def normalize_rgb(rgb: tuple[float, float, float]) -> tuple[float, float, float]:
    """
    Normalize an RGB color to the [0, 1] range.

    :param rgb: RGB color to normalize
    :type rgb: tuple[float, float, float]
    :return: normalized RGB color
    :rtype: tuple[float, float, float]
    """
    normalize = partial(normalize_value, range_min=0.0, range_max=255.0)
    return (normalize(rgb[0]), normalize(rgb[1]), normalize(rgb[2]))


def main() -> None:
    """Demonstrate how to generate a post-mission report."""
    # Create a mission report directory
    report_dir = os.path.join(os.getcwd(), "results")

    if not os.path.isdir(report_dir):
        os.mkdir(report_dir)

    fig, ax = plt.subplots(subplot_kw={"projection": "3d"})

    # Set the colors for the plot
    colors = plt.cm.get_cmap("plasma")

    trajectories = {
        10: np.array(
            [
                [40.8519039, 40.8519039, 40.851912],
                [-96.6085634, -96.6085634, -96.608669],
                [0, 3.0, 3.0],
            ]
        ),
        5: np.array(
            [
                [40.8519161, 40.8519161, 40.851912],
                [-96.6087240, -96.6087240, -96.608669],
                [0, 3.0, 3.0],
            ]
        ),
    }

    # Plot each trajectory
    for idx, agent_id in enumerate(trajectories):
        ax.plot(
            trajectories[agent_id][0],
            trajectories[agent_id][1],
            trajectories[agent_id][2],
            label=agent_id,
            color=colors(normalize_value(idx, 0, len(trajectories))),
        )

    # Plot the collision point
    ax.scatter(40.851912, -96.608669, 3.0, s=70, color=colors(np.random.rand()))

    # Configure the labels
    ax.set_title("Planned Trajectories", color="#364a68")
    ax.set_xlabel("Latitude", labelpad=15, color="#364a68")
    ax.set_ylabel("Longitude", labelpad=15, color="#364a68")
    ax.zaxis.set_rotate_label(False)
    ax.set_zlabel("Altitude (m)", labelpad=2, rotation=92, color="#364a68")

    # Reduce the number of ticks along the side
    ax.locator_params(axis="x", nbins=5)
    ax.locator_params(axis="y", nbins=5)

    # Set the pane color
    pane_color = normalize_rgb((229, 236, 246))

    ax.w_xaxis.set_pane_color((*pane_color, 1.0))
    ax.w_yaxis.set_pane_color((*pane_color, 1.0))
    ax.w_zaxis.set_pane_color((*pane_color, 1.0))

    # Hide the axis lines
    ax.w_xaxis.line.set_color("white")
    ax.w_yaxis.line.set_color("white")
    ax.w_zaxis.line.set_color("white")

    # Set the tick label color
    ax.tick_params(axis="x", colors="#4e607a")
    ax.tick_params(axis="y", colors="#4e607a")
    ax.tick_params(axis="z", colors="#4e607a")

    # Hide the ticks
    ax.xaxis._axinfo["grid"]["color"] = "#ffffff"
    ax.xaxis._axinfo["tick"]["inward_factor"] = 0.0
    ax.xaxis._axinfo["tick"]["outward_factor"] = 0.0

    ax.yaxis._axinfo["grid"]["color"] = "#ffffff"
    ax.yaxis._axinfo["tick"]["inward_factor"] = 0.0
    ax.yaxis._axinfo["tick"]["outward_factor"] = 0.0

    ax.zaxis._axinfo["grid"]["color"] = "#ffffff"
    ax.zaxis._axinfo["tick"]["inward_factor"] = 0.0
    ax.zaxis._axinfo["tick"]["outward_factor"] = 0.0

    # Remove scientific notation
    ax.ticklabel_format(useOffset=False)

    # Set the initial camera view
    ax.view_init(azim=-135)

    # Add a legend
    legend = ax.legend(
        bbox_to_anchor=(1.05, 0.95),
        loc="upper left",
        borderaxespad=0,
        title="system_id",
        frameon=False,
        labelcolor="#364a68",
    )

    # Set the legend color
    plt.setp(legend.get_title(), color="#364a68")

    plt.savefig(os.path.join(report_dir, "planned_trajectories.png"))

    return


if __name__ == "__main__":
    main()
