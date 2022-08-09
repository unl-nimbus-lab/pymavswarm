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
from argparse import ArgumentParser
from functools import partial
from typing import Any

import matplotlib.pyplot as plt
import pandas as pd

from pymavswarm.utils import parse_log_file


def parse_args() -> Any:
    """
    Parse the script arguments.

    :return: argument namespace
    :rtype: Any
    """
    parser = ArgumentParser()
    parser.add_argument(
        "logfile", type=str, help="full path to the log file that should be parsed"
    )
    return parser.parse_args()


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
    args = parse_args()

    # Parse the log file
    log = parse_log_file(args.logfile)

    # Create a mission report directory
    report_dir = os.path.join(os.getcwd(), "evaluations")

    if not os.path.isdir(report_dir):
        os.mkdir(report_dir)

    agent_ids_df = log["HEARTBEAT"][["system_id", "component_id"]].apply(pd.to_numeric)
    agent_ids_df = agent_ids_df[agent_ids_df["component_id"] == 1]
    agent_ids = set(map(tuple, agent_ids_df.to_numpy()))

    fig, ax = plt.subplots(subplot_kw={"projection": "3d"})

    # Set the colors for the plot
    colors = plt.cm.get_cmap("Set1")

    # Plot each trajectory
    for idx, agent_id in enumerate(agent_ids):
        trajectories_df = log["GLOBAL_POSITION_INT"][
            ["system_id", "lat", "lon", "relative_alt"]
        ].apply(pd.to_numeric)
        trajectories_df = trajectories_df[trajectories_df["system_id"] == agent_id[0]]

        trajectories_df[["lat", "lon"]] = trajectories_df[["lat", "lon"]].div(1e7)
        trajectories_df["relative_alt"] = trajectories_df["relative_alt"].div(1e3)

        ax.plot(
            trajectories_df["lat"].to_numpy(),
            trajectories_df["lon"].to_numpy(),
            trajectories_df["relative_alt"].to_numpy(),
            label=agent_id[0],
            color=colors(normalize_value(idx, 0, len(agent_ids))),
        )

    # Configure the labels
    ax.set_title("Multi-Agent Collision Avoidance", color="#364a68", loc="left")
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

    plt.savefig(os.path.join(report_dir, "trajectories.png"))

    return


if __name__ == "__main__":
    main()
