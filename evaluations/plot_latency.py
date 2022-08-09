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
from typing import Any

import pandas as pd
import plotly.express as px

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


def main() -> None:
    """Demonstrate how to generate a post-mission report."""
    args = parse_args()

    # Parse the log file
    log = parse_log_file(args.logfile)

    # Create a mission report directory
    report_dir = os.path.join(os.getcwd(), "evaluations")

    if not os.path.isdir(report_dir):
        os.mkdir(report_dir)

    # Evaluate the network latency
    ping_df = log["PING"][["timestamp", "system_id", "component_id", "ping"]].apply(
        pd.to_numeric
    )
    ping_df["timestamp"] = (
        ping_df["timestamp"].subtract(ping_df["timestamp"].min()).divide(1000)
    )
    ping_df = ping_df[ping_df["component_id"] == 1]

    ping_fig = px.line(
        ping_df,
        x="timestamp",
        y="ping",
        color="system_id",
        title="Network Latency",
        labels={"timestamp": "Timestamp (s)", "ping": "Latency (ms)"},
    )
    ping_fig.write_image(os.path.join(report_dir, "latency_plot.png"))

    return


if __name__ == "__main__":
    main()
