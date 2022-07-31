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
    report_dir = os.path.join(os.getcwd(), "mission_report")

    if not os.path.isdir(report_dir):
        os.mkdir(report_dir)

    # Get the agents seen in the network
    agent_ids_df = log["HEARTBEAT"][["system_id", "component_id"]]
    agent_ids = set(map(tuple, agent_ids_df.to_numpy()))

    # Evaluate the network latency
    ping_df = log["PING"][["timestamp", "system_id", "component_id", "ping"]].astype(
        int
    )
    ping = ping_df["ping"].astype(int).to_numpy()

    ping_fig = px.line(
        ping_df,
        x="timestamp",
        y="ping",
        color="system_id",
        title="Network Latency",
        labels={"timestamp": "Timestamp", "ping": "Latency (ms)"},
    )
    ping_fig.write_html(os.path.join(report_dir, "latency_plot.html"))

    # Evaluate the agent trajectories
    trajectories_df = log["GLOBAL_POSITION_INT"][
        ["system_id", "lat", "lon", "relative_alt"]
    ].astype(int)

    trajectories_df[["lat", "lon"]] = trajectories_df[["lat", "lon"]].div(1e7)
    trajectories_df["relative_alt"] = trajectories_df["relative_alt"].div(1e3)

    trajectory_fig = px.line_3d(
        trajectories_df,
        x="lat",
        y="lon",
        z="relative_alt",
        color="system_id",
        title="Multi-Agent Mission Trajectories",
        labels={"lat": "Latitude", "lon": "Longitude", "relative_alt": "Altitude (m)"},
    )
    trajectory_fig.update_traces(line=dict(width=2.0))
    trajectory_fig.write_html(os.path.join(report_dir, "trajectories.html"))

    # Evaluate the radio status
    radio_status_df = log["RADIO_STATUS"][
        ["timestamp", "system_id", "rssi", "noise", "remnoise", "rxerrors"]
    ].astype(int)

    rssi_fig = px.line(
        radio_status_df,
        x="timestamp",
        y="rssi",
        color="system_id",
        title="Received Signal Strength Indicator",
        labels={
            "timestamp": "Timestamp",
            "rssi": "Received Signal Strength Indicator (RSSI)",
        },
    )
    rssi_fig.write_html(os.path.join(report_dir, "rssi.html"))

    noise_fig = px.line(
        radio_status_df,
        x="timestamp",
        y="noise",
        color="system_id",
        title="Local Background Noise Levels",
        labels={
            "timestamp": "Timestamp",
            "noise": "Noise Level",
        },
    )
    noise_fig.write_html(os.path.join(report_dir, "noise.html"))

    remnoise_fig = px.line(
        radio_status_df,
        x="timestamp",
        y="remnoise",
        color="system_id",
        title="Remote Background Noise Levels",
        labels={
            "timestamp": "Timestamp",
            "remnoise": "Noise Level",
        },
    )
    remnoise_fig.write_html(os.path.join(report_dir, "remnoise.html"))

    rxerrors_fig = px.line(
        radio_status_df,
        x="timestamp",
        y="rxerrors",
        color="system_id",
        title="Radio Packet Recieve Errors",
        labels={
            "timestamp": "Timestamp",
            "rxerrors": "Receive Errors (since boot)",
        },
    )
    rxerrors_fig.write_html(os.path.join(report_dir, "rxerrors.html"))

    # Create a basic report
    report = f"""
    ===========================================================================
    pymavswarm Mission Report
    ===========================================================================
    Agent IDs Seen: {agent_ids}
    ---------------------------------------
    Average Network Latency: {ping.mean()}
    Minimum Network Latency: {ping.min()}
    Maximum Network Latency: {ping.max()}
    ---------------------------------------
    Average Radio RSSI: {radio_status_df["rssi"].to_numpy().mean()}
    Minimum Radio RSSI: {radio_status_df["rssi"].to_numpy().min()}
    Maximum Radio RSSI: {radio_status_df["rssi"].to_numpy().max()}
    ---------------------------------------
    Average Background Noise: {radio_status_df["noise"].to_numpy().mean()}
    Minimum Background Noise: {radio_status_df["noise"].to_numpy().min()}
    Maximum Background Noise: {radio_status_df["noise"].to_numpy().max()}
    ---------------------------------------
    Average Remote Background Noise: {radio_status_df["remnoise"].to_numpy().mean()}
    Minimum Remote Background Noise: {radio_status_df["remnoise"].to_numpy().min()}
    Maximum Remote Background Noise: {radio_status_df["remnoise"].to_numpy().max()}
    ---------------------------------------
    Average Radio Packet Receive Errors: {radio_status_df["rxerrors"].to_numpy().mean()}
    Minimum Radio Packet Receive Errors: {radio_status_df["rxerrors"].to_numpy().min()}
    Maximum Radio Packet Receive Errors: {radio_status_df["rxerrors"].to_numpy().max()}
    """

    with open(os.path.join(report_dir, "mission_report.txt"), "w") as file:
        file.write(report)

    return


if __name__ == "__main__":
    main()
