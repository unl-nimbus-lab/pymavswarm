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
from argparse import ArgumentParser
from concurrent.futures import Future
from typing import Any

from pymavswarm import MavSwarm


def parse_args() -> Any:
    """
    Parse the script arguments.

    :return: argument namespace
    :rtype: Any
    """
    parser = ArgumentParser()
    parser.add_argument(
        "port", type=str, help="port to establish a MAVLink connection over"
    )
    parser.add_argument("baud", type=int, help="baudrate to establish a connection at")
    parser.add_argument("config", type=str, help="Pre-planned positions to load")
    parser.add_argument(
        "--takeoff_alt", type=float, default=3.0, help="altitude to takeoff to [m]"
    )
    parser.add_argument(
        "--ground_speed",
        type=float,
        default=3.0,
        help="ground speed that the agents should fly at when flying to the target "
        "location",
    )
    return parser.parse_args()


def print_message_response_cb(future: Future) -> None:
    """
    Print the result of the future.

    :param future: message execution future
    :type future: Future
    """
    responses = future.result()

    if isinstance(responses, list):
        for response in responses:
            print(
                f"Result of {response.message_type} message sent to "
                f"({response.target_agent_id}): {response.code}"
            )
    else:
        print(
            f"Result of {responses.message_type} message sent to "
            f"({responses.target_agent_id}): {responses.code}"
        )

    return


def main() -> None:
    """Demonstrate collision avoidance without flying."""
    # Parse the script arguments
    args = parse_args()

    # Create a new MavSwarm instance
    mavswarm = MavSwarm(log_to_file=True)

    # Attempt to create a new MAVLink connection
    if not mavswarm.connect(args.port, args.baud):
        return

    # Get the target agents specified in the config file
    target_agents = list(mavswarm.parse_yaml_mission(args.config)[0].keys())

    # Wait for the swarm to register all target agents
    while not all(agent_id in mavswarm.agent_ids for agent_id in target_agents):
        print("Waiting for the system to recognize all target agents...")
        time.sleep(0.5)

    # Enable collision avoidance
    mavswarm.enable_collision_avoidance(1, 1.5, 0.1, MavSwarm.COLLISION_RESPONSE_NONE)

    input("Hit 'enter' to exit the example")

    mavswarm.disable_collision_avoidance()

    # Disconnect from the swarm
    mavswarm.disconnect()

    return


if __name__ == "__main__":
    main()
