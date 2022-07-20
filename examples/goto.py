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
    parser.add_argument(
        "--takeoff_alt", type=float, default=3.0, help="altitude to takeoff to [m]"
    )
    return parser.parse_args()


def main() -> None:
    """Demonstrate how to command agents to go to a location."""
    # Parse the script arguments
    args = parse_args()

    # Create a new MavSwarm instance
    mavswarm = MavSwarm()

    # Attempt to create a new MAVLink connection
    if not mavswarm.connect(args.port, args.baud):
        return

    # Wait for the swarm to auto-register new agents
    while not list(filter(lambda agent_id: agent_id[1] == 1, mavswarm.agent_ids)):
        print("Waiting for the system to recognize agents in the network...")
        time.sleep(0.5)

    # Set each agent to guided mode before attempting a takeoff sequence
    future = mavswarm.set_mode("GUIDED", retry=True)

    while not future.done():
        pass

    responses = future.result()

    for response in responses:
        if not response.result:
            print(
                "Failed to set the flight mode of all agents to GUIDED prior to the "
                "takeoff sequence. Exiting."
            )
            return

    # Perform takeoff with all agents in the swarm; retry on message failure
    responses = mavswarm.takeoff_sequence(
        args.takeoff_alt, verify_state=True, retry=True
    )

    if isinstance(responses, list):
        for response in responses:
            print(
                f"Result of {response.message_type} message sent to "
                f"({response.target_agent_id}): {response.code}"
            )
    else:
        print(
            f"Result of {responses.message_type} message sent to "
            f"({response.target_agent_id}): {responses.code}"
        )

    target_locations: dict[tuple[int, int], tuple[float, float, float]] = {}

    # Get the target positions
    for agent_id in mavswarm.agent_ids:
        lat = float(input(f"Enter the target latitude for agent {agent_id}: "))
        lon = float(input(f"Enter the target longitude for agent {agent_id}: "))
        alt = float(input(f"Enter the target altitude for agent {agent_id}: "))

        target_locations[agent_id] = (lat, lon, alt)

    print(f"Target locations specified: {target_locations}")

    # Wait for the user to indicate that the agents should fly to their waypoints
    input("Press any key to command the agents to fly to their waypoints")

    for agent_id in target_locations.keys():
        future = mavswarm.goto(
            target_locations[agent_id][0],
            target_locations[agent_id][1],
            target_locations[agent_id][2],
            agent_ids=agent_id,
            retry=True,
        )

        while not future.done():
            pass

        response = future.result()

        print(
            f"Result of {response.message_type} message sent to "
            f"({response.target_agent_id}): {response.code}"
        )

        if not response.result:
            print(
                f"Failed to command agent {agent_id} to location "
                f"{target_locations[agent_id]}"
            )

    # Wait for user input
    input("Press any key to command the agents to land")

    # Attempt to land the agents
    future = mavswarm.set_mode("LAND", retry=True, verify_state=True)

    # Wait for the land command to complete
    while not future.done():
        pass

    landing_responses = future.result()

    for response in landing_responses:
        print(
            f"Result of {response.message_type} message sent to "
            f"({response.target_agent_id}): {response.code}"
        )

    # Disconnect from the swarm
    mavswarm.disconnect()

    return


if __name__ == "__main__":
    main()
