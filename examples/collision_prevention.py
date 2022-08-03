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
        default=0.5,
        help="ground speed that the agents should fly at when flying toward each other",
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
    """Demonstrate how to use collision avoidance."""
    # Parse the script arguments
    args = parse_args()

    # Create a new MavSwarm instance
    mavswarm = MavSwarm()

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
    mavswarm.enable_collision_avoidance(
        3.0, 2.5, 0.1, MavSwarm.COLLISION_RESPONSE_LOITER
    )

    # Set each agent to guided mode before attempting a takeoff sequence
    future = mavswarm.set_mode(
        "GUIDED", agent_ids=target_agents, retry=True, verify_state=True
    )
    future.add_done_callback(print_message_response_cb)

    while not future.done():
        pass

    responses = future.result()

    # Exit if all agents didn't successfully switch into GUIDED mode
    if isinstance(responses, list):
        for response in responses:
            if not response.result:
                print(
                    "Failed to set the flight mode of agent "
                    f"{response.target_agent_id} to GUIDED prior to the takeoff "
                    "sequence. Exiting."
                )
                return
    else:
        if not responses.result:
            print(
                "Failed to set the flight mode of agent {responses.target_agent_id} to "
                "GUIDED prior to the takeoff sequence. Exiting."
            )
            return

    # Perform takeoff with all agents in the swarm; retry on message failure
    responses = mavswarm.takeoff_sequence(
        args.takeoff_alt, agent_ids=target_agents, verify_state=True, retry=True
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

    # Wait for the user to indicate that the agents should fly to their waypoints
    input("Press the 'enter' key to command the agents to fly to their waypoints")

    # Command the agent to the target location
    # We don't need to specify the target agents because these are captured from the
    # config file
    future = mavswarm.goto(config_file=args.config, retry=True)
    future.add_done_callback(print_message_response_cb)

    while not future.done():
        pass

    # Set the groundspeed; make sure that this is set low when running the examples on
    # real hardware
    future = mavswarm.set_groundspeed(
        args.ground_speed, agent_ids=target_agents, retry=True
    )
    future.add_done_callback(print_message_response_cb)

    while not future.done():
        pass

    # Wait for user input
    input("Press the 'enter' key to command the agents to land\n")

    # Attempt to land the agents
    future = mavswarm.set_mode(
        "LAND", agent_ids=target_agents, retry=True, verify_state=True
    )
    future.add_done_callback(print_message_response_cb)

    # Wait for the land command to complete
    while not future.done():
        pass

    # Disable collision avoidance
    mavswarm.disable_collision_avoidance()

    # Disconnect from the swarm
    mavswarm.disconnect()

    return


if __name__ == "__main__":
    main()
