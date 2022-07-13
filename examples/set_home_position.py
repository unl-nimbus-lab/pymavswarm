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
    return parser.parse_args()


# Define a callback to attach to a future
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
        for response in responses:
            print(
                f"Result of {response.message_type} message sent to "
                f"({response.target_agent_id}): {response.code}"
            )

    return


def main() -> None:
    """Demonstrate how to read and set the home position of agents."""
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

    future = mavswarm.get_home_position(retry=True)
    future.add_done_callback(print_message_response_cb)

    # Print out the current home position of the agents
    for agent_id in list(filter(lambda agent_id: agent_id[1] == 1, mavswarm.agent_ids)):
        agent = mavswarm.get_agent_by_id(agent_id)

        if agent is not None:
            print(
                f"The home position of agent ({agent.system_id}, {agent.component_id}) "
                f"is: {agent.home_position}"
            )

    while not future.done():
        pass

    future = mavswarm.set_home_position(retry=True, verify_state=True)
    future.add_done_callback(print_message_response_cb)

    while not future.done():
        pass

    future = mavswarm.get_home_position(retry=True)
    future.add_done_callback(print_message_response_cb)

    while not future.done():
        pass

    # Print out the current home position of the agents
    for agent_id in list(filter(lambda agent_id: agent_id[1] == 1, mavswarm.agent_ids)):
        agent = mavswarm.get_agent_by_id(agent_id)

        if agent is not None:
            print(
                f"The home position of agent ({agent.system_id}, {agent.component_id}) "
                f"is: {agent.home_position}"
            )

    # Disconnect from the swarm
    mavswarm.disconnect()

    return


if __name__ == "__main__":
    main()
