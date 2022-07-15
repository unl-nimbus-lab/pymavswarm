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
    parser.add_argument("id", type=str, help="parameter ID to read/write")
    parser.add_argument(
        "value", type=float, help="value that the parameter should be set to"
    )
    parser.add_argument(
        "type", type=int, help="type of parameter value to set", choices=range(1, 11)
    )
    return parser.parse_args()


def main() -> None:
    """Demonstrate how to read and write parameters on agents."""
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

    # Get the value of the parameter before it is set
    future = mavswarm.read_parameter(args.id, retry=True)

    # Wait for the read parameter value to complete
    while not future.done():
        pass

    responses = future.result()

    for response in responses:
        print(
            f"Result of {response.message_type} message sent to "
            f"({response.target_agent_id}): {response.code}"
        )

        if response.result:
            agent = mavswarm.get_agent_by_id(response.target_agent_id)
            if agent is not None:
                print(
                    f"Initial value of parameter {args.id} on agent ("
                    f"{response.target_agent_id}): "
                    f"{agent.last_params_read.parameters[-1]}"
                )

    # Set the parameter to the desired value
    future = mavswarm.set_parameter(
        args.id,
        args.value,
        args.type,
        retry=True,
    )

    # Wait for the operation to finish
    while not future.done():
        pass

    # Get the value of the parameter after it has been set
    future = mavswarm.read_parameter(args.id, retry=True)

    while not future.done():
        pass

    responses = future.result()

    for response in responses:
        print(
            f"Result of {response.message_type} message sent to "
            f"({response.target_agent_id}): {response.code}"
        )

        if response.result:
            agent = mavswarm.get_agent_by_id(response.target_agent_id)
            if agent is not None:
                print(
                    f"Resulting value of parameter {args.id} on agent ("
                    f"{response.target_agent_id}): "
                    f"{agent.last_params_read.parameters[-1]}"
                )

    # Disconnect from the swarm
    mavswarm.disconnect()

    return


if __name__ == "__main__":
    main()
