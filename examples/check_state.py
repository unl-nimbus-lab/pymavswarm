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
        "--duration", type=float, default=5.0, help="duration to print out state [s]"
    )
    parser.add_argument(
        "--log_file",
        type=str,
        default=None,
        help="log file to save the resulting tlog to (including its path)",
    )
    return parser.parse_args()


def main() -> None:
    """Demonstrate how to get the state of agents."""
    # Parse script arguments
    args = parse_args()

    # Create a new MavSwarm instance
    mavswarm = MavSwarm(log_file=args.log_file)

    # Attempt to create a new MAVLink connection
    if not mavswarm.connect(args.port, args.baud):
        return

    # Wait for the swarm to auto-register new agents
    while not list(filter(lambda agent_id: agent_id[1] == 1, mavswarm.agent_ids)):
        print("Waiting for the system to recognize agents in the network...")
        time.sleep(0.5)

    print(f"Logging the attitude of swarm agents for {args.duration} seconds")

    start_t = time.time()

    while time.time() - start_t < args.duration:
        for agent_id in list(
            filter(lambda agent_id: agent_id[1] == 1, mavswarm.agent_ids)
        ):
            agent = mavswarm.get_agent_by_id(agent_id)

            if agent is not None:
                print(f"The current attitude of {agent} is: {agent.attitude}")

            time.sleep(0.5)

    # Disconnect from the swarm
    mavswarm.disconnect()

    return


if __name__ == "__main__":
    main()
