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

import logging
import time
from argparse import ArgumentParser

from pymavswarm import MavSwarm
from pymavswarm.utils import init_logger


def main() -> None:
    """Demonstrate how to perform a takeoff sequence with swarm agents."""
    # Get the desired port and baudrate of the source radio as arguments
    parser = ArgumentParser()
    parser.add_argument(
        "port", type=str, help="port to establish a MAVLink connection over"
    )
    parser.add_argument("baud", type=int, help="baudrate to establish a connection at")
    parser.add_argument("altitude", type=float, help="altitude to takeoff to")
    args = parser.parse_args()

    # Create a new MavSwarm instance
    mavswarm = MavSwarm(log_level=logging.DEBUG)

    # Attempt to create a new MAVLink connection
    if not mavswarm.connect(args.port, args.baud):
        return

    logger = init_logger("takeoff_sequence_example", logging.DEBUG)

    # In our configuration there are some agents that we want to blacklist and avoid
    # interacting with. Add or remove agents from here as need-be.
    blacklisted_agent_ids = [(1, 0)]

    # Wait for the swarm to auto-register new agents
    while not list(
        filter(
            lambda agent_id: not (agent_id in blacklisted_agent_ids),
            mavswarm.agent_ids,
        )
    ):
        logger.info("Waiting for the system to recognize agents in the network...")
        time.sleep(0.5)

    # Get the list of target agent ids
    target_agent_ids = list(
        filter(
            lambda agent_id: not (agent_id in blacklisted_agent_ids),
            mavswarm.agent_ids,
        )
    )

    # Perform takeoff with all agents in the swarm; retry on message failure
    responses = mavswarm.takeoff_sequence(
        args.altitude, agent_ids=target_agent_ids, verify_state=True, retry=True
    )

    if isinstance(responses, list):
        for response in responses:
            logger.info(
                f"Result of {response.message_type} message sent to "
                f"({response.target_system}, {response.target_component}): "
                f"{response.code}"
            )
    else:
        logger.info(
            f"Result of {responses.message_type} message sent to "
            f"({responses.target_system}, {responses.target_component}): "
            f"{responses.code}"
        )

    # Delay to let the agents takeoff
    time.sleep(10)

    # Attempt to land the agents
    future = mavswarm.set_mode(
        "LAND", agent_ids=target_agent_ids, retry=True, verify_state=True
    )

    # Wait for the land command to complete
    if future is not None:
        while not future.done():
            pass

        landing_responses = future.result()

        for response in landing_responses:
            logger.info(
                f"Result of {response.message_type} message sent to "
                f"({response.target_system}, {response.target_component}): "
                f"{response.code}"
            )

    # Disconnect from the swarm
    mavswarm.disconnect()

    return


if __name__ == "__main__":
    main()