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
from concurrent.futures import Future
from typing import Any, Dict, List, Optional, Tuple, Union

import monotonic

from pymavswarm import Agent, MavSwarm
from pymavswarm.utils import init_logger


class CustomMavSwarm(MavSwarm):
    """Demonstrate how to subclass the MavSwarm class."""

    def __init__(self, log_level: int = logging.INFO) -> None:
        """
        Create a new custom MavSwarm class.

        :param log_level: logging level, defaults to logging.INFO
        :type log_level: int, optional
        """
        super().__init__(log_level)

        self.add_custom_message_handler("HEARTBEAT", self.custom_handler)

        return

    def fun_command(
        self,
        agent_ids: Optional[Union[Tuple[int, int], List[Tuple[int, int]]]] = None,
        retry: bool = False,
        message_timeout: float = 2.5,
        ack_timeout: float = 0.5,
    ) -> Future:
        """
        Send a fun command to the specified agents.

        :param agent_ids: optional list of target agent IDs, defaults to None
        :type agent_ids: Optional[Union[Tuple[int, int], List[Tuple[int, int]]]],
            optional
        :param retry: retry sending the fun command to an agent on failure, defaults to
            False
        :type retry: bool, optional
        :param message_timeout: maximum amount of time allowed to try sending a fun
            command to an agent before a timeout occurs, defaults to 2.5 [s]
        :type message_timeout: float, optional
        :param ack_timeout: maximum amount of time allowed per attempt to verify
            acknowledgement of an arming attempt, defaults to 0.5 [s]
        :type ack_timeout: float, optional
        :return: future response
        :rtype: Future
        """
        self._logger.info(f"Attempting to send my fun command to agents {agent_ids}")

        return self.send_debug_message(
            "party-time",
            1,
            agent_ids=agent_ids,
            retry=retry,
            message_timeout=message_timeout,
            ack_timeout=ack_timeout,
        )

    def custom_handler(
        self, message: Any, agents: Dict[Tuple[int, int], Agent]
    ) -> None:
        """
        Create a custom message handler.

        :param message: message to handle
        :type message: Any
        :param agents: swarm agents
        :type agents: Dict[Tuple[int, int], Agent]
        """
        sys_id = message.get_srcSystem()
        comp_id = message.get_srcComponent()

        agent_id = (sys_id, comp_id)

        self._logger.info(
            f"Received a message for agent {agent_id} in my custom handler!"
        )

        # Demonstrate how to modify the properties of the swarm agents

        # Notice that we don't need to use self to access the agents property here
        # this is because we take advantage of a Python property in which the
        # interpreter doesn't interpret mutable objects until its last function call.
        # You can learn more about this at the following link:
        # https://learnandlearn.com/python-programming/python-how-to/python-function-arguments-mutable-and-immutable
        agents[agent_id].last_heartbeat.value = monotonic.monotonic()

        return


def main() -> None:
    """Demonstrate how to use the custom subclass."""
    # Get the desired port and baudrate of the source radio as arguments
    parser = ArgumentParser()
    parser.add_argument(
        "port", type=str, help="port to establish a MAVLink connection over"
    )
    parser.add_argument("baud", type=int, help="baudrate to establish a connection at")
    args = parser.parse_args()

    # Create a new CustomMavSwarm instance
    mavswarm = CustomMavSwarm(log_level=logging.DEBUG)

    # Attempt to create a new MAVLink connection
    if not mavswarm.connect(args.port, args.baud):
        return

    logger = init_logger("custom_mavswarm_example", logging.DEBUG)

    # In our configuration there are some agents that we want to blacklist and avoid
    # interacting with. Add or remove agents from here as need-be.
    blacklisted_agent_ids = [(1, 0)]

    # Wait for the swarm to auto-register new agents
    # We should also see our custom handler called here
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

    # Send our fun new command
    mavswarm.fun_command(target_agent_ids)

    # Disconnect from the swarm
    mavswarm.disconnect()

    return


if __name__ == "__main__":
    main()
