from argparse import ArgumentParser
from typing import Any

from pymavswarm import MavSwarm, mavswarm
from pymavswarm.mission import Command, Mission, Stage


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


def main() -> None:
    # Parse the script arguments
    args = parse_args()

    # Create a new MavSwarm instance
    mavswarm = MavSwarm()

    # Attempt to create a new MAVLink connection
    if not mavswarm.connect(args.port, args.baud):
        return

    mission = Mission(
        [
            Stage(
                [
                    Command(),
                ]
            ),
            Stage(
                [
                    Command(),
                ]
            ),
        ]
    )

    while not all(
        agent_id in mavswarm.agent_ids for agent_id in mission.target_agent_ids
    ):
        pass


if __name__ == "__main__":
    main()
