import time
from argparse import ArgumentParser

from pymavswarm import MavSwarm
from pymavswarm.msg import MsgMap, SystemCommandMsg


def main():
    """
    Demonstrate how to arm and disarm an agent.

    Ensure that all propellers have been disconnected prior to running this example!
    """

    # Add the port and baudrate as arguments
    parser = ArgumentParser()

    parser.add_argument(
        "port", type=str, help="port to establish a MAVLink connection over"
    )
    parser.add_argument("baud", type=int, help="baudrate to establish a connection at")

    args = parser.parse_args()

    # Create a new MavSwarm instance
    mavswarm = MavSwarm()

    # Attempt to connect
    if mavswarm.connect(args.port, args.baud):
        print("Successfully established a MAVLink connection!")
    else:
        print(
            f"Failed to establish a MAVLink connection at {args.port} with "
            f"baudrate: {args.baud}"
        )

    # Wait for the swarm to recognize agents
    while not mavswarm.agents:
        time.sleep(0.5)

    # Arm the first agent in the swarm
    system_id = mavswarm.agents[0].sys_id
    component_id = mavswarm.agents[0].comp_id

    # Arm the agent
    mavswarm.send_msg(
        [
            SystemCommandMsg(
                MsgMap().system_commands.arm, system_id, component_id, retry=False
            )
        ]
    )

    # Delay for a bit so that we can see the agent arm
    time.sleep(5)

    # Disarm the agent
    mavswarm.send_msg(
        [
            SystemCommandMsg(
                MsgMap().system_commands.disarm,
                system_id,
                component_id,
                False,
            )
        ]
    )

    # Disconnect
    mavswarm.disconnect()

    return


if __name__ == "__main__":
    main()
