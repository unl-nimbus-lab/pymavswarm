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

    # Ground control station identifier
    gcs_system_id = 255
    gcs_component_id = 0

    # Attempt to connect
    if mavswarm.connect(
        args.port,
        args.baud,
        source_system=gcs_system_id,
        source_component=gcs_component_id,
    ):
        print("Successfully established a MAVLink connection!")
    else:
        print(
            f"Failed to establish a MAVLink connection at {args.port} with "
            f"baudrate: {args.baud}"
        )

    # Wait for the swarm to recognize agents
    while not mavswarm.agents:
        print("Waiting for the system to recognize agents in the network...")
        time.sleep(0.5)

    # Get all agents excluding the ground control station and non-flight controller
    # devices (devices that don't have a component ID of 1)
    agents = [
        agent
        for agent in mavswarm.agents
        if agent.sys_id != gcs_system_id
        and agent.comp_id != gcs_component_id
        and agent.comp_id == 1
    ]

    # Arm each agent in the network
    for agent in agents:
        mavswarm.send_msg(
            [
                SystemCommandMsg(
                    MsgMap().system_commands.arm,
                    agent.sys_id,
                    agent.comp_id,
                    retry=False,
                )
            ]
        )

    # Delay for a bit so that we can see the agent(s) arm
    time.sleep(5)

    # Disarm each agent
    for agent in agents:
        mavswarm.send_msg(
            [
                SystemCommandMsg(
                    MsgMap().system_commands.disarm,
                    agent.sys_id,
                    agent.comp_id,
                    retry=False,
                )
            ]
        )

    # Disconnect
    mavswarm.disconnect()

    return


if __name__ == "__main__":
    main()
