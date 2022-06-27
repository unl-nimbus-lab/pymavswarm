import time
from argparse import ArgumentParser

from pymavswarm import MavSwarm
from pymavswarm.Agent import Agent
from pymavswarm.msg import AgentMsg, MsgMap, SystemCommandMsg


def main():
    """
    Demonstrate how to check the state of an agent.

    In this case, check flight mode, arm state, and location.

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

    # Get the first agent in the swarm
    # system_id = mavswarm.agents[0].sys_id
    # component_id = mavswarm.agents[0].comp_id
    agentTest = mavswarm.agents[0]

    # Print these values out
    print(
        f"Flight mode: {agentTest.flight_mode} \n"
        f"Arm state: {agentTest.armed} \n"
        f"Location (latitude): {agentTest.location.latitude} \n"
        f"System ID: {agentTest.sys_id} \ncomp_id: {agentTest.comp_id} \n"
    )

    # Delay for a bit so that we can see the agent post mode-change
    time.sleep(5)

    # Disconnect
    mavswarm.disconnect()

    return


if __name__ == "__main__":
    main()
