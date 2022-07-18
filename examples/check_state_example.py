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
    agents = filter(lambda agent: not (agent.sys_id == gcs_system_id and agent.comp_id == gcs_component_id) and agent.comp_id == 1, mavswarm.agents)

    # Print these values out for each agent in the network
    for agent in agents:
        print(
            f"Flight mode: {agent.flight_mode} \n"
            f"Arm state: {agent.armed} \n"
            f"Location (latitude): {agent.location.latitude} \n"
            f"Location (longitude): {agent.location.longitude} \n"
            f"Location (altitude): {agent.location.altitude} \n"
            f"System ID: {agent.sys_id} \n"
            f"Component ID: {agent.comp_id} \n"
            "---------------------------------------- \n"
        )
    

    # Delay for a bit so that we can see the agent post mode-change
    time.sleep(5)

    # Disconnect
    mavswarm.disconnect()

    return


if __name__ == "__main__":
    main()
