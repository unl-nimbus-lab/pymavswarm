import time
from argparse import ArgumentParser

from pymavswarm import MavSwarm
from pymavswarm.msg import MsgMap, SystemCommandMsg    


def main():
    """
    Example demonstrating how to use pymavswarm to change the flight mode of each agent in the swarm from a companion computer.

    Ensure that all propellers have been disconnected prior to running this example!
    """

    # Add the port and baudrate as arguments
    parser = ArgumentParser()

    parser.add_argument(
        "port", type=str, help="port to establish a MAVLink connection over"
    )
    parser.add_argument("baud", type=int, help="baudrate to establish a connection at")
    
    parser.add_argument("companion_computer_system_id", type=int, help="system ID of the companion computer")
    
    parser.add_argument("companion_computer_component_id", type=int, help="component ID of the companion computer")

    args = parser.parse_args()

    # Create a new MavSwarm instance
    mavswarm = MavSwarm()

# send messages to all agents except for the companion computer
# read ardupilot documention site for more: https://ardupilot.org/dev/docs/companion-computers.html
# Ground control station identifier
    companion_computer_system_id = args.companion_computer_system_id
    companion_computer_component_id = args.companion_computer_component_id

    # Attempt to connect
    if mavswarm.connect(
        args.port,
        args.baud,
        source_system=companion_computer_system_id,
        source_component=companion_computer_component_id,
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
    agents = filter(lambda agent: not (agent.sys_id == companion_computer_system_id and agent.comp_id == companion_computer_component_id) and agent.comp_id == 1, mavswarm.agents)
    
    # Print the number of agents in the swarm:
    print(f"Number of agents in the swarm: {len(agents)} \n")
    
    # for all the agents in the swarm, change their flight mode to loiter
    for agent in agents:
        print(f"Pre-change agent flight mode: {agent.flight_mode}")
        mavswarm.send_msg(
            [
                SystemCommandMsg(
                    MsgMap().flight_modes.loiter, agent.sys_id, agent.comp_id, retry=False
                )
            ]
        )
        print(f"Post-change agent flight mode: {agent.flight_mode}")
        
    
    # Delay for a bit so that we can see the agent post mode-change
    time.sleep(5)
    
    #Disconnect 
    mavswarm.disconnect
    
    return

if __name__ == "__main__":
    main()
