import time
from argparse import ArgumentParser

from pymavswarm import MavSwarm
from pymavswarm.msg import AgentMsg, MsgMap, SystemCommandMsg

def main():
    """
    Demonstrate how to change the flight mode of an agent.

    Ensure that all propellers have been disconnected prior to running this example!
    """

    # Add the port and baudrate as arguments
    parser = ArgumentParser()

    parser.add_argument(
        "port", type=str, help="port to establish a MAVLink connection over"
    )
    parser.add_argument("baud", type=int, help="baudrate to establish a connection at")
    
    parser.add_argument(
        "flight_mode", 
        type=str, 
        choices=[
            MsgMap().flight_modes.stabilize, 
            MsgMap().flight_modes.acro, 
            MsgMap().flight_modes.alt_hold, 
            MsgMap().flight_modes.auto, 
            MsgMap().flight_modes.loiter, 
            MsgMap().flight_modes.rtl, 
            MsgMap().flight_modes.land,  
            MsgMap().flight_modes.throw, 
            MsgMap().flight_modes.systemid, 
            MsgMap().flight_modes.guided
        ], 
        help="the flight mode that all of the agents in the swarm will be set to"
    )

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
          
    # Get system and component id of the first agent in the swarm
    system_id = mavswarm.agents[0].sys_id
    component_id = mavswarm.agents[0].comp_id
    
    # Change the flight mode of this agent (in this case, to loiter)
    mavswarm.send_msg(
        [
            SystemCommandMsg(
                args.flight_mode, system_id, component_id, retry=False
            )
        ]
    )
    
    # Delay for a bit so that we can see the agent post mode-change
    time.sleep(5)
    
    print(f"Current agent flight mode: {mavswarm.agents[0].flight_mode}")
    
    # Disconnect
    mavswarm.disconnect()

    return

if __name__ == "__main__":
    main()
