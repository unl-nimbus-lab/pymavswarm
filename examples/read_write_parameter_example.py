import time
from argparse import ArgumentParser

from pymavswarm import MavSwarm
from pymavswarm.Agent import Agent
from pymavswarm.msg import AgentMsg, MsgMap, SystemCommandMsg
from pymavswarm.param.Parameter import Parameter

def main():
    """
    Demonstrate how to read/write a parameter of an agent.
    (In this case, we can read/write the sysid of the agent)

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
    system_id = mavswarm.agents[0].sys_id
    component_id = mavswarm.agents[0].comp_id
    agent_test = mavswarm.agents[0]
    
    # read the sysid of the agent
    param = Parameter(system_id, component_id, "SYSID_THISMAV", False)
    param_list = [param]
    old_sys_id_of_test = mavswarm.read_param(param_list)
    # Try making the new sys_id a float
    new_sysid = 4
    new_sysid_param = Parameter(system_id, component_id, "SYSID_THISMAV", False, new_sysid)
    param_list2 = [new_sysid_param]
    mavswarm.set_param(param_list2)
    
    # Delay for a bit so that we can see the agent post mode-change
    time.sleep(5)
    
    # Disconnect
    mavswarm.disconnect()

    return

if __name__ == "__main__":
    main()