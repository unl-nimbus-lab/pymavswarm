from pymavswarm import MavSwarm, SystemCommandMsg, MsgMap

# Create a new pymavswarm interface
mavswarm = MavSwarm()

# Establish a connection with a USB telemetry device
mavswarm.connect('/dev/tty.usbserial-AI06ITM', 57600, 255, 0)

# Create a list of messages to send to the respective agents
msgs = []

# Send an arming message to Agent (2, 1)
msgs.append(SystemCommandMsg(MsgMap().system_commands.arm, 2, 1, True))

# Send the desired messages and require that the messages be acknowledged
mavswarm.send_msg(msgs)

# Read the current state of the swarm agents
for agent in mavswarm.get_agents():
    print(f'Latitude: {agent.location.latitude} Longitude: {agent.location.longitude}')

# Close the pymavswarm connection
mavswarm.disconnect()