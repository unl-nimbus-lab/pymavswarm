# pymavswarm

## Introduction
`pymavswarm` is a Python library implemented to enable interaction with robotic swarms using the MAVLink protocol. This library supports reading MAVLink messages sent from multiple agents in a swarm and sending MAVLink messages to agents within the swarm. Such functionality ultimately enables development of new swarm applications such as ground control stations.


## Installation
`pymavswarm` must currently be installed manually. To do so, refer to the steps below:
1. Navigate to the `pymavswarm/` repository directory
```bash
cd path/to/pymavswarm/
```
2. Install the `pymavswarm` Python package
```bash
pip3 install .
```

## Getting Started
`pymavswarm` has been implemented to enable easy interfacing with robotic swarms. Refer to the following code snippet for a simple example to get started with the library. For more comprehensive documentation and examples, checkout the project Wiki.

```python
from pymavswarm import MavSwarm, OutgoingMsg, MsgMap

# Create a new pymavswarm interface
mavswarm = MavSwarm()

# Establish a connection with a USB telemetry device
mavswarm.connect('/dev/ttyUSB0', 115200, 255, 0)

# Create a list of messages to send to the respective agents
msgs = []

# Send an arming message to Agent (2, 1)
msgs.append(OutgoingMsg(MsgMap().system_commands.arm, 2, 1))

# Send the desired messages and check for message acknowledgement
mavswarm.send_msg(msgs, ack=True)

# Read the current state of the swarm agents
for agent in mavswarm.get_agents():
    print(f'Latitude: {agent.location.latitude} Longitude: {agent.location.longitude}')

# Close the pymavswarm connection
mavswarm.disconnect()
```

## License
pymavswarm is released under the GNU General Public License v3 or later
