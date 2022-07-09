# pymavswarm

`pymavswarm` is a Python library implemented to enable interaction with drone
swarms using the MAVLink protocol. This library supports reading MAVLink
messages sent from multiple agents in a swarm and sending MAVLink messages to
agents within the swarm. Such functionality ultimately enables development of
new swarm applications such as ground control stations.

## Dependencies

`pymavswarm` depends on Python versions 3.9 or greater. Ensure that this
dependency is met prior to installation.

## Installation

`pymavswarm` must currently be installed manually. To do so, refer to the steps
below:

1. Clone this repository
2. Navigate to the `pymavswarm/` repository directory

```bash
cd path/to/pymavswarm/
```

3. Install the `pymavswarm` Python package

```bash
pip3 install .
```

## Quick Start

`pymavswarm` has been implemented to enable easy interfacing with drone
swarms. Refer to the following code snippet for a simple example to get started
with the library. Additional examples may be found in the `pymavswarm/examples`.
directory.

```python
import time

from pymavswarm import MavSwarm

# Create a new pymavswarm interface
mavswarm = MavSwarm()

# Establish a connection with a USB telemetry device
mavswarm.connect('/dev/ttyUSB0', 115200, 255, 0)

# Wait for the swarm to be populated
while not mavswarm.agents:
    pass

# Arm each agent in the swarm
mavswarm.arm()

# Briefly delay to allow all agents to arm
time.sleep(5.0)

# Disarm each agent in the swarm
mavswarm.disarm()

# Close the pymavswarm connection
mavswarm.disconnect()
```

## License

`pymavswarm` is released under the GNU General Public License v3 or later
