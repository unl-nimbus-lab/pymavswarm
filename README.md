# pymavswarm

## What is pymavswarm?

`pymavswarm` is a Python library implemented to enable interaction with drone
swarms using the MAVLink protocol. This library supports reading MAVLink
messages sent from multiple agents in a swarm and sending MAVLink messages to
agents within the swarm. Such functionality ultimately enables development of
new swarm applications such as ground control stations.

## Main features

Here are some of the main features of `pymavswarm`:

- Easily send MAVLink commands to multiple drones, simultaneously
- Monitor the state of swarm drone agents
- Implement custom commands and swarm algorithms
- Develop applications for on-board companion computers
- Implement custom swarm ground control stations
- Log incoming MAVLink messages for future evaluation and debugging

## Dependencies

`pymavswarm` depends on Python versions 3.8 or greater. Ensure that this
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

## Quick start

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

## Getting help

If you have questions regarding `pymavswarm` usage or contribution please ask a
question on our [Discussions](https://github.com/unl-nimbus-lab/pymavswarm/discussions)
board!

## Contributing

All contributions and ideas are welcome! Detailed guidelines regarding how to
contribute can be found in the `.github/CONTRIBUTING.md` document.

## License

`pymavswarm` is released under the GNU General Public License v3 or later
