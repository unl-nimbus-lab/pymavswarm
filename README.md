# pymavswarm

[![PyPI Version](https://img.shields.io/pypi/v/pymavswarm?color=gr)](https://pypi.org/project/pymavswarm/)
[![Contribution Frequency](https://img.shields.io/github/commit-activity/m/unl-nimbus-lab/pymavswarm)](https://github.com/unl-nimbus-lab/pymavswarm/commits/main)
[![Project License](https://img.shields.io/github/license/unl-nimbus-lab/pymavswarm)](https://github.com/unl-nimbus-lab/pymavswarm/blob/main/LICENSE)
[![Open Issues](https://img.shields.io/github/issues/unl-nimbus-lab/pymavswarm?color=purple)](https://github.com/unl-nimbus-lab/pymavswarm/issues)

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
- Construct pre-planned missions
- Multi-agent collision avoidance using reachability analysis

## Installation

`pymavswarm` may be installed from
[PyPI](https://pypi.org/project/pymavswarm/#description) by running

```bash
pip3 install pymavswarm
```

To build `pymavswarm` from source, first ensure that at least Python 3.10 is
installed. Once this dependency has been met, refer to the steps below:

1. Clone the project [repository](https://github.com/unl-nimbus-lab/pymavswarm)
2. Navigate to the `pymavswarm/` base directory

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
with the library. Additional examples may be found in the project
[examples](https://github.com/unl-nimbus-lab/pymavswarm/tree/main/examples).

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
contribute can be found in the [contribution guidelines](https://github.com/unl-nimbus-lab/pymavswarm/blob/main/.github/CONTRIBUTING.md).

## License

`pymavswarm` is released under the GNU General Public License v3 or later
