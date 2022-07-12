# pymavswarm Examples

Several examples have been implemented to demonstrate how to use and extend the
`pymavswarm` library. The implemented examples include:

- `arming.py`: demonstrates how to arm and disarm swarm agents
- `check_state.py`: demonstrates how to check the state of swarm agents
- `companion_computer.py`: demonstrates how to send commands from a companion
computer
- `custom_observer.py`: demonstrates how to implement a callback for an agent
state change
- `goto.py`: demonstrates how to command an agent to fly to a specified location
- `set_flight_mode.py`: demonstrates how to set the agents' flight mode
- `set_home_position.py`: demonstrates how to set the swarm agents' home
position
- `set_parameter.py`: demonstrates how to set an agent's parameter
- `subclass_mavswarm.py`: demonstrates how to extend the functionality of
`MavSwarm`
- `takeoff_sequence.py`: demonstrates how to execute a full takeoff sequence

## Running the examples

Prior to running the examples, ensure that there is an active connection between
a ground control station and the swarm agents. When these examples were first
developed, the ground control station and the swarm agents utilized RFD900
radios configured to use the multi-point radio setting. In the case of the
`companion_computer.py` example, ensure that the script has been uploaded to the
companion computer for execution.

Once the network has been configured, the examples may be run using the
following command:

```bash
python3 <example_script.py> ...
```

where `<example_script.py>` is replaced with the name of the example script to
run and `...` is replaced with the example's required arguments. To see the
required arguments, run the following command:

```bash
python3 <example_script.py> -h
```
