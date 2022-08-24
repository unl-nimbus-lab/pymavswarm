# pymavswarm Examples

Several examples have been implemented to demonstrate how to use and extend the
`pymavswarm` library. The implemented examples include:

- `arming.py`: demonstrates how to arm and disarm swarm agents.
- `check_state.py`: demonstrates how to check the state of swarm agents; also
  demonstrates how to log data to a log file.
- `collision_prevention_no_flight.py`: demonstrates how to accomplish collision
  avoidance without requiring drone flight. This can be used to tune the
  collision avoidance parameters.
- `collision_prevention.py`: demonstrates how to use perform collision avoidance
  on agents.
- `goto.py`: demonstrates how to command an agent to fly to a specified location
using a configuration file; an example configuration file is provided in
`resources/goto.yaml`.
- `mission_report.py`: demonstrates how to use the `parse_log_file` method to
  create a simple mission report.
- `set_flight_mode.py`: demonstrates how to set the agents' flight mode.
- `set_home_position.py`: demonstrates how to set the swarm agents' home
position.
- `set_parameter.py`: demonstrates how to set an agent's parameter.
- `subclass_mavswarm.py`: demonstrates how to extend the functionality of
`MavSwarm` through subclassing.
- `takeoff_sequence.py`: demonstrates how to execute a full takeoff sequence.

## Running the examples

Prior to running the examples, ensure that there is an active connection between
a ground control station and the swarm agents. When these examples were first
developed, the ground control station and the swarm agents utilized RFD900
radios configured to use the multi-point radio setting.

Once the network has been configured, the examples may be run using the
following command:

```bash
python3 <example_script.py> <args>
```

where `<example_script.py>` is replaced with the name of the example script to
run and `<args>` is replaced with the example's arguments. To see the
arguments, run the following command:

```bash
python3 <example_script.py> -h
```
