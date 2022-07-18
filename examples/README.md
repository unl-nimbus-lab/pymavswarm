# Examples

Several examples have been implemented to demonstrate usage of the `pymavswarm` system.
The examples provided include the following:

- `arming_example.py`: example demonstrating how to arm and disarm agents in the swarm
- `check_state_example.py`: example demonstrating how to check various states (flight mode, arm state, location, system ID, component ID) agents in the swarm
- `companion_computer_example.py`: example demonstrating how to use pymavswarm to change the flight mode of each agent (to "loiter") in the swarm from a companion computer
- `flight_mode_change_example.py`: *brief description of example*
- `read_write_parameter_example.py`: *brief description of example*

To get started with these examples, refer to 
[Hardware Configuration](#hardware-configuration)

## Hardware Configuration
Ensure that the following hardware configuration has been integrated to run the
`pymavswarm` examples:

For `companion_computer_example.py`, it needs to be run on a companion computer (e.g., a RPi), and this companion computer should be connected to a flight controller (e.g., a pixhawk blue cube).

To run the others, run them from a groud control station (e.g., a laptop) connected to a radio modem. Also use companion computers (e.g., a RPi) connected to flight controllers (e.g., a pixhawk blue cube) as agents.

Navigate to [Running the Examples](#running-the-examples) to learn how to run each
example.

## Running the Examples

*description discussing how to run the examples*
- `arming_example.py`: Go into the `examples` folder and run `arming_example.py` with the respective arguments: <port> <baudrate>
- `check_state_example.py`: Go into the `examples` folder and run `check_state_example.py` with the respective arguments: <port> <baudrate>
- `companion_computer_example.py`: Go into the `examples` folder and run `companion_computer_example.py` with the respective arguments: <port> <baudrate> <companion_computer_system_id> <companion_computer_component_id>
- `flight_mode_change_example.py`: *brief description of respective arguments*
- `read_write_parameter_example.py`: *brief description of respective arguments*
