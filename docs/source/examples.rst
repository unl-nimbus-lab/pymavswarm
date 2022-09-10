Examples
========

Here we provide a collection of examples that demonstrate how to use the ``pymavswarm``
system. You can find the full source code for these examples on the project
`repository`_. Note that not all functionality implemented in ``pymavswarm`` has an
associated example. For a full list of the functionality provided, please reference
the API documentation.

.. _repository: https://github.com/unl-nimbus-lab/pymavswarm/tree/main/examples

Connecting to a radio network
-----------------------------

In this example, we demonstrate how to create a MAVLink connection to a swarm network.
For this example, we assume that the network has already been configured and is
observable by the system.

.. code-block:: python

    from pymavswarm import MavSwarm

    if __name__ == "__main__":
        # Create a new MavSwarm instance
        mavswarm = MavSwarm()

        port = "/dev/ttyUSB0" # specify the path to the radio device
        baud = 230400 # specify the baudrate used by the radio device

        if mavswarm.connect(port, baud):
            print("Whooooo! I successfully connected to the swarm network!")
        else:
            print("I failed to connect to the swarm; this is the worst day ever.")
            return

        # Disconnect from the network
        mavswarm.disconnect()

        return


Checking the registered agents
------------------------------

This example demonstrates how to see which agents have been registered in the network
and how to check the state of those agents. We assume that you are able to successfully
establish a MAVLink connection for this example.

.. code-block:: python

    from pymavswarm import MavSwarm

    if __name__ == "__main__":
        # Create a new MavSwarm instance
        mavswarm = MavSwarm()

        port = "/dev/ttyUSB0" # specify the path to the radio device
        baud = 230400 # specify the baudrate used by the radio device

        # Attempt to create a new MAVLink connection
        if not mavswarm.connect(port, baud):
            return

        # Wait for the swarm to auto-register new agents
        while not list(filter(lambda agent_id: agent_id[1] == 1, mavswarm.agent_ids)):
            print("Waiting for the system to recognize agents in the network...")
            time.sleep(0.5)

        duration = 5  # specify the duration that we should print the state for
        start_t = time.time()

        while time.time() - start_t < duration:
            for agent_id in list(
                filter(lambda agent_id: agent_id[1] == 1, mavswarm.agent_ids)
            ):
                agent = mavswarm.get_agent_by_id(agent_id)

                if agent is not None:
                    print(f"The current attitude of {agent} is: {agent.attitude}")

                time.sleep(0.5)

        # Disconnect from the network
        mavswarm.disconnect()

        return

Let's break down this example now.

.. code-block:: python

    while not list(filter(lambda agent_id: agent_id[1] == 1, mavswarm.agent_ids)):
        print("Waiting for the system to recognize agents in the network...")
        time.sleep(0.5)

In this code block, we wait for ``MavSwarm`` to recognize an agent in the swarm with
a component ID of 1. This is accomplished by filtering the list of agent IDs that have
been observed in the swarm (made available through ``mavswarm.agent_ids``). Using this
method we could also wait for specific agents to be registered:

.. code-block:: python

    wait_for_agents: list[AgentID] = [
        (1, 1),
        (2, 1),
        (3, 1),
    ]

    while not wait_for_agents in mavswarm.agent_ids:
        print("Waiting for the system to recognize agents in the network...")
        time.sleep(0.5)

This method ensures that all agents that should have commands sent to them are
registered prior to sending any commands.

.. code-block:: python

    duration = 5  # specify the duration that we should print the state for
    start_t = time.time()

    while time.time() - start_t < duration:
        for agent_id in list(
            filter(lambda agent_id: agent_id[1] == 1, mavswarm.agent_ids)
        ):
            agent = mavswarm.get_agent_by_id(agent_id)

            if agent is not None:
                print(f"The current attitude of {agent} is: {agent.attitude}")

            time.sleep(0.5)

In the above code block, the attitude of each agent in the network with a component ID
of 1 is printed out. To get a specific agent from the list of registered agents, the
command

.. code-block:: python

    agent = mavswarm.get_agent_by_id(agent_id)

is used. This method enables accessing a specific agent object using its respective
identifier. Lastly, we print the current state by accessing the ``attitude`` property.

Ignoring specific agents
------------------------

Sometimes there are agents in the network that we don't particularly want to interact
with. An example of this may be a ground control station. ``pymavswarm`` provides a
simple way to ignore those agents to ensure that any messages sent by them are ignored.
``pymavswarm`` enables specifying specific agent IDs that should be ignored, specific
system IDs that should be ignored, and specific component IDs that should be ignored.

.. code-block:: python

    agents_to_ignore = [
        (255, 0), # Ignore the ground control station
        (1, None), # Ignore any agent with a system ID of 1
        (None, 0), # Ignore any agent with a component ID of 0
    ]

    mavswarm = MavSwarm(ignore_ids=agents_to_ignore)

Logging MAVLink messages to a file
----------------------------------

Logging incoming messages to a file can be a useful tool for debugging and for
evaluating the performance of the system. This can be accomplished using the
``pymavswarm`` logging interface.

.. code-block:: python

    mavswarm = MavSwarm(log_to_file=True, log_filename="my-first-log.log")

In the above code block, we first specify that file logging should be done by setting
the ``log_to_file`` argument to ``True``. Next, the file name of the log file is
specified by setting ``log_filename`` to ``my-first-log.log``. This argument can be
left as its default value (``None``) to name the log file as the date and time that
the log was created. All log files are stored in a ``logs/`` directory that is created
in the current working directory that the script is run from.

After a log file has been generated, the file can be parsed for usage using the
``parse_log_file`` method:

.. code-block:: python

    from pymavswarm.utils import parse_log_file

    logfile = "path/to/my/logfile.log"

    msg_df = parse_log_file(logfile)

The ``parse_log_file`` method should be passed the full path to the log file that was
generated. The return value is a dictionary. The dictionary keys are the MAVLink
message types that were logged (e.g., ``GLOBAL_POSITION_INT``), and the value for a key
is a `pandas DataFrame`_. This provides an easy way to interact with the data and
perform statistical analyses on the data.

.. _pandas DataFrame: https://pandas.pydata.org/pandas-docs/stable/reference/api/pandas.DataFrame.html


Arming & disarming
------------------

This example demonstrates how to arm and disarm swarm agents. This example assumes that
a MAVLink can be successfully established and that swarm agents can be recognized.
When running this example, ensure that all propellers have been removed.

.. code-block:: python

    # Arm all agents in the swarm; retry on message failure
    future = mavswarm.arm(verify_state=True, retry=True)

    # Wait for the arm command to complete
    while not future.done():
        pass

    # Let each of the agents idle for a few seconds
    time.sleep(5)

    # Disarm each of the agents; retry on message failure
    future = mavswarm.disarm(retry=True, verify_state=True, force=True)

    # Wait for the disarm command to complete
    while not future.done():
        pass

Let's break down this example.

.. code-block:: python

    # Arm all agents in the swarm; retry on message failure
    future = mavswarm.arm(verify_state=True, retry=True)

    # Wait for the arm command to complete
    while not future.done():
        pass

In the above code block, all swarm agents in the network are commanded to arm. Two
parameters worth noting are set in this example:

1. ``verify_state``
2. ``retry``

The ``verify_state`` parameter informs ``MavSwarm`` that it should check to make sure
that each agent that it sends the arming message to switches into the armed state. The
``retry`` parameter informs ``MavSwarm`` that it should retry sending the arming
MAVLink message if ``MavSwarm`` does not receive a message acknowledgement or the agent
fails to switch into the armed state.

Commanding specific agents
--------------------------

``pymavswarm`` also provides an interface for specifying the specific agents that a
command should be sent to:

.. code-block:: python

    agents_to_arm: list[AgentID] = [(1, 1), (2, 1)]

    future = mavswarm.arm(verify_state=True, retry=True, agent_ids=agents_to_arm)

This enables users to send specific commands to certain agents without commanding all
swarm agents.

Using message futures
---------------------

When a command is sent to swarm agents by a method in the ``pymavswarm`` interface, a
`Future`_ instance is generally returned. The result of this future is a message
``Response``. The message response includes information regarding each agent that a
message was sent to, the MAVLink message that was sent, the result of the method, and a
response code. By utilizing future objects, ``pymavswarm`` provides a way to bind
callbacks to the message result and to prevent system blocking to accomplish message
sending.

.. _Future: https://docs.python.org/3/library/concurrent.futures.html#future-objects

An example demonstrating how this can be used is as follows:

.. code-block:: python

    def print_message_response_cb(future: Future) -> None:
        """
        Print the result of the future.

        :param future: message execution future
        :type future: Future
        """
        # Get the message responses
        responses = future.result()

        for response in responses:
            print(
                f"Result of {response.message_type} message sent to "
                f"({response.target_agent_id}): {response.code}"
            )

        return

    # Execute some command
    future = mavswarm.arm(verify_state=True, retry=True)

    # Attach the callback
    future.add_done_callback(print_message_response_cb)

Using collision avoidance
-------------------------

``pymavswarm`` implements support for multi-agent collision avoidance using
reachability analysis. When enabled, ``MavSwarm`` will compute the reachable sets of
agents according to their current state. The reachable space is computed forward to a
specific time using the face lifting method. Using the reachable sets computed,
``MavSwarm`` will check for potential collisions. If a potential collision is detected,
a specified collision response will be executed on the agents that may collide. Please
note that this is a highly experimental feature and should be used with caution.

To enable collision avoidance, the following code may be executed:

.. code-block:: python

    reach_time = 3.0 # Time (s) that the reachable sets should be computed forward to
    gps_error = 2.5 # 3D GPS error that should be accounted for
    velocity_error = 0.1 # 3D velocity error

    # Enable collision avoidance
    # Switch each agent that may collide into the Loiter flight mode when potential
    # collisions are detected
    mavswarm.enable_collision_avoidance(
        reach_time, gps_error, velocity_error, MavSwarm.COLLISION_RESPONSE_LOITER
    )

Collision avoidance can be later disabled using

.. code-block:: python

    mavswarm.disable_collision_avoidance()

Subclassing MavSwarm
--------------------

``pymavswarm`` has been designed to be extensible to support the various applications
and algorithms that may be implemented for drone swarms. This is accomplished by
enabling support for sub-classing the ``MavSwarm`` class. By sub-classing ``MavSwarm``,
users can easily override existing ``MavSwarm`` commands, add new commands, and add
custom message callbacks. This is demonstrated in the following example:

.. code-block:: python

    from __future__ import annotations

    import time
    from concurrent.futures import Future
    from typing import Any

    import monotonic

    from pymavswarm import Agent, MavSwarm
    from pymavswarm.types import AgentID


    class CustomMavSwarm(MavSwarm):
        """Demonstrate how to subclass the MavSwarm class."""

        def __init__(self, log_level: int = logging.INFO) -> None:
            """
            Create a new custom MavSwarm class.

            :param log_level: logging level, defaults to logging.INFO
            :type log_level: int, optional
            """
            super().__init__(log_level)

            self.add_custom_message_handler("HEARTBEAT", self.custom_handler)

            return

        def fun_command(
            self,
            agent_ids: AgentID | list[AgentID] | None = None,
            retry: bool = False,
            message_timeout: float = 2.5,
            ack_timeout: float = 0.5,
        ) -> Future:
            """
            Send a fun command to the specified agents.

            :param agent_ids: optional list of target agent IDs, defaults to None
            :type agent_ids: AgentID | list[AgentID] | None,
                optional
            :param retry: retry sending the fun command to an agent on failure, defaults
                to False
            :type retry: bool, optional
            :param message_timeout: maximum amount of time allowed to try sending a fun
                command to an agent before a timeout occurs, defaults to 2.5 [s]
            :type message_timeout: float, optional
            :param ack_timeout: maximum amount of time allowed per attempt to verify
                acknowledgement of the fun command, defaults to 0.5 [s]
            :type ack_timeout: float, optional
            :return: future response
            :rtype: Future
            """
            self._logger.info("Attempting to send my fun command!")

            return self.send_debug_message(
                "party-time",
                1,
                agent_ids=agent_ids,
                retry=retry,
                message_timeout=message_timeout,
                ack_timeout=ack_timeout,
            )

        def custom_handler(
            self, message: Any, agents: dict[tuple[int, int], Agent]
        ) -> None:
            """
            Create a custom message handler.

            :param message: message to handle
            :type message: Any
            :param agents: swarm agents
            :type agents: dict[tuple[int, int], Agent]
            """
            sys_id = message.get_srcSystem()
            comp_id = message.get_srcComponent()

            agent_id = (sys_id, comp_id)

            self._logger.info(
                f"Received a message for agent {agent_id} in my custom handler!"
            )

            # Demonstrate how to modify the properties of the swarm agents
            agents[agent_id].last_heartbeat.value = monotonic.monotonic()

            return

To break this down, first consider the following line, executed within the
``CustomMavSwarm`` sub-class:

.. code-block:: python

    self.add_custom_message_handler("HEARTBEAT", self.custom_handler)

In this line, we use the ``add_custom_message_handler`` to assign the ``custom_handler``
callback function to the ``HEARTBEAT`` MAVLink message.

.. code-block:: python

    def custom_handler(
        self, message: Any, agents: dict[AgentID, Agent]
    ) -> None:
        """
        Create a custom message handler.

        :param message: message to handle
        :type message: Any
        :param agents: swarm agents
        :type agents: dict[AgentID, Agent]
        """
        sys_id = message.get_srcSystem()
        comp_id = message.get_srcComponent()

        agent_id = (sys_id, comp_id)

        self._logger.info(
            f"Received a message for agent {agent_id} in my custom handler!"
        )

        # Demonstrate how to modify the properties of the swarm agents
        agents[agent_id].last_heartbeat.value = monotonic.monotonic()

        return

The ``custom_handler`` method, demonstrates a basic callback function. Here, the
``message`` parameter is a MAVLink message that the callback function was assigned to
receive. The ``agents`` parameter includes the agents that have been registered by
the ``pymavswarm`` system. In the example, it is worth noting that the ``agents``
dictionary is directly modified. This will be reflected in the upstream ``agents``
because the ``agents`` parameter value is a mutable object.

.. code-block:: python

    def fun_command(
        self,
        agent_ids: AgentID | list[AgentID] | None = None,
        retry: bool = False,
        message_timeout: float = 2.5,
        ack_timeout: float = 0.5,
    ) -> Future:
        """
        Send a fun command to the specified agents.

        :param agent_ids: optional list of target agent IDs, defaults to None
        :type agent_ids: AgentID | list[AgentID] | None,
            optional
        :param retry: retry sending the fun command to an agent on failure, defaults
            to False
        :type retry: bool, optional
        :param message_timeout: maximum amount of time allowed to try sending a fun
            command to an agent before a timeout occurs, defaults to 2.5 [s]
        :type message_timeout: float, optional
        :param ack_timeout: maximum amount of time allowed per attempt to verify
            acknowledgement of the fun command, defaults to 0.5 [s]
        :type ack_timeout: float, optional
        :return: future response
        :rtype: Future
        """
        self._logger.info("Attempting to send my fun command!")

        return self.send_debug_message(
            "party-time",
            1,
            agent_ids=agent_ids,
            retry=retry,
            message_timeout=message_timeout,
            ack_timeout=ack_timeout,
        )

Finally, we demonstrate how to add a custom command via the ``fun_command``. This
command demonstrates how to use an existing message (i.e., the ``send_debug_message``)
command to send a debug MAVLink message.
