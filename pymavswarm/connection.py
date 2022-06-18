# pymavswarm is an interface for swarm control and interaction
# Copyright (C) 2022  Evan Palmer

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

"""Wrapper for MAVLink connection."""

import atexit
import logging
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from typing import Any, Dict, Tuple, Union

import monotonic
from pymavlink import mavutil

import pymavswarm.messages as swarm_messages
import pymavswarm.utils as swarm_utils
from pymavswarm import Agent
from pymavswarm.handlers import MessageReceivers, Senders
from pymavswarm.messages import responses
from pymavswarm.plugins import supported_plugins
from pymavswarm.utils import Event


class Connection:
    """Handle interaction with the network and the MAVLink connection."""

    def __init__(
        self,
        max_workers: int = 5,
        logger_name: str = "connection",
        log_level: int = logging.INFO,
    ) -> None:
        """
        Construct a new Connection object.

        :param max_workers: maximum number of workers available in the thread pool used
            to send messages, defaults to 5
        :type: max_workers: int, optional

        :param logger_name: name of the logger
        :type logger_name: str, optional

        :param log_level: log level of the connection logger, defaults to
            logging.INFO
        :type debug: int, optional
        """
        # pylint: disable=too-many-instance-attributes
        self.__logger = swarm_utils.init_logger(logger_name, log_level=log_level)

        self.__mavlink_connection = None
        self.__connected = False
        self.__agents: Dict[Tuple[int, int], Agent] = {}
        self.__agent_list_changed = Event()
        self.__agent_timeout = 0.0
        self.__plugins: list = []

        # Sender and receiver interfaces
        self.__message_senders = Senders(log_level=log_level)
        self.__message_receivers = MessageReceivers(log_level=log_level)

        # Mutexes
        self.__read_message_mutex = threading.RLock()
        self.__send_message_mutex = threading.RLock()

        # Load the plugins
        self.__load_plugins(supported_plugins)

        # Register the exit callback
        atexit.register(self.disconnect)

        # Threads
        self.__heartbeat_thread = threading.Thread(target=self.__send_heartbeat)
        self.__heartbeat_thread.daemon = True

        self.__incoming_message_thread = threading.Thread(target=self.__receive_message)
        self.__incoming_message_thread.daemon = True

        # Thread pool for sending messages
        self.__send_message_thread_pool_executor = ThreadPoolExecutor(
            max_workers=max_workers
        )

        return

    @property
    def mavlink_connection(self) -> Any:
        """
        Mavlink connection.

        :return: mavlink connection
        :rtype: Any
        """
        return self.__mavlink_connection

    @property
    def connected(self) -> bool:
        """
        Mavlink connection status.

        :return: connection status
        :rtype: bool
        """
        return self.__connected

    @property
    def agent_timeout(self) -> float:
        """
        Agent timeout duration.

        Maximum time that an agent has to send a heartbeat message before it is
        considered timed out.

        :return: agent timeout duration
        :rtype: float
        """
        return self.__agent_timeout

    @property
    def agents(self) -> dict:
        """
        Set of recognized agents.

        The dict keys are (system ID, component ID) tuples

        :return: swarm agents
        :rtype: dict
        """
        return self.__agents

    @property
    def agent_list_changed(self) -> Event:
        """
        Event indicating that the list of agents had a new agent added or removed.

        :return: event signaling when the set of agents changes
        :rtype: Event
        """
        return self.__agent_list_changed

    @property
    def read_message_mutex(self) -> threading.RLock:
        """
        Mutex restricting access to the recv message method.

        :rtype: threading.RLock
        """
        return self.__read_message_mutex

    @property
    def send_message_mutex(self) -> threading.RLock:
        """
        Mutex restricting access to the send message method.

        :rtype: threading.RLock
        """
        return self.__send_message_mutex

    def __load_plugins(self, plugins: list) -> None:
        """
        Add the plugin handlers to the connection handlers.

        :param plugins: list of plugins to load
        :type plugins: List[Callable]
        """
        # Instantiate each of the plugins
        self.__plugins = [plugin() for plugin in plugins]

        # Add the plugin handlers to the primary handlers
        for plugin in self.__plugins:
            self.__message_senders.senders.update(plugin.senders)
            self.__message_receivers.receivers.update(plugin.receivers)

        return

    def __send_heartbeat(self) -> None:
        """Send a GCS heartbeat to the network."""
        while self.__connected and self.__mavlink_connection is not None:
            self.__mavlink_connection.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0,
                0,
                0,
            )

            # Send a heartbeat at the recommended 1 Hz interval
            time.sleep(1)

        return

    def __update_agent_timeout_states(self) -> None:
        """Update the timeout status of each agent in the network."""
        for agent in self.__agents.values():
            if agent.last_heartbeat.value is not None:
                agent.timeout.value = (
                    monotonic.monotonic() - agent.last_heartbeat.value
                ) >= agent.timeout_period.value

        return

    def __receive_message(self) -> None:
        """Handle incoming messages and distribute them to their respective handlers."""
        while self.__connected and self.__mavlink_connection is not None:
            self.__update_agent_timeout_states()

            message = None

            # Attempt to read the message
            # Note that a timeout has been integrated. Consequently not ALL messages
            # may be received from an agent
            if self.__read_message_mutex.acquire(timeout=0.1):
                try:
                    message = self.__mavlink_connection.recv_msg()
                except Exception:
                    self.__logger.debug(
                        "An error occurred on MAVLink message reception", exc_info=True
                    )
                finally:
                    self.__read_message_mutex.release()

            # Continue if the message read was not read properly
            if not message:
                continue

            # Execute the respective message handler(s)
            if message.get_type() in self.__message_receivers.receivers:
                for function in self.__message_receivers.receivers[message.get_type()]:
                    try:
                        function(message, self)
                    except Exception:
                        self.__logger.exception(
                            "Exception in message handler for %s",
                            message.get_type(),
                            exc_info=True,
                        )

        return

    def send_message(self, message: Union[Any, swarm_messages.MessagePackage]) -> bool:
        """
        Send a message.

        :param message: message to send
        :type message: Union[Any, swarm_messages.MessagePackage]
        """
        if self.__connected:
            if isinstance(message, swarm_messages.MessagePackage):
                future = self.__send_message_thread_pool_executor.submit(
                    self.__send_message_package, message
                )
            else:
                future = self.__send_message_thread_pool_executor.submit(
                    self.__send_message, message
                )

        result = False

        try:
            result = future.result()
        except Exception:
            pass

        return result

    def __send_message_package(self, package: swarm_messages.MessagePackage) -> bool:
        """
        Send a message package.

        :param package: package of messages to send
        :type package: MessagePackage
        """
        # Initial attempt at sending each message in the package
        for message in package.messages:
            if not self.__send_message(message):
                package.messages_failed.append(message)
            else:
                package.messages_succeeded.append(message)

        # Retry sending the failed messages
        if package.retry and len(package.messages_failed) > 0:
            for _ in range(package.max_retry_attempts):
                # Only retry sending the failed messages
                if len(package.messages_failed) > 0:
                    for message in package.messages_failed:
                        if self.__send_message(message):
                            package.messages_failed.remove(message)
                            package.messages_succeeded.append(message)
                else:
                    break

        # The package send is considered successful if there are no messages that
        # failed
        package_success = len(package.messages_failed) == 0

        if package_success:
            package.response = responses.SUCCESS
            package.package_result_event.notify(context=package.context)
        else:
            package.response = responses.PACKAGE_FAILURE
            package.package_result_event.notify(context=package.context)

        return package_success

    def __send_message(self, message: Any) -> bool:
        """
        Send a message to an agent in the network.

        :param message: message to send
        :type message: Any

        :return: message success
        :rtype: bool
        """
        # Send the message if there is a message sender for it
        if message.message_type not in self.__message_senders.senders:
            self.__logger.info(
                "Attempted to send an unsupported message type: %s",
                message.message_type,
            )
            return False

        # Determine whether the agent has been recognized
        if (message.target_system, message.target_comp) in self.__agents:
            agent_exists = True
        else:
            self.__logger.info(
                "The current set of registered agents does not include Agent "
                "(%s, %s}). The provided message will still be sent; however, the "
                "system may not be able to confirm reception of the message.",
                message.target_system,
                message.target_component,
            )

        message_success = True

        # Call each handler for the message
        for function_id, function in enumerate(
            self.__message_senders.senders[message.message_type]
        ):
            with self.__send_message_mutex:
                function_result = False

                try:
                    function_result = function(
                        message,
                        self,
                        function_id=function_id,
                        agent_exists=agent_exists,
                    )
                except Exception:
                    self.__logger.exception(
                        "Exception in message sender for %s",
                        message.message_type,
                        exc_info=True,
                    )
                finally:
                    if not function_result:
                        message_success = False

        return message_success

    def read_parameter(self):
        return

    def set_parameter(self):
        return

    def connect(
        self,
        port: str,
        baudrate: int,
        source_system: int = 255,
        source_component: int = 0,
        connection_attempt_timeout: float = 2.0,
        agent_timeout: float = 30.0,
    ) -> bool:
        """
        Establish a MAVLink connection.

        Attempt to establish a MAVLink connection using the provided configurations.

        :param port: port over which a connection should be established
        :type port: str

        :param baud: baudrate that a connection should be established with
        :type baud: int

        :param source_system: system ID of the connection, defaults to 255
        :type source_system: int, optional

        :param source_component: component ID of the connection, defaults to 0
        :type source_component: int, optional

        :param connection_attempt_timeout: maximum amount of time allowed to attempt
            to establish a connection, defaults to 2.0 [s]
        :type connection_attempt_timeout: float, optional

        :param agent_timeout: amount of time allowable between agent heartbeats
            before an agent is considered timed out, defaults to 30.0
        :type agent_timeout: float, optional

        :return: whether or not the connection attempt was successful
        :rtype: bool
        """
        # Create a new mavlink connection
        self.__mavlink_connection = mavutil.mavlink_connection(
            port,
            baud=baudrate,
            source_system=source_system,
            source_component=source_component,
            autoreconnect=True,
        )

        if self.__mavlink_connection is None:
            return False

        # Ensure that a connection has been successfully established
        # Integrate a 2 second timeout
        resp = self.__mavlink_connection.wait_heartbeat(
            timeout=connection_attempt_timeout
        )

        if not resp:
            connected = False
        else:
            connected = True

            # Set the agent timeout
            self.__agent_timeout = agent_timeout

            # Start background threads
            self.__heartbeat_thread.start()
            self.__incoming_message_thread.start()

        # Update the internal connection state
        self.__connected = connected

        return connected

    def disconnect(self) -> None:
        """Close the connection and stop all threads."""
        # Stop the main loop of the threads
        self.__connected = False

        # Join system threads
        if self.__heartbeat_thread is not None:
            self.__heartbeat_thread.join()

        if self.__incoming_message_thread is not None:
            self.__incoming_message_thread.join()

        # Shutdown the thread pool executor
        self.__send_message_thread_pool_executor.shutdown(cancel_futures=True)

        # Clear the agents list
        self.__agents.clear()
        self.__agent_list_changed.listeners.clear()

        # Shutdown the mavlink connection
        if self.__mavlink_connection is not None:
            self.__mavlink_connection.close()

        return
