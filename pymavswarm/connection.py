import atexit
import logging
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from typing import Any, Callable, List, Union

import monotonic
from pymavlink import mavutil

import pymavswarm.msg as swarm_msgs
from pymavswarm.event import Event
from pymavswarm.handlers import Receivers, Senders
from pymavswarm.msg import responses
from pymavswarm.plugins import supported_plugins


class Connection:
    """
    Handles all interaction with the network and the MAVLink master device.
    """

    def __init__(
        self,
        max_workers: int = 5,
        logger_name: str = "connection",
        log_level: int = logging.INFO,
    ) -> None:
        """
        Constructor.

        :param max_workers: maximum number of workers available in the thread pool used
            to send messages, defaults to 5
        :type: max_workers: int, optional

        :param logger_name: name of the logger
        :type logger_name: str, optional

        :param log_level: log level of the connection logger, defaults to
            logging.INFO
        :type debug: int, optional

        :raises TimeoutError: The system was unable to establish a connection
        """
        self.__logger = self.__init_logger(logger_name, log_level=log_level)
        self.__mavlink_connection = None
        self.__connected = False
        self.__agents = {}
        self.__agent_list_changed = Event()
        self.__agent_timeout = 0.0
        self.__plugins = []

        # Sender and receiver interfaces
        self.__senders = Senders()
        self.__receivers = Receivers()

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

        self.__incoming_message_thread = threading.Thread(
            target=self.__handle_incoming_message
        )
        self.__incoming_message_thread.daemon = True

        # Thread pool for sending messages
        self.__send_message_thread_pool_executor = ThreadPoolExecutor(
            max_workers=max_workers
        )

        return

    @property
    def mavlink_connection(self) -> Any:
        """
        Mavlink connection used to send and receive commands.

        :rtype: Any
        """
        return self.__mavlink_connection

    @property
    def connected(self) -> bool:
        """
        Flag indicating whether the system currently has a connection.

        :rtype: bool
        """
        return self.__connected

    @property
    def agent_timeout(self) -> float:
        """
        Maximum time that an agent has to send a heartbeat message before it is
        considered timed out.

        :rtype: float
        """
        return self.__agent_timeout

    @property
    def agents(self) -> dict:
        """
        Current set of recognized agents.

        The dict keys are (system ID, component ID) tuples

        :rtype: dict
        """
        return self.__agents

    @property
    def agent_list_changed(self) -> Event:
        """
        Event indicating that the list of agents had a new agent added or removed

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

    def __init_logger(self, name: str, log_level: int = logging.INFO) -> logging.Logger:
        """
        Initialize the logger with the desired debug levels.

        :param name: The name of the logger
        :type name: str

        :param log_level: The log level to display, defaults to logging.INFO
        :type log_level: int, optional

        :return: A newly configured logger
        :rtype: logging.Logger
        """
        logging.basicConfig()
        logger = logging.getLogger(name)
        logger.setLevel(log_level)

        return logger

    def __load_plugins(self, plugins: List[Callable]) -> None:
        """
        Add the plugin handlers to the connection handlers.

        :param plugins: list of plugins to load
        :type plugins: List[Callable]
        """
        # Instantiate each of the plugins
        self.__plugins = [plugin() for plugin in plugins]

        # Add the plugin handlers to the primary handlers
        for plugin in self.__plugins:
            self.__senders.senders.update(plugin.senders)
            self.__receivers.receivers.update(plugin.receivers)

        return

    def __send_heartbeat(self) -> None:
        """
        Function used to sent a heartbeat to the network indicating that the GCS
        is still operating.
        """
        while self.__connected:
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

    def __handle_incoming_message(self) -> None:
        """
        Handle incoming messages and distribute them to their respective handlers.
        """
        while self.__connected:
            # Update the timeout state of an agent
            for agent in self.__agents.values():
                if agent.last_heartbeat.value is not None:
                    agent.timeout.value = (
                        monotonic.monotonic() - agent.last_heartbeat.value
                    ) >= agent.timeout_period.value

            message = None

            # Attempt to read the message
            if self.__read_message_mutex.acquire(timeout=0.1):
                try:
                    message = self.__mavlink_connection.recv_msg()
                except Exception:
                    self.__logger.debug(
                        "An error occurred on MAVLink message reception", exc_info=True
                    )
                finally:
                    self.__read_message_mutex.release()

            # Continue of the message read was not read properly
            if not message:
                continue

            # Execute the respective message handler(s)
            if message.get_type() in self.__receivers.receivers:
                for function in self.__receivers.receivers[message.get_type()]:
                    try:
                        function(message, self)
                    except Exception:
                        self.__logger.exception(
                            f"Exception in message handler for {message.get_type()}",
                            exc_info=True,
                        )

        return

    def send_message(self, message: Union[Any, swarm_msgs.MsgPackage]) -> None:
        """
        Send a message.

        :param message: message to send
        :type message: Union[Any, swarm_msgs.MsgPackage]
        """
        if self.__connected:
            if isinstance(message, swarm_msgs.MsgPackage):
                self.__send_message_thread_pool_executor.submit(
                    self.__send_msg_package_handler, message
                )
            else:
                self.__send_message_thread_pool_executor.submit(
                    self.__send_msg_handler, message
                )
        return

    def __send_msg_package_handler(self, package: swarm_msgs.MsgPackage) -> None:
        """
        Helper function used to send each message in a package and to
        manage each message's success/failure

        :param package: Package of messages to send
        :type package: MsgPackage
        """

        def get_msg_success(msg: Any) -> list:
            msg_results = self.__send_msg_handler(msg)

            # Overall message success is a failure if any of the functions fails
            msg_success = True
            for result in msg_results:
                if not result[2]:
                    msg_success = False

            return msg_success

        # Initial attempt at sending each message in the package
        for msg in package.msgs:
            if not get_msg_success(msg):
                package.msgs_failed.append(msg)
            else:
                package.msgs_succeeded.append(msg)

        # Retry sending the failed messages
        if package.retry and len(package.msgs_failed) > 0:
            for _ in range(package.max_retry_attempts):
                # Only retry sending the failed messages
                if len(package.msgs_failed) > 0:
                    for msg in package.msgs_failed:
                        if get_msg_success(msg):
                            package.msgs_failed.remove(msg)
                            package.msgs_succeeded.append(msg)
                else:
                    break

        if len(package.msgs_failed) > 0:
            package.response = responses.SUCCESS
            package.package_result_event.notify(context=package.context)
        else:
            package.response = responses.PACKAGE_FAILURE
            package.package_result_event.notify(context=package.context)

        return

    def __send_msg_handler(self, msg: Any) -> list:
        """
        Handle sending messages to the agents on the network

        :param msg: The message to send
        :type msg: Any
        """
        # List of the results for each function called by the message
        msg_function_results = []

        try:
            # Send the message if there is a message sender for it
            if msg.msg_type in self.__message_senders:
                agent_exists = False

                if (msg.target_system, msg.target_comp) in self.__agents:
                    agent_exists = True
                else:
                    self.__logger.info(
                        "The current set of registered agents does not include Agent "
                        f"({msg.target_system, msg.target_comp}). The provided message "
                        "will still be sent; however, the system may not be able to "
                        "confirm reception of the message."
                    )

                for function_id, function in enumerate(
                    self.__message_senders[msg.msg_type]
                ):
                    # Prevent multiple sends from occurring at once
                    self.__send_message_mutex.acquire()

                    # Result of the particular function called
                    function_result = False

                    # Execute the command
                    try:
                        function_result = function(
                            self,
                            msg,
                            function_id=function_id,
                            agent_exists=agent_exists,
                        )
                    except Exception:
                        self.__logger.exception(
                            f"Exception in message sender for {msg.msg_type}",
                            exc_info=True,
                        )
                    finally:
                        msg_function_results.append(
                            (msg.msg_type, function_id, function_result)
                        )
                        self.__send_message_mutex.release()
        except Exception:
            self.__logger.exception(
                "An error occurred while attempting to send the provided message",
                exc_info=True,
            )

        return msg_function_results

    def connect(
        self,
        port: str,
        baud: int,
        source_system: int = 255,
        source_component: int = 0,
        connection_attempt_timeout: float = 2.0,
        agent_timeout: float = 30.0,
    ) -> bool:
        """
        Establish a MAVLink connection.

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
            baud=baud,
            source_system=source_system,
            source_component=source_component,
            autoreconnect=True,
        )

        # Ensure that a connection has been successfully established
        # Integrate a 2 second timeout
        resp = self.__mavlink_connection.wait_heartbeat(
            timeout=connection_attempt_timeout
        )

        if resp is None:
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
        """
        Close the connection and disconnect all threads
        """
        self.__connected = False

        if self.__heartbeat_thread is not None:
            self.__heartbeat_thread.join()

        if self.__incoming_message_thread is not None:
            self.__incoming_message_thread.join()

        self.__agents.clear()
        self.__agent_list_changed.listeners.clear()

        if self.__mavlink_connection is not None:
            self.__mavlink_connection.close()

        return
