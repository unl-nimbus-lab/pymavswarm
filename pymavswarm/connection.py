import atexit
import logging
import threading
import time
from sqlite3 import connect
from typing import Any, Callable, List

import monotonic
from pymavlink import mavutil

import pymavswarm.msg as swarm_msgs
import pymavswarm.state as swarm_state
from pymavswarm.event import Event
from pymavswarm.handlers import Receivers, Senders
from pymavswarm.msg import responses
from pymavswarm.param import Parameter
from pymavswarm.plugins import plugins


class Connection:
    """
    Handles all interaction with the network and the MAVLink master device.
    """

    def __init__(
        self,
        logger_name: str = "connection",
        log_level: int = logging.INFO,
    ) -> None:
        """
        Constructor.

        :param logger_name: name of the logger
        :type logger_name: str, optional

        :param log_level: log level of the connection logger, defaults to
            logging.INFO
        :type debug: int, optional

        :raises TimeoutError: The system was unable to establish a connection
        """
        self.__logger = self.__init_logger(logger_name, log_level=log_level)
        self.__connection = None
        self.__connected = False
        self.__devices = {}
        self.__device_list_changed = Event()
        self.__agent_timeout = 0.0
        self.__plugins = []

        # Sender and receiver interfaces
        self.__senders = Senders()
        self.__receivers = Receivers()

        # Mutexes
        self.__read_msg_mutex = threading.RLock()
        self.__send_msg_mutex = threading.RLock()

        # Load the plugins
        self.__load_plugins(plugins)

        # Register the exit callback
        atexit.register(self.disconnect)

        # Threads
        self.__heartbeat_t = threading.Thread(target=self.__send_heartbeat)
        self.__heartbeat_t.daemon = True

        self.__incoming_msg_t = threading.Thread(target=self.__incoming_msg_handler)
        self.__incoming_msg_t.daemon = True

        return

    @property
    def mavlink_connection(self) -> Any:
        """
        mavlink connection used to send and receive commands

        :rtype: Any
        """
        return self.__connection

    @property
    def connected(self) -> bool:
        """
        Flag indicating whether the system currently has a connection

        :rtype: bool
        """
        return self.connected

    @property
    def agent_timeout(self) -> float:
        """
        The maximum time that an agent has to send a heartbeat message before it is
        considered timed out.

        :rtype: float
        """
        return self.__agent_timeout

    @property
    def devices(self) -> dict:
        """
        The current set of recognized devices

        The dict keys are (system ID, component ID) tuples

        :rtype: dict
        """
        return self.__devices

    @property
    def device_list_changed(self) -> Event:
        """
        Event indicating that the list of devices had a new device added or removed

        :rtype: Event
        """
        return self.__device_list_changed

    @property
    def read_message_mutex(self) -> threading.RLock:
        """
        Mutex restricting access to the recv message method.

        :rtype: threading.RLock
        """
        return self.__read_msg_mutex

    @property
    def send_message_mutex(self) -> threading.RLock:
        """
        Mutex restricting access to the send message method.

        :rtype: threading.RLock
        """
        return self.__send_msg_mutex

    def __init_logger(self, name: str, log_level: int = logging.INFO) -> logging.Logger:
        """
        Initialize the logger with the desired debug levels

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

    def __send_heartbeat(self) -> None:
        """
        Function used to sent a heartbeat to the network indicating that the GCS
        is still operating
        """
        while self.__connected:
            self.__connection.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0,
                0,
                0,
            )

            # Send a heartbeat every 2 seconds
            time.sleep(2)

        return

    def __load_plugins(self, plugin_handlers: List[Callable]) -> None:
        """
        Add the plugin handlers to the connection handlers.

        :param plugins: list of plugins to load
        :type plugins: List[Callable]
        """
        # Instantiate each of the plugins
        self.__plugins = [plugin() for plugin in plugin_handlers]

        # Add the plugin handlers to the primary handlers
        for plugin in self.__plugins:
            self.__senders.senders.update(plugin.senders)
            self.__receivers.receivers.update(plugin.receivers)

        return

    def __incoming_msg_handler(self) -> None:
        """
        Handle incoming messages and distribute them to their respective handlers.
        """
        while self.__connected:
            # Update the timeout flag for each device
            for device in self.__devices.values():
                if device.last_heartbeat.value is not None:
                    device.timeout.value = (
                        monotonic.monotonic() - device.last_heartbeat.value
                    ) >= device.timeout_period.value

            # Read a new message
            try:
                if not self.__read_msg_mutex.acquire(timeout=1.0):
                    msg = None
                else:
                    msg = self.__connection.recv_msg()
                    self.__read_msg_mutex.release()
            except mavutil.mavlink.MAVError:
                self.__logger.debug("An error occurred on MAVLink message reception")
                msg = None
            except Exception:
                # Log any other unexpected exception
                self.__logger.exception(
                    "Exception while receiving message: ", exc_info=True
                )
                msg = None

            if not msg:
                continue

            # Apply the respective message handler(s)
            if msg.get_type() in self.__receivers.receivers:
                for function in self.__receivers.receivers[msg.get_type()]:
                    try:
                        function(msg, self)
                    except Exception:
                        self.__logger.exception(
                            f"Exception in message handler for {msg.get_type()}",
                            exc_info=True,
                        )

        return

    def __retry_param_send(self, param: Any, function: Callable) -> bool:
        """
        Retry a parameter send until the an acknowledgement is received or a timeout
        occurs

        :param param: The parameter to retry sending
        :type msg: Any

        :param function: The function to call using the message
        :type function: function

        :return: Indicate whether the retry was successful
        :rtype: bool
        """
        ack = False
        start_time = time.time()

        # Don't let the message come back here and create an infinite loop
        param.retry = False

        while time.time() - start_time <= param.msg_timeout:
            # Reattempt the message send
            if function(param):
                ack = True
                break

        return ack

    def send_msg_handler(self, msg: Any) -> None:
        """
        Public method that is accessed by the mavswarm interface to signal the handler
        to complete message sending

        :param msg: The message to send
        :type msg: Any
        """
        # Make sure that a connection is established before attempting to send a message
        if self.__connected:
            handler_t = threading.Thread(target=self.__send_msg_handler, args=(msg,))

            # Send the message
            handler_t.start()

        return

    def send_msg_package_handler(self, package: swarm_msgs.MsgPackage) -> None:
        """
        Public method that is accessed by the mavswarm interface to signal the handler
        to send a message package

        :param package: Package of messages to send
        :type package: MsgPackage
        """
        # Make sure that a connection is established before attempting to send a message
        if self.__connected:
            handler_t = threading.Thread(
                target=self.__send_msg_package_handler, args=(package,)
            )

            # Send the message
            handler_t.start()

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
                device_exists = False

                if (msg.target_system, msg.target_comp) in self.__devices:
                    device_exists = True
                else:
                    self.__logger.info(
                        "The current set of registered devices does not include Agent "
                        f"({msg.target_system, msg.target_comp}). The provided message "
                        "will still be sent; however, the system may not be able to "
                        "confirm reception of the message."
                    )

                for function_id, function in enumerate(
                    self.__message_senders[msg.msg_type]
                ):
                    # Prevent multiple sends from occurring at once
                    self.__send_msg_mutex.acquire()

                    # Result of the particular function called
                    function_result = False

                    # Execute the command
                    try:
                        function_result = function(
                            self,
                            msg,
                            function_id=function_id,
                            device_exists=device_exists,
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
                        self.__send_msg_mutex.release()
        except Exception:
            self.__logger.exception(
                "An error occurred while attempting to send the provided message",
                exc_info=True,
            )

        return msg_function_results

    def set_param_handler(self, param: Parameter) -> None:
        """
        Set the value of a parameter on a given agent

        :param param: The parameter to set
        :type param: Parameter
        """
        # Make sure that a connection is established before attempting to set a param
        if self.__connected:
            handler_t = threading.Thread(target=self.__set_param_handler, args=(param,))

            # Set the parameter
            handler_t.start()

        return

    def __set_param_handler(self, param: Parameter) -> None:
        """
        Handle setting parameters on an agent in the network

        :param param: The parameter to set
        :type param: Parameter
        """
        # Prevent multiple sends from occurring at once
        self.__send_msg_mutex.acquire()

        try:
            self.__set_param(param)
        except Exception:
            self.__logger.exception(
                "An error occurred while attempting to send the provided message",
                exc_info=True,
            )
        finally:
            self.__send_msg_mutex.release()

        return

    def __set_param(self, param: Parameter) -> bool:
        """
        Set the value of a parameter.

        NOTE: This sets the parameter value in RAM and not to EEPROM. Therefore, on
        reboot, the parameters will be reset to their default values

        :param param: The parameter to set
        :type param: Parameter

        :return: Indicates whether the parameter was successfully set
        :rtype: bool
        """
        try:
            # NOTE: In the current state, we only support float parameter value types
            #       Additional types may be added in the future
            self.__connection.mav.param_set_send(
                param.sys_id,
                param.comp_id,
                str.encode(param.param_id),
                param.param_value,
                9,
            )
        except Exception:
            self.__logger.error(
                f"An error occurred while attempting to set {param.param_id} to "
                f"{param.param_value}",
                exc_info=True,
            )
            return False

        ack = False

        if self.__ack_msg("PARAM_VALUE", timeout=param.ack_timeout)[0]:
            ack = True
        else:
            if param.retry:
                if self.__retry_param_send(param, self.__set_param):
                    ack = True

        if ack:
            self.__logger.info(
                f"Successfully set {param.param_id} to {param.param_value} on "
                f"Agent ({param.sys_id}, {param.comp_id})"
            )
        else:
            self.__logger.error(
                f"Failed to set {param.param_id} to {param.param_value} on Agent "
                f"({param.sys_id}, {param.comp_id})"
            )

        return ack

    def read_param_handler(self, param: Parameter) -> None:
        """
        Read the value of a parameter

        :param param: The parameter to read
        :type param: Parameter
        """
        # Make sure that a connection is established before attempting to set a param
        if self.__connected:
            handler_t = threading.Thread(
                target=self.__read_param_handler, args=(param,)
            )

            # Send the message
            handler_t.start()

        return

    def __read_param_handler(self, param: Parameter) -> None:
        """
        Handler responsible for reading requested parameters.

        NOTE: This thread is primarily responsible for handling read requests and
        verifying that a read was accomplished on the message listener thread. The
        agent state itself is updated on the message listener thread

        :param param: The parameter to read
        :type param: Parameter
        """
        # Prevent multiple reads from occurring at once
        self.__send_msg_mutex.acquire()

        try:
            self.__read_param(param)
        except Exception:
            self.__logger.exception(
                "An error occurred while attempting to send the provided message"
            )
        finally:
            self.__send_msg_mutex.release()

        return

    def __read_param(self, param: Parameter) -> bool:
        """
        Read a desired parameter value

        :param param: The parameter to read
        :type param: Parameter

        :return: Indicates whether the parameter was successfully read
        :rtype: bool
        """
        try:
            self.__connection.mav.param_request_read_send(
                param.sys_id, param.comp_id, str.encode(param.param_id), -1
            )
        except Exception:
            self.__logger.exception(
                f"An exception occurred while attempting to read {param.param_id} "
                f"from Agent ({param.sys_id}, {param.comp_id})",
                exc_info=True,
            )
            return False

        ack = False

        ack, msg = self.__ack_msg("PARAM_VALUE", timeout=param.ack_timeout)

        if ack:
            read_param = swarm_state.ReadParameter(
                msg["param_id"],
                msg["param_value"],
                msg["param_type"],
                msg["param_index"],
                msg["param_count"],
            )

            self.__devices[(param.sys_id, param.comp_id)].last_params_read.append(
                read_param
            )
        else:
            if param.retry:
                if self.__retry_param_send(param, self.__read_param):
                    ack = True

        if ack:
            self.__logger.info(
                f"Successfully read {param.param_id} from Agent ({param.sys_id}, "
                f"{param.comp_id}). Value: {msg}"
            )
        else:
            self.__logger.error(
                f"Failed to read {param.param_id} from Agent ({param.sys_id}, "
                f"{param.comp_id})"
            )

        return ack

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
        self.__connection = mavutil.mavlink_connection(
            port,
            baud=baud,
            source_system=source_system,
            source_component=source_component,
            autoreconnect=True,
        )

        # Ensure that a connection has been successfully established
        # Integrate a 2 second timeout
        resp = self.__connection.wait_heartbeat(timeout=connection_attempt_timeout)

        if resp is None:
            connected = False
        else:
            connected = True

            # Set the agent timeout
            self.__agent_timeout = agent_timeout

            # Start background threads
            self.__heartbeat_t.start()
            self.__incoming_msg_t.start()

        # Update the internal connection state
        self.__connected = connected

        return connected

    def disconnect(self) -> None:
        """
        Close the connection and disconnect all threads
        """
        self.__connected = False

        if self.__heartbeat_t is not None:
            self.__heartbeat_t.join()

        if self.__incoming_msg_t is not None:
            self.__incoming_msg_t.join()

        self.__devices.clear()
        self.__device_list_changed.listeners.clear()

        if self.__connection is not None:
            self.__connection.close()

        return
