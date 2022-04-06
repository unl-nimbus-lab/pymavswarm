import logging
from .Agent import Agent
from typing import Optional, Union
from .Connection import Connection


class MavSwarm:
    """
    Primary interface for pymavswarm and enables users to
    send commands to the swarm and read the swarm state
    """

    def __init__(self, debug: bool = False) -> None:
        """
        :param debug: Flag indicating whether to log debug messages, defaults to False
        :type debug: bool, optional
        """
        super().__init__()

        # Initialize loggers
        self.__debug = debug
        self.__logger = self.__init_logger("mavswarm", debug=debug)

        # Class variables
        self.__connection = None

        return

    @property
    def connection(self) -> Connection:
        """
        The MAVLink connect, provides an interface to the network

        :rtype: Connection
        """
        return self.__connection

    @connection.setter
    def connection(self, connection: Union[Connection, None]) -> None:
        """
        connection setter

        :param connection: New connection
        :type connection: Union[Connection, None]
        """
        self.__connection = connection
        return

    def __init_logger(self, name: str, debug: bool = False) -> logging.Logger:
        """
        Initialize the logger with the desired debug levels

        :param name: The name of the logger
        :type name: str

        :param debug: Flag indicating whether to log debug level messages, defaults to 
            False
        :type debug: bool, optional

        :return: Configured logger
        :rtype: logging.Logger
        """
        logging.basicConfig()

        # Set the desired debug level
        if debug:
            logger = logging.getLogger(name)
            logger.setLevel(logging.DEBUG)
            return logger
        else:
            return logging.getLogger(name)

    def connect(
        self,
        port: str,
        baudrate: int,
        source_system: int = 255,
        source_component: int = 0,
    ) -> bool:
        """
        Create a new connection using the specified serial port

        :param port: The serial port to attempt connection on
        :type port: str

        :param baudrate: The serial connection baudrate
        :type baudrate: int

        :param source_system: The system ID for the source system, defaults to 255
        :type source_system: int, optional

        :param source_component: The component ID for the source system, defaults to 0
        :type source_component: int, optional

        :return: Flag indicating whether connection was successful
        :rtype: bool
        """
        if self.__connection is None:
            try:
                self.__connection = Connection(
                    port, baudrate, source_system, source_component, debug=self.__debug
                )
                self.__connection.start_connection()
            except Exception:
                # Ensure that the connection is none on failure
                self.__connection = None
                
                # Handle the error message
                self.__logger.debug(
                    "MavSwarm was unable to establish a connection with the "
                    "specified device",
                    exc_info=True,
                )

                return False

        return True

    def disconnect(self) -> bool:
        """
        Disconnect from the network

        :return: Flag indicating whether the disconnect attempt was successful
        :rtype: bool
        """
        if self.__connection is not None:
            self.__connection.disconnect()
            self.__connection = None

        return True

    def send_msg(self, msgs: list) -> None:
        """
        Add the message to the connection's outgoing messages queue

        :param msgs: A list of messages to send to the network/agents
        :type msgs: list
        """
        for msg in msgs:
            # Ensure that the intended agent is in the network
            if (msg.target_system, msg.target_comp) in self.__connection.devices:
                self.__connection.send_msg_handler(msg)

        return

    def set_param(self, params: list) -> None:
        """
        Add the params to the connection's outgoing parameter queue

        :param params: A list of parameters to set/read from agents
        :type params: list
        """
        for param in params:
            # Ensure that the intended agent is in the network
            if (param.sys_id, param.comp_id) in self.__connection.devices:
                self.__connection.set_param_handler(param)

        return

    def read_param(self, params: list) -> None:
        """
        Read a desired parameter value. Note that, in the current configuration
        each agent stores the most recent 5 parameter values read in a circular
        buffer. Therefore, when performing a bulk parameter read, ensure that
        a maximum of five parameters are read at once on the specific agent
        """
        for param in params:
            # Ensure that the intended agent is in the network
            if (param.sys_id, param.comp_id) in self.__connection.devices:
                self.__connection.read_param_handler(param)

        return

    def get_agents(self) -> list:
        """
        Get the list of agents in the network
        """
        if self.__connection is not None:
            return [*self.__connection.devices.values()]
        else:
            return []

    def get_agent_by_id(self, sys_id: int, comp_id: int) -> Optional[Agent]:
        """
        Get a specific agent by its system ID and component ID
        """
        device_id = (sys_id, comp_id)

        if device_id in self.__connection.devices:
            return self.__connection.devices[device_id]

        return None

    def get_agent_by_name(self, name: str) -> Optional[Agent]:
        """
        Get the first agent in the swarm with the specified name
        """
        for agent in self.__connection.devices.values():
            if agent.name == name:
                return agent

        return None
