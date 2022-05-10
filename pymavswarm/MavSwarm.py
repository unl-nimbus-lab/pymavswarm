import logging
from pymavswarm.event import Event
from pymavswarm.Agent import Agent
from pymavswarm.Connection import Connection


class MavSwarm:
    """
    Primary interface for pymavswarm and enables users to
    send commands to the swarm and read the swarm state
    """

    def __init__(self, log_level: int = logging.INFO) -> None:
        """
        :param log_level: Log level of the system, defaults to logging.INFO
        :type log_level: int, optional
        """
        super().__init__()

        # Initialize loggers
        self.__log_level = log_level
        self.__logger = self.__init_logger("mavswarm", log_level=log_level)

        # System connection
        self.__connection = None

        return

    @property
    def agents(self) -> dict:
        """
        Get the connection devices

        Used to provide a layer of abstraction from the connection

        :rtype: dict
        """
        if self.__connection is not None:
            return self.__connection.devices
        else:
            return {}

    @property
    def agents_as_list(self) -> list:
        """
        Get the connection devices as a list

        Used to provide a layer of abstraction from the connection

        :rtype: list
        """
        if self.__connection is not None:
            return [*self.__connection.devices.values()]
        else:
            return []

    @property
    def device_list_changed(self) -> Event:
        """
        Event that signals when the list of devices has changed

        :rtype: Event
        """
        return self.__connection.device_list_changed


    def __init_logger(self, name: str, log_level: int = logging.INFO) -> logging.Logger:
        """
        Initialize the logger with the desired log level

        :param name: The name of the logger
        :type name: str

        :param log_level: Log level of the system logger, defaults to logging.INFO
        :type log_level: int, optional

        :return: Configured logger
        :rtype: logging.Logger
        """
        logging.basicConfig()
        logger = logging.getLogger(name)
        logger.setLevel(log_level)
        return logger

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
                    port,
                    baudrate,
                    source_system,
                    source_component,
                    log_level=self.__log_level,
                )
                self.__connection.start_connection()
            except Exception:
                # Ensure that the connection is none on failure
                self.__connection = None

                # Handle the error message
                self.__logger.info(
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
        Read a desired parameter value. Note that, in the current configuration,
        each agent stores the most recent 5 parameter values read in a circular
        buffer. Therefore, when performing a bulk parameter read, ensure that
        a maximum of five parameters are read at once on the specific agent

        :param params: The list of params to read
        :type params: list
        """
        for param in params:
            # Ensure that the intended agent is in the network
            if (param.sys_id, param.comp_id) in self.__connection.devices:
                self.__connection.read_param_handler(param)

        return

    def get_agent_by_id(self, sys_id: int, comp_id: int) -> Agent:
        """
        Get a specific agent by its system ID and component ID

        :param sys_id: The system ID of the agent to access
        :type sys_id: int

        :param comp_id: The component ID of the agent to access
        :type comp_id: int

        :return: The identified agent if found
        :rtype: Optional[Agent]
        """
        device_id = (sys_id, comp_id)

        if device_id in self.__connection.devices:
            return self.__connection.devices[device_id]

        return None

    def get_agent_by_name(self, name: str) -> Agent:
        """
        Get the first agent in the swarm with the specified name

        :param name: The name of the agent to access
        :type name: str

        :return: The first agent identified with the given name
        :rtype: Optional[Agent]
        """
        for agent in self.__connection.devices.values():
            if agent.name == name:
                return agent

        return None
