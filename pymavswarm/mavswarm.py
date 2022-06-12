import logging
from typing import Any

from pymavswarm import Connection
from pymavswarm.agent import Agent
from pymavswarm.event import Event
from pymavswarm.msg import MsgPackage, SupportedMsgs
from pymavswarm.param import Parameter, ParameterPackage


class MavSwarm:
    """
    Primary interface for pymavswarm and enables users to
    send commands to the swarm and read the swarm state
    """

    def __init__(self, log_level: int = logging.INFO) -> None:
        """
        Constructor.

        :param log_level: log level of the system, defaults to logging.INFO
        :type log_level: int, optional
        """
        super().__init__()

        # Initialize loggers
        self.__log_level = log_level
        self.__logger = self.__init_logger("mavswarm", log_level=log_level)

        # System connection
        self.__connection = Connection()

        return

    @property
    def supported_messages(self) -> SupportedMsgs:
        """
        Message types that are supported by the pymavswarm interface

        :return: mapping to enable simple configuration of commands
        :rtype: SupportedMsgs
        """
        return SupportedMsgs()

    @property
    def agents(self) -> dict:
        """
        Get the connection agents.

        Used to provide a layer of abstraction from the connection.

        :rtype: dict
        """
        if self.__connection is not None:
            return self.__connection.agents
        else:
            return {}

    @property
    def agents_as_list(self) -> list:
        """
        Get the connection agents as a list.

        Used to provide a layer of abstraction from the connection.

        :rtype: list
        """
        if self.__connection is not None:
            return [*self.__connection.agents.values()]
        else:
            return []

    @property
    def agent_list_changed(self) -> Event:
        """
        Event that signals when the list of agents has changed.

        :rtype: Event
        """
        return self.__connection.agent_list_changed

    def __init_logger(self, name: str, log_level: int = logging.INFO) -> logging.Logger:
        """
        Initialize the logger with the desired log level.

        :param name: name of the logger
        :type name: str

        :param log_level: log level of the system logger, defaults to logging.INFO
        :type log_level: int, optional

        :return: configured logger
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
        Create a new connection using the specified serial port.

        :param port: serial port to attempt connection on
        :type port: str

        :param baudrate: serial connection baudrate
        :type baudrate: int

        :param source_system: system ID for the source system, defaults to 255
        :type source_system: int, optional

        :param source_component: component ID for the source system, defaults to 0
        :type source_component: int, optional

        :return: flag indicating whether connection was successful
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
                    "specified agent",
                    exc_info=True,
                )

                return False

        return True

    def disconnect(self) -> bool:
        """
        Disconnect from the network.

        :return: flag indicating whether the disconnect attempt was successful
        :rtype: bool
        """
        if self.__connection is not None:
            self.__connection.disconnect()
            self.__connection = None

        return True

    def send_msg(self, msg: Any) -> None:
        """
        Send a single message to an agent.

        :param msg: message to send
        :type msgs: pymavswarm message
        """
        if (msg.target_system, msg.target_comp) in self.__connection.agents:
            self.__connection.send_msg_handler(msg)

        return

    def send_msg_package(self, package: MsgPackage) -> None:
        """
        Send a message package.

        :param package: message package that should be sent
        :type package: MsgPackage
        """
        # Warn the user if there are any messages that are intended for non-existent
        # agents in the swarm
        for msg in package.msgs:
            if (msg.target_system, msg.target_comp) not in self.__connection.agents:
                self.__logger.warning(
                    f"Agent ({msg.target_system, msg.target_comp}) targeted by the "
                    "message package does not exist in the swarm."
                )

        self.__connection.send_msg_package_handler(package)

        return

    def set_param(self, param: Parameter) -> None:
        """
        Add the params to the connection's outgoing parameter queue.

        :param param: parameter to set on an agent
        :type params: Parameter
        """
        if (param.sys_id, param.comp_id) in self.__connection.agents:
            self.__connection.set_param_handler(param)

        return

    def set_param_package(self, package: ParameterPackage) -> None:
        """
        Send a parameter package.

        :param package: parameter package to send, note that all parameters in the
            package should be intended to set a parameter on an agent
        :type package: ParameterPackage
        """
        for param in package.params:
            if (param.sys_id, param.comp_id) not in self.__connection.agents:
                self.__logger.warning(
                    f"Agent ({param.sys_id}, {param.comp_id}) targeted by the"
                    "parameter package does not exist in the swarm."
                )

        return

    def read_param(self, param: Parameter) -> None:
        """
        Read a desired parameter value. Note that, in the current configuration,
        each agent stores the most recent 5 parameter values read in a circular
        buffer. Therefore, when performing a bulk parameter read, ensure that
        a maximum of five parameters are read at once on the specific agent.

        :param params: parameter to read from an agent
        :type params: Parameter
        """
        if (param.sys_id, param.comp_id) in self.__connection.agents:
            self.__connection.read_param_handler(param)

        return

    def get_agent_by_id(self, sys_id: int, comp_id: int) -> Agent:
        """
        Get a specific agent by its system ID and component ID.

        :param sys_id: system ID of the agent to access
        :type sys_id: int

        :param comp_id: component ID of the agent to access
        :type comp_id: int

        :return: identified agent, if found
        :rtype: Optional[Agent]
        """
        agent_id = (sys_id, comp_id)

        if agent_id in self.__connection.agents:
            return self.__connection.agents[agent_id]

        return None

    def get_agent_by_name(self, name: str) -> Agent:
        """
        Get the first agent in the swarm with the specified name.

        :param name: name of the agent to access
        :type name: str

        :return: first agent identified with the given name
        :rtype: Optional[Agent]
        """
        for agent in self.__connection.agents.values():
            if agent.name == name:
                return agent

        return None
