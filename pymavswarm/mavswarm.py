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

"""User-level interface for swarm control."""

import logging
from typing import Any, Optional, Union

import pymavswarm.utils as swarm_utils
from pymavswarm import Connection
from pymavswarm.agent import Agent
from pymavswarm.messages import MessagePackage, SupportedCommands
from pymavswarm.param import Parameter, ParameterPackage
from pymavswarm.utils import Event


class MavSwarm:
    """
    User-facing pymavswarm interface.

    Provides an interface for controlling robot swarms, obtaining swarm state, and
    configuring agents.
    """

    def __init__(self, log_level: int = logging.INFO) -> None:
        """
        Construct MavSwarm interface.

        :param log_level: log level of the system, defaults to logging.INFO
        :type log_level: int, optional
        """
        super().__init__()

        self.__logger = swarm_utils.init_logger("mavswarm", log_level=log_level)
        self.__connection = Connection(log_level=log_level)

        return

    @property
    def supported_messages(self) -> SupportedCommands:
        """
        Messages supported by pymavswarm.

        :return: mapping to enable simple configuration of commands
        :rtype: SupportedMessages
        """
        return SupportedCommands()

    @property
    def agents(self) -> dict:
        """
        Agents in the swarm.

        Provided as a dictionary where the key is the (system ID, component ID) of the
        agent.

        :return: dictionary with swarm agents
        :rtype: dict
        """
        return self.__connection.agents

    @property
    def agents_as_list(self) -> list:
        """
        Get the swarm agents as a list.

        Provides the set of agents as a list of agents.

        :return: list of swarm agents
        :rtype: list
        """
        return [*self.__connection.agents.values()]

    @property
    def agent_list_changed(self) -> Event:
        """
        Event that signals when the list of agents has changed.

        Used to trigger functionality when the list of agents changes.

        :return: event signaling that the set of agents changed
        :rtype: Event
        """
        return self.__connection.agent_list_changed

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
        Connect to the network.

        Attempt to establish a MAVLink connection using the provided configurations.

        :param port: serial port to attempt connection on
        :type port: str

        :param baudrate: serial connection baudrate
        :type baudrate: int

        :param source_system: system ID for the source system, defaults to 255
        :type source_system: int, optional

        :param source_component: component ID for the source system, defaults to 0
        :type source_component: int, optional

        :param connection_attempt_timeout: maximum time taken to establish a connection,
            defaults to 2.0 [s]
        :type connection_attempt_timeout: float, optional

        :param agent_timeout: amount of time an agent has to send a message before
            being considered timed-out, defaults to 30.0 [s]
        :type agent_timeout: float, optional

        :return: flag indicating whether connection was successful
        :rtype: bool
        """
        if not self.__connection.connected:
            try:
                return self.__connection.connect(
                    port,
                    baudrate,
                    source_system=source_system,
                    source_component=source_component,
                    connection_attempt_timeout=connection_attempt_timeout,
                    agent_timeout=agent_timeout,
                )
            except Exception:
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

        Disconnect from the MAVLink network and reset the connection state.

        :return: flag indicating whether the disconnect attempt was successful
        :rtype: bool
        """
        if self.__connection.connected:
            self.__connection.disconnect()

        return True

    def send_message(self, message: Union[Any, MessagePackage]) -> bool:
        """
        Send a message to an agent.

        Send a message or message package to the target agent(s) over the MAVLink
        connection. If the target agent(s) are not in the network, the message will
        still be sent but may not be properly acknowledged or verified.

        :param message: message to send
        :type messages: pymavswarm message or package

        :return: whether or not the message was successfully sent
        :rtype: bool
        """
        # Notify the user if any messages will be sent to an agent that is not in the
        # list of agents.
        if isinstance(message, MessagePackage):
            for sub_message in message.messages:
                if (
                    sub_message.target_system,
                    sub_message.target_comp,
                ) not in self.__connection.agents:
                    self.__logger.warning(
                        "Agent (%s, %s) targeted by the message package does not "
                        "exist in the swarm.",
                        sub_message.target_system,
                        sub_message.target_comp,
                    )
        else:
            if (
                message.target_system,
                message.target_comp,
            ) not in self.__connection.agents:
                self.__logger.warning(
                    "Agent (%s, %s) targeted by the message does not exist in the "
                    "swarm.",
                    message.target_system,
                    message.target_comp,
                )

        return self.__connection.send_message(message)

    def set_param(self, param: Parameter) -> None:
        """
        Add the params to the connection's outgoing parameter queue.

        :param param: parameter to set on an agent
        :type params: Parameter
        """
        if (param.system_id, param.component_id) in self.__connection.agents:
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
            if (param.system_id, param.component_id) not in self.__connection.agents:
                self.__logger.warning(
                    f"Agent ({param.system_id}, {param.component_id}) targeted by the"
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
        if (param.system_id, param.component_id) in self.__connection.agents:
            self.__connection.read_param_handler(param)

        return

    def get_agent_by_id(self, system_id: int, component_id: int) -> Optional[Agent]:
        """
        Get an agent by its ID.

        Get the agent from the swarm that has the provided system ID and component ID.

        :param sys_id: system ID of the agent to access
        :type sys_id: int

        :param comp_id: component ID of the agent to access
        :type comp_id: int

        :return: identified agent, if found
        :rtype: Optional[Agent]
        """
        agent_id = (system_id, component_id)

        if agent_id not in self.__connection.agents:
            return None

        return self.__connection.agents[agent_id]

    def get_agent_by_name(self, name: str) -> Optional[Agent]:
        """
        Get an agent by its name.

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
