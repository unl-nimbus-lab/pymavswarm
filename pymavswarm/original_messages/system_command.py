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

from typing import List

from pymavswarm.handlers import MessageSenders
from pymavswarm.messages.agent_message import AgentMessage


class SystemCommand(AgentMessage):
    """Perform a system-level operation on an agent."""

    ARM = MessageSenders.ARM
    DISARM = MessageSenders.DISARM
    REBOOT = MessageSenders.REBOOT
    SHUTDOWN = MessageSenders.SHUTDOWN
    KILL = MessageSenders.KILL

    @classmethod
    def get_system_command_types(cls) -> List[str]:
        """
        Get the supported system commands.

        :return: system command types
        :rtype: List[str]
        """
        return [
            cls.ARM,
            cls.DISARM,
            cls.REBOOT,
            cls.SHUTDOWN,
            cls.KILL,
        ]

    def __init__(
        self,
        command: str,
        target_system: int,
        target_component: int,
        retry: bool,
        message_timeout: float = 5,
        ack_timeout: float = 1,
        state_timeout: float = 5,
        state_delay: float = 3,
    ) -> None:
        """
        Create a system command.

        :param command: system command
        :type command: str
        :param target_system: target system ID
        :type target_system: int
        :param target_component: target component ID
        :type target_component: int
        :param retry: indicate whether pymavswarm should retry sending the message
            until acknowledgement
        :type retry: bool
        :param message_timeout: amount of time that pymavswarm should attempt to resend
            a message if acknowledgement is not received. This is only used when
            retry is set to true, defaults to 5.0
        :type message_timeout: float, optional
        :param ack_timeout: amount of time that pymavswarm should wait to check for
            an acknowledgement from an agent. This is only used when retry is set
            to true. This should be kept as short as possible to keep agent state
            information up-to-date, defaults to 1.0
        :type ack_timeout: float, optional
        :param state_timeout: amount of time that pymavswarm should wait for a
            given agent's state to change after receiving a mavlink message, defaults
            to 5.0
        :type state_timeout: float, optional
        :param state_delay: amount of time that pymavswarm should wait after
            sending a command prior to sending another command. This parameter is used
            for sequence-driven commands such as the full takeoff command sequence,
            defaults to 3.0
        :type state_delay: float, optional
        :raises ValueError: invalid system command
        """
        if command not in SystemCommand.get_system_command_types():
            raise ValueError(
                f"{command} is not a supported system command. Supported system "
                f"commands include: {SystemCommand.get_system_command_types()}"
            )

        super().__init__(
            command,
            target_system,
            target_component,
            retry,
            message_timeout,
            ack_timeout,
            state_timeout,
            state_delay,
        )

        self.__command = command

        return

    @property
    def command(self) -> str:
        """
        System command to execute.

        :return: command
        :rtype: str
        """
        return self.__command
