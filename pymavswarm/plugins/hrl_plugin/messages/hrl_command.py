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

from typing import Optional

from pymavswarm.messages.agent_message import AgentMessage
from pymavswarm.plugins.hrl_plugin.hrl_senders import HrlSenders


class HRLCommand(AgentMessage):
    """Execute an HRL command."""

    START_PATH_EXECUTION = 0
    RESET_PATH_EXECUTION = 1
    STOP_PATH_EXECUTION = 2
    START_LIVE_EXECUTION = 3

    def __init__(
        self,
        hrl_command: int,
        target_system: int,
        target_component: int,
        retry: bool,
        message_timeout: float = 5,
        ack_timeout: float = 1,
        state_timeout: float = 5,
        state_delay: float = 3,
        optional_context_props: Optional[dict] = None,
    ) -> None:
        """
        Create an HRL command.

        :param hrl_command: HRL command to send
        :type hrl_command: int
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
        :param optional_context_props: optional properties to append to the message
            context, defaults to None
        :type optional_context_props: Optional[dict], optional
        """
        super().__init__(
            HrlSenders.HRL_COMMAND,
            target_system,
            target_component,
            retry,
            message_timeout,
            ack_timeout,
            state_timeout,
            state_delay,
            optional_context_props,
        )

        self.__hrl_command = hrl_command

        return

    @property
    def hrl_command(self) -> int:
        """
        Desired HRL swarm state.

        :return: HRL command
        :rtype: int
        """
        return self.__hrl_command

    @property
    def context(self) -> dict:
        """
        Message context.

        :return: message context
        :rtype: dict
        """
        context = super().context
        context["hrl_command"] = self.__hrl_command

        return context
