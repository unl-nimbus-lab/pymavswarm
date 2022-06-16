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

"""Base class for a swarm agent command."""

from typing import Optional

from pymavswarm.messages.message import Message


class AgentCommand(Message):
    """Parent class used to construct MAVLink commands."""

    def __init__(
        self,
        message_type: str,
        target_system: int,
        target_component: int,
        retry: bool,
        message_timeout: float = 5.0,
        ack_timeout: float = 1.0,
        state_timeout: float = 5.0,
        state_delay: float = 3.0,
        optional_context_props: Optional[dict] = None,
    ) -> None:
        """
        Create a new agent command.

        :param msg_type: sub-message type for a message
        :type msg_type: str

        :param target_system: target system ID
        :type target_system: int

        :param target_component: target component ID
        :type target_component: int

        :param retry: indicate whether pymavswarm should retry sending the message
            until acknowledgement
        :type retry: bool

        :param msg_timeout: amount of time that pymavswarm should attempt to resend
            a message if acknowledgement is not received. This is only used when
            retry is set to true, defaults to 5.0
        :type msg_timeout: float, optional

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
        :type optional_context_props: dict, optional
        """
        super().__init__(
            target_system,
            target_component,
            retry,
            message_timeout,
            ack_timeout,
            optional_context_props,
        )
        self.__msg_type = message_type

        if state_timeout < 0.0 or state_delay < 0.0:
            raise ValueError(
                "An invalid timeout or delay was provided. Ensure that "
                "all timeouts and delays are non-negative"
            )

        self.__state_timeout = state_timeout
        self.__state_delay = state_delay

        return

    @property
    def message_type(self) -> str:
        """
        Type of message.

        :return: message type
        :rtype: str
        """
        return self.__msg_type

    @message_type.setter
    def message_type(self, msg_type: str) -> None:
        """
        Set the message type.

        :param msg_type: Message type
        :type msg_type: str
        """
        self.__msg_type = msg_type
        return

    @property
    def state_timeout(self) -> float:
        """
        Agent state verification timout.

        Maximum amount of time that pymavswarm should wait for a given agent's
        state to change after receiving a mavlink message

        :return: state timeout
        :rtype: float
        """
        return self.__state_timeout

    @property
    def state_delay(self) -> float:
        """
        Delay between sequence commands.

        Amount of time that pymavswarm should wait after sending a command
        prior to sending another command. This parameter is used for sequence-driven
        commands such as the full takeoff command sequence.

        :return: state delay
        :rtype: float
        """
        return self.__state_delay

    @property
    def context(self) -> dict:
        """
        Message context.

        :return: Dictionary with the message context
        :rtype: dict
        """
        context = super().context

        # Update the message context with the base command context
        context["message_type"] = self.__msg_type
        context["state_timeout"] = self.__state_timeout
        context["state_delay"] = self.__state_delay

        return context
