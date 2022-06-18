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
from typing import Optional, Union

from pymavswarm.handlers import MessageSenders
from pymavswarm.messages.agent_message import AgentMessage


class Parameter(AgentMessage):
    """Key/value pair that may be sent to an agent to read/write a parameter value."""

    def __init__(
        self,
        parameter_id: str,
        target_system: int,
        target_component: int,
        retry: bool,
        parameter_value: Optional[Union[float, int]] = None,
        message_timeout: float = 5,
        ack_timeout: float = 1,
        optional_context_props: Optional[dict] = None,
    ) -> None:
        """
        Create a new parameter.

        :param parameter_id: id of the parameter to read/write
        :type parameter_id: str

        :param target_system: target system ID
        :type target_system: int

        :param target_component: target component ID
        :type target_component: int

        :param retry: retry sending the message on failure
        :type retry: bool

        :param parameter_value: desired parameter value when setting a parameter,
            defaults to None
        :type parameter_value: Optional[Union[float, int]], optional

        :param message_timeout: The amount of time that pymavswarm should attempt to
            resend a message if acknowledgement is not received. This is only used when
            retry is set to true, defaults to 5.0
        :type message_timeout: float, optional

        :param ack_timeout: The amount of time that pymavswarm should wait to check for
            an acknowledgement from an agent. This is only used when retry is set
            to true. This should be kept as short as possible to keep agent state
            information up-to-date, defaults to 1.0
        :type ack_timeout: float, optional

        :param optional_context_props: optional properties to append to the message
            context, defaults to None
        :type optional_context_props: Optional[dict], optional
        """
        if parameter_value is None:
            super().__init__(
                MessageSenders.READ_PARAMETER,
                target_system,
                target_component,
                retry,
                message_timeout,
                ack_timeout,
                optional_context_props=optional_context_props,
            )
        else:
            super().__init__(
                MessageSenders.SET_PARAMETER,
                target_system,
                target_component,
                retry,
                message_timeout,
                ack_timeout,
                optional_context_props=optional_context_props,
            )

        self.__parameter_id = parameter_id
        self.__parameter_value = parameter_value

        return

    @property
    def parameter_id(self) -> str:
        """
        ID of the parameter that should be set/read on an agent.

        :return: parameter ID
        :rtype: str
        """
        return self.__parameter_id

    @property
    def parameter_value(self) -> Optional[Union[float, int]]:
        """
        Value that the parameter should be set to.

        :return: desired parameter value
        :rtype: Union[float, int]
        """
        return self.__parameter_value

    @property
    def context(self) -> dict:
        """
        Context of the parameter when an event occurred.

        :rtype: dict
        """
        context = super().context

        context["parameter_id"] = self.__parameter_id
        context["parameter_value"] = self.__parameter_value

        return context
