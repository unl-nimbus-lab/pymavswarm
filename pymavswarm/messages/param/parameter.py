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

from pymavswarm.messages.message import Message


class Parameter(Message):
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
        super().__init__(
            target_system,
            target_component,
            retry,
            message_timeout,
            ack_timeout,
            optional_context_props,
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
        The current context of the parameter when an event occurred

        :rtype: dict
        """
        context = {
            "sys_id": self.__sys_id,
            "comp_id": self.__comp_id,
            "param_id": self.__param_id,
            "param_value": self.__param_value,
            "retry": self.__retry,
            "msg_timeout": self.__msg_timeout,
            "ack_timeout": self.__ack_timeout,
        }
        context.update(self.__optional_context_props)

        return context
