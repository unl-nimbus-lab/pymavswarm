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

from typing import Tuple


class Response:
    """Message response."""

    def __init__(
        self,
        target_system: int,
        target_component: int,
        message_type: str,
        result: bool,
        code: Tuple[int, str],
    ) -> None:
        """
        Create a message response.

        :param target_system: system ID of the target agent
        :type target_system: int
        :param target_component: component ID of the target agent
        :type target_component: int
        :param message_type: type of message sent
        :type message_type: str
        :param result: message result
        :type result: bool
        :param code: message code
        :type code: Tuple[int, str]
        """
        self.__target_system = target_system
        self.__target_component = target_component
        self.__message_type = message_type
        self.__result = result
        self.__code = code

        return

    @property
    def target_system(self) -> int:
        """
        System ID of the target agent.

        :return: system ID
        :rtype: int
        """
        return self.__target_system

    @property
    def target_component(self) -> int:
        """
        Component ID of the target agent.

        :return: component ID
        :rtype: int
        """
        return self.__target_component

    @property
    def message_type(self) -> str:
        """
        Type of message sent.

        :return: message type
        :rtype: str
        """
        return self.__message_type

    @property
    def result(self) -> bool:
        """
        Message result.

        :return: result
        :rtype: bool
        """
        return self.__result

    @property
    def code(self) -> Tuple[int, str]:
        """
        Message result code.

        :return: code
        :rtype: Tuple[int, str]
        """
        return self.__code