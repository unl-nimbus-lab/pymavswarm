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

from typing import Any


class MessageWrapper:
    """Parent class used to construct MAVLink messages."""

    def __init__(
        self,
        target_system: int,
        target_component: int,
        mavlink_message: Any,
        message_type: str,
        retry: bool,
        message_timeout: float = 5.0,
        ack_timeout: float = 1.0,
        state_timeout: float = 5.0,
        state_delay: float = 3.0,
    ) -> None:
        """
        Create a MAVLink message wrapper.

        :param target_system: target system ID
        :type target_system: int
        :param target_component: target component ID
        :type target_component: int
        :param retry: flag indicating whether to retry sending until timeout/success
        :type retry: bool
        :param message_timeout: max time to successfully send a message, defaults to
            5.0 [s]
        :type message_timeout: float, optional
        :param ack_timeout: max time to wait for an acknowledgement, defaults to 1.0 [s]
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
        """
        self.__target_system = target_system
        self.__target_component = target_component
        self.__mavlink_message = mavlink_message
        self.__message_type = message_type
        self.__retry = retry
        self.__msg_timeout = message_timeout
        self.__ack_timeout = ack_timeout
        self.__state_timeout = state_timeout
        self.__state_delay = state_delay

        return

    @property
    def target_system(self) -> int:
        """
        Target system ID.

        :return: system ID
        :rtype: int
        """
        return self.__target_system

    @property
    def target_component(self) -> int:
        """
        Target component ID.

        :return: component ID
        :rtype: int
        """
        return self.__target_component

    @property
    def mavlink_message(self) -> Any:
        """
        MAVLink message to send.

        :return: MAVLink message
        :rtype: Any
        """
        return self.__mavlink_message

    @property
    def message_type(self) -> Any:
        """
        MAVLink message type.

        Used within the message Response for debugging.

        :return: message type
        :rtype: Any
        """
        return self.__message_type

    @property
    def retry(self) -> bool:
        """
        Flag indicating whether to retry sending until timeout/success.

        :return: flag
        :rtype: bool
        """
        return self.__retry

    @retry.setter
    def retry(self, retry: bool) -> None:
        """
        Set the retry flag.

        :param retry: flag
        :type retry: bool
        """
        self.__retry = retry
        return

    @property
    def message_timeout(self) -> float:
        """
        Max time taken to successfully send a message.

        :return: message timeout
        :rtype: float
        """
        return self.__msg_timeout

    @property
    def ack_timeout(self) -> float:
        """
        Max time allowed to acknowledge a message.

        :return: acknowledgment timeout
        :rtype: float
        """
        return self.__ack_timeout

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
