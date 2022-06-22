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

from typing import Optional, Tuple

from pymavswarm.utils import Event


class MessagePackage:
    """
    Collection of pymavswarm messages.

    Wrapper for multiple messages that allows for verification of a group of messages
    rather than single messages.
    """

    def __init__(
        self,
        messages: list,
        retry: bool = False,
        max_retry_attempts: int = 2,
        optional_context_props: Optional[dict] = None,
    ) -> None:
        """
        Create a message package.

        :param messages: list of messages that should be sent as part of the package
        :type messages: list
        :param retry: flag indicating whether the system should re-attempt sending the
            failed messages, defaults to False
        :type retry: bool, optional
        :param max_retry_attempts: The maximum number of attempts that should be made
            at successfully sending any previously failed messages, defaults to 2
        :type max_retry_attempts: int, optional
        :param optional_context_props: optional properties to append to the message
            context, defaults to None
        :type optional_context_props: Optional[dict], optional
        """
        self.__msgs = messages
        self.__retry = retry
        self.__msgs_succeeded: list = []
        self.__msgs_failed: list = []
        self.__max_retry_attempts = max_retry_attempts
        self.__package_result_event = Event()
        self.__optional_context_props = optional_context_props
        self.__response: Optional[Tuple[int, str]] = None

        return

    @property
    def messages(self) -> list:
        """
        List of messages in the package.

        :return: package messages
        :rtype: list
        """
        return self.__msgs

    @property
    def retry(self) -> bool:
        """
        Retry sending the messages that failed.

        :return: flag
        :rtype: bool
        """
        return self.__retry

    @property
    def messages_succeeded(self) -> list:
        """
        List of msgs in the package that were successfully sent.

        :return: successful messages
        :rtype: list
        """
        return self.__msgs_succeeded

    @property
    def messages_failed(self) -> list:
        """
        List of msgs in the package that were not successfully sent.

        :return: failed messages
        :rtype: list
        """
        return self.__msgs_failed

    @property
    def max_retry_attempts(self) -> int:
        """
        Maximum retry attempts.

        Maximum number of attempts to retry sending any failed messages in a
        package before considering the package failed.

        :return: maximum retry attempts
        :rtype: int
        """
        return self.__max_retry_attempts

    @property
    def package_result_event(self) -> Event:
        """
        Event indicating the result of the package.

        :return: result event
        :rtype: Event
        """
        return self.__package_result_event

    @property
    def response(self) -> Optional[Tuple[int, str]]:
        """
        Package response.

        :return: response
        :rtype: Optional[Tuple[int, str]]
        """
        return self.__response

    @response.setter
    def response(self, code: Optional[Tuple[int, str]]) -> None:
        """
        Set the package response.

        :param code: The response code
        :type code: Optional[Tuple[int, str]]
        """
        self.__response = code

        return

    @property
    def context(self) -> dict:
        """
        Context of the package.

        :return: package context
        :rtype: dict
        """
        context = {
            "msgs_succeeded": self.__msgs_succeeded,
            "msgs_failed": self.__msgs_failed,
            "response": self.__response,
        }

        if self.__optional_context_props is not None:
            context.update(self.__optional_context_props)

        return context
