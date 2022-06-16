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

"""Wrapper for a robot parameter read/write request."""

from typing import Optional, Union

from pymavswarm.event import Event


class Parameter:
    """Key/value pair that may be sent to an agent to read/write a parameter value."""

    def __init__(
        self,
        system_id: int,
        component_id: int,
        parameter_id: str,
        retry: bool,
        parameter_value: Optional[Union[float, int]] = None,
        message_timeout: float = 3.0,
        acknowledgement_timeout: float = 1.0,
        optional_context_properties: Optional[dict] = None,
    ) -> None:
        """
        Create a parameter.

        :param system_id: system ID of the agent whose parameters should be set/read
        :type system_id: int

        :param component_id: component ID of the agent whose parameters should be
            set/read
        :type component_id: int

        :param parameter_id: parameter ID that should be set/read
        :type parameter_id: str

        :param retry: flag indicating whether the system should retry message sending
            should acknowledgment fail
        :type retry: bool

        :param parameter_value: value that the parameter should be set to, defaults to
            None
        :type parameter_value: Optional[Union[float, int]], optional

        :param message_timeout: amount of time that pymavswarm should attempt to resend
            a message if acknowledgement is not received. This is only used when
            retry is set to true, defaults to 3.0
        :type message_timeout: float, optional

        :param acknowledgement_timeout: amount of time that pymavswarm should wait for
            acknowledgement before considering that the system failed to acknowledge
            the parameter setting/reading, defaults to 1.0
        :type acknowledgement_timeout: float, optional

        :param optional_context_props: optional properties to append to the parameter
            context, defaults to None
        :type optional_context_properties: Optional[dict], optional
        """
        self.__sys_id = system_id
        self.__comp_id = component_id
        self.__param_id = parameter_id
        self.__retry = retry
        self.__param_value = parameter_value
        self.__optional_context_props = optional_context_properties

        if message_timeout < 0.0 or acknowledgement_timeout < 0.0:
            raise ValueError(
                "An invalid timeout or delay was provided. Ensure that "
                "all timeouts and delays are non-negative"
            )

        self.__msg_timeout = message_timeout
        self.__ack_timeout = acknowledgement_timeout
        self.__parameter_read_result_event = Event()
        self.__parameter_write_result_event = Event()

        return

    @property
    def system_id(self) -> int:
        """
        System ID of the agent whose param value should be set/read.

        :return: system ID
        :rtype: int
        """
        return self.__sys_id

    @property
    def component_id(self) -> int:
        """
        Component ID of the agent whose param value should be set/read.

        :return: component ID
        :rtype: int
        """
        return self.__comp_id

    @property
    def parameter_id(self) -> str:
        """
        ID of the parameter that should be set/read on an agent.

        :return: parameter ID
        :rtype: str
        """
        return self.__param_id

    @property
    def parameter_value(self) -> Optional[Union[float, int]]:
        """
        Value that the parameter should be set to.

        :return: desired parameter value
        :rtype: Union[float, int]
        """
        return self.__param_value

    @property
    def retry(self) -> bool:
        """
        Retry setting the parameter/reading the parameter if failure.

        :return: flag indicating whether to retry
        :rtype: bool
        """
        return self.__retry

    @retry.setter
    def retry(self, retry: bool) -> None:
        """
        Set the retry property.

        :param retry: Flag
        :type retry: bool
        """
        self.__retry = retry
        return

    @property
    def msg_timeout(self) -> float:
        """
        The period of time that pymavswarm should attempt to re-set a parameter
        if acknowledgement fails (parameter setting only)

        :rtype: float
        """
        return self.__msg_timeout

    @property
    def ack_timeout(self) -> float:
        """
        The amount of time that pymavswarm should wait for acknowledgement before
        considering that the system failed to acknowledge the parameter setting/reading

        :rtype: float
        """
        return self.__ack_timeout

    @property
    def parameter_read_result_event(self) -> Event:
        """
        Event signaling the result of a parameter read result

        :rtype: Event
        """
        return self.__parameter_read_result_event

    @property
    def parameter_write_result_event(self) -> Event:
        """
        Event signaling the result of a parameter write result

        :rtype: Event
        """
        return self.__parameter_write_result_event

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
