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

from pymavswarm.event import Event
from pymavswarm.param import Parameter


class ParameterPackage:
    """
    Wrapper for multiple messages that allows for verification of parameter reception
    for multiple parameters.
    """

    def __init__(
        self,
        params: list,
        retry: bool = False,
        max_retry_attempts: int = 2,
        optional_context_props: dict = {},
    ) -> None:
        """
        Constructor.

        :param params: list of parameters to set/read
        :type params: list

        :param retry: flag indicating whether or not to retry sending failed parameters,
            defaults to False
        :type retry: bool, optional

        :param max_retry_attempts: maximum number of attempts that should be made when
            to retry sending failed parameters, defaults to 2
        :type max_retry_attempts: int, optional

        :param optional_context_props: optional properties to append to the package
            context, defaults to {}
        :type optional_context_props: dict, optional
        """
        self.__params = params
        self.__retry = retry
        self.__max_retry_attempts = max_retry_attempts
        self.__params_succeeded = []
        self.__params_failed = []
        self.__package_result_event = Event()
        self.__optional_context_props = optional_context_props
        self.__response = None

        return

    @property
    def params(self) -> List[Parameter]:
        """
        List of parameters to set/read.

        :rtype: list
        """
        return self.__params

    @property
    def retry(self) -> bool:
        """
        Retry sending the parameters that failed.

        :rtype: bool
        """
        return self.__retry

    @property
    def params_succeeded(self) -> list:
        """
        List of parameters that were successfully sent/read.

        :rtype: list
        """
        return self.__params_succeeded

    @property
    def params_failed(self) -> list:
        """
        List of the parameters that failed to be sent/read.

        :rtype: list
        """
        return self.__params_failed

    @property
    def max_retry_attempts(self) -> int:
        """
        Maximum number of attempts made to retry sending any failed parameters.

        :rtype: int
        """
        return self.__max_retry_attempts

    @property
    def package_result_event(self) -> Event:
        """
        Event indicating the result of the package.

        :rtype: Event
        """
        return self.__package_result_event

    @property
    def response(self) -> int:
        """
        The package response (e.g., SUCCESS)

        :rtype: int
        """
        return self.__response

    @response.setter
    def response(self, code: int) -> None:
        """
        response setter.

        :param code: response code
        :type code: int
        """
        self.__response = code
        return

    @property
    def context(self) -> dict:
        """
        Current context of the package.

        :rtype: dict
        """
        context = {
            "params_succeeded": self.__params_succeeded,
            "params_failed": self.__params_failed,
            "response": self.__response,
        }
        context.update(self.__optional_context_props)

        return context
