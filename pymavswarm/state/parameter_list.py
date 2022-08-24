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

from __future__ import annotations

from collections import deque

from pymavswarm.state.parameter import Parameter
from pymavswarm.state.state import State


class ParameterList(State):
    """
    List of parameters read from an agent.

    Implemented as a circular buffer.
    """

    def __init__(
        self, max_length: int, optional_context_props: dict | None = None
    ) -> None:
        """
        Create a new list of read parameters.

        :param max_length: maximum length of the circular buffer
        :type max_length: int
        :param optional_context_props: optional context properties, defaults to None
        :type optional_context_props: dict | None, optional
        """
        super().__init__(optional_context_props)

        self.__max_length = max_length
        self.__params: deque = deque(maxlen=max_length)

        return

    @property
    def parameters(self) -> list[Parameter]:
        """
        List of stored parameters.

        :return: stored parameters
        :rtype: List[Parameter]
        """
        return [*self.__params]

    @property
    def max_length(self) -> int:
        """
        Maximum length of the circular buffer.

        :return: length
        :rtype: int
        """
        return self.__max_length

    @property
    def context(self) -> dict:
        """
        Parameter list context.

        :return: context
        :rtype: dict
        """
        context = super().context
        context["parameters"] = self.parameters

        return context

    def append(self, item: Parameter) -> None:
        """
        Add a parameter to the buffer.

        :param item: parameter to add to the buffer
        :type item: Parameter
        """
        self.__params.append(item)
        self.state_changed_event.notify(**self.context)

        return

    def remove(self, item: Parameter) -> None:
        """
        Remove a parameter from the buffer.

        :param item: parameter to remove
        :type item: Parameter
        """
        self.__params.remove(item)
        self.state_changed_event.notify(**self.context)

        return

    def clear(self) -> None:
        """Clear all items from the parameter buffer."""
        self.__params.clear()
        self.state_changed_event.notify(**self.context)

        return

    def extend(self, params: list[Parameter]) -> None:
        """
        Extend the parameter buffer to include the iterable collection of parameters.

        :param params: list of parameters to add to the parameter buffer
        :type params: list
        """
        self.__params.extend(params)
        self.state_changed_event.notify(**self.context)

        return

    def __str__(self) -> str:
        """
        Print parameter list in a human-readable format.

        :return: parameters
        :rtype: str
        """
        return f"ParameterList: {self.context}"
