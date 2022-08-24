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

from typing import Any

from pymavswarm.state.state import State


class Generic(State):
    """Generic state object used to provide event capabilities to basic types."""

    def __init__(
        self, name: str, value: Any, optional_context_props: dict | None = None
    ) -> None:
        """
        Create a generic state object.

        :param name: name of the state value to provide when getting the context
        :type name: str
        :param value: value to initialize the state as
        :type value: Any
        :param optional_context_props: properties to add to the context, defaults to
            None
        :type optional_context_props: dict | None, optional
        """
        super().__init__(optional_context_props)

        self.__value = value
        self.__name = name

        return

    @property
    def value(self) -> Any:
        """
        Value of the state property.

        :return: state value
        :rtype: Any
        """
        return self.__value

    @value.setter
    def value(self, value: Any) -> None:
        """
        Set the state value.

        :param value: desired state value
        :type value: Any
        """
        prev_value = self.__value
        self.__value = value

        # Signal state change event
        if self.__value != prev_value:
            self.state_changed_event.notify(**self.context)

        return

    @property
    def name(self) -> str:
        """
        Name associated with the state to provide when retrieving the context.

        :return: name
        :rtype: str
        """
        return self.__name

    @property
    def context(self) -> dict:
        """
        State context.

        :return: context
        :rtype: dict
        """
        context = super().context

        context[self.__name] = self.__value

        return context

    def __str__(self) -> str:
        """
        Print state information in a human-readable format.

        :return: state information
        :rtype: str
        """
        return f"Generic: {self.context}"
