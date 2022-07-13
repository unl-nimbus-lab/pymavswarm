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

from abc import ABC

from pymavswarm.utils import Event


class State(ABC):
    """Base state class."""

    def __init__(self, optional_context_props: dict | None = None) -> None:
        """Create a state object."""
        super().__init__()

        self.__state_changed_event = Event()
        self.__optional_context_props = optional_context_props

        return

    @property
    def state_changed_event(self) -> Event:
        """
        Event signaling state changes.

        :return: state change event
        :rtype: Event
        """
        return self.__state_changed_event

    @property
    def context(self) -> dict:
        """
        State context.

        :return: context
        :rtype: dict
        """
        context = {}

        if self.__optional_context_props is not None:
            context.update(self.__optional_context_props)

        return context
