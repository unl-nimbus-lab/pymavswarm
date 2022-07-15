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

from typing import Any, Callable


class Event:
    """Interface used to represent events and signal event listeners."""

    def __init__(self) -> None:
        """Create an event."""
        self.__listeners: list[Callable] = []

        return

    @property
    def listeners(self) -> list:
        """
        List of methods that are called on the event.

        :return: callback functions
        :rtype: list
        """
        return self.__listeners

    def add_listener(self, function: Callable) -> None:
        """
        Add a listener to the list of listeners.

        :param fn: Function to call on event
        :type fn: function
        """
        self.__listeners.append(function)

        return

    def remove_listener(self, function: Callable) -> None:
        """
        Remove a listener from the list of listeners.

        :param fn: Function to remove
        :type fn: function
        """
        if function in self.__listeners:
            self.__listeners.remove(function)

        return

    def notify(self, *args, **kwargs: Any) -> None:
        """Notify all listeners that the event was triggered."""
        for listener in self.__listeners:
            if len(kwargs) > 0:
                listener(*args, **kwargs)
            else:
                listener()

        return
