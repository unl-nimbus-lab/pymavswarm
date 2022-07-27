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

from collections.abc import MutableMapping
from typing import Any, Iterator

from pymavswarm.utils import Event


class NotifierDict(MutableMapping):
    """
    Dictionary that notifies on add and del.

    Dictionary-like object implemented to enable calling events when elements are added
    and removed from the dictionary.

    This has been inspired by the following source:
    https://stackoverflow.com/questions/3387691/how-to-perfectly-override-a-dict
    """

    def __init__(self, event: Event, *args, **kwargs):
        """
        Create a NotifierDict.

        :param event: event to fire on value change
        :type event: Event
        """
        self.__store: dict[Any, Any] = {}
        self.__event = event
        self.update(dict(*args, **kwargs))

    def __getitem__(self, key: Any) -> Any:
        """
        Get an item from the dictionary.

        :param key: dictionary key
        :type key: Any
        :return: value associated with the key
        :rtype: Any
        """
        return self.__store[key]

    def __setitem__(self, key: Any, value: Any) -> None:
        """
        Set the key's value.

        :param key: dictionary key
        :type key: Any
        :param value: dictionary value
        :type value: Any
        """
        self.__event.notify(operation="set", key=key, value=value)
        self.__store[key] = value

        return

    def __delitem__(self, key: Any) -> None:
        """
        Delete the key's value.

        :param key: key whose value should be deleted
        :type key: Any
        """
        self.__event.notify(operation="del", key=key, value=self.__store[key])
        del self.__store[key]

        return

    def __iter__(self) -> Iterator:
        """
        Get iterator from dictionary.

        :return: dictionary iterator
        :rtype: Iterator
        """
        return iter(self.__store)

    def __len__(self) -> int:
        """
        Get the length of the dictionary.

        :return: dictionary length
        :rtype: int
        """
        return len(self.__store)
