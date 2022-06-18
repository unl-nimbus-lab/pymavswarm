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

from pymavswarm.utils import Event


class State:
    """
    Parent class used to provide relationship between state objects
    """

    def __init__(self) -> None:
        self.__state_changed_event = Event()

        return

    @property
    def state_changed_event(self) -> Event:
        """
        Indicates that the current property values of a state class have changed

        :rtype: Event
        """
        return self.__state_changed_event

    @property
    def context(self) -> dict:
        """
        Get the current properties of a state object

        :rtype: dict
        """
        raise NotImplementedError("This method has not been implemented")
