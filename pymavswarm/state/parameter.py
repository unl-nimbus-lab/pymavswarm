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


class Parameter:
    """Parameter that has been read from an agent."""

    def __init__(
        self,
        parameter_id: str,
        parameter_value: float | int,
        parameter_type: int,
        parameter_index: int,
        parameter_count: int,
    ) -> None:
        """
        Create a new parameter.

        :param param_id: parameter ID
        :type param_id: str
        :param param_value: parameter value
        :type param_value: float | int
        :param param_type: type of value that the parameter may contain (should be
            6: float or int)
        :type param_type: int
        :param param_index: index of the parameter
        :type param_index: int
        :param param_count: parameter count
        :type param_count: int
        """
        self.__parameter_id = parameter_id
        self.__parameter_value = parameter_value
        self.__parameter_type = parameter_type
        self.__parameter_index = parameter_index
        self.__parameter_count = parameter_count

        return

    @property
    def parameter_id(self) -> str:
        """
        Parameter ID.

        :return: parameter ID
        :rtype: str
        """
        return self.__parameter_id

    @property
    def parameter_value(self) -> float | int:
        """
        Parameter value.

        :return: parameter value
        :rtype: float | int
        """
        return self.__parameter_value

    @property
    def parameter_type(self) -> int:
        """
        Type of value that the parameter may contain.

        Should be 6: float or int.

        :return: parameter type
        :rtype: int
        """
        return self.__parameter_type

    @property
    def parameter_index(self) -> int:
        """
        Index of the parameter.

        :return: parameter index
        :rtype: int
        """
        return self.__parameter_index

    @property
    def parameter_count(self) -> int:
        """
        Parameter count.

        :return: parameter count
        :rtype: int
        """
        return self.__parameter_count

    def __str__(self) -> str:
        """
        Print agent information in a human-readable format.

        :return: agent information
        :rtype: str
        """
        context = {
            "id": self.__parameter_id,
            "value": self.__parameter_value,
            "type": self.__parameter_type,
            "index": self.__parameter_index,
            "count": self.__parameter_count,
        }

        return f"Parameter: {context}"
