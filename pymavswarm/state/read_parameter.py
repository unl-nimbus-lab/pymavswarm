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

from typing import Union


class ReadParameter:
    """
    A parameter that has been read from an agent
    """

    def __init__(
        self,
        param_id: str,
        param_value: Union[float, int],
        param_type: int,
        param_index: int,
        param_count: int,
    ) -> None:
        """
        :param param_id: The parameter ID
        :type param_id: str

        :param param_value: The parameter value
        :type param_value: Union[float, int]

        :param param_type: The type of value that the parameter may contain (Should be
            6: float or int)
        :type param_type: int

        :param param_index: The index of the parameter
        :type param_index: int

        :param param_count: The parameter count
        :type param_count: int
        """
        self.__param_id = param_id
        self.__param_value = param_value
        self.__param_type = param_type
        self.__param_index = param_index
        self.__param_count = param_count

        return

    @property
    def param_id(self) -> str:
        """
        The parameter ID

        :rtype: str
        """
        return self.__param_id

    @property
    def param_value(self) -> Union[float, int]:
        """
        The parameter value

        :rtype: Union[float, int]
        """
        return self.__param_value

    @property
    def param_type(self) -> int:
        """
        The type of value that the parameter may contain (Should be 6: float or int)

        :rtype: int
        """
        return self.__param_type

    @property
    def param_index(self) -> int:
        """
        The index of the parameter

        :rtype: int
        """
        return self.__param_index

    @property
    def param_count(self) -> int:
        """
        The parameter count

        :rtype: int
        """
        return self.__param_count
