from typing import Union


class Parameter:
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
