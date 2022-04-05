from typing import Any
from .State import State


class Common(State):
    """
    Common state object used to provide observer capabilities to basic types
    """

    def __init__(self, value: Any, name: str, callbacks: list = []) -> None:
        """
        :param value: Value to initialize the state as
        :type value: Any

        :param name: The name of the state value to provide when getting the current
            state
        :type name: str

        :param callbacks: Observers to call on state change, defaults to []
        :type callbacks: list, optional
        """
        super().__init__(callbacks)

        self.__value = value
        self.__name = name

        return

    def get_current_state(self) -> dict:
        """
        Get the current state as a dictionary for callbacks

        :return: Current value of the state
        :rtype: dict
        """
        return {self.__name: self.id}

    @property
    def value(self) -> int:
        """
        The current value of the state property

        :rtype: int
        """
        return self.__value

    @value.setter
    def value(self, value: int) -> None:
        """
        value setter

        :param value: The desired state value
        :type value: int
        """
        self.__value = value

        for cb in self.callbacks:
            cb(self.get_current_state())

        return
