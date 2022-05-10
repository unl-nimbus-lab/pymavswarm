from typing import Any
from pymavswarm.state.State import State


class Common(State):
    """
    Common state object used to provide observer capabilities to basic types
    """

    def __init__(
        self, value: Any, name: str, optional_context_props: dict = {}
    ) -> None:
        """
        :param value: Value to initialize the state as
        :type value: Any

        :param name: The name of the state value to provide when getting the context
        :type name: str

        :param optional_context_props: Optional properties to add to the context
        :type optional_context_props: dict, optional
        """
        super().__init__()

        self.__value = value
        self.__name = name
        self.__optional_context_props = optional_context_props

        return

    @property
    def context(self) -> dict:
        """
        Get the current state as a dictionary for callbacks

        :return: Current value of the state
        :rtype: dict
        """
        context = {self.__name: self.__value}
        context.update(self.__optional_context_props)

        return context

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
        prev_value = self.__value
        self.__value = value

        # Signal state change event
        if self.__value != prev_value:
            self.state_changed_event.notify(context=self.context)

        return

    @property
    def name(self) -> str:
        """
        The name associated with the state to provide when retrieving the context

        :rtype: str
        """
        return self.__name
