from collections import deque
from pymavswarm.state import ReadParameter, State


class ParameterList(State):
    """
    List of parameters implemented as a circular buffer
    """

    def __init__(self, max_length: int, optional_context_props: dict = {}) -> None:
        """
        :param max_length: The maximum length of the circular buffer
        :type max_length: int

        :param optional_context_props: Optional context properties, defaults to {}
        :type optional_context_props: dict, optional
        """
        super().__init__()

        self.__max_length = max_length
        self.__params = deque(maxlen=max_length)
        self.__optional_context_props = optional_context_props

        return

    @property
    def context(self) -> dict:
        """
        The current state of the parameter buffer

        :rtype: dict
        """
        context = {"parameters": self.__params}
        context.update(self.__optional_context_props)

        return context

    @property
    def max_length(self) -> int:
        """
        Maximum length of the circular buffer

        :rtype: int
        """
        return self.__max_length

    def append(self, item: ReadParameter) -> None:
        """
        Add a parameter to the parameter buffer

        :param item: The parameter to add to the buffer
        :type item: Parameter
        """
        self.__params.append(item)
        self.state_changed_event.notify(context=self.context)

        return

    def remove(self, item: ReadParameter) -> None:
        """
        Remove a parameter from the buffer

        :param item: _description_
        :type item: Parameter
        """
        self.__params.remove(item)
        self.state_changed_event.notify(context=self.context)

        return

    def clear(self) -> None:
        """
        Clear all items from the parameter buffer
        """
        self.__params.clear()
        self.state_changed_event.notify(context=self.context)

        return

    def extend(self, params: list) -> None:
        """
        Extend the parameter buffer to include the iterable collection of parameters

        :param params: List of parameters to add to the parameter buffer
        :type params: list
        """
        self.__params.extend(params)
        self.state_changed_event.notify(context=self.context)

        return
