from typing import Any, Callable


class Event:
    """
    Interface used to represent events and signal event listeners.
    """

    def __init__(self) -> None:
        """
        Constructor.
        """
        self.__listeners = []

        return

    @property
    def listeners(self) -> list:
        """
        The list of methods that are called on the event

        :rtype: list
        """
        return self.__listeners

    def add_listener(self, function: Callable) -> None:
        """
        Add a listener to the listeners

        :param fn: Function to call on event
        :type fn: function
        """
        self.__listeners.append(function)

        return

    def remove_listener(self, function: Callable) -> None:
        """
        Remove a listener

        :param fn: Function to remove
        :type fn: function
        """
        if function in self.__listeners:
            self.__listeners.remove(function)

        return

    def notify(self, **kwargs: Any) -> None:
        """
        Notify all listeners that the event was triggered
        """
        for listener in self.__listeners:
            if len(kwargs) > 0:
                listener(kwargs)
            else:
                listener()

        return
