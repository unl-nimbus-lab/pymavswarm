from typing import Callable


class State:
    """
    Parent class used to provide relationship between state objects
    """

    def __init__(self, callbacks: list = []) -> None:
        """
        :param callbacks: State change observers, defaults to []
        :type callbacks: list, optional
        """
        self.callbacks = callbacks

        return

    def add_callback(self, fn: Callable) -> None:
        """
        Add a new observer callback

        :param fn: function to add to the list of observers
        :type fn: function
        """
        self.callbacks.append(fn)
        return

    def remove_callback(self, fn: Callable) -> None:
        """
        Remove a given function from the list of observers

        :param fn: Function to remove
        :type fn: function
        """
        if fn in self.callbacks:
            self.callbacks.remove(fn)
        return

    def get_current_state(self) -> dict:
        """
        Abstract method, get the current properties of a state object

        :rtype: dict
        """
        raise NotImplementedError
