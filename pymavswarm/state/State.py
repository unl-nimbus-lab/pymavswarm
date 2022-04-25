import sys

sys.path.append("../..")

from event import Event


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
