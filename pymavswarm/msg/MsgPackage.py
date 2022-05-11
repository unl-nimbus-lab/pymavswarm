from pymavswarm.event import Event


class MsgPackage:
    """
    Wrapper for multiple messages that allows for verification of a group of messages
    rather than single messages
    """

    def __init__(self, msgs: list, retry: bool = False) -> None:
        self.__msgs = msgs
        self.__retry = retry
        self.__msgs_succeeded = []
        self.__msgs_failed = []
        self.__package_result_event = Event()

        return

    @property
    def context(self) -> dict:
        return {
            "msgs_succeeded": self.__msgs_succeeded,
            "msgs_failed": self.__msgs_failed,
        }

    @property
    def msgs(self) -> list:
        """
        The list of messages in the package

        :rtype: list
        """
        return self.__msgs

    @property
    def retry(self) -> bool:
        """
        Retry sending the messages that failed

        :rtype: bool
        """
        return self.__retry

    @property
    def msgs_succeeded(self) -> list:
        """
        The list of msgs in the package that were successfully sent

        :rtype: list
        """
        return self.__msgs_succeeded

    @property
    def msgs_failed(self) -> list:
        """
        The list of msgs in the package that were not successfully sent

        :rtype: list
        """
        return self.__msgs_failed

    @property
    def package_result_event(self) -> Event:
        """
        Event indicating the result of the package

        :rtype: Event
        """
        return self.__package_result_event
