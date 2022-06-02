from pymavswarm.event import Event


class MsgPackage:
    """
    Wrapper for multiple messages that allows for verification of a group of messages
    rather than single messages.
    """

    def __init__(
        self, msgs: list, retry: bool = False, max_retry_attempts: int = 2
    ) -> None:
        """
        Constructor.

        :param msgs: The list of messages that should be sent as part of the package
        :type msgs: list

        :param retry: Flag indicating whether the system should re-attempt sending the
            failed messages, defaults to False
        :type retry: bool, optional

        :param max_retry_attempts: The maximum number of attempts that should be made
            at successfully sending any previously failed messages, defaults to 2
        :type max_retry_attempts: int, optional
        """
        self.__msgs = msgs
        self.__retry = retry
        self.__msgs_succeeded = []
        self.__msgs_failed = []
        self.__max_retry_attempts = max_retry_attempts
        self.__package_result_event = Event()
        self.__response = None

        return

    @property
    def context(self) -> dict:
        """
        Current context of the package.

        :rtype: dict
        """
        return {
            "msgs_succeeded": self.__msgs_succeeded,
            "msgs_failed": self.__msgs_failed,
            "response": self.__response,
        }

    @property
    def msgs(self) -> list:
        """
        List of messages in the package.

        :rtype: list
        """
        return self.__msgs

    @property
    def retry(self) -> bool:
        """
        Retry sending the messages that failed.

        :rtype: bool
        """
        return self.__retry

    @property
    def msgs_succeeded(self) -> list:
        """
        List of msgs in the package that were successfully sent.

        :rtype: list
        """
        return self.__msgs_succeeded

    @property
    def msgs_failed(self) -> list:
        """
        List of msgs in the package that were not successfully sent.

        :rtype: list
        """
        return self.__msgs_failed

    @property
    def max_retry_attempts(self) -> int:
        """
        Maximum number of attempts to retry sending any failed messages in a
        package before considering the package failed.

        :rtype: int
        """
        return self.__max_retry_attempts

    @property
    def package_result_event(self) -> Event:
        """
        Event indicating the result of the package.

        :rtype: Event
        """
        return self.__package_result_event

    @property
    def response(self) -> int:
        """
        Package response (e.g., SUCCESS).

        :rtype: int
        """
        return self.__response

    @response.setter
    def response(self, code: int) -> None:
        """
        response setter.

        :param code: The response code
        :type code: int
        """
        self.__response = code
        return
