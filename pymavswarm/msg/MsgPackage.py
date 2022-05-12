from pymavswarm.event import Event


class MsgPackage:
    """
    Wrapper for multiple messages that allows for verification of a group of messages
    rather than single messages
    """

    def __init__(
        self, msgs: list, retry: bool = False, max_retry_attempts: int = 2
    ) -> None:
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
        return {
            "msgs_succeeded": self.__msgs_succeeded,
            "msgs_failed": self.__msgs_failed,
            "response": self.__response,
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
    def max_retry_attempts(self) -> int:
        """
        The maximum number of attempts to retry sending any failed messages in a
        package before considering the package failed

        :rtype: int
        """
        return self.__max_retry_attempts

    @property
    def package_result_event(self) -> Event:
        """
        Event indicating the result of the package

        :rtype: Event
        """
        return self.__package_result_event

    @property
    def response(self) -> int:
        """
        The package response (e.g., SUCCESS)

        :rtype: int
        """
        return self.__response

    @response.setter
    def response(self, code: int) -> None:
        """
        response setter

        :param code: The response code
        :type code: int
        """
        self.__response = code
        return
