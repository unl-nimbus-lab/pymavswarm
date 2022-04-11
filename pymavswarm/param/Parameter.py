from typing import Union, Optional


class Parameter:
    """
    A key/value pair that may be sent to an agent to read/write a parameter value
    """

    def __init__(
        self,
        sys_id: int,
        comp_id: int,
        param_id: str,
        retry: bool,
        msg_timeout: float = 3.0,
        param_value: Optional[Union[float, int]] = None,
        ack_timeout: float = 1.0,
        callbacks: list = [],
    ) -> None:
        """
        :param sys_id: The system ID of the agent whose parameters should be set/read
        :type sys_id: int

        :param comp_id: The component ID of the agent whose parameters should be
            set/read
        :type comp_id: int

        :param param_id: The parameter ID that should be set/read
        :type param_id: str

        :param retry: Flag indicating whether the system should retry message sending
            should acknowledgment fail
        :type retry: bool

        :param msg_timeout: The amount of time that pymavswarm should attempt to resend
            a message if acknowledgement is not received. This is only used when
            retry is set to true, defaults to 3.0
        :type msg_timeout: float, optional

        :param param_value: The value that the parameter should be set to, defaults to
            None
        :type param_value: Optional[Union[float, int]], optional

        :param ack_timeout: _description_, defaults to 1.0
        :type ack_timeout: float, optional

        :param callbacks: _description_, defaults to []
        :type callbacks: list, optional
        """
        self.__sys_id = sys_id
        self.__comp_id = comp_id
        self.__param_id = param_id
        self.__param_value = param_value
        self.__retry = retry
        self.__msg_timeout = msg_timeout
        self.__ack_timeout = ack_timeout
        self.__callbacks = callbacks

        return

    def add_param_callback(self, fn) -> None:
        self.__callbacks.append(fn)
        return

    def remove_param_callback(self, fn) -> None:
        if fn in self.__callbacks:
            self.__callbacks.remove(fn)
        return

    @property
    def sys_id(self) -> int:
        """
        The system ID of the agent whose param value should be set/read

        :rtype: int
        """
        return self.__sys_id

    @property
    def comp_id(self) -> int:
        """
        The component ID of the agent whose param value should be set/read

        :rtype: int
        """
        return self.__comp_id

    @property
    def param_id(self) -> str:
        """
        The ID of the parameter that should be set/read on an agent

        :rtype: str
        """
        return self.__param_id

    @property
    def param_value(self) -> Union[float, int]:
        """
        The value that the parameter should be set to

        :rtype: Union[float, int]
        """
        return self.__param_value

    @property
    def retry(self) -> bool:
        """
        Flag indicating whether the system should re-attempt parameter setting/reading
        if the system fails to acknowledge it

        :rtype: bool
        """
        return self.__retry

    @retry.setter
    def retry(self, retry: bool) -> None:
        """
        retry setter

        :param retry: Flag
        :type retry: bool
        """
        self.__retry = retry
        return

    @property
    def msg_timeout(self) -> float:
        """
        The period of time that pymavswarm should attempt to re-set a parameter
        if acknowledgement fails (parameter setting only)

        :rtype: float
        """
        return self.__msg_timeout

    @property
    def ack_timeout(self) -> float:
        """
        The amount of time that pymavswarm should wait for acknowledgement before
        considering that the system failed to acknowledge the parameter setting/reading

        :rtype: float
        """
        return self.__ack_timeout
