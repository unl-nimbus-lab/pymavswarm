class AgentMsg:
    """
    Parent class used to construct MAVLink commands

    ..py:attribute:: msg_type
        The sub-message type for a message

    ..py:attribute:: target_system
        The target system ID

    ..py:attribute:: target_comp
        The target component ID

    ..py:attribute:: retry
        Indicate whether pymavswarm should retry sending the message
        until acknowledgement

    ..py:attribute:: msg_timeout
        The amount of time that pymavswarm should attempt to resend
        a message if acknowledgement is not received. This is only used when check_ack
        is set to true.

    ..py:attribute:: ack_timeout
        The amount of time that pymavswarm should wait to check for an
        acknowledgement from an agent. This is only used when check_ack is set to true.
        This should be kept as short as possible to keep agent state information
        up-to-date.

    ..py:attribute:: state_timeout
        The amount of time that pymavswarm should wait for a given agent's
        state to change after receiving a mavlink message

    ..py:attribute:: state_delay
        The amount of time that pymavswarm should wait after sending a command
        prior to sending another command. This parameter is used for sequence-driven
        commands such as the full takeoff command sequence.

    ..py:attribute:: validate_state
        Flag indicating that pymavswarm should check to ensure that the
        message caused the desired state change in the system
    """

    def __init__(
        self,
        msg_type: str,
        target_system: int,
        target_comp: int,
        retry: bool,
        msg_timeout: float = 5.0,
        ack_timeout: float = 1.0,
        state_timeout: float = 5.0,
        state_delay: float = 3.0,
        validate_state: bool = False,
        callbacks: list = [],
    ) -> None:
        """
        :param msg_type: The sub-message type for a message
        :type msg_type: str

        :param target_system: The target system ID
        :type target_system: int

        :param target_comp: The target component ID
        :type target_comp: int

        :param retry: Indicate whether pymavswarm should retry sending the message
            until acknowledgement
        :type retry: bool

        :param msg_timeout: The amount of time that pymavswarm should attempt to resend
            a message if acknowledgement is not received. This is only used when
            check_ack is set to true.
        :type msg_timeout: float

        :param ack_timeout: The amount of time that pymavswarm should wait to check for
            an acknowledgement from an agent. This is only used when check_ack is set
            to true. This should be kept as short as possible to keep agent state
            information up-to-date.
        :type ack_timeout: float

        :param state_timeout: The amount of time that pymavswarm should wait for a
            given agent's state to change after receiving a mavlink message
        :type state_timeout: float

        :param state_delay: The amount of time that pymavswarm should wait after
            sending a command prior to sending another command. This parameter is used
            for sequence-driven commands such as the full takeoff command sequence.
        :type state_delay: float

        :param validate_state: Flag indicating that pymavswarm should check to ensure
            that the message caused the desired state change in the system
        :type validate_state: bool

        :param callbacks: List of methods that should be signaled on message events
        :type callbacks: list
        """
        self.__msg_type = msg_type
        self.__target_system = target_system
        self.__target_comp = target_comp
        self.__retry = retry
        self.__msg_timeout = msg_timeout
        self.__ack_timeout = ack_timeout
        self.__state_timeout = state_timeout
        self.__state_delay = state_delay
        self.__validate_state = validate_state
        self.__callbacks = callbacks

        return

    def add_message_callback(self, fn) -> None:
        self.__callbacks.append(fn)
        return

    def remove_message_callback(self, fn) -> None:
        if fn in self.__callbacks:
            self.__callbacks.remove(fn)
        return

    @property
    def msg_type(self) -> str:
        return self.__msg_type

    @property
    def target_system(self) -> int:
        return self.__target_system

    @property
    def target_comp(self) -> int:
        return self.__target_comp

    @property
    def retry(self) -> bool:
        return self.__retry

    @retry.setter
    def retry(self, retry: bool) -> None:
        self.__retry = retry
        return

    @property
    def msg_timeout(self) -> float:
        return self.__msg_timeout

    @property
    def ack_timeout(self) -> float:
        return self.__ack_timeout

    @property
    def state_timeout(self) -> float:
        return self.__state_timeout

    @property
    def state_delay(self) -> float:
        return self.__state_delay

    @property
    def validate_state(self) -> bool:
        return self.__validate_state
