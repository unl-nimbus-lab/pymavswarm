from pymavswarm.event import Event


class AgentMessage:
    """
    Parent class used to construct MAVLink commands.
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
        optional_context_props: dict = {},
    ) -> None:
        """
        Constructor.

        :param msg_type: sub-message type for a message
        :type msg_type: str

        :param target_system: target system ID
        :type target_system: int

        :param target_comp: target component ID
        :type target_comp: int

        :param retry: indicate whether pymavswarm should retry sending the message
            until acknowledgement
        :type retry: bool

        :param msg_timeout: amount of time that pymavswarm should attempt to resend
            a message if acknowledgement is not received. This is only used when
            retry is set to true, defaults to 5.0
        :type msg_timeout: float, optional

        :param ack_timeout: amount of time that pymavswarm should wait to check for
            an acknowledgement from an agent. This is only used when retry is set
            to true. This should be kept as short as possible to keep agent state
            information up-to-date, defaults to 1.0
        :type ack_timeout: float, optional

        :param state_timeout: amount of time that pymavswarm should wait for a
            given agent's state to change after receiving a mavlink message, defaults
            to 5.0
        :type state_timeout: float, optional

        :param state_delay: amount of time that pymavswarm should wait after
            sending a command prior to sending another command. This parameter is used
            for sequence-driven commands such as the full takeoff command sequence,
            defaults to 3.0
        :type state_delay: float, optional

        :param optional_context_props: optional properties to append to the message
            context, defaults to {}
        :type optional_context_props: dict, optional
        """
        self.__msg_type = msg_type
        self.__target_system = target_system
        self.__target_comp = target_comp
        self.__retry = retry
        self.__optional_context_props = optional_context_props

        if (
            msg_timeout < 0.0
            or ack_timeout < 0.0
            or state_timeout < 0.0
            or state_delay < 0.0
        ):
            raise ValueError(
                "An invalid timeout or delay was provided. Ensure that "
                "all timeouts and delays are non-negative"
            )

        self.__msg_timeout = msg_timeout
        self.__ack_timeout = ack_timeout
        self.__state_timeout = state_timeout
        self.__state_delay = state_delay
        self.__message_result_event = Event()
        self.__response = None

        return

    @property
    def msg_type(self) -> str:
        """
        Sub-message type for a message.

        :rtype: str
        """
        return self.__msg_type

    @msg_type.setter
    def msg_type(self, msg_type: str) -> None:
        """
        msg_type setter.

        :param msg_type: Message type
        :type msg_type: str
        """
        self.__msg_type = msg_type
        return

    @property
    def target_system(self) -> int:
        """
        Target system ID.

        :rtype: int
        """
        return self.__target_system

    @property
    def target_comp(self) -> int:
        """
        Target component ID.

        :rtype: int
        """
        return self.__target_comp

    @property
    def retry(self) -> bool:
        """
        Indicate whether pymavswarm should retry sending the message
        until acknowledgement.

        :rtype: bool
        """
        return self.__retry

    @retry.setter
    def retry(self, retry: bool) -> None:
        """
        retry setter.

        :param retry: Flag
        :type retry: bool
        """
        self.__retry = retry
        return

    @property
    def msg_timeout(self) -> float:
        """
        Amount of time that pymavswarm should attempt to resend
        a message if acknowledgement is not received. This is only used when retry
        is set to true.

        :rtype: float
        """
        return self.__msg_timeout

    @property
    def ack_timeout(self) -> float:
        """
        Amount of time that pymavswarm should wait to check for an
        acknowledgement from an agent. This is only used when retry is set to true.
        This should be kept as short as possible to keep agent state information
        up-to-date.

        :rtype: float
        """
        return self.__ack_timeout

    @property
    def state_timeout(self) -> float:
        """
        Amount of time that pymavswarm should wait for a given agent's
        state to change after receiving a mavlink message

        :rtype: float
        """
        return self.__state_timeout

    @property
    def state_delay(self) -> float:
        """
        Amount of time that pymavswarm should wait after sending a command
        prior to sending another command. This parameter is used for sequence-driven
        commands such as the full takeoff command sequence.

        :rtype: float
        """
        return self.__state_delay

    @property
    def message_result_event(self) -> Event:
        """
        Event signaling the result of a message send.

        :rtype: Event
        """
        return self.__message_result_event

    @property
    def response(self) -> int:
        """
        Message response (e.g., SUCCESS).

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

    @property
    def context(self) -> dict:
        """
        Current context of the message.

        :return: Dictionary with the message context
        :rtype: dict
        """
        context = {
            "msg_type": self.__msg_type,
            "target_system": self.__target_system,
            "target_comp": self.__target_comp,
            "retry": self.__retry,
            "msg_timeout": self.__msg_timeout,
            "ack_timeout": self.__ack_timeout,
            "state_timeout": self.__state_timeout,
            "state_delay": self.__state_delay,
            "response": self.__response,
        }
        context.update(self.__optional_context_props)

        return context
