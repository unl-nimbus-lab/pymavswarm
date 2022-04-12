import sys

sys.path.append("../..")

from event import Event


class AgentMsg:
    """
    Parent class used to construct MAVLink commands
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
            retry is set to true, defaults to 5.0
        :type msg_timeout: float

        :param ack_timeout: The amount of time that pymavswarm should wait to check for
            an acknowledgement from an agent. This is only used when retry is set
            to true. This should be kept as short as possible to keep agent state
            information up-to-date, defaults to 1.0
        :type ack_timeout: float

        :param state_timeout: The amount of time that pymavswarm should wait for a
            given agent's state to change after receiving a mavlink message, defaults
            to 5.0
        :type state_timeout: float

        :param state_delay: The amount of time that pymavswarm should wait after
            sending a command prior to sending another command. This parameter is used
            for sequence-driven commands such as the full takeoff command sequence,
            defaults to 3.0
        :type state_delay: float

        :param validate_state: Flag indicating that pymavswarm should check to ensure
            that the message caused the desired state change in the system, defaults to
            False
        :type validate_state: bool
        """
        self.__msg_type = msg_type
        self.__target_system = target_system
        self.__target_comp = target_comp
        self.__retry = retry

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
        self.__validate_state = validate_state
        self.__message_result_event = Event()

        return

    @property
    def msg_type(self) -> str:
        """
        The sub-message type for a message

        :rtype: str
        """
        return self.__msg_type

    @property
    def target_system(self) -> int:
        """
        The target system ID

        :rtype: int
        """
        return self.__target_system

    @property
    def target_comp(self) -> int:
        """
        The target component ID

        :rtype: int
        """
        return self.__target_comp

    @property
    def retry(self) -> bool:
        """
        Indicate whether pymavswarm should retry sending the message
        until acknowledgement

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
        The amount of time that pymavswarm should attempt to resend
        a message if acknowledgement is not received. This is only used when retry
        is set to true.

        :rtype: float
        """
        return self.__msg_timeout

    @property
    def ack_timeout(self) -> float:
        """
        The amount of time that pymavswarm should wait to check for an
        acknowledgement from an agent. This is only used when retry is set to true.
        This should be kept as short as possible to keep agent state information
        up-to-date.

        :rtype: float
        """
        return self.__ack_timeout

    @property
    def state_timeout(self) -> float:
        """
        The amount of time that pymavswarm should wait for a given agent's
        state to change after receiving a mavlink message

        :rtype: float
        """
        return self.__state_timeout

    @property
    def state_delay(self) -> float:
        """
        The amount of time that pymavswarm should wait after sending a command
        prior to sending another command. This parameter is used for sequence-driven
        commands such as the full takeoff command sequence.

        :rtype: float
        """
        return self.__state_delay

    @property
    def validate_state(self) -> bool:
        """
        Flag indicating that pymavswarm should check to ensure that the
        message caused the desired state change in the system

        :rtype: bool
        """
        return self.__validate_state

    @property
    def message_result_event(self) -> Event:
        """
        Event signaling the result of a message send

        :rtype: Event
        """
        return self.__message_result_event

    @property
    def context(self) -> dict:
        """
        Current context of the message

        :return: Dictionary with the message context
        :rtype: dict
        """
        return {
            "msg_type": self.__msg_type,
            "target_system": self.__target_system,
            "target_comp": self.__target_comp,
            "retry": self.__retry,
            "msg_timeout": self.__msg_timeout,
            "ack_timeout": self.__ack_timeout,
            "state_timeout": self.__state_timeout,
            "state_delay": self.__state_delay,
            "validate_state": self.__validate_state,
        }
