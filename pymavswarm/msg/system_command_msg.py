from pymavswarm.msg import AgentMsg
from pymavswarm.msg import SupportedMsgs as supported_msgs


class SystemCommandMsg(AgentMsg):
    """
    Signal a system-level operation on an agent.
    """

    def __init__(
        self,
        command: str,
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

        :param command: command to execute
        :type command: str

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
        :type msg_timeout: float, optional

        :param ack_timeout: The amount of time that pymavswarm should wait to check for
            an acknowledgement from an agent. This is only used when retry is set
            to true. This should be kept as short as possible to keep agent state
            information up-to-date, defaults to 1.0
        :type ack_timeout: float, optional

        :param state_timeout: The amount of time that pymavswarm should wait for a
            given agent's state to change after receiving a mavlink message, defaults
            to 5.0
        :type state_timeout: float, optional

        :param state_delay: The amount of time that pymavswarm should wait after
            sending a command prior to sending another command. This parameter is used
            for sequence-driven commands such as the full takeoff command sequence,
            defaults to 3.0
        :type state_delay: float, optional

        :param optional_context_props: optional properties to append to the message
            context, defaults to {}
        :type optional_context_props: dict, optional
        """
        if command not in supported_msgs.system_commands.get_supported_types():
            raise ValueError(
                f"{command} is not a supported system command. Supported system "
                "commands include: "
                f"{supported_msgs.system_commands.get_supported_types()}"
            )

        super().__init__(
            command,
            target_system,
            target_comp,
            retry,
            msg_timeout=msg_timeout,
            ack_timeout=ack_timeout,
            state_timeout=state_timeout,
            state_delay=state_delay,
            optional_context_props=optional_context_props,
        )

        self.__command = command

        return

    @property
    def command(self) -> str:
        """
        System command to execute.

        :rtype: str
        """
        return self.__command

    @property
    def context(self) -> dict:
        """
        Message context.

        :return: current message context
        :rtype: dict
        """
        context = super().context
        context["command"] = self.__command

        return context
